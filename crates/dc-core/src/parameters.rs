//! Disarmed-only parameter transactions (T0-18 / H5); flight commands keep ADR-0009.

use std::collections::{BTreeMap, HashMap};
use std::fmt;
use std::io::Write;
use std::path::{Path, PathBuf};
use std::sync::Mutex;
use std::time::{Duration, Instant};

use mavlink_codec::Message;
use mavlink_codec::enums::{MavModeFlag, MavParamType, MavType};
use mavlink_codec::messages::{ParamRequestRead, ParamSet};
use protocols::params::snapshot::{ParamChange, ParamSnapshot, SnapshotError};
use protocols::params::{Param, SetStatus, decode_id, encode_id};
use tokio::sync::broadcast;

use crate::{Core, NavThresholds, ParamError};

/// A parameter write requires an autopilot disarmed heartbeat younger than 1.5 seconds.
/// This fixed vehicle-state requirement does not depend on the displayed link state.
pub const PARAM_HEARTBEAT_MAX_AGE: Duration = Duration::from_millis(1500);
const WRITE_LIFETIME: Duration = Duration::from_secs(10);
const RETRY: Duration = Duration::from_secs(1);

/// Minimum conservative risk labels for H5. This is not a full parameter metadata catalog.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ParamRisk {
    /// Not identified as an arming/failsafe/fence/safety setting by the minimum labels.
    Ordinary,
    /// Requires a dedicated dangerous-parameter approval path, not yet exposed.
    Dangerous,
}

/// Label failsafe, fence and arming/safety parameters conservatively. Battery failsafes
/// (`BATT_FS_*`, `BATT2_FS_*`) and RC failsafes are covered as well as `FS_*`.
pub fn parameter_risk(name: &str) -> ParamRisk {
    let name = name.to_ascii_uppercase();
    if name.starts_with("FS_")
        || name.contains("_FS_")
        || name.starts_with("FENCE_")
        || name.starts_with("ARMING_")
        || name.starts_with("BRD_SAFETY")
    {
        ParamRisk::Dangerous
    } else {
        ParamRisk::Ordinary
    }
}

/// Failure before any parameter set is sent.
#[derive(Debug)]
pub enum ParamOperationError {
    /// The target, armed-state check or dangerous-parameter policy refused the operation.
    Refused(String),
    /// Another write/rollback is active; intents are never silently queued.
    Busy,
    /// A complete baseline could not be downloaded.
    Download(ParamError),
    /// The baseline or requested edits were invalid.
    Snapshot(SnapshotError),
    /// The baseline could not be safely persisted/read; nothing is sent.
    Storage(String),
}

impl fmt::Display for ParamOperationError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Refused(reason) | Self::Storage(reason) => f.write_str(reason),
            Self::Busy => f.write_str("a parameter write or rollback is already active"),
            Self::Download(error) => error.fmt(f),
            Self::Snapshot(error) => error.fmt(f),
        }
    }
}

impl std::error::Error for ParamOperationError {}

/// Result for one attempted parameter. An echo alone never establishes success.
#[derive(Debug, Clone, PartialEq)]
pub struct ParamWriteResult {
    /// Parameter name.
    pub name: String,
    /// Verified explicit read-back, mismatched read-back, or unconfirmed timeout.
    pub status: SetStatus,
}

/// Batch result, including an interruption after some parameters may have changed.
#[derive(Debug, Clone, PartialEq)]
pub struct ParamReport {
    /// Vehicle/component used for every request.
    pub target: (u8, u8),
    /// Original baseline on disk; retained even on mismatch, timeout or interruption.
    pub snapshot: PathBuf,
    /// Number of changes planned (including those not reached after a failure).
    pub planned: usize,
    /// Attempted parameters, in deterministic name order.
    pub results: Vec<ParamWriteResult>,
    /// Why transmission stopped. No automatic rollback is attempted while armed/unknown.
    pub interruption: Option<String>,
}

impl ParamReport {
    /// Every planned change received matching value/type in an explicit read-back.
    pub fn verified(&self) -> bool {
        self.interruption.is_none()
            && self.results.len() == self.planned
            && self
                .results
                .iter()
                .all(|r| matches!(r.status, SetStatus::Verified(_)))
    }
}

#[derive(Default)]
pub(super) struct ParameterState {
    /// Serialize writes across Core clones, without delaying unrelated flight commands.
    gate: tokio::sync::Mutex<()>,
    /// Latch armed reports and heartbeat gaps even if a later heartbeat is disarmed again.
    epochs: Mutex<HashMap<u8, u64>>,
    nav: Mutex<HashMap<u8, f32>>,
}

impl Core {
    pub(super) fn observe_parameter_safety(
        &self,
        sysid: u8,
        compid: u8,
        msg: &Message,
        now: Instant,
    ) {
        if compid != vehicle::AUTOPILOT_COMPID {
            return;
        }
        if let Message::Heartbeat(h) = msg {
            if h.r#type == MavType::GCS.0 {
                return;
            }
            let gap = self
                .fleet()
                .get(sysid)
                .and_then(|v| v.heartbeat.as_ref())
                .is_some_and(|hb| hb.age(now) >= PARAM_HEARTBEAT_MAX_AGE);
            if h.base_mode & MavModeFlag::SAFETY_ARMED.0 != 0 || gap {
                let mut epochs = self
                    .parameters
                    .epochs
                    .lock()
                    .unwrap_or_else(|e| e.into_inner());
                *epochs.entry(sysid).or_default() += 1;
            }
        }
        if let Message::ParamValue(p) = msg
            && decode_id(&p.param_id) == "FS_EKF_THRESH"
            && p.param_type == MavParamType::REAL32.0
            && p.param_value.is_finite()
            && p.param_value >= 0.0
        {
            self.parameters
                .nav
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .insert(sysid, p.param_value);
        }
    }

    /// Current navigation thresholds, using the autopilot's last reported `FS_EKF_THRESH`
    /// when available (including zero, which disables that variance failsafe on ArduPilot).
    pub fn nav_thresholds(&self, sysid: u8) -> NavThresholds {
        let mut thresholds = self.cfg.nav;
        if let Some(value) = self
            .parameters
            .nav
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .get(&sysid)
        {
            thresholds.ekf_fail = *value;
        }
        thresholds
    }

    fn parameter_epoch(&self, sysid: u8) -> u64 {
        *self
            .parameters
            .epochs
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .get(&sysid)
            .unwrap_or(&0)
    }

    fn check_parameter_state(
        &self,
        target: (u8, u8),
        epoch: u64,
    ) -> Result<(), ParamOperationError> {
        if target.0 == 0 || target.1 != vehicle::AUTOPILOT_COMPID {
            return Err(ParamOperationError::Refused(
                "parameter writes require a specific autopilot target".into(),
            ));
        }
        let fleet = self.fleet();
        let hb = fleet.get(target.0).and_then(|v| v.heartbeat.as_ref());
        if hb.is_none_or(|hb| {
            hb.value.compid != target.1
                || hb.value.armed()
                || hb.age(Instant::now()) >= PARAM_HEARTBEAT_MAX_AGE
        }) || self.parameter_epoch(target.0) != epoch
        {
            return Err(ParamOperationError::Refused("parameter writes require a recent, continuously known-disarmed autopilot heartbeat".into()));
        }
        Ok(())
    }

    /// Snapshot a complete download, persist it to a **new** file, then apply ordinary edits.
    /// Fresh disarmed state is required before every set/retry/read. Dangerous parameters
    /// are refused until a dedicated typed-confirmation path exists. Each write has a 10 s
    /// lifetime and success requires an explicit `PARAM_REQUEST_READ` response of equal
    /// name, value and type. Stop on the first mismatch, timeout or safety interruption.
    pub async fn write_params(
        &self,
        target: (u8, u8),
        edits: BTreeMap<String, f64>,
        snapshot_path: impl AsRef<Path>,
    ) -> Result<ParamReport, ParamOperationError> {
        self.write_params_checked(target, edits, snapshot_path.as_ref(), None)
            .await
    }

    async fn write_params_checked(
        &self,
        target: (u8, u8),
        edits: BTreeMap<String, f64>,
        snapshot_path: &Path,
        expected_types: Option<BTreeMap<String, u8>>,
    ) -> Result<ParamReport, ParamOperationError> {
        let _guard = self
            .parameters
            .gate
            .try_lock()
            .map_err(|_| ParamOperationError::Busy)?;
        let epoch = self.parameter_epoch(target.0);
        self.check_parameter_state(target, epoch)?;
        for name in edits.keys() {
            if parameter_risk(name) == ParamRisk::Dangerous {
                return Err(ParamOperationError::Refused(format!(
                    "{name} is dangerous: dedicated confirmation is not yet available"
                )));
            }
        }
        let mut count = 0;
        let params = self
            .download_params(target, |_, total| count = total)
            .await
            .map_err(ParamOperationError::Download)?;
        if let Some(types) = expected_types {
            for (name, ty) in types {
                if params.get(&name).is_none_or(|p| p.ty != ty) {
                    return Err(ParamOperationError::Refused(format!(
                        "{name} changed type or disappeared since the original snapshot"
                    )));
                }
            }
        }
        let count = u16::try_from(count)
            .map_err(|_| ParamOperationError::Snapshot(SnapshotError::IncompleteDownload))?;
        let baseline =
            ParamSnapshot::capture(target, count, params).map_err(ParamOperationError::Snapshot)?;
        let plan = baseline
            .plan(&edits)
            .map_err(ParamOperationError::Snapshot)?;
        // Downloading a baseline is read-only. Past gaps during that preparation must
        // not reject a currently fresh disarmed state before any writes have begun.
        let epoch = self.parameter_epoch(target.0);
        self.check_parameter_state(target, epoch)?;
        let path = snapshot_path.to_path_buf();
        persist_snapshot(path.clone(), baseline.to_text()).await?;
        let epoch = self.parameter_epoch(target.0);
        Ok(self
            .apply_parameter_changes(target, plan.changes(), path, epoch)
            .await)
    }

    /// Load an original baseline and restore only the supplied names, after a new complete
    /// download from its recorded target. The recovery baseline is persisted to a new file
    /// before restoration. Uses the same state, risk and read-back checks as normal writes.
    /// Specify only parameters attempted by the original report, not the whole baseline.
    pub async fn rollback_params(
        &self,
        original_snapshot: impl AsRef<Path>,
        names: &[String],
        recovery_snapshot: impl AsRef<Path>,
    ) -> Result<ParamReport, ParamOperationError> {
        let text = std::fs::read_to_string(original_snapshot)
            .map_err(|e| ParamOperationError::Storage(e.to_string()))?;
        let original = ParamSnapshot::from_text(&text).map_err(ParamOperationError::Snapshot)?;
        let mut edits = BTreeMap::new();
        let mut types = BTreeMap::new();
        for name in names {
            let param = original.params().get(name).ok_or_else(|| {
                ParamOperationError::Snapshot(SnapshotError::UnknownParameter(name.clone()))
            })?;
            edits.insert(name.clone(), f64::from(param.value));
            types.insert(name.clone(), param.ty);
        }
        self.write_params_checked(
            original.target(),
            edits,
            recovery_snapshot.as_ref(),
            Some(types),
        )
        .await
    }

    async fn apply_parameter_changes(
        &self,
        target: (u8, u8),
        changes: &[ParamChange],
        path: PathBuf,
        epoch: u64,
    ) -> ParamReport {
        let mut report = ParamReport {
            target,
            snapshot: path,
            planned: changes.len(),
            results: Vec::new(),
            interruption: None,
        };
        for change in changes {
            if let Err(error) = self.check_parameter_state(target, epoch) {
                report.interruption = Some(error.to_string());
                break;
            }
            let (status, interruption) = self.write_parameter(target, change, epoch).await;
            report.results.push(ParamWriteResult {
                name: change.name.clone(),
                status,
            });
            report.interruption = interruption;
            if !matches!(status, SetStatus::Verified(_)) || report.interruption.is_some() {
                break;
            }
        }
        report
    }

    async fn write_parameter(
        &self,
        target: (u8, u8),
        change: &ParamChange,
        epoch: u64,
    ) -> (SetStatus, Option<String>) {
        let set = Message::ParamSet(ParamSet {
            target_system: target.0,
            target_component: target.1,
            param_id: encode_id(&change.name),
            param_value: change.after.value,
            param_type: change.after.ty,
        });
        let read = Message::ParamRequestRead(ParamRequestRead {
            target_system: target.0,
            target_component: target.1,
            param_id: encode_id(&change.name),
            param_index: -1,
        });
        let mut rx = self.incoming.subscribe();
        let deadline = Instant::now() + WRITE_LIFETIME;
        let mut last_send = None;
        let mut reading = false;
        let mut tick = tokio::time::interval(Duration::from_millis(25));
        loop {
            if let Err(error) = self.check_parameter_state(target, epoch) {
                return (SetStatus::Failed(None), Some(error.to_string()));
            }
            if Instant::now() >= deadline {
                return (SetStatus::Failed(None), None);
            }
            if last_send.is_none_or(|at| Instant::now().duration_since(at) >= RETRY) {
                if !self.send(if reading { &read } else { &set }).await {
                    return (
                        SetStatus::Failed(None),
                        Some("parameter transport closed".into()),
                    );
                }
                last_send = Some(Instant::now());
            }
            tokio::select! {
                _ = tick.tick() => {},
                incoming = rx.recv() => match incoming {
                    Ok(inc) if (inc.sysid, inc.compid) == target => {
                        if let Message::ParamValue(p) = &inc.msg
                            && decode_id(&p.param_id) == change.name
                        {
                            let actual = Param { value: p.param_value, ty: p.param_type, index: p.param_index };
                            if actual.value != change.after.value || actual.ty != change.after.ty {
                                return (SetStatus::Failed(Some(actual)), None);
                            }
                            if reading {
                                if let Err(error) = self.check_parameter_state(target, epoch) {
                                    return (SetStatus::Verified(actual), Some(error.to_string()));
                                }
                                return (SetStatus::Verified(actual), None);
                            }
                            reading = true;
                            last_send = None;
                        }
                    },
                    Ok(_) => {},
                    Err(broadcast::error::RecvError::Lagged(_)) => return (SetStatus::Failed(None), Some("parameter receive stream lagged; result unconfirmed".into())),
                    Err(broadcast::error::RecvError::Closed) => return (SetStatus::Failed(None), Some("parameter receive stream closed".into())),
                }
            }
        }
    }
}

async fn persist_snapshot(path: PathBuf, text: String) -> Result<(), ParamOperationError> {
    tokio::task::spawn_blocking(move || -> std::io::Result<()> {
        let mut file = std::fs::OpenOptions::new()
            .write(true)
            .create_new(true)
            .open(&path)?;
        file.write_all(text.as_bytes())?;
        file.sync_all()?;
        // Directory durability is supported on Unix; Windows File::open on a directory
        // requires different flags. File contents are sync_all'd on every platform.
        #[cfg(unix)]
        std::fs::File::open(
            path.parent()
                .filter(|p| !p.as_os_str().is_empty())
                .unwrap_or(Path::new(".")),
        )?
        .sync_all()?;
        Ok(())
    })
    .await
    .map_err(|e| ParamOperationError::Storage(e.to_string()))?
    .map_err(|e| ParamOperationError::Storage(e.to_string()))
}

#[cfg(test)]
mod tests {
    use super::*;
    use mavlink_codec::messages::ParamValue;

    #[tokio::test]
    async fn nav_threshold_uses_only_valid_autopilot_float_values_for_each_vehicle() {
        let core = Core::start(crate::CoreConfig::default()).await.unwrap();
        let value = |v, ty| {
            Message::ParamValue(ParamValue {
                param_id: encode_id("FS_EKF_THRESH"),
                param_value: v,
                param_type: ty,
                param_count: 1,
                param_index: 0,
            })
        };
        core.observe_parameter_safety(1, 100, &value(0.0, MavParamType::REAL32.0), Instant::now());
        assert_eq!(core.nav_thresholds(1).ekf_fail, 0.8);
        core.observe_parameter_safety(1, 1, &value(0.9, MavParamType::REAL32.0), Instant::now());
        assert_eq!(core.nav_thresholds(1).ekf_fail, 0.9);
        assert_eq!(core.nav_thresholds(2).ekf_fail, 0.8);
        for (v, ty) in [
            (f32::NAN, MavParamType::REAL32.0),
            (f32::INFINITY, MavParamType::REAL32.0),
            (-1.0, MavParamType::REAL32.0),
            (0.0, MavParamType::INT8.0),
        ] {
            core.observe_parameter_safety(1, 1, &value(v, ty), Instant::now());
            assert_eq!(core.nav_thresholds(1).ekf_fail, 0.9);
        }
        core.observe_parameter_safety(1, 1, &value(0.0, MavParamType::REAL32.0), Instant::now());
        assert_eq!(core.nav_thresholds(1).ekf_fail, 0.0);
    }
}
