//! Parameter transactions (T0-18 / H5); flight commands keep ADR-0009.
//!
//! Every write snapshots a complete baseline first and is verified by an explicit read-back.
//! By default the autopilot must be known disarmed. Since the owner's decision of 2026-10-13
//! (ADR-0012) a write may also go to an armed or flying vehicle when the operator has
//! acknowledged [`ARMED_WRITE_WARNING`] ([`ArmedWrite::Acknowledged`]); without that the
//! core answers [`ParamOperationError::ArmedWarningRequired`] so the client can show it.

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

/// A parameter write requires an autopilot heartbeat younger than 1.5 seconds (disarmed,
/// unless the in-flight warning was acknowledged). This fixed vehicle-state requirement does
/// not depend on the displayed link state.
pub const PARAM_HEARTBEAT_MAX_AGE: Duration = Duration::from_millis(1500);
const WRITE_LIFETIME: Duration = Duration::from_secs(10);
const RETRY: Duration = Duration::from_secs(1);

/// What the client shows before a write to an armed vehicle (ADR-0012, 2026-10-13); the
/// operator must acknowledge it.
pub const ARMED_WRITE_WARNING: &str = "The vehicle is ARMED. Parameter changes take effect immediately, in flight: a wrong value can change how it flies, disable a failsafe or cause loss of control. No automatic rollback is attempted while armed.";

/// Whether a write may go to an armed vehicle.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ArmedWrite {
    /// Only to a known-disarmed vehicle; an armed one answers
    /// [`ParamOperationError::ArmedWarningRequired`].
    #[default]
    DisarmedOnly,
    /// The operator acknowledged [`ARMED_WRITE_WARNING`]: write whether armed or not.
    Acknowledged,
}

/// Minimum conservative risk labels for H5. This is not a full parameter metadata catalog.
/// The label is for display (the single confirmation names what is being changed); it does
/// not refuse a write (owner decision, ADR-0012 item 1).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ParamRisk {
    /// Not identified as an arming/failsafe/fence/safety setting by the minimum labels.
    Ordinary,
    /// An arming, failsafe, fence or board-safety setting: written like any other, but the
    /// confirmation should say so.
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
    /// The target or vehicle-state check refused the operation.
    Refused(String),
    /// The vehicle is armed and the write did not carry the operator's acknowledgement of
    /// this warning ([`ARMED_WRITE_WARNING`]); nothing was sent. Show it, then repeat with
    /// [`ArmedWrite::Acknowledged`].
    ArmedWarningRequired(String),
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
            Self::Refused(reason) | Self::Storage(reason) | Self::ArmedWarningRequired(reason) => {
                f.write_str(reason)
            }
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

/// `RC_CHANNELS.rssi` with what it measures ([`Core::rc_signal`]).
#[derive(Debug, Clone, PartialEq)]
pub struct RcSignal {
    /// Raw value, 0–254.
    pub value: u8,
    /// As a percentage of RSSI or link quality, per `kind`.
    pub pct: f32,
    /// RSSI, link quality, or not known yet.
    pub kind: vehicle::rc_link::SignalKind,
    /// What the vehicle said about its CRSF/ELRS receiver, if anything.
    pub link: Option<vehicle::rc_link::RcLinkInfo>,
}

/// `RSSI_TYPE` and `RC_OPTIONS` of one vehicle, when reported.
type RcSignalParams = (Option<u8>, Option<u32>);

/// Counts of armed reports and heartbeat gaps; a write compares them before every step.
type Epoch = (u64, u64);

#[derive(Default)]
pub(super) struct ParameterState {
    /// Serialize writes across Core clones, without delaying unrelated flight commands.
    gate: tokio::sync::Mutex<()>,
    /// Latch armed reports and heartbeat gaps even if a later heartbeat is disarmed again:
    /// (armed reports, heartbeat gaps) per vehicle.
    epochs: Mutex<HashMap<u8, Epoch>>,
    nav: Mutex<HashMap<u8, f32>>,
    /// `RSSI_TYPE` and `RC_OPTIONS` per vehicle, as last reported (what `RC_CHANNELS.rssi`
    /// measures).
    rc_signal: Mutex<HashMap<u8, RcSignalParams>>,
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
            let armed = h.base_mode & MavModeFlag::SAFETY_ARMED.0 != 0;
            if armed || gap {
                let mut epochs = self
                    .parameters
                    .epochs
                    .lock()
                    .unwrap_or_else(|e| e.into_inner());
                let e = epochs.entry(sysid).or_default();
                e.0 += u64::from(armed);
                e.1 += u64::from(gap);
            }
        }
        if let Message::ParamValue(p) = msg
            && p.param_value.is_finite()
            && p.param_value >= 0.0
        {
            let name = decode_id(&p.param_id);
            if name == "RSSI_TYPE" || name == "RC_OPTIONS" {
                let mut m = self
                    .parameters
                    .rc_signal
                    .lock()
                    .unwrap_or_else(|e| e.into_inner());
                let e = m.entry(sysid).or_default();
                if name == "RSSI_TYPE" {
                    e.0 = Some(p.param_value as u8);
                } else {
                    e.1 = Some(p.param_value as u32);
                }
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

    /// What `RC_CHANNELS.rssi` from `sysid` means now, with its value as a percentage
    /// (plan 09 Step 2). `None` without RC input or with RSSI disabled on the vehicle. The
    /// kind comes from the autopilot's CRSF/ELRS message, else from `RSSI_TYPE` and
    /// `RC_OPTIONS` as they pass in parameter traffic (a download or a read); nothing is
    /// requested for it, so it can stay [`SignalKind::Unknown`](vehicle::rc_link::SignalKind)
    /// until one of them is seen.
    pub fn rc_signal(&self, sysid: u8) -> Option<RcSignal> {
        let fleet = self.fleet();
        let v = fleet.get(sysid)?;
        let value = v.rc.as_ref()?.value.rssi?;
        let link = v.rc_link.as_ref().map(|l| l.value.clone());
        let (rssi_type, rc_options) = self
            .parameters
            .rc_signal
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .get(&sysid)
            .copied()
            .unwrap_or_default();
        let kind = vehicle::rc_link::signal_kind(
            link.as_ref().and_then(|l| l.rssi_as_lq),
            rssi_type,
            rc_options,
        );
        Some(RcSignal {
            value,
            pct: vehicle::rc_link::signal_pct(value, kind),
            kind,
            link,
        })
    }

    fn parameter_epoch(&self, sysid: u8) -> Epoch {
        *self
            .parameters
            .epochs
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .get(&sysid)
            .unwrap_or(&(0, 0))
    }

    fn check_parameter_state(
        &self,
        target: (u8, u8),
        epoch: Epoch,
        mode: ArmedWrite,
    ) -> Result<(), ParamOperationError> {
        if target.0 == 0 || target.1 != vehicle::AUTOPILOT_COMPID {
            return Err(ParamOperationError::Refused(
                "parameter writes require a specific autopilot target".into(),
            ));
        }
        let fleet = self.fleet();
        let hb = fleet.get(target.0).and_then(|v| v.heartbeat.as_ref());
        let now = self.parameter_epoch(target.0);
        if mode == ArmedWrite::Acknowledged {
            // Armed or not, the vehicle state must stay continuously known: a heartbeat gap
            // makes the outcome of a write unknown.
            if hb.is_none_or(|hb| {
                hb.value.compid != target.1 || hb.age(Instant::now()) >= PARAM_HEARTBEAT_MAX_AGE
            }) || now.1 != epoch.1
            {
                return Err(ParamOperationError::Refused(
                    "parameter writes require a recent, continuous autopilot heartbeat".into(),
                ));
            }
            return Ok(());
        }
        if hb.is_none_or(|hb| {
            hb.value.compid != target.1
                || hb.value.armed()
                || hb.age(Instant::now()) >= PARAM_HEARTBEAT_MAX_AGE
        }) || now != epoch
        {
            return Err(ParamOperationError::Refused("parameter writes require a recent, continuously known-disarmed autopilot heartbeat".into()));
        }
        Ok(())
    }

    /// Read one parameter by name (`PARAM_REQUEST_READ`, retried each second, up to 5 s):
    /// read-only, for checking a few values without downloading the whole list.
    pub async fn read_param(&self, target: (u8, u8), name: &str) -> Result<Param, String> {
        let read = Message::ParamRequestRead(ParamRequestRead {
            target_system: target.0,
            target_component: target.1,
            param_id: encode_id(name),
            param_index: -1,
        });
        let mut rx = self.incoming.subscribe();
        let deadline = Instant::now() + Duration::from_secs(5);
        let mut last_send: Option<Instant> = None;
        let mut tick = tokio::time::interval(Duration::from_millis(25));
        while Instant::now() < deadline {
            if last_send.is_none_or(|at| at.elapsed() >= RETRY) {
                if !self.send(&read).await {
                    return Err("parameter transport closed".into());
                }
                last_send = Some(Instant::now());
            }
            tokio::select! {
                _ = tick.tick() => {},
                incoming = rx.recv() => match incoming {
                    Ok(inc) if (inc.sysid, inc.compid) == target => {
                        if let Message::ParamValue(p) = &inc.msg
                            && decode_id(&p.param_id) == name
                        {
                            return Ok(Param { value: p.param_value, ty: p.param_type, index: p.param_index });
                        }
                    }
                    Ok(_) | Err(broadcast::error::RecvError::Lagged(_)) => {}
                    Err(broadcast::error::RecvError::Closed) => return Err("receive stream closed".into()),
                }
            }
        }
        Err(format!(
            "{name}: no answer within 5 s (unknown name, or link down)"
        ))
    }

    /// The vehicle's latest heartbeat says armed.
    fn reported_armed(&self, sysid: u8) -> bool {
        self.fleet()
            .get(sysid)
            .and_then(|v| v.heartbeat.as_ref())
            .is_some_and(|hb| hb.value.armed())
    }

    /// Snapshot a complete download, persist it to a **new** file, then apply the edits.
    /// Fresh disarmed state is required before every set/retry/read (see
    /// [`Core::write_params_with`] for writes to an armed vehicle). Every parameter may be
    /// written, failsafe, arming, fence and board-safety ones included (ADR-0012 item 1; the
    /// caller asks for one confirmation, see [`parameter_risk`]). Each write has a 10 s
    /// lifetime and success requires an explicit `PARAM_REQUEST_READ` response of equal
    /// name, value and type. Stop on the first mismatch, timeout or safety interruption.
    pub async fn write_params(
        &self,
        target: (u8, u8),
        edits: BTreeMap<String, f64>,
        snapshot_path: impl AsRef<Path>,
    ) -> Result<ParamReport, ParamOperationError> {
        self.write_params_with(target, edits, snapshot_path, ArmedWrite::DisarmedOnly)
            .await
    }

    /// [`Core::write_params`], also to an armed or flying vehicle when `mode` is
    /// [`ArmedWrite::Acknowledged`] (the operator saw [`ARMED_WRITE_WARNING`]). Without the
    /// acknowledgement an armed vehicle answers [`ParamOperationError::ArmedWarningRequired`]
    /// before anything is sent. The baseline, read-back and stop-on-first-failure rules are
    /// the same; a heartbeat gap still stops the batch.
    pub async fn write_params_with(
        &self,
        target: (u8, u8),
        edits: BTreeMap<String, f64>,
        snapshot_path: impl AsRef<Path>,
        mode: ArmedWrite,
    ) -> Result<ParamReport, ParamOperationError> {
        self.write_params_checked(target, edits, snapshot_path.as_ref(), None, mode)
            .await
    }

    async fn write_params_checked(
        &self,
        target: (u8, u8),
        edits: BTreeMap<String, f64>,
        snapshot_path: &Path,
        expected_types: Option<BTreeMap<String, u8>>,
        mode: ArmedWrite,
    ) -> Result<ParamReport, ParamOperationError> {
        let _guard = self
            .parameters
            .gate
            .try_lock()
            .map_err(|_| ParamOperationError::Busy)?;
        if mode == ArmedWrite::DisarmedOnly && self.reported_armed(target.0) {
            self.audit_note(
                target.0,
                format!(
                    "parameter write of {} refused: vehicle armed, in-flight warning not acknowledged",
                    edits.keys().cloned().collect::<Vec<_>>().join(", ")
                ),
            );
            return Err(ParamOperationError::ArmedWarningRequired(
                ARMED_WRITE_WARNING.into(),
            ));
        }
        let epoch = self.parameter_epoch(target.0);
        self.check_parameter_state(target, epoch, mode)?;
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
        self.check_parameter_state(target, epoch, mode)?;
        let path = snapshot_path.to_path_buf();
        persist_snapshot(path.clone(), baseline.to_text()).await?;
        let epoch = self.parameter_epoch(target.0);
        self.audit_note(
            target.0,
            format!(
                "parameter write: {} change(s){}; baseline {}",
                plan.changes().len(),
                if self.reported_armed(target.0) {
                    " while ARMED, in-flight warning acknowledged"
                } else {
                    ""
                },
                path.display()
            ),
        );
        let report = self
            .apply_parameter_changes(target, plan.changes(), path, epoch, mode)
            .await;
        for (r, c) in report.results.iter().zip(plan.changes()) {
            let outcome = match &r.status {
                SetStatus::Verified(_) => "verified by read-back".to_string(),
                SetStatus::Failed(Some(p)) => format!("FAILED: vehicle reports {}", p.value),
                SetStatus::Failed(None) | SetStatus::Pending => {
                    "UNCONFIRMED (no read-back)".to_string()
                }
            };
            self.audit_note(
                target.0,
                format!(
                    "parameter {} {} -> {}: {outcome}",
                    r.name, c.before.value, c.after.value
                ),
            );
        }
        if let Some(why) = &report.interruption {
            self.audit_note(target.0, format!("parameter write stopped: {why}"));
        }
        Ok(report)
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
        self.rollback_params_with(
            original_snapshot,
            names,
            recovery_snapshot,
            ArmedWrite::DisarmedOnly,
        )
        .await
    }

    /// [`Core::rollback_params`] with the same armed-write rule as
    /// [`Core::write_params_with`] (a rollback the operator starts in flight).
    pub async fn rollback_params_with(
        &self,
        original_snapshot: impl AsRef<Path>,
        names: &[String],
        recovery_snapshot: impl AsRef<Path>,
        mode: ArmedWrite,
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
            mode,
        )
        .await
    }

    async fn apply_parameter_changes(
        &self,
        target: (u8, u8),
        changes: &[ParamChange],
        path: PathBuf,
        epoch: Epoch,
        mode: ArmedWrite,
    ) -> ParamReport {
        let mut report = ParamReport {
            target,
            snapshot: path,
            planned: changes.len(),
            results: Vec::new(),
            interruption: None,
        };
        for change in changes {
            if let Err(error) = self.check_parameter_state(target, epoch, mode) {
                report.interruption = Some(error.to_string());
                break;
            }
            let (status, interruption) = self.write_parameter(target, change, epoch, mode).await;
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
        epoch: Epoch,
        mode: ArmedWrite,
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
            if let Err(error) = self.check_parameter_state(target, epoch, mode) {
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
                                if let Err(error) = self.check_parameter_state(target, epoch, mode) {
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
