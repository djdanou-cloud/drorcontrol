//! Opt-in settings for vehicle-changing actions (ADR-0012 items 2–4 and 6, as the owner
//! amended them on 2026-10-13): each is **off by default** and carries a short warning
//! about its danger, shown when it is turned on. While a setting is off the core refuses the
//! action and nothing is sent ([`VehicleChangeError::SettingOff`]).
//!
//! * [`VehicleChange::ArmedMissionUpload`]: mission upload to an armed or flying vehicle (or
//!   one whose armed state is unknown). Off: missions upload only to a known-disarmed
//!   vehicle, as before.
//! * [`VehicleChange::FenceRallyUpload`]: geofence and rally point upload, on the ground and
//!   in flight.
//! * [`VehicleChange::LogErase`]: erasing every dataflash log (`LOG_ERASE`).
//!
//! With a setting on, an action on an armed vehicle still needs the operator's
//! acknowledgement of an in-flight warning ([`ArmedWrite::Acknowledged`]), every upload is
//! read back and compared, and every action is written to the audit log.

use std::fmt;
use std::time::{Duration, Instant};

use mavlink_codec::Message;
use mavlink_codec::enums::MavState;
use mavlink_codec::messages::LogErase;
use protocols::mission::{self, MissionUpload, TransferConfig};

use crate::{ArmedWrite, Core, LogEntry, MissionItem, MissionReport, MissionType};

/// One opt-in setting.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VehicleChange {
    /// Mission upload while armed or flying.
    ArmedMissionUpload,
    /// Geofence and rally point upload (ground and flight).
    FenceRallyUpload,
    /// Erase every dataflash log on the vehicle.
    LogErase,
}

impl VehicleChange {
    /// Every setting, in display order.
    pub const ALL: [VehicleChange; 3] = [
        VehicleChange::ArmedMissionUpload,
        VehicleChange::FenceRallyUpload,
        VehicleChange::LogErase,
    ];

    /// Name in settings files and on the command line.
    pub fn key(self) -> &'static str {
        match self {
            VehicleChange::ArmedMissionUpload => "armed-mission-upload",
            VehicleChange::FenceRallyUpload => "fence-rally-upload",
            VehicleChange::LogErase => "log-erase",
        }
    }

    /// The setting from its [`key`](Self::key).
    pub fn from_key(key: &str) -> Option<VehicleChange> {
        Self::ALL.into_iter().find(|c| c.key() == key)
    }

    /// Short title for a settings screen.
    pub fn title(self) -> &'static str {
        match self {
            VehicleChange::ArmedMissionUpload => "Mission upload while armed or flying",
            VehicleChange::FenceRallyUpload => "Geofence and rally point upload",
            VehicleChange::LogErase => "Erase logs on the vehicle",
        }
    }

    /// The danger, shown next to the setting and when it is turned on.
    pub fn warning(self) -> &'static str {
        match self {
            VehicleChange::ArmedMissionUpload => {
                "Lets you replace the mission on an armed or flying vehicle. The new mission \
                 takes effect at once: a wrong waypoint, altitude or command can fly the \
                 vehicle into the ground, an obstacle or restricted airspace."
            }
            VehicleChange::FenceRallyUpload => {
                "Lets you change the geofence and rally points, on the ground and in flight. A \
                 wrong fence can trigger its breach action (RTL or land) at once, or stop \
                 protecting where it should; a wrong rally point changes where the vehicle \
                 returns to."
            }
            VehicleChange::LogErase => {
                "Lets you erase every dataflash log on the vehicle. Erased logs cannot be \
                 recovered: anything needed to investigate an incident is lost."
            }
        }
    }
}

/// What the client shows before a mission upload to an armed vehicle; the operator must
/// acknowledge it ([`ArmedWrite::Acknowledged`]).
pub const ARMED_MISSION_UPLOAD_WARNING: &str = "The vehicle is ARMED or its state is unknown. The new mission replaces the one it is flying, immediately. Update mission on active vehicle?";

/// What the client shows before a fence or rally upload to an armed vehicle.
pub const ARMED_FENCE_RALLY_WARNING: &str = "The vehicle is ARMED or its state is unknown. The new geofence or rally points take effect immediately, in flight: if the vehicle is outside the new fence its breach action (RTL or land) starts at once.";

/// The opt-in settings; all off by default.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct VehicleChangeSettings {
    /// [`VehicleChange::ArmedMissionUpload`].
    pub armed_mission_upload: bool,
    /// [`VehicleChange::FenceRallyUpload`].
    pub fence_rally_upload: bool,
    /// [`VehicleChange::LogErase`].
    pub log_erase: bool,
}

impl VehicleChangeSettings {
    /// Whether `change` is on.
    pub fn get(&self, change: VehicleChange) -> bool {
        match change {
            VehicleChange::ArmedMissionUpload => self.armed_mission_upload,
            VehicleChange::FenceRallyUpload => self.fence_rally_upload,
            VehicleChange::LogErase => self.log_erase,
        }
    }

    /// Turn `change` on or off.
    pub fn set(&mut self, change: VehicleChange, on: bool) {
        match change {
            VehicleChange::ArmedMissionUpload => self.armed_mission_upload = on,
            VehicleChange::FenceRallyUpload => self.fence_rally_upload = on,
            VehicleChange::LogErase => self.log_erase = on,
        }
    }

    /// Read a settings file: `key = true|false` lines; blank lines and `#` comments are
    /// ignored; a setting not named stays off. Unknown keys and values are errors, so a typo
    /// never silently leaves a setting in an unexpected state.
    pub fn parse(text: &str) -> Result<VehicleChangeSettings, String> {
        let mut s = VehicleChangeSettings::default();
        for (n, line) in text.lines().enumerate() {
            let line = line.split('#').next().unwrap_or("").trim();
            if line.is_empty() {
                continue;
            }
            let (key, value) = line
                .split_once('=')
                .ok_or_else(|| format!("line {}: expected `key = true|false`", n + 1))?;
            let change = VehicleChange::from_key(key.trim())
                .ok_or_else(|| format!("line {}: unknown setting `{}`", n + 1, key.trim()))?;
            let on = match value.trim() {
                "true" => true,
                "false" => false,
                v => return Err(format!("line {}: `{v}` is not true or false", n + 1)),
            };
            s.set(change, on);
        }
        Ok(s)
    }

    /// The settings file text, each setting with its warning as a comment.
    pub fn to_text(&self) -> String {
        let mut out =
            String::from("# DrorControl vehicle-change settings: all off by default (ADR-0012).\n");
        for c in VehicleChange::ALL {
            out.push_str(&format!(
                "\n# {}: {}\n{} = {}\n",
                c.title(),
                c.warning(),
                c.key(),
                self.get(c)
            ));
        }
        out
    }
}

/// Why a vehicle-changing action did not start; nothing was sent.
#[derive(Debug, Clone, PartialEq)]
pub enum VehicleChangeError {
    /// The setting that allows this action is off.
    SettingOff(VehicleChange),
    /// The vehicle is armed (or its state unknown) and the request did not carry the
    /// operator's acknowledgement of this warning: show it, then repeat with
    /// [`ArmedWrite::Acknowledged`].
    ArmedWarningRequired(&'static str),
    /// Any other reason (vehicle still booting, armed for a log erase, no logs listed, …).
    Refused(String),
}

impl fmt::Display for VehicleChangeError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            VehicleChangeError::SettingOff(VehicleChange::ArmedMissionUpload) => f.write_str(
                "the vehicle is armed or its armed state is unknown, and mission upload while \
                 armed is off in the settings (armed-mission-upload)",
            ),
            VehicleChangeError::SettingOff(c) => {
                write!(f, "{} is off in the settings ({})", c.title(), c.key())
            }
            VehicleChangeError::ArmedWarningRequired(w) => f.write_str(w),
            VehicleChangeError::Refused(why) => f.write_str(why),
        }
    }
}

impl std::error::Error for VehicleChangeError {}

/// Outcome of [`Core::erase_logs`].
#[derive(Debug, Clone, PartialEq)]
pub struct EraseReport {
    /// The logs listed before the erase.
    pub before: Vec<LogEntry>,
    /// The logs listed afterwards.
    pub after: Vec<LogEntry>,
}

impl EraseReport {
    /// No log listed before is listed any more. A log only grows, so a listed log counts as
    /// an old one when the same number was listed before at the same or a smaller size; a
    /// smaller one, or a number not listed before, is a new log the vehicle started after
    /// the erase. ArduPilot does that whenever its log file was still open, i.e. after any
    /// flight since boot (`AP_Logger_File::EraseAll`, `erase.was_logging`; a 77 kB log on
    /// 4.6.3 SITL). A log still being written can outgrow the old one with its number and
    /// then reads as not verified: see [`ERASE_NOT_VERIFIED_HINT`].
    pub fn verified(&self) -> bool {
        self.still_listed().is_empty()
    }

    /// Logs listed afterwards that can be ones from before the erase.
    pub fn still_listed(&self) -> Vec<LogEntry> {
        self.after
            .iter()
            .filter(|a| self.before.iter().any(|b| b.id == a.id && b.size <= a.size))
            .copied()
            .collect()
    }

    /// Logs the vehicle started after the erase.
    pub fn new_logs(&self) -> Vec<LogEntry> {
        let old = self.still_listed();
        self.after
            .iter()
            .filter(|a| !old.contains(a))
            .copied()
            .collect()
    }
}

/// Why an erase can read as not verified, and what to do.
pub const ERASE_NOT_VERIFIED_HINT: &str = "a log listed before the erase is still listed at the same or a larger size. ArduPilot keeps logging for 15 s after disarming or a failed arming attempt (and always with LOG_DISARMED set): wait until the vehicle has stopped logging, then erase again";

/// How long to keep listing the logs after `LOG_ERASE` (an SD card erase takes a while).
const ERASE_CONFIRM_TIME: Duration = Duration::from_secs(30);

impl Core {
    /// The vehicle-change settings now in force.
    pub fn vehicle_changes(&self) -> VehicleChangeSettings {
        *self
            .vehicle_changes
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
    }

    /// Turn a vehicle-change setting on or off (written to the audit log).
    pub fn set_vehicle_change(&self, change: VehicleChange, on: bool) {
        let mut s = self
            .vehicle_changes
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner);
        if s.get(change) != on {
            s.set(change, on);
            self.audit_note(
                0,
                format!(
                    "setting {} turned {}",
                    change.key(),
                    if on { "ON" } else { "off" }
                ),
            );
        }
    }

    /// Upload a mission, geofence or rally list to `target`, then download it again and
    /// compare (hazard H4): only [`MissionReport::verified`] means the vehicle holds exactly
    /// these items.
    ///
    /// * A fence or rally upload needs [`VehicleChange::FenceRallyUpload`] on.
    /// * A mission upload to a vehicle not known to be disarmed needs
    ///   [`VehicleChange::ArmedMissionUpload`] on.
    /// * To a vehicle not known to be disarmed, `armed` must be [`ArmedWrite::Acknowledged`]
    ///   (the operator saw [`ARMED_MISSION_UPLOAD_WARNING`] / [`ARMED_FENCE_RALLY_WARNING`]),
    ///   else [`VehicleChangeError::ArmedWarningRequired`].
    pub async fn upload_items(
        &self,
        target: (u8, u8),
        kind: MissionType,
        items: Vec<MissionItem>,
        armed: ArmedWrite,
    ) -> Result<MissionReport, VehicleChangeError> {
        let settings = self.vehicle_changes();
        let geo = kind != MissionType::Mission;
        if geo && !settings.fence_rally_upload {
            return Err(VehicleChangeError::SettingOff(
                VehicleChange::FenceRallyUpload,
            ));
        }
        let disarmed = self.facts(target.0).armed == Some(false);
        if !disarmed {
            if !geo && !settings.armed_mission_upload {
                return Err(VehicleChangeError::SettingOff(
                    VehicleChange::ArmedMissionUpload,
                ));
            }
            if armed != ArmedWrite::Acknowledged {
                return Err(VehicleChangeError::ArmedWarningRequired(if geo {
                    ARMED_FENCE_RALLY_WARNING
                } else {
                    ARMED_MISSION_UPLOAD_WARNING
                }));
            }
        }
        // ArduPilot reports MAV_STATE_BOOT until init_ardupilot() has finished, which includes
        // initialising mission storage; an upload before that is refused as NO_SPACE (seen in
        // SITL under load). Say what is really wrong instead.
        let booting = self
            .fleet()
            .get(target.0)
            .and_then(|v| v.heartbeat.as_ref())
            .is_none_or(|h| h.value.system_status == MavState::BOOT.0);
        if booting {
            return Err(VehicleChangeError::Refused(
                "the vehicle is still booting (MAV_STATE_BOOT); upload once it reports ready"
                    .into(),
            ));
        }
        let what = kind_name(kind);
        let allowed_by = match (geo, disarmed) {
            (true, _) => " (setting fence-rally-upload)",
            (false, false) => " (setting armed-mission-upload)",
            (false, true) => "",
        };
        self.audit_note(
            target.0,
            format!(
                "{what} upload started: {} items{}{allowed_by}",
                items.len(),
                if disarmed {
                    ""
                } else {
                    ", vehicle ARMED or state unknown, in-flight warning acknowledged"
                }
            ),
        );
        let (mut up, first) = MissionUpload::start(
            target,
            items.clone(),
            kind.mavlink(),
            TransferConfig::default(),
            Instant::now(),
        );
        self.drive(
            target,
            &mut up,
            first,
            MissionUpload::on_message,
            MissionUpload::poll,
            MissionUpload::status,
        )
        .await;
        // Read back whatever the upload said: a lost final ACK or a partial write shows here.
        let (readback, on_vehicle) = self.download_items(target, kind).await;
        let mismatches = if readback == mission::TransferStatus::Done {
            mission::verify_as(kind, &items, &on_vehicle)
        } else {
            Vec::new()
        };
        let report = MissionReport {
            upload: up.status().clone(),
            readback,
            on_vehicle,
            mismatches,
        };
        self.audit_note(
            target.0,
            if report.verified() {
                format!("{what} upload verified by read-back")
            } else {
                format!(
                    "{what} upload NOT verified: upload {:?}, read-back {:?}, {} mismatches",
                    report.upload,
                    report.readback,
                    report.mismatches.len()
                )
            },
        );
        Ok(report)
    }

    /// Erase every dataflash log on `target` (`LOG_ERASE`), then list the logs until the
    /// list is empty (up to 30 s). Needs [`VehicleChange::LogErase`] on and the
    /// vehicle known to be disarmed: ArduPilot ignores log requests while armed
    /// (`AP_Logger::handle_log_message`, 4.6.3 and 4.7.1). The logs cannot be recovered.
    pub async fn erase_logs(&self, target: (u8, u8)) -> Result<EraseReport, VehicleChangeError> {
        if !self.vehicle_changes().log_erase {
            return Err(VehicleChangeError::SettingOff(VehicleChange::LogErase));
        }
        self.require_disarmed(target.0)
            .map_err(VehicleChangeError::Refused)?;
        let before = self
            .list_logs(target)
            .await
            .map_err(|e| VehicleChangeError::Refused(format!("could not list the logs: {e}")))?;
        self.audit_note(
            target.0,
            format!(
                "log erase sent (setting log-erase): {} logs, {} bytes",
                before.len(),
                before.iter().map(|e| u64::from(e.size)).sum::<u64>()
            ),
        );
        self.send(&Message::LogErase(LogErase {
            target_system: target.0,
            target_component: target.1,
        }))
        .await;
        let deadline = Instant::now() + ERASE_CONFIRM_TIME;
        let mut after = before.clone();
        while Instant::now() < deadline {
            tokio::time::sleep(Duration::from_millis(500)).await;
            if let Ok(now) = self.list_logs(target).await {
                after = now;
                let r = EraseReport {
                    before: before.clone(),
                    after: after.clone(),
                };
                if r.verified() {
                    break;
                }
            }
        }
        let report = EraseReport { before, after };
        self.audit_note(
            target.0,
            if report.verified() {
                format!(
                    "log erase verified: {} earlier logs gone, {} new logs listed",
                    report.before.len(),
                    report.new_logs().len()
                )
            } else {
                format!(
                    "log erase NOT verified: {} logs listed after {} s ({ERASE_NOT_VERIFIED_HINT})",
                    report.after.len(),
                    ERASE_CONFIRM_TIME.as_secs()
                )
            },
        );
        Ok(report)
    }
}

fn kind_name(kind: MissionType) -> &'static str {
    match kind {
        MissionType::Mission => "mission",
        MissionType::Fence => "geofence",
        MissionType::Rally => "rally point",
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn everything_is_off_by_default() {
        let s = VehicleChangeSettings::default();
        assert!(VehicleChange::ALL.iter().all(|c| !s.get(*c)));
        assert_eq!(VehicleChangeSettings::parse("").unwrap(), s);
    }

    #[test]
    fn settings_file_round_trips_and_rejects_typos() {
        let mut s = VehicleChangeSettings::default();
        s.set(VehicleChange::FenceRallyUpload, true);
        let text = s.to_text();
        assert!(text.contains("fence-rally-upload = true"));
        assert!(text.contains(VehicleChange::LogErase.warning()));
        assert_eq!(VehicleChangeSettings::parse(&text).unwrap(), s);
        assert!(VehicleChangeSettings::parse("log-erase = yes").is_err());
        assert!(VehicleChangeSettings::parse("log_erase = true").is_err());
        assert!(VehicleChangeSettings::parse("log-erase").is_err());
        let on = VehicleChangeSettings::parse("  log-erase=true # erase\n").unwrap();
        assert!(on.log_erase && !on.fence_rally_upload);
    }

    #[test]
    fn every_setting_has_a_key_title_and_warning() {
        for c in VehicleChange::ALL {
            assert_eq!(VehicleChange::from_key(c.key()), Some(c));
            assert!(!c.title().is_empty() && c.warning().len() > 40);
        }
    }

    #[test]
    fn erase_is_verified_when_no_old_log_is_listed() {
        let e = |id, size| LogEntry {
            id,
            size,
            time_utc: 0,
        };
        let r = |after| EraseReport {
            before: vec![e(1, 100), e(2, 200)],
            after,
        };
        assert!(r(vec![]).verified());
        // A new, smaller log 1 started after the erase (ArduPilot after a flight).
        assert!(r(vec![e(1, 5)]).verified());
        assert_eq!(r(vec![e(1, 5)]).new_logs(), [e(1, 5)]);
        // An old log, unchanged or still growing: not erased.
        assert!(!r(vec![e(2, 200)]).verified());
        assert!(!r(vec![e(1, 5), e(2, 260)]).verified());
        assert_eq!(r(vec![e(1, 5), e(2, 260)]).still_listed(), [e(2, 260)]);
    }
}
