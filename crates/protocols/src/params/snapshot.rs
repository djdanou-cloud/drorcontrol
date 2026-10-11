//! Offline groundwork for parameter changes (T0-18, hazard H5).
//!
//! Capture a complete download before preparing edits. A plan preserves the original
//! values and MAVLink types for rollback, and includes only edited parameters. Nothing
//! here sends a message, grants approval, or decides whether an armed vehicle may be edited.

use std::collections::{BTreeMap, BTreeSet};
use std::fmt;
use std::fmt::Write;

use sha2::{Digest, Sha256};

use super::Param;

/// An invalid snapshot or proposed edit. Planning fails as a whole, never partly.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum SnapshotError {
    /// A broadcast target cannot identify the vehicle/component to restore.
    BroadcastTarget,
    /// The download is empty, incomplete, or has duplicate/out-of-range indices.
    IncompleteDownload,
    /// Parameter identifiers must be 1–16 ASCII letters, digits or underscores.
    InvalidName(String),
    /// An edit names a parameter absent from the original download.
    UnknownParameter(String),
    /// The value is non-finite or cannot be represented by its C-cast MAVLink type.
    InvalidValue(String),
    /// An unrecognised MAVLink parameter type cannot be used for restoration.
    InvalidType(String),
    /// The stored snapshot is malformed or uses an unsupported format version.
    InvalidFormat,
    /// Stored bytes do not match the snapshot's corruption-detection checksum.
    ChecksumMismatch,
    /// A comparison used a snapshot from a different system/component.
    TargetMismatch,
}

impl fmt::Display for SnapshotError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::BroadcastTarget => {
                f.write_str("snapshot requires a specific system and component")
            }
            Self::IncompleteDownload => {
                f.write_str("snapshot requires a complete parameter download")
            }
            Self::InvalidName(name) => write!(f, "invalid parameter name {name:?}"),
            Self::UnknownParameter(name) => write!(f, "parameter {name:?} absent from snapshot"),
            Self::InvalidValue(name) => write!(f, "value for {name:?} cannot be represented"),
            Self::InvalidType(name) => write!(f, "unsupported parameter type for {name:?}"),
            Self::InvalidFormat => f.write_str("invalid parameter snapshot format"),
            Self::ChecksumMismatch => f.write_str("parameter snapshot checksum mismatch"),
            Self::TargetMismatch => f.write_str("parameter snapshots belong to different targets"),
        }
    }
}

impl std::error::Error for SnapshotError {}

/// An immutable baseline from a completed parameter download, bound to one component.
///
/// [`Self::to_text`] preserves it across restarts. The eventual write driver must durably
/// persist its baseline and obtain policy approval before sending any parameter set.
#[derive(Debug, Clone, PartialEq)]
pub struct ParamSnapshot {
    target: (u8, u8),
    params: BTreeMap<String, Param>,
}

/// One edit in a preview, including the value and type needed to undo it.
#[derive(Debug, Clone, PartialEq)]
pub struct ParamChange {
    /// Parameter name (never truncated to fit the wire format).
    pub name: String,
    /// Original downloaded value, type and index.
    pub before: Param,
    /// Proposed C-cast value with the original type and index.
    pub after: Param,
}

/// A deterministic preview of a batch. This is data, not permission to perform writes.
#[derive(Debug, Clone, PartialEq)]
pub struct ParamPlan {
    target: (u8, u8),
    changes: Vec<ParamChange>,
}

/// A difference between a planned value and a subsequent downloaded snapshot.
#[derive(Debug, Clone, PartialEq)]
pub struct ParamMismatch {
    /// Parameter whose value or type differs, or which is absent.
    pub name: String,
    /// Value and type requested by the preview.
    pub expected: Param,
    /// What was downloaded, or `None` if the parameter is absent.
    pub actual: Option<Param>,
}

impl ParamSnapshot {
    /// Capture exactly `expected_count` distinct indices from a completed download.
    ///
    /// The caller must supply the vehicle's announced `param_count`, not a partial map's
    /// length. The classic parameter protocol reserves `u16::MAX` as an unknown count/index.
    pub fn capture(
        target: (u8, u8),
        expected_count: u16,
        params: BTreeMap<String, Param>,
    ) -> Result<Self, SnapshotError> {
        if target.0 == 0 || target.1 == 0 {
            return Err(SnapshotError::BroadcastTarget);
        }
        if expected_count == 0
            || expected_count == u16::MAX
            || params.len() != usize::from(expected_count)
        {
            return Err(SnapshotError::IncompleteDownload);
        }
        let mut indices = BTreeSet::new();
        for (name, param) in &params {
            if name.is_empty()
                || name.len() > 16
                || !name.bytes().all(|b| b.is_ascii_alphanumeric() || b == b'_')
            {
                return Err(SnapshotError::InvalidName(name.clone()));
            }
            representable(name, f64::from(param.value), param.ty)?;
            if param.index >= expected_count || !indices.insert(param.index) {
                return Err(SnapshotError::IncompleteDownload);
            }
        }
        Ok(Self { target, params })
    }

    /// The exact vehicle/component whose values were captured.
    pub fn target(&self) -> (u8, u8) {
        self.target
    }

    /// Original values, types and indices. Preparing a plan never mutates them.
    pub fn params(&self) -> &BTreeMap<String, Param> {
        &self.params
    }

    /// Serialize the complete baseline, including its target, count, types and indices.
    /// Float bits are encoded exactly (including signed zero); a SHA-256 trailer detects
    /// accidental corruption. This is a versioned snapshot format, not a `.param` file.
    /// The checksum is not an authentication signature or a policy approval.
    pub fn to_text(&self) -> String {
        let mut text = format!(
            "DC_PARAM_SNAPSHOT_V1,{},{},{}\n",
            self.target.0,
            self.target.1,
            self.params.len()
        );
        for (name, param) in &self.params {
            writeln!(
                text,
                "{name},{:08x},{},{}",
                param.value.to_bits(),
                param.ty,
                param.index
            )
            .expect("writing to a String cannot fail");
        }
        let digest = checksum(&text);
        writeln!(text, "SHA256,{digest}").expect("writing to a String cannot fail");
        text
    }

    /// Load a saved baseline, checking corruption and revalidating completeness and values.
    /// An edited target or lost row cannot silently become a rollback baseline.
    pub fn from_text(text: &str) -> Result<Self, SnapshotError> {
        let (body, trailer) = text
            .rsplit_once("\nSHA256,")
            .ok_or(SnapshotError::InvalidFormat)?;
        let body = format!("{body}\n");
        let digest = trailer
            .strip_suffix('\n')
            .ok_or(SnapshotError::InvalidFormat)?;
        if digest.len() != 64 || !digest.bytes().all(|b| b.is_ascii_hexdigit()) {
            return Err(SnapshotError::InvalidFormat);
        }
        if checksum(&body) != digest {
            return Err(SnapshotError::ChecksumMismatch);
        }
        let mut lines = body.lines();
        let header: Vec<_> = lines
            .next()
            .ok_or(SnapshotError::InvalidFormat)?
            .split(',')
            .collect();
        let ["DC_PARAM_SNAPSHOT_V1", sysid, compid, count] = header.as_slice() else {
            return Err(SnapshotError::InvalidFormat);
        };
        let parse_u16 = |s: &str| s.parse::<u16>().map_err(|_| SnapshotError::InvalidFormat);
        let parse_u8 = |s: &str| s.parse::<u8>().map_err(|_| SnapshotError::InvalidFormat);
        let target = (parse_u8(sysid)?, parse_u8(compid)?);
        let count = parse_u16(count)?;
        let mut params = BTreeMap::new();
        for line in lines {
            let cols: Vec<_> = line.split(',').collect();
            let [name, bits, ty, index] = cols.as_slice() else {
                return Err(SnapshotError::InvalidFormat);
            };
            if bits.len() != 8 || !bits.bytes().all(|b| b.is_ascii_hexdigit()) {
                return Err(SnapshotError::InvalidFormat);
            }
            let value = f32::from_bits(
                u32::from_str_radix(bits, 16).map_err(|_| SnapshotError::InvalidFormat)?,
            );
            let param = Param {
                value,
                ty: parse_u8(ty)?,
                index: parse_u16(index)?,
            };
            if params.insert(name.to_string(), param).is_some() {
                return Err(SnapshotError::IncompleteDownload);
            }
        }
        Self::capture(target, count, params)
    }

    /// Preview edits (e.g. values from [`super::parse_param_file`]). Unknown names,
    /// non-finite values, integer fractions and integers losing wire precision are errors.
    /// Float values are compared after narrowing to the protocol's `f32` representation.
    pub fn plan(&self, edits: &BTreeMap<String, f64>) -> Result<ParamPlan, SnapshotError> {
        let mut changes = Vec::new();
        for (name, &value) in edits {
            let before = self
                .params
                .get(name)
                .ok_or_else(|| SnapshotError::UnknownParameter(name.clone()))?;
            let value = representable(name, value, before.ty)?;
            if value != before.value {
                changes.push(ParamChange {
                    name: name.clone(),
                    before: *before,
                    after: Param { value, ..*before },
                });
            }
        }
        Ok(ParamPlan {
            target: self.target,
            changes,
        })
    }
}

fn checksum(text: &str) -> String {
    let hash = Sha256::digest(text.as_bytes());
    hash.iter().map(|byte| format!("{byte:02x}")).collect()
}

impl ParamPlan {
    /// Original target; a rollback must never be retargeted to another component.
    pub fn target(&self) -> (u8, u8) {
        self.target
    }

    /// Changed parameters in name order, with unchanged entries omitted.
    pub fn changes(&self) -> &[ParamChange] {
        &self.changes
    }

    /// Compare this batch's planned values/types with a later complete download from
    /// the same target. Unrelated parameters and renumbered indices are ignored.
    /// An empty result only means these supplied snapshots agree; it does not establish
    /// that writes were sent or that the download is fresh. The future driver owns that.
    pub fn compare(&self, downloaded: &ParamSnapshot) -> Result<Vec<ParamMismatch>, SnapshotError> {
        if downloaded.target != self.target {
            return Err(SnapshotError::TargetMismatch);
        }
        Ok(self
            .changes
            .iter()
            .filter_map(|change| {
                let actual = downloaded.params.get(&change.name).copied();
                if actual.is_some_and(|p| p.value == change.after.value && p.ty == change.after.ty)
                {
                    None
                } else {
                    Some(ParamMismatch {
                        name: change.name.clone(),
                        expected: change.after,
                        actual,
                    })
                }
            })
            .collect())
    }

    /// Preview restoring only this batch's edited parameters to their original values.
    ///
    /// Restoration needs its own current-state checks, approval and read-back verification
    /// in the future write driver. This method does not restore the vehicle or mark success.
    pub fn rollback(&self) -> Self {
        Self {
            target: self.target,
            changes: self
                .changes
                .iter()
                .map(|c| ParamChange {
                    name: c.name.clone(),
                    before: c.after,
                    after: c.before,
                })
                .collect(),
        }
    }
}

fn representable(name: &str, value: f64, ty: u8) -> Result<f32, SnapshotError> {
    let invalid = || SnapshotError::InvalidValue(name.to_string());
    let wire = value as f32;
    if !value.is_finite() || !wire.is_finite() || (value != 0.0 && wire == 0.0) {
        return Err(invalid());
    }
    // MAV_PARAM_TYPE values are pinned in definitions/common.xml. Bounds are exclusive
    // at the upper end, avoiding f64 rounding of i64::MAX/u64::MAX to a power of two.
    let bounds = match ty {
        1 => Some((0.0, 256.0)),
        2 => Some((-128.0, 128.0)),
        3 => Some((0.0, 65536.0)),
        4 => Some((-32768.0, 32768.0)),
        5 => Some((0.0, 4294967296.0)),
        6 => Some((-2147483648.0, 2147483648.0)),
        7 => Some((0.0, 18446744073709551616.0)),
        8 => Some((-9223372036854775808.0, 9223372036854775808.0)),
        9 | 10 => None,
        _ => return Err(SnapshotError::InvalidType(name.to_string())),
    };
    if let Some((min, end)) = bounds
        && (value.fract() != 0.0 || value < min || value >= end || f64::from(wire) != value)
    {
        return Err(invalid());
    }
    Ok(wire)
}
