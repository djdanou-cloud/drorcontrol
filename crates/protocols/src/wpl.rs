//! Mission files in the `QGC WPL 110` text format (Mission Planner's `.waypoints`).
//!
//! One header line, then one line per item, fields separated by tabs (any whitespace is
//! accepted on reading):
//!
//! ```text
//! QGC WPL 110
//! seq  current  frame  command  p1  p2  p3  p4  x  y  z  autocontinue
//! ```
//!
//! Line 0 is the home position, as on ArduPilot. `x` and `y` are latitude and longitude in
//! degrees for commands that carry a location; for other commands they are plain numbers. That
//! is the rule ArduPilot applies when it converts `MISSION_ITEM` to `MISSION_ITEM_INT`
//! (`AP_Mission::cmd_has_location`), so a file read here means what it means to the vehicle.

use crate::mission::{MissionItem, has_location};

/// First line of every file.
pub const HEADER: &str = "QGC WPL 110";

/// A file that cannot be read as a mission.
#[derive(Debug, Clone, PartialEq)]
pub struct WplError {
    /// 1-based line number.
    pub line: usize,
    /// What is wrong.
    pub reason: String,
}

impl std::fmt::Display for WplError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "line {}: {}", self.line, self.reason)
    }
}

impl std::error::Error for WplError {}

/// Read a mission file. Item numbers must run 0, 1, 2, … without gaps; blank lines are
/// skipped. Nothing is guessed: a value that does not fit its field is an error.
pub fn parse(text: &str) -> Result<Vec<MissionItem>, WplError> {
    let err = |line: usize, reason: String| WplError { line, reason };
    let mut lines = text
        .lines()
        .enumerate()
        .filter(|(_, l)| !l.trim().is_empty());
    match lines.next() {
        Some((_, l)) if l.trim() == HEADER => {}
        Some((n, l)) => {
            return Err(err(
                n + 1,
                format!("expected \"{HEADER}\", found {:?}", l.trim()),
            ));
        }
        None => return Err(err(1, "empty file".into())),
    }
    let mut items = Vec::new();
    for (n, l) in lines {
        let line = n + 1;
        let f: Vec<&str> = l.split_whitespace().collect();
        if f.len() != 12 {
            return Err(err(line, format!("expected 12 fields, found {}", f.len())));
        }
        let int = |i: usize, name: &str, max: u64| -> Result<u64, WplError> {
            f[i].parse::<u64>()
                .ok()
                .filter(|v| *v <= max)
                .ok_or_else(|| {
                    err(
                        line,
                        format!("{name} {:?} is not an integer 0..={max}", f[i]),
                    )
                })
        };
        let num = |i: usize, name: &str| -> Result<f64, WplError> {
            f[i].parse::<f64>()
                .map_err(|_| err(line, format!("{name} {:?} is not a number", f[i])))
        };
        let seq = int(0, "item number", u64::from(u16::MAX))?;
        if seq != items.len() as u64 {
            return Err(err(
                line,
                format!("item number {seq}, expected {}", items.len()),
            ));
        }
        int(1, "current flag", 1)?;
        let frame = int(2, "frame", u64::from(u8::MAX))? as u8;
        let command = int(3, "command", u64::from(u16::MAX))? as u16;
        let mut params = [0f32; 4];
        for (k, p) in params.iter_mut().enumerate() {
            *p = num(4 + k, &format!("param{}", k + 1))? as f32;
        }
        let (x, y) = (num(8, "x")?, num(9, "y")?);
        let (x, y) = if has_location(command) {
            if !(-90.0..=90.0).contains(&x) || !(-180.0..=180.0).contains(&y) {
                return Err(err(
                    line,
                    format!("position {x}, {y} is not a latitude, longitude"),
                ));
            }
            ((x * 1e7).round() as i32, (y * 1e7).round() as i32)
        } else {
            let whole = |v: f64, name: &str| {
                (v.fract() == 0.0 && v >= f64::from(i32::MIN) && v <= f64::from(i32::MAX))
                    .then_some(v as i32)
                    .ok_or_else(|| {
                        err(
                            line,
                            format!("{name} {v} must be a whole number for command {command}"),
                        )
                    })
            };
            (whole(x, "x")?, whole(y, "y")?)
        };
        let z = num(10, "z")? as f32;
        let autocontinue = int(11, "autocontinue", 1)? == 1;
        items.push(MissionItem {
            frame,
            command,
            params,
            x,
            y,
            z,
            autocontinue,
        });
    }
    Ok(items)
}

/// Write a mission file (item 0 is marked current, as Mission Planner does for home).
pub fn to_string(items: &[MissionItem]) -> String {
    let mut s = String::from(HEADER);
    s.push('\n');
    for (seq, i) in items.iter().enumerate() {
        let (x, y) = if has_location(i.command) {
            (
                format!("{:.7}", f64::from(i.x) * 1e-7),
                format!("{:.7}", f64::from(i.y) * 1e-7),
            )
        } else {
            (i.x.to_string(), i.y.to_string())
        };
        let p = i.params;
        s.push_str(&format!(
            "{seq}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{x}\t{y}\t{}\t{}\n",
            u8::from(seq == 0),
            i.frame,
            i.command,
            p[0],
            p[1],
            p[2],
            p[3],
            i.z,
            u8::from(i.autocontinue),
        ));
    }
    s
}

#[cfg(test)]
mod tests {
    use super::*;

    /// As Mission Planner 1.3 writes it (home, takeoff, waypoint, speed change, RTL).
    const MP_FILE: &str = "QGC WPL 110\r\n\
0\t1\t0\t16\t0\t0\t0\t0\t-35.363261\t149.165230\t584.090000\t1\r\n\
1\t0\t3\t22\t0.00000000\t0.00000000\t0.00000000\t0.00000000\t0.00000000\t0.00000000\t20.000000\t1\r\n\
2\t0\t3\t16\t0.00000000\t0.00000000\t0.00000000\t0.00000000\t-35.36210320\t149.16588690\t30.000000\t1\r\n\
3\t0\t3\t178\t1.00000000\t8.00000000\t-1.00000000\t0.00000000\t0.00000000\t0.00000000\t0.000000\t1\r\n\
4\t0\t3\t20\t0.00000000\t0.00000000\t0.00000000\t0.00000000\t0.00000000\t0.00000000\t0.000000\t1\r\n";

    #[test]
    fn reads_a_mission_planner_file() {
        let m = parse(MP_FILE).unwrap();
        assert_eq!(m.len(), 5);
        assert_eq!(
            (m[0].x, m[0].y, m[0].z),
            (-353_632_610, 1_491_652_300, 584.09)
        );
        assert_eq!((m[1].command, m[1].frame, m[1].z), (22, 3, 20.0));
        assert_eq!((m[2].x, m[2].y), (-353_621_032, 1_491_658_869));
        assert_eq!(m[3].params, [1.0, 8.0, -1.0, 0.0]);
        assert!(m.iter().all(|i| i.autocontinue));
    }

    #[test]
    fn round_trips() {
        let m = parse(MP_FILE).unwrap();
        assert_eq!(parse(&to_string(&m)).unwrap(), m);
        assert!(to_string(&m).starts_with("QGC WPL 110\n0\t1\t0\t16\t"));
    }

    #[test]
    fn fence_and_rally_items_round_trip() {
        // A 3-vertex inclusion polygon (NAV_FENCE_POLYGON_VERTEX_INCLUSION 5001, param1 =
        // vertex count), an exclusion circle (5004, param1 = radius) and a rally point (5100).
        let it = |command: u16, p1: f32, lat: f64, lon: f64, z: f32| MissionItem {
            frame: if command == 5100 { 3 } else { 0 },
            command,
            params: [p1, 0.0, 0.0, 0.0],
            x: (lat * 1e7).round() as i32,
            y: (lon * 1e7).round() as i32,
            z,
            autocontinue: true,
        };
        let fence = vec![
            it(5001, 3.0, -35.36, 149.16, 0.0),
            it(5001, 3.0, -35.36, 149.17, 0.0),
            it(5001, 3.0, -35.37, 149.165, 0.0),
            it(5004, 30.0, -35.365, 149.165, 0.0),
            it(5100, 0.0, -35.362, 149.163, 25.0),
        ];
        let text = to_string(&fence);
        assert!(
            text.contains("\t5001\t3\t0\t0\t0\t-35.3600000\t149.1600000\t"),
            "{text}"
        );
        assert_eq!(parse(&text).unwrap(), fence);
    }

    #[test]
    fn non_location_commands_keep_raw_x_y() {
        // Commands without a location (here DO_SET_SERVO) keep integers in x/y, unscaled.
        let text = "QGC WPL 110\n0\t1\t0\t16\t0\t0\t0\t0\t0\t0\t0\t1\n\
                    1\t0\t2\t183\t9\t1500\t0\t0\t7\t-3\t0\t1\n";
        let m = parse(text).unwrap();
        assert_eq!((m[1].x, m[1].y), (7, -3));
        assert_eq!(parse(&to_string(&m)).unwrap(), m);
        let bad = text.replace("\t7\t-3\t", "\t7.5\t-3\t");
        assert!(parse(&bad).unwrap_err().reason.contains("whole number"));
    }

    #[test]
    fn rejects_what_it_cannot_read() {
        let e = |t: &str| parse(t).unwrap_err();
        assert_eq!(e("").reason, "empty file");
        assert_eq!(e("QGC WPL 120\n").line, 1);
        let ok = "QGC WPL 110\n0\t1\t0\t16\t0\t0\t0\t0\t-35\t149\t0\t1\n";
        assert!(parse(ok).is_ok());
        assert_eq!(
            e(&format!("{ok}2\t0\t3\t16\t0\t0\t0\t0\t-35\t149\t9\t1\n")).line,
            3
        );
        assert!(
            e(&format!("{ok}1\t0\t3\t16\t0\t0\t0\t0\t-35\t149\n"))
                .reason
                .contains("12 fields")
        );
        assert!(
            e(&format!("{ok}1\t0\t3\t16\t0\t0\t0\t0\t-95\t149\t9\t1\n"))
                .reason
                .contains("latitude")
        );
        assert!(
            e(&format!("{ok}1\t0\t3\t16\tx\t0\t0\t0\t-35\t149\t9\t1\n"))
                .reason
                .contains("param1")
        );
        assert!(
            e(&format!("{ok}1\t0\t3\t70000\t0\t0\t0\t0\t-35\t149\t9\t1\n"))
                .reason
                .contains("command")
        );
    }
}
