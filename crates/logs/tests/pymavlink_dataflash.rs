//! The dataflash parser against pymavlink `DFReader` (the reference) on real logs: an
//! ArduCopter 4.7.1 SITL log and an ArduCopter 4.6.2 flight on a MatekH743 (see
//! docs/corpus/README.md). Fixtures and references from `scripts/dataflash_vectors.py`.

use std::path::Path;

use logs::Log;
use serde_json::Value;

const SITL: &str = include_str!("data/sitl-copter-4.7.1-head.json");
const MATEK: &str = include_str!("data/copter-4.6.2-matekh743-head.json");

fn same(a: f64, b: &Value) -> bool {
    match b.as_f64() {
        Some(v) => a == v || (a - v).abs() <= 1e-12 * v.abs().max(1.0),
        None => false,
    }
}

fn data_dir() -> std::path::PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/data")
}

#[test]
fn matches_pymavlink_on_sitl_log() {
    check(SITL, &data_dir());
}

#[test]
fn matches_pymavlink_on_real_flight_log() {
    check(MATEK, &data_dir());
}

/// Full-size logs that are not committed: point `DC_LOG_CORPUS` at a directory of
/// `scripts/dataflash_vectors.py` outputs and run `cargo test -p logs -- --ignored`.
#[test]
#[ignore = "needs DC_LOG_CORPUS"]
fn matches_pymavlink_on_external_corpus() {
    let dir = std::env::var("DC_LOG_CORPUS").expect("set DC_LOG_CORPUS");
    let dir = Path::new(&dir);
    let mut n = 0;
    for e in std::fs::read_dir(dir).unwrap() {
        let p = e.unwrap().path();
        if p.extension().is_some_and(|x| x == "json") {
            eprintln!("checking {}", p.display());
            check(&std::fs::read_to_string(&p).unwrap(), dir);
            n += 1;
        }
    }
    assert!(n > 0, "no *.json references in {}", dir.display());
}

fn check(reference: &str, dir: &Path) {
    let r: Value = serde_json::from_str(reference).unwrap();
    let path = dir.join(r["fixture"].as_str().unwrap());
    let log = Log::open(&path).unwrap();
    assert_eq!(log.stats.skipped_bytes, 0, "fixture is clean");

    // 1. Same messages, same counts.
    let counts = log.counts();
    let want: std::collections::BTreeMap<String, usize> = r["counts"]
        .as_object()
        .unwrap()
        .iter()
        .map(|(k, v)| (k.clone(), v.as_u64().unwrap() as usize))
        .collect();
    assert_eq!(counts, want);

    // 2. Every numeric column: count, min, max, first, last.
    let mut failures = Vec::new();
    for (key, s) in r["stats"].as_object().unwrap() {
        let (msg, field) = key.split_once('.').unwrap();
        let Some(col) = log.column(msg, field, None) else {
            failures.push(format!("{key}: column missing"));
            continue;
        };
        // Same rule as the reference script: non-finite values (e.g. a NaN parameter
        // default) are not representable in JSON and are excluded on both sides.
        let v: Vec<f64> = col
            .values
            .iter()
            .copied()
            .filter(|x| x.is_finite())
            .collect();
        let ok = v.len() as u64 == s["count"].as_u64().unwrap()
            && same(v.iter().cloned().fold(f64::INFINITY, f64::min), &s["min"])
            && same(
                v.iter().cloned().fold(f64::NEG_INFINITY, f64::max),
                &s["max"],
            )
            && same(v[0], &s["first"])
            && same(*v.last().unwrap(), &s["last"]);
        if !ok {
            failures.push(format!(
                "{key}: rust n={} min/max/first/last differ from {s}",
                v.len()
            ));
        }
    }

    // 3. Sampled records: every numeric field.
    for smp in r["samples"].as_array().unwrap() {
        let msg = smp["msg"].as_str().unwrap();
        let i = smp["index"].as_u64().unwrap() as usize;
        let fmt = log.format_by_name(msg).unwrap();
        let rec = log.record(msg, i).unwrap();
        for (field, want) in smp["fields"].as_object().unwrap() {
            let fi = fmt.field(field).unwrap();
            match fmt.value(rec, fi) {
                Some(got) if same(got, want) => {}
                got => failures.push(format!(
                    "{msg}[{i}].{field}: rust {got:?} vs pymavlink {want}"
                )),
            }
        }
    }
    assert!(
        failures.is_empty(),
        "{}: {} mismatches:\n{}",
        path.display(),
        failures.len(),
        failures.join("\n")
    );
}

#[test]
fn survives_corruption_and_truncation() {
    let r: Value = serde_json::from_str(SITL).unwrap();
    let path = data_dir().join(r["fixture"].as_str().unwrap());
    let mut bytes = std::fs::read(&path).unwrap();
    let clean = Log::open(&path).unwrap().stats.records;
    // Smash 64 bytes in the middle and cut the tail mid-record.
    let mid = bytes.len() / 2;
    bytes[mid..mid + 64].fill(0x55);
    bytes.truncate(bytes.len() - 7);
    let tmp = std::env::temp_dir().join(format!("dc-logs-corrupt-{}.bin", std::process::id()));
    std::fs::write(&tmp, &bytes).unwrap();
    let log = Log::open(&tmp).unwrap();
    let _ = std::fs::remove_file(&tmp);
    assert!(log.stats.skipped_bytes > 0);
    assert!(
        log.stats.records >= clean - 4,
        "lost at most the damaged records: {} vs {clean}",
        log.stats.records
    );
    assert!(log.column("ATT", "Roll", None).is_some());
}
