//! Generates the MAVLink dialect module from the vendored XML definitions.

use std::env;
use std::path::PathBuf;

fn main() {
    let root = if env::var_os("CARGO_FEATURE_DIALECT_ALL").is_some() {
        "all.xml"
    } else {
        "ardupilotmega.xml"
    };
    let manifest = PathBuf::from(env::var("CARGO_MANIFEST_DIR").expect("CARGO_MANIFEST_DIR"));
    let defs = manifest.join("definitions");
    println!("cargo:rerun-if-changed=definitions");

    let dialect = mavlink_gen::load_dialect(&defs.join(root))
        .unwrap_or_else(|e| panic!("cannot load MAVLink dialect {root}: {e}"));
    let code = mavlink_gen::generate(&dialect, root);
    let out = PathBuf::from(env::var("OUT_DIR").expect("OUT_DIR")).join("dialect.rs");
    std::fs::write(&out, code).unwrap_or_else(|e| panic!("cannot write {}: {e}", out.display()));
}
