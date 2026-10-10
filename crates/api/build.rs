//! Compiles `proto/dc/v1/*.proto` with protox (pure Rust), so no system `protoc` is needed
//! on any developer machine or CI runner.

use std::path::PathBuf;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let proto = PathBuf::from(std::env::var("CARGO_MANIFEST_DIR")?).join("../../proto");
    println!("cargo:rerun-if-changed={}", proto.display());
    let fds = protox::compile(["dc/v1/core.proto"], [&proto])?;
    tonic_prost_build::configure().compile_fds(fds)?;
    Ok(())
}
