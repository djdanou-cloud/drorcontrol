# DrorControl task shortcuts. `just` is optional: every recipe wraps `cargo xtask`, which is what CI runs.
# Install just: `brew install just` / `winget install Casey.Just` / `cargo install just`.

default:
    @just --list

# Install the pinned Rust toolchain from rust-toolchain.toml
bootstrap:
    rustup toolchain install
    cargo xtask doctor

# Build the whole workspace
build:
    cargo build --workspace --locked

# Everything CI runs
ci:
    cargo xtask ci

fmt:
    cargo xtask fmt

lint:
    cargo xtask lint

test:
    cargo xtask test

doctor:
    cargo xtask doctor
