# ADR-0002: Four IPC planes (control, state, events, bulk)

* Status: proposed
* Date: 2026-10-10
* Validation: T-PERF-01 soak (200 k msg/s burst, zero dropped UI frames)

## Context
Telemetry bursts must never starve the UI thread; remote/headless clients need the same API; high-rate state is "latest value wins", commands/events are not.

## Options
ZeroMQ pub/sub everywhere; gRPC everywhere; in-process FFI + gRPC + lock-free snapshots; shared-memory only.

## Decision
P1 control = async calls via FFI in-process / gRPC remote; P2 state = lock-free double-buffered snapshot pulled by the UI at vsync (remote: decimated gRPC stream); P3 events = ordered bounded queue with replay cursor; P4 bulk = mmap/handles/chunked. ZeroMQ is not adopted.

## Consequences
* + UI cost independent of ingest rate; schema evolution via Protobuf + `buf breaking`.
* − Two access paths (FFI, gRPC) must stay semantically identical → shared conformance test suite.
