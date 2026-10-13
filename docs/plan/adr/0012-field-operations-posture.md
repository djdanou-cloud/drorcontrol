# ADR-0012: Field-operations posture (owner decisions of 2026-10-12)

* Status: accepted
* Date: 2026-10-12
* Deciders: Owner (DrorAero), with the hazard-log review (I1–I3, H14) signed off in the same
  decision
* Validation: to be added per item as it is implemented (see "Consequences")

## Context
The open decisions in plan README §5, the hazard log's open items (`docs/safety/hazards.md` §7)
and the features held back at the safety gate (parameter writes, mission/fence/rally upload,
camera commands, log erase) were put to the owner on 2026-10-12. The owner set the product's
design principle and answered every item.

**Design principle:** DrorControl is an advanced operational tool for power users in
high-tempo field operations. Operators must not be blocked by intrusive safeguards, slow typed
confirmations or certification paperwork; the software must be lean, permissive and responsive.

## Decision

### Vehicle-changing actions
| # | Item | Decision |
|---|------|----------|
| 1 | Parameter writes | Enabled for **all** parameters, including failsafe, arming, fence and board safety. **No typed confirmation**: a single-click modal or toggle is enough. **Also while armed and flying** (owner, 2026-10-13): the client must first show a warning pop-up that the vehicle is armed and the change takes effect in flight; the write goes ahead once the operator acknowledges it. |
| 2 | Mission upload in flight (T0-19) | Allowed, with one single-click confirmation ("Update mission on active vehicle?"). |
| 3 | Mission upload from the CLI | Enabled with no state lock (not limited to disarmed). |
| 4 | Geofence and rally upload | Enabled on the ground and in flight, **with read-back verification**. |
| 5 | Camera trigger commands | May be added to generated missions (surveys). |
| 6 | Dataflash log erase | Allowed immediately; no download required first. |
| 7 | Hazard log | Interpretations I1–I3 signed off (I3 as amended by item 2), H14 residual risk accepted, review approved. Feature work does not wait on certification paperwork. |

### Legal and licensing
| # | Item | Decision |
|---|------|----------|
| 8 | Product licence | Closed source, clean-room; no GPL code is brought in. |
| 9 | MAVLink XML, generated code, pymavlink | XML and generated code in the repository; pymavlink as a dev/test tool. |
| 10 | Third-party code | Permissive licences (BSD, MIT, Apache-2.0; e.g. MapLibre) are approved. Prefer mature crates over custom low-level code (e.g. FFT, maths). |
| 11 | Trademark "DrorControl" | Deferred, low priority. |
| 12 | Font | Inter (OFL). |

### Scope and architecture
| # | Item | Decision |
|---|------|----------|
| 13 | Hebrew / RTL | P3; English is the primary interface. |
| 14 | Sandboxed Mac App Store build | No. |
| 15 | ArduPilot versions | The fleet runs 4.6.x: add ArduPilot 4.6 SITL scenarios to CI. |
| 16 | Logs in the repository | Never commit production or field logs with real-world positions; only synthetic, sanitised or SITL logs. |
| 17 | UI frame target | "No frame over 25 ms" (40+ fps baseline), recorded in ADR-0001; no work on idle sub-frame jitter. |
| 18 | gRPC API | Configurable bind address, including all interfaces, for distributed field set-ups; secured with a pre-shared key (bearer token). |
| 21 | CI minutes | Full 4-OS SITL matrices on releases and schedules only; lean checks on pushes and PRs. |

## What stays unchanged
Not part of these decisions, so ADR-0011 and the core keep them:
* Two-stage (slide, no typed text) confirmation for in-flight disarm, autopilot reboot and
  emergency motor termination (ADR-0011 D2); reboot blocked while armed (D4); props-off
  attestation for motor test (D5); command lifetimes (D3).
* Force-arm (class D, typed) and parameter reset: not addressed; unchanged until decided.
* Read-back verification of every upload, the tamper-evident audit log, and the vehicle's own
  pre-arm checks.

## Consequences
* The core's command policy changes: parameter writes lose the "dangerous parameter" refusal
  (T0-18) and need only a single confirmation token; a write to an armed vehicle is answered
  with the in-flight warning (`ParamOperationError::ArmedWarningRequired`) until the request
  carries the operator's acknowledgement (`ArmedWrite::Acknowledged`), with the same
  baseline, read-back and stop-on-heartbeat-gap rules; in-flight mission upload becomes a
  single-confirmation class; fence/rally upload and log erase are new commands. Each lands with
  tests and SITL scenarios, and every change stays in the audit log.
* Residual risks the owner accepts with this posture: a wrong failsafe, arming or fence value
  can be written with one click; an in-flight mission or fence change takes effect at once;
  erased logs cannot be recovered. They are recorded in the hazard log (§6).
* CI gains a 4.6 SITL build; the full SITL matrix moves to scheduled and release workflows.
* The gRPC API gains authentication before it may listen beyond loopback.
* Custom primitives written while dependencies were frozen (the radix-2 FFT in
  `crates/logs/src/spectrum.rs`) may be replaced by mature crates.
