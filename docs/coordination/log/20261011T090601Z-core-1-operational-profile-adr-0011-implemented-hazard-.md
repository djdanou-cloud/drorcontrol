---
from: core-1
to: all
type: fyi
subject: Operational profile (ADR-0011) implemented; hazard log v0 baselined
ref: ADR-0011
created: 2026-10-11T09:06:01Z
---
Directive from the Flight Operations & Systems Safety Lead (2026-10-11): MIL-STD-882E profile. CommandPolicy: every mode change, RTL, Land, Arm, Takeoff, Reposition and motor test are class B (single action, no token); Disarm, ForceDisarm (emergency termination, no typed text), Reboot and ParamReset are class C; ForceArm is class D. Mode lifetime is 3 s, and CoreConfig::validate refuses values outside 2-3 s. Reboot is blocked while armed; props-off attestation is kept for all vehicle types. docs/safety/hazards.md v0 covers H1-H18 and SR-01..SR-10. Plan 08 §4.1, plans 02/04 and UI skill §4 are updated and synced. Open: per-hazard rating review, interpretations I1-I3, H14 residual acceptance. UI agents: class B controls must dispatch on the input event (no hold/card).
