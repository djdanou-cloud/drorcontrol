---
from: architect
to: human
type: decision-request
subject: Reconcile drorcontrol-ui skill with plan 04 (palette, hold times, stack, touch targets)
ref: .claude/skills/drorcontrol-ui/SKILL.md
created: 2026-10-10T21:05:32Z
---
The owner added .claude/skills/drorcontrol-ui/SKILL.md (human instruction => ranks above docs/plan). It differs from docs/plan/04-ux-design-system.md and ADR-0001 in these places. Until resolved, UI agents: follow the SKILL for semantics/patterns, keep colours as tokens (never literals) so either palette can be applied.

1. STACK: skill assumes web tech (MapLibre/Cesium/Mapbox, CSS backdrop-filter, WebSocket/WebSerial, Zustand). ADR-0001 = Rust core + Qt 6/QML. Proposed: keep ADR-0001; apply the skill's rules as design requirements, translated to QML/RHI (e.g. glass blur via ShaderEffectSource is GPU/battery-costly -> gate behind a setting; Daylight theme stays matte/opaque for sunlight legibility).
2. STATUS PALETTE: skill = critical #FF3B30, warning #FF9500, caution #FFCC00, nominal #34C759, guidance #00C7BE, standby #8E8E93 (6 levels). Plan 04 s1.2 = warning #FF3B30, caution #FF9400 (brand orange), normal #2ECC71, advisory #22D3EE, guidance #E040FB (red/amber/green/cyan/magenta). Proposed: adopt the skill's 6-level taxonomy and hex values as the Standard-theme tokens (it is the newer owner instruction); Daylight and Night-Ops variants still need derived values (#FFCC00 on white fails AAA); keep shape+text redundancy for colour-blind safety. Note skill puts "Motors Armed" at Critical - confirm that is intended (plan treats armed as a state, not an alarm).
3. SURFACES: skill = #0B0E14 / #121824 / #1B2333 slate; DrorAero brand = Carbon #212121 / Graphite #424242. Proposed: use slate for flight/C2 UI, brand colours for marketing/docs/PDF and brand accents outside status semantics.
4. HOLD TIME: skill = slide-to-confirm or hold 1.5 s for Arm/Disarm/RTL/Kill/Override. Plan 08 s4.1 = RTL press-and-hold 0.8 s + <=5 s confirm card (emergency speed), Kill 1.5 s + typed confirm. Proposed: 1.5 s default for all listed commands; RTL emergency exception needs your explicit approval (safety path).
5. TOUCH TARGETS: skill 48x48 px minimum; plan 44 pt (56 pt gloved profile). Proposed: 48 px minimum everywhere, 56 px in gloved profile.
6. ENUMS: skill's mode grouping (Manual/Stabilized vs Autonomous vs Emergency) is a UI classification only; mode lists still come from ArduPilot metadata per vehicle.
Needed: yes/no per item 1-5 (or edits). Architect will then update docs/plan/04 + ADRs and create design tokens (task T0-12).
