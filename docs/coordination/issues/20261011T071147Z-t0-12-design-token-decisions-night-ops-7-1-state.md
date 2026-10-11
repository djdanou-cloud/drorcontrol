---
title: T0-12 design-token decisions: Night-Ops 7:1, state colours on panels, Daylight caution
severity: S3
state: open
owner: -
reporter: ux-1
needs_human: yes
created: 2026-10-11T07:11:47Z
updated: 2026-10-11T07:11:47Z
---

## Description

Building the tokens from plan 04 exposed three places where the plan's colours miss its own
contrast rules (computed with WCAG 2.x ratios):

1. **Night-Ops telemetry text.** The plan allows #B30000-#FF4D4D. The brightest, #FF4D4D, is
   6.42:1 on black, so the 7:1 telemetry rule cannot hold. Exempted at 6.0 for now.
2. **Standard theme, state colours on graphite panels (#424242).** Warning #FF3B30 is 2.83:1 and
   standby #757575 is 2.18:1, below the 3:1 rule for meaningful graphics. Exempted; panels show
   states as filled chips with dark ink, which pass (>= 4.5:1).
3. **Daylight caution #B35F00** is 4.42:1 on the Daylight canvas, below 4.5:1 for text.
   The tokens use #A85900 instead: same hue, 4.91:1.

Plus two choices the plan leaves open: glass opacity 0.90 (at 0.85, telemetry over a white map
is 6.2:1), and two new roles, `accentText` #4DB8FF and `textMuted` #B0B0B0.

## Evidence

`cargo test -p design-tokens -- --nocapture` prints every pair. Details in `ui/tokens/README.md`.

## Proposed resolution

1. Choose: keep #FF4D4D (dark adaptation first) or allow #FF5E5E (7.0:1).
2. Choose: darker panels #333333 (warning 3.56:1), or keep #424242 and outline state marks with
   a carbon keyline.
3. Accept #A85900. Then plan 04 §1.2 is updated to match the tokens and the exemptions are
   removed or made permanent.

## Updates

- 2026-10-11 (ux-1): Product owner decided: keep Night-Ops #FF4D4D (exemption made permanent),
  Standard panels #333333 (warning red 3.56:1 on panels; glass back to 85 %, telemetry 7.49:1
  over white), accept Daylight caution #A85900. Plan 04 and the UI skill are updated. Remaining:
  standby #757575 on #333333 is 2.74:1 (exempted; option #808080 = 3.2:1).

