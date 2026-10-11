# Design tokens

[`tokens.json`](tokens.json) is the single source for every visual constant: colours for the
four themes, typography, spacing, strokes, motion and touch targets
([plan 04 §1–2](../../docs/plan/04-ux-design-system.md)). Components never use literal values.

| Output | Path | Use |
|---|---|---|
| QML singleton | `generated/qml/DrorControl/Tokens/` | Add `generated/qml` to the import path, `import DrorControl.Tokens`, then `Tokens.canvas`, `Tokens.stateWarning`, `Tokens.sizeTelemetryL`. Switch at runtime: `Tokens.theme = "daylight"`. |
| CSS variables | `generated/tokens.css` | Docs site and web viewer: `var(--dc-color-canvas)`, theme via `data-theme="night-ops"` on any ancestor. Standard is the default. |
| Rust | `crates/design-tokens` | GPU-drawn instruments (`Tokens::color("standard", "pfd.skyTop")`) and the contrast checks. |

Edit `tokens.json`, run `python3 scripts/gen_tokens.py`, and commit both. `cargo xtask ci` fails
if the generated files are stale. The `qml-tokens` CI job lints the singleton and runs
[`tests/tst_tokens.qml`](tests/tst_tokens.qml) on Qt 6.

## Themes

`standard` (default, brand look), `daylight` (matte, +2 pt text, +25 % strokes, 56 px targets),
`nightOps` (red monochrome for dark adaptation and NVG), `highContrast` (black/white, follows
macOS Increase Contrast). Every theme defines the same tokens; a test enforces it.

## Colour roles

* `text`, `textTelemetry`: primary text and live readouts, at least 7:1 on every background (Night-Ops: 6.2:1, by decision),
  including glass over a white map tile.
* `textMuted`: units and secondary labels. `accentText`: azure text on panels, where brand azure
  is too dark (4.33:1 on #333333).
* `state.*`: reserved safety colours (warning, caution, advisory, normal, guidance, standby).
  They are used for icons, frames, chips and short labels on the canvas, never for decoration.
  On panels, a state is shown as a filled chip with `stateInk.*` text.
* Telemetry digits stay in `textTelemetry`. The state is carried by a chip, frame or icon next
  to the value, plus a label or shape change (never colour alone).

## Contrast checks

`crates/design-tokens` evaluates every rule in `tokens.json` → `contrast.rules` for every theme
(WCAG 2.x ratios). Telemetry text must reach 7:1, other text 4.5:1, and meaningful graphics 3:1.
`cargo test -p design-tokens -- --nocapture` prints the full table. An exemption must name a
reason and an issue, and a test fails as soon as an exemption is no longer needed.

### Decisions (product owner, 2026-10-11)

* **Night-Ops** keeps #FF4D4D: dark adaptation comes first. Telemetry is 6.42:1 on black and
  6.24:1 on Night-Ops panels; the exemption records this.
* **Standard panels are #333333**, darker than brand graphite #424242. Warning red on panels goes
  from 2.83:1 to 3.56:1, and glass is back to the plan's 85 % (telemetry over a white map tile:
  7.49:1).
* **Daylight caution is #A85900** (4.91:1). Plan 04 is updated to match.

Still open: standby grey #757575 on the #333333 panels is 2.74:1, below 3:1. It is exempted
because standby always comes with a text label. The alternative is #808080 (3.2:1). Also
`accentText` #4DB8FF and `textMuted` #B0B0B0 are roles the plan does not name (see
`deviations`).

Not yet covered: PFD symbology over the sky/ground gradient. It relies on the `pfd.halo` outline
and needs a rendered-image check. Fonts are referenced by name only; bundling waits on the
licence check (plan README §5).
