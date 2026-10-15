# Future upgrades: 5K and ProMotion displays

**Owner decision, 2026-10-12:** no Mac with a 5K display or a ProMotion (120 Hz) display is
available. Tests and targets that need one are taken out of the current plan and kept here.
They come back when that hardware is available. Until then, Mac display tests run on what we
have: an M1 MacBook Air with its built-in Retina screen (2880×1800, 60 Hz).

Nothing here is a current Phase 0 or Phase 1 task, and nothing here blocks one.

## 1. 5K display (for example, Apple Studio Display, 5120×2880)

| Item | Was in | What to do when a 5K display is available | Pass |
|------|--------|-------------------------------------------|------|
| Map at 5K full screen (spike S0-3, T0-07) | plan 01 §5.1 go/no-go gate and ADR-0003 criterion "60 fps at 5K"; Mac runbook T0-07; `ui/spikes/s0-3-map/RESULTS.md` results table | `ui/spikes/s0-3-map/build/spike-s0-3-map --fullscreen --report s03-m1-5k.json` on the 5K screen, with `sudo powermetrics --samplers gpu_power -i 1000` alongside (read "GPU HW active residency") | API `Metal`; warm pass ≥ 59 fps with < 1 % missed vsync; GPU ≤ 8 % on M1 |
| External 5K in the macOS test matrix | plan 03 §8 "Displays" | Add 5K back to the display row; run the flight view and map on it | Same frame-rate targets as Retina |

Note: the 2880×1800 run already shows about 42 % GPU against the 8 % target, so the 5K run is
not expected to change ADR-0003's outcome; it would confirm frame rate at the larger size.

## 2. ProMotion (120 Hz) display

| Item | Was in | What to do when a ProMotion Mac is available | Pass |
|------|--------|----------------------------------------------|------|
| UI bridge frame timing at 120 Hz (spike S0-1, T0-06) | Mac runbook T0-06 step 4; `ui/spikes/s0-1-bridge/RESULTS.md` "Not yet measured"; issue `20261010T223250Z-…` | `for r in 0 200000 400000; do S01_RATE=$r S01_REPORT=s01-120hz-$r.json ui/spikes/s0-1-bridge/target/release/spike-s0-1-bridge; done` on the 120 Hz screen, plugged in | 0 dropped frames; frame-time bound to be set with ADR-0001's restated criterion (one 120 Hz frame is 8.3 ms) |
| 120 fps PFD and map when plugged in | plan README §4 "PFD + map frame rate" target | Measure the flight view at 120 Hz, plugged in and on battery | 120 fps plugged in; adaptive 30 fps on battery / Low Power Mode still holds |
| ProMotion in the macOS test matrix | plan 03 §8 "Displays" | Add ProMotion back to the display row | As above |

## 3. Map GPU load (accepted at ≈ 42 % on the M1, ADR-0013)

The map spike passed everything but GPU load (≈ 42 % mean, 56 % max, against ≤ 8 %) while
flying a route continuously at 60 fps full screen. The owner accepted it for now (2026-10-15);
these are the suggested ways to bring it down, each to be measured with
`sudo powermetrics --samplers gpu_power -i 1000` ("GPU HW active residency") before and after:

| Suggestion | Why it should help |
|---|---|
| Redraw only when something changed (camera, tiles, vehicle position, overlay) instead of every vsync | Most of the time a ground-station map is still; an idle map should cost almost nothing |
| Cap the map's own redraw rate (e.g. 30 fps) while the vehicle is the only thing moving, 60 fps during gestures | Halves the work in the common case without making panning feel slow |
| Draw the vehicle, track and mission as a light overlay layer, not by re-rendering the style | The base map then redraws only when the camera moves |
| Fewer style layers, labels and symbol collisions at the zoom levels used in flight | Symbol placement is a large part of MapLibre's per-frame cost |
| Turn off or lower multisampling (MSAA) on Retina | At 2× pixel density the edge smoothing adds little and costs fill rate |
| Lower the render scale on battery or Low Power Mode | Fewer pixels; plan README already asks for an adaptive rate on battery |
| Keep decoded tiles cached on the GPU and prefetch along the route | Avoids re-uploading textures as the view follows the vehicle |

Pass: ≤ 8 % GPU on the M1 for the S0-3 route, or a new limit the owner sets.

## 4. Bringing an item back

1. Run the steps above on the new hardware and record the results where the item "Was in".
2. Restore the target or criterion in the plan or ADR (an ADR change needs the architect).
3. Remove the row from this file and log it in `docs/coordination/log/`.
