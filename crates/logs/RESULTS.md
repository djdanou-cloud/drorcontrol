# Spike S0-4 results — 4 GB dataflash index + plot

**Targets (plan 01 §9, 05 §1):** index a 4 GB `.bin` in ≤ 20 s; first plot ≤ 1 s; RSS ≤ 1.2 GB.

## Data
* Real log: ArduCopter **4.7.1 SITL** (built from source), `LOG_DISARMED=1`, `LOG_BITMASK=1048575`,
  95 s → 32 MB, 791 063 complete records, 61 message types (191 FMT definitions).
* 4 GB test file: the clean 32 MB log concatenated 128× → **3 965 MiB, 101 256 064 records**
  (repeated identical FMTs, timestamps restart per copy — irrelevant for indexing cost).
* Correctness fixture (committed): first 900 KB of the real log, 21 277 records / 50 types, with
  reference values from pymavlink `DFReader` (`scripts/dataflash_vectors.py`).

## Method
FMT pre-scan (`A3 95 80` + full validation, so mid-log FMTs are found) → parallel walk of 64 MiB
blocks with chain-of-4 resynchronisation → per-type `u32` block-relative offsets. Columns are
materialised on demand in parallel; MinMaxLTTB (ratio 4) to 2 000 points. File pages are
released from the process (`MADV_DONTNEED`) after each block, so RSS is dominated by the
offset index and the requested column.

## Results (Linux x86_64, 4 vCPU shared, NVMe-backed VM disk, release build)

| Step | Cold page cache | Warm |
|------|----------------:|-----:|
| Index 3 965 MiB / 101 M records | **2.9 s** (5.8 s in one earlier cold run) | 1.0–1.25 s |
| Plot `ATT.Roll` (118 k samples) | 49 ms | — |
| Plot `IMU.AccX`, instance 0 (4.7 M samples) | 579 ms | — |
| Plot `ACC.AccX` (21.3 M samples, largest column) | **930 ms** | 486 ms |
| Peak RSS after index | 598 MiB (offsets ≈ 405 MiB) | 583 MiB |
| Peak RSS with the 21 M-sample column loaded | **999 MiB** | 940 MiB |

All three targets met on this machine. **Margins:** the largest-column cold plot is close to
the 1 s limit (930 ms), and peak memory with that column is 999 MiB of the 1.2 GB budget.
Without page release, peak RSS was 4 GB (every file page stayed resident).

## Correctness (CI: `cargo test -p logs`)
* Against pymavlink on the real fixture: identical message counts (50 types); identical
  count/min/max/first/last for every numeric column; every numeric field of 252 sampled
  records identical. Only difference found: pymavlink-side JSON cannot carry NaN (one parameter
  `Default` is NaN) — both sides exclude non-finite values.
* Corruption: 64 smashed bytes + a tail cut mid-record lose only the damaged records.
* A real truncated final record (SITL killed mid-write) is skipped, not mis-read.

## Not done here (follow-ups for Phase 4, plan 05 §1)
* Sidecar index file (instant re-open), min/max pyramid for O(pixels) zoom, streaming mode for
  logs still downloading, FMTU/UNIT/MULT units, automatic instance split.
* Memory: offsets are 4 B/record; delta/varint encoding would cut the ~405 MiB index ~2×,
  and `f32` columns would halve column memory. Needed if 8 GB logs are in scope.
* Re-measure on Apple Silicon (plan's reference machine).
