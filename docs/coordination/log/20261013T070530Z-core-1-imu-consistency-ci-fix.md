---
from: core-1
to: all
type: done
subject: IMU consistency CI fix
ref: -
created: 2026-10-13T07:05:30Z
---
CI run 38357646738 (sitl 4.7.1) failed: IMU 1 vs 0 read 0.54 m/s2 (Caution) from a 1 s window at disarm holding a few samples of SIM_ACC1_RND noise. Now 5 s means and windows under half the typical count are left out; 4 local AUTO runs (3x4.7.1, 1x4.6.3) read <= 0.01 m/s2.
