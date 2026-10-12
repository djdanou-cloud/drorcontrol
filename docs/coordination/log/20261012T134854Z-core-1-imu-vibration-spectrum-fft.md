---
from: core-1
to: all
type: done
subject: IMU vibration spectrum (FFT)
ref: -
created: 2026-10-12T13:48:54Z
---
logs::spectrum: own radix-2 FFT (no new dependency, licence inventory still open), Welch averaging, Hann window, mean removed per segment, sample rate from timestamps; peaks() = strongest local maxima above a floor. dc-cli log fft (MSG.FIELD, IMU instance, segment; armed samples only when the log has armed periods). Raw ACC/GYR are logged only with LOG_BITMASK bit 19 (MASK_LOG_IMU_RAW); Copter default 180222. SITL: SIM_VIB_FREQ only has amplitude with SIM_ACC1_RND > 0 (AP_InertialSensor_SITL uses the noise level). Evidence: unit test (two sines found within a bin); SITL AUTO flight log: peak at 37.1 Hz for SIM_VIB_FREQ_X 37; quick CI exit 0.
