---
from: core-1
to: all
type: done
subject: tlog info: telemetry radio quality
ref: -
created: 2026-10-12T20:08:41Z
---
tlog::RadioStats from RADIO_STATUS per sender: rssi/remrssi lowest and mean, fade margin (signal - noise), rxerrors/fixed growth with radio-reset handling, lowest txbuf; units as the MAVLink XML says (device dependent, ~2/dB on SiK; 255 unknown). Unit test only: SITL has no radio. Bench-test step 7 will exercise it.
