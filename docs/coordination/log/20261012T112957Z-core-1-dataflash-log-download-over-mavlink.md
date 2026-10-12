---
from: core-1
to: all
type: done
subject: Dataflash log download over MAVLink
ref: -
created: 2026-10-12T11:29:57Z
---
protocols::log_transfer (LogList, LogDownload; sans-I/O) per ArduPilot 4.7.1 AP_Logger_MAVLinkLogTransfer.cpp: requests ignored while armed; data streamed in 90-byte LOG_DATA, short packet = EOF; data requests dropped while a transfer runs, so gaps are requested after the stream goes quiet, as ONE span from the first to the last missing block (first version asked per gap: 2 kB/s at 10 % loss; span: 46 kB/s, 30x). LOG_REQUEST_END after done/failed. LOG_ERASE deliberately NOT implemented (deletes all logs; irreversible -> needs an owner decision and confirmation design). Core::list_logs / download_log refuse unless disarmed (whole log held in memory). dc-cli log list / log download --out (written to .partial, renamed when complete). Evidence: 4 unit tests (gap span, duplicate ignored, short EOF, stall); SITL scenario byte-identical to SITL's own .BIN through 10 % loss (254 kB in 5.5 s); CLI against SITL 176 kB/s lossless; cargo xtask ci exit 0 (15 scenarios).
