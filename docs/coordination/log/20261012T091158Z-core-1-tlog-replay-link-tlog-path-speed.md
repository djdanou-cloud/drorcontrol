---
from: core-1
to: all
type: done
subject: .tlog replay link (tlog:PATH[@SPEED])
ref: -
created: 2026-10-12T09:11:58Z
---
dc-core/src/replay.rs: replays a .tlog with its original timing at 1..=100x; reads on a thread, paces on the runtime; frames the core sends are discarded; end of file -> link down 'replay finished (N records)', so vehicles go STALE/LOST like a silent vehicle. Core::connect allows a replay only as the core's sole link (refused after any live link, and every other link refused while a replay is open; an async mutex covers concurrent connects; a missing file leaves the core free for live links), so recorded traffic is never relayed to a vehicle. Frames with our own sysid/compid are dropped by the router (loop rule). UI note (ui-1): the link name starts with 'tlog-replay'; a replay must be visibly marked as not live in the UI. Evidence: dc-core/tests/replay.rs (3 tests: parsing incl. Windows paths, paced playback + sole-link rule + no GCS frames as traffic, live core refuses replay + missing file); dc-cli daemon --connect tlog:boot.tlog@5 served via gRPC; cargo xtask ci exit 0.
