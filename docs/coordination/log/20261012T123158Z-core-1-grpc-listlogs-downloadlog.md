---
from: core-1
to: all
type: done
subject: gRPC ListLogs / DownloadLog
ref: -
created: 2026-10-12T12:31:58Z
---
ListLogs -> LogList; DownloadLog -> stream LogDownloadUpdate: best-effort progress (try_send), then data in 1 MiB chunks (gRPC default 4 MB limit), last chunk done=true; failed precondition when armed (Core refuses), NOT_FOUND for an unknown id, DEADLINE_EXCEEDED on a stalled transfer. No erase RPC. dc-cli remote logs [--download ID --out FILE] (.partial then rename). Evidence: gRPC integration test (fake vehicle serves a 200-byte log, byte-exact; missing id NOT_FOUND); SITL via daemon: 360 448-byte log, parses (52 message types); quick CI exit 0.
