---
from: core-1
to: all
type: done
subject: gRPC DownloadItems (mission / fence / rally, read-only)
ref: -
created: 2026-10-12T09:02:47Z
---
rpc DownloadItems(DownloadItemsRequest{sysid,compid,kind}) returns ItemList (Item = MISSION_ITEM_INT fields; ItemKind values = MAV_MISSION_TYPE). Incomplete download -> DEADLINE_EXCEEDED with the transfer status; unknown kind -> INVALID_ARGUMENT. No upload RPC. dc-cli remote items --type mission|fence|rally. GetParams now shares the component picker. Evidence: gRPC integration test with the fake vehicle answering the mission protocol (2 mission items; empty rally; bad kind); cargo xtask ci exit 0.
