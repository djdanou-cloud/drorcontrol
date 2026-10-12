---
from: core-1
to: all
type: done
subject: Firmware and board details (AUTOPILOT_VERSION)
ref: -
created: 2026-10-12T12:20:03Z
---
Requested with MAV_CMD_REQUEST_MESSAGE(148) on the first autopilot heartbeat, retried every 3 s up to 5 times until it arrives (same pattern as the banner request; read-only, sent directly, not through CommandPolicy, like DO_SEND_BANNER and SET_MESSAGE_INTERVAL). vehicle::AutopilotVersion per GCS_MAVLINK::send_autopilot_version 4.7.1: version major.minor.patch + FIRMWARE_VERSION_TYPE, 8-char git hash, board id = board_version >> 16 (APJ_BOARD_ID), vendor/product, uid, capabilities. gRPC Vehicle.autopilot_version. Evidence: unit test; SITL connect scenario asserts 4.7.1 and an 8-char hash; quick CI exit 0.
