---
from: core-1
to: all
type: done
subject: Tlog sanitize
ref: -
created: 2026-10-13T04:08:08Z
---
dc-cli tlog sanitize moves every position (found from the new generated mavlink_codec::message_fields layout) by one offset, recomputes CRCs, reports unsigned frames. Verified on a SITL recording.
