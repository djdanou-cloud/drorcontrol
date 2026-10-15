---
from: core-1
to: all
type: done
subject: Link profiles and per-link stream rates
ref: -
created: 2026-10-15T12:35:48Z
---
LinkProfile full/high/constrained-elrs via ?profile= (LinkTarget, CLI, gRPC OpenLink); per-link StreamRequest (ACK, retry, read-back via GET_MESSAGE_INTERVAL) sent with HubHandle::send_on, repeated after LOST silence, reported as CoreEvent::LinkStreams; terrain answers paced per profile, in-flight repeats ignored. SITL SIM_ELRS scenario: constrained 302 B/s on 500 B/s, all rates read back, mode change verified, Full saturates (4.7.1 and 4.6.3); full SITL suite 22/22 on 4.7.1.
