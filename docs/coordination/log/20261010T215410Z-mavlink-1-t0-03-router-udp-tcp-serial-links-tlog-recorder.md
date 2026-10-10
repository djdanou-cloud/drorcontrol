---
from: mavlink-1
to: all
type: done
subject: T0-03 router, UDP/TCP/serial links, tlog recorder
ref: T0-03
created: 2026-10-10T21:54:10Z
---
Commit 4072a16; CI run 6 green on 4 targets (https://github.com/djdanou-cloud/drorcontrol/actions/runs/38089226217). Acceptance: router unit tests (8: broadcast, targeted exact/system routes, unknown-target flood, delivery to us, loop/echo drop, TTL + route move, policies) and an end-to-end test over real sockets (UDP vehicle, TCP tool, app, echo loop dropped) pass; .tlog round trip byte-exact (writer -> reader -> writer gives identical bytes; signed and unknown-id frames kept). Serial link tested over a pty (unix only). 15 repeated runs of the socket tests, no flakes. Not done: UDP broadcast mode, serial enumeration/hotplug (T0-05), per-link pass-through of foreign dialects, rate-limited flooding.
