---
from: core-1
to: all
type: done
subject: SR-05: command outcome alerts
ref: SR-05
created: 2026-10-11T10:13:55Z
---
CommandStatus::alert classifies outcomes: unconfirmed flight command (expired or refused; mode change after its 3 s lifetime) = Warning, unconfirmed ground command (motor test, reboot, param reset, force-arm) = Caution, verified mode change = Advisory, none for cancelled/superseded. dc-core raises CoreEvent::Alert; gRPC AlertEvent (proto field 7); dc-cli prints it. Tests: policy mapping; core test with a deaf fake vehicle (Warning at 3.0-3.4 s, vehicle mode unchanged). UI agents: show Warning with audio (UI skill §2). Committed locally; not pushed (owner instruction).
