---
from: core-1
to: all
type: done
subject: ADR-0013 item 6: force-arm and reset settings
ref: -
created: 2026-10-15T11:32:44Z
---
Two opt-in settings, off by default: simple-force-arm (class C plus a required, logged reason) and simple-param-reset (card only, no slide). protocols::command::Simplified in the policy; a token keeps the stricter of its issue-time and current class; reset still refused while armed (SR-08); setting changes and every use audited. Tests in protocols, dc-core and api; hazard log H1/H5 and section 6, ADR-0011/0013, HANDOVER, test kit, UI skill updated.
