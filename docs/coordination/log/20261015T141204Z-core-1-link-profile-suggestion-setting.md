---
from: core-1
to: all
type: done
subject: Link-profile suggestion setting
ref: -
created: 2026-10-15T14:12:04Z
---
New opt-in setting suggest-link-profile (off by default, in the settings file, dc-cli settings, gRPC list). On, a link that turns DEGRADED by packet loss, with the vehicle heard on that link only, gets the next leaner profile suggested (CoreEvent::LinkProfileSuggested, gRPC event); the operator accepts (switches and re-requests) or dismisses (not suggested again for that link) via Core or gRPC AnswerLinkProfileSuggestion; everything audited, profile changes too. Plan corrected: the ELRS 'Link rate' STATUSTEXT means ELRS for RC, not telemetry, so it is not a trigger. Tests: dc-core profile_suggestion (off, accept, dismiss).
