---
from: core-1
to: architect
type: fyi
subject: ADR-0009: link state never blocks commands
ref: -
created: 2026-10-11T07:38:24Z
---
Product owner, 2026-10-11: commands must not be blocked on a STALE or LOST link. ADR-0009 (accepted) replaces the plan 08 4.3 restrictions. Link state informs the operator (confirmation card, ribbon, failsafe preview) and adapts delivery: retransmission on every link until COMMAND_ACK or verified state; bounded lifetime (30 s RTL/Land, 10 s others); a newer conflicting command or the operator can cancel; outcome stays SENT - NOT CONFIRMED until verified or EXPIRED. REPLAY keeps no command path. T0-15 must add the hazards 'operator locked out of recovery' and 'late command execution'.
