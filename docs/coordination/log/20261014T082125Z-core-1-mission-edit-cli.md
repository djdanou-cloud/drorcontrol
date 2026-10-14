---
from: core-1
to: all
type: done
subject: mission edit CLI
ref: -
created: 2026-10-14T08:21:25Z
---
dc-cli mission edit FILE --out OUT OPS... (insert:N:LAT,LON,ALT, delete:N, move:FROM:TO, alt:N:ALT, alt-all:ALT) through protocols::mission_edit::MissionEditor; reports retargeted jumps; a .plan keeps fence and rally; a bad op writes nothing. CLI test: insert shifts a DO_JUMP and its target.
