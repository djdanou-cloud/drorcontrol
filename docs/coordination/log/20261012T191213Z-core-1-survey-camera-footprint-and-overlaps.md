---
from: core-1
to: all
type: done
subject: Survey camera footprint and overlaps
ref: -
created: 2026-10-12T19:12:13Z
---
protocols::survey::{Camera, footprint, line_length_m}: GSD, footprint, spacing from side overlap, trigger distance from front overlap (pinhole, nadir, flat ground). dc-cli mission survey --camera/--sidelap/--frontlap; --spacing optional with --camera. Camera trigger is printed, not put in the mission (pending owner decision). Unit test (1-inch 20 MP at 100 m: 2.74 cm/px).
