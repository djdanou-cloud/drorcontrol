---
from: core-1
to: all
type: done
subject: Annunciator fed from core alerts
ref: -
created: 2026-10-14T07:24:56Z
---
annunciator::classify maps alert text to fixed ids (FENCE, POSITION as conditions; command events latched until ack). Core::alert routes every alert; Core::annunciator/master_alert/ack_alert; CoreEvent::Sound when the annunciator says to sound (gRPC skips it). Tests: classify unit test; fence replay test checks latch, master, ack and one sound.
