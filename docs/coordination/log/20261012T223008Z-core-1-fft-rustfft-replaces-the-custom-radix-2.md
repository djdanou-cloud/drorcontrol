---
from: core-1
to: all
type: done
subject: FFT: rustfft replaces the custom radix-2
ref: -
created: 2026-10-12T22:30:08Z
---
logs::spectrum uses rustfft 6.4.1 (MIT/Apache-2.0; deps num-complex, num-traits, num-integer, primal-check, strength_reduce, transpose, autocfg: all MIT/Apache-2.0), per ADR-0012 item 10. Segments of any length >= 8. Unit tests incl. a 1000-sample segment; SITL AUTO 37 Hz peak still found on 4.7.1 and 4.6.3.
