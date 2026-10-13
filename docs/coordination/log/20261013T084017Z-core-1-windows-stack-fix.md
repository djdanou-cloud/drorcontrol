---
from: core-1
to: all
type: done
subject: Windows stack fix
ref: -
created: 2026-10-13T08:40:17Z
---
Full CI run 38367130086: windows-x86_64 failed, all 6 tlog_fixture tests (first tests to run the dc-cli binary on Windows). Reproduced on Linux with ulimit -s 1024: debug dc-cli overflows its stack in argument parsing; release did not. main now runs the program on a 16 MB thread; fixture tests report stderr on failure.
