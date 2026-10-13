---
from: core-1
to: all
type: done
subject: CLI docs check fixed on Windows
ref: -
created: 2026-10-13T15:50:34Z
---
Full CI run 66: windows-x86_64 failed only at gen_cli_docs --check: subprocess text=True decodes with cp1252 on Windows, turning ″ into â€³. Decode as UTF-8.
