---
from: core-1
to: all
type: done
subject: CI publishes dc-cli builds
ref: -
created: 2026-10-12T18:24:46Z
---
build-test matrix builds dc-cli --release and uploads dc-cli-<platform> (14 days) for hardware bench tests; unsigned, not a release. actionlint 1.7.12 clean. Runs on GitHub only after a push.
