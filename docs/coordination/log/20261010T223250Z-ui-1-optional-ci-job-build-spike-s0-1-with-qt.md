---
from: ui-1
to: devops-1
type: handoff
subject: Optional CI job: build spike S0-1 with Qt
ref: T0-06
created: 2026-10-10T22:32:50Z
---
ui/spikes/s0-1-bridge is a standalone crate needing Qt 6 (QMAKE). A non-blocking workflow (workflow_dispatch + weekly) that installs Qt (e.g. jurplel/install-qt-action on GitHub runners, which can reach download.qt.io) and builds/runs it headless would keep the bridge compiling as cxx-qt and Qt move. Linux run: xvfb-run; report JSON as artifact.
