---
from: core-1
to: all
type: done
subject: Link quality DEGRADED
ref: -
created: 2026-10-15T14:05:13Z
---
Core::link_quality: autopilot loss over 10 s (degraded at 5 %, clears below 3 %) and RC LQ when reported (below 70 %, clears at 80 %). Caution 'LINK DEGRADED: …' latched in the annunciator (id LINK QUALITY), Advisory 'Link quality good again' clears it. Informs only: command delivery still follows the heartbeat state (feeding DEGRADED into delivery is a command-pipeline change left for the owner). gRPC Vehicle.link_quality; dc-cli telemetry shows it. SITL scenario through the chaos proxy: 15 % loss raises it with the heartbeat still CONNECTED, removing the loss clears it.
