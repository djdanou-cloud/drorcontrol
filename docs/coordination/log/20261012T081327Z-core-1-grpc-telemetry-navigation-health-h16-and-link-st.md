---
from: core-1
to: all
type: done
subject: gRPC telemetry: navigation health (H16) and link state
ref: -
created: 2026-10-12T08:13:27Z
---
Vehicle.nav (PositionTrust + reasons, from Core::nav_health) and Vehicle.link (VehicleLinkEvent.State) set by ListVehicles and StreamTelemetry via api::vehicle_with_health; vehicle_to_pb stays a pure state conversion. Evidence: grpc integration test (fake vehicle: CONNECTED, UNKNOWN with reasons); SITL via daemon: Untrusted ['EKF not initialised','no absolute position'] then Good after convergence; xtask ci exit 0.
