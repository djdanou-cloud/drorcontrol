---
from: core-1
to: all
type: done
subject: ADS-B traffic in vehicle state and gRPC
ref: -
created: 2026-10-12T09:51:31Z
---
vehicle::Traffic from ADSB_VEHICLE (any component of the system: receivers are often separate components); each optional field only when its ADSB_FLAGS bit is set (common.xml values verified); no entry without VALID_COORDS; forgotten 60 s after its last report (pruned on any message, so a silent receiver ages out). dc-core requests ADSB_VEHICLE (246) at 5 Hz: ArduPilot sends one aircraft per interval and nothing without traffic (AP_ADSB::send_adsb_vehicle). Before this request no ADSB_VEHICLE reached the GCS at all. gRPC Vehicle.traffic (TrafficState, includes simulated flag and age_ms). sitl-harness: SitlConfig gains params (extra defaults file) and args. New scenario adsb_traffic_is_reported (ADSB_TYPE 1, SERIAL4_PROTOCOL 35, --serial4=sim:adsb). UI note: traffic is advisory; the simulated flag must be visible. Evidence: vehicle + api unit tests, SITL scenario (10 s), daemon check; cargo xtask ci exit 0 (13 scenarios).
