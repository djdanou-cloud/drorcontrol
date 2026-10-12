---
from: core-1
to: all
type: progress
subject: ADS-B traffic relative to the vehicle
ref: -
created: 2026-10-12T13:53:49Z
---
Traffic::relative_to (haversine distance, initial bearing, height vs vehicle MSL); gRPC TrafficState distance_m/bearing_deg/above_m (15-17). Unit test; SITL adsb scenario checks distances < 12 km and plausible heights. Quick CI green.
