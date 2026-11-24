string amr_id
string current_zone

# Zones lane-reachable from BEHIND the robot: the space another AMR could drive
# into and rear-end this one. Derived from pose + heading + the nav graph, never
# from the planned path — the planner plans from where the robot IS, so a
# sequence-based rear zone is empty exactly when a stationary robot needs one.
#
# A LIST: at the centre of a 90-degree junction with the heading perpendicular
# to the route there is no single zone straight back, and the zones to either
# side are both behind. Empty when nothing is reachable behind (map edge).
string[] rear_zones
# Up to the next 3 upcoming zones (fewer if fewer remain).
string[] next_zone
# Remaining zones from current onward (passed zones are dropped).
string[] zone_sequence
# Fixed core RMF plan ETA (frozen once at plan), e.g. "1 min 44 s (104.4 s)".
# Patrol: ETA to destination. Delivery: pickup ETA first, then dropoff ETA.
string eta
# Live remaining ETA from fleet pose along planned path (+ stop-hold). Empty when unknown.
string eta_update
