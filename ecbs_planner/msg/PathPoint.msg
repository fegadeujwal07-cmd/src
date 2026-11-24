# One timed waypoint on /to_zone_manager.
builtin_interfaces/Time t     # Planned arrival time
float64 x                     # Map x (m)
float64 y                     # Map y (m)
float64 wait_sec              # ECBS dwell / hold at this waypoint
WaitDependency[] waiting_for  # Who/what must clear before leaving
