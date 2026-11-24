builtin_interfaces/Time t
float64 x
float64 y
float64 yaw

bool obey_approach_speed_limit false
# Speed limit of the lane leading to this waypoint in m/s
float32 approach_speed_limit

string level_name
uint64 index

# traffic_manager / full_control: true = ECBS-approved segment; execution sends only
# these waypoints in the VDA5050 order. false = reserved for later release.
bool release false

# ECBS WAIT: hold at this waypoint (last released node) until traffic_manager extends release.
bool traffic_hold false
float64 traffic_wait_sec 0.0
string traffic_blocker_amr

