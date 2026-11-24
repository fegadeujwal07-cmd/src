# full_control → traffic_manager: full leg path awaiting release decision.

builtin_interfaces/Time stamp

string fleet_name
string robot_name

# Correlates intent with PathRelease (monotonic per fleet adapter).
uint64 request_id

# Mirrors rmf_fleet_msgs/PathRequest leg identity.
string task_id
string dispatch_task_id
uint32 phase_index
uint32 path_revision
uint32 cursor_index

rmf_fleet_msgs/Location[] path
