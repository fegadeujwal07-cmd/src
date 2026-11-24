# Unified mutex group request status message
# Published for each robot that is requesting/approaching a mutex

# Robot name
string robot_name

# Mutex group name
string mutex_name

# Distance to mutex entry point (meters)
float64 dist_to_entry

# Timestamp
builtin_interfaces/Time timestamp

# State: REQUESTING, QUEUED, OWNER, NORMAL
string state

# Additional info: queue position (if queued), or 0 if not queued
uint32 queue_position
