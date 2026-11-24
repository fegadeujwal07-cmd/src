string name
string model

# task_id is copied in from the most recent Request message,
# such as ModeRequest, DestinationRequest, or PathRequest
string task_id

# The sequence number of this message. Every new message should increment the
# sequence number by 1.
uint64 seq

RobotMode mode
float32 battery_percent
Location location
Location[] path

# Payload state of the robot.
# 0 = unknown (no state yet from MQTT/VDA5050)
# 2 = unloaded (loads array empty)
# 4 = loaded   (loads array non-empty)
# Used by the fleet adapter to gate delivery task assignment.
int16 load_status 0

# VDA5050 operating mode from MQTT state topic operatingMode field.
# -1 = unknown (not yet received from MQTT/VDA5050)
#  0 = MANUAL
#  1 = AUTOMATIC
int16 operating_mode -1

# Current linear speed of the robot in m/s (may be negative when reversing).
float32 speed

# Charging signal from the MQTT/VDA5050 state topic powerSupply.charging field.
# -1 = unknown (field missing, not true/false, or no fresh state from robot)
#  0 = not charging
#  1 = charging
# Used by the fleet adapter to finish a charge task (charging true -> false).
int16 charging_status -1
