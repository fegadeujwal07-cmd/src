# Conflict edge event for external traffic / VDA5050 nodes.
# Published only on Detected / Resolved transitions (not every timer tick).
#
# Handshake for another node:
#   type=TYPE_DETECTED  + action=ACTION_START_PAUSE + robot_ids → pause those AMRs
#   type=TYPE_RESOLVED  + action=ACTION_STOP_PAUSE  + robot_ids → resume those AMRs
# robot_ids on Resolved are robots whose pause refcount hit zero (safe to resume).

uint8 TYPE_DETECTED=0
uint8 TYPE_RESOLVED=1

string ACTION_START_PAUSE=startPause
string ACTION_STOP_PAUSE=stopPause

builtin_interfaces/Time stamp
uint8 type
string zone_id
# AMRs that should receive `action` (VDA5050 instantActions consumer).
string[] robot_ids
# "startPause" or "stopPause"
string action
