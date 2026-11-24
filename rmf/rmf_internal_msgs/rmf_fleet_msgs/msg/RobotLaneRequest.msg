# Per-robot lane closure request.
#
# Unlike LaneRequest (which closes lanes for the WHOLE fleet), this closes or
# opens lanes for a single robot only. The targeted robot is detached onto its
# own planner whose closures are (fleet-wide closures) U (close_lanes); every
# other robot in the fleet is unaffected.
#
# Typical use: a zone conflict where one robot should route around the contested
# lanes while the other holds position.
#
# The fleet adapter triggers a replan for the targeted robot after applying the
# request.
#
# WARNING: closures set this way do NOT expire. The publisher is responsible for
# sending open_lanes (or a request with empty close_lanes) once the reason has
# passed, otherwise the robot permanently plans on a reduced graph. While
# closures are active the robot is excluded from task bidding.

string fleet_name
string robot_name
# Lanes to close for this robot only.
uint64[] close_lanes
# Lanes to reopen. Reopening any lane clears ALL private closures for the robot
# and re-attaches it to the shared fleet planner.
uint64[] open_lanes
# When true the fleet adapter applies the request WITHOUT replanning the robot.
# Used to reopen lanes after a plan was made on the reduced graph, so that plan
# is kept instead of being immediately replaced by one on the full graph.
# Default false = previous behaviour (replan after every request).
bool skip_replan
