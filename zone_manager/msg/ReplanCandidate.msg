# Published on opposite-direction conflicts (straight or inverted-T).
# Identifies which AMR should be considered for replanning after both are paused.

builtin_interfaces/Time stamp
string zone_id
# Robot selected for replan (lowest priority if both healthy; healthy if not).
string robot_id
# All robots involved in the opposite-direction conflict.
string[] conflicting_robots
# Human-readable selection reason.
string reason
