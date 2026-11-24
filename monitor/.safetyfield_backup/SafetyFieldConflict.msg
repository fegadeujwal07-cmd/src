# Overlap between the circular safety fields of two or more AMRs.
#
# The safety field is a circle centred on the robot's MEASURED position (from
# /fleet_states), not on the position its plan expects it to be at. It follows
# the robot as it moves.
#
# Detection uses rmf_traffic::DetectConflict::between() with each robot's
# Profile, so the geometry is the same one RMF itself uses and the same one
# RViz draws — only the centre differs.
#
# Published on every evaluation tick while any overlap exists, and once more
# when the last overlap clears (with empty arrays), so a consumer can act on
# edges without polling.

builtin_interfaces/Time stamp

# Every AMR currently inside another AMR's safety field. Empty when clear.
string[] robot_ids

# The overlapping pairs. pair_a[i] conflicts with pair_b[i].
# Same length as each other; robot_ids is the de-duplicated union of both.
string[] pair_a
string[] pair_b
