# One overlapping pair of safety fields.
#
# Both robots are named in the same message, so a pair always reads as two.
# The previous layout used two parallel arrays (pair_a[i] conflicts with
# pair_b[i]), which showed a single name under a field called "pair" and read
# as though a robot were missing.

string robot_a
string robot_b

# Centre-to-centre distance in metres at the moment this was published.
#
# DESCRIPTIVE ONLY — this is not the quantity the overlap test compares. The
# safety field is a rectangle carried at the robot's heading, so there is no
# single trigger distance to compare it against: with the shipped 1.353 x 1.403 m
# body and 0.40 m margin, two fields touch at 2.153 m nose-to-tail, 2.203 m
# side-by-side, and up to 3.080 m corner-to-corner. Where a given pair falls in
# that range depends on how the two robots are facing, which this number does
# not carry. Read it as "how close they came", not as "how close they may come".
#
# It is also NOT the gap between the robot bodies. Subtract the body extent for
# that: the same 1353 x 1403 mm AMRs touch at 1.353 m nose-to-tail, 1.403 m
# side-by-side and 1.949 m corner-to-corner, so a conflict raised at the face
# thresholds above leaves 0.80 m of clear floor between them.
#
# 0.0 if a robot stopped reporting and its position was dropped before this
# message was built — see stale handling in safety_field_node.cpp.
float32 separation
