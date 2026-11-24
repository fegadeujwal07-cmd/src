# Overlap between the rectangular safety fields of two or more AMRs.
#
# The safety field is a rectangle centred on the robot's MEASURED position
# (from /fleet_states) and turned to its MEASURED heading, so it is carried by
# the robot rather than lying on the map underneath it. It is the robot body
# inflated by a fixed margin on every side; trigger and release use the same
# rectangle.
#
# zone_manager subscribes to this topic and owns the pause. This message is an
# observation, not a command.
#
# DELTA, NOT A ROSTER. robot_ids on a DETECTED message names only the robots
# that just ENTERED conflict, and on a RESOLVED message only those that just
# LEFT it. Exactly one DETECTED per robot per hold and exactly one matching
# RESOLVED — never a repeat, never a periodic re-announcement.
#
# That contract is load-bearing: zone_manager's note_conflict_detected()
# increments its per-robot pause refcount unconditionally, so naming a robot in
# two DETECTED messages without a RESOLVED between them leaves the count at 1
# after the release and the robot paused for good. A consumer that wants the
# current roster must accumulate the deltas.
#
# The predecessor of this contract fired only on the none->any and any->none
# edges. With two AMRs the two behave identically; with three it would go silent
# when one pair cleared while another still overlapped, stranding any robot whose
# pair had cleared earlier.
#
# The topic is latched (transient local, depth 1), so a subscriber that starts
# late is handed the most recent transition rather than nothing. Note the limit
# that follows from the delta semantics: one latched sample is not a snapshot of
# who is currently held, so a consumer restarting mid-conflict recovers the last
# transition only and picks the rest up at the next one.

uint8 RESOLVED = 0
uint8 DETECTED = 1

builtin_interfaces/Time stamp

# RESOLVED or DETECTED, from the constants above.
uint8 event

# The robots entering conflict (DETECTED) or leaving it (RESOLVED), sorted and
# de-duplicated. Never empty: no message is published when nothing changed.
string[] robot_ids

# On DETECTED, every pair currently overlapping — not only those involving the
# robots named above, so that with three or more AMRs a consumer can tell
# "amr3 joined the amr1/amr2 conflict" from "amr3 started its own".
#
# On RESOLVED, the pairs that just ended, with separations measured at the
# distance the robots are at NOW — the distance that cleared the conflict, not
# the one that opened it.
SafetyFieldPair[] pairs
