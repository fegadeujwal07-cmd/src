string fleet_name
string robot_name
Location[] path
 
# task_id must be copied into future RobotState messages (segment counter in full_control)
string task_id
 
# Dispatch booking id from /dispatch_states (e.g. patrol.dispatch-0, delivery.dispatch-1)
string dispatch_task_id
 
# 0-based index of this navigation leg within the active task.
# A Delivery has leg 0 (pickup) and leg 1 (dropoff); a Patrol has one leg per
# destination. Stays the same across every path published for that leg,
# including replans.
uint32 phase_index
 
# 0 for the initial path of this leg; incremented once per replan of the same
# leg. Combined with phase_index: revision up + phase unchanged means the path
# was replanned; phase up means a new leg started.
uint32 path_revision
