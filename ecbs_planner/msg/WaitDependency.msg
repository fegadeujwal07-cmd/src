# AMR we are waiting on, safe hold for this overlap, and cells it must clear.
string amr_id                 # Blocker robot id
WaitPoint wait_point          # Safe waiting point for THIS overlap
ClearPoint[] clear_points     # Cells / poses that must be vacated
# Geometry for TM arbitration (see wait_reason.hpp / FMS_RULEBOOK).
string topology               # ownership|opposite|same_way|merge|cross|junction
bool self_blocks_other        # easiness: self clears peer if peer waits
bool other_blocks_self        # easiness: peer clears self if self waits
