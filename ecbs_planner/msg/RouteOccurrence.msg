# Position on a planned route. Vertex id is NOT a route-array index.
# Repeated visits to the same vertex remain distinguishable via occurrence.

uint32 path_revision
# 0-based visit count along the route array (waypoint sequence index).
uint32 route_occurrence
# Metres along the segment from route_occurrence toward the next waypoint.
float64 segment_offset_m
# Secondary convenience (may equal graph vertex id when known; 0 is valid).
int64 waypoint_index
float64 x
float64 y
string level_name
