
# ID of the schedule participant whose itinerary is being set
uint64 participant

# ID of this plan
uint64 plan

# The new itinerary for this participant
Route[] itinerary

# The storage location for these routes
uint64 storage_base

uint64 itinerary_version

# Total number of waypoints across all routes in this plan (sum of trajectory.waypoints)
uint64 waypoint_count

# Robot name for this participant (for easier identification; empty if unknown)
string robot_name
