# One waypoint in EvaluatePath evaluate / FromZoneManager path seed.
# evaluate / seed: geometry only (t_sec,x,y,yaw,level_name,index).
# ZM wait-decision register does NOT send waypoints — use RegisterUpdate.updates[].
float64 t_sec
float64 x
float64 y
float64 yaw
string level_name
# Graph waypoint id; use -1 when unknown / not set.
int64 index
# Unused on evaluate. Optional on path seed; ZM register uses RegisterUpdate.
float64 wait_sec
WaitDependency[] waiting_for
