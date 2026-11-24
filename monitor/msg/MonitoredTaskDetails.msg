string task_id
string amr_id
string fleet_name
string category
string priority
string status
string load_status
string phase
# CSV "x,y[,yaw]" strings from execution (kept for wire compatibility).
string[] planned_path
# Optional numeric path (preferred by zone_monitor when both are set).
float64[] planned_path_x
float64[] planned_path_y
string current_zone
# Up to the next 3 upcoming zones (fewer if fewer remain).
string[] next_zone
# Remaining zones from current onward (passed zones are dropped; empty from execution).
string[] zone_sequence
