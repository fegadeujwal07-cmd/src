# Robot → traffic_manager on /BaseRequests (VDA5050 rolling base extension handshake).

builtin_interfaces/Time stamp

string fleet_name
string robot_name
string task_id
string dispatch_task_id
# VDA5050 orderId (execution mirrors MQTT /state).
string order_id

# Must match the orderUpdateId the robot last applied (execution: phase_base + path_revision).
uint32 order_update_id

# 1 = robot needs next released segment; 0 = ack after applying an order update.
bool new_base_request
