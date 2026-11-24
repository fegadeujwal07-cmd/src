// =============================================================================
// File: zone_manager/src/main.cpp
// =============================================================================
// OBJECTIVE
//   Process entry for zone_manager_node: create lifecycle node, configure,
//   activate, and spin. Conflict detection + /zone_conflicts handshake run
//   inside ZoneManagerNode (VDA5050 MQTT is handled by an external node).
//
// INPUTS:  argc/argv (ROS args). Topic I/O belongs to ZoneManagerNode.
// OUTPUTS: CONFIGURE → ACTIVATE autostart; blocking executor spin.
//
// POINTERS: std::shared_ptr<ZoneManagerNode> keeps the node alive for the
//           executor (reference-counted; no manual delete).
// =============================================================================

#include <memory>

#include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp/rclcpp.hpp"
#include "zone_manager/zone_manager_node.hpp"

int main(int argc, char ** argv)
{
  // 1) Init ROS 2 context for this process.
  rclcpp::init(argc, argv);

  // 2) Heap-allocate node via shared_ptr (shared ownership with executor).
  auto node = std::make_shared<zone_manager::ZoneManagerNode>();

  // 3) Autostart lifecycle: on_configure (I/O) then on_activate (timer).
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  // 4) Single-threaded executor delivers callbacks until shutdown.
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node->get_node_base_interface());
  exec.spin();

  // 5) Clean ROS teardown after spin returns.
  rclcpp::shutdown();
  return 0;
}
