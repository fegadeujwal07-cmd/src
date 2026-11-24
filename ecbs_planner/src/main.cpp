#include <memory>  // std::make_shared

#include "ecbs_planner/ecbs_node.hpp"
#include "rclcpp/rclcpp.hpp"

/// Optional standalone entrypoint (not needed when EcbsGate is in fleet adapter).
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);                                 // start ROS 2
  auto node = std::make_shared<ecbs_planner::EcbsNode>();  // create ECBS node
  rclcpp::spin(node);                                       // process callbacks
  rclcpp::shutdown();                                       // clean exit
  return 0;
}
