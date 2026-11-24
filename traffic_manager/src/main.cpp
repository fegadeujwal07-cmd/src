#include "traffic_manager/traffic_manager_node.hpp"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<traffic_manager::TrafficManagerNode>());
  rclcpp::shutdown();
  return 0;
}
