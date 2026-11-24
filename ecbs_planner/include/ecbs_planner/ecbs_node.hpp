#ifndef ECBS_PLANNER__ECBS_NODE_HPP_
#define ECBS_PLANNER__ECBS_NODE_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include "ecbs_planner/ecbs_solver.hpp"
#include "ecbs_planner/msg/ecbs_replan.hpp"
#include "ecbs_planner/msg/to_zone_manager.hpp"
#include "ecbs_planner/schedule_cache.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rmf_fleet_msgs/msg/path_request.hpp"      // optional candidate input
#include "rmf_traffic_msgs/msg/itinerary_set.hpp"   // optional cache seed

namespace ecbs_planner
{

/// Optional standalone ROS node (NOT required when EcbsGate is in fleet adapter).
/// Subscribes to path candidates, runs the same EcbsSolver, publishes lean msgs.
class EcbsNode : public rclcpp::Node
{
public:
  /// Declare params, create pubs/subs, build solver.
  explicit EcbsNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  /// Callback: greedy PathRequest candidate → solve → publish.
  void on_path_candidate(const rmf_fleet_msgs::msg::PathRequest::SharedPtr msg);

  /// Callback: seed cache from traffic itinerary if AMR unknown.
  void on_itinerary_set(const rmf_traffic_msgs::msg::ItinerarySet::SharedPtr msg);

  /// PathRequest → TimedPath.
  TimedPath from_path_request(const rmf_fleet_msgs::msg::PathRequest & msg) const;

  /// ItinerarySet → TimedPath for one AMR.
  TimedPath from_itinerary(
    const std::string & amr_id,
    const rmf_traffic_msgs::msg::ItinerarySet & msg) const;

  /// TimedPath → lean ToZoneManager message.
  msg::ToZoneManager to_zone_msg(const TimedPath & path) const;

  /// Publish one path on /to_zone_manager.
  void publish_path(const TimedPath & path);

  /// Publish fail on /ecbs_replan_candidate.
  void publish_replan(const SolveResult & result);

  ScheduleCache cache_;   // Committed paths for this node process
  EcbsSolver solver_;     // Wait-only FIFO solver
  std::mutex mutex_;      // Protects cache_ / participant map / solve

  /// Map traffic participant id → robot name when itinerary lacks robot_name.
  std::unordered_map<uint64_t, std::string> participant_to_amr_;

  double safety_margin_sec_{2.0};    // ROS param → SolverConfig
  double max_wait_sec_{30.0};
  int max_iters_{20};
  double position_epsilon_m_{0.35};

  rclcpp::Subscription<rmf_fleet_msgs::msg::PathRequest>::SharedPtr candidate_sub_;
  rclcpp::Subscription<rmf_traffic_msgs::msg::ItinerarySet>::SharedPtr itinerary_sub_;
  rclcpp::Publisher<msg::ToZoneManager>::SharedPtr to_zone_pub_;
  rclcpp::Publisher<msg::EcbsReplan>::SharedPtr replan_pub_;
};

}  // namespace ecbs_planner

#endif  // ECBS_PLANNER__ECBS_NODE_HPP_
