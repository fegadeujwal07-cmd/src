#include "ecbs_planner/ecbs_node.hpp"

#include <algorithm>
#include <cmath>  // floor, llround for time conversion

#include "builtin_interfaces/msg/time.hpp"
#include "ecbs_planner/msg/clear_point.hpp"
#include "ecbs_planner/msg/path_point.hpp"
#include "ecbs_planner/msg/wait_dependency.hpp"
#include "ecbs_planner/evaluate_path.hpp"
#include "ecbs_planner/path_utils.hpp"

namespace ecbs_planner
{
namespace
{

/// ROS Time → seconds as double.
double stamp_to_sec(const builtin_interfaces::msg::Time & t)
{
  return static_cast<double>(t.sec) + 1e-9 * static_cast<double>(t.nanosec);
}

/// Seconds as double → ROS Time.
builtin_interfaces::msg::Time sec_to_stamp(double t_sec)
{
  builtin_interfaces::msg::Time out;
  if (t_sec < 0.0) {
    t_sec = 0.0;
  }
  const auto sec = static_cast<int32_t>(std::floor(t_sec));
  out.sec = sec;
  out.nanosec = static_cast<uint32_t>(std::llround((t_sec - sec) * 1e9));
  if (out.nanosec >= 1000000000u) {
    out.sec += 1;
    out.nanosec -= 1000000000u;
  }
  return out;
}

}  // namespace

/// Construct standalone ECBS node (optional; production uses EcbsGate instead).
EcbsNode::EcbsNode(const rclcpp::NodeOptions & options)
: Node("ecbs_planner", options)
{
  // Solver tunables — defaults from SolverConfig (config/ecbs_params.yaml).
  const SolverConfig defaults;
  safety_margin_sec_ =
    declare_parameter<double>("safety_margin_sec", defaults.safety_margin_sec);
  max_wait_sec_ = declare_parameter<double>("max_wait_sec", defaults.max_wait_sec);
  max_iters_ = declare_parameter<int>("max_iters", defaults.max_iters);
  position_epsilon_m_ =
    declare_parameter<double>("position_epsilon_m", defaults.position_epsilon_m);

  // Topic names (overridable via launch).
  const auto candidate_topic =
    declare_parameter<std::string>("candidate_topic", "/ecbs_path_candidates");
  const auto itinerary_topic =
    declare_parameter<std::string>("itinerary_set_topic", "/rmf_traffic/itinerary_set");
  const auto to_zone_topic =
    declare_parameter<std::string>("to_zone_manager_topic", "/to_zone_manager");
  const auto replan_topic =
    declare_parameter<std::string>("replan_topic", "/ecbs_replan_candidate");

  // Build wait-only solver from params.
  SolverConfig cfg;
  cfg.safety_margin_sec = safety_margin_sec_;
  cfg.max_wait_sec = max_wait_sec_;
  cfg.max_iters = max_iters_;
  cfg.position_epsilon_m = position_epsilon_m_;
  cfg.safety_distance_m =
    declare_parameter<double>("safety_distance_m", defaults.safety_distance_m);
  cfg.clearance_m = declare_parameter<double>("clearance_m", defaults.clearance_m);
  cfg.amr_length_m = declare_parameter<double>("amr_length_m", defaults.amr_length_m);
  cfg.amr_width_m = declare_parameter<double>("amr_width_m", defaults.amr_width_m);
  cfg.evaluate_lookahead_margin_m = declare_parameter<double>(
    "evaluate_lookahead_margin_m", defaults.evaluate_lookahead_margin_m);
  cfg.junction_release_m =
    declare_parameter<double>("junction_release_m", defaults.junction_release_m);
  solver_ = EcbsSolver(cfg);

  const auto qos = rclcpp::SystemDefaultsQoS();

  to_zone_pub_ = create_publisher<msg::ToZoneManager>(to_zone_topic, qos);
  replan_pub_ = create_publisher<msg::EcbsReplan>(replan_topic, qos);

  // Greedy path candidates in.
  candidate_sub_ = create_subscription<rmf_fleet_msgs::msg::PathRequest>(
    candidate_topic, qos,
    std::bind(&EcbsNode::on_path_candidate, this, std::placeholders::_1));

  // Optional schedule seed from traffic itinerary.
  itinerary_sub_ = create_subscription<rmf_traffic_msgs::msg::ItinerarySet>(
    itinerary_topic, qos,
    std::bind(&EcbsNode::on_itinerary_set, this, std::placeholders::_1));

  RCLCPP_INFO(
    get_logger(),
    "ECBS ready: candidates=%s → %s (lean path + wait deps)",
    candidate_topic.c_str(), to_zone_topic.c_str());
  const auto & c = solver_.config();
  RCLCPP_INFO(
    get_logger(),
    "ECBS SolverConfig: safety_distance_m=%.3f clearance_m=%.3f "
    "amr_length_m=%.3f amr_width_m=%.3f position_epsilon_m=%.3f "
    "evaluate_lookahead_margin_m=%.3f junction_release_m=%.3f "
    "safety_margin_sec=%.3f max_wait_sec=%.3f max_iters=%d",
    c.safety_distance_m, c.clearance_m, c.amr_length_m, c.amr_width_m,
    c.position_epsilon_m, c.evaluate_lookahead_margin_m, c.junction_release_m,
    c.safety_margin_sec, c.max_wait_sec, c.max_iters);
}

/// Convert PathRequest locations into TimedPath.
TimedPath EcbsNode::from_path_request(const rmf_fleet_msgs::msg::PathRequest & msg) const
{
  TimedPath path;
  path.amr_id = msg.robot_name;
  path.fleet_name = msg.fleet_name;
  path.task_id = msg.task_id;
  path.dispatch_task_id = msg.dispatch_task_id;
  path.phase_index = msg.phase_index;
  path.path_revision = msg.path_revision;
  path.waypoints.reserve(msg.path.size());
  for (const auto & loc : msg.path) {
    TimedPoint wp;
    wp.t_sec = stamp_to_sec(loc.t);
    wp.x = loc.x;
    wp.y = loc.y;
    wp.yaw = loc.yaw;
    wp.level_name = loc.level_name;
    // Only copy index when level is set (same convention as PathRequest usage).
    if (!loc.level_name.empty()) {
      wp.index = loc.index;
    }
    wp.wait_duration_sec = 0.0;  // waits filled by solver later
    path.waypoints.push_back(wp);
  }
  if (!path.waypoints.empty()) {
    path.waypoints.front().t_sec = 0.0;
    stamp_timed_path_with_eta(path, 0.0);
  }
  return path;
}

/// Flatten itinerary trajectories into a TimedPath for cache seeding.
TimedPath EcbsNode::from_itinerary(
  const std::string & amr_id,
  const rmf_traffic_msgs::msg::ItinerarySet & msg) const
{
  TimedPath path;
  path.amr_id = amr_id;
  for (const auto & route : msg.itinerary) {
    for (const auto & tw : route.trajectory.waypoints) {
      TimedPoint wp;
      // traffic msgs store time in nanoseconds.
      wp.t_sec = 1e-9 * static_cast<double>(tw.time);
      if (tw.position.size() >= 2) {
        wp.x = tw.position[0];
        wp.y = tw.position[1];
      }
      if (tw.position.size() >= 3) {
        wp.yaw = tw.position[2];
      }
      wp.level_name = route.map;
      path.waypoints.push_back(wp);
    }
  }
  if (!path.waypoints.empty()) {
    path.waypoints.front().t_sec = 0.0;
    stamp_timed_path_with_eta(path, 0.0);
  }
  return path;
}

/// Seed cache from itinerary only if this AMR has no committed path yet.
void EcbsNode::on_itinerary_set(const rmf_traffic_msgs::msg::ItinerarySet::SharedPtr msg)
{
  if (!msg) {
    return;
  }

  std::string amr_id = msg->robot_name;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!msg->robot_name.empty()) {
      // Remember participant → name for later msgs missing robot_name.
      participant_to_amr_[msg->participant] = msg->robot_name;
    } else {
      const auto it = participant_to_amr_.find(msg->participant);
      if (it != participant_to_amr_.end()) {
        amr_id = it->second;
      }
    }
  }

  if (amr_id.empty()) {
    return;  // cannot key cache without an id
  }

  std::lock_guard<std::mutex> lock(mutex_);
  // Do not overwrite a path ECBS already committed.
  if (cache_.get(amr_id).has_value()) {
    return;
  }
  auto path = from_itinerary(amr_id, *msg);
  if (path.waypoints.empty()) {
    return;
  }
  cache_.upsert(path);
}

/// Build lean /to_zone_manager message from TimedPath.
msg::ToZoneManager EcbsNode::to_zone_msg(const TimedPath & path) const
{
  msg::ToZoneManager out;
  out.fleet_name = path.fleet_name;
  out.amr_id = path.amr_id;
  out.path.reserve(path.waypoints.size());

  for (const auto & wp : path.waypoints) {
    msg::PathPoint pt;
    pt.t = sec_to_stamp(wp.t_sec);
    pt.x = wp.x;
    pt.y = wp.y;
    pt.wait_sec = published_wait_sec(wp);
    for (const auto & dep : wp.waiting_for) {
      msg::WaitDependency d;
      d.amr_id = dep.amr_id;
      d.topology = dep.topology;
      d.self_blocks_other = dep.self_blocks_other;
      d.other_blocks_self = dep.other_blocks_self;
      d.wait_point.x = dep.wait_point.x;
      d.wait_point.y = dep.wait_point.y;
      d.wait_point.index = dep.wait_point.index.has_value() ?
        static_cast<int64_t>(*dep.wait_point.index) : static_cast<int64_t>(-1);
      d.wait_point.wait_sec = dep.wait_point.wait_duration_sec;
      for (const auto & c : dep.clear_points) {
        msg::ClearPoint cp;
        cp.x = c.x;
        cp.y = c.y;
        d.clear_points.push_back(cp);
      }
      pt.waiting_for.push_back(std::move(d));
    }
    out.path.push_back(std::move(pt));
  }
  return out;
}

/// Publish path and log how many wait points it contains.
void EcbsNode::publish_path(const TimedPath & path)
{
  auto msg = to_zone_msg(path);
  to_zone_pub_->publish(msg);

  std::size_t wait_count = 0;
  for (const auto & p : msg.path) {
    if (p.wait_sec > 1e-6) {
      ++wait_count;
    }
  }
  RCLCPP_INFO(
    get_logger(),
    "Published /to_zone_manager amr=%s fleet=%s wps=%zu waits=%zu",
    path.amr_id.c_str(), path.fleet_name.c_str(),
    msg.path.size(), wait_count);
}

/// Publish replan candidate when solve fails.
void EcbsNode::publish_replan(const SolveResult & result)
{
  msg::EcbsReplan msg;
  msg.stamp = now();
  msg.amr_id = result.path.amr_id;
  msg.task_id = result.path.task_id;
  msg.reason = result.reason;
  msg.conflicting_amrs = result.conflicting_amrs;
  replan_pub_->publish(msg);
  RCLCPP_WARN(
    get_logger(),
    "ECBS replan needed for %s: %s",
    msg.amr_id.c_str(), msg.reason.c_str());
}

/// Main standalone pipeline: candidate → solve → publish (or replan).
void EcbsNode::on_path_candidate(const rmf_fleet_msgs::msg::PathRequest::SharedPtr msg)
{
  if (!msg || msg->robot_name.empty()) {
    return;
  }

  TimedPath candidate = from_path_request(*msg);

  SolveResult result;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    result = solver_.solve(candidate, cache_);
  }

  if (result.status == SolveStatus::FailReplan) {
    publish_replan(result);
    return;
  }

  publish_path(result.path);
  // Also publish any earlier AMRs we had to delay.
  for (const auto & other : result.updated_others) {
    publish_path(other);
  }
}

}  // namespace ecbs_planner
