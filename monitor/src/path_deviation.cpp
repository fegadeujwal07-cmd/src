// =============================================================================
// File: monitor/src/path_deviation.cpp
// =============================================================================
// Standalone AMR path deviation monitor node.
//
// Separated out of zone_monitor_node.cpp to decouple path tracking and cross-track
// error monitoring from navigation graph zone management.
// =============================================================================

#include "monitor/path_deviation.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "monitor/msg/monitored_task_details.hpp"
#include "monitor/zone_tracker.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rmf_fleet_msgs/msg/fleet_state.hpp"
#include "rmf_fleet_msgs/msg/robot_mode.hpp"
#include "std_msgs/msg/string.hpp"
#include "yaml-cpp/yaml.h"

namespace monitor
{

// ============================================================================
// Set path deviation tolerance here
//============================================================================
constexpr double kPathDeviationToleranceMeters = 1.0;

PathDeviationNode::PathDeviationNode(const rclcpp::NodeOptions & options)
: Node("path_deviation", options)
{
  declare_parameter<bool>("enable_path_deviation_check", true);
  declare_parameter<double>("path_deviation_tolerance_m", kPathDeviationToleranceMeters);
  declare_parameter<int>("path_deviation_debounce_ticks", 8);
  declare_parameter<std::string>("path_deviation_topic", "/path_deviation");
  declare_parameter<std::string>("fleet_states_topic", "/fleet_states");
  declare_parameter<std::string>("monitored_task_topic", "/Monitored_task_details");
  declare_parameter<std::string>("task_state_topic", "/rmf_task_state_updates");
  declare_parameter<std::string>("level", "L1");
  declare_parameter<double>("check_rate_hz", 10.0);

  enable_path_deviation_check_ = get_parameter("enable_path_deviation_check").as_bool();
  path_deviation_tolerance_m_ = get_parameter("path_deviation_tolerance_m").as_double();
  path_deviation_debounce_ticks_ = get_parameter("path_deviation_debounce_ticks").as_int();
  path_deviation_topic_ = get_parameter("path_deviation_topic").as_string();
  fleet_states_topic_ = get_parameter("fleet_states_topic").as_string();
  monitored_task_topic_ = get_parameter("monitored_task_topic").as_string();
  task_state_topic_ = get_parameter("task_state_topic").as_string();
  level_ = get_parameter("level").as_string();

  constexpr int kQosDepth = 200;

  // Publisher for /path_deviation alerts & recoveries
  auto path_deviation_qos = rclcpp::QoS(rclcpp::KeepLast(50)).reliable();
  path_deviation_pub_ = create_publisher<std_msgs::msg::String>(
    path_deviation_topic_, path_deviation_qos);

  // Fleet state subscriber (robot live pose and mode)
  fleet_sub_ = create_subscription<rmf_fleet_msgs::msg::FleetState>(
    fleet_states_topic_,
    kQosDepth,
    std::bind(&PathDeviationNode::on_fleet_state, this, std::placeholders::_1));

  // Monitored task subscriber (planned path coordinates)
  task_sub_ = create_subscription<monitor::msg::MonitoredTaskDetails>(
    monitored_task_topic_,
    kQosDepth,
    std::bind(&PathDeviationNode::on_monitored_task, this, std::placeholders::_1));

  // Task state updates subscriber (completion / cancellation tracking)
  auto task_state_qos = rclcpp::QoS(rclcpp::KeepLast(20));
  task_state_qos.reliable();
  task_state_qos.transient_local();
  task_state_sub_ = create_subscription<std_msgs::msg::String>(
    task_state_topic_,
    task_state_qos,
    std::bind(&PathDeviationNode::on_task_state_update, this, std::placeholders::_1));

  // Check Timer — 10 Hz default (100 ms)
  const double rate_hz = std::max(get_parameter("check_rate_hz").as_double(), 0.1);
  timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / rate_hz),
    std::bind(&PathDeviationNode::check_path_deviations, this));

  RCLCPP_INFO(
    get_logger(),
    "Path deviation monitor ready (sub: %s, %s | pub: %s | "
    "tol: %.3fm (%.1f mm), debounce: %d ticks @ %.1f Hz)",
    fleet_states_topic_.c_str(),
    monitored_task_topic_.c_str(),
    path_deviation_topic_.c_str(),
    path_deviation_tolerance_m_,
    path_deviation_tolerance_m_ * 1000.0,
    path_deviation_debounce_ticks_,
    rate_hz);
}

bool PathDeviationNode::is_terminal_status(const std::string & status)
{
  std::string key = status;
  std::transform(key.begin(), key.end(), key.begin(), ::tolower);
  return key == "completed" || key == "canceled" || key == "cancelled" || key == "failed";
}

void PathDeviationNode::clear_task_tracking(const std::string & robot_id)
{
  planned_path_by_robot_.erase(robot_id);
  robot_deviation_states_.erase(robot_id);
}

void PathDeviationNode::on_fleet_state(const rmf_fleet_msgs::msg::FleetState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto & robot : msg->robots) {
    const std::string robot_id = ZoneTracker::normalize_robot_id(robot.name);
    if (robot_id.empty()) {
      continue;
    }
    if (!level_.empty() && !robot.location.level_name.empty() &&
      robot.location.level_name != level_)
    {
      continue;
    }

    // Read Robot Data (Position x,y · Mode · Task ID · Timestamp)
    auto & dev_state = robot_deviation_states_[robot_id];
    dev_state.x = robot.location.x;
    dev_state.y = robot.location.y;
    dev_state.mode = robot.mode.mode;
    dev_state.task_id = robot.task_id;
    dev_state.valid = true;

    if (robot.task_id.empty()) {
      planned_path_by_robot_.erase(robot_id);
      robot_deviation_states_.erase(robot_id);
      continue;
    }
  }
}

void PathDeviationNode::on_monitored_task(const monitor::msg::MonitoredTaskDetails::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);

  const std::string robot_id = ZoneTracker::normalize_robot_id(msg->amr_id);
  if (robot_id.empty()) {
    return;
  }

  if (is_terminal_status(msg->status)) {
    clear_task_tracking(robot_id);
    return;
  }

  const bool has_numeric_path =
    !msg->planned_path_x.empty() &&
    msg->planned_path_x.size() == msg->planned_path_y.size();
  if (!has_numeric_path && msg->planned_path.empty()) {
    return;
  }

  // Store full planned path keyed by robot + task ID
  if (!msg->task_id.empty()) {
    std::vector<Point2D> waypoints;
    if (has_numeric_path) {
      waypoints.reserve(msg->planned_path_x.size());
      for (std::size_t i = 0; i < msg->planned_path_x.size(); ++i) {
        waypoints.push_back({msg->planned_path_x[i], msg->planned_path_y[i]});
      }
    } else if (!msg->planned_path.empty()) {
      const auto pts = ZoneTracker::parse_planned_path_strings(
        std::vector<std::string>(msg->planned_path.begin(), msg->planned_path.end()));
      waypoints.reserve(pts.size());
      for (const auto & pt : pts) {
        waypoints.push_back({pt.first, pt.second});
      }
    }
    if (waypoints.size() >= 2) {
      uint64_t next_version = 1;
      const auto existing_it = planned_path_by_robot_.find(robot_id);
      if (existing_it != planned_path_by_robot_.end()) {
        next_version = existing_it->second.version + 1;
      }
      planned_path_by_robot_[robot_id] = {msg->task_id, std::move(waypoints), next_version};
    }
  }
}

void PathDeviationNode::on_task_state_update(const std_msgs::msg::String::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);

  YAML::Node task_state;
  try {
    task_state = YAML::Load(msg->data);
  } catch (...) {
    return;
  }

  const YAML::Node booking = task_state["booking"];
  std::string task_id;
  if (booking && booking.IsMap() && booking["id"]) {
    try {
      task_id = booking["id"].as<std::string>();
    } catch (...) {}
  }
  if (task_id.empty()) {
    return;
  }

  std::string status;
  if (task_state["status"]) {
    try {
      status = task_state["status"].as<std::string>();
    } catch (...) {}
  }

  std::string robot_name;
  const YAML::Node assigned_to = task_state["assigned_to"];
  if (assigned_to && assigned_to.IsMap() && assigned_to["name"]) {
    try {
      robot_name = assigned_to["name"].as<std::string>();
    } catch (...) {}
  }
  const std::string robot_id = ZoneTracker::normalize_robot_id(robot_name);

  if (is_terminal_status(status)) {
    if (!robot_id.empty()) {
      const auto path_it = planned_path_by_robot_.find(robot_id);
      if (path_it == planned_path_by_robot_.end() ||
          path_it->second.task_id.empty() ||
          path_it->second.task_id == task_id)
      {
        clear_task_tracking(robot_id);
      }
    }
  }
}

std::size_t PathDeviationNode::find_closest_segment_index(
  const double px, const double py,
  const std::vector<Point2D> & waypoints) const
{
  if (waypoints.size() < 2) {
    return 0;
  }
  std::size_t closest_idx = 0;
  double min_dist = std::numeric_limits<double>::infinity();

  for (std::size_t i = 0; i + 1 < waypoints.size(); ++i) {
    const double d = point_to_segment_distance(
      px, py,
      waypoints[i].x, waypoints[i].y,
      waypoints[i + 1].x, waypoints[i + 1].y);
    if (d < min_dist) {
      min_dist = d;
      closest_idx = i;
    }
  }
  return closest_idx;
}

double PathDeviationNode::compute_deviation_to_segments(
  const double px, const double py,
  const std::vector<Point2D> & waypoints,
  std::size_t & current_seg_idx) const
{
  const std::size_t num_pts = waypoints.size();
  if (num_pts < 2) {
    return 0.0;
  }
  const std::size_t num_segs = num_pts - 1;
  if (current_seg_idx >= num_segs) {
    current_seg_idx = num_segs - 1;
  }

  // Segment 1 (current)
  const auto & p0 = waypoints[current_seg_idx];
  const auto & p1 = waypoints[current_seg_idx + 1];
  double min_dist = point_to_segment_distance(px, py, p0.x, p0.y, p1.x, p1.y);
  std::size_t best_seg_idx = current_seg_idx;

  // Segment 2 (next)
  if (current_seg_idx + 1 < num_segs) {
    const auto & p2 = waypoints[current_seg_idx + 2];
    const double d2 = point_to_segment_distance(px, py, p1.x, p1.y, p2.x, p2.y);
    if (d2 < min_dist) {
      min_dist = d2;
      best_seg_idx = current_seg_idx + 1;
    }
  }

  // Segment 3 (next + 1)
  if (current_seg_idx + 2 < num_segs) {
    const auto & p2 = waypoints[current_seg_idx + 2];
    const auto & p3 = waypoints[current_seg_idx + 3];
    const double d3 = point_to_segment_distance(px, py, p2.x, p2.y, p3.x, p3.y);
    if (d3 < min_dist) {
      min_dist = d3;
      best_seg_idx = current_seg_idx + 2;
    }
  }

  // Advance segment index when robot progresses along path
  if (best_seg_idx > current_seg_idx) {
    current_seg_idx = best_seg_idx;
  } else {
    const double t = segment_projection_t(px, py, p0.x, p0.y, p1.x, p1.y);
    if (t >= 1.0 && current_seg_idx + 1 < num_segs) {
      current_seg_idx++;
    }
  }

  return min_dist;
}

void PathDeviationNode::publish_path_deviation_alert(
  const std::string & robot_id,
  const std::string & task_id,
  const double deviation)
{
  if (!path_deviation_pub_) {
    return;
  }

  // Format JSON payload string: Robot · Task · Deviation (m)
  std::ostringstream ss;
  ss << std::fixed << std::setprecision(4);
  ss << "{\"robot\":\"" << robot_id
     << "\",\"task_id\":\"" << task_id
     << "\",\"deviation\":" << deviation << "}";

  std_msgs::msg::String alert;
  alert.data = ss.str();
  path_deviation_pub_->publish(alert);

  RCLCPP_WARN(
    get_logger(),
    "🚨 [PATH DEVIATION CONFIRMED] Published %s: "
    "robot=%s, task=%s, deviation=%.4fm (tolerance=%.3fm)",
    path_deviation_topic_.c_str(), robot_id.c_str(), task_id.c_str(),
    deviation, path_deviation_tolerance_m_);
}

void PathDeviationNode::publish_path_recovered_alert(
  const std::string & robot_id,
  const std::string & task_id,
  const double deviation)
{
  if (!path_deviation_pub_) {
    return;
  }

  std::ostringstream ss;
  ss << std::fixed << std::setprecision(4);
  ss << "{\"event\":\"recovered\",\"robot\":\"" << robot_id
     << "\",\"task_id\":\"" << task_id
     << "\",\"deviation\":" << deviation << "}";

  std_msgs::msg::String alert;
  alert.data = ss.str();
  path_deviation_pub_->publish(alert);

  RCLCPP_INFO(
    get_logger(),
    "[PATH RECOVERED] Published %s: robot=%s, task=%s, deviation=%.4fm",
    path_deviation_topic_.c_str(), robot_id.c_str(), task_id.c_str(), deviation);
}

void PathDeviationNode::check_path_deviations()
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (!enable_path_deviation_check_) {
    return;
  }

  for (auto & [robot_id, dev_state] : robot_deviation_states_) {
    if (!dev_state.valid) {
      continue;
    }

    // Check if robot has a stored planned path
    const auto path_it = planned_path_by_robot_.find(robot_id);
    if (path_it == planned_path_by_robot_.end() || path_it->second.waypoints.size() < 2) {
      continue;
    }

    const auto & planned = path_it->second;

    // Stage 2 (Gate Check C): New task or new path revision/replan?
    if (dev_state.current_path_task_id != planned.task_id ||
        dev_state.current_path_version != planned.version)
    {
      dev_state.current_path_task_id = planned.task_id;
      dev_state.current_path_version = planned.version;
      dev_state.debounce_ticks = 0;
      dev_state.recovery_debounce_ticks = 0;
      dev_state.deviation_reported = false;

      // Stage 2 (C1): Find current position on path to calculate path deviation freshly
      dev_state.current_segment_index = find_closest_segment_index(
        dev_state.x, dev_state.y, planned.waypoints);
    }

    // FleetState publishes the RMF path ID, while MonitoredTaskDetails uses
    // the booking/order ID. They identify the same active robot task but are
    // intentionally different strings, so only require an active task here.
    const bool is_moving = (dev_state.mode == rmf_fleet_msgs::msg::RobotMode::MODE_MOVING);
    const bool has_active_task = !dev_state.task_id.empty();
    if (!has_active_task || (!is_moving && !dev_state.deviation_reported)) {
      continue;  // Skip (not navigating or idle)
    }

    if (dev_state.deviation_reported) {
      // Re-anchor closest segment while recovering so operator teleop anywhere
      // along the path is properly recognized
      dev_state.current_segment_index = find_closest_segment_index(
        dev_state.x, dev_state.y, planned.waypoints);

      const double recov_deviation = compute_deviation_to_segments(
        dev_state.x, dev_state.y, planned.waypoints, dev_state.current_segment_index);

      if (recov_deviation <= path_deviation_tolerance_m_) {
        dev_state.recovery_debounce_ticks++;
        if (dev_state.recovery_debounce_ticks >= path_deviation_debounce_ticks_) {
          publish_path_recovered_alert(robot_id, planned.task_id, recov_deviation);
          dev_state.deviation_reported = false;
          dev_state.debounce_ticks = 0;
          dev_state.recovery_debounce_ticks = 0;
        }
      } else {
        dev_state.recovery_debounce_ticks = 0;
      }
      continue;
    }

    // Stage 3 (F, G): Deviation calculation across current + next 2 segments
    const double min_deviation = compute_deviation_to_segments(
      dev_state.x, dev_state.y, planned.waypoints, dev_state.current_segment_index);

    // Stage 4 (H): Threshold check (> configured tolerance)
    if (is_moving && min_deviation > path_deviation_tolerance_m_) {
      // Stage 4 (I): Count +1
      dev_state.debounce_ticks++;

      RCLCPP_DEBUG(
        get_logger(),
        "⚠️ [PATH DEVIATION] Robot %s deviated by %.4fm (limit: %.3fm), debounce %d/%d ticks",
        robot_id.c_str(), min_deviation, path_deviation_tolerance_m_,
        dev_state.debounce_ticks, path_deviation_debounce_ticks_);

      // Stage 4 (J): 3 ticks in a row?
      if (dev_state.debounce_ticks >= path_deviation_debounce_ticks_) {
        // Stage 5 (K, M): PATH DEVIATION CONFIRMED -> Publish /path_deviation
        publish_path_deviation_alert(robot_id, planned.task_id, min_deviation);

        // Stage 5 (N): Latch Robot (report once per task)
        dev_state.deviation_reported = true;
        dev_state.recovery_debounce_ticks = 0;
      }
    } else {
      // Stage 4 (H1): On path -> Reset debounce counter
      dev_state.debounce_ticks = 0;
    }
  }
}

}  // namespace monitor

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<monitor::PathDeviationNode>());
  rclcpp::shutdown();
  return 0;
}
