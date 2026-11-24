// =============================================================================
// File: monitor/include/monitor/path_deviation.hpp
// =============================================================================
// Path deviation monitoring node for AMRs.
//
// Subscribes to:
//   - /Monitored_task_details (monitor/msg/MonitoredTaskDetails): planned path
//   - /fleet_states (rmf_fleet_msgs/msg/FleetState): live robot poses & modes
//   - /rmf_task_state_updates (std_msgs/msg/String): task state changes
//
// Publishes to:
//   - /path_deviation (std_msgs/msg/String): deviation and recovery events
//
// Core features:
//   - Multi-segment cross-track Euclidean distance lookahead
//   - Debounce window at 10 Hz (default 3 ticks)
//   - Automated recovery detection and latching
// =============================================================================

#ifndef MONITOR__PATH_DEVIATION_HPP_
#define MONITOR__PATH_DEVIATION_HPP_

#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "monitor/msg/monitored_task_details.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rmf_fleet_msgs/msg/fleet_state.hpp"
#include "std_msgs/msg/string.hpp"

namespace monitor
{

/**
 * @brief Simple 2D point representation for geometric trajectory calculations.
 */
struct Point2D
{
  double x{0.0};
  double y{0.0};
};

/**
 * @brief Computes the perpendicular cross-track Euclidean distance from a point P(px, py)
 *        to a line segment running from A(ax, ay) to B(bx, by).
 *
 * If the projection of P falls outside the segment endpoints:
 *   - t <= 0.0: closest point is A
 *   - t >= 1.0: closest point is B
 * Otherwise, the perpendicular distance to the line segment is returned.
 *
 * @param px X coordinate of the query point
 * @param py Y coordinate of the query point
 * @param ax X coordinate of segment start
 * @param ay Y coordinate of segment start
 * @param bx X coordinate of segment end
 * @param by Y coordinate of segment end
 * @return Perpendicular or clamped Euclidean distance in meters
 */
inline double point_to_segment_distance(
  const double px, const double py,
  const double ax, const double ay,
  const double bx, const double by)
{
  // Vector components of segment AB
  const double dx = bx - ax;
  const double dy = by - ay;
  const double seg_len_sq = (dx * dx) + (dy * dy);

  // Degenerate segment guard: if A == B (zero length), return direct distance to A
  constexpr double kEpsilonSquared = 1e-8;
  if (seg_len_sq < kEpsilonSquared) {
    return std::hypot(px - ax, py - ay);
  }

  // Projection parameter t of point P onto infinite line passing through A and B:
  // t = ((P - A) . (B - A)) / |B - A|^2
  const double t = (((px - ax) * dx) + ((py - ay) * dy)) / seg_len_sq;

  // Clamping to segment endpoints when projection falls outside:
  if (t <= 0.0) {
    // Closest point is start vertex A
    return std::hypot(px - ax, py - ay);
  }
  if (t >= 1.0) {
    // Closest point is end vertex B
    return std::hypot(px - bx, py - by);
  }

  // Interior cross-track perpendicular distance formula:
  // e = |(x2 - x1) * (y1 - yr) - (x1 - xr) * (y2 - y1)| / sqrt(dx^2 + dy^2)
  const double cross_product = std::abs(dx * (ay - py) - (ax - px) * dy);
  return cross_product / std::sqrt(seg_len_sq);
}

/**
 * @brief Computes projection parameter t of point P(px, py) onto line segment AB.
 *
 * @return Value of t:
 *   - t < 0.0: projection is before start vertex A
 *   - 0.0 <= t <= 1.0: projection is within segment AB
 *   - t > 1.0: projection is past end vertex B
 */
inline double segment_projection_t(
  const double px, const double py,
  const double ax, const double ay,
  const double bx, const double by)
{
  const double dx = bx - ax;
  const double dy = by - ay;
  const double seg_len_sq = (dx * dx) + (dy * dy);

  constexpr double kEpsilonSquared = 1e-8;
  if (seg_len_sq < kEpsilonSquared) {
    return 0.0;
  }
  return (((px - ax) * dx) + ((py - ay) * dy)) / seg_len_sq;
}

/// Stored planned path per robot
struct RobotPlannedPath
{
  std::string task_id;
  std::vector<Point2D> waypoints;
  uint64_t version{0};
};

/// Live tracking and debounce state per robot
struct RobotDeviationState
{
  double x{0.0};
  double y{0.0};
  uint32_t mode{0};
  std::string task_id;
  std::string current_path_task_id;
  uint64_t current_path_version{0};
  std::size_t current_segment_index{0};
  int debounce_ticks{0};
  int recovery_debounce_ticks{0};
  bool deviation_reported{false};
  bool valid{false};
};

class PathDeviationNode : public rclcpp::Node
{
public:
  explicit PathDeviationNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~PathDeviationNode() = default;

  void on_fleet_state(const rmf_fleet_msgs::msg::FleetState::SharedPtr msg);
  void on_monitored_task(const monitor::msg::MonitoredTaskDetails::SharedPtr msg);
  void on_task_state_update(const std_msgs::msg::String::SharedPtr msg);
  void check_path_deviations();

  std::size_t find_closest_segment_index(
    double px, double py,
    const std::vector<Point2D> & waypoints) const;

  double compute_deviation_to_segments(
    double px, double py,
    const std::vector<Point2D> & waypoints,
    std::size_t & current_seg_idx) const;

  void publish_path_deviation_alert(
    const std::string & robot_id,
    const std::string & task_id,
    double deviation);

  void publish_path_recovered_alert(
    const std::string & robot_id,
    const std::string & task_id,
    double deviation);

  void clear_task_tracking(const std::string & robot_id);

  static bool is_terminal_status(const std::string & status);

private:
  bool enable_path_deviation_check_{true};
  double path_deviation_tolerance_m_{0.350};
  int path_deviation_debounce_ticks_{8};
  std::string path_deviation_topic_{"/path_deviation"};
  std::string fleet_states_topic_{"/fleet_states"};
  std::string monitored_task_topic_{"/Monitored_task_details"};
  std::string task_state_topic_{"/rmf_task_state_updates"};
  std::string level_{"L1"};

  mutable std::mutex mutex_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr path_deviation_pub_;
  rclcpp::Subscription<rmf_fleet_msgs::msg::FleetState>::SharedPtr fleet_sub_;
  rclcpp::Subscription<monitor::msg::MonitoredTaskDetails>::SharedPtr task_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr task_state_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::unordered_map<std::string, RobotPlannedPath> planned_path_by_robot_;
  std::unordered_map<std::string, RobotDeviationState> robot_deviation_states_;
};

}  // namespace monitor

#endif  // MONITOR__PATH_DEVIATION_HPP_
