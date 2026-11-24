#pragma once

#include "monitor/eta_robot_params.hpp"

#include <utility>
#include <vector>

namespace monitor
{

/// Extract remaining polyline from robot pose onto planned path (meters XY).
/// Empty if path is empty.
std::vector<std::pair<double, double>> remaining_path_points(
  double robot_x,
  double robot_y,
  const std::vector<std::pair<double, double>> & planned_path);

/// Reach time at each path point, in seconds from the start.
/// Point 0 is 0. The last point is the full drive time.
/// Straight, curve, and the slowdown before a curve each take their own time.
/// The robot stops once, at the last point. It does not stop at every point
/// inside the final approach.
std::vector<double> path_point_reach_times_sec(
  const std::vector<std::pair<double, double>> & path,
  const EtaRobotParams & params,
  double start_speed_mps = 0.0);

/// Estimate travel time (seconds) along a path using ETA motion params.
/// Models accel/cruise/decel, curve slowdown (heading change), approach, and final stop.
/// Does not exceed operating_linear_speed_mps; short segments use reachable peak speed.
///
/// @param path           Polyline to travel (already remaining or full)
/// @param params         From eta_robot_params.yaml
/// @param start_speed_mps Entry speed at path start (0 at task start)
double estimate_path_travel_time_sec(
  const std::vector<std::pair<double, double>> & path,
  const EtaRobotParams & params,
  double start_speed_mps = 0.0);

}  // namespace monitor
