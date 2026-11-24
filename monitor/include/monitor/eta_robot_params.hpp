#pragma once

#include <string>
#include <utility>
#include <vector>

namespace monitor
{

/// Motion model for ETA (loaded from execution/config/eta_robot_params.yaml).
/// Keep ETA parameter loading here so zone_monitor_node.cpp only attaches it.
struct EtaRobotParams
{
  double operating_linear_speed_mps{0.5};
  double linear_accel_mps2{0.5};
  double linear_decel_mps2{0.6};
  double angular_speed_max_rps{0.85};
  double angular_accel_rps2{1.0};

  double curve_linear_speed_mps{0.45};
  double curve_radius_threshold_m{5.0};
  double heading_min_deg{3.0};
  double heading_max_deg{25.0};
  double heading_change_threshold_deg{20.0};

  double approach_start_distance_m{5.0};
  double curve_entry_distance_m{0.5};
  double required_curve_speed_mps{0.45};

  /// speed_mps, distance_m
  std::vector<std::pair<double, double>> stop_decel_table;

  double robot_length_m{1.403};
  double robot_width_m{1.194};
  double robot_height_m{1.153};

  bool loaded{false};
};

/// Load ETA motion params from YAML. Returns defaults with loaded=false on failure.
EtaRobotParams load_eta_robot_params(const std::string & yaml_path);

/// Process-wide params from eta_robot_params.yaml (first successful load wins).
/// Searches ament share + common workspace relative paths.
const EtaRobotParams & cached_eta_robot_params();

}  // namespace monitor
