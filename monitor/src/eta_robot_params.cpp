#include "monitor/eta_robot_params.hpp"

#include <algorithm>
#include <cstdlib>
#include <exception>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "yaml-cpp/yaml.h"

namespace monitor
{
namespace
{

double yaml_double(const YAML::Node & node, const char * key, double fallback)
{
  if (!node || !node.IsMap() || !node[key] || node[key].IsNull()) {
    return fallback;
  }
  try {
    return node[key].as<double>();
  } catch (...) {
    return fallback;
  }
}

}  // namespace

EtaRobotParams load_eta_robot_params(const std::string & yaml_path)
{
  EtaRobotParams params;
  if (yaml_path.empty()) {
    return params;
  }

  YAML::Node root;
  try {
    root = YAML::LoadFile(yaml_path);
  } catch (const std::exception &) {
    return params;
  }

  const YAML::Node eta = root["eta_robot"] ? root["eta_robot"] : root;
  if (!eta || !eta.IsMap()) {
    return params;
  }

  params.operating_linear_speed_mps =
    std::max(0.05, yaml_double(eta, "operating_linear_speed_mps", params.operating_linear_speed_mps));
  params.linear_accel_mps2 =
    std::max(0.01, yaml_double(eta, "linear_accel_mps2", params.linear_accel_mps2));
  params.linear_decel_mps2 =
    std::max(0.01, yaml_double(eta, "linear_decel_mps2", params.linear_decel_mps2));
  params.angular_speed_max_rps =
    std::max(0.01, yaml_double(eta, "angular_speed_max_rps", params.angular_speed_max_rps));
  params.angular_accel_rps2 =
    std::max(0.01, yaml_double(eta, "angular_accel_rps2", params.angular_accel_rps2));

  if (eta["curve_motion"] && eta["curve_motion"].IsMap()) {
    const YAML::Node curve = eta["curve_motion"];
    params.curve_linear_speed_mps =
      std::max(0.05, yaml_double(curve, "curve_linear_speed_mps", params.curve_linear_speed_mps));
    params.curve_radius_threshold_m =
      yaml_double(curve, "curve_radius_threshold_m", params.curve_radius_threshold_m);
    params.heading_min_deg =
      yaml_double(curve, "heading_min_deg", params.heading_min_deg);
    params.heading_max_deg =
      yaml_double(curve, "heading_max_deg", params.heading_max_deg);
    params.heading_change_threshold_deg =
      yaml_double(curve, "heading_change_threshold_deg", params.heading_change_threshold_deg);
  }

  if (eta["curve_approach"] && eta["curve_approach"].IsMap()) {
    const YAML::Node approach = eta["curve_approach"];
    params.approach_start_distance_m =
      yaml_double(approach, "approach_start_distance_m", params.approach_start_distance_m);
    params.curve_entry_distance_m =
      yaml_double(approach, "curve_entry_distance_m", params.curve_entry_distance_m);
    params.required_curve_speed_mps =
      std::max(0.05, yaml_double(approach, "required_curve_speed_mps", params.required_curve_speed_mps));
  }

  if (eta["stop_decel_table"] && eta["stop_decel_table"].IsSequence()) {
    for (const auto & row : eta["stop_decel_table"]) {
      if (!row || !row.IsMap()) {
        continue;
      }
      const double speed = yaml_double(row, "speed_mps", -1.0);
      const double dist = yaml_double(row, "distance_m", -1.0);
      if (speed >= 0.0 && dist >= 0.0) {
        params.stop_decel_table.emplace_back(speed, dist);
      }
    }
  }
  if (params.stop_decel_table.empty()) {
    params.stop_decel_table = {
      {0.5, 3.0}, {1.0, 4.0}, {1.5, 5.0}, {2.0, 7.0}};
  }

  if (eta["robot_dimensions"] && eta["robot_dimensions"].IsMap()) {
    const YAML::Node dim = eta["robot_dimensions"];
    params.robot_length_m = yaml_double(dim, "length_m", params.robot_length_m);
    params.robot_width_m = yaml_double(dim, "width_m", params.robot_width_m);
    params.robot_height_m = yaml_double(dim, "height_m", params.robot_height_m);
  }

  params.loaded = true;
  return params;
}

const EtaRobotParams & cached_eta_robot_params()
{
  static const EtaRobotParams params = [] {
    std::vector<std::string> candidates = {
      "install/execution/share/execution/config/eta_robot_params.yaml",
      "src/execution/config/eta_robot_params.yaml",
    };
    // Also search each AMENT_PREFIX_PATH entry for share/execution/...
    if (const char * prefixes = std::getenv("AMENT_PREFIX_PATH")) {
      std::stringstream ss(prefixes);
      std::string prefix;
      while (std::getline(ss, prefix, ':')) {
        if (!prefix.empty()) {
          candidates.push_back(
            prefix + "/share/execution/config/eta_robot_params.yaml");
        }
      }
    }
    for (const auto & path : candidates) {
      std::ifstream in(path);
      if (!in.good()) {
        continue;
      }
      auto p = load_eta_robot_params(path);
      if (p.loaded) {
        return p;
      }
    }
    return EtaRobotParams{};
  }();
  return params;
}

}  // namespace monitor
