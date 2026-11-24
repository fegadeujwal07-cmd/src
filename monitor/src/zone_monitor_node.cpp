#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "monitor/eta_path_calculator.hpp"
#include "monitor/eta_robot_params.hpp"
#include "monitor/msg/monitored_task_details.hpp"
#include "monitor/msg/robot_zone_status.hpp"
#include "monitor/msg/zone_catalog.hpp"
#include "monitor/nav_graph_store.hpp"
#include "monitor/zone_tracker.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rmf_building_map_msgs/msg/graph.hpp"
#include "rmf_fleet_msgs/msg/fleet_state.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "yaml-cpp/yaml.h"

namespace monitor
{
namespace
{

struct EtaTiming
{
  /// Absolute estimated finish time (RMF unix_millis_finish_time of active phase or task).
  std::optional<int64_t> finish_unix_ms;
  /// Remaining estimate snapshot from RMF (used once to freeze fixed eta).
  /// Remaining estimate snapshot from RMF + when it was observed (for countdown).
  std::optional<int64_t> estimate_millis;
  std::chrono::steady_clock::time_point estimate_observed_at{};
};

/// Per-robot fixed ETA plus how far the robot has walked along that path.
struct EtaRuntime
{
  double fixed_sec{0.0};
  bool fixed_set{false};
  double full_path_m{0.0};
  double hold_sec{0.0};
  std::optional<std::pair<double, double>> last_pose;
  std::chrono::steady_clock::time_point last_sample{};
  std::chrono::steady_clock::time_point last_move_at{};
  /// Furthest segment the robot has reached. Later poses cannot move this backward.
  bool progress_valid{false};
  std::size_t progress_seg{0};
  double progress_t{0.0};
  std::size_t path_points{0};
  double path_end_x{0.0};
  double path_end_y{0.0};
  double full_model_sec{0.0};
  bool full_model_valid{false};
  /// Reach time at each path point, scaled so the last point is the saved task time.
  std::vector<double> reach_sec;
  /// A stop adds time to the path still ahead. It fades as the robot covers that path.
  struct PathDelay
  {
    double seconds{0.0};
    double meters_left{0.0};
    bool open{false};
  };
  std::vector<PathDelay> delays;
  double hold_applied{0.0};
  bool stopped{false};
};

constexpr double kStopMoveEpsM = 0.05;
constexpr double kStopGraceSec = 0.4;
/// Ignore a path match farther than this so a later loop cannot skip the robot ahead.
constexpr double kMaxPathMatchM = 3.0;

struct ParsedPath
{
  std::vector<std::pair<double, double>> xy;
  /// Seconds from the first point, taken from the plan. Empty when the path
  /// has no arrival times.
  std::vector<double> t_from_start;
};

ParsedPath parse_planned_path(const monitor::msg::MonitoredTaskDetails & task)
{
  ParsedPath out;
  if (!task.planned_path.empty()) {
    out.xy.reserve(task.planned_path.size());
    std::vector<double> times;
    bool saw_time = false;
    for (const auto & s : task.planned_path) {
      std::stringstream ss(s);
      std::string part;
      std::vector<std::string> parts;
      while (std::getline(ss, part, ',')) {
        parts.push_back(part);
      }
      if (parts.size() < 2) {
        continue;
      }
      try {
        out.xy.emplace_back(std::stod(parts[0]), std::stod(parts[1]));
        if (parts.size() >= 4) {
          times.push_back(std::stod(parts[3]));
          saw_time = true;
        }
      } catch (...) {
      }
    }
    if (saw_time && times.size() == out.xy.size() &&
      !times.empty() && times.back() - times.front() > 0.5)
    {
      const double t0 = times.front();
      out.t_from_start.reserve(times.size());
      for (const double t : times) {
        out.t_from_start.push_back(std::max(0.0, t - t0));
      }
    }
    if (!out.xy.empty()) {
      return out;
    }
  }
  if (!task.planned_path_x.empty() &&
    task.planned_path_x.size() == task.planned_path_y.size())
  {
    out.xy.reserve(task.planned_path_x.size());
    for (std::size_t i = 0; i < task.planned_path_x.size(); ++i) {
      out.xy.emplace_back(task.planned_path_x[i], task.planned_path_y[i]);
    }
  }
  return out;
}

std::vector<std::pair<double, double>> parse_path_points(
  const monitor::msg::MonitoredTaskDetails & task)
{
  return parse_planned_path(task).xy;
}

double polyline_length_m(const std::vector<std::pair<double, double>> & path)
{
  double len = 0.0;
  for (std::size_t i = 1; i < path.size(); ++i) {
    len += std::hypot(
      path[i].first - path[i - 1].first,
      path[i].second - path[i - 1].second);
  }
  return len;
}

struct PathProgress
{
  bool valid{false};
  std::size_t seg{0};
  double t{0.0};
};

/// Meters still ahead of the robot along the path.
double meters_ahead(
  const std::vector<std::pair<double, double>> & path,
  const PathProgress & cursor)
{
  if (!cursor.valid || path.size() < 2 || cursor.seg + 1 >= path.size()) {
    return 0.0;
  }
  const double seg_len = std::hypot(
    path[cursor.seg + 1].first - path[cursor.seg].first,
    path[cursor.seg + 1].second - path[cursor.seg].second);
  double left = std::max(0.0, (1.0 - cursor.t) * seg_len);
  for (std::size_t i = cursor.seg + 1; i + 1 < path.size(); ++i) {
    left += std::hypot(
      path[i + 1].first - path[i].first,
      path[i + 1].second - path[i].second);
  }
  return left;
}

/// Closest point on one path piece. t is 0 at the start of the piece and 1 at the end.
PathProgress project_on_segment(
  double robot_x,
  double robot_y,
  const std::vector<std::pair<double, double>> & path,
  std::size_t seg,
  double * dist_m)
{
  PathProgress out;
  out.seg = seg;
  const double ax = path[seg].first;
  const double ay = path[seg].second;
  const double bx = path[seg + 1].first;
  const double by = path[seg + 1].second;
  const double abx = bx - ax;
  const double aby = by - ay;
  const double ab2 = abx * abx + aby * aby;
  double t = 0.0;
  if (ab2 > 1e-12) {
    t = ((robot_x - ax) * abx + (robot_y - ay) * aby) / ab2;
    t = std::clamp(t, 0.0, 1.0);
  }
  const double px = ax + t * abx;
  const double py = ay + t * aby;
  out.t = t;
  if (dist_m != nullptr) {
    *dist_m = std::hypot(robot_x - px, robot_y - py);
  }
  return out;
}

/// Move progress only forward. A later loop that passes near the robot cannot rewind it.
PathProgress advance_path_progress(
  double robot_x,
  double robot_y,
  const std::vector<std::pair<double, double>> & path,
  PathProgress cursor)
{
  if (path.size() < 2) {
    return cursor;
  }
  const std::size_t begin = cursor.valid ? cursor.seg : 0;
  bool found = false;
  PathProgress best;
  double best_dist = std::numeric_limits<double>::infinity();
  for (std::size_t i = begin; i + 1 < path.size(); ++i) {
    double dist = 0.0;
    PathProgress proj = project_on_segment(robot_x, robot_y, path, i, &dist);
    if (cursor.valid && i == cursor.seg && proj.t + 1e-3 < cursor.t) {
      continue;
    }
    if (dist < best_dist) {
      best_dist = dist;
      best = proj;
      best.valid = true;
      found = true;
    }
  }
  if (!found || best_dist > kMaxPathMatchM) {
    return cursor;
  }
  return best;
}

std::optional<int64_t> yaml_optional_int64(const YAML::Node & node, const char * key)
{
  if (!node || !node.IsMap() || !node[key] || node[key].IsNull()) {
    return std::nullopt;
  }
  try {
    return node[key].as<int64_t>();
  } catch (...) {
    try {
      return static_cast<int64_t>(node[key].as<double>());
    } catch (...) {
      return std::nullopt;
    }
  }
}

std::string yaml_string(const YAML::Node & node, const char * key, const std::string & fallback = "")
{
  if (!node || !node.IsMap() || !node[key] || node[key].IsNull()) {
    return fallback;
  }
  try {
    return node[key].as<std::string>();
  } catch (...) {
    return fallback;
  }
}
/// Match [itinerary] RMF core ETA style: "1 min 44 s (104.4 s)".
/// Destination settle by cruise speed. Same table as the planner (straight
/// and curve paths): 0.5→5 s, 1.0→8 s, 1.5→18 s, 2.0→21 s.
double linear_align_sec_for_speed(double speed_mps)
{
  const std::vector<std::pair<double, double>> table = {
    {0.5, 5.0}, {1.0, 8.0}, {1.5, 18.0}, {2.0, 21.0}};
  if (speed_mps <= 1e-6) {
    return 0.0;
  }
  if (speed_mps <= table.front().first) {
    const double s0 = std::max(table.front().first, 1e-6);
    return table.front().second * (speed_mps / s0);
  }
  for (std::size_t i = 0; i + 1 < table.size(); ++i) {
    const double s0 = table[i].first;
    const double s1 = table[i + 1].first;
    if (speed_mps <= s1 && s1 > s0) {
      const double t = (speed_mps - s0) / (s1 - s0);
      return table[i].second + t * (table[i + 1].second - table[i].second);
    }
  }
  return table.back().second;
}

std::string format_eta_mins(double minutes)
{
  if (minutes < 0.0) {
    minutes = 0.0;
  }
  const double total_s = minutes * 60.0;
  const int eta_min = static_cast<int>(total_s) / 60;
  const int eta_sec = static_cast<int>(total_s + 0.5) % 60;

  std::ostringstream oss;
  oss << eta_min << " min " << eta_sec << " s ("
      << std::fixed << std::setprecision(1) << total_s << " s)";
  return oss.str();
}

/// Countdown, then the time already used. The second number starts at 0.
std::string format_eta_update_pair(double remain_sec, double elapsed_sec)
{
  if (remain_sec < 0.0) {
    remain_sec = 0.0;
  }
  if (elapsed_sec < 0.0) {
    elapsed_sec = 0.0;
  }
  return format_eta_mins(remain_sec / 60.0) + " (" + format_eta_mins(elapsed_sec / 60.0) + ")";
}

int64_t now_unix_millis()
{
  using namespace std::chrono;
  return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}

/// Extract active-phase (pickup/drop) timing; fall back to task-level (patrol destination).
EtaTiming timing_from_task_state(const YAML::Node & task_state)
{
  EtaTiming timing;
  timing.estimate_observed_at = std::chrono::steady_clock::now();

  const auto active = yaml_optional_int64(task_state, "active");
  const YAML::Node phases = task_state["phases"];
  if (active.has_value() && phases && phases.IsMap()) {
    const std::string active_key = std::to_string(*active);
    YAML::Node phase_node;
    if (phases[active_key]) {
      phase_node = phases[active_key];
    } else if (phases[*active]) {
      phase_node = phases[*active];
    }
    if (phase_node && phase_node.IsMap()) {
      timing.finish_unix_ms = yaml_optional_int64(phase_node, "unix_millis_finish_time");
      timing.estimate_millis = yaml_optional_int64(phase_node, "estimate_millis");
    }
  }

  if (!timing.finish_unix_ms.has_value()) {
    timing.finish_unix_ms = yaml_optional_int64(task_state, "unix_millis_finish_time");
  }
  if (!timing.estimate_millis.has_value()) {
    timing.estimate_millis = yaml_optional_int64(task_state, "estimate_millis");
  }
  return timing;
}

}  // namespace

/// Zone monitor: subscribes to task + fleet state, publishes /robot_zone_status,
/// /zone_catalog, filtered /nav_graphs1, and RViz /zone_markers (waypoints/lanes/labels; no borders).
/// eta = fixed plan time, saved once. eta_update follows the robot pose along
/// that same calculated path time. Passing a coordinate reduces the live time
/// by that coordinate's reach time. A stop adds its seconds to the path still
/// ahead, and those seconds come down as the robot covers the rest. It reaches
/// 0 when the robot has reached the last point.

class ZoneMonitorNode : public rclcpp::Node
{
public:
  ZoneMonitorNode()
  : Node("zone_monitor")
  {
    declare_parameter<std::string>(
      "source_yaml_path",
      "install/execution/share/execution/config/nav_graphs/0.yaml");
    declare_parameter<std::string>(
      "eta_params_path",
      "install/execution/share/execution/config/eta_robot_params.yaml");
    declare_parameter<std::string>("level", "L1");
    declare_parameter<std::string>("fleet_states_topic", "/fleet_states");
    declare_parameter<std::string>("monitored_task_topic", "/Monitored_task_details");
    declare_parameter<std::string>("zone_status_topic", "/robot_zone_status");
    declare_parameter<bool>("publish_robot_zone_status", true);
    declare_parameter<bool>("publish_idle_robot_zones", true);
    declare_parameter<double>("publish_rate_hz", 1.0);
    declare_parameter<bool>("publish_nav_graphs1", true);
    declare_parameter<std::string>("nav_graphs1_topic", "/nav_graphs1");
    declare_parameter<std::string>("graph_name", "amr_fleet");
    declare_parameter<bool>("require_active_zones", true);
    declare_parameter<bool>("publish_zone_markers", true);
    declare_parameter<std::string>("zone_markers_topic", "/zone_markers");
    declare_parameter<std::string>("marker_frame_id", "map");
    declare_parameter<bool>("publish_zone_catalog", true);
    declare_parameter<std::string>("zone_catalog_topic", "/zone_catalog");

    const std::string yaml_path = get_parameter("source_yaml_path").as_string();
    const std::string eta_params_path = get_parameter("eta_params_path").as_string();
    eta_params_ = load_eta_robot_params(eta_params_path);
    if (!eta_params_.loaded) {
      RCLCPP_WARN(
        get_logger(),
        "ETA speeds not loaded from '%s'; eta_update uses built-in defaults",
        eta_params_path.c_str());
    } else {
      RCLCPP_INFO(
        get_logger(),
        "ETA speeds loaded from '%s' (straight %.2f m/s, curve %.2f m/s)",
        eta_params_path.c_str(),
        eta_params_.operating_linear_speed_mps,
        eta_params_.curve_linear_speed_mps);
    }
    level_ = get_parameter("level").as_string();
    monitored_task_input_topic_ = get_parameter("monitored_task_topic").as_string();
    publish_robot_zone_status_ = get_parameter("publish_robot_zone_status").as_bool();
    publish_idle_robot_zones_ = get_parameter("publish_idle_robot_zones").as_bool();
    publish_nav_graphs1_ = get_parameter("publish_nav_graphs1").as_bool();
    graph_name_ = get_parameter("graph_name").as_string();
    require_active_zones_ = get_parameter("require_active_zones").as_bool();
    publish_zone_markers_ = get_parameter("publish_zone_markers").as_bool();
    marker_frame_id_ = get_parameter("marker_frame_id").as_string();
    publish_zone_catalog_ = get_parameter("publish_zone_catalog").as_bool();

    // Single YAML load shared by zone tracking, /nav_graphs1, /zone_markers,
    // and /zone_catalog (for zone_manager — no second 0.yaml import).
    if (!nav_store_.load_map(yaml_path, level_)) {
      RCLCPP_ERROR(
        get_logger(),
        "Failed to load nav graph from '%s' (level=%s). Zone tracking disabled.",
        yaml_path.c_str(), level_.c_str());
    } else if (!tracker_.load_from_store(nav_store_)) {
      RCLCPP_ERROR(
        get_logger(),
        "Nav graph loaded but zone tracker could not adopt the shared cache.");
    } else {
      RCLCPP_INFO(
        get_logger(),
        "Shared nav graph loaded once from '%s' (level=%s): vertices=%zu lanes=%zu zones=%zu",
        yaml_path.c_str(), level_.c_str(),
        nav_store_.vertices().size(), nav_store_.lanes().size(), nav_store_.zones().size());
    }

    constexpr int kQosDepth = 200;

    if (publish_robot_zone_status_) {
      zone_pub_ = create_publisher<monitor::msg::RobotZoneStatus>(
        get_parameter("zone_status_topic").as_string(), kQosDepth);
    }

    if (publish_nav_graphs1_) {
      auto graph_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
      nav_graphs1_pub_ = create_publisher<rmf_building_map_msgs::msg::Graph>(
        get_parameter("nav_graphs1_topic").as_string(), graph_qos);
    }

    if (publish_zone_markers_) {
      auto marker_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
      zone_markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        get_parameter("zone_markers_topic").as_string(), marker_qos);
    }

    if (publish_zone_catalog_) {
      auto catalog_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
      zone_catalog_pub_ = create_publisher<monitor::msg::ZoneCatalog>(
        get_parameter("zone_catalog_topic").as_string(), catalog_qos);
      publish_zone_catalog_from_store(yaml_path);
    }

    fleet_sub_ = create_subscription<rmf_fleet_msgs::msg::FleetState>(
      get_parameter("fleet_states_topic").as_string(),
      kQosDepth,
      std::bind(&ZoneMonitorNode::on_fleet_state, this, std::placeholders::_1));

    task_sub_ = create_subscription<monitor::msg::MonitoredTaskDetails>(
      monitored_task_input_topic_,
      kQosDepth,
      std::bind(&ZoneMonitorNode::on_monitored_task, this, std::placeholders::_1));

    auto task_state_qos = rclcpp::QoS(rclcpp::KeepLast(20));
    task_state_qos.reliable();
    task_state_qos.transient_local();
    task_state_sub_ = create_subscription<std_msgs::msg::String>(
      "/rmf_task_state_updates",
      task_state_qos,
      std::bind(&ZoneMonitorNode::on_task_state_update, this, std::placeholders::_1));

    const double rate_hz = std::max(get_parameter("publish_rate_hz").as_double(), 0.1);
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate_hz),
      std::bind(&ZoneMonitorNode::on_publish_timer, this));

    const std::string zone_status_topic = get_parameter("zone_status_topic").as_string();
    const std::string nav_graphs1_topic = get_parameter("nav_graphs1_topic").as_string();
    const std::string zone_markers_topic = get_parameter("zone_markers_topic").as_string();
    const std::string zone_catalog_topic = get_parameter("zone_catalog_topic").as_string();
    RCLCPP_INFO(
      get_logger(),
      "Zone monitor ready (sub: %s, %s, /rmf_task_state_updates | "
      "pub: %s%s%s%s%s%s%s @ %.1f Hz; eta=fixed core, eta_update=fleet motion)",
      get_parameter("fleet_states_topic").as_string().c_str(),
      monitored_task_input_topic_.c_str(),
      zone_status_topic.c_str(),
      publish_nav_graphs1_ ? ", " : "",
      publish_nav_graphs1_ ? nav_graphs1_topic.c_str() : "",
      publish_zone_markers_ ? ", " : "",
      publish_zone_markers_ ? zone_markers_topic.c_str() : "",
      publish_zone_catalog_ ? ", " : "",
      publish_zone_catalog_ ? zone_catalog_topic.c_str() : "",
      rate_hz);
  }

private:
  void publish_zone_catalog_from_store(const std::string & yaml_path)
  {
    if (!zone_catalog_pub_ || !nav_store_.loaded()) {
      return;
    }

    monitor::msg::ZoneCatalog catalog;
    catalog.level = level_;
    catalog.source_yaml_path = yaml_path;
    catalog.zone_ids.reserve(nav_store_.zones().size());
    for (const auto & [zone_id, _] : nav_store_.zones()) {
      (void)_;
      catalog.zone_ids.push_back(zone_id);
    }
    std::sort(catalog.zone_ids.begin(), catalog.zone_ids.end());
    zone_catalog_pub_->publish(catalog);
    RCLCPP_INFO(
      get_logger(),
      "Published /zone_catalog with %zu zone IDs from shared nav graph",
      catalog.zone_ids.size());
  }

  static bool is_terminal_status(const std::string & status)
  {
    std::string key = status;
    std::transform(key.begin(), key.end(), key.begin(), ::tolower);
    return key == "completed" || key == "canceled" || key == "cancelled" || key == "failed";
  }

  static bool is_completed_status(const std::string & status)
  {
    std::string key = status;
    std::transform(key.begin(), key.end(), key.begin(), ::tolower);
    return key == "completed";
  }

  bool zones_differ_from_last_published(
    const std::string & robot_id,
    const ZoneSnapshot & zones) const
  {
    const auto it = last_published_zone_by_robot_.find(robot_id);
    if (it == last_published_zone_by_robot_.end()) {
      return true;
    }
    return it->second != zones;
  }

  bool eta_differ_from_last_published(
    const std::string & robot_id,
    const std::string & eta,
    const std::string & eta_update) const
  {
    const auto it = last_published_eta_by_robot_.find(robot_id);
    if (it == last_published_eta_by_robot_.end()) {
      return !eta.empty() || !eta_update.empty();
    }
    return it->second.first != eta || it->second.second != eta_update;
  }

  /// Freeze fixed eta once from first core estimate_millis (matches itinerary at plan).
  void maybe_cache_fixed_eta(const std::string & robot_id, const EtaTiming & timing)
  {
    auto & rt = eta_runtime_by_robot_[robot_id];
    if (rt.fixed_set) {
      return;
    }
    double sec = 0.0;
    if (timing.estimate_millis.has_value() && *timing.estimate_millis > 0) {
      sec = static_cast<double>(*timing.estimate_millis) / 1000.0;
    } else if (timing.finish_unix_ms.has_value()) {
      sec = std::max(
        0.0,
        static_cast<double>(*timing.finish_unix_ms - now_unix_millis()) / 1000.0);
    }
    if (sec <= 0.0) {
      return;
    }
    rt.fixed_sec = sec;
    rt.fixed_set = true;
  }

  void refresh_path_length(
    const std::string & robot_id,
    const monitor::msg::MonitoredTaskDetails & task)
  {
    const auto path = parse_path_points(task);
    auto & rt = eta_runtime_by_robot_[robot_id];
    const double end_x = path.empty() ? 0.0 : path.back().first;
    const double end_y = path.empty() ? 0.0 : path.back().second;
    const bool path_changed =
      path.size() != rt.path_points ||
      std::abs(end_x - rt.path_end_x) > 0.05 ||
      std::abs(end_y - rt.path_end_y) > 0.05;
    rt.full_path_m = polyline_length_m(path);
    if (!path_changed) {
      return;
    }
    rt.path_points = path.size();
    rt.path_end_x = end_x;
    rt.path_end_y = end_y;
    rt.progress_valid = false;
    rt.progress_seg = 0;
    rt.progress_t = 0.0;
    rt.full_model_valid = false;
    rt.full_model_sec = 0.0;
    rt.reach_sec.clear();
    rt.delays.clear();
    rt.hold_applied = 0.0;
    rt.hold_sec = 0.0;
    rt.stopped = false;
  }

  void update_stop_hold(const std::string & robot_id, double x, double y)
  {
    const auto now = std::chrono::steady_clock::now();
    auto & rt = eta_runtime_by_robot_[robot_id];
    if (!rt.last_pose.has_value()) {
      rt.last_pose = {x, y};
      rt.last_sample = now;
      rt.last_move_at = now;
      return;
    }
    const double dt = std::chrono::duration<double>(now - rt.last_sample).count();
    const double moved = std::hypot(x - rt.last_pose->first, y - rt.last_pose->second);
    if (moved > kStopMoveEpsM && dt > 1e-3) {
      rt.stopped = false;
      rt.last_pose = {x, y};
      rt.last_move_at = now;
    } else {
      const double stopped_s =
        std::chrono::duration<double>(now - rt.last_move_at).count();
      if (stopped_s > kStopGraceSec) {
        rt.stopped = true;
        rt.hold_sec += std::max(0.0, dt);
      }
    }
    rt.last_sample = now;
  }

  std::string fixed_eta_string(const std::string & robot_id) const
  {
    const auto it = eta_runtime_by_robot_.find(robot_id);
    if (it == eta_runtime_by_robot_.end() || !it->second.fixed_set) {
      return "";
    }
    return format_eta_mins(it->second.fixed_sec / 60.0);
  }

  /// Live remaining from the robot pose. The schedule stamps are not used.
  /// Crossing a coordinate reduces the time by that point's reach time.
  /// A stop adds seconds to the path still ahead. 0 only at the last point.
  std::string compute_eta_update(
    const std::string & robot_id,
    const monitor::msg::MonitoredTaskDetails & task)
  {
    auto rt_it = eta_runtime_by_robot_.find(robot_id);
    if (rt_it == eta_runtime_by_robot_.end() || !rt_it->second.fixed_set) {
      return "";
    }
    auto & rt = rt_it->second;

    const auto pose_it = robot_pose_by_id_.find(robot_id);
    const ParsedPath planned = parse_planned_path(task);
    const auto & path = planned.xy;
    if (pose_it == robot_pose_by_id_.end() || path.size() < 2 || rt.full_path_m <= 1e-3) {
      return format_eta_update_pair(rt.fixed_sec + rt.hold_sec, rt.hold_sec);
    }

    const double robot_x = pose_it->second.first;
    const double robot_y = pose_it->second.second;

    PathProgress cursor;
    cursor.valid = rt.progress_valid;
    cursor.seg = rt.progress_seg;
    cursor.t = rt.progress_t;
    cursor = advance_path_progress(robot_x, robot_y, path, cursor);
    rt.progress_valid = cursor.valid;
    rt.progress_seg = cursor.seg;
    rt.progress_t = cursor.t;

    if (!cursor.valid) {
      return format_eta_update_pair(rt.fixed_sec + rt.hold_sec, rt.hold_sec);
    }

    // Reach time of each coordinate. Saved task time includes destination
    // settle (straight or curve); that belongs only on the last point.
    // 90/180 pivot pads are already inside fixed_sec at those turns.
    if (rt.reach_sec.size() != path.size()) {
      const auto raw = path_point_reach_times_sec(path, eta_params_, 0.0);
      rt.reach_sec.assign(raw.size(), 0.0);
      const double span = raw.empty() ? 0.0 : raw.back();
      const double align = std::min(
        linear_align_sec_for_speed(eta_params_.operating_linear_speed_mps),
        std::max(0.0, rt.fixed_sec));
      const double drive = std::max(0.0, rt.fixed_sec - align);
      if (span > 1e-3) {
        for (std::size_t i = 0; i < raw.size(); ++i) {
          rt.reach_sec[i] = drive * (raw[i] / span);
        }
        if (!rt.reach_sec.empty()) {
          rt.reach_sec.back() += align;
        }
        rt.full_model_sec = rt.reach_sec.empty() ? rt.fixed_sec : rt.reach_sec.back();
        rt.full_model_valid = true;
      } else {
        rt.full_model_valid = false;
        rt.full_model_sec = 0.0;
      }
    }

    const double left_m = meters_ahead(path, cursor);

    // A stop adds its seconds to the path still ahead. Later stops keep their
    // own remaining distance, so each delay comes down only as that path is covered.
    const double add = std::max(0.0, rt.hold_sec - rt.hold_applied);
    if (add > 0.0 && left_m > 1e-3) {
      if (rt.delays.empty() || !rt.delays.back().open) {
        EtaRuntime::PathDelay delay;
        delay.meters_left = left_m;
        delay.open = rt.stopped;
        rt.delays.push_back(delay);
      }
      rt.delays.back().seconds += add;
      rt.hold_applied = rt.hold_sec;
    }
    if (!rt.stopped && !rt.delays.empty() && rt.delays.back().open) {
      rt.delays.back().open = false;
    }

    double here = 0.0;
    if (cursor.seg + 1 < rt.reach_sec.size()) {
      const double t0 = rt.reach_sec[cursor.seg];
      const double t1 = rt.reach_sec[cursor.seg + 1];
      here = t0 + cursor.t * (t1 - t0);
    } else if (!rt.reach_sec.empty()) {
      here = rt.reach_sec.back();
    }
    const double end = rt.reach_sec.empty() ? rt.fixed_sec : rt.reach_sec.back();
    double sec = std::max(0.0, end - here);

    // The stop time stays in the time still left, then falls with the robot.
    for (const auto & delay : rt.delays) {
      if (delay.meters_left <= 1e-6) {
        continue;
      }
      const double covered = std::clamp(left_m / delay.meters_left, 0.0, 1.0);
      sec += delay.seconds * covered;
    }

    if (sec <= 1e-3 && left_m <= 1e-3) {
      sec = 0.0;
    }
    // Time already used: path time passed, plus seconds the robot has been stopped.
    // Starts at 0. With no stop it finishes at the fixed eta. A stop keeps climbing,
    // the same way a stopwatch keeps running.
    const double elapsed = here + rt.hold_sec;
    return format_eta_update_pair(sec, elapsed);
  }

  void clear_task_tracking(const std::string & robot_id)
  {
    tracker_.clear_robot_task(robot_id);
    latest_task_by_robot_.erase(robot_id);
    timing_by_robot_id_.erase(robot_id);
    dirty_robot_ids_.erase(robot_id);
    eta_runtime_by_robot_.erase(robot_id);
  }

  void clear_idle_completion_hint(const std::string & robot_id)
  {
    idle_completion_zone_hint_.erase(robot_id);
  }

  /// Publish the last movement-based eta_update before tearing down task state.
  /// Do not force 0 here. 0 is published only when the robot is already at the last point.
  void publish_task_completion(const std::string & robot_id)
  {
    const auto task_it = latest_task_by_robot_.find(robot_id);
    if (task_it == latest_task_by_robot_.end()) {
      return;
    }

    const ZoneSnapshot zones = tracker_.get_robot_zones_at_completion(robot_id);
    if (zones.current_zone.empty()) {
      return;
    }

    const std::string eta = fixed_eta_string(robot_id);
    const std::string eta_update = compute_eta_update(robot_id, task_it->second);
    const bool zones_changed = zones_differ_from_last_published(robot_id, zones);
    const bool eta_changed = eta_differ_from_last_published(robot_id, eta, eta_update);
    if (!zones_changed && !eta_changed) {
      return;
    }

    publish_robot_zone_status(robot_id, zones, eta, eta_update);
    idle_completion_zone_hint_[robot_id] = zones.current_zone;
  }

  void finalize_and_clear_robot(const std::string & robot_id)
  {
    publish_task_completion(robot_id);
    clear_task_tracking(robot_id);
  }

  /// Publish every robot seen on /fleet_states once per timer tick so downstream
  /// consumers always receive fresh data (idle + active), even when pose/zones/ETA
  /// are unchanged.
  void publish_fleet_robots_locked()
  {
    if (!publish_robot_zone_status_) {
      return;
    }

    for (const auto & robot_id : active_robot_ids_) {
      const auto task_it = latest_task_by_robot_.find(robot_id);
      if (task_it != latest_task_by_robot_.end()) {
        const ZoneSnapshot zones = tracker_.get_robot_zones(robot_id);
        if (zones.zone_sequence.empty() && zones.current_zone.empty()) {
          continue;
        }
        const std::string eta = fixed_eta_string(robot_id);
        const std::string eta_update = compute_eta_update(robot_id, task_it->second);
        publish_robot_zone_status(robot_id, zones, eta, eta_update);
      } else if (publish_idle_robot_zones_) {
        const ZoneSnapshot zones = idle_zones_for_robot(robot_id);
        if (zones.current_zone.empty()) {
          continue;
        }
        publish_idle_zone_status(robot_id, zones);
      }
    }
  }

  void publish_robot_zone_status(
    const std::string & robot_id,
    const ZoneSnapshot & zones,
    const std::string & eta,
    const std::string & eta_update)

  {
    if (publish_robot_zone_status_ && zone_pub_) {
      monitor::msg::RobotZoneStatus zone_only;
      zone_only.amr_id = robot_id;
      zone_only.current_zone = zones.current_zone;
      zone_only.rear_zones = zones.rear_zones;
      zone_only.next_zone = zones.next_zone;
      zone_only.zone_sequence = zones.zone_sequence;
      zone_only.eta = eta;
      zone_only.eta_update = eta_update;
      zone_pub_->publish(zone_only);
    }

    last_published_zone_by_robot_[robot_id] = zones;
    last_published_eta_by_robot_[robot_id] = {eta, eta_update};
  }

  void publish_idle_zone_status(
    const std::string & robot_id,
    const ZoneSnapshot & zones)
  {
    publish_robot_zone_status(robot_id, zones, "", "");
  }

  ZoneSnapshot idle_zones_for_robot(const std::string & robot_id)
  {
    ZoneSnapshot zones = tracker_.get_robot_idle_zones(robot_id);
    const auto hint_it = idle_completion_zone_hint_.find(robot_id);
    if (hint_it == idle_completion_zone_hint_.end()) {
      return zones;
    }

    if (zones.current_zone == hint_it->second) {
      idle_completion_zone_hint_.erase(robot_id);
      return zones;
    }

    ZoneSnapshot pinned;
    pinned.current_zone = hint_it->second;
    pinned.rear_zones = zones.rear_zones;
    pinned.zone_sequence = {hint_it->second};
    return pinned;
  }

  void on_fleet_state(const rmf_fleet_msgs::msg::FleetState::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto & robot : msg->robots) {
      const std::string robot_id = ZoneTracker::normalize_robot_id(robot.name);
      if (robot_id.empty()) {
        continue;
      }
      if (!robot.location.level_name.empty() && robot.location.level_name != level_) {
        continue;
      }
      // yaw matters now: the rear zones are found by walking the lane graph
      // backwards from the robot's heading.
      tracker_.update_robot_pose(
        robot_id, robot.location.x, robot.location.y, robot.location.yaw);
      robot_pose_by_id_[robot_id] = {robot.location.x, robot.location.y};
      if (latest_task_by_robot_.count(robot_id) > 0) {
        update_stop_hold(robot_id, robot.location.x, robot.location.y);
      }
      active_robot_ids_.insert(robot_id);

      if (robot.task_id.empty()) {
        if (latest_task_by_robot_.count(robot_id) > 0) {
          finalize_and_clear_robot(robot_id);
        }
        continue;
      }

      clear_idle_completion_hint(robot_id);

      if (latest_task_by_robot_.count(robot_id) == 0) {
        continue;
      }
      // Pose motion changes path-based ETA; always mark dirty for live countdown.
      dirty_robot_ids_.insert(robot_id);
    }
  }

  void on_monitored_task(const monitor::msg::MonitoredTaskDetails::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    const std::string robot_id = ZoneTracker::normalize_robot_id(msg->amr_id);
    if (robot_id.empty()) {
      return;
    }

    if (is_terminal_status(msg->status)) {
      if (!msg->task_id.empty()) {
        timing_by_task_id_.erase(msg->task_id);
      }
      if (is_completed_status(msg->status)) {
        finalize_and_clear_robot(robot_id);
      } else {
        clear_task_tracking(robot_id);
      }
      return;
    }

    const bool has_numeric_path =
      !msg->planned_path_x.empty() &&
      msg->planned_path_x.size() == msg->planned_path_y.size();
    if (!has_numeric_path && msg->planned_path.empty()) {
      return;
    }

    active_robot_ids_.insert(robot_id);

    const auto prev_it = latest_task_by_robot_.find(robot_id);
    if (prev_it != latest_task_by_robot_.end() &&
      !msg->task_id.empty() &&
      !prev_it->second.task_id.empty() &&
      prev_it->second.task_id != msg->task_id)
    {
      eta_runtime_by_robot_.erase(robot_id);
    }

    if (has_numeric_path) {
      std::vector<std::pair<double, double>> points;
      points.reserve(msg->planned_path_x.size());
      for (std::size_t i = 0; i < msg->planned_path_x.size(); ++i) {
        points.emplace_back(msg->planned_path_x[i], msg->planned_path_y[i]);
      }
      tracker_.update_robot_planned_path_xy(robot_id, points);
    } else {
      tracker_.update_robot_planned_path(
        robot_id,
        std::vector<std::string>(msg->planned_path.begin(), msg->planned_path.end()));
    }

    latest_task_by_robot_[robot_id] = *msg;
    refresh_path_length(robot_id, *msg);
    last_published_zone_by_robot_.erase(robot_id);
    dirty_robot_ids_.insert(robot_id);
  }

  void on_task_state_update(const std_msgs::msg::String::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    YAML::Node task_state;
    try {
      task_state = YAML::Load(msg->data);
    } catch (...) {
      return;
    }

    const YAML::Node booking = task_state["booking"];
    const std::string task_id = yaml_string(booking, "id");
    if (task_id.empty()) {
      return;
    }

    const std::string status = yaml_string(task_state, "status");
    const YAML::Node assigned_to = task_state["assigned_to"];
    const std::string robot_id =
      ZoneTracker::normalize_robot_id(yaml_string(assigned_to, "name"));

    if (is_terminal_status(status)) {
      // Task-scoped state belongs to this task alone and always goes.
      timing_by_task_id_.erase(task_id);

      if (robot_id.empty()) {
        return;
      }

      // Robot-scoped state must only be cleared when the task that ended is
      // the one the robot is actually running.
      //
      // This used to clear unconditionally, so cancelling a QUEUED task wiped
      // the tracking for the task the robot was still driving: its current and
      // next zones stopped being published on /robot_zone_status. That is not
      // only a display problem — the zone manager builds its reserved-zone
      // sequences from that topic, and with a robot missing from them
      // select_robots_coming_from_behind() sees fewer than two sequences and
      // returns nothing, so conflict detection silently stopped covering that
      // robot until its next task.
      //
      // The same "is this the robot's current task?" guard is applied further
      // down for non-terminal updates; it simply was not applied here.
      const auto current_it = latest_task_by_robot_.find(robot_id);
      const bool is_current_task =
        current_it == latest_task_by_robot_.end() ||
        current_it->second.task_id.empty() ||
        current_it->second.task_id == task_id;

      if (!is_current_task) {
        RCLCPP_INFO(
          get_logger(),
          "Task [%s] for robot [%s] reached '%s', but the robot is still "
          "running task [%s]; keeping its zone tracking.",
          task_id.c_str(), robot_id.c_str(), status.c_str(),
          current_it->second.task_id.c_str());
        return;
      }

      timing_by_robot_id_.erase(robot_id);
      if (is_completed_status(status)) {
        finalize_and_clear_robot(robot_id);
      } else {
        clear_task_tracking(robot_id);
      }
      return;
    }

    const EtaTiming timing = timing_from_task_state(task_state);
    if (!timing.finish_unix_ms.has_value() && !timing.estimate_millis.has_value()) {
      return;
    }
    timing_by_task_id_[task_id] = timing;
    if (!robot_id.empty()) {
      timing_by_robot_id_[robot_id] = timing;
      maybe_cache_fixed_eta(robot_id, timing);
    }

    if (robot_id.empty()) {
      return;
    }
    const auto task_it = latest_task_by_robot_.find(robot_id);
    if (task_it == latest_task_by_robot_.end()) {
      return;
    }
    if (!task_it->second.task_id.empty() && task_it->second.task_id != task_id) {
      return;
    }
    dirty_robot_ids_.insert(robot_id);
  }

  std::unordered_set<std::string> collect_active_zones_locked() const
  {
    std::unordered_set<std::string> zones;
    for (const auto & [robot_id, snapshot] : last_published_zone_by_robot_) {
      (void)robot_id;
      if (!snapshot.current_zone.empty()) {
        zones.insert(snapshot.current_zone);
      }
      for (const auto & z : snapshot.next_zone) {
        if (!z.empty()) {
          zones.insert(z);
        }
      }
    }
    return zones;
  }

  void maybe_publish_nav_graphs1_locked()
  {
    const bool want_graph = publish_nav_graphs1_ && nav_graphs1_pub_;
    const bool want_markers = publish_zone_markers_ && zone_markers_pub_;
    if ((!want_graph && !want_markers) || !nav_store_.loaded()) {
      return;
    }

    const auto active_zones = collect_active_zones_locked();
    if (active_zones.empty()) {
      if (require_active_zones_) {
        if (!last_nav_graphs1_zones_.empty()) {
          last_nav_graphs1_zones_.clear();
          if (want_markers) {
            visualization_msgs::msg::MarkerArray clear_msg;
            visualization_msgs::msg::Marker delete_all;
            delete_all.header.stamp = now();
            delete_all.header.frame_id = marker_frame_id_;
            delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
            clear_msg.markers.push_back(delete_all);
            zone_markers_pub_->publish(clear_msg);
          }
          RCLCPP_INFO(get_logger(), "No active zones; cleared /nav_graphs1 visualization");
        }
        return;
      }
    }

    if (active_zones == last_nav_graphs1_zones_) {
      return;
    }

    if (want_graph) {
      const auto graph = nav_store_.build_filtered_graph(active_zones, graph_name_);
      if (graph.vertices.empty()) {
        RCLCPP_WARN(
          get_logger(),
          "Filtered /nav_graphs1 empty for %zu active zone(s); not publishing graph",
          active_zones.size());
      } else {
        nav_graphs1_pub_->publish(graph);
        RCLCPP_INFO(
          get_logger(),
          "Published /nav_graphs1 (vertices=%zu, edges=%zu)",
          graph.vertices.size(), graph.edges.size());
      }
    }

    if (want_markers) {
      std::vector<NavGraphStore::ZoneMarkerView> views;
      views.reserve(last_published_zone_by_robot_.size());
      for (const auto & [robot_id, snapshot] : last_published_zone_by_robot_) {
        NavGraphStore::ZoneMarkerView view;
        view.robot_id = robot_id;
        view.current_zone = snapshot.current_zone;
        view.next_zones = snapshot.next_zone;
        const auto pose_it = robot_pose_by_id_.find(robot_id);
        if (pose_it != robot_pose_by_id_.end()) {
          view.x = pose_it->second.first;
          view.y = pose_it->second.second;
        }
        views.push_back(std::move(view));
      }

      const auto markers = nav_store_.build_zone_markers(views, marker_frame_id_, now());
      zone_markers_pub_->publish(markers);
      RCLCPP_INFO(
        get_logger(),
        "Published /zone_markers (%zu markers for %zu robot(s))",
        markers.markers.size(), views.size());
    }

    last_nav_graphs1_zones_ = active_zones;
  }

  void on_publish_timer()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    publish_fleet_robots_locked();
    maybe_publish_nav_graphs1_locked();
  }

  NavGraphStore nav_store_;
  ZoneTracker tracker_;
  EtaRobotParams eta_params_;
  std::string level_;
  std::string monitored_task_input_topic_;
  std::string graph_name_;
  std::string marker_frame_id_{"map"};
  bool publish_robot_zone_status_{true};
  bool publish_idle_robot_zones_{true};
  bool publish_nav_graphs1_{true};
  bool publish_zone_markers_{true};
  bool require_active_zones_{true};
  bool publish_zone_catalog_{true};
  std::mutex mutex_;
  std::unordered_set<std::string> active_robot_ids_;
  std::unordered_set<std::string> dirty_robot_ids_;
  std::unordered_map<std::string, monitor::msg::MonitoredTaskDetails> latest_task_by_robot_;
  std::unordered_map<std::string, ZoneSnapshot> last_published_zone_by_robot_;
  std::unordered_map<std::string, std::pair<std::string, std::string>>
    last_published_eta_by_robot_;
  std::unordered_map<std::string, EtaTiming> timing_by_task_id_;
  std::unordered_map<std::string, EtaTiming> timing_by_robot_id_;
  std::unordered_map<std::string, std::pair<double, double>> robot_pose_by_id_;
  std::unordered_map<std::string, EtaRuntime> eta_runtime_by_robot_;
  std::unordered_map<std::string, std::string> idle_completion_zone_hint_;
  std::unordered_set<std::string> last_nav_graphs1_zones_;

  rclcpp::Publisher<monitor::msg::RobotZoneStatus>::SharedPtr zone_pub_;
  rclcpp::Publisher<monitor::msg::ZoneCatalog>::SharedPtr zone_catalog_pub_;
  rclcpp::Publisher<rmf_building_map_msgs::msg::Graph>::SharedPtr nav_graphs1_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr zone_markers_pub_;
  rclcpp::Subscription<rmf_fleet_msgs::msg::FleetState>::SharedPtr fleet_sub_;
  rclcpp::Subscription<monitor::msg::MonitoredTaskDetails>::SharedPtr task_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr task_state_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace monitor

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<monitor::ZoneMonitorNode>());
  rclcpp::shutdown();
  return 0;
}
