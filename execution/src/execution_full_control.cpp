// C++ port of execution/execution_full_control.py (Python file is READ-ONLY).
// Single source file — all logic ported section-by-section.

#include "execution/execution_full_control.hpp"

#include "rclcpp/executors/single_threaded_executor.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <cctype>
#include <iomanip>
#include <iostream>
#include <limits>
#include <mutex>
#include <optional>
#include <set>
#include <sstream>
#include <vector>

namespace execution
{

namespace
{
std::string utc_iso_timestamp_millis();
}

// =============================================================================
// SECTION 01 — Python lines 60–252: Types + NavGraphParser
// =============================================================================

namespace
{

// Same fingerprint the publisher uses for nodes 1..N: node 0 is the live pose
// and is excluded, positions are quantized to millimetres.
std::size_t route_xy_hash(const std::vector<std::pair<double, double>> & xy)
{
  std::size_t path_hash = 0;
  for (std::size_t i = 1; i < xy.size(); ++i) {
    const long long qx =
      static_cast<long long>(std::llround(xy[i].first * 1000.0));
    const long long qy =
      static_cast<long long>(std::llround(xy[i].second * 1000.0));
    path_hash = path_hash * 1000003u + static_cast<std::size_t>(qx);
    path_hash = path_hash * 1000003u + static_cast<std::size_t>(qy);
  }
  return path_hash;
}

// Python: properties.get(key, default) — key must exist as a map entry.
YAML::Node yaml_map_get(const YAML::Node & properties, const char * key, const YAML::Node & default_value)
{
  if (!properties || !properties.IsMap()) {
    return default_value;
  }
  const YAML::Node value = properties[key];
  if (!value.IsDefined()) {
    return default_value;
  }
  return value;
}

std::string yaml_map_get_string(
  const YAML::Node & properties,
  const char * key,
  const std::string & default_value = "")
{
  const YAML::Node value = yaml_map_get(properties, key, YAML::Node());
  if (!value.IsDefined() || value.IsNull()) {
    return default_value;
  }
  return value.as<std::string>();
}

// Python: properties.get('is_holding_point', False) — stored in a bool field at runtime.
// Values like [4, true] are truthy in Python; mirror that when assigning to bool.
bool yaml_map_get_bool_default_false(const YAML::Node & properties, const char * key)
{
  const YAML::Node value = yaml_map_get(properties, key, YAML::Node(false));
  if (!value.IsDefined() || value.IsNull()) {
    return false;
  }
  if (value.IsScalar()) {
    try {
      return value.as<bool>();
    } catch (...) {
      try {
        return value.as<int>() != 0;
      } catch (...) {
        return false;
      }
    }
  }
  if (value.IsSequence()) {
    return value.size() > 0;
  }
  if (value.IsMap()) {
    return value.size() > 0;
  }
  return false;
}

YAML::Node empty_yaml_map()
{
  return YAML::Node(YAML::NodeType::Map);
}

std::string to_lower_ascii(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(), ::tolower);
  return value;
}

// Python: robot_data.get(key, default) + .lower() where applicable.
std::string py_map_get_string(const AnyMap & data, const std::string & key, const std::string & default_value)
{
  const auto it = data.find(key);
  if (it == data.end()) {
    return default_value;
  }
  return it->second.as<std::string>();
}

bool py_load_status_in_2_4(const AnyMap & data, const std::string & key, int * out_value = nullptr)
{
  const auto it = data.find(key);
  if (it == data.end()) {
    return false;
  }
  const YAML::Node & value = it->second;
  try {
    const int v = value.as<int>();
    if (v == 2 || v == 4) {
      if (out_value) {
        *out_value = v;
      }
      return true;
    }
  } catch (...) {
    try {
      const double v = value.as<double>();
      if (v == 2.0 || v == 4.0) {
        if (out_value) {
          *out_value = static_cast<int>(v);
        }
        return true;
      }
    } catch (...) {
    }
  }
  return false;
}

bool py_operating_mode_in_0_1(const AnyMap & data, const std::string & key, int * out_value = nullptr)
{
  const auto it = data.find(key);
  if (it == data.end() || it->second.IsNull()) {
    return false;
  }
  const YAML::Node & value = it->second;
  try {
    const int v = value.as<int>();
    if (v == 0 || v == 1) {
      if (out_value) {
        *out_value = v;
      }
      return true;
    }
  } catch (...) {
    try {
      const double v = value.as<double>();
      if (v == 0.0 || v == 1.0) {
        if (out_value) {
          *out_value = static_cast<int>(v);
        }
        return true;
      }
    } catch (...) {
    }
  }
  return false;
}

double anymap_get_double(const AnyMap & data, const std::string & key, double default_value = 0.0)
{
  const auto it = data.find(key);
  if (it == data.end()) {
    return default_value;
  }
  try {
    return it->second.as<double>();
  } catch (...) {
    return default_value;
  }
}

std::string anymap_get_string(
  const AnyMap & data, const std::string & key, const std::string & default_value = "")
{
  const auto it = data.find(key);
  if (it == data.end()) {
    return default_value;
  }
  try {
    return it->second.as<std::string>();
  } catch (...) {
    return default_value;
  }
}

bool anymap_get_bool(const AnyMap & data, const std::string & key, bool default_value = false)
{
  const auto it = data.find(key);
  if (it == data.end()) {
    return default_value;
  }
  try {
    return it->second.as<bool>();
  } catch (...) {
    try {
      return it->second.as<int>() != 0;
    } catch (...) {
      return default_value;
    }
  }
}

/// ECBS WAIT points are marked traffic_hold=true. The AMR must remain on that
/// waypoint (and keep the RMF task active) until a later PathRequest clears the
/// hold and extends release past it.
bool anymap_is_traffic_hold(const AnyMap & data)
{
  return anymap_get_bool(data, "traffic_hold", false);
}

std::optional<std::string> py_map_get_optional_string(const AnyMap & data, const std::string & key)
{
  const auto it = data.find(key);
  if (it == data.end() || it->second.IsNull()) {
    return std::nullopt;
  }
  try {
    return it->second.as<std::string>();
  } catch (...) {
    try {
      return std::to_string(it->second.as<int>());
    } catch (...) {
      return std::nullopt;
    }
  }
}

std::optional<int> py_map_get_optional_int(const AnyMap & data, const std::string & key)
{
  const auto it = data.find(key);
  if (it == data.end() || it->second.IsNull()) {
    return std::nullopt;
  }
  try {
    return it->second.as<int>();
  } catch (...) {
    try {
      return static_cast<int>(it->second.as<double>());
    } catch (...) {
      try {
        return std::stoi(it->second.as<std::string>());
      } catch (...) {
        return std::nullopt;
      }
    }
  }
}

bool py_map_get_bool(const AnyMap & data, const std::string & key, bool default_value = false)
{
  const auto it = data.find(key);
  if (it == data.end()) {
    return default_value;
  }
  if (it->second.IsNull()) {
    return default_value;
  }
  try {
    return it->second.as<bool>();
  } catch (...) {
    try {
      return it->second.as<int>() != 0;
    } catch (...) {
      return default_value;
    }
  }
}

std::string event_scalar_to_string(const YAML::Node & node)
{
  if (!node.IsDefined() || node.IsNull()) {
    return "None";
  }
  try {
    return node.as<std::string>();
  } catch (...) {
    try {
      return std::to_string(node.as<int>());
    } catch (...) {
      try {
        return std::to_string(node.as<double>());
      } catch (...) {
        return "None";
      }
    }
  }
}

double current_epoch_seconds()
{
  return std::chrono::duration<double>(
    std::chrono::system_clock::now().time_since_epoch()).count();
}

void trim_processed_event_set(std::set<std::string> & events, std::size_t keep_count = 500)
{
  if (events.size() <= 1000) {
    return;
  }
  std::vector<std::string> as_list(events.begin(), events.end());
  if (as_list.size() > keep_count) {
    as_list.erase(as_list.begin(), as_list.end() - static_cast<std::ptrdiff_t>(keep_count));
  }
  events = std::set<std::string>(as_list.begin(), as_list.end());
}

std::string map_keys_join(const std::map<std::string, AnyMap> & data)
{
  std::string result;
  for (const auto & [key, _] : data) {
    if (!result.empty()) {
      result += ", ";
    }
    result += key;
  }
  return result;
}

double yaml_node_double(const YAML::Node & node, double default_value = 0.0)
{
  if (!node.IsDefined() || node.IsNull()) {
    return default_value;
  }
  try {
    return node.as<double>();
  } catch (...) {
    return default_value;
  }
}

std::pair<double, double> path_point_xy(const YAML::Node & point)
{
  if (point.IsMap()) {
    return {yaml_node_double(point["x"]), yaml_node_double(point["y"])};
  }
  return {yaml_node_double(point["x"]), yaml_node_double(point["y"])};
}

std::optional<int> parse_load_status_int(const YAML::Node & status_raw)
{
  if (!status_raw.IsDefined() || status_raw.IsNull()) {
    return std::nullopt;
  }
  try {
    return status_raw.as<int>();
  } catch (...) {
    try {
      return static_cast<int>(status_raw.as<double>());
    } catch (...) {
      try {
        return std::stoi(status_raw.as<std::string>());
      } catch (...) {
        return std::nullopt;
      }
    }
  }
}

std::string utc_iso_timestamp_millis()
{
  const auto now = std::chrono::system_clock::now();
  const auto ms =
    std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch());
  const std::time_t t = std::chrono::system_clock::to_time_t(now);
  std::tm tm = *std::gmtime(&t);
  std::ostringstream oss;
  oss << std::put_time(&tm, "%FT%T") << "." << std::setfill('0') << std::setw(3) <<
    (ms.count() % 1000) << "Z";
  return oss.str();
}

YAML::Node anymap_get_yaml_node(const AnyMap & data, const std::string & key)
{
  const auto it = data.find(key);
  if (it == data.end()) {
    return YAML::Node();
  }
  return it->second;
}

AnyMap yaml_map_to_anymap(const YAML::Node & node)
{
  AnyMap out;
  if (!node.IsMap()) {
    return out;
  }
  for (const auto & kv : node) {
    out[kv.first.as<std::string>()] = kv.second;
  }
  return out;
}

double nested_coord_double(const YAML::Node & parent, const char * section, const char * axis)
{
  if (!parent.IsMap()) {
    return 0.0;
  }
  const YAML::Node section_node = parent[section];
  if (!section_node.IsMap()) {
    return 0.0;
  }
  return yaml_node_double(section_node[axis]);
}

double anymap_nested_coord(const AnyMap & parent, const char * section, const char * axis)
{
  const YAML::Node section_node = anymap_get_yaml_node(parent, section);
  if (!section_node.IsMap()) {
    return 0.0;
  }
  return yaml_node_double(section_node[axis]);
}

bool yaml_coords_has_xy(const YAML::Node & coords)
{
  return coords.IsMap() && coords["x"].IsDefined() && coords["y"].IsDefined();
}

std::string yaml_node_as_string(const YAML::Node & node, const std::string & default_value = "")
{
  if (!node.IsDefined() || node.IsNull()) {
    return default_value;
  }
  try {
    return node.as<std::string>();
  } catch (...) {
    try {
      return std::to_string(node.as<int>());
    } catch (...) {
      try {
        return std::to_string(node.as<double>());
      } catch (...) {
        return default_value;
      }
    }
  }
}

std::string string_map_to_log_dict(const std::map<std::string, std::string> & data)
{
  std::ostringstream oss;
  oss << "{";
  bool first = true;
  for (const auto & [key, value] : data) {
    if (!first) {
      oss << ", ";
    }
    first = false;
    oss << "'" << key << "': '" << value << "'";
  }
  oss << "}";
  return oss.str();
}

bool yaml_map_has_key(const YAML::Node & map_node, const char * key)
{
  return map_node.IsMap() && map_node[key].IsDefined() && !map_node[key].IsNull();
}

// Safe sequence lookup: const Node::operator[] on a missing key returns Invalid,
// and IsSequence()/size() on Invalid throws InvalidNode. Always check IsDefined first.
YAML::Node yaml_safe_sequence(const YAML::Node & parent, const char * key)
{
  if (!parent.IsMap()) {
    return YAML::Node(YAML::NodeType::Sequence);
  }
  const YAML::Node child = parent[key];
  if (!child.IsDefined() || child.IsNull()) {
    return YAML::Node(YAML::NodeType::Sequence);
  }
  if (!child.IsSequence()) {
    return YAML::Node(YAML::NodeType::Sequence);
  }
  return YAML::Clone(child);
}

YAML::Node yaml_safe_map(const YAML::Node & parent, const char * key)
{
  if (!parent.IsMap()) {
    return YAML::Node(YAML::NodeType::Map);
  }
  const YAML::Node child = parent[key];
  if (!child.IsDefined() || child.IsNull() || !child.IsMap()) {
    return YAML::Node(YAML::NodeType::Map);
  }
  return child;
}

std::string infer_category_from_task_request(const YAML::Node & task_req)
{
  if (!task_req.IsMap()) {
    return "";
  }
  std::string category = to_lower_ascii(yaml_map_get_string(task_req, "category"));
  const YAML::Node description = yaml_safe_map(task_req, "description");

  if (category == "compose") {
    const std::string inner = to_lower_ascii(yaml_map_get_string(description, "category"));
    if (inner == "multi_delivery" || inner.find("delivery") != std::string::npos) {
      return "delivery";
    }
    if (yaml_safe_map(description, "pickup").size() > 0 ||
      yaml_safe_map(description, "dropoff").size() > 0)
    {
      return "delivery";
    }
    if (yaml_safe_sequence(description, "places").size() > 0) {
      return "patrol";
    }
    if (!yaml_map_get_string(description, "place").empty()) {
      return "drop";
    }
    return category;
  }

  if (!category.empty()) {
    return category;
  }

  if (yaml_safe_map(description, "pickup").size() > 0 ||
    yaml_safe_map(description, "dropoff").size() > 0)
  {
    return "delivery";
  }
  if (yaml_safe_sequence(description, "places").size() > 0) {
    return "patrol";
  }
  if (!yaml_map_get_string(description, "place").empty()) {
    return "drop";
  }
  return "";
}

YAML::Node resolve_delivery_leg(
  const YAML::Node & detail,
  const YAML::Node & booking_desc,
  const YAML::Node & pending_desc,
  const char * leg)
{
  YAML::Node node = yaml_safe_map(detail, leg);
  if (node.size() == 0) {
    node = yaml_safe_map(booking_desc, leg);
  }
  if (node.size() == 0) {
    node = yaml_safe_map(pending_desc, leg);
  }
  if (node.size() > 0) {
    return YAML::Clone(node);
  }
  return YAML::Node(YAML::NodeType::Map);
}

YAML::Node string_vector_to_yaml_sequence(const std::vector<std::string> & items)
{
  YAML::Node seq(YAML::NodeType::Sequence);
  for (const auto & item : items) {
    seq.push_back(item);
  }
  return seq;
}

bool target_position_is_empty(const AnyMap & target)
{
  if (target.empty()) {
    return true;
  }
  return target.find("x") == target.end() || target.find("y") == target.end();
}

std::string robot_status_to_string_local(RobotStatus status)
{
  switch (status) {
    case RobotStatus::Idle: return "idle";
    case RobotStatus::Moving: return "moving";
    case RobotStatus::Navigating: return "navigating";
    case RobotStatus::Charging: return "charging";
    case RobotStatus::Error: return "error";
    case RobotStatus::Offline: return "offline";
  }
  return "idle";
}

std::string trim_copy(const std::string & input)
{
  auto is_not_space = [](unsigned char c) { return !std::isspace(c); };
  auto begin = std::find_if(input.begin(), input.end(), is_not_space);
  if (begin == input.end()) {
    return "";
  }
  auto end = std::find_if(input.rbegin(), input.rend(), is_not_space).base();
  return std::string(begin, end);
}

std::string yaml_value_to_json(const YAML::Node & node);

std::string yaml_node_to_json_string(const YAML::Node & node)
{
  if (!node || !node.IsMap()) {
    return "";
  }
  std::ostringstream oss;
  oss << "{";
  bool first = true;
  for (const auto & kv : node) {
    if (!first) {
      oss << ", ";
    }
    first = false;
    oss << '"' << kv.first.as<std::string>() << "\": " << yaml_value_to_json(kv.second);
  }
  oss << "}";
  return oss.str();
}

std::string yaml_value_to_json(const YAML::Node & node)
{
  if (!node || node.IsNull()) {
    return "null";
  }
  if (node.IsMap()) {
    return yaml_node_to_json_string(node);
  }
  if (node.IsSequence()) {
    std::ostringstream oss;
    oss << "[";
    bool first = true;
    for (const auto & item : node) {
      if (!first) {
        oss << ", ";
      }
      first = false;
      oss << yaml_value_to_json(item);
    }
    oss << "]";
    return oss.str();
  }
  try {
    const bool value = node.as<bool>();
    return value ? "true" : "false";
  } catch (...) {
  }
  try {
    return std::to_string(node.as<int>());
  } catch (...) {
  }
  try {
    std::ostringstream oss;
    oss << node.as<double>();
    return oss.str();
  } catch (...) {
  }
  try {
    const std::string value = node.as<std::string>();
    std::ostringstream oss;
    oss << '"';
    for (const char c : value) {
      if (c == '"' || c == '\\') {
        oss << '\\';
      }
      oss << c;
    }
    oss << '"';
    return oss.str();
  } catch (...) {
    return "null";
  }
}

}  // namespace

// =============================================================================
// S0 — Python execution/mqtt_client.py → mqtt_client.hpp / mqtt_client.cpp
// =============================================================================
// Python lines 36–40: from execution.mqtt_client import MQTTFleetClient
// Broker subscribe/publish implementation lives in mqtt_client.cpp.
// Below: adapter methods that call MQTTFleetClient (execution_full_control.py).

// =============================================================================
// SECTION 03 — Config + MQTT setup (Python lines 545–642)
// =============================================================================
void RMFIntegratedFleetAdapter::load_configuration()
{
  // Python lines 545–592
  try {
    const std::string resolved_config_path = resolve_file_path(config_path);
    const YAML::Node config_data = YAML::LoadFile(resolved_config_path);

    YAML::Node fleet_config_node;
    if (config_data["execution"] && config_data["execution"]["ros__parameters"]) {
      fleet_config_node = config_data["execution"]["ros__parameters"];
    }
    if (!fleet_config_node || !fleet_config_node.IsMap()) {
      fleet_config_node = config_data;
    }

    std::string fleet_nav_graph_path = nav_graph_path;
    if (fleet_config_node["nav_graph_path"]) {
      fleet_nav_graph_path = fleet_config_node["nav_graph_path"].as<std::string>();
    }

    std::string fleet_map_name = "L1";
    if (fleet_config_node["map_name"]) {
      fleet_map_name = fleet_config_node["map_name"].as<std::string>();
    }

    fleet_config = FleetConfig{
      fleet_name,
      {},
      fleet_nav_graph_path,
      fleet_map_name,
    };

    RCLCPP_INFO(
      get_logger(),
      "Loaded fleet configuration; robots register from MQTT/VDA pose (not config.yaml)");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to load configuration: %s", e.what());
    fleet_config = FleetConfig{
      fleet_name,
      {},
      nav_graph_path,
      "L1",
    };
  }
}

void RMFIntegratedFleetAdapter::initialize_nav_graph_parser()
{
  // Python lines 594–602
  try {
    if (!fleet_config.has_value()) {
      throw std::runtime_error("fleet_config is not loaded");
    }
    const std::string resolved_nav_graph_path = resolve_file_path(fleet_config->nav_graph_path);
    nav_graph_parser = std::make_unique<NavGraphParser>(resolved_nav_graph_path);
    RCLCPP_INFO(
      get_logger(), "Navigation graph loaded with %zu waypoints",
      nav_graph_parser->waypoints.size());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to load navigation graph: %s", e.what());
    nav_graph_parser.reset();
  }
}

std::string RMFIntegratedFleetAdapter::resolve_file_path(const std::string & file_path)
{
  // Python lines 604–627
  const std::filesystem::path source_file = std::filesystem::path(__FILE__);
  const std::filesystem::path cwd = std::filesystem::current_path();
  const std::filesystem::path package_dir = source_file.parent_path().parent_path() / "execution";

  std::vector<std::filesystem::path> candidates;
  if (std::filesystem::path(file_path).is_absolute()) {
    candidates.push_back(std::filesystem::path(file_path));
  }
  candidates.insert(candidates.end(), {
    cwd / file_path,
    package_dir / file_path,
    package_dir.parent_path() / file_path,
    package_dir.parent_path().parent_path().parent_path() / file_path,
    package_dir.parent_path().parent_path() / "share" / "execution" / file_path,
    cwd / "install" / "execution" / "share" / "execution" / file_path,
    std::filesystem::path("/opt/ros/humble/share/execution") / file_path,
  });

  for (const auto & candidate : candidates) {
    if (candidate.empty()) {
      continue;
    }
    if (std::filesystem::exists(candidate)) {
      return std::filesystem::absolute(candidate).string();
    }
  }

  throw std::runtime_error("Could not find file: " + file_path);
}

void RMFIntegratedFleetAdapter::setup_mqtt_client()
{
  // Python lines 629–642
  try {
    mqtt_client = std::make_unique<MQTTFleetClient>(
      mqtt_broker_host,
      mqtt_broker_port,
      "rmf_fleet_adapter_" + fleet_name);
    if (mqtt_client->start()) {
      RCLCPP_INFO(get_logger(), "MQTT client connected and subscribed");
    } else {
      RCLCPP_ERROR(get_logger(), "MQTT client failed to start (is libmosquitto available?)");
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to setup MQTT client: %s", e.what());
  }
}


// =============================================================================
// SECTION 04 — Robot state from MQTT (Python lines 644–804)
// =============================================================================
void RMFIntegratedFleetAdapter::publish_base_extension_from_mqtt(
  const std::map<std::string, AnyMap> & robots)
{
  if (!base_extension_pub_) {
    return;
  }
  for (const auto & [robot_id, state] : robots) {
    if (state.find("vda_order_update_id") == state.end() ||
      state.find("vda_new_base_request") == state.end())
    {
      continue;
    }
    traffic_manager::msg::BaseExtensionRequest msg;
    msg.stamp = get_clock()->now();
    msg.fleet_name = fleet_name;
    msg.robot_name = robot_id;
    const int reported_update_id =
      static_cast<int>(anymap_get_double(state, "vda_order_update_id"));
    // traffic_manager decodes acks as release_update_id + leg revision (no
    // phase base). Strip the drop-leg base that build_vda5050_order_payload
    // added, or every drop-leg ack lands one slot high and the base is never
    // extended past the first release (delivery.dispatch-0, 2026-11-23:
    // 451× ack_mismatch received_slot=2 expected=1).
    int update_base = 0;
    const auto pub_it = published_order_by_robot.find(robot_id);
    if (pub_it != published_order_by_robot.end()) {
      const std::string state_order_id = anymap_get_string(state, "current_task");
      if (state_order_id.empty() || state_order_id == pub_it->second.order_id) {
        update_base = vda_order_update_base(pub_it->second.phase);
      }
    }
    if (reported_update_id < update_base) {
      // Robot still echoes the previous (pick) leg of the same orderId —
      // not an ack for anything traffic_manager has released on this leg.
      RCLCPP_DEBUG(
        get_logger(),
        "[BaseRequests] %s: skipping stale ack orderUpdateId=%d below leg base %d "
        "(phase=%s)",
        robot_id.c_str(), reported_update_id, update_base,
        pub_it->second.phase.c_str());
      continue;
    }
    msg.order_update_id = static_cast<uint32_t>(reported_update_id - update_base);
    msg.new_base_request = anymap_get_bool(state, "vda_new_base_request", false);
    const auto robot_it = robot_states.find(robot_id);
    if (robot_it != robot_states.end()) {
      msg.dispatch_task_id = robot_it->second.current_task;
      msg.task_id = robot_state_publish_task_id(robot_it->second);
      msg.order_id = robot_it->second.current_task;
    } else if (const auto booking = py_map_get_optional_string(state, "current_task")) {
      msg.dispatch_task_id = *booking;
      msg.order_id = *booking;
    }
    if (const auto order_from_state = py_map_get_optional_string(state, "current_task")) {
      msg.order_id = *order_from_state;
    }
    base_extension_pub_->publish(msg);
  }
}

void RMFIntegratedFleetAdapter::poll_amr_states_tick()
{
  // Python lines 644–662
  try {
    if (!mqtt_client) {
      return;
    }
    const auto robots = mqtt_client->get_robot_states();
    if (!robots.empty()) {
      update_robot_states(robots);
      publish_base_extension_from_mqtt(robots);
    }
    for (const auto & event : mqtt_client->get_and_clear_load_status_events()) {
      handle_load_status_event(event);
    }
    for (const auto & event : mqtt_client->get_and_clear_nav_status_events()) {
      handle_nav_status_event(event);
    }

    // Retry orders held back by the idle gate. flush_pending_vda_orders() is
    // otherwise only driven by task-status changes, so an order held because the
    // robot was still driving would never be re-attempted — the robot stops and
    // nothing notices. Robot states were just refreshed above, so this sees the
    // newest driving/idle value.
    if (!order_hold_since_by_task.empty()) {
      std::vector<std::string> held_tasks;
      held_tasks.reserve(order_hold_since_by_task.size());
      for (const auto & [held_task_id, _] : order_hold_since_by_task) {
        held_tasks.push_back(held_task_id);
      }
      for (const auto & held_task_id : held_tasks) {
        flush_pending_vda_orders(held_task_id);
      }
    }

    publish_fleet_state();
    publish_tf_frames(robots);
    monitor_task_completion();
    if (!robots.empty()) {
      RCLCPP_DEBUG(get_logger(), "Updated %zu robots", robots.size());
    }
  } catch (const std::exception & e) {
    RCLCPP_WARN(get_logger(), "Failed to poll AMR states: %s", e.what());
  }
}

AMRRobot & RMFIntegratedFleetAdapter::register_robot_from_mqtt(
  const std::string & robot_id,
  const AnyMap & robot_data)
{
  // Python lines 664–693
  const auto pose_it = robot_data.find("pose");
  const YAML::Node pose = (pose_it != robot_data.end()) ? pose_it->second : YAML::Node();
  const auto battery_it = robot_data.find("battery");
  const YAML::Node battery = (battery_it != robot_data.end()) ? battery_it->second : YAML::Node();

  double initial_battery = 0.0;
  if (battery && battery.IsMap() && battery["percentage"]) {
    initial_battery = battery["percentage"].as<double>();
  }

  AMRRobot robot;
  robot.robot_id = robot_id;
  robot.pose.x = (pose && pose.IsMap() && pose["x"]) ? pose["x"].as<double>() : 0.0;
  robot.pose.y = (pose && pose.IsMap() && pose["y"]) ? pose["y"].as<double>() : 0.0;
  robot.pose.yaw = (pose && pose.IsMap() && pose["yaw"]) ? pose["yaw"].as<double>() : 0.0;
  robot.twist.linear = 0.0;    // [SPEED] velocity from mqtt_client; 0.0 until the first one arrives
  robot.twist.angular = 0.0;
  robot.battery_percentage = initial_battery;
  robot.status = RobotStatus::Idle;
  robot.current_task = "";
  robot.namespace_ = "/" + robot_id;

  fleet_config->robots[robot_id] = robot;
  nav_idle_by_robot[robot_id] = true;
  int mqtt_load = 0;
  if (py_load_status_in_2_4(robot_data, "load_status", &mqtt_load)) {
    last_load_status_by_robot[robot_id] = mqtt_load;
  } else {
    last_load_status_by_robot[robot_id] = std::nullopt;
  }

  int mqtt_operating_mode = 0;
  if (py_operating_mode_in_0_1(robot_data, "operating_mode", &mqtt_operating_mode)) {
    last_operating_mode_by_robot[robot_id] = mqtt_operating_mode;
  } else {
    last_operating_mode_by_robot[robot_id] = std::nullopt;
  }

  const auto & inserted = fleet_config->robots[robot_id];
  RCLCPP_INFO(
    get_logger(), "🤖 Registered robot %s from MQTT pose (%.2f, %.2f)",
    robot_id.c_str(), inserted.pose.x, inserted.pose.y);
  return fleet_config->robots[robot_id];
}

bool RMFIntegratedFleetAdapter::mqtt_task_confirmed_by_api(
  const std::string & robot_id,
  const std::string & task_id) const
{
  // Python lines 695–712
  if (task_id.empty()) {
    return true;
  }

  const auto underway_it = currently_underway_task.find(robot_id);
  if (underway_it != currently_underway_task.end() && underway_it->second == task_id) {
    return true;
  }

  std::string status;
  const auto status_it = known_task_status.find(task_id);
  if (status_it != known_task_status.end()) {
    status = status_it->second;
  }

  std::string assigned;
  const auto known_robot_it = known_task_robot.find(task_id);
  if (known_robot_it != known_task_robot.end()) {
    assigned = known_robot_it->second;
  }
  if (assigned.empty()) {
    const auto dispatch_it = dispatch_assignments.find(task_id);
    if (dispatch_it != dispatch_assignments.end()) {
      assigned = dispatch_it->second;
    }
  }

  if (!assigned.empty() && assigned != robot_id) {
    return false;
  }
  if (status == "underway" || status == "queued") {
    return assigned.empty() || assigned == robot_id;
  }
  return false;
}

void RMFIntegratedFleetAdapter::update_robot_states(const std::map<std::string, AnyMap> & robots)
{
  // Python lines 714–804
  if (!fleet_config.has_value()) {
    return;
  }

  for (const auto & [robot_id, robot_data] : robots) {
    if (fleet_config->robots.find(robot_id) == fleet_config->robots.end()) {
      register_robot_from_mqtt(robot_id, robot_data);
    }

    auto robot_it = fleet_config->robots.find(robot_id);
    if (robot_it == fleet_config->robots.end()) {
      continue;
    }
    auto & robot = robot_it->second;

    const auto pose_it = robot_data.find("pose");
    const YAML::Node pose = (pose_it != robot_data.end()) ? pose_it->second : YAML::Node();
    robot.pose.x = (pose && pose.IsMap() && pose["x"]) ? pose["x"].as<double>() : 0.0;
    robot.pose.y = (pose && pose.IsMap() && pose["y"]) ? pose["y"].as<double>() : 0.0;
    robot.pose.yaw = (pose && pose.IsMap() && pose["yaw"]) ? pose["yaw"].as<double>() : 0.0;

    const auto twist_it = robot_data.find("twist");
    const YAML::Node twist = (twist_it != robot_data.end()) ? twist_it->second : YAML::Node();
    // [SPEED] twist.linear = the robot's `velocity` (m/s, sign kept as received) put
    // into the data map by mqtt_client.cpp. No special values: it is passed on as is.
    robot.twist.linear = (twist && twist.IsMap() && twist["linear"]) ? twist["linear"].as<double>() : 0.0;
    robot.twist.angular = (twist && twist.IsMap() && twist["angular"]) ? twist["angular"].as<double>() : 0.0;

    const auto battery_it = robot_data.find("battery");
    const YAML::Node battery = (battery_it != robot_data.end()) ? battery_it->second : YAML::Node();
    if (battery && battery.IsMap() && battery["percentage"]) {
      try {
        robot.battery_percentage = battery["percentage"].as<double>();
      } catch (...) {
      }
    }

    // [CHARGING] powerSupply.charging (bool or null) put into the data map by
    // mqtt_client.cpp. It is passed to RMF as RobotState.charging_status, where
    // a charge task completes on 1 -> 0 and fails on -1 (unknown). A value from
    // a state older than CHARGING_STATE_MAX_AGE_SEC also counts as unknown,
    // because the stored state is re-read every tick even when the robot has
    // stopped sending.
    {
      int charging_status = -1;
      const auto charging_it = robot_data.find("charging");
      const auto received_it = robot_data.find("state_received_at");
      const YAML::Node charging =
        (charging_it != robot_data.end()) ? charging_it->second : YAML::Node();
      double received_at = 0.0;
      if (received_it != robot_data.end() && received_it->second &&
        !received_it->second.IsNull())
      {
        try {
          received_at = received_it->second.as<double>();
        } catch (...) {
        }
      }
      const double now_sec = std::chrono::duration<double>(
        std::chrono::system_clock::now().time_since_epoch()).count();
      const bool fresh = received_at > 0.0 &&
        (now_sec - received_at) <= CHARGING_STATE_MAX_AGE_SEC;
      if (fresh && charging && !charging.IsNull()) {
        try {
          charging_status = charging.as<bool>() ? 1 : 0;
        } catch (...) {
        }
      }
      if (charging_status != robot.charging_status) {
        RCLCPP_INFO(
          get_logger(),
          "🔌 [CHARGING] Robot %s charging status %d -> %d "
          "(1 = charging, 0 = not charging, -1 = unknown), battery %.1f%%",
          robot_id.c_str(), robot.charging_status, charging_status,
          robot.battery_percentage);
      }
      robot.charging_status = charging_status;
    }

    std::string status_str = to_lower_ascii(py_map_get_string(robot_data, "status", "idle"));
    if (robot.current_task.empty()) {
      if (status_str == "moving") {robot.status = RobotStatus::Moving;}
      else if (status_str == "navigating") {robot.status = RobotStatus::Navigating;}
      else if (status_str == "charging") {robot.status = RobotStatus::Charging;}
      else if (status_str == "error") {robot.status = RobotStatus::Error;}
      else if (status_str == "offline") {robot.status = RobotStatus::Offline;}
      else {robot.status = RobotStatus::Idle;}
    } else {
      // While a task is active the adapter used to ignore the robot's reported
      // status entirely, keeping whatever it set itself on path dispatch
      // (Navigating). That made RobotState.mode a record of "we sent this robot
      // a path", not "this robot is moving" — the FMS could not tell a stalled
      // robot from a driving one, and a replan could never observe the robot
      // coming to a stop.
      //
      // The robot now reports VDA5050 `driving` (derived from the nav2
      // navigate_through_poses goal status), so moving/idle is authoritative
      // even mid-task. Error/offline still win over both.
      if (status_str == "error") {robot.status = RobotStatus::Error;}
      else if (status_str == "offline") {robot.status = RobotStatus::Offline;}
      else if (status_str == "moving") {robot.status = RobotStatus::Moving;}
      else if (status_str == "idle") {robot.status = RobotStatus::Idle;}
    }

    const std::string api_task = py_map_get_string(robot_data, "current_task", "");
    RCLCPP_DEBUG(
      get_logger(), "🆔 [TASK ID FLOW] Robot %s: Current RMF task='%s', API task='%s'",
      robot_id.c_str(), robot.current_task.c_str(), api_task.c_str());

    if (robot.current_task.empty()) {
      const auto cleared_it = robot_task_cleared.find(robot_id);
      const bool task_cleared = (cleared_it != robot_task_cleared.end()) ? cleared_it->second : false;
      if (!task_cleared) {
        if (!api_task.empty() && !mqtt_task_confirmed_by_api(robot_id, api_task)) {
          robot.current_task = "";
          RCLCPP_WARN(
            get_logger(),
            "🚫 [TASK ID FLOW] Robot %s: Ignoring stale MQTT orderId '%s' (API status='%s')",
            robot_id.c_str(),
            api_task.c_str(),
            known_task_status.count(api_task) ? known_task_status[api_task].c_str() : "unknown");
        } else {
          robot.current_task = api_task;
          RCLCPP_DEBUG(
            get_logger(), "🆔 [TASK ID FLOW] Robot %s: Set task from API: '%s'",
            robot_id.c_str(), api_task.c_str());
        }
      } else {
        RCLCPP_DEBUG(
          get_logger(),
          "🆔 [TASK ID FLOW] Robot %s: Task was intentionally cleared by fleet adapter, "
          "not restoring from API: '%s'",
          robot_id.c_str(), api_task.c_str());
      }
    } else if (!robot.fleet_publish_task_id.empty()) {
      RCLCPP_DEBUG(
        get_logger(),
        "🆔 [TASK ID FLOW] Robot %s: Preserving booking '%s' (RMF segment id='%s')",
        robot_id.c_str(), robot.current_task.c_str(), robot.fleet_publish_task_id.c_str());
    } else if (robot.current_task.rfind("delivery.dispatch", 0) == 0) {
      RCLCPP_DEBUG(
        get_logger(), "🆔 [TASK ID FLOW] Robot %s: Preserving booking: '%s'",
        robot_id.c_str(), robot.current_task.c_str());
    } else if (robot.current_task.rfind("patrol.dispatch", 0) == 0) {
      RCLCPP_DEBUG(
        get_logger(), "🆔 [TASK ID FLOW] Robot %s: Preserving booking: '%s'",
        robot_id.c_str(), robot.current_task.c_str());
    } else {
      RCLCPP_DEBUG(
        get_logger(), "🆔 [TASK ID FLOW] Robot %s: Preserving task: '%s'",
        robot_id.c_str(), robot.current_task.c_str());
    }

    int mqtt_load = 0;
    if (py_load_status_in_2_4(robot_data, "load_status", &mqtt_load)) {
      last_load_status_by_robot[robot_id] = mqtt_load;
    }

    int mqtt_operating_mode = 0;
    if (py_operating_mode_in_0_1(robot_data, "operating_mode", &mqtt_operating_mode)) {
      last_operating_mode_by_robot[robot_id] = mqtt_operating_mode;
    }
  }
}

// =============================================================================
// MonitoredTaskManager — Python monitor/monitor_logic.py
// =============================================================================
MonitoredTaskManager::MonitoredTaskManager(
  const std::string & fleet_name,
  rclcpp::Publisher<monitor::msg::MonitoredTaskDetails>::SharedPtr monitored_task_pub,
  rclcpp::Logger logger)
: fleet_name_(fleet_name),
  pub_(std::move(monitored_task_pub)),
  logger_(logger)
{
}

bool MonitoredTaskManager::has_started(const std::string & task_id) const
{
  return started_tasks_.count(task_id) > 0;
}

void MonitoredTaskManager::set_planned_path(
  const std::string & robot_name,
  const std::vector<std::string> & planned_path)
{
  if (!robot_name.empty()) {
    planned_path_by_robot_[robot_name] = planned_path;
  }
}

std::vector<std::string> MonitoredTaskManager::get_planned_path(const std::string & robot_name) const
{
  if (robot_name.empty()) {
    return {};
  }
  const auto it = planned_path_by_robot_.find(robot_name);
  return (it != planned_path_by_robot_.end()) ? it->second : std::vector<std::string>{};
}

void MonitoredTaskManager::set_load_status(
  const std::string & robot_name,
  const std::string & load_status)
{
  if (!robot_name.empty() && !load_status.empty()) {
    load_status_by_robot_[robot_name] = load_status;
  }
}

std::string MonitoredTaskManager::get_load_status(const std::string & robot_name) const
{
  if (robot_name.empty()) {
    return "";
  }
  const auto it = load_status_by_robot_.find(robot_name);
  return (it != load_status_by_robot_.end()) ? it->second : "";
}

std::string MonitoredTaskManager::status_for_topic(const std::string & status)
{
  return trim_copy(status);
}

std::string MonitoredTaskManager::priority_for_topic(
  const std::string & task_id, const AnyMap & booking)
{
  YAML::Node priority_value;
  const auto it = booking.find("priority");
  if (it != booking.end() && !it->second.IsNull()) {
    priority_value = it->second;
    priority_cache_[task_id] = priority_value;
  } else {
    const auto cached = priority_cache_.find(task_id);
    if (cached != priority_cache_.end()) {
      priority_value = cached->second;
    }
  }
  if (!priority_value.IsDefined() || priority_value.IsNull()) {
    return "";
  }
  if (priority_value.IsMap()) {
    const auto type_it = priority_value["type"];
    if (type_it && type_it.as<std::string>() == "binary") {
      const YAML::Node val = priority_value["value"];
      int v = 0;
      if (val && !val.IsNull()) {
        try {
          v = val.as<int>();
        } catch (...) {
          try {
            const std::string s = val.as<std::string>();
            v = s.empty() ? 0 : std::stoi(s);
          } catch (...) {
            try {
              v = static_cast<int>(val.as<double>());
            } catch (...) {
              v = 0;
            }
          }
        }
      }
      return v > 0 ? "high" : "low";
    }
    return yaml_node_to_json_string(priority_value);
  }
  try {
    return priority_value.as<std::string>();
  } catch (...) {
    try {
      return std::to_string(priority_value.as<int>());
    } catch (...) {
      return "";
    }
  }
}

void MonitoredTaskManager::update_task_cache(const std::string & task_id, const AnyMap & updates)
{
  if (task_id.empty()) {
    return;
  }
  AnyMap & entry = task_cache_[task_id];
  if (entry.empty()) {
    entry = {
      {"task_id", YAML::Node(task_id)},
      {"amr_id", YAML::Node("")},
      {"fleet_name", YAML::Node(fleet_name_)},
      {"category", YAML::Node("")},
      {"priority", YAML::Node("")},
      {"status", YAML::Node("")},
      {"load_status", YAML::Node("")},
      {"phase", YAML::Node("")},
      {"planned_path", YAML::Node(YAML::NodeType::Sequence)},
    };
  }
  for (const auto & [key, value] : updates) {
    if (value.IsNull()) {
      continue;
    }
    entry[key] = value;
  }
}

bool MonitoredTaskManager::publish_task(const std::string & task_id)
{
  const auto it = task_cache_.find(task_id);
  if (it == task_cache_.end()) {
    return false;
  }
  const AnyMap & entry = it->second;
  std::vector<std::string> planned_path;
  const auto path_it = entry.find("planned_path");
  if (path_it != entry.end() && path_it->second.IsSequence()) {
    for (const auto & node : path_it->second) {
      try {
        planned_path.push_back(node.as<std::string>());
      } catch (...) {
        planned_path.push_back("");
      }
    }
  }
  std::vector<std::string> signature_with_path{
    anymap_get_string(entry, "task_id"),
    anymap_get_string(entry, "amr_id"),
    anymap_get_string(entry, "fleet_name", fleet_name_),
    anymap_get_string(entry, "category"),
    anymap_get_string(entry, "priority"),
    anymap_get_string(entry, "status"),
    anymap_get_string(entry, "load_status"),
    anymap_get_string(entry, "phase"),
  };
  for (const auto & p : planned_path) {
    signature_with_path.push_back(p);
  }
  const auto sig_it = last_published_signature_.find(task_id);
  if (sig_it != last_published_signature_.end() && sig_it->second == signature_with_path) {
    return false;
  }

  monitor::msg::MonitoredTaskDetails msg;
  msg.task_id = anymap_get_string(entry, "task_id");
  msg.amr_id = anymap_get_string(entry, "amr_id");
  msg.fleet_name = anymap_get_string(entry, "fleet_name", fleet_name_);
  msg.category = anymap_get_string(entry, "category");
  msg.priority = anymap_get_string(entry, "priority");
  msg.status = anymap_get_string(entry, "status");
  msg.load_status = anymap_get_string(entry, "load_status");
  msg.phase = anymap_get_string(entry, "phase");
  msg.planned_path = planned_path;
  pub_->publish(msg);
  last_published_signature_[task_id] = signature_with_path;
  return true;
}

void MonitoredTaskManager::clear_task_state(const std::string & task_id)
{
  std::string robot_name;
  const auto it = task_cache_.find(task_id);
  if (it != task_cache_.end()) {
    robot_name = anymap_get_string(it->second, "amr_id");
    task_cache_.erase(it);
  }
  started_tasks_.erase(task_id);
  observed_status_.erase(task_id);
  priority_cache_.erase(task_id);
  last_published_signature_.erase(task_id);
  if (!robot_name.empty()) {
    planned_path_by_robot_.erase(robot_name);
    load_status_by_robot_.erase(robot_name);
  }
}

MonitoredPublishRuleResult MonitoredTaskManager::handle_publish_rule(
  const std::string & task_id, const std::string & new_status)
{
  MonitoredPublishRuleResult result;
  if (task_id.empty()) {
    return result;
  }

  const auto prev_it = observed_status_.find(task_id);
  const std::string prev_status_key = (prev_it != observed_status_.end()) ? prev_it->second : "";
  const std::string status_value = status_for_topic(new_status);
  std::string status_key = status_value;
  std::transform(status_key.begin(), status_key.end(), status_key.begin(), ::tolower);
  observed_status_[task_id] = status_key;
  update_task_cache(task_id, {{"status", YAML::Node(status_value)}});

  result.reached_final =
    (status_key == "completed" || status_key == "canceled" || status_key == "failed");

  if (status_key == "underway") {
    if (prev_status_key != "underway") {
      started_tasks_.insert(task_id);
      result.published = publish_task(task_id);
      result.should_flush_pending = true;
    }
    return result;
  }

  // 'standby' also releases a queued order. RMF parks a task in standby
  // between assigning it and its first event going active, and a task can sit
  // there indefinitely — patrol.dispatch-3 (Aug 2026) never left standby, so
  // waiting for 'underway' meant its VDA5050 order was never published and the
  // robot stood still while RViz happily drew its planned path.
  //
  // This is safe because flush_pending_vda_orders() only publishes an order
  // that already carries valid waypoints: the order is built from a path
  // request that RMF itself issued, and can_publish_vda_order_for_phase() is
  // still consulted per order before anything goes out.
  if (status_key == "standby") {
    if (prev_status_key != "standby" && started_tasks_.count(task_id) == 0) {
      started_tasks_.insert(task_id);
      result.published = publish_task(task_id);
      result.should_flush_pending = true;
    }
    return result;
  }

  if (started_tasks_.count(task_id) > 0 && !prev_status_key.empty() && prev_status_key != status_key) {
    result.published = publish_task(task_id);
    result.should_flush_pending = true;
  }

  if (result.reached_final) {
    clear_task_state(task_id);
  }
  return result;
}

std::string MonitoredTaskManager::resolve_task_id(
  const std::string & robot_name,
  const std::string & candidate_task_id,
  const std::map<std::string, std::string> & currently_underway_task,
  const std::map<std::string, std::string> & robot_to_coords_key)
{
  if (!robot_name.empty()) {
    const auto underway_it = currently_underway_task.find(robot_name);
    if (underway_it != currently_underway_task.end() && !underway_it->second.empty()) {
      return underway_it->second;
    }
    const auto coords_it = robot_to_coords_key.find(robot_name);
    if (coords_it != robot_to_coords_key.end() && !coords_it->second.empty()) {
      return coords_it->second;
    }
  }
  return candidate_task_id;
}
NavGraphParser::NavGraphParser(const std::string & nav_graph_path_in)
: nav_graph_path(nav_graph_path_in)
{
  parse_nav_graph();
}

void NavGraphParser::parse_nav_graph()
{
  try {
    const YAML::Node data = YAML::LoadFile(nav_graph_path);

    // Python: building_name = data.get('building_name', 'R3')  (read, not stored on parser)
    (void)data["building_name"];

    const YAML::Node levels = data["levels"] ? data["levels"] : empty_yaml_map();

    // Python: level_keys = list(levels.keys()); if level_keys: ...
    if (!levels.IsMap() || levels.size() == 0) {
      return;
    }

    map_name = levels.begin()->first.as<std::string>();
    const YAML::Node level_data = levels[map_name];

    const YAML::Node vertices = level_data["vertices"] ?
      level_data["vertices"] : YAML::Node(YAML::NodeType::Sequence);

    corridor_geometry_.vertices.clear();
    corridor_geometry_.lanes.clear();

    std::size_t i = 0;
    for (const auto & vertex : vertices) {
      double x = 0.0;
      double y = 0.0;
      std::string name;
      YAML::Node properties = empty_yaml_map();

      if (vertex.IsSequence() && vertex.size() >= 2) {
        // Python: x, y = vertex[0], vertex[1]
        x = vertex[0].as<double>();
        y = vertex[1].as<double>();
        // Python: properties = vertex[2] if len(vertex) > 2 else {}
        properties = (vertex.size() > 2) ? vertex[2] : empty_yaml_map();
        if (!properties.IsMap()) {
          throw std::runtime_error("vertex properties must be a dict");
        }
        // Python: name = properties.get('name', f'vertex_{i}')
        name = yaml_map_get_string(properties, "name", "vertex_" + std::to_string(i));
      } else if (vertex.IsMap()) {
        // Python: x = vertex.get('x', 0.0); y = vertex.get('y', 0.0)
        x = vertex["x"] ? vertex["x"].as<double>() : 0.0;
        y = vertex["y"] ? vertex["y"].as<double>() : 0.0;
        // Python: name = vertex.get('name', f'vertex_{i}')
        name = yaml_map_get_string(vertex, "name", "vertex_" + std::to_string(i));
        // Python: properties = {k: v for k, v in vertex.items() if k not in ['x', 'y', 'name']}
        properties = empty_yaml_map();
        for (const auto & item : vertex) {
          const std::string key = item.first.as<std::string>();
          if (key != "x" && key != "y" && key != "name") {
            properties[key] = item.second;
          }
        }
      } else {
        // NaN placeholder keeps vertex indices equal to the YAML array's.
        corridor_geometry_.vertices.push_back(
          {std::numeric_limits<double>::quiet_NaN(),
           std::numeric_limits<double>::quiet_NaN()});
        ++i;
        continue;
      }

      Waypoint waypoint;
      waypoint.id = std::to_string(i);
      waypoint.x = x;
      waypoint.y = y;
      waypoint.yaw = 0.0;
      waypoint.name = name;
      // Python: properties.get('is_holding_point', False) / properties.get('is_charger', False)
      waypoint.is_holding_point = yaml_map_get_bool_default_false(properties, "is_holding_point");
      waypoint.is_charger = yaml_map_get_bool_default_false(properties, "is_charger");

      waypoints[waypoint.id] = waypoint;

      // Python: if waypoint.name and waypoint.name != waypoint.id:
      if (waypoint.name.has_value() && !waypoint.name->empty() && *waypoint.name != waypoint.id) {
        waypoints[*waypoint.name] = waypoint;
      }

      corridor_geometry_.vertices.push_back({x, y});
      ++i;
    }

    const YAML::Node lanes_node = level_data["lanes"] ?
      level_data["lanes"] : YAML::Node(YAML::NodeType::Sequence);

    std::size_t lane_i = 0;
    for (const auto & lane : lanes_node) {
      int start_idx = 0;
      int end_idx = 0;
      YAML::Node properties = empty_yaml_map();

      if (lane.IsSequence() && lane.size() >= 2) {
        start_idx = lane[0].as<int>();
        end_idx = lane[1].as<int>();
        properties = (lane.size() > 2) ? lane[2] : empty_yaml_map();
      } else if (lane.IsMap()) {
        start_idx = lane["v_idx1"] ? lane["v_idx1"].as<int>() : 0;
        end_idx = lane["v_idx2"] ? lane["v_idx2"].as<int>() : 1;
        properties = empty_yaml_map();
        for (const auto & item : lane) {
          const std::string key = item.first.as<std::string>();
          if (key != "v_idx1" && key != "v_idx2") {
            properties[key] = item.second;
          }
        }
      } else {
        corridor_geometry_.lanes.push_back({-1, -1});
        ++lane_i;
        continue;
      }

      Lane lane_obj;
      lane_obj.id = "lane_" + std::to_string(lane_i);
      lane_obj.start_waypoint = std::to_string(start_idx);
      lane_obj.end_waypoint = std::to_string(end_idx);
      corridor_geometry_.lanes.push_back({start_idx, end_idx});
      if (properties.IsMap()) {
        for (const auto & item : properties) {
          lane_obj.properties[item.first.as<std::string>()] = item.second;
        }
      }

      lanes[lane_obj.id] = lane_obj;
      ++lane_i;
    }

    // ---- Zones -> lane indices (Phase 2) --------------------------------
    // Re-walk the raw lane list so we can key on YAML lane INDEX, which is what
    // RMF lane closures expect. `lanes` (above) is keyed by "lane_<i>" strings.
    zone_lanes.clear();
    {
      std::vector<std::pair<int, int>> lane_endpoints;
      lane_endpoints.reserve(lanes_node.size());
      for (const auto & lane : lanes_node) {
        if (lane.IsSequence() && lane.size() >= 2) {
          lane_endpoints.emplace_back(lane[0].as<int>(), lane[1].as<int>());
        } else if (lane.IsMap()) {
          lane_endpoints.emplace_back(
            lane["v_idx1"] ? lane["v_idx1"].as<int>() : 0,
            lane["v_idx2"] ? lane["v_idx2"].as<int>() : 1);
        } else {
          lane_endpoints.emplace_back(-1, -1);
        }
      }

      const YAML::Node zones_node = level_data["zones"];
      if (zones_node && zones_node.IsSequence()) {
        for (const auto & zone_node : zones_node) {
          if (!zone_node["id"]) {
            continue;
          }
          const std::string zone_id = yaml_node_as_string(zone_node["id"]);
          if (zone_id.empty()) {
            continue;
          }

          std::set<int> zone_wps;
          if (zone_node["waypoints"] && zone_node["waypoints"].IsSequence()) {
            for (const auto & idx : zone_node["waypoints"]) {
              try {
                zone_wps.insert(idx.as<int>());
              } catch (...) {
              }
            }
          }
          if (zone_wps.empty()) {
            continue;
          }

          // internal + crossing: either endpoint inside the zone
          std::vector<std::size_t> indices;
          for (std::size_t i = 0; i < lane_endpoints.size(); ++i) {
            const auto & [a, b] = lane_endpoints[i];
            if (zone_wps.count(a) > 0 || zone_wps.count(b) > 0) {
              indices.push_back(i);
            }
          }
          if (!indices.empty()) {
            zone_lanes[zone_id] = std::move(indices);
          }
        }
      }
    }

    std::set<std::string> unique_waypoints;
    for (const auto & [wp_id, wp] : waypoints) {
      (void)wp_id;
      // Python: if wp.name and wp.name != f'vertex_{wp.id}':
      if (wp.name.has_value() && !wp.name->empty() && *wp.name != ("vertex_" + wp.id)) {
        unique_waypoints.insert(*wp.name);
      }
    }

    std::cout << "✅ Navigation graph parsed successfully:" << std::endl;
    std::cout << "   📍 Unique Waypoints: " << unique_waypoints.size() << std::endl;
    std::cout << "   🛤️  Lanes: " << lanes.size() << std::endl;
    std::cout << "   🧩 Zones with lanes: " << zone_lanes.size() << std::endl;

    for (const auto & [wp_id, wp] : waypoints) {
      (void)wp_id;
      if (wp.name.has_value() && !wp.name->empty() && *wp.name != ("vertex_" + wp.id)) {
        std::cout << "   - " << *wp.name << ": (" << wp.x << ", " << wp.y << ")" << std::endl;
      }
    }
  } catch (const std::exception & e) {
    std::cerr << "❌ Error parsing nav_graph: " << e.what() << std::endl;
  }
}

std::vector<std::size_t> NavGraphParser::lanes_for_zone(const std::string & zone_id) const
{
  const auto it = zone_lanes.find(trim_copy(zone_id));
  if (it == zone_lanes.end()) {
    return {};
  }
  return it->second;
}

std::optional<Waypoint> NavGraphParser::get_waypoint_by_name(const std::string & name) const
{
  const auto it = waypoints.find(name);
  if (it != waypoints.end()) {
    return it->second;
  }
  return std::nullopt;
}

std::optional<Waypoint> NavGraphParser::get_waypoint_by_id(const std::string & wp_id) const
{
  const auto it = waypoints.find(wp_id);
  if (it != waypoints.end()) {
    return it->second;
  }
  return std::nullopt;
}

std::vector<Waypoint> NavGraphParser::plan_path(
  const std::string & start_name,
  const std::string & end_name)
{
  try {
    const auto start_wp = get_waypoint_by_name(start_name);
    const auto end_wp = get_waypoint_by_name(end_name);

    if (!start_wp || !end_wp) {
      std::cerr << "❌ Could not find waypoints: " << start_name << " -> " << end_name << std::endl;
      return {};
    }

    std::cout << "🛤️  Planning path: " << start_name << " -> " << end_name << std::endl;
    return {*start_wp, *end_wp};
  } catch (const std::exception & e) {
    std::cerr << "❌ Error planning path: " << e.what() << std::endl;
    return {};
  }
}

// =============================================================================
// SECTION 02 — Python lines 273–543: Node init + ROS wiring
// =============================================================================

RMFIntegratedFleetAdapter::RMFIntegratedFleetAdapter()
: Node("rmf_integrated_fleet_adapter"),
  path_qos(rclcpp::KeepLast(10))
{
  // Python lines 277–283: declare parameters
  fleet_name = declare_parameter<std::string>("fleet_name", "amr_fleet");
  mqtt_broker_host = declare_parameter<std::string>("mqtt_broker_host", "localhost");
  mqtt_broker_port = declare_parameter<int>("mqtt_broker_port", 1883);
  config_path = declare_parameter<std::string>("config_path", "config/config.yaml");
  nav_graph_path = declare_parameter<std::string>("nav_graph_path", "config/nav_graphs/0.yaml");
  state_poll_interval = declare_parameter<double>("state_poll_interval", 0.04);
  map_conversion_mode = declare_parameter<std::string>("map_conversion_mode", "auto");
  spin_threshold_deg = declare_parameter<double>("spin_threshold_deg", 45.0);

  // Python lines 295–361: state dict init (empty containers / zero defaults above)

  // Python line 364: TF broadcaster
  tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this);

  // Python lines 370–375: fleet_qos
  auto fleet_qos = rclcpp::QoS(rclcpp::KeepLast(10));
  fleet_qos.reliable();
  fleet_qos.durability(rclcpp::DurabilityPolicy::Volatile);

  // Python lines 377–381
  fleet_states_pub = create_publisher<rmf_fleet_msgs::msg::FleetState>(
    "/fleet_states", fleet_qos);

  declare_parameter<std::string>(
    "base_extension_request_topic", "/BaseRequests");
  base_extension_pub_ = create_publisher<traffic_manager::msg::BaseExtensionRequest>(
    get_parameter("base_extension_request_topic").as_string(), fleet_qos);

  // Python lines 383–388: workcell_qos
  auto workcell_qos = rclcpp::QoS(rclcpp::KeepLast(10));
  workcell_qos.reliable();
  workcell_qos.durability(rclcpp::DurabilityPolicy::Volatile);

  // Python lines 389–403
  dispenser_results_pub = create_publisher<rmf_dispenser_msgs::msg::DispenserResult>(
    "/dispenser_results", workcell_qos);
  ingestor_results_pub = create_publisher<rmf_ingestor_msgs::msg::IngestorResult>(
    "/ingestor_results", workcell_qos);
  monitored_task_pub = create_publisher<monitor::msg::MonitoredTaskDetails>(
    "/Monitored_task_details", workcell_qos);

  // Python lines 404–408
  monitored_task_manager = std::make_unique<MonitoredTaskManager>(
    fleet_name, monitored_task_pub, get_logger());

  // Python lines 412–417: path_qos
  path_qos.reliable();
  path_qos.transient_local();

  // Python lines 418–423
  path_request_sub = create_subscription<rmf_fleet_msgs::msg::PathRequest>(
    "/robot_path_requests",
    path_qos,
    std::bind(&RMFIntegratedFleetAdapter::path_request_callback, this, std::placeholders::_1));

  // Python lines 427–438: dispatch_qos
  auto dispatch_qos = rclcpp::QoS(rclcpp::KeepLast(10));
  dispatch_qos.reliable();
  dispatch_qos.durability(rclcpp::DurabilityPolicy::Volatile);

  dispatch_states_sub = create_subscription<rmf_task_msgs::msg::DispatchStates>(
    "/dispatch_states",
    dispatch_qos,
    std::bind(&RMFIntegratedFleetAdapter::dispatch_states_callback, this, std::placeholders::_1));

  // Python lines 440–445: task_api_qos
  auto task_api_qos = rclcpp::QoS(rclcpp::KeepLast(10));
  task_api_qos.reliable();
  task_api_qos.transient_local();

  // Python lines 446–453
  task_api_sub = create_subscription<rmf_task_msgs::msg::ApiResponse>(
    "/task_api_responses",
    task_api_qos,
    std::bind(&RMFIntegratedFleetAdapter::task_api_response_callback, this, std::placeholders::_1));
  RCLCPP_INFO(
    get_logger(),
    "📡 [SUBSCRIPTION] Subscribed to /task_api_responses with QoS: reliable, transient_local, depth=10");
  RCLCPP_INFO(get_logger(), "📡 [SUBSCRIPTION] Subscription object: %p", static_cast<void *>(task_api_sub.get()));

  // Python lines 457–469: volatile_qos backup subscription
  auto volatile_qos = rclcpp::QoS(rclcpp::KeepLast(10));
  volatile_qos.reliable();
  volatile_qos.durability(rclcpp::DurabilityPolicy::Volatile);

  task_api_sub_volatile = create_subscription<rmf_task_msgs::msg::ApiResponse>(
    "/task_api_responses",
    volatile_qos,
    std::bind(&RMFIntegratedFleetAdapter::task_api_response_callback, this, std::placeholders::_1));
  RCLCPP_INFO(get_logger(), "📡 [SUBSCRIPTION] Also subscribed with VOLATILE durability as backup");

  // Python lines 472–478
  task_api_request_sub = create_subscription<rmf_task_msgs::msg::ApiRequest>(
    "/task_api_requests",
    task_api_qos,
    std::bind(&RMFIntegratedFleetAdapter::task_api_request_callback, this, std::placeholders::_1));
  RCLCPP_INFO(get_logger(), "📡 [SUBSCRIPTION] Subscribed to /task_api_requests for cancel task requests");

  // Python lines 481–492: mode_qos
  auto mode_qos = rclcpp::QoS(rclcpp::KeepLast(10));
  mode_qos.reliable();
  mode_qos.transient_local();

  mode_request_sub = create_subscription<rmf_fleet_msgs::msg::ModeRequest>(
    "/robot_mode_requests",
    mode_qos,
    std::bind(&RMFIntegratedFleetAdapter::mode_request_callback, this, std::placeholders::_1));

  // Python lines 495–507: task_state_qos depth=20
  auto task_state_qos = rclcpp::QoS(rclcpp::KeepLast(20));
  task_state_qos.reliable();
  task_state_qos.transient_local();

  task_state_update_sub = create_subscription<std_msgs::msg::String>(
    "/rmf_task_state_updates",
    task_state_qos,
    std::bind(&RMFIntegratedFleetAdapter::task_state_update_callback, this, std::placeholders::_1));
  RCLCPP_INFO(
    get_logger(),
    "📡 [SUBSCRIPTION] Subscribed to /rmf_task_state_updates for real-time task status");

  auto zone_conflict_qos = rclcpp::QoS(rclcpp::KeepLast(200));
  zone_conflict_qos.reliable();
  zone_conflicts_sub = create_subscription<zone_manager::msg::Conflict>(
    "/zone_conflicts",
    zone_conflict_qos,
    std::bind(&RMFIntegratedFleetAdapter::zone_conflict_callback, this, std::placeholders::_1));
  RCLCPP_INFO(get_logger(), "📡 [SUBSCRIPTION] Subscribed to /zone_conflicts for zone pause/resume");

  auto path_deviation_qos = rclcpp::QoS(rclcpp::KeepLast(50)).reliable();
  path_deviation_sub = create_subscription<std_msgs::msg::String>(
    "/path_deviation",
    path_deviation_qos,
    std::bind(&RMFIntegratedFleetAdapter::path_deviation_callback, this, std::placeholders::_1));
  RCLCPP_INFO(get_logger(), "📡 [SUBSCRIPTION] Subscribed to /path_deviation for AMR path monitoring");

  // ---- Phase 2: per-robot lane closures on zone replan candidates --------
  private_closure_timeout_sec =
    declare_parameter<double>("private_closure_timeout_sec", 60.0);
  order_idle_gate_timeout_sec =
    declare_parameter<double>("order_idle_gate_timeout_sec", 3.0);
  order_idle_gate_warn_after_sec =
    declare_parameter<double>("order_idle_gate_warn_after_sec", 5.0);
  order_idle_gate_warn_repeat_sec =
    declare_parameter<double>("order_idle_gate_warn_repeat_sec", 10.0);

  robot_lane_request_pub = create_publisher<rmf_fleet_msgs::msg::RobotLaneRequest>(
    "/robot_lane_requests", rclcpp::QoS(rclcpp::KeepLast(50)).reliable());

  // Topic name must match zone_manager's `replan_candidate_topic` parameter
  // (its default is the SINGULAR "/replan_candidate"). Exposed as a parameter
  // here so the two cannot silently drift apart again.
  const std::string replan_candidate_topic =
    declare_parameter<std::string>("replan_candidate_topic", "/replan_candidate");

  auto replan_candidate_qos = rclcpp::QoS(rclcpp::KeepLast(50));
  replan_candidate_qos.reliable();
  replan_candidates_sub = create_subscription<zone_manager::msg::ReplanCandidate>(
    replan_candidate_topic,
    replan_candidate_qos,
    std::bind(
      &RMFIntegratedFleetAdapter::replan_candidate_callback, this, std::placeholders::_1));
  RCLCPP_INFO(
    get_logger(),
    "📡 [SUBSCRIPTION] Subscribed to %s; per-robot lane closures published on "
    "/robot_lane_requests, held until the leg ends (no timeout release)",
    replan_candidate_topic.c_str());

  // Zone sequences, used to find the zone entered just before a conflict zone
  // so the closure can block both. Best-effort: if this is missing the closure
  // falls back to the conflict zone alone.
  const std::string robot_zone_status_topic =
    declare_parameter<std::string>("robot_zone_status_topic", "/robot_zone_status");
  robot_zone_status_sub = create_subscription<monitor::msg::RobotZoneStatus>(
    robot_zone_status_topic,
    rclcpp::QoS(rclcpp::KeepLast(20)),
    std::bind(
      &RMFIntegratedFleetAdapter::robot_zone_status_callback, this,
      std::placeholders::_1));
  RCLCPP_INFO(
    get_logger(),
    "📡 [SUBSCRIPTION] Subscribed to %s for zone sequences (used to block the "
    "zone preceding a conflict)",
    robot_zone_status_topic.c_str());

  private_closure_timer = create_wall_timer(
    std::chrono::seconds(5),
    std::bind(&RMFIntegratedFleetAdapter::private_closure_timeout_tick, this));

  // ---- 4D / ASRS drop corridor --------------------------------------------
  drop_corridor_enabled_ =
      declare_parameter<bool>("drop_corridor_enabled", true);
  drop_corridor_release_distance_m_ =
      declare_parameter<double>("drop_corridor_release_distance_m", 0.5);
  drop_corridor_travel_start_m_ =
      declare_parameter<double>("drop_corridor_travel_start_m", 0.2);
  drop_corridor_plan_timeout_s_ =
      declare_parameter<double>("drop_corridor_plan_timeout_s", 30.0);
  // Defaults = A1462 RFID_Location_Sheet, "1500 loc G0" orange lines that lie
  // on the nav graph: X highways Z7 (y 28.35) and Z14 (y 21.00); Z highways
  // X11, X24, X46, X55 (x = (X-1)*1.45). Set a list to [] to infer it from
  // the graph geometry instead.
  drop_corridor_layout_.x_highway_ys = declare_parameter<std::vector<double>>(
      "drop_corridor_x_highway_ys", std::vector<double>{28.35, 21.00});
  drop_corridor_layout_.z_highway_xs = declare_parameter<std::vector<double>>(
      "drop_corridor_z_highway_xs",
      std::vector<double>{14.50, 33.35, 65.25, 78.30});
  drop_corridor_timer_ = create_wall_timer(
      std::chrono::milliseconds(200), [this]() { DropCorridorTick(); });
  RCLCPP_INFO(get_logger(),
              "🛣️ [DROP CORRIDOR] %s: when loaded, free all X highways + main "
              "Z spines + this robot's pick/drop Z; other pick/drop spurs "
              "blocked for this robot only (ECBS waits if peers share a drop); "
              "lanes freed without replan once drop is planned and travel "
              "starts (%.2fm) or within %.2fm of drop; no corridor plan "
              "within %.0fs -> freed and planned on the full graph",
              drop_corridor_enabled_ ? "enabled" : "disabled",
              drop_corridor_travel_start_m_,
              drop_corridor_release_distance_m_,
              drop_corridor_plan_timeout_s_);
  {
    const auto join = [](const std::vector<double>& v) {
        std::ostringstream ss;
        for (std::size_t i = 0; i < v.size(); ++i) {
          ss << (i ? "," : "") << v[i];
        }
        return v.empty() ? std::string("<infer from graph>") : ss.str();
      };
    RCLCPP_INFO(get_logger(),
                "🛣️ [DROP CORRIDOR] layout highways: X@y={%s} Z@x={%s} "
                "(Z highways free only between the X highways)",
                join(drop_corridor_layout_.x_highway_ys).c_str(),
                join(drop_corridor_layout_.z_highway_xs).c_str());
  }

  // Python lines 509–519: state dict init (members default-constructed above)

  // Python lines 522–528
  load_configuration();
  initialize_nav_graph_parser();
  setup_mqtt_client();

  // Python lines 533–536
  state_poll_timer = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(state_poll_interval)),
    std::bind(&RMFIntegratedFleetAdapter::poll_amr_states_tick, this));
  diagnostic_timer = create_wall_timer(
    std::chrono::seconds(10),
    std::bind(&RMFIntegratedFleetAdapter::diagnostic_check, this));

  // fms_sprint1: one-shot flush for TRANSIENT_LOCAL path/api backlog after subscriptions are ready
  startup_flush_timer = create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&RMFIntegratedFleetAdapter::flush_pending_after_startup, this));

  // Python lines 538–543
  RCLCPP_INFO(get_logger(), "🚀 Enhanced RMF Integrated Fleet Adapter (Full Control) initialized");
  RCLCPP_INFO(get_logger(), "Fleet name: %s", fleet_name.c_str());
  RCLCPP_INFO(
    get_logger(), "MQTT broker: %s:%d", mqtt_broker_host.c_str(), mqtt_broker_port);
  RCLCPP_INFO(get_logger(), "📍 Path Planning: ENABLED");
  RCLCPP_INFO(get_logger(), "🚚 Delivery Tasks: SUPPORTED");
  RCLCPP_INFO(
    get_logger(),
    "Full control fleet adapter will automatically add robots when it receives FleetState messages");
}

void RMFIntegratedFleetAdapter::diagnostic_check()
{
  // Section 15 — Python lines 3527–3544
  RCLCPP_INFO(
    get_logger(),
    "🔍 [DIAGNOSTIC] /task_api_responses messages received: %d",
    task_api_response_count);
  RCLCPP_INFO(
    get_logger(),
    "🔍 [DIAGNOSTIC] Active dispatch assignments: %zu",
    dispatch_assignments.size());
  if (!dispatch_assignments.empty()) {
    RCLCPP_INFO(
      get_logger(), "🔍 [DIAGNOSTIC] Dispatch assignments: %s",
      string_map_to_log_dict(dispatch_assignments).c_str());
  }
  if (task_api_sub) {
    RCLCPP_INFO(
      get_logger(),
      "🔍 [DIAGNOSTIC] /task_api_responses subscription active: true");
  } else {
    RCLCPP_WARN(get_logger(), "⚠️ [DIAGNOSTIC] /task_api_responses subscription is None!");
  }

  if (task_api_response_count == 0 && !dispatch_assignments.empty()) {
    RCLCPP_WARN(
      get_logger(),
      "⚠️ [DIAGNOSTIC] WARNING: Have dispatch assignments but NO /task_api_responses messages received!");
    RCLCPP_WARN(get_logger(), "⚠️ [DIAGNOSTIC] This might mean:");
    RCLCPP_WARN(get_logger(), "⚠️ [DIAGNOSTIC]   1. /task_api_responses topic is not publishing");
    RCLCPP_WARN(get_logger(), "⚠️ [DIAGNOSTIC]   2. QoS mismatch between publisher and subscriber");
    RCLCPP_WARN(
      get_logger(),
      "⚠️ [DIAGNOSTIC]   3. Messages were published before subscription was created");
    RCLCPP_WARN(
      get_logger(),
      "⚠️ [DIAGNOSTIC] Try: ros2 topic echo /task_api_responses "
      "--qos-reliability=reliable --qos-durability=transient_local");
  }
}

// ============================================================================
// Section 13 — Load + workcells (Python lines 3768–4091)
// ============================================================================

void RMFIntegratedFleetAdapter::handle_load_status_event(const AnyMap & event)
{
  // Python lines 3768–3998
  const std::string event_id = anymap_get_string(event, "id");
  const auto status_raw_it = event.find("status");
  const std::string timestamp = anymap_get_string(event, "timestamp");

  if (event_id.empty() || status_raw_it == event.end() || status_raw_it->second.IsNull()) {
    RCLCPP_WARN(get_logger(), "⚠️ [DELIVERY] Invalid load status event");
    return;
  }

  const std::optional<int> status_opt = parse_load_status_int(status_raw_it->second);
  if (!status_opt.has_value()) {
    RCLCPP_WARN(
      get_logger(), "⚠️ [DELIVERY] Invalid status value type for load status event");
    return;
  }
  const int status = *status_opt;

  const std::string event_key = event_id + "_" + std::to_string(status) + "_" + timestamp;
  if (processed_load_events.count(event_key) > 0) {
    RCLCPP_DEBUG(get_logger(), "🔄 [DELIVERY] Skipping duplicate load status event: %s", event_key.c_str());
    return;
  }
  processed_load_events.insert(event_key);
  trim_processed_event_set(processed_load_events);

  const std::string robot_id = anymap_get_string(event, "robot_id");
  const auto event_provided_opt = py_map_get_optional_string(event, "base_request_id");
  const bool has_event_provided =
    event_provided_opt.has_value() && !event_provided_opt->empty();
  const std::string base_request_id =
    has_event_provided ? *event_provided_opt : event_id;

  RCLCPP_INFO(
    get_logger(), "📦 [DELIVERY] Load status update: robot=%s, status=%d",
    robot_id.c_str(), status);

  if (!robot_id.empty()) {
    last_load_status_by_robot[robot_id] = status;
    const std::string load_status = monitored_load_status_text(status);
    if (!load_status.empty()) {
      monitored_task_manager->set_load_status(robot_id, load_status);
      const std::string target_task_id = resolve_monitored_task_id(robot_id);
      if (!target_task_id.empty()) {
        const std::string phase = monitored_phase_for_task(target_task_id, "");
        update_monitored_task_cache(
          target_task_id,
          {
            {"task_id", YAML::Node(target_task_id)},
            {"amr_id", YAML::Node(robot_id)},
            {"fleet_name", YAML::Node(fleet_name)},
            {"load_status", YAML::Node(load_status)},
            {"phase", YAML::Node(phase)},
          });
        if (monitored_task_manager->has_started(target_task_id)) {
          publish_monitored_task(target_task_id);
        }
      }
    }
    try_send_buffered_path_request(robot_id);

    const auto underway_it = currently_underway_task.find(robot_id);
    const std::string current_task_id =
      (underway_it != currently_underway_task.end()) ? underway_it->second : "";
    const auto nav_idle_it = nav_idle_by_robot.find(robot_id);
    const bool nav_is_idle =
      (nav_idle_it != nav_idle_by_robot.end()) ? nav_idle_it->second : false;

    if (
      status == 2 &&
      nav_is_idle &&
      !current_task_id.empty() &&
      delivery_task_details.count(current_task_id) > 0 &&
      (dropoff_in_progress.find(current_task_id) == dropoff_in_progress.end() ||
      !dropoff_in_progress.at(current_task_id)) &&
      delivery_wait_log_phase[robot_id] != "pickup")
    {
      delivery_wait_log_phase[robot_id] = "pickup";
      RCLCPP_INFO(get_logger(), "waiting for pallet to load the amr");
    } else if (
      status == 4 &&
      nav_is_idle &&
      !current_task_id.empty() &&
      dropoff_in_progress.find(current_task_id) != dropoff_in_progress.end() &&
      dropoff_in_progress.at(current_task_id) &&
      delivery_wait_log_phase[robot_id] != "drop")
    {
      delivery_wait_log_phase[robot_id] = "drop";
      RCLCPP_INFO(get_logger(), "waiting for pallet to unload the amr");
    } else if (status != 2 && status != 4) {
      delivery_wait_log_phase.erase(robot_id);
    }
  }

  if (status == 4) {
    if (has_event_provided) {
      const auto details_it = delivery_task_details.find(base_request_id);
      const bool drop_task =
        (details_it != delivery_task_details.end()) &&
        anymap_get_string(details_it->second, "pickup_handler", "") == "" &&
        (dropoff_in_progress.find(base_request_id) != dropoff_in_progress.end() &&
        dropoff_in_progress.at(base_request_id));
      if (drop_task) {
        RCLCPP_INFO(
          get_logger(),
          "📦 [DROP] Re-confirmed load_status=4 for drop task %s; no dispenser result "
          "needed (drop tasks have no pickup phase).",
          base_request_id.c_str());
      } else {
        // Flowchart: Pickup completed -> block every lane except the drop
        // corridor BEFORE the dispenser result lets RMF plan the drop leg.
        ApplyDropCorridorClosure(robot_id, base_request_id);
        publish_dispenser_result(base_request_id, event_id);
        dropoff_in_progress[base_request_id] = true;
        update_monitored_task_cache(
          base_request_id,
          {
            {"phase", YAML::Node("drop")},
            {"load_status", YAML::Node("loaded")},
          });
        if (monitored_task_manager->has_started(base_request_id)) {
          publish_monitored_task(base_request_id);
        }
        RCLCPP_INFO(
          get_logger(),
          "📦 [DELIVERY] Pickup complete for %s; expect drop segment via RMF PathRequest on VDA5050 order (no separate MQTT).",
          base_request_id.c_str());
        if (!robot_id.empty()) {
          try_send_buffered_path_request(robot_id);
        }
      }
    } else {
      const auto underway_it = currently_underway_task.find(robot_id);
      const std::string inferred_task =
        (underway_it != currently_underway_task.end()) ? underway_it->second : "";
      AnyMap details;
      if (!inferred_task.empty()) {
        const auto inferred_details_it = delivery_task_details.find(inferred_task);
        if (inferred_details_it != delivery_task_details.end()) {
          details = inferred_details_it->second;
        }
      }
      const std::string pickup_handler = anymap_get_string(details, "pickup_handler", "");
      const bool already_past_pickup =
        !inferred_task.empty() &&
        dropoff_in_progress.find(inferred_task) != dropoff_in_progress.end() &&
        dropoff_in_progress.at(inferred_task);

      const bool is_pickup_complete_recovery =
        !robot_id.empty() &&
        !inferred_task.empty() &&
        !pickup_handler.empty() &&
        !already_past_pickup;

      if (is_pickup_complete_recovery) {
        RCLCPP_WARN(
          get_logger(),
          "🚑 [DELIVERY RECOVERY] load_status=4 arrived for robot %s without a matching "
          "base_request_id (likely due to robot-side MQTT publisher restart). Recovering from "
          "FMS-side state — completing pickup for inferred task '%s'.",
          robot_id.c_str(), inferred_task.c_str());
        ApplyDropCorridorClosure(robot_id, inferred_task);
        publish_dispenser_result(inferred_task, event_id);
        dropoff_in_progress[inferred_task] = true;
        update_monitored_task_cache(
          inferred_task,
          {
            {"phase", YAML::Node("drop")},
            {"load_status", YAML::Node("loaded")},
          });
        if (monitored_task_manager->has_started(inferred_task)) {
          publish_monitored_task(inferred_task);
        }
        RCLCPP_INFO(
          get_logger(),
          "📦 [DELIVERY] Pickup complete for %s (via recovery); expect drop segment via RMF "
          "PathRequest on VDA5050 order (no separate MQTT).",
          inferred_task.c_str());
        if (!robot_id.empty()) {
          try_send_buffered_path_request(robot_id);
        }
      } else {
        RCLCPP_DEBUG(
          get_logger(),
          "📦 [DELIVERY] Load status 4 without base_request_id and no eligible delivery in "
          "pickup phase on robot %s — skipping dispenser publish (treating as readiness-only signal).",
          robot_id.c_str());
      }
    }
  } else if (status == 2) {
    bool is_dropoff_complete =
      has_event_provided &&
      (dropoff_in_progress.find(base_request_id) != dropoff_in_progress.end() &&
      dropoff_in_progress.at(base_request_id)) &&
      ((current_delivery_task_id.has_value() && *current_delivery_task_id == base_request_id) ||
      (currently_underway_task.find(robot_id) != currently_underway_task.end() &&
      currently_underway_task.at(robot_id) == base_request_id));

    const auto underway_it = currently_underway_task.find(robot_id);
    const std::string inferred_task =
      (underway_it != currently_underway_task.end()) ? underway_it->second : "";
    const bool is_dropoff_complete_recovery =
      !is_dropoff_complete &&
      !robot_id.empty() &&
      !inferred_task.empty() &&
      (dropoff_in_progress.find(inferred_task) != dropoff_in_progress.end() &&
      dropoff_in_progress.at(inferred_task));

    std::string effective_base_request_id = base_request_id;
    if (is_dropoff_complete_recovery) {
      RCLCPP_WARN(
        get_logger(),
        "🚑 [DELIVERY RECOVERY] load_status=2 arrived for robot %s without a matching "
        "base_request_id (likely due to robot-side MQTT publisher restart). Recovering from "
        "FMS-side state — completing dropoff for inferred task '%s'.",
        robot_id.c_str(), inferred_task.c_str());
      effective_base_request_id = inferred_task;
    }

    if (is_dropoff_complete || is_dropoff_complete_recovery) {
      // Python: publish IngestorResult immediately on load_status=2 after dropoff_in_progress.
      ReleaseDropCorridor(robot_id, "drop completed", /*replan=*/true);
      publish_ingestor_result(effective_base_request_id, event_id);
      RCLCPP_INFO(
        get_logger(),
        "✅ [DELIVERY QUEUE] Delivery task %s completed for robot %s",
        effective_base_request_id.c_str(), robot_id.c_str());
      dropoff_in_progress.erase(effective_base_request_id);
      pending_ingestor_by_robot.erase(robot_id);
      if (!delivery_task_queue.empty() &&
        delivery_task_queue.front().count("task_id") > 0 &&
        delivery_task_queue.front().at("task_id") == effective_base_request_id)
      {
        delivery_task_queue.erase(delivery_task_queue.begin());
      }
      if (current_delivery_task_id.has_value() &&
        *current_delivery_task_id == effective_base_request_id)
      {
        current_delivery_task_id = std::nullopt;
      }
      const auto robot_underway_it = currently_underway_task.find(robot_id);
      if (robot_underway_it != currently_underway_task.end() &&
        robot_underway_it->second == effective_base_request_id)
      {
        currently_underway_task.erase(robot_id);
      }
    } else {
      const bool dropoff_flag =
        dropoff_in_progress.find(base_request_id) != dropoff_in_progress.end() &&
        dropoff_in_progress.at(base_request_id);
      const std::string current_task_str =
        current_delivery_task_id.has_value() ? *current_delivery_task_id : "None";
      RCLCPP_INFO(
        get_logger(),
        "ℹ️ [DELIVERY] load_status == 2 received - treating as readiness value "
        "(dropoff_in_progress=%s, current_task=%s)",
        dropoff_flag ? "True" : "False",
        current_task_str.c_str());
      const auto underway_task_it = currently_underway_task.find(robot_id);
      if (underway_task_it != currently_underway_task.end() && !underway_task_it->second.empty()) {
        RCLCPP_INFO(
          get_logger(),
          "🔄 [DELIVERY] Robot %s load_status=2, retrying pickup for underway task %s",
          robot_id.c_str(), underway_task_it->second.c_str());
        send_pickup_for_underway_task(robot_id, underway_task_it->second);
      }
    }
  }
}

// ============================================================================
// Section 14 — Nav status (Python lines 4000–4036)
// ============================================================================

void RMFIntegratedFleetAdapter::handle_nav_status_event(const AnyMap & event)
{
  // Python lines 4000–4036
  try {
    const std::string robot_id = anymap_get_string(event, "robot_id");
    if (robot_id.empty()) {
      RCLCPP_WARN(get_logger(), "⚠️ [NAV STATUS] Event missing robot_id");
      return;
    }

    const std::string timestamp = anymap_get_string(event, "timestamp");
    const auto status_it = event.find("status");
    const std::string status_repr =
      (status_it != event.end()) ? event_scalar_to_string(status_it->second) : "None";
    const bool idle = py_map_get_bool(event, "idle", false);

    const std::string event_key =
      robot_id + "_" + status_repr + "_" + (idle ? "True" : "False") + "_" + timestamp;
    if (processed_nav_events.count(event_key) > 0) {
      RCLCPP_DEBUG(get_logger(), "🔄 [NAV STATUS] Skipping duplicate nav status event: %s", event_key.c_str());
      return;
    }
    processed_nav_events.insert(event_key);
    trim_processed_event_set(processed_nav_events);

    nav_idle_by_robot[robot_id] = idle;
    RCLCPP_INFO(
      get_logger(), "🔄 [NAV STATUS] Robot %s nav status updated: idle=%s, status=%s",
      robot_id.c_str(), idle ? "True" : "False", status_repr.c_str());

    if (idle) {
      const auto underway_it = currently_underway_task.find(robot_id);
      if (underway_it != currently_underway_task.end() && !underway_it->second.empty()) {
        RCLCPP_INFO(
          get_logger(),
          "🔄 [NAV STATUS] Robot %s is idle, retrying pickup for underway task %s",
          robot_id.c_str(), underway_it->second.c_str());
        send_pickup_for_underway_task(robot_id, underway_it->second);
      }
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "❌ [NAV STATUS] Failed to handle nav status event: %s", e.what());
  }
}

void RMFIntegratedFleetAdapter::publish_fleet_state()
{
  // Python lines 805–1034
  try {
    sequence_counter += 1;

    rmf_fleet_msgs::msg::FleetState fleet_state;
    fleet_state.name = fleet_name;
    fleet_state.robots.clear();

    for (auto & [robot_id, robot] : fleet_config->robots) {
      rmf_fleet_msgs::msg::RobotState robot_state;
      robot_state.name = robot_id;
      robot_state.model = "amr";
      const std::string publish_tid = robot_state_publish_task_id(robot);
      robot_state.task_id = publish_tid;
      robot_state.mode.mode = convert_status_to_mode(robot.status);
      robot_state.battery_percent = robot.battery_percentage;

      const auto cached_load_it = last_load_status_by_robot.find(robot_id);
      const bool has_known_load =
        (cached_load_it != last_load_status_by_robot.end()) &&
        cached_load_it->second.has_value() &&
        (*cached_load_it->second == 2 || *cached_load_it->second == 4);
      robot_state.load_status = has_known_load ? *cached_load_it->second : 0;

      const auto cached_op_it = last_operating_mode_by_robot.find(robot_id);
      const bool has_known_operating_mode =
        (cached_op_it != last_operating_mode_by_robot.end()) &&
        cached_op_it->second.has_value() &&
        (*cached_op_it->second == 0 || *cached_op_it->second == 1);
      robot_state.operating_mode =
        has_known_operating_mode ? static_cast<int16_t>(*cached_op_it->second) : static_cast<int16_t>(-1);

      // [SPEED] Robot velocity (m/s, +/-) from mqtt_client, copied unchanged into
      // /fleet_states for speed-based zone monitoring in the monitor node.
      robot_state.speed = static_cast<float>(robot.twist.linear);

      // [CHARGING] 1 / 0 / -1 (unknown). RMF finishes a charge task on 1 -> 0.
      robot_state.charging_status = static_cast<int16_t>(robot.charging_status);

      RCLCPP_DEBUG(
        get_logger(),
        "🆔 [FLEET STATE] Robot %s: Publishing task_id='%s' (booking='%s') to RMF",
        robot_id.c_str(),
        publish_tid.c_str(),
        robot.current_task.c_str());

      robot_state.seq = sequence_counter;

      rmf_fleet_msgs::msg::Location location;
      location.t = get_clock()->now();
      location.x = robot.pose.x;
      location.y = robot.pose.y;
      location.yaw = robot.pose.yaw;
      location.level_name = fleet_config->map_name;
      robot_state.location = location;

      advance_waypoint_sequence_if_progressed(robot_id, robot);

      const bool is_at_target = is_robot_at_target(robot_id, robot);
      RCLCPP_DEBUG(
        get_logger(), "🔍 [TARGET CHECK] Robot %s: at_target=%s, task='%s', status='%s'",
        robot_id.c_str(), is_at_target ? "True" : "False", robot.current_task.c_str(),
        robot_status_to_string_local(robot.status).c_str());

      if (!robot.current_task.empty() && is_at_target) {
        auto remaining_waypoints_it = robot_waypoint_sequences.find(robot_id);
        std::vector<AnyMap> remaining_waypoints =
          (remaining_waypoints_it != robot_waypoint_sequences.end()) ?
          remaining_waypoints_it->second : std::vector<AnyMap>{};
        RCLCPP_INFO(
          get_logger(), "🔍 [WAYPOINT COMPLETION] Robot %s: %zu remaining waypoints",
          robot_id.c_str(), remaining_waypoints.size());

        // ECBS WAIT: robot arrived at the traffic hold — stay put, keep the
        // task and path alive so full_control does not call check_path_finish.
        if (!remaining_waypoints.empty() &&
          anymap_is_traffic_hold(remaining_waypoints.front()))
        {
          robot_state.task_id = robot_state_publish_task_id(robot);
          robot_state.mode.mode = rmf_fleet_msgs::msg::RobotMode::MODE_WAITING;
          task_completion_delays.erase(robot_id);
          RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "⏸️  [TRAFFIC HOLD] Robot %s waiting at (%.3f, %.3f) — task %s held "
            "(blocker=%s, wait=%.1fs); not advancing / not completing",
            robot_id.c_str(),
            anymap_get_double(remaining_waypoints.front(), "x"),
            anymap_get_double(remaining_waypoints.front(), "y"),
            robot.current_task.c_str(),
            anymap_get_string(remaining_waypoints.front(), "traffic_blocker_amr", "").c_str(),
            anymap_get_double(remaining_waypoints.front(), "traffic_wait_sec", 0.0));
        } else if (remaining_waypoints.size() > 1 &&
          anymap_get_bool(remaining_waypoints.front(), "release", false))
        {
          AnyMap completed_waypoint = remaining_waypoints.front();
          remaining_waypoints.erase(remaining_waypoints.begin());
          robot_waypoint_sequences[robot_id] = remaining_waypoints;

          mark_waypoint_as_visited(
            robot_id,
            anymap_get_double(completed_waypoint, "x"),
            anymap_get_double(completed_waypoint, "y"));

          const AnyMap & next_waypoint = remaining_waypoints.front();
          // Next point may be unreleased (base end). Keep it as the hold
          // target so the robot does not look "arrived at destination".
          target_positions[robot_id] = {
            {"x", YAML::Node(anymap_get_double(next_waypoint, "x"))},
            {"y", YAML::Node(anymap_get_double(next_waypoint, "y"))},
          };

          RCLCPP_INFO(
            get_logger(),
            "🎯 [WAYPOINT SEQUENCE] Robot %s completed waypoint, moving to next: (%.6f, %.6f) - %zu waypoints remaining%s",
            robot_id.c_str(),
            anymap_get_double(next_waypoint, "x"),
            anymap_get_double(next_waypoint, "y"),
            remaining_waypoints.size(),
            anymap_get_bool(next_waypoint, "release", false) ? "" : " [awaiting base]");

          robot_state.task_id = robot_state_publish_task_id(robot);
        } else if (!remaining_waypoints.empty() &&
          !anymap_get_bool(remaining_waypoints.front(), "release", false))
        {
          // Released prefix finished; unreleased tail still open. Do not clear
          // the task or path — wait for the next horizon / base extension.
          robot_state.task_id = robot_state_publish_task_id(robot);
          robot_state.mode.mode = rmf_fleet_msgs::msg::RobotMode::MODE_WAITING;
          task_completion_delays.erase(robot_id);
          RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "⏳ [BASE EXTENSION] Robot %s at released base end — %zu waypoints "
            "remain (front unreleased); keeping task %s and path feedback",
            robot_id.c_str(), remaining_waypoints.size(),
            robot.current_task.c_str());
        } else if (remaining_waypoints.size() == 1) {
          AnyMap final_waypoint = remaining_waypoints.front();
          remaining_waypoints.clear();
          robot_waypoint_sequences[robot_id] = {};

          mark_waypoint_as_visited(
            robot_id,
            anymap_get_double(final_waypoint, "x"),
            anymap_get_double(final_waypoint, "y"));

          auto delay_it = task_completion_delays.find(robot_id);
          if (delay_it == task_completion_delays.end()) {
            task_completion_delays[robot_id] = {robot.current_task, 1};
            RCLCPP_INFO(
              get_logger(),
              "🎯 [WAYPOINT SEQUENCE] Robot %s reached final waypoint - starting completion delay (1/2 cycles) for task %s",
              robot_id.c_str(), robot.current_task.c_str());
          } else {
            auto & [task_id, cycles] = delay_it->second;
            if (task_id == robot.current_task) {
              cycles += 1;
              RCLCPP_INFO(
                get_logger(),
                "🎯 [WAYPOINT SEQUENCE] Robot %s at final waypoint - completion delay (%d/2 cycles) for task %s",
                robot_id.c_str(), cycles, task_id.c_str());
              if (cycles >= 2) {
                const double dist_to_goal = robot_distance_to_nav_goal(robot_id, robot);
                if (dist_to_goal > RMF_PATH_FINISH_TOLERANCE_M) {
                  RCLCPP_INFO(
                    get_logger(),
                    "⏳ [WAYPOINT SEQUENCE] Robot %s at final waypoint delay complete but "
                    "%.3fm from goal (RMF tolerance=%.2fm) — keeping task_id for full_control",
                    robot_id.c_str(), dist_to_goal, RMF_PATH_FINISH_TOLERANCE_M);
                  robot_state.task_id = robot_state_publish_task_id(robot);
                } else {
                const bool awaiting_drop = delivery_awaiting_drop_vda_order(task_id);
                if (awaiting_drop) {
                  RCLCPP_INFO(
                    get_logger(),
                    "⏳ [WAYPOINT SEQUENCE] Robot %s reached segment end but delivery drop "
                    "VDA not sent yet — keeping task %s active for RMF drop PathRequest",
                    robot_id.c_str(), task_id.c_str());
                  robot_state.task_id = robot_state_publish_task_id(robot);
                } else {
                RCLCPP_INFO(
                  get_logger(),
                  "🎯 [WAYPOINT SEQUENCE] Robot %s completed waypoint sequence - but keeping task active for potential next segment",
                  robot_id.c_str());
                RCLCPP_INFO(
                  get_logger(),
                  "🎯 [WAYPOINT SEQUENCE] Task %s may have more segments (pickup->drop), keeping active_task_robots mapping",
                  task_id.c_str());
                robot.current_task = "";
                robot.fleet_publish_task_id = "";
                robot_state.task_id = "";
                robot_state.mode.mode = rmf_fleet_msgs::msg::RobotMode::MODE_IDLE;
                robot_task_cleared[robot_id] = true;
                task_completion_delays.erase(robot_id);
                robot_waypoint_sequences.erase(robot_id);
                target_positions.erase(robot_id);
                }
                }
              }
            } else {
              robot_state.task_id = robot_state_publish_task_id(robot);
              RCLCPP_DEBUG(
                get_logger(),
                "🆔 [FLEET STATE] Robot %s: Keeping task_id during final delay",
                robot_id.c_str());
            }
          }
        } else {
          const double dist_to_goal = robot_distance_to_nav_goal(robot_id, robot);
          if (dist_to_goal > RMF_PATH_FINISH_TOLERANCE_M) {
            RCLCPP_WARN(
              get_logger(),
              "⚠️ [WAYPOINT SEQUENCE] Robot %s has no remaining waypoints but is %.3fm from "
              "goal (RMF tolerance=%.2fm) — keeping task_id active",
              robot_id.c_str(), dist_to_goal, RMF_PATH_FINISH_TOLERANCE_M);
            robot_state.task_id = robot_state_publish_task_id(robot);
          } else {
          const bool awaiting_drop = delivery_awaiting_drop_vda_order(robot.current_task);
          if (awaiting_drop) {
            RCLCPP_INFO(
              get_logger(),
              "⏳ [WAYPOINT SEQUENCE] Robot %s has no remaining waypoints but delivery drop "
              "VDA not sent yet — keeping task %s active for RMF drop PathRequest",
              robot_id.c_str(), robot.current_task.c_str());
            robot_state.task_id = robot_state_publish_task_id(robot);
          } else {
          RCLCPP_WARN(
            get_logger(),
            "⚠️ [WAYPOINT SEQUENCE] Robot %s has no remaining waypoints but task is still active",
            robot_id.c_str());
          robot.current_task = "";
          robot.fleet_publish_task_id = "";
          robot_state.task_id = "";
          robot_state.mode.mode = rmf_fleet_msgs::msg::RobotMode::MODE_IDLE;
          robot_task_cleared[robot_id] = true;
          task_completion_delays.erase(robot_id);
          robot_waypoint_sequences.erase(robot_id);
          target_positions.erase(robot_id);
          last_known_tasks[robot_id] = "";
          }
          }
        }
      } else {
        auto delay_it = task_completion_delays.find(robot_id);
        if (delay_it != task_completion_delays.end()) {
          RCLCPP_INFO(
            get_logger(),
            "🎯 Robot %s moved away from target - clearing delay for task %s",
            robot_id.c_str(),
            delay_it->second.first.c_str());
          task_completion_delays.erase(delay_it);
        }

        robot_state.task_id = robot_state_publish_task_id(robot);
        RCLCPP_DEBUG(
          get_logger(), "🆔 [FLEET STATE] Robot %s: Keeping task_id for navigation",
          robot_id.c_str());
      }

      RCLCPP_DEBUG(
        get_logger(), "🔍 [FLEET STATE DEBUG] Robot %s: task='%s', status='%s'",
        robot_id.c_str(), robot.current_task.c_str(), robot_status_to_string_local(robot.status).c_str());
      bool task_status_terminal = false;
      if (!robot.current_task.empty()) {
        const auto known_status_it = known_task_status.find(robot.current_task);
        if (known_status_it != known_task_status.end() &&
          is_terminal_task_status(known_status_it->second))
        {
          task_status_terminal = true;
        }
      }
      if (!robot.current_task.empty() &&
        !task_status_terminal &&
        (robot.status == RobotStatus::Moving || robot.status == RobotStatus::Navigating))
      {
        auto remaining_waypoints_it = robot_waypoint_sequences.find(robot_id);
        std::vector<AnyMap> remaining_waypoints =
          (remaining_waypoints_it != robot_waypoint_sequences.end()) ?
          remaining_waypoints_it->second : std::vector<AnyMap>{};
        RCLCPP_DEBUG(
          get_logger(), "🔍 [FLEET STATE DEBUG] Robot %s: %zu remaining waypoints",
          robot_id.c_str(), remaining_waypoints.size());

        robot_state.path.clear();
        if (!remaining_waypoints.empty()) {
          const std::size_t before_trim = remaining_waypoints.size();
          trim_waypoint_sequence_to_robot_progress(
            remaining_waypoints, robot_id, robot.pose.x, robot.pose.y);
          if (remaining_waypoints.size() != before_trim) {
            robot_waypoint_sequences[robot_id] = remaining_waypoints;
            RCLCPP_DEBUG(
              get_logger(),
              "✂️ [FLEET STATE] Robot %s trimmed path feedback %zu → %zu waypoints",
              robot_id.c_str(), before_trim, remaining_waypoints.size());
          }
          // Full remaining leg through destination (released=false included).
          // Stopping at the first unreleased node made fleet_state.path empty
          // at every base boundary and looked like the task was finished.
          for (const auto & waypoint : remaining_waypoints) {
            const bool wp_release = anymap_get_bool(waypoint, "release", false);
            rmf_fleet_msgs::msg::Location path_location;
            path_location.t = get_clock()->now();
            path_location.x = anymap_get_double(waypoint, "x");
            path_location.y = anymap_get_double(waypoint, "y");
            path_location.yaw = anymap_get_double(waypoint, "yaw", 0.0);
            path_location.level_name = fleet_config->map_name;
            path_location.release = wp_release;
            path_location.traffic_hold = anymap_get_bool(waypoint, "traffic_hold", false);
            path_location.traffic_wait_sec =
              anymap_get_double(waypoint, "traffic_wait_sec", 0.0);
            path_location.traffic_blocker_amr =
              anymap_get_string(waypoint, "traffic_blocker_amr", "");
            robot_state.path.push_back(path_location);
          }
          RCLCPP_DEBUG(
            get_logger(), "🛤️ [PATH REPORTING] Robot %s reporting %zu remaining waypoints to RMF",
            robot_id.c_str(), robot_state.path.size());
        } else {
          RCLCPP_DEBUG(
            get_logger(), "🛤️ [PATH REPORTING] Robot %s: No remaining waypoints - reporting empty path",
            robot_id.c_str());
        }
      } else {
        RCLCPP_DEBUG(
          get_logger(),
          "🔍 [FLEET STATE DEBUG] Robot %s: Falling into 'no active task' branch - task='%s', status='%s'",
          robot_id.c_str(), robot.current_task.c_str(), robot_status_to_string_local(robot.status).c_str());
        const bool path_was_assigned =
          robot_path_assigned.find(robot_id) != robot_path_assigned.end() && robot_path_assigned[robot_id];
        RCLCPP_DEBUG(
          get_logger(), "🔍 [PATH CLEARING] Robot %s: path_was_assigned=%s, task='%s'",
          robot_id.c_str(), path_was_assigned ? "True" : "False", robot.current_task.c_str());

        robot_state.path.clear();
        if (path_was_assigned && robot.current_task.empty()) {
          auto clear_it = robot_path_cleared.find(robot_id);
          if (clear_it == robot_path_cleared.end()) {
            robot_path_cleared[robot_id] = 1;
            RCLCPP_INFO(
              get_logger(),
              "🛤️ [FLEET STATE] Robot %s: Path was assigned and task cleared - empty path for completion (1/2 cycles)",
              robot_id.c_str());
          } else {
            clear_it->second += 1;
            const int cycles = clear_it->second;
            if (cycles >= 2) {
              robot_path_assigned[robot_id] = false;
              robot_path_cleared.erase(robot_id);
              RCLCPP_INFO(
                get_logger(),
                "🛤️ [FLEET STATE] Robot %s: Path clearing delay completed - resetting to normal state",
                robot_id.c_str());
            } else {
              RCLCPP_INFO(
                get_logger(), "🛤️ [FLEET STATE] Robot %s: Path clearing delay (%d/2 cycles)",
                robot_id.c_str(), cycles);
            }
          }
        } else {
          RCLCPP_DEBUG(
            get_logger(),
            "🛤️ [FLEET STATE] Robot %s: No active task and no path assigned - reporting empty path",
            robot_id.c_str());
        }
      }

      const auto pending_ingestor_it = pending_ingestor_by_robot.find(robot_id);
      if (pending_ingestor_it != pending_ingestor_by_robot.end()) {
        try_complete_dropoff(
          robot_id,
          pending_ingestor_it->second.task_id,
          pending_ingestor_it->second.event_id,
          true);
      }

      fleet_state.robots.push_back(robot_state);
    }

    fleet_states_pub->publish(fleet_state);
    RCLCPP_DEBUG(get_logger(), "Published FleetState with %zu robots", fleet_state.robots.size());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to publish fleet state: %s", e.what());
  }
}

bool RMFIntegratedFleetAdapter::is_robot_at_target(const std::string & robot_id, const AMRRobot & robot)
{
  // Python lines 1053–1074
  try {
    auto target_it = target_positions.find(robot_id);
    if (target_it == target_positions.end() || target_position_is_empty(target_it->second)) {
      RCLCPP_DEBUG(get_logger(), "🔍 [TARGET CHECK] Robot %s: No target position set", robot_id.c_str());
      return false;
    }

    const AnyMap & target = target_it->second;
    const double tx = anymap_get_double(target, "x");
    const double ty = anymap_get_double(target, "y");
    const double dx = robot.pose.x - tx;
    const double dy = robot.pose.y - ty;
    const double dist = std::sqrt(dx * dx + dy * dy);

    RCLCPP_DEBUG(
      get_logger(),
      "🔍 [TARGET CHECK] Robot %s: pos=(%.3f, %.3f), target=(%.3f, %.3f), dist=%.3fm",
      robot_id.c_str(), robot.pose.x, robot.pose.y, tx, ty, dist);

    const bool at_target = dist < ARRIVAL_THRESHOLD_M;
    if (at_target) {
      RCLCPP_INFO(get_logger(), "🎯 Robot %s reached target (distance: %.2fm)", robot_id.c_str(), dist);
    }
    return at_target;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Error checking robot target: %s", e.what());
    return false;
  }
}

double RMFIntegratedFleetAdapter::robot_distance_to_nav_goal(
  const std::string & robot_id, const AMRRobot & robot) const
{
  auto target_it = target_positions.find(robot_id);
  if (target_it != target_positions.end() && !target_position_is_empty(target_it->second)) {
    const double dx = robot.pose.x - anymap_get_double(target_it->second, "x");
    const double dy = robot.pose.y - anymap_get_double(target_it->second, "y");
    return std::hypot(dx, dy);
  }

  const auto remaining_it = robot_waypoint_sequences.find(robot_id);
  if (remaining_it != robot_waypoint_sequences.end() && !remaining_it->second.empty()) {
    const AnyMap & goal = remaining_it->second.back();
    const double dx = robot.pose.x - anymap_get_double(goal, "x");
    const double dy = robot.pose.y - anymap_get_double(goal, "y");
    return std::hypot(dx, dy);
  }

  return std::numeric_limits<double>::infinity();
}

bool RMFIntegratedFleetAdapter::is_robot_within_rmf_path_finish_tolerance(
  const std::string & robot_id, const AMRRobot & robot) const
{
  const double dist = robot_distance_to_nav_goal(robot_id, robot);
  return dist <= RMF_PATH_FINISH_TOLERANCE_M;
}

bool RMFIntegratedFleetAdapter::try_complete_dropoff(
  const std::string & robot_id,
  const std::string & base_request_id,
  const std::string & event_id,
  bool allow_defer)
{
  if (robot_id.empty() || base_request_id.empty()) {
    return false;
  }

  const auto status_it = known_task_status.find(base_request_id);
  if (status_it != known_task_status.end() && is_terminal_task_status(status_it->second)) {
    pending_ingestor_by_robot.erase(robot_id);
    return false;
  }

  if (!fleet_config.has_value() ||
    fleet_config->robots.find(robot_id) == fleet_config->robots.end())
  {
    return false;
  }

  const AMRRobot & robot = fleet_config->robots.at(robot_id);
  double dist_to_drop = robot_distance_to_nav_goal(robot_id, robot);

  const auto coords_it = task_pickup_drop_coords.find(base_request_id);
  if (coords_it != task_pickup_drop_coords.end()) {
    const auto dropoff_it = coords_it->second.find("dropoff");
    if (dropoff_it != coords_it->second.end()) {
      const YAML::Node & dropoff = dropoff_it->second;
      const double dx = robot.pose.x - yaml_node_double(dropoff["x"]);
      const double dy = robot.pose.y - yaml_node_double(dropoff["y"]);
      dist_to_drop = std::min(dist_to_drop, std::hypot(dx, dy));
    }
  }

  if (allow_defer && dist_to_drop > RMF_PATH_FINISH_TOLERANCE_M) {
    pending_ingestor_by_robot[robot_id] = {base_request_id, event_id};
    return false;
  }

  ReleaseDropCorridor(robot_id, "drop completed", /*replan=*/true);
  publish_ingestor_result(base_request_id, event_id);
  RCLCPP_INFO(
    get_logger(),
    "✅ [DELIVERY QUEUE] Delivery task %s completed for robot %s (dist_to_drop=%.3fm)",
    base_request_id.c_str(), robot_id.c_str(), dist_to_drop);
  finalize_robot_path_on_task_end(robot_id, base_request_id);
  dropoff_in_progress.erase(base_request_id);
  pending_ingestor_by_robot.erase(robot_id);
  if (!delivery_task_queue.empty() &&
    delivery_task_queue.front().count("task_id") > 0 &&
    delivery_task_queue.front().at("task_id") == base_request_id)
  {
    delivery_task_queue.erase(delivery_task_queue.begin());
  }
  if (current_delivery_task_id.has_value() &&
    *current_delivery_task_id == base_request_id)
  {
    current_delivery_task_id = std::nullopt;
  }
  const auto robot_underway_it = currently_underway_task.find(robot_id);
  if (robot_underway_it != currently_underway_task.end() &&
    robot_underway_it->second == base_request_id)
  {
    currently_underway_task.erase(robot_id);
  }
  return true;
}

void RMFIntegratedFleetAdapter::clear_delivery_completion_state(
  const std::string & robot_name, const std::string & task_id)
{
  dropoff_in_progress.erase(task_id);
  pending_ingestor_by_robot.erase(robot_name);
  delivery_task_details.erase(task_id);
  if (current_delivery_task_id.has_value() && *current_delivery_task_id == task_id) {
    current_delivery_task_id = std::nullopt;
  }
}

void RMFIntegratedFleetAdapter::finalize_robot_path_on_task_end(
  const std::string & robot_name, const std::string & task_id)
{
  // Python publish_fleet_state path-clearing flow: after navigation ends, clear
  // waypoint queue + current_task so FleetState reports empty path for 2 cycles
  // (robot_path_assigned delay). Without this, leftover waypoints keep the green
  // trajectory visible on the dashboard even when the task is completed.
  robot_waypoint_sequences.erase(robot_name);
  target_positions.erase(robot_name);
  task_completion_delays.erase(robot_name);

  if (!fleet_config.has_value() ||
    fleet_config->robots.find(robot_name) == fleet_config->robots.end())
  {
    return;
  }

  AMRRobot & robot = fleet_config->robots.at(robot_name);
  if (!task_id.empty() && robot.current_task != task_id) {
    return;
  }

  robot.current_task = "";
  robot.fleet_publish_task_id = "";
  robot.status = RobotStatus::Idle;
  robot_task_cleared[robot_name] = true;
  RCLCPP_INFO(
    get_logger(),
    "🛤️ [PATH FINALIZE] Robot %s: cleared nav path state for ended task %s "
    "(path_was_assigned=%s)",
    robot_name.c_str(),
    task_id.empty() ? "(any)" : task_id.c_str(),
    (robot_path_assigned.count(robot_name) > 0 && robot_path_assigned.at(robot_name)) ?
    "true" : "false");
}

bool RMFIntegratedFleetAdapter::is_task_executing_on_robot(
  const std::string & robot_name, const std::string & task_id) const
{
  if (robot_name.empty() || task_id.empty()) {
    return false;
  }

  if (fleet_config.has_value() &&
    fleet_config->robots.find(robot_name) != fleet_config->robots.end())
  {
    const std::string current_task = trim_copy(fleet_config->robots.at(robot_name).current_task);
    if (current_task == task_id) {
      return true;
    }
  }

  const auto underway_it = currently_underway_task.find(robot_name);
  if (underway_it != currently_underway_task.end() && underway_it->second == task_id) {
    return true;
  }

  const auto pending_it = pending_ingestor_by_robot.find(robot_name);
  if (pending_it != pending_ingestor_by_robot.end() && pending_it->second.task_id == task_id) {
    return true;
  }

  const auto drop_it = dropoff_in_progress.find(task_id);
  if (drop_it != dropoff_in_progress.end() && drop_it->second) {
    const auto assign_it = dispatch_assignments.find(task_id);
    if (assign_it != dispatch_assignments.end() && assign_it->second == robot_name) {
      return true;
    }
  }

  const auto active_it = active_task_robots.find(task_id);
  if (active_it != active_task_robots.end() && active_it->second == robot_name) {
    return true;
  }

  const auto assign_it = dispatch_assignments.find(task_id);
  if (assign_it != dispatch_assignments.end() && assign_it->second == robot_name) {
    const auto status_it = known_task_status.find(task_id);
    if (status_it != known_task_status.end()) {
      const std::string & st = trim_copy(status_it->second);
      if (st == "underway" || st == "queued") {
        return true;
      }
    }
  }

  return false;
}

int RMFIntegratedFleetAdapter::advance_waypoint_sequence_if_progressed(
  const std::string & robot_id,
  const AMRRobot & robot)
{
  // Python lines 1076–1172
  try {
    auto remaining_it = robot_waypoint_sequences.find(robot_id);
    if (remaining_it == robot_waypoint_sequences.end() || remaining_it->second.empty()) {
      return 0;
    }

    auto & remaining = remaining_it->second;
    const double rx = robot.pose.x;
    const double ry = robot.pose.y;
    int popped = 0;

    while (remaining.size() > 1) {
      const AnyMap & head = remaining.front();
      if (!anymap_get_bool(head, "release", false)) {
        break;
      }
      // Never advance past an ECBS waiting point while traffic_hold is set.
      if (anymap_is_traffic_hold(head)) {
        break;
      }
      const double dist = std::hypot(rx - anymap_get_double(head, "x"), ry - anymap_get_double(head, "y"));
      if (dist > ARRIVAL_THRESHOLD_M) {
        break;
      }
      const AnyMap wp = remaining.front();
      remaining.erase(remaining.begin());
      mark_waypoint_as_visited(robot_id, anymap_get_double(wp, "x"), anymap_get_double(wp, "y"));
      popped += 1;
      RCLCPP_INFO(
        get_logger(),
        "⏩ [WAYPOINT SWEEP-A] Robot %s popped waypoint (%.3f, %.3f) at dist %.3fm — %zu waypoints remaining",
        robot_id.c_str(), anymap_get_double(wp, "x"), anymap_get_double(wp, "y"), dist, remaining.size());
    }

    if (remaining.size() > 1) {
      bool has_traffic_hold = false;
      bool has_unreleased = false;
      for (const auto & wp : remaining) {
        if (anymap_is_traffic_hold(wp)) {
          has_traffic_hold = true;
        }
        if (!anymap_get_bool(wp, "release", false)) {
          has_unreleased = true;
        }
      }
      // Do not collapse through a waiting point — the robot must stop there.
      // Do not collapse an unreleased tail either: being near the eventual
      // destination must not erase the mid-leg feedback / base-extension wait.
      if (!has_traffic_hold && !has_unreleased) {
        const AnyMap & final_wp = remaining.back();
        const double dist_to_final = std::hypot(
          rx - anymap_get_double(final_wp, "x"), ry - anymap_get_double(final_wp, "y"));
        if (dist_to_final <= ARRIVAL_THRESHOLD_M) {
          int collapsed = 0;
          while (remaining.size() > 1) {
            const AnyMap wp = remaining.front();
            remaining.erase(remaining.begin());
            mark_waypoint_as_visited(robot_id, anymap_get_double(wp, "x"), anymap_get_double(wp, "y"));
            collapsed += 1;
          }
          popped += collapsed;
          RCLCPP_INFO(
            get_logger(),
            "⏭️  [WAYPOINT SWEEP-B] Robot %s reached final waypoint (%.3f, %.3f) at dist %.3fm — collapsed %d intermediate waypoints; sequence now [final-only]",
            robot_id.c_str(), anymap_get_double(final_wp, "x"), anymap_get_double(final_wp, "y"),
            dist_to_final, collapsed);
        }
      }
    }

    if (popped > 0) {
      robot_waypoint_sequences[robot_id] = remaining;
      if (!remaining.empty()) {
        target_positions[robot_id] = {
          {"x", YAML::Node(anymap_get_double(remaining.front(), "x"))},
          {"y", YAML::Node(anymap_get_double(remaining.front(), "y"))},
        };
      }
    }
    return popped;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "❌ [WAYPOINT SWEEP] Robot %s sweep error: %s", robot_id.c_str(), e.what());
    return 0;
  }
}

double RMFIntegratedFleetAdapter::calculate_distance(double x1, double y1, double x2, double y2) const
{
  const double dx = x1 - x2;
  const double dy = y1 - y2;
  return std::sqrt(dx * dx + dy * dy);
}

std::vector<rmf_fleet_msgs::msg::Location>
RMFIntegratedFleetAdapter::trim_path_from_robot_pose(
  const std::vector<rmf_fleet_msgs::msg::Location> & path,
  const double robot_x,
  const double robot_y,
  const double reached_m)
{
  if (path.empty()) {
    return {};
  }
  std::vector<rmf_fleet_msgs::msg::Location> out = path;
  while (out.size() > 1) {
    const double d0 = std::hypot(out.front().x - robot_x, out.front().y - robot_y);
    if (d0 > reached_m) {
      break;
    }
    out.erase(out.begin());
  }
  return out;
}

void RMFIntegratedFleetAdapter::trim_waypoint_sequence_from_pose(
  std::vector<AnyMap> & sequence,
  const std::string & robot_id,
  const double robot_x,
  const double robot_y,
  const double reached_m)
{
  while (sequence.size() > 1) {
    const AnyMap & head = sequence.front();
    if (anymap_is_traffic_hold(head)) {
      break;
    }
    const double d = std::hypot(
      robot_x - anymap_get_double(head, "x"),
      robot_y - anymap_get_double(head, "y"));
    if (d > reached_m) {
      break;
    }
    mark_waypoint_as_visited(
      robot_id,
      anymap_get_double(head, "x"),
      anymap_get_double(head, "y"));
    sequence.erase(sequence.begin());
  }
}

void RMFIntegratedFleetAdapter::trim_waypoint_sequence_to_robot_progress(
  std::vector<AnyMap> & sequence,
  const std::string & robot_id,
  const double robot_x,
  const double robot_y)
{
  if (sequence.empty()) {
    return;
  }

  constexpr double k_passed_epsilon_m = 0.08;

  auto dist_to_wp = [&](const AnyMap & wp) {
    return std::hypot(
      robot_x - anymap_get_double(wp, "x"),
      robot_y - anymap_get_double(wp, "y"));
  };

  // Pop waypoints the robot has reached or clearly passed (next wp is closer).
  // Never pop past an ECBS traffic hold, and never pop an unreleased waypoint
  // (that tail is destination feedback until the base is extended).
  bool progressed = true;
  while (progressed && sequence.size() > 1) {
    progressed = false;
    if (anymap_is_traffic_hold(sequence.front()) ||
      !anymap_get_bool(sequence.front(), "release", false))
    {
      break;
    }
    const double d0 = dist_to_wp(sequence.front());
    if (d0 <= ARRIVAL_THRESHOLD_M) {
      mark_waypoint_as_visited(
        robot_id,
        anymap_get_double(sequence.front(), "x"),
        anymap_get_double(sequence.front(), "y"));
      sequence.erase(sequence.begin());
      progressed = true;
      continue;
    }
    const double d1 = dist_to_wp(sequence[1]);
    if (d0 > d1 + k_passed_epsilon_m) {
      mark_waypoint_as_visited(
        robot_id,
        anymap_get_double(sequence.front(), "x"),
        anymap_get_double(sequence.front(), "y"));
      sequence.erase(sequence.begin());
      progressed = true;
    }
  }

  // Robot may be mid-leg while queue still starts at route origin (PathRequest reset).
  if (sequence.size() > 2 && !anymap_is_traffic_hold(sequence.front()) &&
    anymap_get_bool(sequence.front(), "release", false))
  {
    std::size_t nearest = 0;
    double nearest_d = dist_to_wp(sequence.front());
    for (std::size_t i = 1; i < sequence.size(); ++i) {
      if (anymap_is_traffic_hold(sequence[i]) ||
        !anymap_get_bool(sequence[i], "release", false))
      {
        // Cap the jump at the waiting point — do not skip past it.
        if (dist_to_wp(sequence[i]) + k_passed_epsilon_m < nearest_d) {
          nearest_d = dist_to_wp(sequence[i]);
          nearest = i;
        }
        break;
      }
      const double d = dist_to_wp(sequence[i]);
      if (d + k_passed_epsilon_m < nearest_d) {
        nearest_d = d;
        nearest = i;
      }
    }
    const double d_front = dist_to_wp(sequence.front());
    if (nearest > 0 && nearest_d + 1.0 < d_front) {
      for (std::size_t i = 0; i < nearest; ++i) {
        mark_waypoint_as_visited(
          robot_id,
          anymap_get_double(sequence[i], "x"),
          anymap_get_double(sequence[i], "y"));
      }
      sequence.erase(sequence.begin(), sequence.begin() + static_cast<std::ptrdiff_t>(nearest));
    }
  }

  // Only trim travelled released prefix; leave the unreleased destination tail.
  while (sequence.size() > 1) {
    const AnyMap & head = sequence.front();
    if (anymap_is_traffic_hold(head) ||
      !anymap_get_bool(head, "release", false))
    {
      break;
    }
    const double d = std::hypot(
      robot_x - anymap_get_double(head, "x"),
      robot_y - anymap_get_double(head, "y"));
    if (d > ARRIVAL_THRESHOLD_M) {
      break;
    }
    mark_waypoint_as_visited(
      robot_id,
      anymap_get_double(head, "x"),
      anymap_get_double(head, "y"));
    sequence.erase(sequence.begin());
  }
}

void RMFIntegratedFleetAdapter::sync_release_flags_on_sequence(
  std::vector<AnyMap> & sequence,
  const std::vector<rmf_fleet_msgs::msg::Location> & full_path,
  const double xy_match_m)
{
  for (auto & wp : sequence) {
    const double wx = anymap_get_double(wp, "x");
    const double wy = anymap_get_double(wp, "y");
    for (const auto & loc : full_path) {
      if (std::hypot(loc.x - wx, loc.y - wy) <= xy_match_m) {
        wp["release"] = YAML::Node(loc.release);
        wp["traffic_hold"] = YAML::Node(loc.traffic_hold);
        wp["traffic_wait_sec"] = YAML::Node(loc.traffic_wait_sec);
        wp["traffic_blocker_amr"] = YAML::Node(loc.traffic_blocker_amr);
        break;
      }
    }
  }
}

void RMFIntegratedFleetAdapter::append_missing_tail_waypoints(
  std::vector<AnyMap> & dst,
  const std::vector<AnyMap> & src,
  const double xy_match_m)
{
  for (const auto & swp : src) {
    const double sx = anymap_get_double(swp, "x");
    const double sy = anymap_get_double(swp, "y");
    bool found = false;
    for (const auto & dwp : dst) {
      if (std::hypot(
          sx - anymap_get_double(dwp, "x"),
          sy - anymap_get_double(dwp, "y")) <= xy_match_m)
      {
        found = true;
        break;
      }
    }
    if (!found) {
      dst.push_back(swp);
    }
  }
}

std::vector<rmf_fleet_msgs::msg::Location> RMFIntegratedFleetAdapter::store_and_filter_waypoints(
  const std::string & robot_id,
  const std::vector<rmf_fleet_msgs::msg::Location> & waypoints,
  double min_distance,
  bool filter_visited)
{
  // Python lines 1180–1262
  try {
    if (!fleet_config.has_value() || fleet_config->robots.find(robot_id) == fleet_config->robots.end()) {
      RCLCPP_WARN(
        get_logger(), "⚠️ [WAYPOINT STORAGE] Robot %s not found in fleet config",
        robot_id.c_str());
      return waypoints;
    }

    const auto & robot = fleet_config->robots[robot_id];
    const double current_x = robot.pose.x;
    const double current_y = robot.pose.y;

    std::vector<AnyMap> visited_waypoints;
    if (filter_visited) {
      auto visited_it = robot_visited_waypoints.find(robot_id);
      if (visited_it != robot_visited_waypoints.end()) {
        visited_waypoints = visited_it->second;
      }
    }

    std::vector<AnyMap> stored_waypoints;
    stored_waypoints.reserve(waypoints.size());
    for (std::size_t i = 0; i < waypoints.size(); ++i) {
      const auto & waypoint = waypoints[i];
      AnyMap waypoint_data{
        {"x", YAML::Node(waypoint.x)},
        {"y", YAML::Node(waypoint.y)},
        {"yaw", YAML::Node(waypoint.yaw)},
        {"index", YAML::Node(static_cast<int>(i))},
        {"release", YAML::Node(waypoint.release)},
        {"traffic_hold", YAML::Node(waypoint.traffic_hold)},
        {"traffic_wait_sec", YAML::Node(waypoint.traffic_wait_sec)},
        {"traffic_blocker_amr", YAML::Node(waypoint.traffic_blocker_amr)},
      };
      stored_waypoints.push_back(waypoint_data);
      RCLCPP_INFO(
        get_logger(), "📦 [WAYPOINT STORAGE] Stored waypoint %zu: (%.6f, %.6f, yaw=%.3f)",
        i + 1, waypoint.x, waypoint.y, waypoint.yaw);
    }

    std::vector<AnyMap> filtered_stored;
    std::vector<rmf_fleet_msgs::msg::Location> waypoints_to_send;
    filtered_stored.reserve(stored_waypoints.size());
    waypoints_to_send.reserve(stored_waypoints.size());

    for (std::size_t i = 0; i < stored_waypoints.size(); ++i) {
      const auto & waypoint_data = stored_waypoints[i];
      const double waypoint_x = anymap_get_double(waypoint_data, "x");
      const double waypoint_y = anymap_get_double(waypoint_data, "y");
      const double distance_from_current = calculate_distance(current_x, current_y, waypoint_x, waypoint_y);

      bool was_visited = false;
      if (filter_visited && !visited_waypoints.empty()) {
        for (const auto & visited_wp : visited_waypoints) {
          const double visited_x = anymap_get_double(visited_wp, "x");
          const double visited_y = anymap_get_double(visited_wp, "y");
          const double distance_from_visited =
            calculate_distance(visited_x, visited_y, waypoint_x, waypoint_y);
          if (distance_from_visited < min_distance) {
            was_visited = true;
            RCLCPP_INFO(
              get_logger(),
              "🚫 [WAYPOINT STORAGE] Waypoint (%.6f, %.6f) - REMOVED (previously visited at %.6f, %.6f)",
              waypoint_x, waypoint_y, visited_x, visited_y);
            break;
          }
        }
      }

      if (was_visited) {
        continue;
      }

      filtered_stored.push_back(waypoint_data);
      waypoints_to_send.push_back(waypoints[i]);
      RCLCPP_INFO(
        get_logger(),
        "✅ [WAYPOINT STORAGE] Waypoint (%.6f, %.6f) - distance from pose: %.2fm - KEPT",
        waypoint_x, waypoint_y, distance_from_current);
    }

    RCLCPP_INFO(
      get_logger(),
      "🔍 [WAYPOINT STORAGE] Stored %zu waypoints, filtered to %zu waypoints for robot %s",
      stored_waypoints.size(), filtered_stored.size(), robot_id.c_str());
    RCLCPP_INFO(
      get_logger(), "🔍 [VISITED TRACKING] Robot %s has %zu visited waypoints",
      robot_id.c_str(), visited_waypoints.size());

    std::size_t before_trim = waypoints_to_send.size();
    waypoints_to_send = trim_path_from_robot_pose(
      waypoints_to_send, current_x, current_y, ARRIVAL_THRESHOLD_M);
    if (waypoints_to_send.size() > 2) {
      std::size_t nearest = 0;
      double nearest_d = std::hypot(
        waypoints_to_send[0].x - current_x, waypoints_to_send[0].y - current_y);
      for (std::size_t i = 1; i < waypoints_to_send.size(); ++i) {
        const double d = std::hypot(
          waypoints_to_send[i].x - current_x, waypoints_to_send[i].y - current_y);
        if (d + 0.08 < nearest_d) {
          nearest_d = d;
          nearest = i;
        }
      }
      const double d_front = std::hypot(
        waypoints_to_send.front().x - current_x,
        waypoints_to_send.front().y - current_y);
      if (nearest > 0 && nearest_d + 1.0 < d_front) {
        waypoints_to_send.erase(
          waypoints_to_send.begin(),
          waypoints_to_send.begin() + static_cast<std::ptrdiff_t>(nearest));
      }
    }
    if (waypoints_to_send.size() != before_trim) {
      RCLCPP_INFO(
        get_logger(),
        "✂️ [WAYPOINT STORAGE] Trimmed %zu → %zu waypoints from robot pose for %s "
        "(fleet_state reports remaining path only)",
        before_trim, waypoints_to_send.size(), robot_id.c_str());
    }

    return waypoints_to_send;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      get_logger(), "❌ [WAYPOINT STORAGE] Error storing and filtering waypoints: %s", e.what());
    return waypoints;
  }
}

void RMFIntegratedFleetAdapter::mark_waypoint_as_visited(
  const std::string & robot_id, double waypoint_x, double waypoint_y)
{
  // Python lines 1264–1283
  try {
    auto & visited = robot_visited_waypoints[robot_id];
    AnyMap v{
      {"x", YAML::Node(waypoint_x)},
      {"y", YAML::Node(waypoint_y)},
      {"timestamp", YAML::Node(current_epoch_seconds())},
    };
    visited.push_back(v);
    RCLCPP_INFO(
      get_logger(), "✅ [VISITED TRACKING] Robot %s marked waypoint (%.6f, %.6f) as visited",
      robot_id.c_str(), waypoint_x, waypoint_y);
    RCLCPP_INFO(
      get_logger(), "🔍 [VISITED TRACKING] Robot %s now has %zu visited waypoints",
      robot_id.c_str(), visited.size());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "❌ [VISITED TRACKING] Error marking waypoint as visited: %s", e.what());
  }
}

int RMFIntegratedFleetAdapter::convert_status_to_mode(RobotStatus status) const
{
  // Python lines 1284–1294
  switch (status) {
    case RobotStatus::Idle: return rmf_fleet_msgs::msg::RobotMode::MODE_IDLE;
    case RobotStatus::Moving: return rmf_fleet_msgs::msg::RobotMode::MODE_MOVING;
    case RobotStatus::Navigating: return rmf_fleet_msgs::msg::RobotMode::MODE_MOVING;
    case RobotStatus::Charging: return rmf_fleet_msgs::msg::RobotMode::MODE_CHARGING;
    case RobotStatus::Error: return rmf_fleet_msgs::msg::RobotMode::MODE_ADAPTER_ERROR;
    case RobotStatus::Offline: return rmf_fleet_msgs::msg::RobotMode::MODE_ADAPTER_ERROR;
  }
  return rmf_fleet_msgs::msg::RobotMode::MODE_IDLE;
}

std::string RMFIntegratedFleetAdapter::robot_state_publish_task_id(const AMRRobot & robot) const
{
  // Python lines 1710–1712
  return trim_copy(robot.fleet_publish_task_id);
}

void RMFIntegratedFleetAdapter::publish_tf_frames(const std::map<std::string, AnyMap> & robots)
{
  // Python lines 1296–1326
  try {
    for (const auto & [robot_id, robot_data] : robots) {
      YAML::Node pose;
      const auto pose_it = robot_data.find("pose");
      if (pose_it != robot_data.end()) {
        pose = pose_it->second;
      }
      const double x = (pose && pose.IsMap() && pose["x"]) ? pose["x"].as<double>() : 4.65;
      const double y = (pose && pose.IsMap() && pose["y"]) ? pose["y"].as<double>() : -11.8;
      const double yaw = (pose && pose.IsMap() && pose["yaw"]) ? pose["yaw"].as<double>() : 0.0;

      geometry_msgs::msg::TransformStamped transform;
      transform.header.stamp = get_clock()->now();
      transform.header.frame_id = "map";
      transform.child_frame_id = robot_id + "/base_link";
      transform.transform.translation.x = x;
      transform.transform.translation.y = y;
      transform.transform.translation.z = 0.0;
      transform.transform.rotation.x = 0.0;
      transform.transform.rotation.y = 0.0;
      transform.transform.rotation.z = std::sin(yaw / 2.0);
      transform.transform.rotation.w = std::cos(yaw / 2.0);
      tf_broadcaster->sendTransform(transform);
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to publish TF frames: %s", e.what());
  }
}

void RMFIntegratedFleetAdapter::monitor_task_completion()
{
  // Section 15 — Python lines 4267–4289
  try {
    for (auto & [robot_id, robot] : fleet_config->robots) {
      const std::string & current_task = robot.current_task;
      const auto last_it = last_known_tasks.find(robot_id);
      const std::string last_task =
        (last_it != last_known_tasks.end()) ? last_it->second : "";

      if (!current_task.empty() && current_task != last_task) {
        last_known_tasks[robot_id] = current_task;
        RCLCPP_INFO(
          get_logger(), "🎯 [TASK TRACKING] Started tracking task %s for %s",
          current_task.c_str(), robot_id.c_str());
      } else if (current_task.empty() && !last_task.empty()) {
        RCLCPP_INFO(
          get_logger(), "🎯 [TASK COMPLETION] Task %s completed for robot %s",
          last_task.c_str(), robot_id.c_str());
        trigger_task_completion_callback(robot_id, last_task);
        last_known_tasks[robot_id] = "";
      }
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to monitor task completion: %s", e.what());
  }
}

void RMFIntegratedFleetAdapter::trigger_task_completion_callback(
  const std::string & robot_id, const std::string & task_id)
{
  // Section 15 — Python lines 4291–4297
  RCLCPP_INFO(
    get_logger(), "🎯 [TASK COMPLETION] Task %s completed for robot %s",
    task_id.c_str(), robot_id.c_str());
  RCLCPP_INFO(
    get_logger(),
    "📋 [TASK SUMMARIES] Task completion detected - summaries should now be published");
}

// ============================================================================
// Section 08 — Monitored task (Python lines 1919–2167) + monitor/monitor_logic.py
// ============================================================================

std::string RMFIntegratedFleetAdapter::monitored_load_status_text(int vda_load_status)
{
  // Python lines 1932–1938
  if (vda_load_status == 2) {
    return "Unloaded";
  }
  if (vda_load_status == 4) {
    return "loaded";
  }
  return "";
}

std::string RMFIntegratedFleetAdapter::monitored_phase_for_task(
  const std::string & task_id, const std::string & category) const
{
  // Python lines 1940–1952
  std::string cat = category;
  cat.erase(cat.begin(), std::find_if(cat.begin(), cat.end(), [](unsigned char c) { return !std::isspace(c); }));
  cat.erase(std::find_if(cat.rbegin(), cat.rend(), [](unsigned char c) { return !std::isspace(c); }).base(), cat.end());
  std::transform(cat.begin(), cat.end(), cat.begin(), ::tolower);

  if (cat == "patrol" || task_patrol_coords.count(task_id) > 0) {
    return "patrol";
  }
  if (cat == "drop") {
    return "drop";
  }
  if (task_pickup_drop_coords.count(task_id) > 0 || cat == "delivery") {
    const auto drop_it = dropoff_in_progress.find(task_id);
    const bool in_drop = (drop_it != dropoff_in_progress.end()) && drop_it->second;
    return in_drop ? "drop" : "pick";
  }
  return "";
}

std::string RMFIntegratedFleetAdapter::resolve_monitored_task_id(
  const std::string & robot_name, const std::string & candidate_task_id) const
{
  // Python lines 2085–2092
  return monitored_task_manager->resolve_task_id(
    robot_name, candidate_task_id, currently_underway_task, robot_to_coords_key);
}

void RMFIntegratedFleetAdapter::update_monitored_task_cache(
  const std::string & task_id, const AnyMap & updates)
{
  monitored_task_manager->update_task_cache(task_id, updates);
}

void RMFIntegratedFleetAdapter::publish_monitored_task(const std::string & task_id)
{
  // Python lines 1954–1956
  monitored_task_manager->publish_task(task_id);
}

std::string RMFIntegratedFleetAdapter::status_for_monitored_topic(const std::string & status) const
{
  // Python lines 1919–1921
  return monitored_task_manager->status_for_topic(status);
}

std::string RMFIntegratedFleetAdapter::priority_for_monitored_topic(
  const std::string & task_id, const AnyMap & booking) const
{
  // Python lines 1923–1925
  return monitored_task_manager->priority_for_topic(task_id, booking);
}

void RMFIntegratedFleetAdapter::handle_monitored_publish_rule(
  const std::string & task_id, const std::string & new_status)
{
  // Python lines 1958–1965
  const MonitoredPublishRuleResult result =
    monitored_task_manager->handle_publish_rule(task_id, new_status);
  if (result.should_flush_pending) {
    flush_pending_vda_orders(task_id);
  }
  if (result.reached_final) {
    pending_vda_orders_by_task.erase(task_id);
  }
}

void RMFIntegratedFleetAdapter::clear_monitored_task_state(const std::string & task_id)
{
  // Python lines 2080–2083
  monitored_task_manager->clear_task_state(task_id);
  pending_vda_orders_by_task.erase(task_id);
}

std::optional<std::string> RMFIntegratedFleetAdapter::extract_place_from_phase_detail(
  const std::string & detail_text)
{
  // Python lines 2094–2101
  if (detail_text.empty()) {
    return std::nullopt;
  }
  static const std::regex pattern(R"(place:([^\]\s]+))");
  std::smatch match;
  if (!std::regex_search(detail_text, match, pattern) || match.size() < 2) {
    return std::nullopt;
  }
  const std::string place = trim_copy(match[1].str());
  return place.empty() ? std::nullopt : std::optional<std::string>{place};
}

std::optional<std::string> RMFIntegratedFleetAdapter::extract_place_from_labels(
  const YAML::Node & labels, const std::string & key_name)
{
  // Python lines 2103–2108
  if (!labels || !labels.IsSequence()) {
    return std::nullopt;
  }
  const std::string prefix = key_name + "=";
  for (const auto & label : labels) {
    if (!label.IsScalar()) {
      continue;
    }
    try {
      const std::string label_text = label.as<std::string>();
      if (label_text.rfind(prefix, 0) == 0) {
        const std::string place = trim_copy(label_text.substr(prefix.size()));
        return place.empty() ? std::nullopt : std::optional<std::string>{place};
      }
    } catch (...) {
    }
  }
  return std::nullopt;
}

void RMFIntegratedFleetAdapter::record_task_phase_places(
  const std::string & task_id, const YAML::Node & task_state)
{
  // Python lines 2110–2146
  const YAML::Node phases = task_state["phases"] ? task_state["phases"] : YAML::Node(YAML::NodeType::Map);
  const YAML::Node booking = task_state["booking"] ? task_state["booking"] : YAML::Node(YAML::NodeType::Map);
  const YAML::Node labels = booking["labels"] ? booking["labels"] : YAML::Node(YAML::NodeType::Sequence);

  std::map<std::string, std::string> phase_places;
  if (phases && phases.IsMap()) {
    for (const auto & phase_entry : phases) {
      const YAML::Node phase_data = phase_entry.second;
      if (!phase_data || !phase_data.IsMap()) {
        continue;
      }
      std::string category;
      try {
        category = to_lower_ascii(phase_data["category"].as<std::string>());
      } catch (...) {
        category = "";
      }
      std::string detail_text;
      try {
        detail_text = phase_data["detail"].as<std::string>();
      } catch (...) {
        detail_text = "";
      }
      const auto place_opt = extract_place_from_phase_detail(detail_text);
      if (!place_opt.has_value()) {
        continue;
      }
      if (category.find("pick") != std::string::npos) {
        phase_places["pick"] = *place_opt;
      } else if (category.find("drop") != std::string::npos) {
        phase_places["drop"] = *place_opt;
      } else if (category.find("patrol") != std::string::npos) {
        phase_places["patrol"] = *place_opt;
      }
    }
  }

  if (phase_places.find("pick") == phase_places.end()) {
    const auto pickup_label = extract_place_from_labels(labels, "pickup");
    if (pickup_label.has_value()) {
      phase_places["pick"] = *pickup_label;
    }
  }
  if (phase_places.find("drop") == phase_places.end()) {
    const auto drop_label = extract_place_from_labels(labels, "destination");
    if (drop_label.has_value()) {
      phase_places["drop"] = *drop_label;
    }
  }
  if (phase_places.find("patrol") == phase_places.end()) {
    const auto patrol_label = extract_place_from_labels(labels, "destination");
    if (patrol_label.has_value()) {
      phase_places["patrol"] = *patrol_label;
    }
  }

  if (!phase_places.empty()) {
    task_phase_places[task_id] = phase_places;
  }
}

// ============================================================================
// Section 07 — Coord + task ID helpers (Python lines 1579–1890)
// ============================================================================

bool RMFIntegratedFleetAdapter::coords_match(
  double x1, double y1, double x2, double y2) const
{
  // Python lines 1579–1583
  return std::abs(x1 - x2) <= COORD_MATCH_EPSILON && std::abs(y1 - y2) <= COORD_MATCH_EPSILON;
}

bool RMFIntegratedFleetAdapter::is_dispatch_booking_id(const std::string & s) const
{
  // fms_sprint1: reject pure numeric RMF path segment ids; accept booking ids (delivery, patrol, UUID, …)
  if (s.empty()) {
    return false;
  }
  return !std::all_of(s.begin(), s.end(), [](unsigned char c) { return std::isdigit(c); });
}

bool RMFIntegratedFleetAdapter::is_patrol_dispatch_id(const std::string & task_id)
{
  const std::string tid = trim_copy(task_id);
  return tid.rfind("patrol.", 0) == 0;
}

// Recognise the uuid4 shape used as the task id for robot-targeted (direct)
// dispatch: 8-4-4-4-12 lowercase/uppercase hex with dashes.
bool RMFIntegratedFleetAdapter::is_uuid_task_id(const std::string & task_id)
{
  const std::string tid = trim_copy(task_id);
  if (tid.size() != 36) {
    return false;
  }
  for (std::size_t i = 0; i < tid.size(); ++i) {
    const char c = tid[i];
    if (i == 8 || i == 13 || i == 18 || i == 23) {
      if (c != '-') {
        return false;
      }
    } else if (!std::isxdigit(static_cast<unsigned char>(c))) {
      return false;
    }
  }
  return true;
}

void RMFIntegratedFleetAdapter::remember_task_category(
  const std::string & task_id, const std::string & category)
{
  const std::string tid = trim_copy(task_id);
  if (tid.empty() || category.empty()) {
    return;
  }
  const auto existing = task_category_by_id.find(tid);
  if (existing != task_category_by_id.end() && existing->second == category) {
    return;
  }
  task_category_by_id[tid] = category;
  RCLCPP_INFO(
    get_logger(), "🏷️ [TASK CATEGORY] Recorded '%s' for task %s",
    category.c_str(), tid.c_str());
}

// Resolve a task's category without relying on the id text.
//   1) explicit record from the task API response (works for uuid + dispatch ids)
//   2) "<category>.dispatch-" prefix (fleet dispatch safety net, legacy behaviour)
//   3) coordinate registries populated by the payload processors
std::string RMFIntegratedFleetAdapter::task_category_for(const std::string & task_id) const
{
  const std::string tid = trim_copy(task_id);
  if (tid.empty()) {
    return "";
  }

  const auto known = task_category_by_id.find(tid);
  if (known != task_category_by_id.end()) {
    return known->second;
  }

  if (tid.rfind("patrol.", 0) == 0) {
    return "patrol";
  }
  if (tid.rfind("delivery.", 0) == 0) {
    return "delivery";
  }
  if (tid.rfind("drop.", 0) == 0) {
    return "drop";
  }

  if (task_patrol_coords.count(tid) > 0) {
    return "patrol";
  }
  if (task_pickup_drop_coords.count(tid) > 0) {
    return "delivery";
  }
  return "";
}

bool RMFIntegratedFleetAdapter::is_patrol_task(const std::string & task_id) const
{
  return task_category_for(task_id) == "patrol";
}

void RMFIntegratedFleetAdapter::flush_pending_after_startup()
{
  if (startup_flush_done) {
    return;
  }
  startup_flush_done = true;

  std::set<std::string> robots;
  for (const auto & [robot_name, _] : pending_path_request_by_robot) {
    robots.insert(robot_name);
  }
  if (fleet_config.has_value()) {
    for (const auto & [robot_name, _] : fleet_config->robots) {
      robots.insert(robot_name);
    }
  }
  for (const std::string & robot_name : robots) {
    try_send_buffered_path_request(robot_name);
  }
}

std::pair<bool, std::string> RMFIntegratedFleetAdapter::official_dispatch_path_matches_booking(
  const std::string & path_dispatch_task_id,
  const std::string & coords_booking_id) const
{
  // Python lines 1818–1846
  const std::string dispatch_id = trim_copy(path_dispatch_task_id);
  const std::string booking_id = trim_copy(coords_booking_id);
  if (dispatch_id.empty()) {
    return {false, "dispatch_task_id is empty (not an official RMF task itinerary)"};
  }
  if (!is_dispatch_booking_id(dispatch_id)) {
    return {false, "dispatch_task_id '" + dispatch_id + "' is not a dispatch booking id"};
  }
  if (booking_id.empty()) {
    return {false, "no booking id from task_api_responses for coordinate lookup"};
  }
  if (dispatch_id != booking_id) {
    return {
      false,
      "dispatch_task_id '" + dispatch_id + "' does not match task_api booking '" + booking_id + "'"};
  }
  if (task_patrol_coords.find(dispatch_id) == task_patrol_coords.end() &&
    task_pickup_drop_coords.find(dispatch_id) == task_pickup_drop_coords.end())
  {
    return {false, "booking '" + dispatch_id + "' has no coords from task_api_responses yet"};
  }
  return {true, ""};
}

std::tuple<bool, std::string, int> RMFIntegratedFleetAdapter::patrol_path_matches_selected_places(
  const std::string & task_id,
  const YAML::Node & path_points,
  double current_robot_x,
  double current_robot_y,
  const bool skip_first_pose_check) const
{
  // Python lines 1585–1651
  //
  // SINGLE-PLACE PATROL. A patrol task targets exactly one destination (the
  // dashboard enforces one place / one round, and WCS does not create
  // multi-place patrols). The previous implementation walked a
  // task_patrol_progress cursor across a list of places and refused any path
  // once the cursor passed the end — which rejected every REPLANNED path for a
  // one-place patrol, because the first path had already consumed place 0.
  // There is no cursor any more: a path matches if it ends at the task's
  // destination, however many times it is re-planned.
  const auto patrol_it = task_patrol_coords.find(task_id);
  if (patrol_it == task_patrol_coords.end() || patrol_it->second.empty()) {
    return {false, "no patrol coords available for task " + task_id, -1};
  }
  if (!path_points || !path_points.IsSequence() || path_points.size() == 0) {
    return {false, "path has no waypoints", -1};
  }

  YAML::Node places_coords = anymap_get_yaml_node(patrol_it->second, "route_coordinates");
  if (!places_coords || !places_coords.IsSequence() || places_coords.size() == 0) {
    places_coords = anymap_get_yaml_node(patrol_it->second, "places_coordinates");
  }
  if (!places_coords || !places_coords.IsSequence() || places_coords.size() == 0) {
    return {false, "patrol route coordinates are empty", -1};
  }

  std::vector<std::pair<double, double>> expected;
  for (const auto & pc : places_coords) {
    if (!pc.IsMap()) {
      continue;
    }
    const YAML::Node coords = pc["coordinates"];
    expected.emplace_back(yaml_node_double(coords["x"]), yaml_node_double(coords["y"]));
  }
  if (expected.empty()) {
    return {false, "patrol route coordinates are empty", -1};
  }

  // The destination is the LAST configured place. For a single-place patrol
  // that is simply place 0; keeping the last-element form means a legacy
  // multi-place task still stored somewhere resolves to its final destination
  // rather than failing outright.
  const std::size_t dest_idx = expected.size() - 1;
  const auto [last_x, last_y] = path_point_xy(path_points[path_points.size() - 1]);
  if (!coords_match(last_x, last_y, expected[dest_idx].first, expected[dest_idx].second)) {
    std::ostringstream oss;
    oss << "last waypoint (" << last_x << ", " << last_y
        << ") does not match the patrol destination ("
        << expected[dest_idx].first << ", " << expected[dest_idx].second << ")";
    return {false, oss.str(), -1};
  }
  const int matched_j = static_cast<int>(dest_idx);

  const auto [first_x, first_y] = path_point_xy(path_points[0]);
  bool first_ok = coords_match(first_x, first_y, current_robot_x, current_robot_y);
  if (!first_ok && matched_j > 0) {
    first_ok = coords_match(
      first_x, first_y,
      expected[static_cast<std::size_t>(matched_j - 1)].first,
      expected[static_cast<std::size_t>(matched_j - 1)].second);
  }
  if (!skip_first_pose_check && !first_ok) {
    return {false, "first waypoint does not match current robot pose (or previous patrol place)", -1};
  }
  return {true, "", matched_j};
}

bool RMFIntegratedFleetAdapter::delivery_awaiting_drop_vda_order(
  const std::string & booking_id) const
{
  if (booking_id.empty() || task_pickup_drop_coords.count(booking_id) == 0) {
    return false;
  }
  const auto sent_it = sent_path_request_phase.find(booking_id);
  if (sent_it == sent_path_request_phase.end()) {
    return false;
  }
  return sent_it->second.count("pick") > 0 && sent_it->second.count("drop") == 0;
}

bool RMFIntegratedFleetAdapter::path_last_waypoint_matches_delivery_target(
  const std::string & coords_booking_id,
  double last_wp_x,
  double last_wp_y) const
{
  const auto delivery_coords_it = task_pickup_drop_coords.find(coords_booking_id);
  if (delivery_coords_it == task_pickup_drop_coords.end() ||
    delivery_coords_it->second.empty())
  {
    return false;
  }
  const bool in_drop_phase =
    dropoff_in_progress.find(coords_booking_id) != dropoff_in_progress.end() &&
    dropoff_in_progress.at(coords_booking_id);
  return coords_match(
    last_wp_x, last_wp_y,
    anymap_nested_coord(delivery_coords_it->second, in_drop_phase ? "dropoff" : "pickup", "x"),
    anymap_nested_coord(delivery_coords_it->second, in_drop_phase ? "dropoff" : "pickup", "y"));
}

bool RMFIntegratedFleetAdapter::should_send_path_request_by_coords(
  const std::string & coords_booking_id,
  double first_wp_x,
  double first_wp_y,
  double last_wp_x,
  double last_wp_y,
  double current_robot_x,
  double current_robot_y) const
{
  // Python lines 1653–1708
  bool first_matches_robot =
    coords_match(first_wp_x, first_wp_y, current_robot_x, current_robot_y);

  const auto delivery_coords_it = task_pickup_drop_coords.find(coords_booking_id);
  const bool has_delivery_coords =
    delivery_coords_it != task_pickup_drop_coords.end() &&
    !delivery_coords_it->second.empty();
  if (!first_matches_robot && has_delivery_coords) {
    const bool in_drop_phase =
      dropoff_in_progress.find(coords_booking_id) != dropoff_in_progress.end() &&
      dropoff_in_progress.at(coords_booking_id);
    if (in_drop_phase) {
      // After pickup the robot stands at the pickup stand; RMF may anchor the drop
      // segment on pickup coords rather than the instantaneous pose.
      first_matches_robot = coords_match(
        current_robot_x, current_robot_y,
        anymap_nested_coord(delivery_coords_it->second, "pickup", "x"),
        anymap_nested_coord(delivery_coords_it->second, "pickup", "y"));
    }
  }

  if (!first_matches_robot) {
    return false;
  }
  if (coords_booking_id.empty() || !is_dispatch_booking_id(coords_booking_id)) {
    RCLCPP_INFO(
      get_logger(),
      "⏭️ [PATH REQUEST] No dispatch booking id for coordinate validation yet "
      "(numeric RMF path segment id is not used as coordinate lookup key)");
    return false;
  }

  // Prefer delivery coords when both patrol and delivery entries exist for the same
  // booking (robot_task delivery misclassified as patrol leaves stale patrol coords).
  if (has_delivery_coords)
  {
    const bool in_drop_phase =
      dropoff_in_progress.find(coords_booking_id) != dropoff_in_progress.end() &&
      dropoff_in_progress.at(coords_booking_id);
    return coords_match(
      last_wp_x, last_wp_y,
      anymap_nested_coord(delivery_coords_it->second, in_drop_phase ? "dropoff" : "pickup", "x"),
      anymap_nested_coord(delivery_coords_it->second, in_drop_phase ? "dropoff" : "pickup", "y"));
  }

  const auto patrol_it = task_patrol_coords.find(coords_booking_id);
  if (patrol_it != task_patrol_coords.end() && !patrol_it->second.empty()) {
    const YAML::Node places_coords =
      anymap_get_yaml_node(patrol_it->second, "places_coordinates");
    if (places_coords && places_coords.IsSequence()) {
      for (const auto & pc : places_coords) {
        if (!pc.IsMap()) {
          continue;
        }
        const YAML::Node coords = pc["coordinates"];
        if (coords_match(last_wp_x, last_wp_y, yaml_node_double(coords["x"]),
          yaml_node_double(coords["y"])))
        {
          return true;
        }
      }
    }
    return false;
  }

  RCLCPP_INFO(
    get_logger(),
    "⏭️ [PATH REQUEST] No pickup/drop coords for booking %s yet - skipping until coords available",
    coords_booking_id.c_str());
  return false;
}

bool RMFIntegratedFleetAdapter::is_terminal_task_status(const std::string & status)
{
  // Python lines 1718–1721
  std::string s = trim_copy(status);
  std::transform(s.begin(), s.end(), s.begin(), ::tolower);
  return s == "completed" || s == "canceled" || s == "cancelled" || s == "failed" || s == "killed";
}

std::string RMFIntegratedFleetAdapter::path_request_dispatch_task_id(
  const rmf_fleet_msgs::msg::PathRequest & msg) const
{
  // Python lines 1714–1716
  return trim_copy(msg.dispatch_task_id);
}

bool RMFIntegratedFleetAdapter::clear_stale_local_cancel_status(
  const std::string & task_id, const std::string & reason)
{
  // Python lines 1723–1732
  const auto it = known_task_status.find(task_id);
  std::string prev = (it != known_task_status.end()) ? trim_copy(it->second) : "";
  std::transform(prev.begin(), prev.end(), prev.begin(), ::tolower);
  if (prev != "canceled" && prev != "cancelled") {
    return false;
  }
  known_task_status.erase(task_id);
  RCLCPP_INFO(get_logger(), "♻️ [TASK STATUS] Cleared stale local cancel for %s: %s", task_id.c_str(), reason.c_str());
  return true;
}

std::string RMFIntegratedFleetAdapter::resolve_booking_task_id_for_path_request(
  const std::string & robot_name,
  const std::string & path_task_id,
  const std::vector<rmf_fleet_msgs::msg::Location> * path,
  const std::string & dispatch_task_id)
{
  // Python lines 1734–1810
  const std::string dispatch_id = trim_copy(dispatch_task_id);
  if (is_dispatch_booking_id(dispatch_id)) {
    return dispatch_id;
  }

  const std::string tid = trim_copy(path_task_id);
  if (tid.empty()) {
    return "";
  }
  if (tid.find(".dispatch-") != std::string::npos ||
    tid.rfind("delivery.", 0) == 0 ||
    tid.rfind("patrol.", 0) == 0)
  {
    return tid;
  }

  std::optional<double> last_x;
  std::optional<double> last_y;
  if (path != nullptr && !path->empty()) {
    last_x = (*path)[path->size() - 1].x;
    last_y = (*path)[path->size() - 1].y;
  }

  std::vector<std::string> candidates;
  for (const auto & [bid, rname] : dispatch_assignments) {
    if (rname == robot_name) {
      candidates.push_back(bid);
    }
  }

  const auto rk_it = robot_to_coords_key.find(robot_name);
  const std::string rk = (rk_it != robot_to_coords_key.end()) ? rk_it->second : "";

  auto in_candidates = [&candidates](const std::string & id) {
      return std::find(candidates.begin(), candidates.end(), id) != candidates.end();
    };

  if (!rk.empty() && task_pickup_drop_coords.count(rk) > 0 && in_candidates(rk) && last_x.has_value()) {
    const AnyMap & cd = task_pickup_drop_coords.at(rk);
    const double pu_x = anymap_nested_coord(cd, "pickup", "x");
    const double pu_y = anymap_nested_coord(cd, "pickup", "y");
    const double dr_x = anymap_nested_coord(cd, "dropoff", "x");
    const double dr_y = anymap_nested_coord(cd, "dropoff", "y");
    if (coords_match(*last_x, *last_y, pu_x, pu_y) || coords_match(*last_x, *last_y, dr_x, dr_y)) {
      return rk;
    }
  }

  if (!rk.empty() && task_pickup_drop_coords.count(rk) > 0 && in_candidates(rk)) {
    return rk;
  }

  if (last_x.has_value()) {
    for (const auto & c : candidates) {
      if (task_patrol_coords.find(c) == task_patrol_coords.end()) {
        continue;
      }
      const AnyMap & pd = task_patrol_coords.at(c);
      const YAML::Node places_coords = anymap_get_yaml_node(pd, "places_coordinates");
      if (!places_coords || !places_coords.IsSequence()) {
        continue;
      }
      for (const auto & pc : places_coords) {
        if (!pc.IsMap()) {
          continue;
        }
        const YAML::Node coords = pc["coordinates"];
        if (coords_match(*last_x, *last_y, yaml_node_double(coords["x"]), yaml_node_double(coords["y"]))) {
          return c;
        }
      }
    }
  }

  for (const auto & c : candidates) {
    if (task_patrol_coords.count(c) > 0) {
      return c;
    }
  }
  for (const auto & c : candidates) {
    if (task_pickup_drop_coords.count(c) > 0) {
      return c;
    }
  }

  if (last_x.has_value()) {
    for (const auto & [c, pd] : task_patrol_coords) {
      if (!is_dispatch_booking_id(c)) {
        continue;
      }
      const YAML::Node places_coords = anymap_get_yaml_node(pd, "places_coordinates");
      if (!places_coords || !places_coords.IsSequence()) {
        continue;
      }
      for (const auto & pc : places_coords) {
        if (!pc.IsMap()) {
          continue;
        }
        const YAML::Node coords = pc["coordinates"];
        if (coords_match(*last_x, *last_y, yaml_node_double(coords["x"]), yaml_node_double(coords["y"]))) {
          return trim_copy(c);
        }
      }
    }
  }

  return is_dispatch_booking_id(tid) ? tid : "";
}

std::string RMFIntegratedFleetAdapter::resolve_coords_booking_id(
  const std::string & robot_name, const std::string & booking_from_path_resolve) const
{
  // Python lines 1848–1890
  const std::string cfp = trim_copy(booking_from_path_resolve);
  std::vector<std::string> ordered;
  auto append_unique = [&ordered](const std::string & value) {
      if (!value.empty() &&
        std::find(ordered.begin(), ordered.end(), value) == ordered.end())
      {
        ordered.push_back(value);
      }
    };

  if (is_dispatch_booking_id(cfp)) {
    ordered.push_back(cfp);
  }
  const auto rk_it = robot_to_coords_key.find(robot_name);
  const std::string rk = (rk_it != robot_to_coords_key.end()) ? trim_copy(rk_it->second) : "";
  if (is_dispatch_booking_id(rk)) {
    append_unique(rk);
  }
  const auto uw_it = currently_underway_task.find(robot_name);
  const std::string uw = (uw_it != currently_underway_task.end()) ? trim_copy(uw_it->second) : "";
  if (is_dispatch_booking_id(uw)) {
    append_unique(uw);
  }
  if (current_delivery_task_id.has_value() && is_dispatch_booking_id(*current_delivery_task_id)) {
    const auto dtd_it = delivery_task_details.find(*current_delivery_task_id);
    if (dtd_it != delivery_task_details.end() &&
      anymap_get_string(dtd_it->second, "robot_id") == robot_name)
    {
      append_unique(*current_delivery_task_id);
    }
  }
  for (const auto & [bid, rname] : dispatch_assignments) {
    const std::string bs = trim_copy(bid);
    if (rname == robot_name && is_dispatch_booking_id(bs)) {
      append_unique(bs);
    }
  }
  for (const auto & [key, _] : task_pickup_drop_coords) {
    const std::string ks = trim_copy(key);
    const auto details_it = delivery_task_details.find(key);
    if (is_dispatch_booking_id(ks) &&
      details_it != delivery_task_details.end() &&
      anymap_get_string(details_it->second, "robot_id") == robot_name)
    {
      append_unique(ks);
    }
  }
  for (const auto & [key, _] : task_patrol_coords) {
    const std::string ks = trim_copy(key);
    const auto assign_it = dispatch_assignments.find(key);
    if (is_dispatch_booking_id(ks) &&
      ((assign_it != dispatch_assignments.end() && assign_it->second == robot_name) || ks == cfp))
    {
      append_unique(ks);
    }
  }

  if (std::find(ordered.begin(), ordered.end(), cfp) != ordered.end()) {
    return cfp;
  }
  return ordered.empty() ? "" : ordered.front();
}

std::pair<bool, std::string> RMFIntegratedFleetAdapter::can_publish_vda_order_for_phase(
  const std::string & robot_name, const std::string & phase) const
{
  // Python lines 1988–2010
  const auto load_it = last_load_status_by_robot.find(robot_name);
  if (load_it == last_load_status_by_robot.end() || !load_it->second.has_value()) {
    return {false, "load_status unknown (waiting for VDA state topic)"};
  }
  const int load_status = *load_it->second;
  if (phase == "patrol") {
    return {true, ""};
  }
  if (phase == "pick") {
    if (load_status != 2) {
      return {false, "pick phase requires load_status=2, current=" + std::to_string(load_status)};
    }
    return {true, ""};
  }
  if (phase == "drop") {
    if (load_status != 4) {
      return {false, "drop phase requires load_status=4, current=" + std::to_string(load_status)};
    }
    return {true, ""};
  }
  return {true, ""};
}

void RMFIntegratedFleetAdapter::queue_vda_order_after_monitored(
  const std::string & task_id,
  const std::string & robot_name,
  const std::string & phase,
  const AnyMap & order_payload,
  const std::string & source,
  int phase_index,
  int path_revision)
{
  // Python lines 1967–1986
  if (task_id.empty()) {
    return;
  }
  YAML::Node payload_node(YAML::NodeType::Map);
  for (const auto & [key, value] : order_payload) {
    payload_node[key] = value;
  }
  pending_vda_orders_by_task[task_id].push_back({
    {"task_id", YAML::Node(task_id)},
    {"robot_name", YAML::Node(robot_name)},
    {"phase", YAML::Node(phase)},
    {"order_payload", payload_node},
    {"source", YAML::Node(source)},
    // Carried so flush_pending_vda_orders() can record which leg/revision was
    // actually published (used to detect a later replan of the same leg).
    {"phase_index", YAML::Node(phase_index)},
    {"path_revision", YAML::Node(path_revision)},
  });
}

bool RMFIntegratedFleetAdapter::robot_ready_for_order(const std::string & robot_name) const
{
  if (!fleet_config.has_value()) {
    return true;
  }
  const auto it = fleet_config->robots.find(robot_name);
  if (it == fleet_config->robots.end()) {
    return true;
  }
  // Moving / Navigating => the robot is still executing a goal. Everything else
  // (Idle, Charging, Error, Offline) is treated as "not driving", so a stuck or
  // faulted robot is never blocked from receiving a corrected order.
  const RobotStatus st = it->second.status;
  return st != RobotStatus::Moving && st != RobotStatus::Navigating;
}

std::string RMFIntegratedFleetAdapter::staged_path_key(
  int phase_index, int path_revision)
{
  return std::to_string(phase_index) + "|" + std::to_string(path_revision);
}

int RMFIntegratedFleetAdapter::vda_order_update_base(const std::string & phase)
{
  return (phase == "pick" || phase == "patrol") ? 0 : 1;
}

void RMFIntegratedFleetAdapter::apply_staged_planned_path(
  const std::string & robot_name,
  const std::string & order_id,
  int phase_index,
  int path_revision)
{
  const auto robot_it = staged_planned_path_by_robot.find(robot_name);
  if (robot_it == staged_planned_path_by_robot.end()) {
    return;
  }

  auto & by_key = robot_it->second;
  const auto key_it = by_key.find(staged_path_key(phase_index, path_revision));
  if (key_it == by_key.end()) {
    // No stage for this leg: the order came from a source that did not go
    // through the PathRequest staging path. Leave the previous planned_path
    // alone rather than reporting a route we cannot vouch for.
    return;
  }

  const StagedPlannedPath staged = key_it->second;

  // An order has now gone out for this robot, so every stage it was holding is
  // superseded — including any duplicate route RMF produced for the same leg.
  staged_planned_path_by_robot.erase(robot_it);

  if (!monitored_task_manager) {
    return;
  }

  monitored_task_manager->set_planned_path(robot_name, staged.path);

  bool published_to_topic = false;

  // ---- Which task does this path belong to? ------------------------------
  // order_id is the task of the order being published RIGHT NOW, so it is
  // correct by construction. The staged id is only what
  // resolve_monitored_task_id() returned when the path was STAGED, and at that
  // moment the robot's outgoing task can still be the one on record — giving a
  // stale id that is non-empty and therefore was not rescued by the old
  // "only if empty" fallback.
  //
  // Observed 2026-08-22: amr2's order for task 6a88c0ae published at
  // 1787343816.2628, while the staged id still said 32c1da40 — a task that had
  // already completed. has_started() was false for it, the publish was skipped,
  // and /Monitored_task_details kept amr2's previous WESTBOUND route. Its real
  // route was eastbound (-4.26 -> -3.09 -> -1.90 ...), so zone_monitor reported
  // amr2 as 8->7->6->5->4->3->2 while amr1 was genuinely 9->8->...->2. Both
  // looked like they were travelling the same way, the head-on was classified
  // as following, and no replan candidate was ever published.
  //
  // So prefer order_id; fall back to the staged id, then to a fresh resolve.
  std::string monitored_task_id = trim_copy(order_id);
  if (!monitored_task_id.empty() &&
    !staged.monitored_task_id.empty() &&
    staged.monitored_task_id != monitored_task_id)
  {
    RCLCPP_INFO(
      get_logger(),
      "🗺️ [PLANNED PATH] robot=%s: staged id '%s' is stale — this path belongs "
      "to the order being published, task '%s'",
      robot_name.c_str(), staged.monitored_task_id.c_str(),
      monitored_task_id.c_str());
  }
  if (monitored_task_id.empty()) {
    monitored_task_id = staged.monitored_task_id;
  }
  if (monitored_task_id.empty()) {
    monitored_task_id = resolve_monitored_task_id(robot_name, order_id);
    if (!monitored_task_id.empty()) {
      RCLCPP_INFO(
        get_logger(),
        "🗺️ [PLANNED PATH] robot=%s had no monitored task id when the path was "
        "staged; resolved '%s' at publish time",
        robot_name.c_str(), monitored_task_id.c_str());
    }
  }

  if (!monitored_task_id.empty()) {
    YAML::Node planned_path_node(YAML::NodeType::Sequence);
    for (const auto & p : staged.path) {
      planned_path_node.push_back(p);
    }
    update_monitored_task_cache(
      monitored_task_id,
      {
        {"task_id", YAML::Node(monitored_task_id)},
        {"amr_id", YAML::Node(robot_name)},
        {"fleet_name", YAML::Node(staged.fleet_name)},
        {"planned_path", planned_path_node},
      });

    // Updating the cache is NOT enough — /Monitored_task_details has no
    // periodic publisher, it is only emitted by publish_monitored_task().
    //
    // The queue sites call it in this order:
    //     publish_monitored_task(order_id);     <-- emits the OLD planned_path
    //     flush_pending_vda_orders(order_id);   <-- lands here, updates cache
    //
    // so without this republish the topic keeps whatever path it had BEFORE the
    // order went out, forever. Observed 2026-08-20 on zone 6: amr1's replanned
    // 41-point detour reached the robot and the cache, but /Monitored_task_details
    // still carried the original 16-point eastbound route, so zone_monitor kept
    // deriving amr1: 3->4->5->6->7->8->9 head-on into amr2's 9->8->...->2. The
    // conflict could never clear: both robots sat paused for 360 s, and when a
    // timeout finally resumed them the same stale path re-triggered the same
    // conflict 5 s later.
    if (monitored_task_manager->has_started(monitored_task_id)) {
      publish_monitored_task(monitored_task_id);
      published_to_topic = true;
    } else {
      RCLCPP_WARN(
        get_logger(),
        "⚠️ [PLANNED PATH] robot=%s: task '%s' is not marked started, so the new "
        "route was cached but NOT published on /Monitored_task_details — the zone "
        "layer is still working from the previous path",
        robot_name.c_str(), monitored_task_id.c_str());
    }
  }

  // Report what actually happened. This line used to claim "published"
  // unconditionally, including immediately after the WARN above said the
  // opposite — which made a skipped publish look like a successful one.
  if (published_to_topic) {
    RCLCPP_INFO(
      get_logger(),
      "🗺️ [PLANNED PATH] robot=%s published %zu points to "
      "/Monitored_task_details for task %s (full RMF path; "
      "filtered/remaining=%zu) — the robot now has this route, so the zone "
      "layer may act on it",
      robot_name.c_str(), staged.path.size(), monitored_task_id.c_str(),
      staged.filtered_count);
  } else {
    RCLCPP_WARN(
      get_logger(),
      "⚠️ [PLANNED PATH] robot=%s: %zu points were NOT published to "
      "/Monitored_task_details (task id '%s') — the zone layer still has the "
      "previous route for this robot",
      robot_name.c_str(), staged.path.size(),
      monitored_task_id.empty() ? "unknown" : monitored_task_id.c_str());
  }
}

void RMFIntegratedFleetAdapter::flush_pending_vda_orders(const std::string & task_id)
{
  // Python lines 2012–2078
  const auto pending_it = pending_vda_orders_by_task.find(task_id);
  if (pending_it == pending_vda_orders_by_task.end() || pending_it->second.empty()) {
    return;
  }
  if (!mqtt_client) {
    return;
  }

  std::vector<AnyMap> remaining;
  auto pending = pending_it->second;
  while (!pending.empty()) {
    AnyMap item = pending.front();
    pending.erase(pending.begin());
    const std::string robot_name = anymap_get_string(item, "robot_name");
    const std::string phase = anymap_get_string(item, "phase");
    AnyMap order_payload = yaml_map_to_anymap(anymap_get_yaml_node(item, "order_payload"));
    const std::string source = anymap_get_string(item, "source", "direct");
    const std::string order_id = anymap_get_string(order_payload, "orderId", task_id);

    const auto [can_publish, block_reason] = can_publish_vda_order_for_phase(robot_name, phase);
    if (!can_publish) {
      RCLCPP_INFO(
        get_logger(),
        "⏳ [VDA5050 ORDER] Holding publish for robot=%s, orderId=%s, phase=%s: %s",
        robot_name.c_str(), order_id.c_str(), phase.c_str(), block_reason.c_str());
      remaining.push_back(item);
      continue;
    }

    const int order_update_id = static_cast<int>(anymap_get_double(order_payload, "orderUpdateId"));
    std::optional<std::pair<double, double>> last_wp_xy;
    // Non-const: node 0 is rewritten in place at publish time (see below).
    YAML::Node nodes = anymap_get_yaml_node(order_payload, "nodes");

    // Never send an order with nothing to drive to. This matters now that
    // 'standby' can release a queued order: standby is reported earlier in a
    // task's life than 'underway', so we check the payload rather than trust
    // the status. An empty order is kept queued, not dropped, so that a later
    // flush can publish it once the waypoints are there.
    if (!nodes || !nodes.IsSequence() || nodes.size() == 0) {
      RCLCPP_WARN(
        get_logger(),
        "⏳ [VDA5050 ORDER] Holding publish for robot=%s, orderId=%s, phase=%s: "
        "order has no waypoints yet",
        robot_name.c_str(), order_id.c_str(), phase.c_str());
      remaining.push_back(item);
      continue;
    }

    if (nodes && nodes.IsSequence() && nodes.size() > 0) {
      const YAML::Node last_node = nodes[nodes.size() - 1];
      const YAML::Node last_node_pos = last_node["nodePosition"];
      last_wp_xy = std::make_pair(
        std::round(yaml_node_double(last_node_pos["x"]) * 100.0) / 100.0,
        std::round(yaml_node_double(last_node_pos["y"]) * 100.0) / 100.0);
    }

    const int item_path_revision_pre =
      static_cast<int>(anymap_get_double(item, "path_revision"));
    const int item_phase_index_pre =
      static_cast<int>(anymap_get_double(item, "phase_index"));
    std::size_t released_node_count_pre = 0;
    if (nodes && nodes.IsSequence()) {
      for (const auto & node : nodes) {
        if (node["released"] && node["released"].IsScalar()) {
          try {
            if (node["released"].as<bool>()) {
              ++released_node_count_pre;
            }
          } catch (...) {
          }
        }
      }
    }
    // Furthest released node of this payload, in map coordinates. This —
    // not the released-node count — is what identifies a base extension:
    // an update is re-anchored at the waypoint nearest the robot, so its
    // released count drops even as its base end moves forward (observed
    // 2026-10-21: amr2 extension 17 -> 16 released nodes while the base end
    // moved 24.40,53.50 -> 22.83,49.00; the count test failed, the idle gate
    // below held the update for ~20 s, and the robot stopped at the old
    // base end before driving on).
    std::optional<std::pair<double, double>> released_end_xy_pre;
    if (nodes && nodes.IsSequence()) {
      for (const auto & node : nodes) {
        if (node["released"] && node["released"].IsScalar()) {
          try {
            if (!node["released"].as<bool>()) {
              continue;
            }
          } catch (...) {
            continue;
          }
          const YAML::Node np = node["nodePosition"];
          if (np) {
            released_end_xy_pre = std::make_pair(
              yaml_node_double(np["x"]), yaml_node_double(np["y"]));
          }
        }
      }
    }
    const auto prev_order_pre_it = published_order_by_robot.find(robot_name);
    const bool released_end_advanced_pre =
      prev_order_pre_it != published_order_by_robot.end() &&
      prev_order_pre_it->second.has_released_end &&
      released_end_xy_pre.has_value() &&
      std::hypot(
        released_end_xy_pre->first - prev_order_pre_it->second.released_end_x,
        released_end_xy_pre->second - prev_order_pre_it->second.released_end_y) > 0.10;
    const bool release_extension_publish_pre =
      prev_order_pre_it != published_order_by_robot.end() &&
      prev_order_pre_it->second.order_id == order_id &&
      prev_order_pre_it->second.phase_index == item_phase_index_pre &&
      (released_end_advanced_pre ||
      (item_path_revision_pre > prev_order_pre_it->second.path_revision &&
      released_node_count_pre > prev_order_pre_it->second.released_node_count));

    // ---- Idle gate ------------------------------------------------------
    // Hold the order until the robot reports it has stopped. This is what makes
    // the replan sequence safe: cancelOrder is only 2-3 ms ahead of the order on
    // a different MQTT topic, and publishing the order clears the robot-side
    // cancel latch (/action_requests 2 -> 0) before cancel_current_goal_node
    // can sample the rising edge at 10 Hz — so the nav2 goal was never actually
    // cancelled and the robot kept driving its old route.
    //
    // STRICT: there is no timeout escape. An order is NEVER published into a
    // moving robot, however long that takes. The previous 3 s fall-through
    // published anyway "so the task cannot hang", and that is exactly what went
    // wrong on 2026-08-20: the cancelOrder did not stop amr1, the robot kept
    // driving at ~0.5 m/s for the whole hold, and at t+3 s the order was pushed
    // into a robot travelling at speed.
    //
    // The order is not dropped — it stays queued and publishes on the very next
    // tick after the robot reports idle. What replaces the timeout is noise: a
    // hold that stops being routine is escalated to WARN so a robot that never
    // stops is loud instead of silent.
    //
    // Traffic rolling extensions are order updates without cancelOrder; they
    // must reach the robot while it is still driving the previous segment.
    if (release_extension_publish_pre && !robot_ready_for_order(robot_name)) {
      RCLCPP_INFO(
        get_logger(),
        "🛤️ [VDA5050 ORDER] Base extension for robot=%s, orderId=%s publishing "
        "immediately while driving (new released base end) — no idle wait",
        robot_name.c_str(), order_id.c_str());
    }
    if (!release_extension_publish_pre && !robot_ready_for_order(robot_name)) {
      const rclcpp::Time now = get_clock()->now();
      auto held_it = order_hold_since_by_task.find(task_id);
      if (held_it == order_hold_since_by_task.end()) {
        order_hold_since_by_task[task_id] = now;
        held_it = order_hold_since_by_task.find(task_id);
      }
      const double held_s = (now - held_it->second).seconds();

      // Routine holds are quiet (this runs at 25 Hz). Past the escalation point
      // the hold is no longer normal — most likely the cancel never reached the
      // robot — so say so, and keep saying it periodically rather than once.
      auto & warned_at = order_hold_warned_at_by_task[task_id];
      const bool first_warning =
        held_s >= order_idle_gate_warn_after_sec && warned_at == 0.0;
      const bool repeat_warning =
        warned_at > 0.0 &&
        held_s - warned_at >= order_idle_gate_warn_repeat_sec;

      if (first_warning || repeat_warning) {
        warned_at = held_s;
        RCLCPP_WARN(
          get_logger(),
          "⚠️ [VDA5050 ORDER] Order for robot=%s, orderId=%s, phase=%s has been held "
          "%.0fs waiting for idle — the robot is still reporting that it is driving. "
          "A cancelOrder that never reached the robot looks exactly like this. The "
          "order stays queued and will publish the moment the robot stops.",
          robot_name.c_str(), order_id.c_str(), phase.c_str(), held_s);
      } else {
        RCLCPP_INFO(
          get_logger(),
          "⏸️ [VDA5050 ORDER] Holding order for robot=%s, orderId=%s, phase=%s: robot "
          "still driving (held %.1fs)",
          robot_name.c_str(), order_id.c_str(), phase.c_str(), held_s);
      }
      remaining.push_back(item);
      continue;
    }
    order_hold_since_by_task.erase(task_id);
    order_hold_warned_at_by_task.erase(task_id);

    // ---- Refresh node 0 -------------------------------------------------
    // The payload was built when RMF handed us the path; it is published only
    // now, once the robot is idle. Node 0 is not a routing decision — it is
    // "where the robot is" — so between those two moments it goes stale, and
    // the robot would be told to start from somewhere it no longer is.
    //
    // Measured on 2026-08-20: payload built with node 0 = (-1.594, -2.281),
    // published 3.0 s later with the robot at (-0.008, -2.280) — 1.59 m of
    // error handed to the robot. The strict gate above shrinks that window to
    // the robot's braking distance but does not close it, because the robot is
    // still moving when RMF plans and only stops afterwards.
    //
    // Nodes 1..N are RMF's route and are left exactly as planned. path_hash
    // already skips node 0, so refreshing it cannot affect duplicate detection.
    if (fleet_config.has_value()) {
      const auto robot_it = fleet_config->robots.find(robot_name);
      if (robot_it != fleet_config->robots.end() &&
        nodes && nodes.IsSequence() && nodes.size() > 0)
      {
        YAML::Node node0 = nodes[0];
        YAML::Node node0_pos = node0["nodePosition"];
        if (node0_pos && node0_pos.IsMap()) {
          const double stale_x = yaml_node_double(node0_pos["x"]);
          const double stale_y = yaml_node_double(node0_pos["y"]);
          const double live_x = robot_it->second.pose.x;
          const double live_y = robot_it->second.pose.y;
          const double drift =
            std::hypot(live_x - stale_x, live_y - stale_y);

          node0_pos["x"] = live_x;
          node0_pos["y"] = live_y;
          node0_pos["theta"] = robot_it->second.pose.yaw;

          // ---- Re-derive the spin action against the refreshed pose --------
          // The spin was computed in build_vda5050_order_payload() from the
          // PLANNED node 0. Node 0 has just been rewritten to where the robot
          // actually is, so both of the spin's inputs have moved: `yaw` is the
          // robot's real heading now, and the travel bearing is measured from
          // the new node 0 to node 1. Leaving the built-time action in place
          // would tell the robot to turn from a heading it no longer has.
          //
          // Recomputed rather than patched, using the same compute_spin_action()
          // as the build path, so the two can never drift apart. Any previously
          // injected spin is dropped first; the leg's pick/drop/patrol action on
          // node 0 (only present on a single-node path) is preserved, and the
          // spin is placed ahead of it so the robot squares up before acting.
          if (nodes.size() >= 2) {
            const YAML::Node n1_pos = nodes[1]["nodePosition"];
            const double dx = yaml_node_double(n1_pos["x"]) - live_x;
            const double dy = yaml_node_double(n1_pos["y"]) - live_y;

            YAML::Node kept_actions(YAML::NodeType::Sequence);
            std::optional<YAML::Node> fresh_spin;
            if (std::hypot(dx, dy) > 1e-3) {
              fresh_spin = compute_spin_action(
                robot_name, robot_it->second.pose.yaw, std::atan2(dy, dx),
                static_cast<int>(yaml_node_double(node0["sequenceId"])));
            }
            if (fresh_spin.has_value()) {
              kept_actions.push_back(*fresh_spin);
            }
            const YAML::Node existing_actions = node0["actions"];
            if (existing_actions && existing_actions.IsSequence()) {
              for (const auto & act : existing_actions) {
                if (act["actionType"] &&
                  act["actionType"].as<std::string>("") == "spin")
                {
                  continue;   // stale, computed against the planned pose
                }
                kept_actions.push_back(act);
              }
            }
            node0["actions"] = kept_actions;
          }

          // Only worth reporting when it actually moved; sub-centimetre drift
          // is normal pose noise and would drown the log at 25 Hz.
          if (drift > 0.05) {
            RCLCPP_INFO(
              get_logger(),
              "📍 [VDA5050 ORDER] Refreshed node 0 for robot=%s, orderId=%s: "
              "(%.3f, %.3f) -> (%.3f, %.3f), %.2fm stale while the order waited "
              "for idle",
              robot_name.c_str(), order_id.c_str(),
              stale_x, stale_y, live_x, live_y, drift);
          }

          // nodes is a handle into the payload's node tree, but write the
          // sequence back explicitly so the change cannot depend on YAML::Node
          // aliasing semantics.
          order_payload["nodes"] = nodes;
        }
      }
    }

    // Hash every node position so an identical route is recognised even when
    // orderUpdateId has moved on (RMF bumps path_revision per replan attempt,
    // including attempts that return the same route).
    // Skip node 0: it is the ROBOT'S CURRENT POSE, not a routing decision. RMF
    // anchors every path at wherever the robot happens to be, so including it
    // made an unchanged route look new as soon as the robot had moved a few
    // centimetres (observed: 73 mm drift over 83 s produced a second identical
    // order). Nodes 1..N are the actual route.
    std::size_t path_hash = 0;
    if (nodes && nodes.IsSequence()) {
      std::size_t node_i = 0;
      for (const auto & node : nodes) {
        const std::size_t this_i = node_i++;
        if (this_i == 0) {
          continue;
        }
        const YAML::Node np = node["nodePosition"];
        if (!np) {
          continue;
        }
        const long long qx =
          static_cast<long long>(std::llround(yaml_node_double(np["x"]) * 1000.0));
        const long long qy =
          static_cast<long long>(std::llround(yaml_node_double(np["y"]) * 1000.0));
        path_hash = path_hash * 1000003u + static_cast<std::size_t>(qx);
        path_hash = path_hash * 1000003u + static_cast<std::size_t>(qy);
      }
    }

    // ---- Revision gate ---------------------------------------------------
    // Publishing requires BOTH a higher path_revision AND a changed route.
    // This half checks the revision; the fingerprint compare below checks the
    // route.
    //
    // Why the revision alone is not enough: RMF bumps path_revision on every
    // replan attempt, including attempts that return the identical route
    // (observed: updateId 31..37, byte-identical 7-node path).
    //
    // Why the route alone is not enough: RMF's traffic negotiation reroutes a
    // robot WITHOUT bumping path_revision (observed: 'Resolved negotiation [0]'
    // at 1787179078.4866, followed 1.6 ms later by a second follow_new_path for
    // the same task/phase at revision 0 with a completely different route).
    // Route-only suppression let that through as a second orderUpdateId 0.
    //
    // The same comparison already exists in is_replan_of_published_order(), but
    // that function answers "may this path BYPASS the normal filters?" — when it
    // says no, the path still falls through to ordinary handling, which had no
    // revision condition at all. This is that condition, at the publish point.
    //
    // Only a previous order for the SAME task and SAME leg is a valid baseline:
    // a new task, or the next phase of a delivery, starts its own revision line
    // and must always publish.
    const int item_path_revision =
      static_cast<int>(anymap_get_double(item, "path_revision"));
    const int item_phase_index =
      static_cast<int>(anymap_get_double(item, "phase_index"));

          std::size_t released_node_count = 0;
    if (nodes && nodes.IsSequence()) {
      for (const auto & node : nodes) {
        if (node["released"] && node["released"].IsScalar()) {
          try {
            if (node["released"].as<bool>()) {
              ++released_node_count;
            }
          } catch (...) {
          }
        }
      }
    }

    // Same released-end test as the idle gate above (see there for why a
    // node count cannot be used). A base extension must publish even if a
    // resubmit arrives with an un-advanced path_revision.
    const bool released_end_advanced =
      prev_order_pre_it != published_order_by_robot.end() &&
      released_end_advanced_pre;

    const auto prev_order_it = published_order_by_robot.find(robot_name);
    if (prev_order_it != published_order_by_robot.end()) {
      const PublishedOrder & prev_order = prev_order_it->second;
      if (prev_order.order_id == order_id &&
        prev_order.phase_index == item_phase_index &&
        item_path_revision <= prev_order.path_revision &&
        !released_end_advanced)
      {
        RCLCPP_INFO(
          get_logger(),
          "🛡️ [VDA5050 ORDER] Suppressed publish for robot=%s, orderId=%s, phase=%s, "
          "source=%s: path_revision %d has not advanced past the published %d "
          "(leg %d, %zu nodes) — no replan was requested for this route, so the "
          "robot keeps the order it already has",
          robot_name.c_str(), order_id.c_str(), phase.c_str(), source.c_str(),
          item_path_revision, prev_order.path_revision, item_phase_index,
          nodes && nodes.IsSequence() ? nodes.size() : 0);
        continue;
      }
    }

    const bool release_extension_publish =
      prev_order_it != published_order_by_robot.end() &&
      prev_order_it->second.order_id == order_id &&
      prev_order_it->second.phase_index == item_phase_index &&
      (released_end_advanced ||
      (item_path_revision > prev_order_it->second.path_revision &&
      released_node_count > prev_order_it->second.released_node_count));

    const VdaOrderFingerprint fingerprint{
      order_id, order_update_id, phase, last_wp_xy, path_hash};
    const auto prev_it = last_published_vda_order.find(robot_name);
    if (!release_extension_publish &&
      prev_it != last_published_vda_order.end() && prev_it->second == fingerprint) {
      // Record the revision we just declined to publish.
      //
      // published_order_by_robot used to be written ONLY on a successful
      // publish, so a suppressed route left the baseline frozen at the last
      // revision that made it out. Every later PathRequest was then measured
      // against that stale number, re-qualified as a replan in
      // is_replan_of_published_order(), and fired another cancelOrder — which
      // was suppressed here again, which left the baseline frozen again.
      //
      // Measured 2026-09-22 (log2): baseline stuck at 0 while revisions climbed
      // 1..12, one cancelOrder every 10 s for 176 s, the robot holding no order
      // at all after the first one. The route really is unchanged, so the robot
      // is correctly still driving it — the baseline must say so.
      const auto rec_it = published_order_by_robot.find(robot_name);
      if (rec_it != published_order_by_robot.end() &&
        rec_it->second.order_id == order_id &&
        rec_it->second.phase_index == item_phase_index &&
        item_path_revision > rec_it->second.path_revision)
      {
        rec_it->second.path_revision = item_path_revision;
      }

      RCLCPP_INFO(
        get_logger(),
        "🛡️ [VDA5050 ORDER] Suppressed duplicate publish for robot=%s, orderId=%s, "
        "phase=%s, source=%s: route unchanged (orderUpdateId %d, %zu nodes) — RMF "
        "replanned but returned the same path; revision baseline advanced to %d "
        "so this does not re-qualify as a replan",
        robot_name.c_str(), order_id.c_str(), phase.c_str(), source.c_str(),
        order_update_id, nodes && nodes.IsSequence() ? nodes.size() : 0,
        item_path_revision);
      continue;
    }
    // ---- State-ack gate -------------------------------------------------
    // Robot VDA state already echoes this orderId + orderUpdateId (or a newer
    // update). Republishing re-anchors node 0 at live pose and can reinsert
    // already-passed corners as stale remaining nodes. If feedback already
    // shows the required ids, treat as delivered — no MQTT republish.
    {
      const auto robot_states_now = mqtt_client->get_robot_states();
      const auto state_it = robot_states_now.find(robot_name);
      if (state_it != robot_states_now.end() &&
        state_it->second.find("vda_order_update_id") != state_it->second.end())
      {
        const std::string state_order_id =
          anymap_get_string(state_it->second, "current_task");
        const int state_update_id = static_cast<int>(
          anymap_get_double(state_it->second, "vda_order_update_id"));
        if (!state_order_id.empty() && state_order_id == order_id &&
          state_update_id >= order_update_id)
        {
          last_published_vda_order[robot_name] = fingerprint;

          PublishedOrder published;
          published.order_id = order_id;
          published.phase = phase;
          published.phase_index = item_phase_index;
          published.path_revision = item_path_revision;
          published.released_node_count = released_node_count;
          if (released_end_xy_pre.has_value()) {
            published.released_end_x = released_end_xy_pre->first;
            published.released_end_y = released_end_xy_pre->second;
            published.has_released_end = true;
          }
          if (last_wp_xy.has_value()) {
            published.dest_x = last_wp_xy->first;
            published.dest_y = last_wp_xy->second;
          }
          published_order_by_robot[robot_name] = published;

          apply_staged_planned_path(
            robot_name, order_id, item_phase_index, item_path_revision);

          RCLCPP_INFO(
            get_logger(),
            "🛡️ [VDA5050 ORDER] Suppressed stale republish for robot=%s, "
            "orderId=%s, orderUpdateId=%d, phase=%s, source=%s: robot state "
            "already reports orderId=%s orderUpdateId=%d — no MQTT republish",
            robot_name.c_str(), order_id.c_str(), order_update_id, phase.c_str(),
            source.c_str(), state_order_id.c_str(), state_update_id);
          continue;
        }
      }
    }

    if (release_extension_publish) {
      RCLCPP_INFO(
        get_logger(),
        "🛤️ [VDA5050 ORDER] Release extension publish robot=%s orderId=%s "
        "orderUpdateId=%d released_nodes %zu → %zu",
        robot_name.c_str(), order_id.c_str(), order_update_id,
        prev_order_it->second.released_node_count, released_node_count);
    }

    if (mqtt_client->publish_vda5050_order(robot_name, order_payload)) {
      last_published_vda_order[robot_name] = fingerprint;

      // Remember this order so a later PathRequest for the SAME leg with a
      // higher path_revision can be recognised as a replan and re-sent as an
      // order update instead of being dropped as a duplicate.
      PublishedOrder published;
      published.order_id = order_id;
      published.phase = phase;
      published.phase_index = item_phase_index;
      published.path_revision = item_path_revision;
      published.released_node_count = released_node_count;
      if (released_end_xy_pre.has_value()) {
        published.released_end_x = released_end_xy_pre->first;
        published.released_end_y = released_end_xy_pre->second;
        published.has_released_end = true;
      }
      if (last_wp_xy.has_value()) {
        published.dest_x = last_wp_xy->first;
        published.dest_y = last_wp_xy->second;
      }
      published_order_by_robot[robot_name] = published;

      // The robot now has this route. Only now may /Monitored_task_details —
      // and therefore zone_monitor and zone_manager — be told about it.
      apply_staged_planned_path(
        robot_name, order_id, item_phase_index, item_path_revision);

      RCLCPP_INFO(
        get_logger(),
        "✅ [VDA5050 ORDER] Published topic=uagv/v3/Armstrong/%s/order "
        "(orderId=%s, orderUpdateId=%d, phase=%s, source=%s, after=Monitored_task_details)",
        robot_name.c_str(), order_id.c_str(), order_update_id, phase.c_str(), source.c_str());
    } else {
      RCLCPP_ERROR(
        get_logger(),
        "❌ [VDA5050 ORDER] Failed to publish order for robot=%s, orderId=%s, phase=%s, source=%s",
        robot_name.c_str(), order_id.c_str(), phase.c_str(), source.c_str());
      remaining.push_back(item);
    }
  }

  if (!remaining.empty()) {
    pending_vda_orders_by_task[task_id] = remaining;
  } else {
    pending_vda_orders_by_task.erase(task_id);
  }
}

std::optional<AnyMap> RMFIntegratedFleetAdapter::build_phase_action_parameter(
  const std::string & phase, const std::string & place) const
{
  // Python lines 2148–2157
  if (place.empty()) {
    return std::nullopt;
  }
  const std::string prefix = (phase == "pick") ? "pick" : (phase == "drop") ? "drop" : "";
  if (prefix.empty()) {
    return std::nullopt;
  }
  std::smatch match;
  std::string suffix;
  if (std::regex_search(place, match, std::regex(R"((\d+)$)"))) {
    suffix = match[1].str();
  }
  return AnyMap{
    {"key", YAML::Node(prefix + suffix)},
    {"value", YAML::Node(place)},
  };
}

std::optional<AnyMap> RMFIntegratedFleetAdapter::build_patrol_action_parameter(
  const std::string & /*robot_name*/, const std::string & place) const
{
  // Python lines 2159–2167
  if (place.empty()) {
    return std::nullopt;
  }
  std::smatch match;
  if (std::regex_search(place, match, std::regex(R"((\d+))"))) {
    return AnyMap{
      {"key", YAML::Node("PATROL" + match[1].str())},
      {"value", YAML::Node(place)},
    };
  }
  return AnyMap{
    {"key", YAML::Node("PATROLN")},
    {"value", YAML::Node(place)},
  };
}

// ============================================================================
// Section 09 — VDA5050 order build (Python lines 1892, 2169–2429)
// ============================================================================

int RMFIntegratedFleetAdapter::next_order_header_id(const std::string & robot_name)
{
  // Python lines 2169–2174
  const int current = vda_order_header_id_by_robot[robot_name];
  const int next_id = current + 1;
  vda_order_header_id_by_robot[robot_name] = next_id;
  return next_id;
}

int RMFIntegratedFleetAdapter::next_instant_actions_header_id(const std::string & robot_name)
{
  // Python lines 2176–2181
  const int current = vda_instant_actions_header_id_by_robot[robot_name];
  const int next_id = current + 1;
  vda_instant_actions_header_id_by_robot[robot_name] = next_id;
  return next_id;
}

bool RMFIntegratedFleetAdapter::publish_vda5050_instant_action(
  const std::string & robot_name,
  const std::string & action_type,
  const std::string & action_id,
  const std::vector<AnyMap> * action_parameters,
  const std::string & action_descriptor)
{
  // Python lines 2183–2218
  if (!mqtt_client) {
    return false;
  }
  AnyMap action{
    {"actionId", YAML::Node(action_id)},
    {"actionType", YAML::Node(action_type)},
    {"blockingType", YAML::Node("NONE")},
  };
  if (!action_descriptor.empty()) {
    action["actionDescriptor"] = YAML::Node(action_descriptor);
  }
  if (action_parameters != nullptr && !action_parameters->empty()) {
    YAML::Node params_node(YAML::NodeType::Sequence);
    for (const auto & param : *action_parameters) {
      YAML::Node item(YAML::NodeType::Map);
      for (const auto & [key, value] : param) {
        item[key] = value;
      }
      params_node.push_back(item);
    }
    action["actionParameters"] = params_node;
  }
  const AnyMap payload = MQTTFleetClient::build_vda5050_instant_actions_payload(
    robot_name,
    next_instant_actions_header_id(robot_name),
    {action});
  const bool ok = mqtt_client->publish_vda5050_instant_actions(robot_name, payload);
  if (ok) {
    RCLCPP_INFO(
      get_logger(),
      "✅ [VDA5050 INSTANT] Published topic=uagv/v3/Armstrong/%s/instantActions "
      "(actionType=%s, actionId=%s)",
      robot_name.c_str(), action_type.c_str(), action_id.c_str());
  } else {
    RCLCPP_ERROR(
      get_logger(),
      "❌ [VDA5050 INSTANT] Failed to publish instantActions for robot=%s, actionType=%s",
      robot_name.c_str(), action_type.c_str());
  }
  return ok;
}

void RMFIntegratedFleetAdapter::zone_conflict_callback(
  const zone_manager::msg::Conflict::SharedPtr msg)
{
  if (!msg) {
    return;
  }

  const bool is_detected =
    msg->type == zone_manager::msg::Conflict::TYPE_DETECTED &&
    msg->action == zone_manager::msg::Conflict::ACTION_START_PAUSE;
  const bool is_resolved =
    msg->type == zone_manager::msg::Conflict::TYPE_RESOLVED &&
    msg->action == zone_manager::msg::Conflict::ACTION_STOP_PAUSE;

  if (!is_detected && !is_resolved) {
    RCLCPP_WARN(
      get_logger(),
      "Ignoring /zone_conflicts with mismatched type=%u action=%s zone=%s",
      msg->type, msg->action.c_str(), msg->zone_id.c_str());
    return;
  }

  if (msg->robot_ids.empty()) {
    RCLCPP_WARN(
      get_logger(),
      "Ignoring /zone_conflicts with empty robot_ids (type=%u action=%s zone=%s)",
      msg->type, msg->action.c_str(), msg->zone_id.c_str());
    return;
  }

  if (!mqtt_client) {
    RCLCPP_ERROR(
      get_logger(),
      "Cannot publish zone conflict instant action: MQTT client not ready (zone=%s action=%s)",
      msg->zone_id.c_str(), msg->action.c_str());
    return;
  }

  const std::string action_id_prefix =
    "zone-" + msg->zone_id + "-" + msg->action + "-" + std::to_string(msg->stamp.sec);

  for (const auto & robot_name : msg->robot_ids) {
    if (robot_name.empty()) {
      continue;
    }

    if (is_resolved && paused_by_deviation.count(robot_name) > 0) {
      RCLCPP_WARN(
        get_logger(),
        "🛑 [ZONE RESUME BLOCKED] Robot %s has active path deviation pause — ignoring zone stopPause",
        robot_name.c_str());
      paused_by_zone.erase(robot_name);
      deferred_replan_by_robot.erase(robot_name);
      continue;
    }

    const std::string action_id = action_id_prefix + "-" + robot_name;
    const bool ok = publish_vda5050_instant_action(
      robot_name,
      msg->action,
      action_id,
      nullptr,
      "Zone conflict " + msg->zone_id);
    if (ok) {
      RCLCPP_INFO(
        get_logger(),
        "✅ [ZONE CONFLICT] Published %s to robot %s (zone=%s type=%u)",
        msg->action.c_str(), robot_name.c_str(), msg->zone_id.c_str(), msg->type);
    } else {
      RCLCPP_ERROR(
        get_logger(),
        "❌ [ZONE CONFLICT] Failed to publish %s to robot %s (zone=%s type=%u)",
        msg->action.c_str(), robot_name.c_str(), msg->zone_id.c_str(), msg->type);
    }

    // ---- Phase 2 bookkeeping ------------------------------------------
    // A ReplanCandidate is only honoured for a robot we have actually paused,
    // which gives the "pause first, then replan" ordering. On Resolved the
    // robot is resumed by zone_manager, so drop it and release any closure it
    // is still holding.
    if (is_detected) {
      paused_by_zone.insert(robot_name);
      // The candidate for this conflict may already have arrived (separate
      // topics, separate DDS readers). Now that the robot is recorded as
      // paused, replay anything that was waiting on exactly this.
      flush_deferred_replan_candidate(robot_name);
    } else {
      paused_by_zone.erase(robot_name);
      // The conflict is over; a candidate still waiting for a startPause that
      // will now never come must not close lanes retroactively.
      deferred_replan_by_robot.erase(robot_name);
      // The private closure is deliberately NOT released here. The conflict
      // resolves as soon as the detour is planned — long before the robot has
      // driven it — so reopening the lanes now simply lets RMF route straight
      // back through the conflict zone and the conflict returns on the next
      // tick. The closure is held for the whole leg instead.
    }
  }
}

// ============================================================================
// Path deviation alerts from the monitor package
// ============================================================================

void RMFIntegratedFleetAdapter::path_deviation_callback(
  const std_msgs::msg::String::SharedPtr msg)
{
  if (!msg || msg->data.empty()) {
    return;
  }

  std::string robot_name;
  std::string task_id;
  std::string event;
  double deviation = 0.0;

  try {
    const YAML::Node node = YAML::Load(msg->data);
    robot_name = yaml_map_get_string(node, "robot");
    task_id = yaml_map_get_string(node, "task_id");
    event = yaml_map_get_string(node, "event");
    if (node["deviation"]) {
      deviation = node["deviation"].as<double>();
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      get_logger(),
      "❌ [PATH DEVIATION] Failed to parse /path_deviation payload: %s (error: %s)",
      msg->data.c_str(), e.what());
    return;
  }

  if (robot_name.empty()) {
    RCLCPP_WARN(get_logger(), "⚠️ [PATH DEVIATION] Received alert with empty robot name");
    return;
  }

  if (event == "recovered") {
    const bool was_tracked = (paused_by_deviation.erase(robot_name) > 0);
    RCLCPP_INFO(
      get_logger(),
      "[PATH RECOVERED] Received recovery alert for %s (deviation=%.4fm, task=%s, tracked_in_fms=%s)",
      robot_name.c_str(), deviation, task_id.c_str(), was_tracked ? "true" : "false");

    const std::string action_id =
      "deviation-resume-" + robot_name + "-" +
      std::to_string(get_clock()->now().nanoseconds());
    const bool ok = publish_vda5050_instant_action(
      robot_name,
      "stopPause",
      action_id,
      nullptr,
      "Robot returned within planned path tolerance (task=" + task_id + ")");

    if (ok) {
      RCLCPP_INFO(
        get_logger(),
        "[PATH RECOVERED] Sent one stopPause to %s (deviation=%.4fm, task=%s); task remains active",
        robot_name.c_str(), deviation, task_id.c_str());
    } else {
      RCLCPP_ERROR(
        get_logger(),
        "[PATH RECOVERED] Failed to publish stopPause for %s; deviation pause remains latched",
        robot_name.c_str());
      if (was_tracked) {
        paused_by_deviation.insert(robot_name);
      }
    }
    return;
  }

  if (!event.empty() && event != "deviation") {
    RCLCPP_WARN(
      get_logger(), "[PATH DEVIATION] Ignoring unsupported event '%s' for robot %s",
      event.c_str(), robot_name.c_str());
    return;
  }

  if (paused_by_deviation.count(robot_name) > 0) {
    RCLCPP_DEBUG(
      get_logger(),
      "ℹ️ [PATH DEVIATION] Robot %s already in paused_by_deviation — ignoring duplicate alert",
      robot_name.c_str());
    return;
  }

  const std::string action_id =
    "deviation-pause-" + robot_name + "-" +
    std::to_string(get_clock()->now().nanoseconds());
  const std::string desc =
    "Path deviation " + std::to_string(deviation) + "m exceeded tolerance (task=" + task_id + ")";
  const bool ok = publish_vda5050_instant_action(
    robot_name,
    "startPause",
    action_id,
    nullptr,
    desc);

  if (ok) {
    paused_by_deviation.insert(robot_name);
    RCLCPP_WARN(
      get_logger(),
      "🛑 [PATH DEVIATION] Robot %s marked in paused_by_deviation and stopped via startPause (deviation=%.4fm, task=%s)",
      robot_name.c_str(), deviation, task_id.c_str());
  } else {
    RCLCPP_ERROR(
      get_logger(),
      "❌ [PATH DEVIATION] Failed to publish startPause instantAction for robot %s",
      robot_name.c_str());
  }
}

// ============================================================================
// Phase 2 — per-robot lane closure driven by /replan_candidates
// ============================================================================

void RMFIntegratedFleetAdapter::replan_candidate_callback(
  const zone_manager::msg::ReplanCandidate::SharedPtr msg)
{
  if (!msg) {
    return;
  }

  const std::string robot_name = trim_copy(msg->robot_id);
  const std::string zone_id = trim_copy(msg->zone_id);
  if (robot_name.empty() || zone_id.empty()) {
    RCLCPP_WARN(
      get_logger(),
      "⚠️ [ZONE REPLAN] Ignoring ReplanCandidate with empty robot_id/zone_id");
    return;
  }

  // A candidate is only actionable once the robot is actually paused, so that
  // lanes are closed AFTER the robot has stopped rather than while it is still
  // driving into the conflict.
  //
  // An early candidate is DEFERRED, not dropped. zone_manager now publishes the
  // Conflict before the candidate, but they travel on separate topics with
  // separate DDS readers, so the candidate can still be delivered first. Before
  // this, an early candidate was discarded outright and the replan simply never
  // happened — observed on zone 6, where the candidate was handled 551 us
  // before the startPause was recorded and amr1 stayed paused forever.
  if (paused_by_zone.count(robot_name) == 0) {
    deferred_replan_by_robot[robot_name] =
      DeferredReplanCandidate{msg, get_clock()->now()};
    RCLCPP_INFO(
      get_logger(),
      "⏳ [ZONE REPLAN] Deferring candidate %s for zone %s: startPause not seen "
      "yet — will apply when it arrives (within %.0fs)",
      robot_name.c_str(), zone_id.c_str(), deferred_replan_timeout_sec);
    return;
  }

  apply_replan_candidate(msg, robot_name, zone_id, false);
}

void RMFIntegratedFleetAdapter::apply_replan_candidate(
  const zone_manager::msg::ReplanCandidate::SharedPtr & msg,
  const std::string & robot_name,
  const std::string & zone_id,
  bool deferred)
{
  if (!msg) {
    return;
  }

  if (!nav_graph_parser) {
    RCLCPP_ERROR(
      get_logger(),
      "❌ [ZONE REPLAN] Nav graph not loaded; cannot resolve lanes for zone %s",
      zone_id.c_str());
    return;
  }

  // ---- Block the OTHER robot's first zone ---------------------------------
  // One zone per conflicting robot, not the conflict zone and its successor.
  //
  // The conflict zone is where the two routes MEET; closing it makes the
  // replanning robot turn away at its mouth, which leaves it adjacent to the
  // other robot and lets a small path change put it straight back in. Closing
  // the successor as well denied the whole corridor, which is effective but
  // expensive: it regularly left the robot unable to plan at all (see the
  // starvation note in the task-terminal release below — amr2 held zones 5+6
  // for 542 s and its next task could not be planned).
  //
  // Blocking the zone the OTHER robot is standing in is cheaper and more
  // direct. The replanning robot may still plan and DRIVE right up to that
  // zone, and diverts there if a branch exists:
  //
  //   amr1  3 -> [4] -> 5 -> 6 -> 7 -> 8       (other robot, currently in 4)
  //   amr2  10 -> 9 -> 8 -> 7 -> 6 -> 5 -> 4 -> 3   (replanning, target 3)
  //   blocks 4 only. amr2 runs to zone 5, finds 4 shut, and branches there.
  //   Zone 3 stays open, so its goal is still reachable.
  //
  // THE TARGET ZONE IS NEVER CLOSED. If the other robot's first zone is where
  // this robot's route ENDS, closing it would fence off the goal and the
  // replan could not succeed at all. In that case walk forward through the
  // other robot's sequence — second, third, ... — and take the first zone that
  // is not the target.
  //
  // With three or more robots in the conflict, every other robot contributes
  // one zone by the same rule, so a two-robot conflict closes exactly one.
  //
  // zone_sequence is documented in RobotZoneStatus.msg as "Remaining zones from
  // current onward (passed zones are dropped)", so index 0 IS the robot's
  // current zone and the walk is already in travel order.
  const std::string target_zone = target_zone_of(robot_name);

  std::vector<std::string> blocked_zones;
  std::vector<std::size_t> lanes;

  for (const auto & other_raw : msg->conflicting_robots) {
    const std::string other = trim_copy(other_raw);
    if (other.empty() || other == robot_name) {
      continue;
    }

    const std::string zone = first_zone_other_than(other, target_zone);
    if (zone.empty()) {
      RCLCPP_WARN(
        get_logger(),
        "⚠️ [ZONE REPLAN] robot=%s: no usable zone from %s's sequence (unknown, "
        "or every remaining zone is %s's target %s) — %s contributes no closure",
        robot_name.c_str(), other.c_str(), robot_name.c_str(),
        target_zone.empty() ? "?" : target_zone.c_str(), other.c_str());
      continue;
    }

    if (std::find(blocked_zones.begin(), blocked_zones.end(), zone) !=
      blocked_zones.end())
    {
      continue;
    }

    const std::vector<std::size_t> zone_lanes =
      nav_graph_parser->lanes_for_zone(zone);
    if (zone_lanes.empty()) {
      RCLCPP_WARN(
        get_logger(),
        "⚠️ [ZONE REPLAN] robot=%s: zone %s (from %s) has no lanes in the nav "
        "graph — skipping it",
        robot_name.c_str(), zone.c_str(), other.c_str());
      continue;
    }

    blocked_zones.push_back(zone);
    for (const std::size_t lane : zone_lanes) {
      if (std::find(lanes.begin(), lanes.end(), lane) == lanes.end()) {
        lanes.push_back(lane);
      }
    }
  }

  // Fall back to the conflict zone rather than closing nothing: a replan with
  // an unchanged graph returns the same route and the conflict simply repeats.
  if (blocked_zones.empty()) {
    lanes = nav_graph_parser->lanes_for_zone(zone_id);
    if (!lanes.empty()) {
      blocked_zones.push_back(zone_id);
      RCLCPP_WARN(
        get_logger(),
        "⚠️ [ZONE REPLAN] robot=%s: no other robot yielded a zone to block — "
        "falling back to the conflict zone %s alone",
        robot_name.c_str(), zone_id.c_str());
    }
  }

  if (lanes.empty()) {
    RCLCPP_ERROR(
      get_logger(),
      "❌ [ZONE REPLAN] Nothing to close for robot %s (conflict zone %s) — the "
      "replan would return the same route, so it is not requested",
      robot_name.c_str(), zone_id.c_str());
    return;
  }

  if (deferred) {
    RCLCPP_INFO(
      get_logger(),
      "▶️ [ZONE REPLAN] Applying deferred candidate %s for zone %s now that its "
      "startPause has arrived",
      robot_name.c_str(), zone_id.c_str());
  }

  std::string blocked_list;
  for (std::size_t i = 0; i < blocked_zones.size(); ++i) {
    blocked_list += (i ? "+" : "") + blocked_zones[i];
  }

  apply_private_lane_closure(
    robot_name, zone_id, blocked_list, lanes,
    msg->reason.empty() ? std::string("zone conflict") : msg->reason);
}

void RMFIntegratedFleetAdapter::flush_deferred_replan_candidate(
  const std::string & robot_name)
{
  const auto it = deferred_replan_by_robot.find(robot_name);
  if (it == deferred_replan_by_robot.end()) {
    return;
  }

  const auto msg = it->second.msg;
  const double waited = (get_clock()->now() - it->second.received_at).seconds();
  deferred_replan_by_robot.erase(it);
  if (!msg) {
    return;
  }

  // The sweep below runs on a 5 s timer, so a candidate can still be sitting
  // here past its deadline when the pause finally arrives. Check the age at the
  // point of use as well: that keeps the deadline exact regardless of how often
  // the sweep runs, and stops a stale candidate from closing lanes for a pause
  // it never belonged to.
  if (waited >= deferred_replan_timeout_sec) {
    RCLCPP_WARN(
      get_logger(),
      "⏰ [ZONE REPLAN] Dropping deferred candidate %s (zone=%s): startPause "
      "arrived %.1fs late, past the %.0fs deadline",
      robot_name.c_str(), trim_copy(msg->zone_id).c_str(),
      waited, deferred_replan_timeout_sec);
    return;
  }

  apply_replan_candidate(msg, robot_name, trim_copy(msg->zone_id), true);
}

void RMFIntegratedFleetAdapter::deferred_replan_timeout_tick()
{
  if (deferred_replan_by_robot.empty()) {
    return;
  }

  const rclcpp::Time now = get_clock()->now();
  for (auto it = deferred_replan_by_robot.begin();
    it != deferred_replan_by_robot.end(); )
  {
    const double waited = (now - it->second.received_at).seconds();
    if (waited < deferred_replan_timeout_sec) {
      ++it;
      continue;
    }
    RCLCPP_WARN(
      get_logger(),
      "⏰ [ZONE REPLAN] Discarding deferred candidate %s (zone=%s): no startPause "
      "arrived within %.0fs — not closing lanes for a robot that was never paused",
      it->first.c_str(),
      it->second.msg ? trim_copy(it->second.msg->zone_id).c_str() : "?",
      deferred_replan_timeout_sec);
    it = deferred_replan_by_robot.erase(it);
  }
}

bool RMFIntegratedFleetAdapter::is_replan_of_published_order(
  const std::string & robot_name,
  const std::string & dispatch_task_id,
  int phase_index,
  int path_revision,
  double last_wp_x,
  double last_wp_y) const
{
  const auto it = published_order_by_robot.find(robot_name);
  if (it == published_order_by_robot.end()) {
    return false;                       // nothing published yet for this robot
  }
  const PublishedOrder & prev = it->second;

  // 1) Same task. A path for a DIFFERENT dispatch id belongs to another task and
  //    must keep its normal queued/buffered handling — never touched here.
  if (dispatch_task_id.empty() || dispatch_task_id != prev.order_id) {
    return false;
  }
  // 2) Same leg. phase_index changing (e.g. delivery pickup 0 -> drop 1) means a
  //    NEW order, not an update of the current one.
  if (phase_index != prev.phase_index) {
    return false;
  }
  // 3) Higher revision. RMF bumps path_revision once per replan; the same
  //    revision arriving again is the duplicate/density variant of a path we
  //    already sent, so it is ignored.
  if (path_revision <= prev.path_revision) {
    return false;
  }
  // 4) Same destination. Middle waypoints may differ in density between
  //    publishes; only the end point identifies the leg.
  if (!coords_match(last_wp_x, last_wp_y, prev.dest_x, prev.dest_y)) {
    return false;
  }
  return true;
}

std::size_t RMFIntegratedFleetAdapter::hash_path_request(
  const rmf_fleet_msgs::msg::PathRequest & msg)
{
  // Positions only, quantised to the millimetre. Timestamps and yaw are
  // deliberately excluded: RMF restamps an unchanged route on every replan, and
  // yaw at a waypoint can differ between plans that drive the same ground.
  //
  // Compared against the last path ACCEPTED for this robot, both sides hashed
  // the same way, so this never has to agree with the VDA order fingerprint —
  // which is computed over the published order's nodes, after node 0 has been
  // refreshed to the robot's live pose.
  std::size_t h = 1469598103934665603ull;
  for (const auto & wp : msg.path) {
    const long long qx = static_cast<long long>(std::llround(wp.x * 1000.0));
    const long long qy = static_cast<long long>(std::llround(wp.y * 1000.0));
    h = h * 1000003u + static_cast<std::size_t>(qx);
    h = h * 1000003u + static_cast<std::size_t>(qy);
  }
  return h;
}

bool RMFIntegratedFleetAdapter::send_replan_nav_cancel(
  const std::string & robot_name, const std::string & order_id)
{
  // Cancels the robot's CURRENT NAV2 GOAL only.
  //
  // Chain: cancelOrder -> mqtt_robot_client_node -> /action_requests = 2 ->
  //        cancel_current_goal_node -> navigate_through_poses/_action/cancel_goal
  //
  // cancel_current_goal_node does nothing else: no task state, no task queue,
  // no load status. The RMF/FMS task therefore keeps running — this is NOT a
  // task cancellation and deliberately does not call clear_task_state().
  //
  // The robot-side latch holds /action_requests at 2 until the next order
  // arrives (clear_cancel_latch), so the replanned order we publish immediately
  // afterwards releases it and gives the next replan a fresh 0->2 rising edge.
  const std::string action_id =
    "replan-cancel-" + order_id + "-" + std::to_string(get_clock()->now().nanoseconds());
  const bool ok = publish_vda5050_instant_action(
    robot_name, "cancelOrder", action_id, nullptr, "Replan: abort current nav goal");
  if (ok) {
    RCLCPP_INFO(
      get_logger(),
      "🛑 [REPLAN ORDER] Sent cancelOrder to %s (orderId=%s) — aborting current nav "
      "goal only; FMS task continues",
      robot_name.c_str(), order_id.c_str());
  } else {
    RCLCPP_ERROR(
      get_logger(),
      "❌ [REPLAN ORDER] Failed to send cancelOrder to %s (orderId=%s); replanned "
      "order will NOT be published so the robot keeps its current route",
      robot_name.c_str(), order_id.c_str());
  }
  return ok;
}

void RMFIntegratedFleetAdapter::robot_zone_status_callback(
  const monitor::msg::RobotZoneStatus::SharedPtr msg)
{
  if (!msg) {
    return;
  }
  const std::string robot_name = trim_copy(msg->amr_id);
  if (robot_name.empty()) {
    return;
  }
  std::vector<std::string> seq;
  seq.reserve(msg->zone_sequence.size());
  for (const auto & z : msg->zone_sequence) {
    const std::string t = trim_copy(z);
    if (!t.empty()) {
      seq.push_back(t);
    }
  }
  zone_sequence_by_robot[robot_name] = std::move(seq);
}

std::string RMFIntegratedFleetAdapter::zone_after(
  const std::string & robot_name, const std::string & zone_id) const
{
  const auto it = zone_sequence_by_robot.find(robot_name);
  if (it == zone_sequence_by_robot.end()) {
    return {};
  }
  const auto & seq = it->second;
  for (std::size_t i = 0; i < seq.size(); ++i) {
    if (seq[i] != zone_id) {
      continue;
    }
    // Last in the remaining sequence means the conflict zone is where this
    // robot's route ends; there is no zone beyond it to block.
    return (i + 1 < seq.size()) ? seq[i + 1] : std::string{};
  }
  return {};
}

std::string RMFIntegratedFleetAdapter::target_zone_of(
  const std::string & robot_name) const
{
  const auto it = zone_sequence_by_robot.find(robot_name);
  if (it == zone_sequence_by_robot.end() || it->second.empty()) {
    return {};
  }
  // zone_sequence is the REMAINING route, so its last entry is the zone the
  // robot's destination sits in. Returning "" when it is unknown is safe: the
  // caller then simply has no zone to protect, and an empty string never
  // matches a real zone id.
  return it->second.back();
}

std::string RMFIntegratedFleetAdapter::first_zone_other_than(
  const std::string & robot_name, const std::string & target_zone) const
{
  const auto it = zone_sequence_by_robot.find(robot_name);
  if (it == zone_sequence_by_robot.end()) {
    return {};
  }
  // Index 0 is where this robot is standing right now — the zone the replanning
  // robot must be steered around. Walking forward only happens when that zone
  // is the replanning robot's own target, which must never be closed or the
  // goal becomes unreachable and the replan cannot succeed at all.
  for (const auto & zone : it->second) {
    if (!zone.empty() && zone != target_zone) {
      return zone;
    }
  }
  return {};
}

void RMFIntegratedFleetAdapter::apply_private_lane_closure(
  const std::string & robot_name,
  const std::string & zone_id,
  const std::string & blocked_zones,
  const std::vector<std::size_t> & lanes,
  const std::string & reason,
  bool request_replan)
{
  if (!robot_lane_request_pub) {
    return;
  }

  // A zone-conflict closure replaces a drop corridor rather than merging with
  // it: the corridor is a single route, so adding the conflict zone's lanes on
  // top would usually leave the robot no path to the drop at all.
  const auto prev_it = private_closure_by_robot.find(robot_name);
  if (prev_it != private_closure_by_robot.end() && prev_it->second.corridor_hold &&
    zone_id != "drop_corridor")
  {
    RCLCPP_WARN(
      get_logger(),
      "⚠️ [DROP CORRIDOR] robot=%s: zone conflict %s replaces its drop corridor "
      "(%s) for the rest of this leg",
      robot_name.c_str(), zone_id.c_str(), prev_it->second.blocked_zones.c_str());
  }

  rmf_fleet_msgs::msg::RobotLaneRequest req;
  req.fleet_name = fleet_name;
  req.robot_name = robot_name;
  req.close_lanes.assign(lanes.begin(), lanes.end());
  // Drop corridor: do not replan mid WaitForDispenser; drop GoToPlace uses the
  // private graph on its own. Zone conflicts still force an immediate replan.
  req.skip_replan = !request_replan;
  robot_lane_request_pub->publish(req);

  // Stamp the leg this closure belongs to, so it can be released when the leg
  // changes and at no other point during the task.
  // Stamp the task and leg this closure belongs to. -1 / "" mean unknown (no
  // order published yet); the release checks skip an unknown field rather than
  // guessing, so such a closure is held until a PathRequest for a different
  // task arrives.
  int leg = -1;
  std::string owning_task;
  const auto pub_it = published_order_by_robot.find(robot_name);
  if (pub_it != published_order_by_robot.end()) {
    leg = pub_it->second.phase_index;
    owning_task = pub_it->second.order_id;
  }

  PrivateClosure closure;
  closure.zone_id = zone_id;
  closure.blocked_zones = blocked_zones;
  closure.lanes = lanes;
  closure.applied_at = get_clock()->now();
  closure.reason = reason;
  closure.phase_index = leg;
  closure.order_id = owning_task;
  private_closure_by_robot[robot_name] = std::move(closure);

  std::ostringstream lane_list;
  for (std::size_t i = 0; i < lanes.size(); ++i) {
    lane_list << (i ? "," : "") << lanes[i];
  }
  // The conflict zone is reported but NOT closed (unless it is the fallback),
  // so the two are printed separately rather than joined with a '+'.
  RCLCPP_INFO(
    get_logger(),
    "🚧 [ZONE REPLAN] blocked=[%s] conflict=%s candidate=%s lanes=[%s] task=%s "
    "leg=%d reason='%s' — closed for this robot only until the task or leg "
    "changes; RMF will replan it",
    blocked_zones.c_str(), zone_id.c_str(), robot_name.c_str(),
    lane_list.str().c_str(),
    owning_task.empty() ? "?" : owning_task.c_str(), leg, reason.c_str());
}

void RMFIntegratedFleetAdapter::clear_private_lane_closure(
  const std::string & robot_name, const std::string & reason, bool replan)
{
  const auto it = private_closure_by_robot.find(robot_name);
  if (it == private_closure_by_robot.end()) {
    return;
  }

  const double held_s = (get_clock()->now() - it->second.applied_at).seconds();
  const std::string zone_id = it->second.zone_id;

  if (robot_lane_request_pub) {
    rmf_fleet_msgs::msg::RobotLaneRequest req;
    req.fleet_name = fleet_name;
    req.robot_name = robot_name;
    // Any entry in open_lanes clears ALL private closures for the robot.
    req.open_lanes.assign(it->second.lanes.begin(), it->second.lanes.end());
    req.skip_replan = !replan;
    robot_lane_request_pub->publish(req);
  }

  private_closure_by_robot.erase(it);

  RCLCPP_INFO(
    get_logger(),
    "✅ [ZONE REPLAN] %s cleared (zone=%s) — reason=%s, held %.1fs",
    robot_name.c_str(), zone_id.c_str(), reason.c_str(), held_s);
}

// ---- 4D / ASRS drop corridor glue (logic lives in drop_corridor.cpp) -------
namespace {

// "Z@x=7.25 + X@y={21.00,28.35} + spines@x={14.50,52.20} + Z@x=14.50"
std::string CorridorSummary(const drop_corridor::Lines& lines) {
  std::ostringstream summary;
  summary << std::fixed << std::setprecision(2) << "Z@x="
          << lines.pickup_column_x << " + X@y={";
  for (std::size_t i = 0; i < lines.highway_ys.size(); ++i) {
    summary << (i ? "," : "") << lines.highway_ys[i];
  }
  summary << "}";
  if (!lines.spine_xs.empty()) {
    summary << " + spines@x={";
    for (std::size_t i = 0; i < lines.spine_xs.size(); ++i) {
      summary << (i ? "," : "") << lines.spine_xs[i];
    }
    summary << "}";
  }
  summary << " + Z@x=" << lines.drop_column_x;
  return summary.str();
}

std::vector<drop_corridor::Point> ToPoints(
    const std::vector<rmf_fleet_msgs::msg::Location>& path) {
  std::vector<drop_corridor::Point> points;
  points.reserve(path.size());
  for (const rmf_fleet_msgs::msg::Location& location : path) {
    points.push_back({location.x, location.y});
  }
  return points;
}

}  // namespace

const AMRRobot* RMFIntegratedFleetAdapter::FindRobot(
    const std::string& robot_id) const {
  if (!fleet_config.has_value()) {
    return nullptr;
  }
  const auto robot_it = fleet_config->robots.find(robot_id);
  return robot_it == fleet_config->robots.end() ? nullptr : &robot_it->second;
}

std::optional<drop_corridor::Point> RMFIntegratedFleetAdapter::LookupDropPoint(
    const std::string& task_id) const {
  const auto coords_it = task_pickup_drop_coords.find(task_id);
  if (coords_it == task_pickup_drop_coords.end()) {
    return std::nullopt;
  }
  const auto drop_it = coords_it->second.find("dropoff");
  if (drop_it == coords_it->second.end() || !drop_it->second.IsMap()) {
    return std::nullopt;
  }
  // convert<double>::decode() reports failure by return value, not by throw.
  const YAML::Node x_node = drop_it->second["x"];
  const YAML::Node y_node = drop_it->second["y"];
  drop_corridor::Point drop;
  if (!x_node.IsScalar() || !y_node.IsScalar() ||
      !YAML::convert<double>::decode(x_node, drop.x) ||
      !YAML::convert<double>::decode(y_node, drop.y)) {
    return std::nullopt;
  }
  return drop;
}

void RMFIntegratedFleetAdapter::ApplyDropCorridorClosure(
    const std::string& robot_id, const std::string& task_id) {
  if (!drop_corridor_enabled_ || !nav_graph_parser || task_id.empty()) {
    return;
  }
  const AMRRobot* robot = FindRobot(robot_id);
  const std::optional<drop_corridor::Point> drop = LookupDropPoint(task_id);
  if (robot == nullptr || !drop) {
    RCLCPP_WARN(get_logger(),
                "⚠️ [DROP CORRIDOR] robot=%s task=%s: robot or drop point "
                "unknown, drop leg planned on the full graph",
                robot_id.c_str(), task_id.c_str());
    return;
  }
  // The robot has just finished picking, so where it stands is the pickup.
  const drop_corridor::Point pickup{robot->pose.x, robot->pose.y};
  const std::optional<drop_corridor::Corridor> corridor =
      drop_corridor::FindCorridor(nav_graph_parser->corridor_geometry(),
                                  pickup, *drop, drop_corridor_layout_);
  if (!corridor) {
    RCLCPP_WARN(get_logger(),
                "⚠️ [DROP CORRIDOR] robot=%s task=%s: X/Z highways do not "
                "connect (%.2f,%.2f) to (%.2f,%.2f), drop leg on full graph",
                robot_id.c_str(), task_id.c_str(), pickup.x, pickup.y,
                drop->x, drop->y);
    return;
  }
  CloseLanesOutsideCorridor(robot_id, task_id, *drop, *corridor);
}

void RMFIntegratedFleetAdapter::CloseLanesOutsideCorridor(
    const std::string& robot_id, const std::string& task_id,
    const drop_corridor::Point& drop,
    const drop_corridor::Corridor& corridor) {
  const std::string summary = CorridorSummary(corridor.lines);
  // skip_replan: closures apply now; drop GoToPlace plans on them without a
  // mid-dispenser replan that previously left same-drop AMRs with no phase=1.
  apply_private_lane_closure(robot_id, "drop_corridor", summary,
                             corridor.closed_lanes,
                             "pickup complete: 4D drop corridor",
                             /*request_replan=*/false);
  const auto closure_it = private_closure_by_robot.find(robot_id);
  if (closure_it == private_closure_by_robot.end()) {
    return;  // Lane request publisher not ready; nothing was closed.
  }
  closure_it->second.phase_index = -1;  // Survive pickup->drop leg change.
  drop_corridor::Hold& hold = closure_it->second.corridor_hold.emplace();
  hold.task_id = task_id;
  hold.drop = drop;
  hold.lines = corridor.lines;
  hold.closed_sec = get_clock()->now().seconds();
  RCLCPP_INFO(get_logger(),
              "🛣️ [DROP CORRIDOR] robot=%s task=%s free=%zu lanes [%s], "
              "blocked=%zu lanes until drop is planned and travel starts",
              robot_id.c_str(), task_id.c_str(), corridor.free_lane_count,
              summary.c_str(), corridor.closed_lanes.size());
}

void RMFIntegratedFleetAdapter::ReleaseDropCorridor(
    const std::string& robot_id, const std::string& reason, bool replan) {
  const auto closure_it = private_closure_by_robot.find(robot_id);
  if (closure_it == private_closure_by_robot.end() ||
      !closure_it->second.corridor_hold) {
    return;
  }
  clear_private_lane_closure(robot_id, "drop corridor: " + reason, replan);
}

void RMFIntegratedFleetAdapter::OnDropCorridorPath(
    const rmf_fleet_msgs::msg::PathRequest& request) {
  const auto closure_it = private_closure_by_robot.find(request.robot_name);
  const AMRRobot* robot = FindRobot(request.robot_name);
  if (closure_it == private_closure_by_robot.end() ||
      !closure_it->second.corridor_hold || robot == nullptr) {
    return;
  }
  drop_corridor::Hold& hold = *closure_it->second.corridor_hold;
  if (hold.plan_ready) {
    return;
  }
  if (!drop_corridor::IsPathOnCorridor(hold.lines, hold.drop,
                                       drop_corridor_release_distance_m_,
                                       ToPoints(request.path))) {
    RCLCPP_INFO(get_logger(),
                "🛣️ [DROP CORRIDOR] robot=%s path (%zu points) is not "
                "the corridor drop plan yet, lanes stay blocked",
                request.robot_name.c_str(), request.path.size());
    return;
  }
  // Flowchart: drop planning done. Lanes stay blocked until the robot starts
  // travelling (DropCorridorTick), so any replan before departure is still
  // made on the corridor.
  hold.plan_ready = true;
  hold.plan_pose = {robot->pose.x, robot->pose.y};
  hold.plan_ready_sec = get_clock()->now().seconds();
  RCLCPP_INFO(get_logger(),
              "🛣️ [DROP CORRIDOR] robot=%s drop leg planned on "
              "corridor (%zu points), lanes blocked until travel starts",
              request.robot_name.c_str(), request.path.size());
}

std::optional<std::string>
RMFIntegratedFleetAdapter::DropCorridorReleaseReason(
    const std::string& robot_id, const drop_corridor::Hold& hold,
    double now_sec, bool* replan) const {
  *replan = true;
  const auto drop_it = dropoff_in_progress.find(hold.task_id);
  if (drop_it == dropoff_in_progress.end() || !drop_it->second) {
    return "drop phase over";
  }
  const auto status_it = known_task_status.find(hold.task_id);
  if (status_it != known_task_status.end() &&
      is_terminal_task_status(status_it->second)) {
    return "task " + status_it->second;
  }
  const AMRRobot* robot = FindRobot(robot_id);
  if (robot == nullptr) {
    return std::nullopt;
  }
  const drop_corridor::Point pose{robot->pose.x, robot->pose.y};
  // Flowchart: drop planned + travel to drop started -> free all lanes.
  if (hold.plan_ready &&
      drop_corridor::HasStartedTravel(robot->status == RobotStatus::Moving,
                                      hold.plan_pose, pose,
                                      drop_corridor_travel_start_m_)) {
    *replan = false;  // Keep the path that was planned on the corridor.
    return "drop leg planned and robot started travelling";
  }
  if (drop_corridor::Distance(pose, hold.drop) <=
      drop_corridor_release_distance_m_) {
    return "reached drop";
  }
  // Safety release: the corridor must never leave a loaded robot unable to
  // plan. If RMF has not produced a corridor drop plan in time, free the
  // lanes (with replan) so the drop is planned on the full graph.
  if (!hold.plan_ready && drop_corridor_plan_timeout_s_ > 0.0 &&
      hold.closed_sec > 0.0 &&
      now_sec - hold.closed_sec >= drop_corridor_plan_timeout_s_) {
    std::ostringstream reason;
    reason << "no drop plan on the corridor within "
           << static_cast<int>(drop_corridor_plan_timeout_s_)
           << "s (falling back to the full graph)";
    return reason.str();
  }
  return std::nullopt;
}

void RMFIntegratedFleetAdapter::DropCorridorTick() {
  struct Release {
    std::string robot_id;
    std::string reason;
    bool replan = true;
  };
  // Collect first: ReleaseDropCorridor() erases from the map being walked.
  std::vector<Release> releases;
  const double now_sec = get_clock()->now().seconds();
  for (const auto& [robot_id, closure] : private_closure_by_robot) {
    if (!closure.corridor_hold) {
      continue;
    }
    bool replan = true;
    std::optional<std::string> reason = DropCorridorReleaseReason(
        robot_id, *closure.corridor_hold, now_sec, &replan);
    if (reason) {
      releases.push_back({robot_id, std::move(*reason), replan});
    }
  }
  for (const Release& release : releases) {
    RCLCPP_INFO(get_logger(),
                "🛣️ [DROP CORRIDOR] robot=%s %s, freeing all its lanes %s",
                release.robot_id.c_str(), release.reason.c_str(),
                release.replan ? "with replan" : "without replan");
    ReleaseDropCorridor(release.robot_id, release.reason, release.replan);
  }
}

void RMFIntegratedFleetAdapter::private_closure_timeout_tick()
{
  // Runs first: this sweep has its own state and must not be skipped by the
  // private-closure early-out below.
  deferred_replan_timeout_tick();

  if (private_closure_by_robot.empty()) {
    return;
  }

  // No timeout release. A closure is held for the whole leg by design; a timer
  // that reopened the lanes on its own schedule is precisely what produced the
  // 1 Hz detect/resolve flapping. The releases are: leg change
  // (path_request_callback) and task end (clear_task_state) — and a task cancel
  // reaches the latter, so an operator can always free a robot that cannot
  // plan its way out.
  //
  // private_closure_timeout_sec is no longer used to release anything. It is
  // still declared so existing params files keep loading.

  // Periodic introspection so a long-lived closure stays visible in the logs.
  const rclcpp::Time now = get_clock()->now();
  std::ostringstream ss;
  for (const auto & [robot_name, closure] : private_closure_by_robot) {
    ss << ' ' << robot_name << "(blocked="
       << (closure.blocked_zones.empty() ? "?" : closure.blocked_zones)
       << ",conflict=" << closure.zone_id
       << ",lanes=" << closure.lanes.size()
       << ",leg=" << closure.phase_index
       << ",held=" << static_cast<int>((now - closure.applied_at).seconds())
       << "s)";
  }
  RCLCPP_INFO(
    get_logger(), "🚧 [ZONE REPLAN] active private closures:%s", ss.str().c_str());
}

bool RMFIntegratedFleetAdapter::path_uses_release_flags(
  const std::vector<rmf_fleet_msgs::msg::Location> & path)
{
  for (const auto & wp : path) {
    if (wp.release) {
      return true;
    }
  }
  return false;
}

std::vector<rmf_fleet_msgs::msg::Location> RMFIntegratedFleetAdapter::waypoints_for_robot_order(
  const std::vector<rmf_fleet_msgs::msg::Location> & path)
{
  if (!path_uses_release_flags(path)) {
    return path;
  }
  std::vector<rmf_fleet_msgs::msg::Location> order_path;
  order_path.reserve(path.size());
  for (const auto & wp : path) {
    if (wp.release) {
      order_path.push_back(wp);
    }
  }
  return order_path;
}

AnyMap RMFIntegratedFleetAdapter::build_path_request_payload(
  const rmf_fleet_msgs::msg::PathRequest & msg) const
{
  // Python lines 1892–1917
  YAML::Node path_list(YAML::NodeType::Sequence);
  for (const auto & wp : msg.path) {
    YAML::Node entry(YAML::NodeType::Map);
    entry["t"] = YAML::Node(YAML::NodeType::Map);
    entry["t"]["sec"] = static_cast<int>(wp.t.sec);
    entry["t"]["nanosec"] = static_cast<int>(wp.t.nanosec);
    entry["x"] = wp.x;
    entry["y"] = wp.y;
    entry["yaw"] = wp.yaw;
    entry["obey_approach_speed_limit"] = wp.obey_approach_speed_limit;
    entry["approach_speed_limit"] = wp.approach_speed_limit;
    entry["level_name"] = wp.level_name;
    entry["index"] = static_cast<int>(wp.index);
    entry["release"] = wp.release;
    entry["traffic_hold"] = wp.traffic_hold;
    entry["traffic_wait_sec"] = wp.traffic_wait_sec;
    entry["traffic_blocker_amr"] = wp.traffic_blocker_amr;
    path_list.push_back(entry);
  }
  const std::string payload_fleet_name = msg.fleet_name.empty() ?
    (fleet_config.has_value() ? fleet_config->fleet_name : fleet_name) : msg.fleet_name;
  return {
    {"fleet_name", YAML::Node(payload_fleet_name)},
    {"robot_name", YAML::Node(msg.robot_name)},
    {"task_id", YAML::Node(msg.task_id)},
    {"dispatch_task_id", YAML::Node(path_request_dispatch_task_id(msg))},
    // Leg identity + replan counter from RMF (rmf_fleet_msgs/PathRequest).
    // phase_index: which navigation leg of the task (delivery pickup=0, drop=1).
    // path_revision: 0 for the first path of a leg, +1 per replan of that leg.
    {"phase_index", YAML::Node(static_cast<int>(msg.phase_index))},
    {"path_revision", YAML::Node(static_cast<int>(msg.path_revision))},
    {"path", path_list},
  };
}

namespace {

double wrap_to_pi(double value)
{
  while (value > M_PI) {
    value -= 2.0 * M_PI;
  }
  while (value < -M_PI) {
    value += 2.0 * M_PI;
  }
  return value;
}

double snap_heading_to_cardinal_deg(double theta_rad)
{
  const double deg = wrap_to_pi(theta_rad) * 180.0 / M_PI;
  if (deg >= -45.0 && deg < 45.0) {
    return 0.0;
  }
  if (deg >= 45.0 && deg < 135.0) {
    return 90.0;
  }
  if (deg >= -135.0 && deg < -45.0) {
    return -90.0;
  }
  return (deg >= 0.0) ? 180.0 : -180.0;
}

bool yaml_waypoint_released(const YAML::Node & wp)
{
  if (!wp.IsMap() || !wp["release"] || !wp["release"].IsScalar()) {
    return false;
  }
  try {
    return wp["release"].as<bool>();
  } catch (...) {
    return false;
  }
}

YAML::Node yaml_path_released_prefix(const YAML::Node & path_points)
{
  if (!path_points || !path_points.IsSequence()) {
    return path_points;
  }
  bool uses_release = false;
  for (const auto & wp : path_points) {
    if (yaml_waypoint_released(wp)) {
      uses_release = true;
      break;
    }
  }
  if (!uses_release) {
    return path_points;
  }

  YAML::Node out(YAML::NodeType::Sequence);
  for (const auto & wp : path_points) {
    if (!yaml_waypoint_released(wp)) {
      break;
    }
    out.push_back(wp);
  }
  return out.size() > 0 ? out : path_points;
}

bool yaml_path_fully_released(const YAML::Node & path_points)
{
  if (!path_points || !path_points.IsSequence() || path_points.size() == 0) {
    return false;
  }
  for (const auto & wp : path_points) {
    if (!yaml_waypoint_released(wp)) {
      return false;
    }
  }
  return true;
}

}  // namespace

// =============================================================================
// Spin-action injection for pivoting
// Compares arrival heading at a node to outgoing travel heading (node i -> i+1).
// Navigation format (senior convention):
//   yaw      = arrival map heading (cardinal snap)
//   spin_deg = target travel heading (cardinal snap)
// Returns std::nullopt when |diff| < spin_threshold_deg (ROS param, default 45).
// =============================================================================
std::optional<YAML::Node> RMFIntegratedFleetAdapter::compute_spin_action(
  const std::string & robot_name,
  double arrival_theta_rad,
  double travel_theta_rad,
  int node_sequence_id) const
{
  // Dynamic threshold: change via ROS param spin_threshold_deg (default 45°).
  const double threshold_deg = spin_threshold_deg;
  const double threshold_rad = threshold_deg * M_PI / 180.0;

  const double diff = wrap_to_pi(travel_theta_rad - arrival_theta_rad);
  const double diff_deg = diff * 180.0 / M_PI;
  RCLCPP_INFO(
    get_logger(),
    "🧭 [SPIN-CHECK] Robot %s at node seq=%d: arrival=%.4f rad (%.1f deg), "
    "travel=%.4f rad (%.1f deg), diff=%.4f rad (%.1f deg), threshold=%.1f deg",
    robot_name.c_str(), node_sequence_id,
    arrival_theta_rad, arrival_theta_rad * 180.0 / M_PI,
    travel_theta_rad, travel_theta_rad * 180.0 / M_PI,
    diff, diff_deg, threshold_deg);

  // Spin when |diff| >= threshold (default 45°). Below that: no pivot needed.
  if (std::abs(diff) < threshold_rad) {
    RCLCPP_INFO(
      get_logger(),
      "✅ [SPIN-SKIP] Robot %s at node seq=%d: aligned with travel direction",
      robot_name.c_str(), node_sequence_id);
    return std::nullopt;
  }

  const double yaw = snap_heading_to_cardinal_deg(arrival_theta_rad);
  const double spin_deg = snap_heading_to_cardinal_deg(travel_theta_rad);

  RCLCPP_INFO(
    get_logger(),
    "🔄 [SPIN] Robot %s at node seq=%d needs spin: yaw=%.0f (arrival), "
    "spin_deg=%.0f (travel target)",
    robot_name.c_str(), node_sequence_id, yaw, spin_deg);

  YAML::Node action(YAML::NodeType::Map);
  action["actionType"] = "spin";
  action["actionId"] = "spin-" + robot_name + "-" + std::to_string(node_sequence_id);
  action["blockingType"] = "HARD";
  action["actionParameters"] = YAML::Node(YAML::NodeType::Sequence);

  YAML::Node yaw_param(YAML::NodeType::Map);
  yaw_param["key"] = "yaw";
  yaw_param["value"] = yaw;
  action["actionParameters"].push_back(yaw_param);

  YAML::Node spin_param(YAML::NodeType::Map);
  spin_param["key"] = "spin_deg";
  spin_param["value"] = spin_deg;
  action["actionParameters"].push_back(spin_param);

  return action;
}

std::optional<AnyMap> RMFIntegratedFleetAdapter::build_vda5050_order_payload(
  const std::string & robot_name,
  const std::string & order_id,
  const std::string & phase,
  const AnyMap & path_request_data)
{
  // Python lines 2220–2307
  if (phase != "pick" && phase != "drop" && phase != "patrol") {
    return std::nullopt;
  }
  const YAML::Node full_path_points = anymap_get_yaml_node(path_request_data, "path");
  if (!full_path_points || !full_path_points.IsSequence() || full_path_points.size() == 0) {
    return std::nullopt;
  }
  if (!fleet_config.has_value()) {
    return std::nullopt;
  }

  // VDA5050 base-release: publish the full leg; traffic_manager sets release on
  // each waypoint. Nav (flatland) must follow nodes with released=true only.
  const YAML::Node path_points = full_path_points;
  const bool mission_fully_released = yaml_path_fully_released(full_path_points);

  bool any_release_flag = false;
  std::size_t released_in_payload = 0;
  for (const auto & wp : full_path_points) {
    if (yaml_waypoint_released(wp)) {
      any_release_flag = true;
      ++released_in_payload;
    }
  }
  if (any_release_flag) {
    RCLCPP_INFO(
      get_logger(),
      "🛤️ [VDA5050 ORDER] Robot %s — full leg %zu waypoints, released=true on "
      "%zu (unreleased nodes sent with released=false)",
      robot_name.c_str(), full_path_points.size(), released_in_payload);
  }

  // orderUpdateId = phase base + path_revision.
  //   phase base : 0 for pick/patrol, 1 for drop (unchanged from before, so the
  //                first order of a leg is byte-identical to today).
  //   path_revision : incremented by RMF once per replan of this leg, so each
  //                replanned path publishes the SAME orderId with a higher
  //                orderUpdateId — the VDA5050 way of saying "same order,
  //                updated route".
  const int phase_base = vda_order_update_base(phase);
  const int path_revision =
    static_cast<int>(anymap_get_double(path_request_data, "path_revision"));
  const int order_update_id = phase_base + path_revision;
    const std::size_t path_count = path_points.size();
  const bool reanchor_from_live_pose = order_update_id > phase_base;
  std::size_t vda_start_index = 0;
  bool prepend_live_pose = false;
  double live_x = 0.0;
  double live_y = 0.0;
  double live_yaw = 0.0;
  if (reanchor_from_live_pose) {
    const auto robot_it = fleet_config->robots.find(robot_name);
    if (robot_it != fleet_config->robots.end()) {
      live_x = robot_it->second.pose.x;
      live_y = robot_it->second.pose.y;
      live_yaw = robot_it->second.pose.yaw;
      double best_dist = std::numeric_limits<double>::infinity();
      for (std::size_t i = 0; i < path_count; ++i) {
        const YAML::Node wp = path_points[i];
        const double dx = yaml_node_double(wp["x"]) - live_x;
        const double dy = yaml_node_double(wp["y"]) - live_y;
        const double d = std::hypot(dx, dy);
        if (d < best_dist) {
          best_dist = d;
          vda_start_index = i;
        }
      }
      prepend_live_pose = best_dist > 0.25;
    }
  }
  std::string phase_place;
  const auto phase_it = task_phase_places.find(order_id);
  if (phase_it != task_phase_places.end()) {
    const auto place_it = phase_it->second.find(phase);
    if (place_it != phase_it->second.end()) {
      phase_place = place_it->second;
    }
  }

  std::optional<AnyMap> action_parameter;
  if (phase == "patrol") {
    action_parameter = build_patrol_action_parameter(robot_name, phase_place);
  } else {
    action_parameter = build_phase_action_parameter(phase, phase_place);
  }

  struct VdaSegment
  {
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
    std::string level_name;
    bool released{true};
    bool is_terminal{false};
  };
  std::vector<VdaSegment> segments;
  segments.reserve(path_count + 1);
  if (prepend_live_pose) {
    VdaSegment live;
    live.x = live_x;
    live.y = live_y;
    live.yaw = live_yaw;
    live.released = true;
    segments.push_back(live);
  }
  for (std::size_t index = vda_start_index; index < path_count; ++index) {
    const YAML::Node wp = path_points[index];
    VdaSegment seg;
    seg.x = yaml_node_double(wp["x"]);
    seg.y = yaml_node_double(wp["y"]);
    seg.yaw = yaml_node_double(wp["yaw"]);
    if (any_release_flag) {
      seg.released = yaml_waypoint_released(wp);
    } else {
      seg.released = true;
    }
    if (wp.IsMap() && wp["level_name"] && !wp["level_name"].IsNull()) {
      try {
        seg.level_name = wp["level_name"].as<std::string>();
      } catch (...) {
      }
    }
    seg.is_terminal = (index + 1 == path_count) && mission_fully_released;
    segments.push_back(seg);
  }
  if (segments.empty()) {
    return std::nullopt;
  }

  YAML::Node nodes(YAML::NodeType::Sequence);
  YAML::Node edges(YAML::NodeType::Sequence);
  const std::size_t segment_count = segments.size();
  for (std::size_t index = 0; index < segment_count; ++index) {
    const VdaSegment & seg = segments[index];
    const bool wp_released = seg.released;

    const int sequence_id = static_cast<int>(index) * 2;
    std::string map_id = fleet_config->map_name;
    if (!seg.level_name.empty()) {
      map_id = seg.level_name;
    }
    YAML::Node node(YAML::NodeType::Map);
    node["nodeId"] = "wp_" + std::to_string(index);
    node["sequenceId"] = sequence_id;
    node["released"] = wp_released;
    node["nodePosition"] = YAML::Node(YAML::NodeType::Map);
    node["nodePosition"]["x"] = seg.x;
    node["nodePosition"]["y"] = seg.y;
    node["nodePosition"]["theta"] = seg.yaw;
    node["nodePosition"]["mapId"] = map_id;
    node["actions"] = YAML::Node(YAML::NodeType::Sequence);

    // Spin actions only on released nodes (nav follows released subset only).
    const std::size_t next_index = index + 1;
    if (wp_released && next_index < segment_count) {
      const VdaSegment & next_seg = segments[next_index];
      const double xi = seg.x;
      const double yi = seg.y;
      const double x_next = next_seg.x;
      const double y_next = next_seg.y;
      const double dx_out = x_next - xi;
      const double dy_out = y_next - yi;

      if (std::hypot(dx_out, dy_out) > 1e-3) {
        const double travel_theta = std::atan2(dy_out, dx_out);
        double arrival_theta = seg.yaw;
        bool have_arrival = (index == 0);

        if (index > 0) {
          const VdaSegment & prev_seg = segments[index - 1];
          const double x_prev = prev_seg.x;
          const double y_prev = prev_seg.y;
          const double dx_in = xi - x_prev;
          const double dy_in = yi - y_prev;
          if (std::hypot(dx_in, dy_in) > 1e-3) {
            arrival_theta = std::atan2(dy_in, dx_in);
            have_arrival = true;
          }
        }

        if (have_arrival) {
          auto spin_action = compute_spin_action(
            robot_name, arrival_theta, travel_theta, sequence_id);
          if (spin_action.has_value()) {
            node["actions"].push_back(*spin_action);
          }
        }
      }
    }

    if (seg.is_terminal) {
      const std::string action_type = (phase == "pick") ? "pick" : (phase == "drop") ? "drop" : "patrol";
      YAML::Node action(YAML::NodeType::Map);
      action["actionType"] = action_type;
      action["actionId"] = "act-" + phase + "-" + order_id;
      action["blockingType"] = "HARD";
      action["actionParameters"] = YAML::Node(YAML::NodeType::Sequence);
      if (action_parameter.has_value()) {
        YAML::Node param(YAML::NodeType::Map);
        param["key"] = anymap_get_yaml_node(*action_parameter, "key");
        param["value"] = anymap_get_yaml_node(*action_parameter, "value");
        action["actionParameters"].push_back(param);
      }
      node["actions"].push_back(action);
    }
    nodes.push_back(node);
    if (index + 1 < segment_count) {
      YAML::Node edge(YAML::NodeType::Map);
      edge["sequenceId"] = sequence_id + 1;
      edges.push_back(edge);
    }
  }

  if (nodes.size() == 0) {
    return std::nullopt;
  }

  return AnyMap{
    {"headerId", YAML::Node(next_order_header_id(robot_name))},
    {"timestamp", YAML::Node(utc_iso_timestamp_millis())},
    {"version", YAML::Node("3.0.0")},
    {"manufacturer", YAML::Node("Armstrong")},
    {"serialNumber", YAML::Node(robot_name)},
    {"orderId", YAML::Node(order_id)},
    {"orderUpdateId", YAML::Node(order_update_id)},
    {"nodes", nodes},
    {"edges", edges},
  };
}

std::tuple<bool, std::string, int> RMFIntegratedFleetAdapter::patrol_path_matches_selected_places(
  const std::string & task_id,
  const std::vector<rmf_fleet_msgs::msg::Location> & path_points,
  double current_robot_x,
  double current_robot_y,
  const bool skip_first_pose_check) const
{
  YAML::Node seq(YAML::NodeType::Sequence);
  for (const auto & wp : path_points) {
    YAML::Node point(YAML::NodeType::Map);
    point["x"] = wp.x;
    point["y"] = wp.y;
    point["yaw"] = wp.yaw;
    seq.push_back(point);
  }
  return patrol_path_matches_selected_places(
    task_id, seq, current_robot_x, current_robot_y, skip_first_pose_check);
}

bool RMFIntegratedFleetAdapter::is_traffic_horizon_extension(
  const std::string & robot_name,
  const std::string & dispatch_booking_id,
  const rmf_fleet_msgs::msg::PathRequest & msg) const
{
  if (dispatch_booking_id.empty() || !path_uses_release_flags(msg.path)) {
    return false;
  }
  const auto pub_it = published_order_by_robot.find(robot_name);
  if (pub_it == published_order_by_robot.end()) {
    return false;
  }
  const PublishedOrder & prev = pub_it->second;
  if (prev.order_id != dispatch_booking_id) {
    return false;
  }
  if (prev.phase_index != static_cast<int>(msg.phase_index)) {
    return false;
  }
  std::size_t released_count = 0;
  for (const auto & wp : msg.path) {
    if (wp.release) {
      ++released_count;
    }
  }
  return released_count > prev.released_node_count ||
         static_cast<int>(msg.path_revision) > prev.path_revision;
}

void RMFIntegratedFleetAdapter::try_send_buffered_path_request(const std::string & robot_name)
{
  // Python lines 2309–2429
  const auto bufs_it = pending_path_request_by_robot.find(robot_name);
  if (bufs_it == pending_path_request_by_robot.end() || bufs_it->second.empty()) {
    return;
  }
  const auto underway_it = currently_underway_task.find(robot_name);
  if (underway_it == currently_underway_task.end() || underway_it->second.empty()) {
    return;
  }
  const std::string & underway_key = underway_it->second;

  std::string phase_key;
  std::string phase;
  std::string coords_lookup_id;
  if (task_patrol_coords.find(underway_key) != task_patrol_coords.end()) {
    phase_key = underway_key;
    phase = "patrol";
    coords_lookup_id = phase_key;
  } else {
    std::string delivery_booking = underway_key;
    if (task_pickup_drop_coords.find(underway_key) == task_pickup_drop_coords.end()) {
      const auto rk_it = robot_to_coords_key.find(robot_name);
      if (rk_it != robot_to_coords_key.end()) {
        delivery_booking = rk_it->second;
      }
    }
    if (delivery_booking.empty() ||
      task_pickup_drop_coords.find(delivery_booking) == task_pickup_drop_coords.end())
    {
      return;
    }
    phase_key = delivery_booking;
    const bool in_drop =
      dropoff_in_progress.find(phase_key) != dropoff_in_progress.end() &&
      dropoff_in_progress.at(phase_key);
    phase = in_drop ? "drop" : "pick";
  }

  auto & sent_phases = sent_path_request_phase[phase_key];
  if (phase != "patrol" && sent_phases.count(phase) > 0) {
    pending_path_request_by_robot[robot_name] = {};
    return;
  }

  std::vector<AnyMap> remaining;
  for (const auto & buf : bufs_it->second) {
    bool progressed_patrol = false;
    const std::string task_id = anymap_get_string(buf, "task_id");
    const double first_wp_x = anymap_get_double(buf, "first_wp_x");
    const double first_wp_y = anymap_get_double(buf, "first_wp_y");
    const double last_wp_x = anymap_get_double(buf, "last_wp_x");
    const double last_wp_y = anymap_get_double(buf, "last_wp_y");

    if (!fleet_config.has_value()) {
      return;
    }
    const auto robot_it = fleet_config->robots.find(robot_name);
    if (robot_it == fleet_config->robots.end()) {
      remaining.push_back(buf);
      continue;
    }
    const double current_robot_x = robot_it->second.pose.x;
    const double current_robot_y = robot_it->second.pose.y;
    int patrol_matched_j = -1;
    bool should_send = false;

    // ---- Replan of the leg we already published? --------------------------
    // Checked BEFORE the gates below, exactly as the direct path does at
    // path_request_callback(). A replan revisits the SAME destination on the
    // SAME leg, so both gates reject it:
    //
    //   patrol : patrol_path_matches_selected_places() walks forward from
    //            task_patrol_progress and reports "all destinations already
    //            matched" once the first path consumed the single place.
    //   pick /
    //   drop   : should_send_path_request_by_coords() requires the first
    //            waypoint to equal the robot's pose, but a replanned path is
    //            anchored where the robot was when RMF planned it — measured
    //            drift 0.16 m to 1.13 m, against a 0.15 m tolerance.
    //
    // Without this, a replan that happens to arrive while a path is buffered is
    // validated instead of forwarded — the same failure that originally broke
    // patrol replanning, reached by a different route. This is what makes
    // replanning work for delivery (pick = leg 0, drop = leg 1) and for
    // standalone drop tasks, whose paths are the ones most likely to be
    // buffered while their coords arrive asynchronously.
    const AnyMap buf_payload_peek =
      yaml_map_to_anymap(anymap_get_yaml_node(buf, "path_request_data"));
    const std::string buf_replan_dispatch_id =
      trim_copy(anymap_get_string(buf_payload_peek, "dispatch_task_id"));
    const bool buf_is_replan = is_replan_of_published_order(
      robot_name,
      !buf_replan_dispatch_id.empty() ? buf_replan_dispatch_id : phase_key,
      static_cast<int>(anymap_get_double(buf_payload_peek, "phase_index")),
      static_cast<int>(anymap_get_double(buf_payload_peek, "path_revision")),
      last_wp_x, last_wp_y);

    if (buf_is_replan) {
      const auto & prev = published_order_by_robot.at(robot_name);
      RCLCPP_INFO(
        get_logger(),
        "🔁 [REPLAN ORDER] robot=%s orderId=%s phase=%s leg=%d — revision %d -> %d "
        "on a BUFFERED path, destination unchanged; bypassing patrol/coords gate",
        robot_name.c_str(), buf_replan_dispatch_id.c_str(), phase.c_str(),
        prev.phase_index, prev.path_revision,
        static_cast<int>(anymap_get_double(buf_payload_peek, "path_revision")));
      should_send = true;
    } else if (phase == "patrol") {
      const YAML::Node path_points = anymap_get_yaml_node(buf, "path_request_data")["path"];
      const auto [ok, _reason, matched_j] = patrol_path_matches_selected_places(
        coords_lookup_id, path_points, current_robot_x, current_robot_y);
      should_send = ok;
      patrol_matched_j = matched_j;
    } else {
      should_send = should_send_path_request_by_coords(
        phase_key, first_wp_x, first_wp_y, last_wp_x, last_wp_y,
        current_robot_x, current_robot_y);
    }
    if (!should_send) {
      remaining.push_back(buf);
      continue;
    }

    const AnyMap path_request_data = yaml_map_to_anymap(anymap_get_yaml_node(buf, "path_request_data"));
    const std::string buf_dispatch_id = trim_copy(anymap_get_string(path_request_data, "dispatch_task_id"));
    const std::string coords_booking_for_dispatch = (phase == "patrol") ? coords_lookup_id : phase_key;
    const auto [dispatch_ok, dispatch_reason] = official_dispatch_path_matches_booking(
      buf_dispatch_id, coords_booking_for_dispatch);
    if (!dispatch_ok) {
      RCLCPP_INFO(
        get_logger(),
        "⏭️ [PATH REQUEST BUFFER] Dropping buffered path for %s: %s",
        robot_name.c_str(), dispatch_reason.c_str());
      continue;
    }

    std::string order_id = currently_underway_task.count(robot_name) ?
      currently_underway_task.at(robot_name) : "";
    if (order_id.empty()) {
      order_id = phase_key;
    }
    if (order_id.empty()) {
      order_id = task_id;
    }

    const auto order_payload_opt =
      build_vda5050_order_payload(robot_name, order_id, phase, path_request_data);
    if (order_payload_opt.has_value() && mqtt_client) {
      const auto [can_publish, block_reason] = can_publish_vda_order_for_phase(robot_name, phase);
      if (!can_publish) {
        RCLCPP_INFO(
          get_logger(),
          "⏳ [PATH REQUEST BUFFER] Holding VDA order for robot=%s, orderId=%s, phase=%s: %s",
          robot_name.c_str(), order_id.c_str(), phase.c_str(), block_reason.c_str());
        remaining.push_back(buf);
        continue;
      }
      queue_vda_order_after_monitored(
        order_id, robot_name, phase, *order_payload_opt, "buffer",
        static_cast<int>(anymap_get_double(path_request_data, "phase_index")),
        static_cast<int>(anymap_get_double(path_request_data, "path_revision")));
      progressed_patrol = (phase == "patrol");
      RCLCPP_INFO(
        get_logger(),
        "📥 [VDA5050 ORDER] Queued for monitored-first publish (orderId=%s, phase=%s, source=buffer)",
        order_id.c_str(), phase.c_str());
      if (monitored_task_manager->has_started(order_id)) {
        publish_monitored_task(order_id);
        flush_pending_vda_orders(order_id);
      } else {
        update_monitored_task_cache(
          order_id,
          {
            {"task_id", YAML::Node(order_id)},
            {"amr_id", YAML::Node(robot_name)},
            {"category", YAML::Node(phase)},
          });
        handle_monitored_publish_rule(order_id, "underway");
      }
    } else if (phase == "pick" || phase == "drop" || phase == "patrol") {
      RCLCPP_WARN(
        get_logger(),
        "⚠️ [VDA5050 ORDER] Could not build order payload for robot=%s, orderId=%s, phase=%s, source=buffer",
        robot_name.c_str(), order_id.c_str(), phase.c_str());
    }

    try {
      if (phase == "patrol") {
        // Single-place patrol: no progress cursor to advance.
        (void)progressed_patrol;
        (void)patrol_matched_j;
      } else {
        sent_phases.insert(phase);
      }
      RCLCPP_INFO(
        get_logger(),
        "✅ [PATH REQUEST BUFFER] Legacy custom MQTT publish removed; VDA5050 order already "
        "published for %s (phase=%s, task_id=%s)",
        robot_name.c_str(), phase.c_str(), task_id.c_str());
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "❌ [PATH REQUEST BUFFER] Error updating phase state: %s", e.what());
      remaining.push_back(buf);
    }
    break;
  }
  pending_path_request_by_robot[robot_name] = remaining;
}

void RMFIntegratedFleetAdapter::append_pending_path_request(
  const std::string & robot_name,
  const AnyMap & path_request_data,
  const std::string & task_id,
  const std::string & coords_booking_id,
  double first_wp_x,
  double first_wp_y,
  double last_wp_x,
  double last_wp_y)
{
  YAML::Node prd(YAML::NodeType::Map);
  for (const auto & [k, v] : path_request_data) {
    prd[k] = v;
  }
  pending_path_request_by_robot[robot_name].push_back({
    {"path_request_data", prd},
    {"task_id", YAML::Node(task_id)},
    {"coords_booking_id", YAML::Node(coords_booking_id)},
    {"first_wp_x", YAML::Node(first_wp_x)},
    {"first_wp_y", YAML::Node(first_wp_y)},
    {"last_wp_x", YAML::Node(last_wp_x)},
    {"last_wp_y", YAML::Node(last_wp_y)},
  });
}

void RMFIntegratedFleetAdapter::send_pickup_for_underway_task(
  const std::string & robot_name, const std::string & task_id)
{
  // Python lines 3232–3261 (Section 12)
  try {
    const auto details_it = delivery_task_details.find(task_id);
    if (details_it == delivery_task_details.end()) {
      RCLCPP_INFO(
        get_logger(), "ℹ️ [UNDERWAY PICKUP] No task details yet for %s, waiting...",
        task_id.c_str());
      return;
    }

    AnyMap & task_details = delivery_task_details[task_id];
    if (anymap_get_bool(task_details, "pickup_sent", false)) {
      RCLCPP_INFO(
        get_logger(), "ℹ️ [UNDERWAY PICKUP] Already marked underway for %s, skipping",
        task_id.c_str());
      return;
    }

    task_details["pickup_sent"] = YAML::Node(true);
    current_delivery_task_id = task_id;
    RCLCPP_INFO(
      get_logger(),
      "✅ [UNDERWAY PICKUP] Task %s underway on %s: VDA5050 order from PathRequest carries "
      "waypoints and pick/drop actions (no separate MQTT).",
      task_id.c_str(), robot_name.c_str());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "❌ [UNDERWAY PICKUP] Error for %s: %s", task_id.c_str(), e.what());
  }
}

void RMFIntegratedFleetAdapter::send_patrol_for_underway_task(
  const std::string & robot_name, const std::string & task_id)
{
  // Python lines 3263–3280 (Section 12)
  try {
    if (task_patrol_coords.find(task_id) == task_patrol_coords.end()) {
      RCLCPP_INFO(
        get_logger(), "ℹ️ [UNDERWAY PATROL] No patrol coords yet for %s, waiting...",
        task_id.c_str());
      return;
    }
    RCLCPP_INFO(
      get_logger(),
      "✅ [UNDERWAY PATROL] Task %s underway on %s; "
      "publishing buffered VDA5050 order if path arrived before underway.",
      task_id.c_str(), robot_name.c_str());
    try_send_buffered_path_request(robot_name);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "❌ [UNDERWAY PATROL] Error for %s: %s", task_id.c_str(), e.what());
  }
}

void RMFIntegratedFleetAdapter::publish_dispenser_result(
  const std::string & base_request_id, const std::string & event_id)
{
  // Section 13 — Python lines 4038–4059
  RCLCPP_INFO(
    get_logger(),
    "🔍 [DISPENSER RESULT] Attempting to publish for base_request_id: %s, event_id: %s",
    base_request_id.c_str(), event_id.c_str());
  const auto details_it = delivery_task_details.find(base_request_id);
  if (details_it == delivery_task_details.end()) {
    RCLCPP_ERROR(
      get_logger(),
      "❌ [DISPENSER RESULT] No delivery_task_details found for base_request_id: %s",
      base_request_id.c_str());
    RCLCPP_ERROR(
      get_logger(), "❌ [DISPENSER RESULT] Available keys in delivery_task_details: %s",
      map_keys_join(delivery_task_details).c_str());
    return;
  }

  AnyMap & details = details_it->second;
  if (anymap_get_bool(details, "pickup_complete", false)) {
    RCLCPP_INFO(
      get_logger(),
      "ℹ️ [DISPENSER RESULT] Pickup already marked as complete for %s, skipping",
      base_request_id.c_str());
    return;
  }

  const std::string pickup_handler = trim_copy(anymap_get_string(details, "pickup_handler", "dispenser1"));

  rmf_dispenser_msgs::msg::DispenserResult msg;
  msg.time = get_clock()->now();
  msg.request_guid = base_request_id;
  msg.source_guid = pickup_handler;
  msg.status = rmf_dispenser_msgs::msg::DispenserResult::SUCCESS;
  dispenser_results_pub->publish(msg);
  details["pickup_complete"] = YAML::Node(true);
  RCLCPP_INFO(
    get_logger(), "✅ [DELIVERY] Published DispenserResult for %s via %s",
    base_request_id.c_str(), msg.source_guid.c_str());
  RCLCPP_INFO(
    get_logger(),
    "✅ [DELIVERY] DispenserResult message: request_guid=%s, source_guid=%s, status=%u",
    msg.request_guid.c_str(), msg.source_guid.c_str(), msg.status);
}

void RMFIntegratedFleetAdapter::publish_ingestor_result(
  const std::string & base_request_id, const std::string & event_id)
{
  // Section 13 — Python lines 4061–4091
  RCLCPP_INFO(
    get_logger(),
    "🔍 [INGESTOR RESULT] Attempting to publish for base_request_id: %s, event_id: %s",
    base_request_id.c_str(), event_id.c_str());
  const auto details_it = delivery_task_details.find(base_request_id);
  if (details_it == delivery_task_details.end()) {
    RCLCPP_ERROR(
      get_logger(),
      "❌ [INGESTOR RESULT] No delivery_task_details found for base_request_id: %s",
      base_request_id.c_str());
    RCLCPP_ERROR(
      get_logger(), "❌ [INGESTOR RESULT] Available keys in delivery_task_details: %s",
      map_keys_join(delivery_task_details).c_str());
    return;
  }

  AnyMap & details = details_it->second;
  if (anymap_get_bool(details, "dropoff_complete", false)) {
    RCLCPP_INFO(
      get_logger(),
      "ℹ️ [INGESTOR RESULT] Dropoff already marked as complete for %s, skipping",
      base_request_id.c_str());
    return;
  }

  const std::string dropoff_handler = trim_copy(anymap_get_string(details, "dropoff_handler", "ingestor1"));

  rmf_ingestor_msgs::msg::IngestorResult msg;
  msg.time = get_clock()->now();
  msg.request_guid = base_request_id;
  msg.source_guid = dropoff_handler;
  msg.status = rmf_ingestor_msgs::msg::IngestorResult::SUCCESS;
  ingestor_results_pub->publish(msg);
  details["dropoff_complete"] = YAML::Node(true);
  RCLCPP_INFO(
    get_logger(), "✅ [DELIVERY] Published IngestorResult for %s via %s",
    base_request_id.c_str(), msg.source_guid.c_str());
  RCLCPP_INFO(
    get_logger(),
    "✅ [DELIVERY] IngestorResult message: request_guid=%s, source_guid=%s, status=%u",
    msg.request_guid.c_str(), msg.source_guid.c_str(), msg.status);

  delivery_task_details.erase(base_request_id);
  task_category_by_id.erase(base_request_id);
  if (task_pickup_drop_coords.erase(base_request_id) > 0) {
    for (auto it = robot_to_coords_key.begin(); it != robot_to_coords_key.end(); ) {
      if (it->second == base_request_id) {
        it = robot_to_coords_key.erase(it);
      } else {
        ++it;
      }
    }
    RCLCPP_INFO(
      get_logger(), "🧹 [DELIVERY] Removed %s from task_pickup_drop_coords",
      base_request_id.c_str());
  }
}

void RMFIntegratedFleetAdapter::path_request_callback(
  const rmf_fleet_msgs::msg::PathRequest::SharedPtr msg)
{
  // ---- Phase 2 release: leg change only ---------------------------------
  // This used to clear the closure on ANY path for this robot, on the reasoning
  // that a path means RMF has replanned and the closure has done its job. It
  // has not: the first path to arrive IS the replan the closure just caused, so
  // the lanes reopened before the robot had moved a centimetre (logged as
  // "held 0.0s"). RMF then routed the robot straight back through the conflict
  // zone and the whole cycle repeated — detect/resolve alternating on a 1.000 s
  // tick, four rounds, 7 cancelOrders in 4 s, orderUpdateId to 8 in 8 s.
  //
  // A closure now lives for the whole leg. The only in-task release is the leg
  // itself changing (delivery pickup 0 -> drop 1); otherwise it is held until
  // the task ends (clear_task_state).
  if (msg && !msg->robot_name.empty()) {
    const auto closure_it = private_closure_by_robot.find(msg->robot_name);
    if (closure_it != private_closure_by_robot.end()) {
      const PrivateClosure & held = closure_it->second;
      const std::string incoming_task = trim_copy(msg->dispatch_task_id);

      // Task change. This is the one that covers a task simply finishing —
      // there is no completion hook that releases the closure, so the arrival
      // of a path belonging to a DIFFERENT task is the signal that the task the
      // closure was made for is over. Only compare when both ids are known; a
      // stop marker carries an empty dispatch id and must not release anything.
      const bool task_changed =
        !held.order_id.empty() && !incoming_task.empty() &&
        held.order_id != incoming_task;

      // Leg change within the same task (delivery pickup 0 -> drop 1).
      const bool leg_changed =
        !task_changed && held.phase_index >= 0 &&
        (incoming_task.empty() || held.order_id.empty() ||
        held.order_id == incoming_task) &&
        held.phase_index != static_cast<int>(msg->phase_index);

      if (task_changed) {
        RCLCPP_INFO(
          get_logger(),
          "🔓 [ZONE REPLAN] robot=%s moved from task %s to task %s — releasing "
          "its private lane closure (blocked=%s, conflict=%s, held %.0fs)",
          msg->robot_name.c_str(), held.order_id.c_str(),
          incoming_task.c_str(),
          held.blocked_zones.empty() ? "?" : held.blocked_zones.c_str(),
          held.zone_id.c_str(),
          (get_clock()->now() - held.applied_at).seconds());
        clear_private_lane_closure(msg->robot_name, "task changed");
      } else if (leg_changed) {
        RCLCPP_INFO(
          get_logger(),
          "🔓 [ZONE REPLAN] robot=%s moved from leg %d to leg %u — releasing its "
          "private lane closure",
          msg->robot_name.c_str(), held.phase_index, msg->phase_index);
        clear_private_lane_closure(msg->robot_name, "leg changed");
      } else if (held.corridor_hold) {
        OnDropCorridorPath(*msg);
      }
    }
  }

  // Python lines 1328–1545
  try {
    RCLCPP_INFO(get_logger(), "📨 Received path request for robot: %s", msg->robot_name.c_str());
    RCLCPP_INFO(get_logger(), "🆔 Task ID: %s", msg->task_id.c_str());
    RCLCPP_INFO(get_logger(), "🛤️  Original path with %zu waypoints", msg->path.size());

    if (msg->task_id.empty() || trim_copy(msg->task_id).empty()) {
      RCLCPP_WARN(get_logger(), "⚠️  Ignoring path request - no valid task ID: '%s'", msg->task_id.c_str());
      return;
    }

    if (!is_valid_rmf_task(msg->task_id)) {
      RCLCPP_WARN(get_logger(), "⚠️  Ignoring path request - invalid task format: '%s'", msg->task_id.c_str());
      return;
    }

    if (!fleet_config.has_value() || fleet_config->robots.find(msg->robot_name) == fleet_config->robots.end()) {
      RCLCPP_WARN(get_logger(), "⚠️  Unknown robot: %s", msg->robot_name.c_str());
      return;
    }

    if (msg->path.empty()) {
      RCLCPP_WARN(get_logger(), "⚠️  Ignoring path request - no waypoints provided");
      return;
    }

    const std::string path_dispatch_task_id = path_request_dispatch_task_id(*msg);
    if (!path_dispatch_task_id.empty()) {
      clear_stale_local_cancel_status(path_dispatch_task_id, "RMF PathRequest with dispatch_task_id");
    }
    if (!path_dispatch_task_id.empty()) {
      const auto status_it = known_task_status.find(path_dispatch_task_id);
      const std::string status = (status_it != known_task_status.end()) ? status_it->second : "";
      if (is_terminal_task_status(status)) {
        RCLCPP_WARN(
          get_logger(),
          "⚠️ [PATH REQUEST] Ignoring path for terminal task %s (status=%s)",
          path_dispatch_task_id.c_str(), status.c_str());
        return;
      }
    }

    AMRRobot & robot = fleet_config->robots[msg->robot_name];

    auto delay_it = task_completion_delays.find(msg->robot_name);
    if (delay_it != task_completion_delays.end()) {
      const std::string old_task_id = delay_it->second.first;
      RCLCPP_INFO(
        get_logger(),
        "🎯 [TASK ASSIGNMENT] Clearing completion delay for old task %s on robot %s",
        old_task_id.c_str(), msg->robot_name.c_str());
      task_completion_delays.erase(delay_it);
      if (active_task_robots.erase(old_task_id) > 0) {
        sent_path_request_phase.erase(old_task_id);
        RCLCPP_INFO(
          get_logger(), "🧹 [TASK ASSIGNMENT] Cleared old task %s from active_task_robots",
          old_task_id.c_str());
      }
    }

    auto wp_seq_it = robot_waypoint_sequences.find(msg->robot_name);
    if (wp_seq_it != robot_waypoint_sequences.end()) {
      RCLCPP_INFO(
        get_logger(),
        "🎯 [TASK ASSIGNMENT] Clearing %zu old waypoints for robot %s",
        wp_seq_it->second.size(), msg->robot_name.c_str());
      robot_waypoint_sequences.erase(wp_seq_it);
    }

    target_positions.erase(msg->robot_name);
    robot_path_assigned[msg->robot_name] = false;
    robot_task_cleared[msg->robot_name] = false;
    robot_path_cleared.erase(msg->robot_name);

    auto visited_it = robot_visited_waypoints.find(msg->robot_name);
    if (visited_it != robot_visited_waypoints.end()) {
      const std::size_t old_visited_count = visited_it->second.size();
      visited_it->second.clear();
      RCLCPP_INFO(
        get_logger(),
        "🔄 [VISITED TRACKING] Cleared %zu visited waypoints for new task on robot %s",
        old_visited_count, msg->robot_name.c_str());
    }

    const std::string rmf_path_task_id = trim_copy(msg->task_id);
    const std::string resolved_booking = resolve_booking_task_id_for_path_request(
      msg->robot_name, rmf_path_task_id, &msg->path, path_dispatch_task_id);
    std::string booking_id = resolve_coords_booking_id(msg->robot_name, resolved_booking);
    if (booking_id.empty() && is_dispatch_booking_id(resolved_booking)) {
      booking_id = resolved_booking;
    }

    if (!booking_id.empty() && is_dispatch_booking_id(booking_id)) {
      const auto booking_status_it = known_task_status.find(booking_id);
      const std::string booking_status =
        (booking_status_it != known_task_status.end()) ? booking_status_it->second : "";
      if (is_terminal_task_status(booking_status)) {
        RCLCPP_WARN(
          get_logger(),
          "⚠️ [PATH REQUEST] Ignoring resolved booking %s — terminal status",
          booking_id.c_str());
        return;
      }
      dispatch_assignments[booking_id] = msg->robot_name;
      if (!path_dispatch_task_id.empty()) {
        RCLCPP_INFO(
          get_logger(),
          "📋 [PATH REQUEST] Booking from dispatch_task_id='%s' for robot %s",
          path_dispatch_task_id.c_str(), msg->robot_name.c_str());
      }
    }

    for (auto it = active_task_robots.begin(); it != active_task_robots.end(); ) {
      if (it->second == msg->robot_name) {
        it = active_task_robots.erase(it);
      } else {
        ++it;
      }
    }

    if (booking_id.empty() || !is_dispatch_booking_id(booking_id)) {
      RCLCPP_WARN(
        get_logger(),
        "⚠️ [PATH REQUEST] Booking id not resolved. robot=%s, rmf_path_task_id=%s. "
        "Will not use numeric path task id for coordinate lookup.",
        msg->robot_name.c_str(), rmf_path_task_id.c_str());
      robot.fleet_publish_task_id = rmf_path_task_id;
      path_request_send_or_buffer(msg);
      return;
    }

    robot.fleet_publish_task_id = rmf_path_task_id;
    robot.current_task = booking_id;
    robot.status = RobotStatus::Navigating;
    robot_path_assigned[msg->robot_name] = true;

    RCLCPP_INFO(
      get_logger(),
      "🆔 [RMF TASK ASSIGNMENT] Robot %s: RMF path task_id='%s', internal booking='%s', "
      "fleet_state will publish numeric id",
      msg->robot_name.c_str(), rmf_path_task_id.c_str(), booking_id.c_str());
    RCLCPP_INFO(
      get_logger(),
      "🆔 [RMF TASK ASSIGNMENT] Robot %s: robot.current_task (booking)='%s'",
      msg->robot_name.c_str(), robot.current_task.c_str());

    double first_wp_x = 0.0;
    double first_wp_y = 0.0;
    double last_wp_x = 0.0;
    double last_wp_y = 0.0;
    if (!msg->path.empty()) {
      first_wp_x = msg->path.front().x;
      first_wp_y = msg->path.front().y;
      last_wp_x = msg->path.back().x;
      last_wp_y = msg->path.back().y;
    }

    const bool is_patrol_booking = !booking_id.empty() &&
      (is_patrol_task(booking_id) || task_patrol_coords.count(booking_id) > 0);
    const bool is_delivery_booking = task_pickup_drop_coords.count(booking_id) > 0;
    std::optional<std::string> early_patrol_task_key;
    if (is_delivery_booking &&
      dispatch_assignments.count(booking_id) > 0 &&
      dispatch_assignments.at(booking_id) == msg->robot_name)
    {
      // Delivery itinerary takes precedence over stale patrol coords on same robot.
    } else if (task_patrol_coords.count(booking_id) > 0) {
      early_patrol_task_key = booking_id;
    } else if (is_patrol_task(booking_id)) {
      early_patrol_task_key = booking_id;
    } else if (is_patrol_task(path_dispatch_task_id)) {
      early_patrol_task_key = path_dispatch_task_id;
    }

    // ---- Replan of the leg already published to this robot? --------------
    // MUST be evaluated BEFORE the patrol/coords gate below. A replan revisits
    // the SAME destination, but patrol_path_matches_selected_places() walks
    // forward from task_patrol_progress and reports "all patrol destinations
    // already matched" for a single-place patrol whose place was consumed by
    // the FIRST path. That rejected every replanned path before it could ever
    // reach path_request_send_or_buffer().
    const std::string replan_dispatch_id =
      !path_dispatch_task_id.empty() ? path_dispatch_task_id : booking_id;
    const bool early_traffic_extension = is_traffic_horizon_extension(
      msg->robot_name, replan_dispatch_id, *msg);
    const bool early_is_replan = !early_traffic_extension &&
      is_replan_of_published_order(
      msg->robot_name,
      replan_dispatch_id,
      static_cast<int>(msg->phase_index),
      static_cast<int>(msg->path_revision),
      last_wp_x, last_wp_y);

    bool early_should_send = false;
    std::string early_reject_reason;
    if (early_is_replan) {
      // Same task, same leg, higher revision, same destination: this path is a
      // replan and must be forwarded regardless of patrol progress.
      early_should_send = true;
      const auto & prev = published_order_by_robot.at(msg->robot_name);
      RCLCPP_INFO(
        get_logger(),
        "🔁 [REPLAN ORDER] robot=%s orderId=%s leg=%d — revision %d -> %d, "
        "waypoints=%zu, destination unchanged; bypassing patrol/coords gate",
        msg->robot_name.c_str(), replan_dispatch_id.c_str(), prev.phase_index,
        prev.path_revision, static_cast<int>(msg->path_revision), msg->path.size());
    } else if (early_traffic_extension) {
      early_should_send = true;
      RCLCPP_INFO(
        get_logger(),
        "🛤️ [PATH REQUEST] Traffic horizon extension robot=%s orderId=%s — "
        "path_revision=%u, bypassing patrol/coords gate",
        msg->robot_name.c_str(), replan_dispatch_id.c_str(), msg->path_revision);
    } else if (early_patrol_task_key.has_value()) {
      const auto [ok, reason, _matched_j] = patrol_path_matches_selected_places(
        *early_patrol_task_key, msg->path, robot.pose.x, robot.pose.y);
      early_should_send = ok;
      early_reject_reason = reason;
    } else {
      early_should_send = should_send_path_request_by_coords(
        booking_id, first_wp_x, first_wp_y, last_wp_x, last_wp_y,
        robot.pose.x, robot.pose.y);
      early_reject_reason = "coordinates do not match the task's expected pickup/dropoff";
    }

    if (!early_should_send) {
      const bool waiting_patrol_coords = early_patrol_task_key.has_value() &&
        task_patrol_coords.count(booking_id) == 0;
      const bool waiting_delivery_coords =
        is_delivery_booking && task_pickup_drop_coords.count(booking_id) == 0;
      const bool no_booking_for_coords =
        booking_id.empty() || !is_dispatch_booking_id(booking_id);
      // !is_replan: a replan targets the SAME destination by definition, so
      // path_last_waypoint_matches_delivery_target() always matches and the
      // path would be parked in the buffer instead of forwarded. A replan must
      // never be buffered — it is the robot's correction to a conflict already
      // in progress.
      const bool buffer_delivery_path =
        !early_is_replan && is_delivery_booking &&
        path_last_waypoint_matches_delivery_target(booking_id, last_wp_x, last_wp_y);

      if (waiting_patrol_coords || waiting_delivery_coords || no_booking_for_coords ||
        buffer_delivery_path)
      {
        try {
          const AnyMap path_request_data = build_path_request_payload(*msg);
          append_pending_path_request(
            msg->robot_name, path_request_data, rmf_path_task_id, booking_id,
            first_wp_x, first_wp_y, last_wp_x, last_wp_y);
          RCLCPP_INFO(
            get_logger(),
            "📥 [PATH REQUEST] Buffered path before waypoint storage for robot %s "
            "(booking=%s, rmf_path_task_id=%s, delivery_target_match=%s)",
            msg->robot_name.c_str(), booking_id.c_str(), rmf_path_task_id.c_str(),
            buffer_delivery_path ? "true" : "false");
        } catch (const std::exception & e) {
          RCLCPP_WARN(get_logger(), "⚠️ [PATH REQUEST] Failed to buffer: %s", e.what());
        }
      } else {
        const bool in_drop =
          dropoff_in_progress.count(booking_id) > 0 && dropoff_in_progress.at(booking_id);
        const char * phase_desc = early_patrol_task_key.has_value() ? "patrol" :
          (in_drop ? "drop" : "pickup");
        // Log the ACTUAL reason from the matcher. The previous generic wording
        // ("last waypoint must match expected coords") pointed at coordinates
        // even when they matched perfectly and the real cause was patrol
        // progress exhaustion — which made a replan rejection very hard to
        // diagnose.
        RCLCPP_INFO(
          get_logger(),
          "⏭️ [PATH REQUEST] Ignoring before waypoint storage - reason: %s "
          "[first wp (%.2f, %.2f) vs robot pose (%.2f, %.2f), last wp (%.2f, %.2f), "
          "task %s, phase %s, revision %u]",
          early_reject_reason.empty() ? "no matching rule" : early_reject_reason.c_str(),
          first_wp_x, first_wp_y, robot.pose.x, robot.pose.y,
          last_wp_x, last_wp_y, booking_id.c_str(), phase_desc, msg->path_revision);
      }
      return;
    }

    RCLCPP_INFO(
      get_logger(), "🔍 [DEBUG] Robot %s current position: (%.6f, %.6f)",
      msg->robot_name.c_str(), robot.pose.x, robot.pose.y);
    RCLCPP_INFO(get_logger(), "🔍 [DEBUG] Received %zu waypoints from RMF", msg->path.size());
    for (std::size_t i = 0; i < msg->path.size(); ++i) {
      RCLCPP_INFO(
        get_logger(), "🔍 [DEBUG] Waypoint %zu: (%.6f, %.6f, yaw=%.3f)",
        i + 1, msg->path[i].x, msg->path[i].y, msg->path[i].yaw);
    }

    const std::size_t released_in_msg = std::count_if(
      msg->path.begin(), msg->path.end(),
      [](const rmf_fleet_msgs::msg::Location & wp) { return wp.release; });
    RCLCPP_INFO(
      get_logger(),
      "🛤️ [PATH REQUEST] robot=%s full path=%zu waypoints, release=true on %zu "
      "(VDA5050 full leg with released flags; fleet_state reports released remaining only)",
      msg->robot_name.c_str(), msg->path.size(), released_in_msg);

    const std::vector<rmf_fleet_msgs::msg::Location> filtered_path = store_and_filter_waypoints(
      msg->robot_name, msg->path, 0.5, !is_patrol_booking);

    // planned_path is built from the FULL RMF path (msg->path), NOT from
    // filtered_path.
    //
    // filtered_path drops waypoints the robot has already visited, and this
    // callback fires on every PathRequest, so planned_path used to shrink as the
    // robot advanced (observed: 15 points -> 2 points on one patrol task) while
    // the VDA5050 order still carried the full route. Two consequences:
    //
    //   1. /Monitored_task_details disagreed with the order actually sent.
    //   2. zone_monitor derives /robot_zone_status from planned_path, and
    //      zone_manager's classify_travel_direction() needs the zones BEFORE the
    //      conflict zone (prefix_before) to recognise a head-on approach. With a
    //      truncated path those zones vanish, the conflict is classified as
    //      "behind" (class=1), no pause target is chosen, and no ReplanCandidate
    //      is published — so per-robot replanning never triggers.
    //
    // zone_monitor still positions the sequence itself via
    // ZoneTracker::apply_remaining_and_lookahead(), which trims to
    // "current + remaining" using the robot's current zone index. Trimming here
    // as well was a double trim.
    std::vector<std::string> planned_path;
    planned_path.reserve(msg->path.size());
    for (const auto & wp : msg->path) {
      std::ostringstream oss;
      oss << std::fixed << std::setprecision(3) << wp.x << "," << wp.y << "," << wp.yaw;
      planned_path.push_back(oss.str());
    }
    // ---- Guard 2: stage, do not publish -----------------------------------
    // See StagedPlannedPath in the header. The zone layer must not learn about
    // this route until the robot has actually been given it, otherwise the
    // conflict that is holding the robot resolves against a path the robot is
    // not yet driving and zone_manager resumes it onto its old route.
    const std::string monitored_task_id =
      resolve_monitored_task_id(msg->robot_name, booking_id);
    const std::string msg_fleet_name =
      msg->fleet_name.empty() ? fleet_name : msg->fleet_name;

    const std::string stage_key = staged_path_key(
      static_cast<int>(msg->phase_index), static_cast<int>(msg->path_revision));
    auto & robot_stages = staged_planned_path_by_robot[msg->robot_name];

    const auto stage_result = robot_stages.emplace(
      stage_key,
      StagedPlannedPath{
        planned_path, monitored_task_id, msg_fleet_name, filtered_path.size()});

    if (stage_result.second) {
      RCLCPP_INFO(
        get_logger(),
        "🗂️ [PLANNED PATH] robot=%s staged %zu points for leg %u revision %u "
        "(full RMF path; filtered/remaining=%zu) — held back from "
        "/Monitored_task_details until the order reaches the robot",
        msg->robot_name.c_str(), planned_path.size(),
        msg->phase_index, msg->path_revision, filtered_path.size());
    } else {
      // Same leg, same revision, second route. Only the first one can publish
      // (the revision gate suppresses the rest), so the first stage is the one
      // that describes what the robot will actually be driving.
      RCLCPP_INFO(
        get_logger(),
        "🗂️ [PLANNED PATH] robot=%s already has a staged path for leg %u "
        "revision %u — keeping the first (%zu points), ignoring this one "
        "(%zu points)",
        msg->robot_name.c_str(), msg->phase_index, msg->path_revision,
        stage_result.first->second.path.size(), planned_path.size());
    }

    if (!filtered_path.empty()) {
      std::vector<AnyMap> waypoint_sequence;
      waypoint_sequence.reserve(filtered_path.size());
      for (std::size_t i = 0; i < filtered_path.size(); ++i) {
        const auto & waypoint = filtered_path[i];
        waypoint_sequence.push_back({
            {"x", YAML::Node(waypoint.x)},
            {"y", YAML::Node(waypoint.y)},
            {"yaw", YAML::Node(waypoint.yaw)},
            {"index", YAML::Node(static_cast<int>(i))},
            {"release", YAML::Node(waypoint.release)},
            {"traffic_hold", YAML::Node(waypoint.traffic_hold)},
            {"traffic_wait_sec", YAML::Node(waypoint.traffic_wait_sec)},
            {"traffic_blocker_amr", YAML::Node(waypoint.traffic_blocker_amr)},
          });
      }

      if (early_traffic_extension) {
        const auto prev_it = robot_waypoint_sequences.find(msg->robot_name);
        if (prev_it != robot_waypoint_sequences.end() && !prev_it->second.empty()) {
          std::vector<AnyMap> prior = prev_it->second;
          sync_release_flags_on_sequence(prior, msg->path);
          trim_waypoint_sequence_from_pose(
            prior, msg->robot_name, robot.pose.x, robot.pose.y, ARRIVAL_THRESHOLD_M);
          sync_release_flags_on_sequence(waypoint_sequence, msg->path);
          trim_waypoint_sequence_from_pose(
            waypoint_sequence, msg->robot_name, robot.pose.x, robot.pose.y,
            ARRIVAL_THRESHOLD_M);
          if (!prior.empty() && !waypoint_sequence.empty()) {
            const double d_prior = std::hypot(
              robot.pose.x - anymap_get_double(prior.front(), "x"),
              robot.pose.y - anymap_get_double(prior.front(), "y"));
            const double d_new = std::hypot(
              robot.pose.x - anymap_get_double(waypoint_sequence.front(), "x"),
              robot.pose.y - anymap_get_double(waypoint_sequence.front(), "y"));
            if (d_prior + 0.15 < d_new) {
              append_missing_tail_waypoints(prior, waypoint_sequence);
              waypoint_sequence = std::move(prior);
              RCLCPP_INFO(
                get_logger(),
                "🛤️ [WAYPOINT SEQUENCE] Traffic extension: kept progress (%zu wps) for %s",
                waypoint_sequence.size(), msg->robot_name.c_str());
            }
          }
        }
      }

      advance_waypoint_sequence_if_progressed(msg->robot_name, robot);
      trim_waypoint_sequence_to_robot_progress(
        waypoint_sequence, msg->robot_name, robot.pose.x, robot.pose.y);

      robot_waypoint_sequences[msg->robot_name] = waypoint_sequence;

      if (!waypoint_sequence.empty()) {
        // Next released waypoint is the drive target. Advancement stops at any
        // traffic_hold waypoint so the AMR waits until ECBS clears it.
        const AnyMap * drive_target = nullptr;
        for (const auto & wp : waypoint_sequence) {
          if (anymap_get_bool(wp, "release", false)) {
            drive_target = &wp;
            break;
          }
        }
        if (drive_target != nullptr) {
        target_positions[msg->robot_name] = {
            {"x", YAML::Node(anymap_get_double(*drive_target, "x"))},
            {"y", YAML::Node(anymap_get_double(*drive_target, "y"))},
        };
        RCLCPP_INFO(
          get_logger(),
            "🎯 [WAYPOINT SEQUENCE] Stored %zu waypoints for %s (release flags preserved)",
          waypoint_sequence.size(), msg->robot_name.c_str());
        RCLCPP_INFO(
          get_logger(),
            "🎯 [WAYPOINT SEQUENCE] Current released target: (%.6f, %.6f)%s",
            anymap_get_double(*drive_target, "x"),
            anymap_get_double(*drive_target, "y"),
            anymap_is_traffic_hold(*drive_target) ? " [TRAFFIC HOLD]" : "");
        } else {
          RCLCPP_WARN(
            get_logger(),
            "🎯 [WAYPOINT SEQUENCE] Stored %zu waypoints for %s but no released target",
            waypoint_sequence.size(), msg->robot_name.c_str());
        }
      } else {
        RCLCPP_WARN(
          get_logger(), "⚠️ [WAYPOINT SEQUENCE] No waypoints in filtered path for %s",
          msg->robot_name.c_str());
      }
    } else {
      RCLCPP_WARN(
        get_logger(), "⚠️ [WAYPOINT SEQUENCE] All waypoints filtered out for %s",
        msg->robot_name.c_str());
    }

    RCLCPP_INFO(
      get_logger(), "✅ Processing valid RMF task: %s for robot %s",
      msg->task_id.c_str(), msg->robot_name.c_str());
    RCLCPP_INFO(get_logger(), "🚀 Robot %s status set to NAVIGATING", msg->robot_name.c_str());

    active_task_robots[booking_id] = msg->robot_name;
    last_known_tasks[msg->robot_name] = booking_id;
    RCLCPP_INFO(
      get_logger(),
      "📋 [PATH REQUEST] Stored booking '%s' -> robot '%s' for path request matching",
      booking_id.c_str(), msg->robot_name.c_str());
    {
      std::ostringstream mapping;
      mapping << "{";
      bool first = true;
      for (const auto & [tid, rname] : active_task_robots) {
        if (!first) {
          mapping << ", ";
        }
        first = false;
        mapping << "'" << tid << "': '" << rname << "'";
      }
      mapping << "}";
      RCLCPP_INFO(get_logger(), "📋 [PATH REQUEST] Active tasks mapping: %s", mapping.str().c_str());
    }
    RCLCPP_INFO(
      get_logger(), "📋 [PATH REQUEST] Robot %s current_task set to: '%s'",
      msg->robot_name.c_str(), robot.current_task.c_str());

    path_request_send_or_buffer(msg);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "❌ Failed to handle path request: %s", e.what());
  }
}

bool RMFIntegratedFleetAdapter::is_valid_rmf_task(const std::string & task_id)
{
  // Python lines 1547–1574
  if (task_id.empty() || trim_copy(task_id).empty()) {
    return false;
  }

  static const char * valid_patterns[] = {
    "delivery.dispatch-",
    "patrol.dispatch-",
    "drop.dispatch-",
  };
  for (const char * pattern : valid_patterns) {
    if (task_id.rfind(pattern, 0) == 0) {
      RCLCPP_INFO(get_logger(), "✅ Accepting RMF task: %s", task_id.c_str());
      return true;
    }
  }

  // Robot-targeted (direct) dispatch: rmf_fleet_adapter reuses the api-server
  // request_id as the task id, which is a uuid4. Accept it when we have already
  // recorded its category, and by shape otherwise.
  const std::string trimmed = trim_copy(task_id);
  if (task_category_by_id.count(trimmed) > 0) {
    RCLCPP_INFO(
      get_logger(), "✅ Accepting known %s task: %s",
      task_category_by_id.at(trimmed).c_str(), task_id.c_str());
    return true;
  }
  if (is_uuid_task_id(trimmed)) {
    RCLCPP_INFO(
      get_logger(), "✅ Accepting direct-dispatch (uuid) task ID: %s", task_id.c_str());
    return true;
  }

  // NOTE: previously std::stoi() was used here, which accepts any string that
  // merely *starts* with digits (e.g. "35f89c7a-..." -> 3). That let roughly
  // 62% of uuid task ids through at random while rejecting the rest. Require
  // the whole id to be digits instead.
  const bool all_digits = !trimmed.empty() &&
    std::all_of(
    trimmed.begin(), trimmed.end(),
    [](unsigned char c) { return std::isdigit(c) != 0; });
  if (all_digits) {
    RCLCPP_INFO(get_logger(), "⚠️  Accepting numeric task ID: %s", task_id.c_str());
    return true;
  }

  RCLCPP_WARN(get_logger(), "🚫 Rejecting unknown task ID format: %s", task_id.c_str());
  return false;
}

// ============================================================================
// Section 10 — Path send/buffer (Python lines 2431–2779)
// ============================================================================

void RMFIntegratedFleetAdapter::path_request_send_or_buffer(
  const rmf_fleet_msgs::msg::PathRequest::SharedPtr msg)
{
  // Python lines 2431–2779
  try {
    RCLCPP_INFO(
      get_logger(),
      "📨 [PATH REQUEST] Processing path request for robot: %s, task_id: %s, waypoints: %zu",
      msg->robot_name.c_str(), msg->task_id.c_str(), msg->path.size());

    const std::string robot_name = msg->robot_name;
    if (robot_name.empty() || !fleet_config.has_value() ||
      fleet_config->robots.find(robot_name) == fleet_config->robots.end())
    {
      std::ostringstream valid_robots;
      if (fleet_config.has_value()) {
        bool first = true;
        for (const auto & [rid, _] : fleet_config->robots) {
          if (!first) {
            valid_robots << ", ";
          }
          first = false;
          valid_robots << rid;
        }
      }
      RCLCPP_WARN(
        get_logger(),
        "⚠️ [PATH REQUEST] Invalid or unknown robot: robot_name='%s', valid robots: %s",
        robot_name.c_str(), valid_robots.str().c_str());
      return;
    }

    AMRRobot & robot = fleet_config->robots[robot_name];
    const std::string rmf_path_task_id = trim_copy(msg->task_id);
    const std::string path_dispatch_task_id = path_request_dispatch_task_id(*msg);
    if (!path_dispatch_task_id.empty()) {
      clear_stale_local_cancel_status(
        path_dispatch_task_id, "RMF PathRequest with dispatch_task_id (send/buffer)");
    }
    if (!path_dispatch_task_id.empty()) {
      const auto status_it = known_task_status.find(path_dispatch_task_id);
      const std::string status = (status_it != known_task_status.end()) ? status_it->second : "";
      if (is_terminal_task_status(status)) {
        RCLCPP_WARN(
          get_logger(),
          "⚠️ [PATH REQUEST] Skipping send/buffer for terminal task %s",
          path_dispatch_task_id.c_str());
        return;
      }
    }

    std::string booking_for_path = resolve_booking_task_id_for_path_request(
      robot_name, rmf_path_task_id, &msg->path, path_dispatch_task_id);
    const std::string coords_booking_id = resolve_coords_booking_id(robot_name, booking_for_path);
    std::string task_id = trim_copy(robot.current_task);
    if (task_id.empty()) {
      task_id = coords_booking_id;
    }
    if (task_id.empty()) {
      task_id = booking_for_path;
    }
    if (task_id.empty()) {
      task_id = rmf_path_task_id;
    }
    task_id = trim_copy(task_id);
    if (task_id.empty()) {
      RCLCPP_WARN(
        get_logger(), "⚠️ [PATH REQUEST] No task_id for robot %s, skipping send/buffer",
        robot_name.c_str());
      return;
    }
    if (!coords_booking_id.empty()) {
      RCLCPP_INFO(
        get_logger(),
        "📋 [PATH REQUEST] Coordinate lookup booking id=%s (rmf_path_task_id=%s)",
        coords_booking_id.c_str(), rmf_path_task_id.c_str());
    }

    const auto rk_it = robot_to_coords_key.find(robot_name);
    const std::string robot_delivery_coords_key = (rk_it != robot_to_coords_key.end()) ? rk_it->second : "";
    const bool robot_has_delivery_itinerary =
      !robot_delivery_coords_key.empty() &&
      task_pickup_drop_coords.count(robot_delivery_coords_key) > 0 &&
      dispatch_assignments.count(robot_delivery_coords_key) > 0 &&
      dispatch_assignments.at(robot_delivery_coords_key) == robot_name &&
      coords_booking_id == robot_delivery_coords_key;

    std::optional<std::string> patrol_task_key;
    if (robot_has_delivery_itinerary) {
      RCLCPP_INFO(
        get_logger(),
        "📋 [PATH REQUEST] Delivery itinerary for %s (%s): path matched booking — "
        "skip patrol validation (avoids stale patrol underway during handoff)",
        robot_name.c_str(), robot_delivery_coords_key.c_str());
    } else if (task_patrol_coords.count(coords_booking_id) > 0) {
      patrol_task_key = coords_booking_id;
    } else if (is_patrol_task(coords_booking_id)) {
      patrol_task_key = coords_booking_id;
    } else if (is_patrol_task(path_dispatch_task_id)) {
      patrol_task_key = path_dispatch_task_id;
    }

    double first_wp_x = 0.0;
    double first_wp_y = 0.0;
    double last_wp_x = 0.0;
    double last_wp_y = 0.0;
    if (!msg->path.empty()) {
      first_wp_x = msg->path.front().x;
      first_wp_y = msg->path.front().y;
      last_wp_x = msg->path.back().x;
      last_wp_y = msg->path.back().y;
    }
    const double current_robot_x = robot.pose.x;
    const double current_robot_y = robot.pose.y;

    const bool patrol_task = patrol_task_key.has_value();
    const std::string extension_booking = !path_dispatch_task_id.empty() ?
      path_dispatch_task_id : coords_booking_id;
    const bool traffic_extension = is_traffic_horizon_extension(
      robot_name, extension_booking, *msg);
    int patrol_matched_end_idx = -1;
    bool should_send = false;
    std::string patrol_reason;
    if (patrol_task) {
      const auto [ok, reason, matched_j] = patrol_path_matches_selected_places(
        *patrol_task_key, msg->path, current_robot_x, current_robot_y,
        traffic_extension);
      should_send = ok;
      patrol_reason = reason;
      patrol_matched_end_idx = matched_j;
      if (traffic_extension && ok) {
        RCLCPP_INFO(
          get_logger(),
          "🛤️ [PATH REQUEST] Traffic horizon extension for robot %s booking=%s "
          "(path_revision=%u, released waypoints advanced)",
          robot_name.c_str(), extension_booking.c_str(), msg->path_revision);
      }
    } else {
      should_send = should_send_path_request_by_coords(
        coords_booking_id, first_wp_x, first_wp_y, last_wp_x, last_wp_y,
        current_robot_x, current_robot_y) ||
        traffic_extension;
    }

    if (!should_send) {
      const std::string lookup_key = !coords_booking_id.empty() ? coords_booking_id : task_id;
      const bool waiting_patrol_coords = patrol_task_key.has_value() &&
        (coords_booking_id.empty() ||
        task_patrol_coords.count(coords_booking_id) == 0) &&
        (is_patrol_task(coords_booking_id) ||
        is_patrol_task(path_dispatch_task_id));
      const bool waiting_delivery_coords = !patrol_task_key.has_value() &&
        !coords_booking_id.empty() &&
        task_pickup_drop_coords.count(coords_booking_id) == 0;
      const bool no_booking_for_coords =
        coords_booking_id.empty() || !is_dispatch_booking_id(coords_booking_id);
      const bool is_delivery_booking =
        !patrol_task_key.has_value() &&
        task_pickup_drop_coords.count(coords_booking_id) > 0;
      // A replan targets the SAME destination by definition, so
      // path_last_waypoint_matches_delivery_target() always matches and the
      // path would be parked in the buffer instead of forwarded. A replan must
      // never be buffered — it is the robot's correction to a conflict already
      // in progress. Computed locally: this function runs before the is_replan
      // flag further down is available.
      const bool send_or_buffer_is_replan = !traffic_extension &&
        is_replan_of_published_order(
        msg->robot_name,
        !path_dispatch_task_id.empty() ? path_dispatch_task_id : coords_booking_id,
        static_cast<int>(msg->phase_index),
        static_cast<int>(msg->path_revision),
        last_wp_x, last_wp_y);
      const bool buffer_delivery_path =
        !send_or_buffer_is_replan && is_delivery_booking &&
        path_last_waypoint_matches_delivery_target(coords_booking_id, last_wp_x, last_wp_y);

      if (waiting_patrol_coords || waiting_delivery_coords || no_booking_for_coords ||
        buffer_delivery_path)
      {
        try {
          const AnyMap path_request_data = build_path_request_payload(*msg);
          append_pending_path_request(
            robot_name, path_request_data, task_id, coords_booking_id,
            first_wp_x, first_wp_y, last_wp_x, last_wp_y);
          RCLCPP_INFO(
            get_logger(),
            "📥 [PATH REQUEST] Buffered path request for robot %s "
            "(booking_coords=%s, rmf_path_task_id=%s, delivery_target_match=%s)",
            robot_name.c_str(),
            coords_booking_id.empty() ? "pending" : coords_booking_id.c_str(),
            rmf_path_task_id.c_str(),
            buffer_delivery_path ? "true" : "false");
        } catch (const std::exception & e) {
          RCLCPP_WARN(get_logger(), "⚠️ [PATH REQUEST] Failed to buffer: %s", e.what());
        }
      } else {
        const bool in_drop =
          dropoff_in_progress.count(lookup_key) > 0 && dropoff_in_progress.at(lookup_key);
        const char * phase_desc = patrol_task ? "patrol" : (in_drop ? "drop" : "pickup");
        if (patrol_task) {
          RCLCPP_INFO(
            get_logger(),
            "⏭️ [PATH REQUEST] Ignoring patrol path for task %s: %s",
            task_id.c_str(), patrol_reason.c_str());
        } else {
          RCLCPP_INFO(
            get_logger(),
            "⏭️ [PATH REQUEST] Ignoring - first waypoint (%.1f, %.1f) must match robot pose "
            "(%.1f, %.1f) and last waypoint (%.1f, %.1f) must match expected coords for task %s (phase: %s)",
            first_wp_x, first_wp_y, current_robot_x, current_robot_y,
            last_wp_x, last_wp_y, task_id.c_str(), phase_desc);
        }
      }
      return;
    }

    const auto [dispatch_ok, dispatch_reason] = official_dispatch_path_matches_booking(
      path_dispatch_task_id, coords_booking_id);
    if (!dispatch_ok) {
      RCLCPP_INFO(
        get_logger(),
        "⏭️ [PATH REQUEST] Ignoring — official itinerary requires dispatch_task_id to match "
        "task_api booking (dispatch_task_id='%s', booking=%s): %s",
        path_dispatch_task_id.c_str(), coords_booking_id.c_str(), dispatch_reason.c_str());
      return;
    }

    RCLCPP_INFO(
      get_logger(),
      "✅ [PATH REQUEST] Path segment accepted: dispatch_task_id matches booking '%s', "
      "first waypoint matches robot pose and last matches expected task target (%s)",
      coords_booking_id.c_str(), patrol_task ? "patrol" : "delivery");
    RCLCPP_INFO(
      get_logger(), "📋 [PATH REQUEST] Total waypoints in message: %zu", msg->path.size());

    std::string task_id_to_use;
    if (!task_id.empty() && active_task_robots.count(task_id) > 0) {
      task_id_to_use = task_id;
      RCLCPP_INFO(
        get_logger(), "✅ [PATH REQUEST] Using task_id from active_task_robots: %s",
        task_id.c_str());
    } else if (!trim_copy(robot.current_task).empty()) {
      if (active_task_robots.count(robot.current_task) > 0) {
        task_id_to_use = robot.current_task;
        RCLCPP_INFO(
          get_logger(), "✅ [PATH REQUEST] Using robot.current_task: %s",
          robot.current_task.c_str());
      } else {
        const auto last_it = last_known_tasks.find(robot_name);
        const std::string last_task = (last_it != last_known_tasks.end()) ? last_it->second : "";
        if (!last_task.empty() && active_task_robots.count(last_task) > 0) {
          task_id_to_use = last_task;
          RCLCPP_INFO(
            get_logger(),
            "🔄 [PATH REQUEST] Detected continuation of task %s (next segment) for robot %s",
            last_task.c_str(), robot_name.c_str());
          robot.current_task = last_task;
          active_task_robots[last_task] = robot_name;
        } else {
          task_id_to_use = robot.current_task;
          active_task_robots[task_id_to_use] = robot_name;
          RCLCPP_INFO(
            get_logger(),
            "🔄 [PATH REQUEST] Using robot.current_task and updating mapping: %s",
            task_id_to_use.c_str());
        }
      }
    } else if (!task_id.empty() && active_task_robots.count(task_id) > 0) {
      task_id_to_use = task_id;
      robot.current_task = task_id;
      robot.status = RobotStatus::Navigating;
      RCLCPP_INFO(
        get_logger(),
        "🔄 [PATH REQUEST] Restored task_id %s for robot %s (next segment)",
        task_id.c_str(), robot_name.c_str());
    } else {
      std::ostringstream mapping;
      mapping << "{";
      bool first = true;
      for (const auto & [tid, rname] : active_task_robots) {
        if (!first) {
          mapping << ", ";
        }
        first = false;
        mapping << "'" << tid << "': '" << rname << "'";
      }
      mapping << "}";
      RCLCPP_WARN(
        get_logger(),
        "⚠️ [PATH REQUEST] Robot %s has no matching task. task_id=%s, robot.current_task=%s",
        robot_name.c_str(), task_id.c_str(), robot.current_task.c_str());
      RCLCPP_WARN(get_logger(), "⚠️ [PATH REQUEST] active_task_robots: %s", mapping.str().c_str());
      std::ostringstream last_tasks;
      last_tasks << "{";
      bool first_lt = true;
      for (const auto & [rname, tid] : last_known_tasks) {
        if (!first_lt) {
          last_tasks << ", ";
        }
        first_lt = false;
        last_tasks << "'" << rname << "': '" << tid << "'";
      }
      last_tasks << "}";
      RCLCPP_WARN(
        get_logger(), "⚠️ [PATH REQUEST] last_known_tasks: %s", last_tasks.str().c_str());
      return;
    }

    task_id = task_id_to_use;

    std::string phase_key;
    if (task_pickup_drop_coords.count(coords_booking_id) > 0) {
      phase_key = coords_booking_id;
    } else if (task_patrol_coords.count(coords_booking_id) > 0) {
      phase_key = coords_booking_id;
    } else if (is_patrol_task(path_dispatch_task_id)) {
      phase_key = path_dispatch_task_id;
    } else {
      phase_key = task_id;
    }

    if (task_pickup_drop_coords.count(phase_key) > 0 ||
      task_patrol_coords.count(phase_key) > 0)
    {
      currently_underway_task[robot_name] = phase_key;
      RCLCPP_INFO(
        get_logger(),
        "📋 [PATH REQUEST] Updated currently_underway_task to %s (path request = RMF started task)",
        phase_key.c_str());
    } else if (is_patrol_task(path_dispatch_task_id)) {
      currently_underway_task[robot_name] = path_dispatch_task_id;
      RCLCPP_INFO(
        get_logger(),
        "📋 [PATH REQUEST] Updated currently_underway_task to %s "
        "(patrol path request = RMF started task)",
        path_dispatch_task_id.c_str());
    }

    const auto underway_it = currently_underway_task.find(robot_name);
    const std::string underway_key = (underway_it != currently_underway_task.end()) ? underway_it->second : "";
    if (task_pickup_drop_coords.count(phase_key) > 0 &&
      !underway_key.empty() &&
      coords_booking_id != underway_key)
    {
      RCLCPP_INFO(
        get_logger(),
        "⏭️ [PATH REQUEST] Skipping - only send itinerary for task currently underway. "
        "Underway=%s, this plan booking=%s",
        underway_key.c_str(), coords_booking_id.c_str());
      return;
    }

    std::string phase;
    if (patrol_task_key.has_value()) {
      phase = "patrol";
    } else if (dropoff_in_progress.count(phase_key) > 0 && dropoff_in_progress.at(phase_key)) {
      phase = "drop";
    } else {
      phase = "pick";
    }

    // ---- Replan of the leg we already published? -----------------------
    // RMF re-issues a PathRequest for the SAME dispatch id / leg with a higher
    // path_revision when it replans. That must reach the robot as a VDA5050
    // order UPDATE, so it has to bypass the one-order-per-phase gate below.
    const std::string replan_dispatch_id =
      !path_dispatch_task_id.empty() ? path_dispatch_task_id : phase_key;
    const bool is_replan = is_replan_of_published_order(
      robot_name,
      replan_dispatch_id,
      static_cast<int>(msg->phase_index),
      static_cast<int>(msg->path_revision),
      last_wp_x, last_wp_y);

    if (is_replan) {
      const auto & prev = published_order_by_robot.at(robot_name);
      std::vector<std::pair<double, double>> incoming_xy;
      incoming_xy.reserve(msg->path.size());
      for (const auto & wp : msg->path) {
        incoming_xy.emplace_back(wp.x, wp.y);
      }
      const auto published_fp = last_published_vda_order.find(robot_name);
      const bool same_route =
        published_fp != last_published_vda_order.end() &&
        published_fp->second.order_id == replan_dispatch_id &&
        published_fp->second.phase == phase &&
        published_fp->second.path_hash == route_xy_hash(incoming_xy);
      // The robot is already driving this exact route. Cancelling it and then
      // dropping the replacement as a duplicate leaves it idle with an empty
      // path (seen on amr1, order ab391395, every 10 s). Leave the current
      // order in place.
      //
      // A traffic-horizon extension is NOT such a repeat: it is the same route
      // by design, with more waypoints released. Dropping it here left the
      // robot stopped at its release boundary forever (orderUpdateId never
      // advanced, so traffic_manager waited for an ACK that could not come).
      // Extensions continue below and are queued as an order update without
      // cancelOrder; flush_pending_vda_orders() publishes them while driving.
      if (same_route && !traffic_extension && !robot_ready_for_order(robot_name)) {
        RCLCPP_INFO(
          get_logger(),
          "🔁 [REPLAN ORDER] robot=%s orderId=%s phase=%s leg=%d — revision %d -> %d "
          "returned the same %zu-node route and the robot is still driving it. "
          "Not sending cancelOrder.",
          robot_name.c_str(), replan_dispatch_id.c_str(), phase.c_str(),
          prev.phase_index, prev.path_revision, static_cast<int>(msg->path_revision),
          msg->path.size());
        return;
      }

      RCLCPP_INFO(
        get_logger(),
        "🔁 [REPLAN ORDER] robot=%s orderId=%s phase=%s leg=%d — revision %d -> %d, "
        "waypoints=%zu, destination unchanged; republishing as an order update",
        robot_name.c_str(), replan_dispatch_id.c_str(), phase.c_str(),
        prev.phase_index, prev.path_revision, static_cast<int>(msg->path_revision),
        msg->path.size());

      // ---- Abort the running nav goal, but ONLY for a real reroute --------
      // The cancel has to happen HERE, before the order is queued. It is what
      // makes the robot stop, and flush_pending_vda_orders() will not publish
      // into a robot that is still driving (robot_ready_for_order). Moving the
      // cancel to the publish point deadlocks the two against each other:
      //
      //   order waits for idle -> robot waits for cancel -> cancel waits for
      //   the order to be published
      //
      // Measured 2026-09-22 with the cancel at the publish point: zero
      // cancelOrders sent, amr2's replanned order held 34.5 s at "robot still
      // driving" while the robot sat motionless reporting status='moving', then
      // dropped undelivered when the task ended. The idle gate's own warning
      // names the shape exactly — "a cancelOrder that never reached the robot
      // looks exactly like this".
      //
      // What the cancel must NOT do is fire for a replan that returns the SAME
      // route: the order is then dropped by the route-unchanged fingerprint
      // gate and the robot is left with its goal cancelled and no replacement.
      // That is the 12-cancels-in-176 s loop of 2026-09-22 (log2), where RMF's
      // stuck detector bumped the revision every 10 s and the planner returned
      // the identical path every time.
      //
      // So the route is compared first, against the last path accepted for this
      // robot. Same route => no cancel; the robot keeps driving what it has, and
      // the duplicate order is suppressed downstream as before.
      const std::size_t incoming_path_hash = hash_path_request(*msg);
      const auto last_hash_it = last_accepted_path_hash_by_robot.find(robot_name);
      const bool route_unchanged =
        last_hash_it != last_accepted_path_hash_by_robot.end() &&
        last_hash_it->second == incoming_path_hash;

      if (traffic_extension) {
        // Base extension: never cancel the goal the robot is driving.
        RCLCPP_INFO(
          get_logger(),
          "🛤️ [REPLAN ORDER] robot=%s: traffic horizon extension (%zu "
          "waypoints) — order update without cancelOrder",
          robot_name.c_str(), msg->path.size());
      } else if (route_unchanged) {
        RCLCPP_INFO(
          get_logger(),
          "🔁 [REPLAN ORDER] robot=%s: replan returned the SAME route (%zu "
          "waypoints) — no cancelOrder sent; the robot keeps the goal it is "
          "already driving",
          robot_name.c_str(), msg->path.size());
      } else if (!send_replan_nav_cancel(robot_name, replan_dispatch_id)) {
        return;
      }
    }

    // Remember the route we just accepted, replan or not, so the next replan can
    // tell a genuine reroute from a repeat. Seeded on the FIRST path too: in the
    // log2 loop the very first replan already returned the original route, so a
    // comparison that only started at the first replan would still have fired
    // one needless cancel and started the loop.
    last_accepted_path_hash_by_robot[robot_name] = hash_path_request(*msg);

    auto & sent_phases = sent_path_request_phase[phase_key];
    if (!is_replan && phase != "patrol" && sent_phases.count(phase) > 0) {
      RCLCPP_INFO(
        get_logger(),
        "⏭️ [PATH REQUEST] Skipping - already sent itinerary for task %s phase '%s' (one per phase only)",
        task_id.c_str(), phase.c_str());
      return;
    }

    RCLCPP_INFO(
      get_logger(),
      "✅ [PATH REQUEST] Matched to robot: %s, task_id: %s, phase: %s",
      robot_name.c_str(), task_id.c_str(), phase.c_str());

    const AnyMap path_request_data = build_path_request_payload(*msg);
    bool progressed_patrol = false;
    bool vda_order_queued = false;

    std::string order_id;
    const auto order_underway_it = currently_underway_task.find(robot_name);
    if (order_underway_it != currently_underway_task.end()) {
      order_id = order_underway_it->second;
    }
    if (order_id.empty()) {
      RCLCPP_WARN(
        get_logger(),
        "⚠️ [VDA5050 ORDER] No underway task for robot %s; buffering path for phase=%s until task is underway",
        robot_name.c_str(), phase.c_str());
      append_pending_path_request(
        robot_name, path_request_data, task_id, coords_booking_id,
        first_wp_x, first_wp_y, last_wp_x, last_wp_y);
      RCLCPP_INFO(
        get_logger(),
        "📥 [PATH REQUEST] Buffered validated path for robot %s (task_id=%s, phase=%s) — "
        "VDA order will publish on underway",
        robot_name.c_str(), task_id.c_str(), phase.c_str());
      return;
    }

    const auto order_payload_opt =
      build_vda5050_order_payload(robot_name, order_id, phase, path_request_data);
    if (order_payload_opt.has_value() && mqtt_client) {
      const auto [can_publish, block_reason] = can_publish_vda_order_for_phase(robot_name, phase);
      if (!can_publish) {
        RCLCPP_INFO(
          get_logger(),
          "⏳ [VDA5050 ORDER] Holding publish for robot=%s, orderId=%s, phase=%s: %s",
          robot_name.c_str(), order_id.c_str(), phase.c_str(), block_reason.c_str());
        append_pending_path_request(
          robot_name, path_request_data, task_id, coords_booking_id,
          first_wp_x, first_wp_y, last_wp_x, last_wp_y);
        return;
      }
      queue_vda_order_after_monitored(
        order_id, robot_name, phase, *order_payload_opt, "direct",
        static_cast<int>(anymap_get_double(path_request_data, "phase_index")),
        static_cast<int>(anymap_get_double(path_request_data, "path_revision")));
      progressed_patrol = (phase == "patrol");
      vda_order_queued = true;
      RCLCPP_INFO(
        get_logger(),
        "📥 [VDA5050 ORDER] Queued for monitored-first publish (orderId=%s, phase=%s, source=direct)",
        order_id.c_str(), phase.c_str());
      if (monitored_task_manager->has_started(order_id)) {
        publish_monitored_task(order_id);
        flush_pending_vda_orders(order_id);
      } else {
        update_monitored_task_cache(
          order_id,
          {
            {"task_id", YAML::Node(order_id)},
            {"amr_id", YAML::Node(robot_name)},
            {"category", YAML::Node(phase)},
          });
        handle_monitored_publish_rule(order_id, "underway");
      }
    } else if (phase == "pick" || phase == "drop" || phase == "patrol") {
      RCLCPP_WARN(
        get_logger(),
        "⚠️ [VDA5050 ORDER] Could not build order payload for robot=%s, orderId=%s, phase=%s",
        robot_name.c_str(), order_id.c_str(), phase.c_str());
    }

    if (phase == "patrol") {
      // Single-place patrol: nothing to advance. Kept as an explicit branch so
      // a patrol never falls into the one-order-per-phase marker below.
      (void)progressed_patrol;
      (void)patrol_matched_end_idx;
    } else if (!is_replan) {
      // Same reasoning for pick/drop: a replan re-sends the current phase, so
      // the one-order-per-phase marker must not be re-armed against it.
      sent_phases.insert(phase);
    }
    if (vda_order_queued) {
      RCLCPP_INFO(
        get_logger(),
        "✅ [VDA5050 ORDER] Order queued for robot %s (phase: %s, task_id: %s)",
        robot_name.c_str(), phase.c_str(), task_id.c_str());
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "❌ [PATH REQUEST] Error in send/buffer: %s", e.what());
  }
}

// ============================================================================
// Section 11 — Task API + dispatch (Python lines 2781–3059, 3546–3746)
// ============================================================================

void RMFIntegratedFleetAdapter::process_patrol_api_payload(
  const std::string & request_id, const YAML::Node & payload)
{
  remember_task_category(request_id, "patrol");
  // Python lines 2885–2951
  // robot_task_request responses are often thin (no places_coordinates). Python
  // uses detail.get(..., []). For robot-specific tasks we also reuse places from
  // the original /task_api_requests payload saved in pending_task_request_*.
  // IMPORTANT: never call IsSequence()/size() on a missing const Node child —
  // yaml-cpp returns InvalidNode and throws InvalidNode.
  try {
    const YAML::Node state = yaml_safe_map(payload, "state");
    const YAML::Node detail = yaml_safe_map(state, "detail");
    const YAML::Node booking = yaml_safe_map(state, "booking");
    const YAML::Node description = yaml_safe_map(booking, "description");

    YAML::Node pending_request;
    const auto pending_req_it = pending_task_request_by_api_request_id.find(request_id);
    if (pending_req_it != pending_task_request_by_api_request_id.end()) {
      pending_request = pending_req_it->second;
      pending_task_request_by_api_request_id.erase(pending_req_it);
    }
    const YAML::Node pending_description = yaml_safe_map(pending_request, "description");

    auto resolve_places = [](const YAML::Node & desc_or_detail) -> YAML::Node {
      YAML::Node places = yaml_safe_sequence(desc_or_detail, "places");
      if (places.size() > 0) {
        return places;
      }
      // Compose/nested: description.description.places
      return yaml_safe_sequence(yaml_safe_map(desc_or_detail, "description"), "places");
    };

    YAML::Node places = resolve_places(detail);
    if (places.size() == 0) {
      places = resolve_places(description);
    }
    if (places.size() == 0) {
      places = resolve_places(pending_description);
    }

    YAML::Node places_coordinates = yaml_safe_sequence(detail, "places_coordinates");
    if (places_coordinates.size() == 0) {
      places_coordinates = yaml_safe_sequence(description, "places_coordinates");
    }
    if (places_coordinates.size() == 0) {
      places_coordinates = yaml_safe_sequence(pending_description, "places_coordinates");
    }

    if (places_coordinates.size() == 0 && places.size() > 0 && nav_graph_parser) {
      YAML::Node built_coords(YAML::NodeType::Sequence);
      for (const auto & place_name_node : places) {
        const std::string place_name = yaml_node_as_string(place_name_node);
        if (place_name.empty()) {
          continue;
        }
        const auto wp = nav_graph_parser->get_waypoint_by_name(place_name);
        if (!wp.has_value()) {
          continue;
        }
        YAML::Node entry(YAML::NodeType::Map);
        entry["place"] = place_name;
        entry["coordinates"]["x"] = wp->x;
        entry["coordinates"]["y"] = wp->y;
        built_coords.push_back(entry);
      }
      if (built_coords.size() > 0) {
        places_coordinates = built_coords;
        RCLCPP_INFO(
          get_logger(),
          "📍 [PATROL COORDS] Built places_coordinates from nav graph for %s (%zu place(s))",
          request_id.c_str(), built_coords.size());
      }
    }

    if (places_coordinates.size() == 0) {
      RCLCPP_WARN(
        get_logger(),
        "⚠️ [PATROL COORDS] No places/places_coordinates for task %s "
        "(response detail empty; robot_task_request places also missing)",
        request_id.c_str());
      return;
    }

    std::map<std::string, std::pair<double, double>> place_to_coords;
    for (const auto & pc : places_coordinates) {
      if (!pc.IsMap()) {
        continue;
      }
      const std::string place_name = trim_copy(yaml_map_get_string(pc, "place"));
      const YAML::Node coords = pc["coordinates"];
      if (place_name.empty()) {
        continue;
      }
      place_to_coords[place_name] = {
        yaml_node_double(coords["x"]), yaml_node_double(coords["y"])};
    }

    YAML::Node route_coordinates(YAML::NodeType::Sequence);
    if (places.size() > 0) {
      for (const auto & place_name_node : places) {
        const std::string place_name = yaml_node_as_string(place_name_node);
        const auto coords_it = place_to_coords.find(place_name);
        if (coords_it == place_to_coords.end()) {
          continue;
        }
        YAML::Node entry(YAML::NodeType::Map);
        entry["place"] = place_name;
        entry["coordinates"]["x"] = coords_it->second.first;
        entry["coordinates"]["y"] = coords_it->second.second;
        route_coordinates.push_back(entry);
      }
    }
    if (route_coordinates.size() == 0) {
      route_coordinates = YAML::Clone(places_coordinates);
    }

    YAML::Node places_owned = places.size() > 0 ?
      YAML::Clone(places) : YAML::Node(YAML::NodeType::Sequence);
    YAML::Node places_coordinates_owned = YAML::Clone(places_coordinates);

    task_patrol_coords[request_id] = {
      {"places", places_owned},
      {"places_coordinates", places_coordinates_owned},
      {"route_coordinates", route_coordinates},
    };
    if (places_owned.size() > 0) {
      auto & phase_places = task_phase_places[request_id];
      phase_places["patrol"] = yaml_node_as_string(places_owned[places_owned.size() - 1]);
    }

    std::ostringstream places_log;
    places_log << "[";
    bool first = true;
    for (const auto & place_node : places_owned) {
      if (!first) {
        places_log << ", ";
      }
      first = false;
      places_log << yaml_node_as_string(place_node);
    }
    places_log << "]";
    RCLCPP_INFO(
      get_logger(),
      "📍 [PATROL COORDS] Stored coords for %s: %zu place(s), places=%s, route_coordinates=%zu",
      request_id.c_str(), places_coordinates_owned.size(), places_log.str().c_str(),
      route_coordinates.size());

    std::string assigned_robot;
    const auto dispatch_it = dispatch_assignments.find(request_id);
    if (dispatch_it != dispatch_assignments.end()) {
      assigned_robot = dispatch_it->second;
    }
    if (assigned_robot.empty()) {
      const auto known_robot_it = known_task_robot.find(request_id);
      if (known_robot_it != known_task_robot.end()) {
        assigned_robot = known_robot_it->second;
      }
    }
    if (assigned_robot.empty()) {
      const YAML::Node dispatch_state = yaml_safe_map(state, "dispatch");
      const YAML::Node assignment = yaml_safe_map(dispatch_state, "assignment");
      assigned_robot = trim_copy(yaml_map_get_string(assignment, "expected_robot_name"));
    }
    if (assigned_robot.empty()) {
      assigned_robot = trim_copy(
        yaml_map_get_string(yaml_safe_map(state, "assigned_to"), "name"));
    }
    if (!assigned_robot.empty()) {
      dispatch_assignments[request_id] = assigned_robot;
      RCLCPP_INFO(
        get_logger(),
        "📍 [PATROL COORDS] Resolved assigned robot %s for task %s — flushing buffered path",
        assigned_robot.c_str(), request_id.c_str());
      try_send_buffered_path_request(assigned_robot);
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      get_logger(), "❌ [PATROL COORDS] Error processing patrol API payload: %s", e.what());
  }
}

void RMFIntegratedFleetAdapter::process_drop_api_payload(
  const std::string & request_id, const YAML::Node & payload)
{
  remember_task_category(request_id, "drop");
  // Python lines 2878–3059
  try {
    const YAML::Node state = payload["state"];
    const YAML::Node detail = state["detail"];

    std::string place_name;
    YAML::Node place_coords(YAML::NodeType::Map);

    const YAML::Node dropoff_detail = detail["dropoff"];
    if (dropoff_detail.IsMap()) {
      place_name = yaml_map_get_string(dropoff_detail, "place");
      const YAML::Node coords = dropoff_detail["coordinates"];
      if (yaml_coords_has_xy(coords)) {
        place_coords["x"] = yaml_node_double(coords["x"]);
        place_coords["y"] = yaml_node_double(coords["y"]);
      }
    }

    if (place_name.empty() || !yaml_coords_has_xy(place_coords)) {
      const YAML::Node place_field = detail["place"];
      if (place_field.IsScalar()) {
        if (place_name.empty()) {
          place_name = yaml_node_as_string(place_field);
        }
      } else if (place_field.IsMap()) {
        if (place_name.empty()) {
          place_name = yaml_map_get_string(place_field, "waypoint");
          if (place_name.empty()) {
            place_name = yaml_map_get_string(place_field, "name");
          }
        }
        const YAML::Node coords = place_field["coordinates"];
        if (!yaml_coords_has_xy(place_coords) && yaml_coords_has_xy(coords)) {
          place_coords["x"] = yaml_node_double(coords["x"]);
          place_coords["y"] = yaml_node_double(coords["y"]);
        }
      }
    }

    if (place_name.empty()) {
      const YAML::Node booking = state["booking"];
      const YAML::Node description = booking["description"];
      if (description.IsMap()) {
        const YAML::Node inner_place = description["place"];
        if (inner_place.IsScalar()) {
          place_name = yaml_node_as_string(inner_place);
        } else if (inner_place.IsMap()) {
          place_name = yaml_map_get_string(inner_place, "waypoint");
          if (place_name.empty()) {
            place_name = yaml_map_get_string(inner_place, "name");
          }
        }
      }
    }

    if (!place_name.empty() && !yaml_coords_has_xy(place_coords) && nav_graph_parser) {
      const auto wp = nav_graph_parser->get_waypoint_by_name(place_name);
      if (wp.has_value()) {
        place_coords["x"] = wp->x;
        place_coords["y"] = wp->y;
      }
    }

    if (place_name.empty()) {
      RCLCPP_WARN(
        get_logger(),
        "⚠️ [DROP COORDS] Could not extract place name from task_api_response for %s",
        request_id.c_str());
      return;
    }
    if (!yaml_coords_has_xy(place_coords)) {
      RCLCPP_WARN(
        get_logger(), "⚠️ [DROP COORDS] No coordinates for place '%s' (task %s)",
        place_name.c_str(), request_id.c_str());
      return;
    }

    std::string assigned_robot;
    const YAML::Node dispatch_state = state["dispatch"];
    const YAML::Node assignment = dispatch_state["assignment"];
    assigned_robot = trim_copy(yaml_map_get_string(assignment, "expected_robot_name"));
    if (assigned_robot.empty()) {
      assigned_robot = trim_copy(yaml_map_get_string(state["assigned_to"], "name"));
    }
    if (assigned_robot.empty()) {
      const auto assign_it = dispatch_assignments.find(request_id);
      if (assign_it != dispatch_assignments.end()) {
        assigned_robot = assign_it->second;
      }
    }
    if (assigned_robot.empty()) {
      const auto known_robot_it = known_task_robot.find(request_id);
      if (known_robot_it != known_task_robot.end()) {
        assigned_robot = known_robot_it->second;
      }
    }

    YAML::Node pickup_coords(YAML::NodeType::Map);
    pickup_coords["x"] = 0.0;
    pickup_coords["y"] = 0.0;
    task_pickup_drop_coords[request_id] = {
      {"pickup", pickup_coords},
      {"dropoff", place_coords},
    };
    dropoff_in_progress[request_id] = true;

    std::string handler = "ingestor1";
    const YAML::Node booking = state["booking"];
    const YAML::Node description = booking["description"];
    if (description.IsMap() && description["handler"].IsScalar()) {
      handler = trim_copy(yaml_node_as_string(description["handler"]));
    } else if (dropoff_detail.IsMap() && dropoff_detail["handler"].IsScalar()) {
      handler = trim_copy(yaml_node_as_string(dropoff_detail["handler"]));
    }
    if (handler.empty()) {
      handler = "ingestor1";
    }

    AnyMap & existing_detail = delivery_task_details[request_id];
    const std::string existing_robot_id = anymap_get_string(existing_detail, "robot_id");
    YAML::Node dropoff_entry(YAML::NodeType::Map);
    dropoff_entry["place"] = place_name;
    dropoff_entry["coordinates"] = place_coords;
    dropoff_entry["handler"] = handler;
    existing_detail["pickup"] = YAML::Node(YAML::NodeType::Map);
    existing_detail["dropoff"] = dropoff_entry;
    existing_detail["pickup_handler"] = YAML::Node("");
    existing_detail["dropoff_handler"] = YAML::Node(handler);
    existing_detail["robot_id"] = YAML::Node(existing_robot_id);
    existing_detail["request_id"] = YAML::Node(request_id);
    existing_detail["pickup_complete"] = YAML::Node(true);
    existing_detail["dropoff_complete"] = YAML::Node(anymap_get_bool(existing_detail, "dropoff_complete", false));
    existing_detail["pickup_sent"] = YAML::Node(true);

    auto & phase_places = task_phase_places[request_id];
    phase_places["drop"] = place_name;

    if (!assigned_robot.empty()) {
      robot_to_coords_key[assigned_robot] = request_id;
      delivery_task_details[request_id]["robot_id"] = YAML::Node(assigned_robot);
      RCLCPP_INFO(
        get_logger(),
        "📍 [DROP COORDS] Stored coords for %s: place=%s (%.6g, %.6g), robot=%s, handler=%s",
        request_id.c_str(), place_name.c_str(),
        yaml_node_double(place_coords["x"]), yaml_node_double(place_coords["y"]),
        assigned_robot.c_str(), handler.c_str());
      try_send_buffered_path_request(assigned_robot);
    } else {
      RCLCPP_INFO(
        get_logger(),
        "📍 [DROP COORDS] Stored coords for %s: place=%s (%.6g, %.6g), handler=%s; "
        "waiting for robot assignment.",
        request_id.c_str(), place_name.c_str(),
        yaml_node_double(place_coords["x"]), yaml_node_double(place_coords["y"]),
        handler.c_str());
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      get_logger(), "❌ [DROP COORDS] Error processing drop API payload: %s", e.what());
  }
}

void RMFIntegratedFleetAdapter::process_delivery_api_payload(
  const std::string & request_id,
  const YAML::Node & payload,
  const std::string & assigned_robot_override)
{
  remember_task_category(request_id, "delivery");
  // Python lines 3647–3746
  // robot_task_request delivery responses are often thin (no pickup/dropoff detail).
  // Reuse pickup/dropoff from the cached /task_api_requests payload, same as patrol places.
  try {
    const YAML::Node state = yaml_safe_map(payload, "state");
    const YAML::Node detail = yaml_safe_map(state, "detail");
    const YAML::Node booking = yaml_safe_map(state, "booking");
    const YAML::Node booking_description = yaml_safe_map(booking, "description");
    const std::string booking_id = yaml_map_get_string(booking, "id", request_id);
    const std::string lookup_key = !booking_id.empty() ? booking_id : request_id;

    YAML::Node pending_request;
    const auto pending_req_it = pending_task_request_by_api_request_id.find(lookup_key);
    if (pending_req_it == pending_task_request_by_api_request_id.end()) {
      const auto pending_req_alt = pending_task_request_by_api_request_id.find(request_id);
      if (pending_req_alt != pending_task_request_by_api_request_id.end()) {
        pending_request = pending_req_alt->second;
        pending_task_request_by_api_request_id.erase(pending_req_alt);
      }
    } else {
      pending_request = pending_req_it->second;
      pending_task_request_by_api_request_id.erase(pending_req_it);
    }
    const YAML::Node pending_description = yaml_safe_map(pending_request, "description");

    YAML::Node pickup_detail = resolve_delivery_leg(detail, booking_description, pending_description, "pickup");
    YAML::Node dropoff_detail = resolve_delivery_leg(detail, booking_description, pending_description, "dropoff");

    const YAML::Node dispatch_state = yaml_safe_map(state, "dispatch");
    const YAML::Node assignment = yaml_safe_map(dispatch_state, "assignment");

    RCLCPP_INFO(
      get_logger(),
      "🔍 [TASK API RESPONSE] Checking robot assignment for %s (lookup_key: %s)",
      request_id.c_str(), lookup_key.c_str());
    RCLCPP_INFO(
      get_logger(), "🔍 [TASK API RESPONSE] dispatch.status: %s",
      yaml_map_has_key(dispatch_state, "status") ?
      yaml_node_as_string(dispatch_state["status"]).c_str() : "N/A");
    RCLCPP_INFO(
      get_logger(), "🔍 [TASK API RESPONSE] assignment.expected_robot_name: %s",
      yaml_map_get_string(assignment, "expected_robot_name", "N/A").c_str());
    RCLCPP_INFO(
      get_logger(), "🔍 [TASK API RESPONSE] state.assigned_to.name: %s",
      yaml_map_get_string(yaml_safe_map(state, "assigned_to"), "name", "N/A").c_str());

    std::ostringstream keys_log;
    keys_log << "[";
    bool first_key = true;
    for (const auto & [key, _] : dispatch_assignments) {
      if (!first_key) {
        keys_log << ", ";
      }
      first_key = false;
      keys_log << "'" << key << "'";
    }
    keys_log << "]";
    RCLCPP_INFO(
      get_logger(), "🔍 [TASK API RESPONSE] Available dispatch_assignments keys: %s",
      keys_log.str().c_str());
    const auto lookup_it = dispatch_assignments.find(lookup_key);
    RCLCPP_INFO(
      get_logger(), "🔍 [TASK API RESPONSE] Looking up '%s' in dispatch_assignments: %s",
      lookup_key.c_str(),
      lookup_it != dispatch_assignments.end() ? lookup_it->second.c_str() : "NOT FOUND");

    std::string assigned_robot = assigned_robot_override;
    if (assigned_robot.empty()) {
      assigned_robot = trim_copy(yaml_map_get_string(assignment, "expected_robot_name"));
    }
    if (assigned_robot.empty()) {
      assigned_robot = trim_copy(yaml_map_get_string(yaml_safe_map(state, "assigned_to"), "name"));
    }
    if (assigned_robot.empty()) {
      if (lookup_it != dispatch_assignments.end()) {
        assigned_robot = lookup_it->second;
        RCLCPP_INFO(
          get_logger(),
          "✅ [TASK API RESPONSE] Matched %s to robot %s via /dispatch_states",
          lookup_key.c_str(), assigned_robot.c_str());
      }
    }
    if (assigned_robot.empty()) {
      const auto known_robot_it = known_task_robot.find(lookup_key);
      if (known_robot_it != known_task_robot.end()) {
        assigned_robot = known_robot_it->second;
        RCLCPP_INFO(
          get_logger(),
          "✅ [TASK API RESPONSE] Matched %s to robot %s via known_task_robot",
          lookup_key.c_str(), assigned_robot.c_str());
      }
    }

    if (assigned_robot.empty()) {
      RCLCPP_WARN(
        get_logger(),
        "⚠️ [TASK API RESPONSE] No robot assignment yet for %s (dispatch.status=%s)",
        request_id.c_str(),
        yaml_map_has_key(dispatch_state, "status") ?
        yaml_node_as_string(dispatch_state["status"]).c_str() : "unknown");
      RCLCPP_WARN(
        get_logger(), "⚠️ [TASK API RESPONSE] Storing payload until RMF assigns a robot");
      pending_delivery_api_responses[lookup_key] = payload;
      return;
    }

    RCLCPP_INFO(
      get_logger(), "✅ [TASK API RESPONSE] Robot assigned: %s for %s",
      assigned_robot.c_str(), request_id.c_str());
    pending_delivery_api_responses.erase(lookup_key);

    const std::string base_request_id = request_id;
    const std::string pickup_handler = trim_copy(
      yaml_map_get_string(
        pickup_detail,
        "handler",
        yaml_map_get_string(
          pickup_detail,
          "place",
          "dispenser1")));
    const std::string dropoff_handler = trim_copy(
      yaml_map_get_string(
        dropoff_detail,
        "handler",
        yaml_map_get_string(
          dropoff_detail,
          "place",
          "ingestor1")));

    AnyMap & existing_detail = delivery_task_details[base_request_id];
    existing_detail["pickup"] = pickup_detail;
    existing_detail["dropoff"] = dropoff_detail;
    existing_detail["pickup_handler"] = YAML::Node(pickup_handler);
    existing_detail["dropoff_handler"] = YAML::Node(dropoff_handler);
    existing_detail["robot_id"] = YAML::Node(assigned_robot);
    existing_detail["request_id"] = YAML::Node(request_id);
    existing_detail["pickup_complete"] = YAML::Node(anymap_get_bool(existing_detail, "pickup_complete", false));
    existing_detail["dropoff_complete"] = YAML::Node(anymap_get_bool(existing_detail, "dropoff_complete", false));
    existing_detail["pickup_sent"] = YAML::Node(anymap_get_bool(existing_detail, "pickup_sent", false));

    task_patrol_coords.erase(base_request_id);

    const std::string pickup_place_label = trim_copy(yaml_map_get_string(pickup_detail, "place"));
    const std::string dropoff_place_label = trim_copy(yaml_map_get_string(dropoff_detail, "place"));
    if (!pickup_place_label.empty() || !dropoff_place_label.empty()) {
      auto & phase_places = task_phase_places[base_request_id];
      if (!pickup_place_label.empty()) {
        phase_places["pick"] = pickup_place_label;
      }
      if (!dropoff_place_label.empty()) {
        phase_places["drop"] = dropoff_place_label;
      }
    }

    YAML::Node pickup_coords = yaml_safe_map(pickup_detail, "coordinates");
    YAML::Node dropoff_coords = yaml_safe_map(dropoff_detail, "coordinates");

    if (nav_graph_parser) {
      std::string pickup_place = yaml_map_get_string(pickup_detail, "place");
      if (pickup_place.empty()) {
        pickup_place = yaml_map_get_string(pickup_detail, "waypoint");
      }
      if (!pickup_place.empty() && !yaml_coords_has_xy(pickup_coords)) {
        const auto wp = nav_graph_parser->get_waypoint_by_name(pickup_place);
        if (wp.has_value()) {
          pickup_coords = YAML::Node(YAML::NodeType::Map);
          pickup_coords["x"] = wp->x;
          pickup_coords["y"] = wp->y;
          RCLCPP_INFO(
            get_logger(),
            "📍 [DELIVERY COORDS] Built pickup coordinates from nav graph for %s at %s",
            base_request_id.c_str(), pickup_place.c_str());
        }
      }

      std::string dropoff_place = yaml_map_get_string(dropoff_detail, "place");
      if (dropoff_place.empty()) {
        dropoff_place = yaml_map_get_string(dropoff_detail, "waypoint");
      }
      if (!dropoff_place.empty() && !yaml_coords_has_xy(dropoff_coords)) {
        const auto wp = nav_graph_parser->get_waypoint_by_name(dropoff_place);
        if (wp.has_value()) {
          dropoff_coords = YAML::Node(YAML::NodeType::Map);
          dropoff_coords["x"] = wp->x;
          dropoff_coords["y"] = wp->y;
          RCLCPP_INFO(
            get_logger(),
            "📍 [DELIVERY COORDS] Built dropoff coordinates from nav graph for %s at %s",
            base_request_id.c_str(), dropoff_place.c_str());
        }
      }
    }

    if (yaml_coords_has_xy(pickup_coords) || yaml_coords_has_xy(dropoff_coords)) {
      YAML::Node pickup_xy(YAML::NodeType::Map);
      pickup_xy["x"] = yaml_node_double(pickup_coords["x"]);
      pickup_xy["y"] = yaml_node_double(pickup_coords["y"]);
      YAML::Node dropoff_xy(YAML::NodeType::Map);
      dropoff_xy["x"] = yaml_node_double(dropoff_coords["x"]);
      dropoff_xy["y"] = yaml_node_double(dropoff_coords["y"]);
      task_pickup_drop_coords[base_request_id] = {
        {"pickup", pickup_xy},
        {"dropoff", dropoff_xy},
      };
      robot_to_coords_key[assigned_robot] = base_request_id;
      RCLCPP_INFO(
        get_logger(),
        "📍 [ITINERARY COORDS] Stored coords for %s: pickup=(%s, %s), dropoff=(%s, %s), "
        "robot_to_coords_key[%s]=%s",
        base_request_id.c_str(),
        yaml_node_as_string(pickup_coords["x"]).c_str(),
        yaml_node_as_string(pickup_coords["y"]).c_str(),
        yaml_node_as_string(dropoff_coords["x"]).c_str(),
        yaml_node_as_string(dropoff_coords["y"]).c_str(),
        assigned_robot.c_str(), base_request_id.c_str());
      try_send_buffered_path_request(assigned_robot);
    } else {
      RCLCPP_WARN(
        get_logger(),
        "⚠️ [DELIVERY COORDS] No pickup/dropoff coordinates for task %s "
        "(thin response; pending robot_task_request pickup/dropoff also missing)",
        base_request_id.c_str());
    }

    RCLCPP_INFO(
      get_logger(),
      "📦 [DELIVERY] Stored task details for %s (robot: %s). "
      "Waiting for task to become 'underway' via /rmf_task_state_updates; "
      "path and pick/drop actions go on VDA5050 order only.",
      base_request_id.c_str(), assigned_robot.c_str());
    RCLCPP_INFO(
      get_logger(), "📦 [DELIVERY] Currently stored tasks: %s",
      map_keys_join(delivery_task_details).c_str());

    const auto underway_it = currently_underway_task.find(assigned_robot);
    if (underway_it != currently_underway_task.end() &&
      underway_it->second == base_request_id)
    {
      RCLCPP_INFO(
        get_logger(),
        "🚀 [DELIVERY] Task %s is ALREADY underway! Syncing underway tracking...",
        base_request_id.c_str());
      send_pickup_for_underway_task(assigned_robot, base_request_id);
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      get_logger(), "❌ [DELIVERY COORDS] Error processing delivery API payload: %s", e.what());
  }
}

void RMFIntegratedFleetAdapter::task_api_response_callback(
  const rmf_task_msgs::msg::ApiResponse::SharedPtr msg)
{
  // Python lines 2781–2820
  const std::string dedupe_key = msg->request_id + "\x1e" + msg->json_msg;
  if (!seen_task_api_response_dedupe.insert(dedupe_key).second) {
    RCLCPP_DEBUG(
      get_logger(),
      "⏭️ [TASK API RESPONSE] Skipping duplicate ApiResponse request_id=%s",
      msg->request_id.c_str());
    return;
  }
  if (seen_task_api_response_dedupe.size() > 500) {
    seen_task_api_response_dedupe.erase(seen_task_api_response_dedupe.begin());
  }

  ++task_api_response_count;
  RCLCPP_INFO(
    get_logger(),
    "🔔 [TASK API RESPONSE] ⭐ CALLBACK TRIGGERED ⭐ Received ApiResponse #%d",
    task_api_response_count);
  RCLCPP_INFO(
    get_logger(),
    "🔔 [TASK API RESPONSE] request_id=%s, json_msg length=%zu",
    msg->request_id.c_str(), msg->json_msg.size());

  YAML::Node payload;
  try {
    payload = YAML::Load(msg->json_msg);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "❌ [TASK API RESPONSE] Failed to parse json_msg: %s", e.what());
    return;
  }

  const YAML::Node state = payload["state"];
  const YAML::Node booking = state["booking"];
  const std::string booking_id = yaml_map_get_string(booking, "id");
  const std::string request_id = !booking_id.empty() ? booking_id :
    (!msg->request_id.empty() ? msg->request_id : "");

  RCLCPP_INFO(
    get_logger(),
    "🔍 [TASK API RESPONSE] Parsed - booking.id: %s, msg.request_id: %s, using request_id: %s",
    booking_id.c_str(), msg->request_id.c_str(), request_id.c_str());

  if (request_id.empty()) {
    RCLCPP_WARN(get_logger(), "⚠️ [TASK API RESPONSE] Missing request_id in ApiResponse");
    return;
  }

  if (!msg->request_id.empty()) {
    const auto pending_robot_it = pending_robot_by_api_request_id.find(msg->request_id);
    if (pending_robot_it != pending_robot_by_api_request_id.end()) {
      const std::string & robot_name = pending_robot_it->second;
      known_task_robot[request_id] = robot_name;
      dispatch_assignments[request_id] = robot_name;
      RCLCPP_INFO(
        get_logger(),
        "✅ [TASK API RESPONSE] Linked robot_task_request robot %s to booking %s",
        robot_name.c_str(), request_id.c_str());
      pending_robot_by_api_request_id.erase(pending_robot_it);
    }
    // Re-key cached request description from API request_id -> booking id so
    // process_*_api_payload(booking_id, ...) can recover places for thin responses.
    if (request_id != msg->request_id) {
      const auto pending_req_it =
        pending_task_request_by_api_request_id.find(msg->request_id);
      if (pending_req_it != pending_task_request_by_api_request_id.end()) {
        pending_task_request_by_api_request_id[request_id] =
          std::move(pending_req_it->second);
        pending_task_request_by_api_request_id.erase(pending_req_it);
      }
    }
  }

  std::string category;
  const auto pending_req_it = pending_task_request_by_api_request_id.find(request_id);
  if (pending_req_it != pending_task_request_by_api_request_id.end()) {
    category = infer_category_from_task_request(pending_req_it->second);
    if (!category.empty()) {
      RCLCPP_INFO(
        get_logger(),
        "🔍 [TASK API RESPONSE] Category from cached robot_task_request: %s",
        category.c_str());
    }
  }
  if (category.empty()) {
    category = to_lower_ascii(yaml_map_get_string(state, "category"));
  }
  if (category.empty()) {
    category = to_lower_ascii(yaml_map_get_string(payload, "category"));
  }
  if (category.empty()) {
    const YAML::Node detail = yaml_safe_map(state, "detail");
    if (
      yaml_safe_map(detail, "pickup").size() > 0 ||
      yaml_safe_map(detail, "dropoff").size() > 0)
    {
      category = "delivery";
    } else if (yaml_safe_sequence(detail, "places").size() > 0) {
      category = "patrol";
    } else if (!yaml_map_get_string(detail, "place").empty()) {
      category = "drop";
    }
  }
  // Thin robot_task responses may omit detail; use cached request description shape.
  if (category.empty() && pending_req_it != pending_task_request_by_api_request_id.end()) {
    category = infer_category_from_task_request(pending_req_it->second);
  }
  RCLCPP_INFO(
    get_logger(), "🔍 [TASK API RESPONSE] Category: %s, request_id: %s",
    category.c_str(), request_id.c_str());

  if (category == "delivery") {
    task_patrol_coords.erase(request_id);
    process_delivery_api_payload(request_id, payload);
  } else if (category == "patrol") {
    process_patrol_api_payload(request_id, payload);
  } else if (category == "drop") {
    process_drop_api_payload(request_id, payload);
  } else {
    RCLCPP_INFO(
      get_logger(), "⏭️ [TASK API RESPONSE] Skipping unsupported task category: %s",
      category.c_str());
  }
}

void RMFIntegratedFleetAdapter::dispatch_states_callback(
  const rmf_task_msgs::msg::DispatchStates::SharedPtr msg)
{
  // Python lines 3546–3645
  try {
    int updates = 0;
    RCLCPP_INFO(
      get_logger(),
      "📋 [DISPATCH STATES] Received %zu active, %zu finished",
      msg->active.size(), msg->finished.size());

    std::set<std::string> active_ids;
    for (const auto & dispatch_state : msg->active) {
      const std::string task_id = trim_copy(dispatch_state.task_id);
      if (!task_id.empty()) {
        active_ids.insert(task_id);
      }
    }
    std::set<std::string> finished_ids;
    for (const auto & dispatch_state : msg->finished) {
      const std::string task_id = trim_copy(dispatch_state.task_id);
      if (!task_id.empty()) {
        finished_ids.insert(task_id);
      }
    }

    std::vector<rmf_task_msgs::msg::DispatchState> all_states = msg->active;
    all_states.insert(all_states.end(), msg->finished.begin(), msg->finished.end());

    for (const auto & dispatch_state : all_states) {
      const std::string task_id = trim_copy(dispatch_state.task_id);
      if (task_id.empty()) {
        continue;
      }

      const auto & assignment = dispatch_state.assignment;
      const int status = static_cast<int>(dispatch_state.status);
      const bool terminal_dispatch =
        status == static_cast<int>(rmf_task_msgs::msg::DispatchState::STATUS_FAILED_TO_ASSIGN) ||
        status == static_cast<int>(rmf_task_msgs::msg::DispatchState::STATUS_CANCELED_IN_FLIGHT);

      const auto known_status_it = known_task_status.find(task_id);
      const std::string known_status =
        known_status_it != known_task_status.end() ? known_status_it->second : "";
      if (is_terminal_task_status(known_status) && active_ids.count(task_id) == 0) {
        dispatch_assignments.erase(task_id);
        continue;
      }

      if (active_ids.count(task_id) > 0) {
        clear_stale_local_cancel_status(task_id, "task still active in /dispatch_states");
      }

      RCLCPP_INFO(
        get_logger(),
        "📋 [DISPATCH STATES] Processing task_id: '%s', is_assigned: %s",
        task_id.c_str(), assignment.is_assigned ? "true" : "false");

      if (assignment.is_assigned && !terminal_dispatch) {
        const std::string robot_name = trim_copy(assignment.expected_robot_name);
        if (!robot_name.empty()) {
          const auto prev_it = dispatch_assignments.find(task_id);
          const std::string prev = prev_it != dispatch_assignments.end() ? prev_it->second : "";
          dispatch_assignments[task_id] = robot_name;
          RCLCPP_INFO(
            get_logger(),
            "📋 [DISPATCH STATES] Stored: '%s' → '%s' (prev: %s)",
            task_id.c_str(), robot_name.c_str(), prev.c_str());
          if (prev != robot_name) {
            ++updates;
          }

          const auto pending_it = pending_delivery_api_responses.find(task_id);
          if (pending_it != pending_delivery_api_responses.end()) {
            RCLCPP_INFO(
              get_logger(),
              "📬 [DISPATCH STATES] Processing deferred ApiResponse for %s now that robot %s is assigned",
              task_id.c_str(), robot_name.c_str());
            const YAML::Node pending_payload = pending_it->second;
            pending_delivery_api_responses.erase(pending_it);
            process_delivery_api_payload(task_id, pending_payload, robot_name);
          }

          if (task_patrol_coords.count(task_id) > 0) {
            RCLCPP_INFO(
              get_logger(),
              "📬 [DISPATCH STATES] Patrol coords already stored for %s — flushing buffered path for %s",
              task_id.c_str(), robot_name.c_str());
            try_send_buffered_path_request(robot_name);
          }

          RCLCPP_INFO(
            get_logger(),
            "📋 [DISPATCH STATES] Task %s assigned to %s. "
            "Waiting for status='underway' from /rmf_task_state_updates to trigger pickup.",
            task_id.c_str(), robot_name.c_str());
        } else {
          RCLCPP_WARN(
            get_logger(),
            "⚠️ [DISPATCH STATES] Task '%s' is assigned but expected_robot_name is empty",
            task_id.c_str());
        }
      } else {
        const auto known_robot_it = known_task_robot.find(task_id);
        const bool keep_robot_specific_assignment =
          known_robot_it != known_task_robot.end() && !known_robot_it->second.empty();
        if (keep_robot_specific_assignment) {
          const std::string & known_robot = known_robot_it->second;
          if (dispatch_assignments[task_id] != known_robot) {
            dispatch_assignments[task_id] = known_robot;
            RCLCPP_INFO(
              get_logger(),
              "📋 [DISPATCH STATES] Preserved robot-specific assignment '%s' → '%s' "
              "(is_assigned=false, robot known from task state)",
              task_id.c_str(), known_robot.c_str());
          }
        } else if (dispatch_assignments.erase(task_id) > 0) {
          RCLCPP_INFO(
            get_logger(),
            "📋 [DISPATCH STATES] Removed unassigned/terminal '%s' from dispatch_assignments",
            task_id.c_str());
        }
        RCLCPP_INFO(
          get_logger(),
          "📋 [DISPATCH STATES] Task '%s' not assigned yet (is_assigned=%s)",
          task_id.c_str(), assignment.is_assigned ? "true" : "false");
      }
    }

    std::vector<std::string> stale_ids;
    for (const auto & [tid, _] : dispatch_assignments) {
      if (active_ids.count(tid) > 0) {
        continue;
      }
      const auto known_it = known_task_status.find(tid);
      const std::string known_status = known_it != known_task_status.end() ? known_it->second : "";
      if (is_terminal_task_status(known_status) || finished_ids.count(tid) > 0) {
        stale_ids.push_back(tid);
      }
    }
    for (const auto & tid : stale_ids) {
      dispatch_assignments.erase(tid);
      RCLCPP_INFO(
        get_logger(), "📋 [DISPATCH STATES] Pruned stale terminal assignment '%s'", tid.c_str());
    }

    if (updates > 0) {
      RCLCPP_INFO(
        get_logger(),
        "📋 [DISPATCH STATES] Recorded %d assignment update(s) from /dispatch_states", updates);
      RCLCPP_INFO(
        get_logger(), "📋 [DISPATCH STATES] Current assignments: %s",
        string_map_to_log_dict(dispatch_assignments).c_str());
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "❌ [DISPATCH STATES] Failed to process dispatch states: %s", e.what());
  }
}

// ============================================================================
// Section 14 — Cancel (Python lines 3282–3525)
// ============================================================================

std::optional<std::string> RMFIntegratedFleetAdapter::find_robot_by_task_id(
  const std::string & task_id)
{
  // Python lines 3354–3372
  RCLCPP_INFO(get_logger(), "🔍 [FIND ROBOT] Looking for robot with task_id: '%s'", task_id.c_str());
  RCLCPP_INFO(
    get_logger(), "🔍 [FIND ROBOT] Checking dispatch_assignments: %s",
    string_map_to_log_dict(dispatch_assignments).c_str());

  const auto it = dispatch_assignments.find(task_id);
  if (it != dispatch_assignments.end()) {
    RCLCPP_INFO(
      get_logger(),
      "✅ [FIND ROBOT] Found robot %s in dispatch_assignments for task_id: %s",
      it->second.c_str(), task_id.c_str());
    return it->second;
  }

  // ---- Fallbacks ---------------------------------------------------------
  // dispatch_assignments alone is not a reliable index of "who is running this
  // task", because dispatch_states_callback() prunes entries it believes are
  // terminal — and rmf_task_dispatcher reports a task as `finished` the moment
  // it has finished DISPATCHING it, not when the robot has finished DOING it.
  //
  // Observed 2026-08-20: patrol.dispatch-0 was stored and then pruned 0.14 ms
  // later off a "Received 0 active, 1 finished" message, while amr1 was only
  // just starting to drive it. Every later cancel for that task found an empty
  // map, returned std::nullopt, and handle_cancel_task_request() gave up before
  // ever reaching the cancelOrder publish — so fleet-dispatched tasks
  // (patrol.dispatch-N) could not be cancelled on the robot at all, while
  // robot-targeted (UUID) tasks could, because those never pass through
  // /dispatch_states.
  //
  // The robot was knowable the whole time from task state that nothing prunes.
  // Consult it here rather than reporting "no robot". Returning a robot for a
  // task that has since finished is harmless: the caller re-checks with
  // is_task_executing_on_robot() and falls through to cleanup-only.

  // 1) Task state updates keep task_id -> robot directly, and are not pruned.
  const auto known_it = known_task_robot.find(task_id);
  if (known_it != known_task_robot.end() && !known_it->second.empty()) {
    RCLCPP_INFO(
      get_logger(),
      "✅ [FIND ROBOT] Found robot %s in known_task_robot for task_id: %s "
      "(dispatch_assignments had been pruned)",
      known_it->second.c_str(), task_id.c_str());
    return known_it->second;
  }

  // 2) The task the robot is actually underway on.
  for (const auto & [robot_name, underway_task] : currently_underway_task) {
    if (trim_copy(underway_task) == task_id && !robot_name.empty()) {
      RCLCPP_INFO(
        get_logger(),
        "✅ [FIND ROBOT] Found robot %s via currently_underway_task for task_id: %s",
        robot_name.c_str(), task_id.c_str());
      return robot_name;
    }
  }

  // 3) The booking the robot is reporting right now.
  if (fleet_config.has_value()) {
    for (const auto & [robot_name, robot] : fleet_config->robots) {
      if (trim_copy(robot.current_task) == task_id && !robot_name.empty()) {
        RCLCPP_INFO(
          get_logger(),
          "✅ [FIND ROBOT] Found robot %s via robot.current_task for task_id: %s",
          robot_name.c_str(), task_id.c_str());
        return robot_name;
      }
    }
  }

  RCLCPP_WARN(get_logger(), "⚠️ [FIND ROBOT] No robot found for task_id: %s", task_id.c_str());
  RCLCPP_WARN(
    get_logger(),
    "⚠️ [FIND ROBOT] Available dispatch_assignments: %s, known_task_robot: %s, "
    "currently_underway_task: %s",
    string_map_to_log_dict(dispatch_assignments).c_str(),
    string_map_to_log_dict(known_task_robot).c_str(),
    string_map_to_log_dict(currently_underway_task).c_str());
  return std::nullopt;
}

void RMFIntegratedFleetAdapter::clear_queued_task_tracking(
  const std::string & task_id, const std::string & robot_name)
{
  // Python lines 3374–3407
  try {
    if (dispatch_assignments.erase(task_id) > 0) {
      RCLCPP_INFO(
        get_logger(), "🧹 [CANCEL QUEUED] Removed %s from dispatch_assignments", task_id.c_str());
    }
    if (delivery_task_details.erase(task_id) > 0) {
      RCLCPP_INFO(
        get_logger(), "🧹 [CANCEL QUEUED] Removed %s from delivery_task_details", task_id.c_str());
    }
    pending_delivery_api_responses.erase(task_id);
    if (task_pickup_drop_coords.erase(task_id) > 0) {
      for (auto rk_it = robot_to_coords_key.begin(); rk_it != robot_to_coords_key.end(); ) {
        if (rk_it->second == task_id) {
          rk_it = robot_to_coords_key.erase(rk_it);
        } else {
          ++rk_it;
        }
      }
      RCLCPP_INFO(
        get_logger(), "🧹 [CANCEL QUEUED] Removed %s from task_pickup_drop_coords", task_id.c_str());
    }
    task_patrol_coords.erase(task_id);
    task_category_by_id.erase(task_id);
    clear_delivery_completion_state(robot_name, task_id);
    RCLCPP_INFO(
      get_logger(),
      "✅ [CANCEL QUEUED] Cleaned up tracking for queued task %s (robot %s)",
      task_id.c_str(), robot_name.c_str());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "❌ [CANCEL QUEUED] Error cleaning up: %s", e.what());
  }
}

void RMFIntegratedFleetAdapter::send_cancel_command_to_server(
  const std::string & robot_name,
  const std::string & task_id,
  const std::string & request_id)
{
  // Python lines 3409–3434
  try {
    std::vector<AnyMap> action_parameters;
    action_parameters.push_back({
      {"key", YAML::Node("task_id")},
      {"value", YAML::Node(task_id)},
    });
    if (!request_id.empty()) {
      action_parameters.push_back({
        {"key", YAML::Node("request_id")},
        {"value", YAML::Node(request_id)},
      });
    }
    action_parameters.push_back({
      {"key", YAML::Node("orderId")},
      {"value", YAML::Node(task_id)},
    });

    RCLCPP_INFO(
      get_logger(), "🌐 [CANCEL TASK] Sending VDA5050 cancelOrder to robot %s",
      robot_name.c_str());
    const bool ok = publish_vda5050_instant_action(
      robot_name,
      "cancelOrder",
      "cancel-" + task_id,
      &action_parameters,
      "Cancel current RMF task");
    if (ok) {
      RCLCPP_INFO(
        get_logger(), "✅ [CANCEL TASK] Successfully sent cancel command to robot %s",
        robot_name.c_str());
    } else {
      RCLCPP_ERROR(
        get_logger(), "❌ [CANCEL TASK] Failed to send cancel command to robot %s",
        robot_name.c_str());
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "❌ [CANCEL TASK] Error sending cancel command: %s", e.what());
  }
}

void RMFIntegratedFleetAdapter::clear_task_state(
  const std::string & robot_name, const std::string & task_id)
{
  // Phase 2: a task ending is a hard boundary — never carry a private lane
  // closure across it.
  clear_private_lane_closure(robot_name, "task ended");
  // Likewise drop the published-order record so a path for the NEXT task can
  // never be mistaken for a replan of the finished one.
  published_order_by_robot.erase(robot_name);
  // Cleared with the published order it is paired with: a stale route hash would
  // make the first path of the NEXT task look like a repeat and suppress its
  // cancel.
  last_accepted_path_hash_by_robot.erase(robot_name);
  // And any route staged for this task but never delivered: leg/revision keys
  // restart at 0 for the next task, so a leftover stage would otherwise be
  // matched and published as if it were the new task's route.
  if (staged_planned_path_by_robot.erase(robot_name) > 0) {
    RCLCPP_INFO(
      get_logger(),
      "🧹 [PLANNED PATH] Dropped undelivered staged path(s) for %s (task ended)",
      robot_name.c_str());
  }

  if (!robot_name.empty() && paused_by_deviation.count(robot_name) > 0) {
    paused_by_deviation.erase(robot_name);
    const std::string action_id =
      "cleartask-resume-" + robot_name + "-" +
      std::to_string(get_clock()->now().nanoseconds());
    publish_vda5050_instant_action(
      robot_name,
      "stopPause",
      action_id,
      nullptr,
      "Task state cleared — unpausing robot for future tasks");
  }

  // Python lines 3436–3525
  try {
    if (!fleet_config.has_value() ||
      fleet_config->robots.find(robot_name) == fleet_config->robots.end())
    {
      RCLCPP_WARN(get_logger(), "⚠️ [CLEAR TASK STATE] Robot %s not found", robot_name.c_str());
      return;
    }

    AMRRobot & robot = fleet_config->robots[robot_name];
    robot.current_task = "";
    robot.fleet_publish_task_id = "";
    robot.status = RobotStatus::Idle;
    RCLCPP_INFO(
      get_logger(), "🧹 [CLEAR TASK STATE] Cleared current_task for robot %s",
      robot_name.c_str());

    known_task_status[task_id] = "canceled";
    dispatch_assignments.erase(task_id);

    const auto underway_it = currently_underway_task.find(robot_name);
    if (underway_it != currently_underway_task.end() && underway_it->second == task_id) {
      currently_underway_task.erase(underway_it);
    }

    if (pending_path_request_by_robot.count(robot_name) > 0) {
      pending_path_request_by_robot[robot_name].clear();
      RCLCPP_INFO(
        get_logger(),
        "🧹 [CLEAR TASK STATE] Cleared buffered path requests for robot %s",
        robot_name.c_str());
    }

    if (active_task_robots.erase(task_id) > 0) {
      RCLCPP_INFO(
        get_logger(),
        "🧹 [CLEAR TASK STATE] Removed task_id %s from active_task_robots",
        task_id.c_str());
    }
    if (sent_path_request_phase.erase(task_id) > 0) {
      RCLCPP_INFO(
        get_logger(),
        "🧹 [CLEAR TASK STATE] Cleared sent_path_request_phase for task_id %s",
        task_id.c_str());
    }

    if (task_pickup_drop_coords.erase(task_id) > 0) {
      for (auto rk_it = robot_to_coords_key.begin(); rk_it != robot_to_coords_key.end(); ) {
        if (rk_it->second == task_id) {
          rk_it = robot_to_coords_key.erase(rk_it);
        } else {
          ++rk_it;
        }
      }
      RCLCPP_INFO(
        get_logger(),
        "🧹 [CLEAR TASK STATE] Removed task_id %s from task_pickup_drop_coords",
        task_id.c_str());
    }
    task_patrol_coords.erase(task_id);
    task_category_by_id.erase(task_id);

    if (robot_waypoint_sequences.erase(robot_name) > 0) {
      RCLCPP_INFO(
        get_logger(),
        "🧹 [CLEAR TASK STATE] Cleared waypoint sequences for robot %s",
        robot_name.c_str());
    }
    if (target_positions.erase(robot_name) > 0) {
      RCLCPP_INFO(
        get_logger(),
        "🧹 [CLEAR TASK STATE] Cleared target positions for robot %s",
        robot_name.c_str());
    }
    if (task_completion_delays.erase(robot_name) > 0) {
      RCLCPP_INFO(
        get_logger(),
        "🧹 [CLEAR TASK STATE] Cleared task completion delays for robot %s",
        robot_name.c_str());
    }
    if (robot_visited_waypoints.erase(robot_name) > 0) {
      RCLCPP_INFO(
        get_logger(),
        "🧹 [CLEAR TASK STATE] Cleared visited waypoints for robot %s",
        robot_name.c_str());
    }
    if (robot_path_assigned.count(robot_name) > 0) {
      robot_path_assigned[robot_name] = false;
      RCLCPP_INFO(
        get_logger(),
        "🧹 [CLEAR TASK STATE] Cleared path assigned flag for robot %s",
        robot_name.c_str());
    }
    if (robot_task_cleared.count(robot_name) > 0) {
      robot_task_cleared[robot_name] = false;
    }
    robot_path_cleared.erase(robot_name);
    if (last_known_tasks.count(robot_name) > 0) {
      last_known_tasks[robot_name] = "";
    }

    clear_delivery_completion_state(robot_name, task_id);

    RCLCPP_INFO(
      get_logger(),
      "✅ [CLEAR TASK STATE] Task state cleared for robot %s, task_id: %s",
      robot_name.c_str(), task_id.c_str());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "❌ [CLEAR TASK STATE] Error clearing task state: %s", e.what());
  }
}

void RMFIntegratedFleetAdapter::handle_cancel_task_request(
  const std::string & task_id, const std::string & request_id)
{
  // Python lines 3303–3352
  const auto robot_name_opt = find_robot_by_task_id(task_id);
  if (!robot_name_opt.has_value()) {
    RCLCPP_WARN(
      get_logger(),
      "⚠️ [CANCEL TASK] No robot found with task_id: %s "
      "(queued/unassigned — waiting for RMF task state to confirm cancel)",
      task_id.c_str());
    return;
  }

  const std::string robot_name = *robot_name_opt;
  RCLCPP_INFO(
    get_logger(), "✅ [CANCEL TASK] Found robot %s for task_id: %s",
    robot_name.c_str(), task_id.c_str());

  const bool is_executing_task = is_task_executing_on_robot(robot_name, task_id);
  std::string current_task;
  if (fleet_config.has_value() &&
    fleet_config->robots.find(robot_name) != fleet_config->robots.end())
  {
    current_task = trim_copy(fleet_config->robots[robot_name].current_task);
  }
  const auto underway_it = currently_underway_task.find(robot_name);
  const std::string underway_task =
    (underway_it != currently_underway_task.end()) ? underway_it->second : "";

  if (is_executing_task && underway_task == task_id && current_task != task_id) {
    RCLCPP_INFO(
      get_logger(),
      "🛑 [CANCEL TASK] Matched executing task via currently_underway_task "
      "(task_id=%s, robot.current_task=%s)",
      task_id.c_str(), current_task.c_str());
  }

  if (is_executing_task) {
    RCLCPP_INFO(
      get_logger(),
      "🛑 [CANCEL TASK] Task %s is the executing task for %s - sending cancel",
      task_id.c_str(), robot_name.c_str());
    send_cancel_command_to_server(robot_name, task_id, request_id);
    clear_task_state(robot_name, task_id);
  } else {
    const char * executing_display = current_task.empty() ? "none" : current_task.c_str();
    RCLCPP_INFO(
      get_logger(),
      "ℹ️ [CANCEL TASK] Task %s is queued for %s (executing: %s) - "
      "skipping robot cancel, cleanup only",
      task_id.c_str(), robot_name.c_str(), executing_display);
    clear_queued_task_tracking(task_id, robot_name);
  }
}

void RMFIntegratedFleetAdapter::task_api_request_callback(
  const rmf_task_msgs::msg::ApiRequest::SharedPtr msg)
{
  // Python lines 3282–3301
  RCLCPP_INFO(
    get_logger(), "🔔 [TASK API REQUEST] Received ApiRequest: request_id=%s",
    msg->request_id.c_str());
  try {
    const YAML::Node payload = YAML::Load(msg->json_msg);
    const std::string request_type = yaml_map_get_string(payload, "type");
    const std::string task_id = yaml_map_get_string(payload, "task_id");

    RCLCPP_INFO(
      get_logger(), "🔍 [TASK API REQUEST] Type: %s, Task ID: %s",
      request_type.c_str(), task_id.c_str());

    if (request_type == "robot_task_request") {
      const std::string robot_name = trim_copy(yaml_map_get_string(payload, "robot"));
      const std::string request_fleet = trim_copy(yaml_map_get_string(payload, "fleet"));
      if (robot_name.empty()) {
        RCLCPP_WARN(
          get_logger(),
          "⚠️ [TASK API REQUEST] robot_task_request missing robot name (request_id=%s)",
          msg->request_id.c_str());
      } else if (!request_fleet.empty() && request_fleet != fleet_name) {
        RCLCPP_DEBUG(
          get_logger(),
          "⏭️ [TASK API REQUEST] robot_task_request for other fleet '%s' (ours: '%s')",
          request_fleet.c_str(), fleet_name.c_str());
      } else {
        pending_robot_by_api_request_id[msg->request_id] = robot_name;
        RCLCPP_INFO(
          get_logger(),
          "✅ [TASK API REQUEST] Recorded robot %s for robot_task_request (request_id=%s)",
          robot_name.c_str(), msg->request_id.c_str());
      }
      // Save original request description (places) — thin robot_task responses often omit it.
      const YAML::Node request_node = payload["request"];
      if (request_node.IsDefined() && !request_node.IsNull() &&
        (request_node.IsMap() || request_node.IsSequence()))
      {
        pending_task_request_by_api_request_id[msg->request_id] = YAML::Clone(request_node);
        RCLCPP_INFO(
          get_logger(),
          "📍 [TASK API REQUEST] Cached robot_task_request description for request_id=%s",
          msg->request_id.c_str());
      }
    } else if (request_type == "cancel_task_request" && !task_id.empty()) {
      RCLCPP_INFO(
        get_logger(), "🛑 [CANCEL TASK] Processing cancel request for task_id: %s",
        task_id.c_str());
      handle_cancel_task_request(task_id, msg->request_id);
    } else {
      RCLCPP_DEBUG(
        get_logger(), "⏭️ [TASK API REQUEST] Ignoring non-cancel request: type=%s",
        request_type.c_str());
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "❌ [TASK API REQUEST] Failed to parse json_msg: %s", e.what());
  }
}

// ============================================================================
// Delivery queue/id helpers (Python lines 3748–3766, 4093–4110)
// ============================================================================

namespace
{

bool all_ascii_digits(const std::string & s)
{
  return !s.empty() &&
    std::all_of(s.begin(), s.end(), [](unsigned char c) { return std::isdigit(c); });
}

std::tuple<std::string, std::string, std::string> rpartition_dash(const std::string & value)
{
  const std::size_t pos = value.rfind('-');
  if (pos == std::string::npos) {
    return {value, "", ""};
  }
  return {value.substr(0, pos), "-", value.substr(pos + 1)};
}

}  // namespace

void RMFIntegratedFleetAdapter::try_start_next_delivery_task()
{
  // Python lines 3748–3766
  try {
    if (current_delivery_task_id.has_value()) {
      return;
    }
    if (delivery_task_queue.empty()) {
      return;
    }

    const JsonDict & task = delivery_task_queue.front();
    const auto robot_it = task.find("robot_id");
    const auto task_it = task.find("task_id");
    if (robot_it == task.end() || task_it == task.end()) {
      return;
    }

    const std::string & robot_id = robot_it->second;
    const std::string & base_request_id = task_it->second;

    RCLCPP_INFO(
      get_logger(),
      "📦 [DELIVERY QUEUE] Activating queued task %s on %s "
      "(VDA5050 order from PathRequest; no separate pick/drop MQTT).",
      base_request_id.c_str(), robot_id.c_str());
    current_delivery_task_id = base_request_id;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "❌ [DELIVERY QUEUE] Error starting next delivery task: %s", e.what());
  }
}

bool RMFIntegratedFleetAdapter::is_pickup_phase(const std::string & request_id)
{
  // Python lines 4093–4097
  const auto [prefix, sep, suffix] = rpartition_dash(request_id);
  (void)prefix;
  (void)sep;
  if (all_ascii_digits(suffix)) {
    return suffix == "0";
  }
  return true;
}

std::optional<std::string> RMFIntegratedFleetAdapter::get_next_delivery_request_id(
  const std::string & request_id)
{
  // Python lines 4099–4104
  if (is_uuid_task_id(request_id)) {
    return std::nullopt;
  }
  const auto [prefix, sep, suffix] = rpartition_dash(request_id);
  if (sep.empty() || !all_ascii_digits(suffix)) {
    return std::nullopt;
  }
  const int next_index = std::stoi(suffix) + 1;
  return prefix + "-" + std::to_string(next_index);
}

std::string RMFIntegratedFleetAdapter::get_base_delivery_request_id(const std::string & request_id)
{
  // Python lines 4106–4110
  // A uuid task id must never be split on its last dash: a uuid whose final
  // segment happens to be all digits would otherwise be rewritten into a
  // "<prefix>-0" id that does not exist.
  if (is_uuid_task_id(request_id)) {
    return request_id;
  }
  const auto [prefix, sep, suffix] = rpartition_dash(request_id);
  if (sep.empty() || !all_ascii_digits(suffix)) {
    return request_id;
  }
  return prefix + "-0";
}

// ============================================================================
// Section 16 — Legacy path helpers (Python lines 4112–4204)
// ============================================================================

bool RMFIntegratedFleetAdapter::is_delivery_task(const std::string & task_id) const
{
  // Python lines 4140–4142
  // Prefer the recorded category so robot-targeted (uuid) deliveries are
  // detected too; fall back to the legacy substring test.
  const std::string category = task_category_for(task_id);
  if (!category.empty()) {
    return category == "delivery";
  }
  std::string lower = task_id;
  std::transform(
    lower.begin(), lower.end(), lower.begin(),
    [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return lower.find("delivery") != std::string::npos;
}

rmf_fleet_msgs::msg::Location RMFIntegratedFleetAdapter::create_location_msg(
  const Waypoint & waypoint, const std::string & level_name)
{
  // Python lines 4196–4204
  rmf_fleet_msgs::msg::Location location;
  location.x = waypoint.x;
  location.y = waypoint.y;
  location.yaw = waypoint.yaw;
  location.level_name = level_name;
  location.t = get_clock()->now();
  return location;
}

std::vector<rmf_fleet_msgs::msg::Location> RMFIntegratedFleetAdapter::plan_delivery_path(
  const std::string & robot_name,
  const Pose2D & current_pos,
  const std::string & task_id)
{
  // Python lines 4144–4194
  try {
    if (!nav_graph_parser) {
      RCLCPP_ERROR(get_logger(), "❌ Navigation graph parser not available");
      return {};
    }

    const std::string pickup_location = "P1";
    const std::string dropoff_location = "P4";

    RCLCPP_INFO(
      get_logger(),
      "🚚 Planning delivery: Current -> %s -> %s",
      pickup_location.c_str(), dropoff_location.c_str());

    const auto pickup_wp = nav_graph_parser->get_waypoint_by_name(pickup_location);
    const auto dropoff_wp = nav_graph_parser->get_waypoint_by_name(dropoff_location);

    if (!pickup_wp || !dropoff_wp) {
      RCLCPP_ERROR(
        get_logger(), "❌ Could not find waypoints: %s, %s",
        pickup_location.c_str(), dropoff_location.c_str());
      return {};
    }

    std::vector<rmf_fleet_msgs::msg::Location> enhanced_path;

    const double current_x = current_pos.x;
    const double current_y = current_pos.y;
    if (std::abs(current_x - pickup_wp->x) > 0.1 || std::abs(current_y - pickup_wp->y) > 0.1) {
      RCLCPP_INFO(
        get_logger(),
        "📍 Adding pickup waypoint: %s (%.2f, %.2f)",
        pickup_location.c_str(), pickup_wp->x, pickup_wp->y);
      enhanced_path.push_back(create_location_msg(*pickup_wp, "L1"));
    }

    RCLCPP_INFO(
      get_logger(),
      "📍 Adding dropoff waypoint: %s (%.2f, %.2f)",
      dropoff_location.c_str(), dropoff_wp->x, dropoff_wp->y);
    enhanced_path.push_back(create_location_msg(*dropoff_wp, "L1"));

    DeliveryTask delivery;
    delivery.task_id = task_id;
    delivery.pickup_location = pickup_location;
    delivery.dropoff_location = dropoff_location;
    delivery.robot_name = robot_name;
    active_deliveries[task_id] = delivery;

    RCLCPP_INFO(
      get_logger(), "✅ Created delivery path with %zu waypoints",
      enhanced_path.size());
    return enhanced_path;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "❌ Error planning delivery path: %s", e.what());
    return {};
  }
}

std::vector<rmf_fleet_msgs::msg::Location> RMFIntegratedFleetAdapter::enhance_path_for_delivery(
  const std::string & robot_name,
  const std::vector<rmf_fleet_msgs::msg::Location> & original_path,
  const std::string & task_id)
{
  // Python lines 4112–4138
  try {
    if (!is_delivery_task(task_id)) {
      return original_path;
    }

    RCLCPP_INFO(get_logger(), "🚚 Detected delivery task: %s", task_id.c_str());

    if (!fleet_config.has_value() ||
      fleet_config->robots.find(robot_name) == fleet_config->robots.end())
    {
      return original_path;
    }

    const AMRRobot & robot = fleet_config->robots.at(robot_name);
    const std::vector<rmf_fleet_msgs::msg::Location> delivery_path =
      plan_delivery_path(robot_name, robot.pose, task_id);

    if (!delivery_path.empty()) {
      RCLCPP_INFO(
        get_logger(), "✅ Generated delivery path with %zu waypoints",
        delivery_path.size());
      return delivery_path;
    }

    RCLCPP_WARN(get_logger(), "⚠️  Failed to generate delivery path, using original");
    return original_path;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "❌ Error enhancing path: %s", e.what());
    return original_path;
  }
}

void RMFIntegratedFleetAdapter::mode_request_callback(
  const rmf_fleet_msgs::msg::ModeRequest::SharedPtr msg)
{
  // Section 15 — Python lines 4206–4265
  try {
    RCLCPP_INFO(get_logger(), "Received mode request for robot: %s", msg->robot_name.c_str());
    RCLCPP_INFO(get_logger(), "Mode: %u", msg->mode.mode);
    RCLCPP_INFO(get_logger(), "Task ID: %s", msg->task_id.c_str());

    if (!fleet_config.has_value() ||
      fleet_config->robots.find(msg->robot_name) == fleet_config->robots.end())
    {
      RCLCPP_WARN(get_logger(), "Unknown robot: %s", msg->robot_name.c_str());
      return;
    }

    AMRRobot & robot = fleet_config->robots[msg->robot_name];

    const auto delay_it = task_completion_delays.find(msg->robot_name);
    if (delay_it != task_completion_delays.end()) {
      RCLCPP_INFO(
        get_logger(),
        "🎯 [MODE REQUEST] Clearing completion delay for old task %s on robot %s",
        delay_it->second.first.c_str(), msg->robot_name.c_str());
      task_completion_delays.erase(delay_it);
    }

    const auto wp_it = robot_waypoint_sequences.find(msg->robot_name);
    if (wp_it != robot_waypoint_sequences.end()) {
      RCLCPP_INFO(
        get_logger(),
        "🎯 [MODE REQUEST] Clearing %zu old waypoints for robot %s",
        wp_it->second.size(), msg->robot_name.c_str());
      robot_waypoint_sequences.erase(wp_it);
    }

    target_positions.erase(msg->robot_name);
    robot_path_assigned[msg->robot_name] = false;
    robot_task_cleared[msg->robot_name] = false;
    robot_path_cleared.erase(msg->robot_name);

    const auto visited_it = robot_visited_waypoints.find(msg->robot_name);
    if (visited_it != robot_visited_waypoints.end()) {
      const std::size_t old_visited_count = visited_it->second.size();
      visited_it->second.clear();
      RCLCPP_INFO(
        get_logger(),
        "🔄 [VISITED TRACKING] Cleared %zu visited waypoints for mode request on robot %s",
        old_visited_count, msg->robot_name.c_str());
    }

    const std::string mid = trim_copy(msg->task_id);
    const bool mid_all_digits = !mid.empty() &&
      std::all_of(mid.begin(), mid.end(), [](unsigned char c) { return std::isdigit(c); });

    if (mid_all_digits) {
      robot.fleet_publish_task_id = mid;
      std::string resolved = resolve_booking_task_id_for_path_request(
        msg->robot_name, mid, nullptr);
      robot.current_task = resolved.empty() ? mid : resolved;
    } else {
      robot.current_task = mid;
      if (mid.find(".dispatch-") != std::string::npos) {
        robot.fleet_publish_task_id = "";
      }
    }

    RCLCPP_INFO(
      get_logger(),
      "Updated robot %s: booking='%s', fleet_publish_task_id='%s'",
      msg->robot_name.c_str(), robot.current_task.c_str(),
      robot.fleet_publish_task_id.c_str());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to handle mode request: %s", e.what());
  }
}

void RMFIntegratedFleetAdapter::task_state_update_callback(
  const std_msgs::msg::String::SharedPtr msg)
{
  // ============================================================================
  // Section 12 — Task state updates (Python lines 3061–3280)
  // ============================================================================
  // Python lines 3061–3230
  try {
    const YAML::Node task_state = YAML::Load(msg->data);

    const YAML::Node booking = task_state["booking"];
    const std::string task_id = yaml_map_get_string(booking, "id");
    const std::string status = yaml_map_get_string(task_state, "status");
    const YAML::Node assigned_to = task_state["assigned_to"];
    std::string robot_name;
    std::string fleet_name_from_msg;
    if (assigned_to.IsMap() && assigned_to.size() > 0) {
      robot_name = yaml_map_get_string(assigned_to, "name");
      fleet_name_from_msg = yaml_map_get_string(assigned_to, "group");
    }
    const std::string category = yaml_map_get_string(task_state, "category");

    if (task_id.empty()) {
      return;
    }

    known_task_status[task_id] = status;

    // ---- Release the private lane closure when its task is over ------------
    // This is the ONLY release that cannot be starved. The others all depend on
    // a PathRequest arriving, and a PathRequest is the OUTPUT of successful
    // planning — which is exactly what a closure can prevent:
    //
    //     closure blocks planning -> no path -> no PathRequest -> no release
    //
    // Observed 2026-08-22: amr2's closure for task ab4ea988 (zones 5+6, 10
    // lanes) survived that task completing at 1787341965, and its next task
    // 91106d20 could not be planned at all — "Beginning new task" with no path
    // ever issued, and the closure still held 542 s later.
    //
    // /rmf_task_state_updates comes from RMF's task state machine and is
    // independent of the planner, so it arrives whether or not a route exists.
    // Matched on the closure's own task id rather than on robot_name, because
    // assigned_to may be absent on some updates.
    if (is_terminal_task_status(status)) {
      for (const auto & [closure_robot, closure] : private_closure_by_robot) {
        if (closure.order_id.empty() || closure.order_id != task_id) {
          continue;
        }
        RCLCPP_INFO(
          get_logger(),
          "🔓 [ZONE REPLAN] task %s reached '%s' — releasing %s's private lane "
          "closure (blocked=%s, conflict=%s, lanes=%zu, held %.0fs)",
          task_id.c_str(), status.c_str(), closure_robot.c_str(),
          closure.blocked_zones.empty() ? "?" : closure.blocked_zones.c_str(),
          closure.zone_id.c_str(), closure.lanes.size(),
          (get_clock()->now() - closure.applied_at).seconds());
        // Copy the key: clear_private_lane_closure() erases from the map we are
        // iterating, so the loop must not continue afterwards.
        const std::string robot_to_clear = closure_robot;
        clear_private_lane_closure(robot_to_clear, "task " + status);
        break;
      }
    }

    if (!robot_name.empty()) {
      known_task_robot[task_id] = robot_name;
      if (!is_terminal_task_status(status)) {
        dispatch_assignments[task_id] = robot_name;
        RCLCPP_INFO(
          get_logger(),
          "📋 [TASK STATE UPDATE] Recorded robot-specific assignment '%s' → '%s' (status=%s)",
          task_id.c_str(), robot_name.c_str(), status.c_str());

        const auto pending_it = pending_delivery_api_responses.find(task_id);
        if (pending_it != pending_delivery_api_responses.end()) {
          RCLCPP_INFO(
            get_logger(),
            "📬 [TASK STATE UPDATE] Processing deferred delivery ApiResponse for %s "
            "using robot assignment %s",
            task_id.c_str(), robot_name.c_str());
          const YAML::Node pending_payload = pending_it->second;
          pending_delivery_api_responses.erase(pending_it);
          process_delivery_api_payload(task_id, pending_payload, robot_name);
        }

        if (task_patrol_coords.count(task_id) > 0) {
          try_send_buffered_path_request(robot_name);
        }
      }
    }

    const AnyMap booking_map = yaml_map_to_anymap(booking);
    const std::string monitored_priority = priority_for_monitored_topic(task_id, booking_map);
    const std::vector<std::string> latest_path = !robot_name.empty() ?
      monitored_task_manager->get_planned_path(robot_name) : std::vector<std::string>{};
    const std::string latest_load_status = !robot_name.empty() ?
      monitored_task_manager->get_load_status(robot_name) : "";
    const std::string category_str = category;
    const std::string phase = monitored_phase_for_task(task_id, category_str);
    update_monitored_task_cache(
      task_id,
      {
        {"task_id", YAML::Node(task_id)},
        {"amr_id", YAML::Node(robot_name)},
        {"fleet_name", YAML::Node(fleet_name_from_msg.empty() ? fleet_name : fleet_name_from_msg)},
        {"category", YAML::Node(category_str)},
        {"priority", YAML::Node(monitored_priority)},
        {"planned_path", string_vector_to_yaml_sequence(latest_path)},
        {"load_status", YAML::Node(latest_load_status)},
        {"phase", YAML::Node(phase)},
      });
    handle_monitored_publish_rule(task_id, status);
    record_task_phase_places(task_id, task_state);

    RCLCPP_INFO(
      get_logger(),
      "📨 [TASK STATE UPDATE] task_id=%s, status=%s, robot=%s, category=%s",
      task_id.c_str(), status.c_str(), robot_name.c_str(), category.c_str());

    if (!category.empty()) {
      const std::string category_lower = to_lower_ascii(category);
      if (category_lower != "delivery" && category_lower != "patrol" && category_lower != "drop") {
        RCLCPP_DEBUG(
          get_logger(), "⏭️ [TASK STATE UPDATE] Skipping unsupported task: %s",
          category.c_str());
        return;
      }
    }

    if (robot_name.empty()) {
      RCLCPP_DEBUG(
        get_logger(), "⏭️ [TASK STATE UPDATE] Task %s not assigned to a robot yet",
        task_id.c_str());
      return;
    }

    const auto prev_underway_it = currently_underway_task.find(robot_name);
    const std::string prev_underway = (prev_underway_it != currently_underway_task.end()) ?
      prev_underway_it->second : "";

    if (status == "underway") {
      if (prev_underway != task_id) {
        RCLCPP_INFO(
          get_logger(),
          "🚀 [TASK STATE UPDATE] Task %s is NOW UNDERWAY for robot %s",
          task_id.c_str(), robot_name.c_str());
        currently_underway_task[robot_name] = task_id;

        if (fleet_config.has_value() &&
          fleet_config->robots.find(robot_name) != fleet_config->robots.end())
        {
          AMRRobot & robot = fleet_config->robots[robot_name];
          const bool task_cleared = robot_task_cleared.count(robot_name) > 0 &&
            robot_task_cleared.at(robot_name);
          if (robot.current_task.empty() && !task_cleared) {
            robot.current_task = task_id;
          }
        }

        if (!category.empty() && to_lower_ascii(category) == "delivery") {
          const auto pending_it = pending_delivery_api_responses.find(task_id);
          if (pending_it != pending_delivery_api_responses.end()) {
            RCLCPP_INFO(
              get_logger(),
              "📬 [TASK STATE UPDATE] Processing deferred delivery ApiResponse for %s "
              "using underway robot assignment %s",
              task_id.c_str(), robot_name.c_str());
            const YAML::Node pending_payload = pending_it->second;
            pending_delivery_api_responses.erase(pending_it);
            process_delivery_api_payload(task_id, pending_payload, robot_name);
          }
        }

        if (!category.empty() && to_lower_ascii(category) == "patrol") {
          send_patrol_for_underway_task(robot_name, task_id);
        } else if (!category.empty() && to_lower_ascii(category) == "drop") {
          RCLCPP_INFO(
            get_logger(),
            "📦 [DROP] Task %s underway for robot %s; "
            "waiting for path_request to publish DROP order.",
            task_id.c_str(), robot_name.c_str());
        } else {
          send_pickup_for_underway_task(robot_name, task_id);
        }

        try_send_buffered_path_request(robot_name);
      }
    } else if (status == "completed") {
      if (currently_underway_task.count(robot_name) > 0 &&
        currently_underway_task.at(robot_name) == task_id)
      {
        RCLCPP_INFO(
          get_logger(),
          "✅ [TASK STATE UPDATE] Task %s COMPLETED for robot %s",
          task_id.c_str(), robot_name.c_str());
        currently_underway_task.erase(robot_name);
      }
      if (dispatch_assignments.erase(task_id) > 0) {
        RCLCPP_INFO(
          get_logger(),
          "🧹 [TASK STATE UPDATE] Removed %s from dispatch_assignments (completed)",
          task_id.c_str());
      }
      active_task_robots.erase(task_id);
      finalize_robot_path_on_task_end(robot_name, task_id);
      task_patrol_coords.erase(task_id);
      if (!category.empty() && to_lower_ascii(category) == "drop") {
        task_pickup_drop_coords.erase(task_id);
        dropoff_in_progress.erase(task_id);
        task_phase_places.erase(task_id);
        RCLCPP_INFO(
          get_logger(),
          "🧹 [TASK STATE UPDATE] Removed %s from task_patrol_coords (completed)",
          task_id.c_str());
      }
    } else if (status == "canceled" || status == "failed") {
      if (currently_underway_task.count(robot_name) > 0 &&
        currently_underway_task.at(robot_name) == task_id)
      {
        std::string status_upper = status;
        std::transform(status_upper.begin(), status_upper.end(), status_upper.begin(), ::toupper);
        RCLCPP_INFO(
          get_logger(),
          "❌ [TASK STATE UPDATE] Task %s %s for robot %s",
          task_id.c_str(), status_upper.c_str(), robot_name.c_str());
        currently_underway_task.erase(robot_name);
      }
      if (dispatch_assignments.erase(task_id) > 0) {
        RCLCPP_INFO(
          get_logger(),
          "🧹 [TASK STATE UPDATE] Removed %s from dispatch_assignments (%s)",
          task_id.c_str(), status.c_str());
      }
      active_task_robots.erase(task_id);
      finalize_robot_path_on_task_end(robot_name, task_id);
      task_patrol_coords.erase(task_id);
      task_pickup_drop_coords.erase(task_id);
      dropoff_in_progress.erase(task_id);
      pending_ingestor_by_robot.erase(robot_name);
      delivery_task_details.erase(task_id);
      if (current_delivery_task_id.has_value() && *current_delivery_task_id == task_id) {
        current_delivery_task_id = std::nullopt;
      }
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "❌ [TASK STATE UPDATE] Failed to process: %s", e.what());
  }
}

}  // namespace execution

// Section 16 — main (Python lines 4300–4332)
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  try {
    std::cout <<
      "Creating RMF Integrated Fleet Adapter (Full Control Integration)..." << std::endl;
    auto fleet_adapter = std::make_shared<execution::RMFIntegratedFleetAdapter>();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(fleet_adapter);

    std::cout << "RMF Integrated Fleet Adapter (Full Control) started successfully" << std::endl;
    std::cout << "Publishing FleetState messages to /fleet_states" << std::endl;
    std::cout <<
      "Full control fleet adapter will automatically add robots when it receives FleetState messages" <<
      std::endl;
    std::cout << "\nPress Ctrl+C to stop..." << std::endl;

    executor.spin();

    if (!rclcpp::ok()) {
      std::cout << "\nShutting down RMF Integrated Fleet Adapter..." << std::endl;
    }
  } catch (const std::exception & e) {
    std::cerr << "Error in RMF Integrated Fleet Adapter: " << e.what() << std::endl;
  }

  std::cout << "Cleaning up..." << std::endl;
  rclcpp::shutdown();
  return 0;
}
