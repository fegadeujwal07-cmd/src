// NavGraphStore implementation: load nav-graph YAML and build filtered graphs / RViz markers.
#include "monitor/nav_graph_store.hpp"

#include <algorithm>  // std::sort for stable vertex index ordering
#include <chrono>     // std::chrono for function execution timing
#include <sstream>    // std::ostringstream for robot zone label text

#include "geometry_msgs/msg/point.hpp"
#include "rmf_building_map_msgs/msg/graph_edge.hpp"
#include "rmf_building_map_msgs/msg/graph_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "yaml-cpp/yaml.h"

namespace monitor
{
namespace
{

// Return the ROS logger used by all NavGraphStore log statements.
rclcpp::Logger nav_graph_store_logger()
{
  return rclcpp::get_logger("nav_graph_store");
}

// Log elapsed wall time when this object goes out of scope (covers all return paths).
class FunctionTimer
{
public:
  explicit FunctionTimer(const char * function_name)
  : function_name_(function_name),
    start_(std::chrono::steady_clock::now())
  {
  }

  ~FunctionTimer()
  {
    const auto end = std::chrono::steady_clock::now();
    const double elapsed_ms =
      std::chrono::duration<double, std::milli>(end - start_).count();
    RCLCPP_DEBUG(
      nav_graph_store_logger(),
      "%s execution time: %.3f ms",
      function_name_,
      elapsed_ms);
  }

private:
  const char * function_name_;
  std::chrono::steady_clock::time_point start_;
};

}  // namespace

// Parse vertices, lanes, and zones from a nav-graph YAML file for one building level.
bool NavGraphStore::load_map(const std::string & yaml_path, const std::string & level)
{
  const FunctionTimer timer("load_map");

  // Reset in-memory caches before loading a new map.
  vertices_.clear();
  lanes_.clear();
  zones_by_id_.clear();
  loaded_ = false;

  // Log the start of map loading for traceability.
  RCLCPP_INFO(
    nav_graph_store_logger(),
    "Loading nav graph from '%s' (level=%s)",
    yaml_path.c_str(),
    level.c_str());

  YAML::Node root;
  try {
    // Read and parse the YAML file from disk.
    root = YAML::LoadFile(yaml_path);
  } catch (const std::exception & ex) {
    // Log parse/read failures and report unsuccessful load.
    RCLCPP_ERROR(
      nav_graph_store_logger(),
      "Failed to load nav graph YAML '%s': %s",
      yaml_path.c_str(),
      ex.what());
    return false;
  }

  // Select the requested level node; vertices are mandatory.
  const YAML::Node level_node = root["levels"][level];
  if (!level_node || !level_node["vertices"]) {
    RCLCPP_ERROR(
      nav_graph_store_logger(),
      "Nav graph YAML '%s' missing level '%s' or vertices",
      yaml_path.c_str(),
      level.c_str());
    return false;
  }

  // Walk each vertex entry: [x, y, optional_properties_map].
  for (const auto & vertex_node : level_node["vertices"]) {
    // Skip malformed entries that are not [x, y, ...] sequences.
    if (!vertex_node.IsSequence() || vertex_node.size() < 2) {
      continue;
    }
    Vertex vertex;
    // First two values are always planar coordinates.
    vertex.x = vertex_node[0].as<double>();
    vertex.y = vertex_node[1].as<double>();
    // Optional third map holds name and zone label tags.
    if (vertex_node.size() > 2 && vertex_node[2].IsMap()) {
      if (vertex_node[2]["name"]) {
        try {
          vertex.name = vertex_node[2]["name"].as<std::string>();
        } catch (...) {
          // Ignore invalid name fields and keep an empty name.
        }
      }
      if (vertex_node[2]["zone_labels"]) {
        // Copy every zone label string attached to this vertex.
        for (const auto & label : vertex_node[2]["zone_labels"]) {
          vertex.zone_labels.push_back(label.as<std::string>());
        }
      }
    }
    vertices_.push_back(std::move(vertex));
  }

  // Lanes are optional; each lane is [from_vertex_index, to_vertex_index, ...].
  if (level_node["lanes"]) {
    for (const auto & lane_node : level_node["lanes"]) {
      if (!lane_node.IsSequence() || lane_node.size() < 2) {
        continue;
      }
      Lane lane;
      try {
        lane.from_vertex = lane_node[0].as<int>();
        lane.to_vertex = lane_node[1].as<int>();
      } catch (...) {
        // Skip lanes whose endpoints are not valid integers.
        continue;
      }
      lanes_.push_back(lane);
    }
  }

  // Zones are optional; each zone defines id, waypoints, and boundary vertex indices.
  if (level_node["zones"]) {
    for (const auto & zone_node : level_node["zones"]) {
      if (!zone_node["id"]) {
        continue;
      }
      Zone zone;
      zone.id = zone_node["id"].as<std::string>();
      if (zone_node["name"]) {
        try {
          zone.name = zone_node["name"].as<std::string>();
        } catch (...) {
          // Ignore invalid zone name fields.
        }
      }
      if (zone_node["waypoints"]) {
        // Waypoint indices reference vertices shown as zone marker spheres.
        for (const auto & idx : zone_node["waypoints"]) {
          zone.waypoints.push_back(idx.as<int>());
        }
      }
      if (zone_node["edge"]) {
        // "edge" holds boundary vertex indices, not lane indices.
        for (const auto & idx : zone_node["edge"]) {
          zone.edge.push_back(idx.as<int>());
        }
      }
      zones_by_id_[zone.id] = std::move(zone);
    }
  }

  // Map is considered loaded only when at least one vertex exists.
  loaded_ = !vertices_.empty();
  if (loaded_) {
    RCLCPP_INFO(
      nav_graph_store_logger(),
      "Loaded nav graph from '%s' (level=%s): vertices=%zu lanes=%zu zones=%zu",
      yaml_path.c_str(),
      level.c_str(),
      vertices_.size(),
      lanes_.size(),
      zones_by_id_.size());
  } else {
    RCLCPP_ERROR(
      nav_graph_store_logger(),
      "Nav graph YAML '%s' (level=%s) contains no vertices",
      yaml_path.c_str(),
      level.c_str());
  }
  return loaded_;
}

// Build an RMF Graph message containing only vertices/lanes for the given zone IDs.
rmf_building_map_msgs::msg::Graph NavGraphStore::build_filtered_graph(
  const std::unordered_set<std::string> & zone_ids,
  const std::string & graph_name) const
{
  const FunctionTimer timer("build_filtered_graph");

  rmf_building_map_msgs::msg::Graph graph_msg;
  graph_msg.name = graph_name;

  // Nothing to publish when the map is missing or no zones are active.
  if (!loaded_ || zone_ids.empty()) {
    RCLCPP_DEBUG(
      nav_graph_store_logger(),
      "Skipping filtered graph build: loaded=%s zone_count=%zu",
      loaded_ ? "true" : "false",
      zone_ids.size());
    return graph_msg;
  }

  // Collect original vertex indices that belong to any requested zone.
  std::unordered_set<int> allowed_indices;

  // Include vertices listed on zone waypoint/edge definitions.
  for (const auto & zone_id : zone_ids) {
    const auto zone_it = zones_by_id_.find(zone_id);
    if (zone_it == zones_by_id_.end()) {
      continue;
    }
    for (int idx : zone_it->second.waypoints) {
      if (idx >= 0 && idx < static_cast<int>(vertices_.size())) {
        allowed_indices.insert(idx);
      }
    }
    for (int idx : zone_it->second.edge) {
      if (idx >= 0 && idx < static_cast<int>(vertices_.size())) {
        allowed_indices.insert(idx);
      }
    }
  }

  // Also include vertices whose zone_labels match (same as legacy visualizer).
  for (std::size_t idx = 0; idx < vertices_.size(); ++idx) {
    for (const auto & label : vertices_[idx].zone_labels) {
      if (zone_ids.count(label) > 0) {
        allowed_indices.insert(static_cast<int>(idx));
        break;
      }
    }
  }

  if (allowed_indices.empty()) {
    RCLCPP_DEBUG(
      nav_graph_store_logger(),
      "Filtered graph empty: no vertices matched %zu active zone(s)",
      zone_ids.size());
    return graph_msg;
  }

  // Sort indices so remapped graph vertex order is stable across publishes.
  std::vector<int> sorted_indices(allowed_indices.begin(), allowed_indices.end());
  std::sort(sorted_indices.begin(), sorted_indices.end());

  // Remap old vertex indices to compact 0..N-1 indices in the output graph.
  std::unordered_map<int, int> old_to_new;
  old_to_new.reserve(sorted_indices.size());
  for (int old_idx : sorted_indices) {
    const Vertex & v = vertices_[static_cast<std::size_t>(old_idx)];
    rmf_building_map_msgs::msg::GraphNode node;
    node.x = v.x;
    node.y = v.y;
    node.name = v.name;
    old_to_new[old_idx] = static_cast<int>(graph_msg.vertices.size());
    graph_msg.vertices.push_back(node);
  }

  // Keep only lanes whose both endpoints survived the zone filter.
  for (const auto & lane : lanes_) {
    const auto from_it = old_to_new.find(lane.from_vertex);
    const auto to_it = old_to_new.find(lane.to_vertex);
    if (from_it == old_to_new.end() || to_it == old_to_new.end()) {
      continue;
    }
    rmf_building_map_msgs::msg::GraphEdge edge;
    edge.v1_idx = static_cast<uint32_t>(from_it->second);
    edge.v2_idx = static_cast<uint32_t>(to_it->second);
    edge.edge_type = rmf_building_map_msgs::msg::GraphEdge::EDGE_TYPE_BIDIRECTIONAL;
    graph_msg.edges.push_back(edge);
  }

  RCLCPP_DEBUG(
    nav_graph_store_logger(),
    "Built filtered graph '%s': vertices=%zu edges=%zu (from %zu zone(s))",
    graph_name.c_str(),
    graph_msg.vertices.size(),
    graph_msg.edges.size(),
    zone_ids.size());

  return graph_msg;
}

// Create one SPHERE_LIST marker for all waypoints belonging to a single zone.
visualization_msgs::msg::Marker NavGraphStore::make_zone_waypoints_marker(
  int marker_id,
  const std::string & ns,
  const Zone & zone,
  const std_msgs::msg::ColorRGBA & color,
  const std::string & frame_id,
  const builtin_interfaces::msg::Time & stamp) const
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = stamp;
  marker.ns = ns;
  marker.id = marker_id;
  marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = 0.22;
  marker.scale.y = 0.22;
  marker.scale.z = 0.22;
  marker.color = color;
  marker.pose.orientation.w = 1.0;

  // Place one sphere at each valid zone waypoint vertex.
  for (int wp_idx : zone.waypoints) {
    if (wp_idx < 0 || wp_idx >= static_cast<int>(vertices_.size())) {
      continue;
    }
    const auto & v = vertices_[static_cast<std::size_t>(wp_idx)];
    geometry_msgs::msg::Point p;
    p.x = v.x;
    p.y = v.y;
    p.z = 0.15;
    marker.points.push_back(p);
  }
  return marker;
}

// Build RViz markers for each robot's current zone, next zones, lanes, and label text.
visualization_msgs::msg::MarkerArray NavGraphStore::build_zone_markers(
  const std::vector<ZoneMarkerView> & views,
  const std::string & frame_id,
  const builtin_interfaces::msg::Time & stamp) const
{
  const FunctionTimer timer("build_zone_markers");

  visualization_msgs::msg::MarkerArray markers;

  // First marker clears all previous markers in RViz for this display.
  visualization_msgs::msg::Marker delete_all;
  delete_all.header.stamp = stamp;
  delete_all.header.frame_id = frame_id;
  delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
  markers.markers.push_back(delete_all);

  if (!loaded_ || views.empty()) {
    RCLCPP_DEBUG(
      nav_graph_store_logger(),
      "Skipping zone markers: loaded=%s view_count=%zu",
      loaded_ ? "true" : "false",
      views.size());
    return markers;
  }

  // Small helper to build RGBA colors used by waypoint and lane markers.
  auto make_color = [](float r, float g, float b, float a = 0.95f) {
    std_msgs::msg::ColorRGBA c;
    c.r = r;
    c.g = g;
    c.b = b;
    c.a = a;
    return c;
  };

  // Color palette: current zone + up to 3 next zones + lane lines.
  const std_msgs::msg::ColorRGBA current_color = make_color(0.1f, 0.95f, 0.2f);
  const std_msgs::msg::ColorRGBA next_colors[3] = {
    make_color(1.0f, 0.55f, 0.0f),
    make_color(1.0f, 0.85f, 0.1f),
    make_color(0.2f, 0.75f, 1.0f),
  };
  const std_msgs::msg::ColorRGBA lanes_color = make_color(0.1f, 0.3f, 1.0f, 0.85f);

  int robot_index = 0;
  // Draw a separate marker group for each robot view.
  for (const auto & view : views) {
    // Reserve non-overlapping marker IDs per robot.
    const int base_id = robot_index * 20;
    std::unordered_set<std::string> robot_zones;
    if (!view.current_zone.empty()) {
      robot_zones.insert(view.current_zone);
    }
    for (const auto & z : view.next_zones) {
      if (!z.empty()) {
        robot_zones.insert(z);
      }
    }

    // Green spheres for the robot's current zone waypoints.
    if (!view.current_zone.empty()) {
      const auto zone_it = zones_by_id_.find(view.current_zone);
      if (zone_it != zones_by_id_.end()) {
        auto wp = make_zone_waypoints_marker(
          base_id, "zone_current_waypoints", zone_it->second, current_color, frame_id, stamp);
        if (!wp.points.empty()) {
          markers.markers.push_back(std::move(wp));
        }
      }
    }

    // Orange/yellow/light-blue spheres for up to three upcoming zones.
    for (std::size_t i = 0; i < view.next_zones.size() && i < 3; ++i) {
      const std::string & next_id = view.next_zones[i];
      if (next_id.empty()) {
        continue;
      }
      const auto zone_it = zones_by_id_.find(next_id);
      if (zone_it == zones_by_id_.end()) {
        continue;
      }
      auto wp = make_zone_waypoints_marker(
        base_id + 1 + static_cast<int>(i), "zone_next_waypoints", zone_it->second, next_colors[i],
        frame_id, stamp);
      if (!wp.points.empty()) {
        markers.markers.push_back(std::move(wp));
      }
    }

    // Collect vertex indices used to draw lane lines for this robot's zones.
    std::unordered_set<int> allowed_indices;
    for (std::size_t idx = 0; idx < vertices_.size(); ++idx) {
      for (const auto & label : vertices_[idx].zone_labels) {
        if (robot_zones.count(label) > 0) {
          allowed_indices.insert(static_cast<int>(idx));
          break;
        }
      }
    }
    for (const auto & zone_id : robot_zones) {
      const auto zone_it = zones_by_id_.find(zone_id);
      if (zone_it == zones_by_id_.end()) {
        continue;
      }
      for (int idx : zone_it->second.waypoints) {
        if (idx >= 0 && idx < static_cast<int>(vertices_.size())) {
          allowed_indices.insert(idx);
        }
      }
      for (int idx : zone_it->second.edge) {
        if (idx >= 0 && idx < static_cast<int>(vertices_.size())) {
          allowed_indices.insert(idx);
        }
      }
    }

    // Blue line pairs for lanes fully contained in the robot's active zones.
    visualization_msgs::msg::Marker lanes_marker;
    lanes_marker.header.frame_id = frame_id;
    lanes_marker.header.stamp = stamp;
    lanes_marker.ns = "zone_lanes";
    lanes_marker.id = base_id + 10;
    lanes_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    lanes_marker.action = visualization_msgs::msg::Marker::ADD;
    lanes_marker.scale.x = 0.08;
    lanes_marker.color = lanes_color;
    lanes_marker.pose.orientation.w = 1.0;

    for (const auto & lane : lanes_) {
      if (allowed_indices.count(lane.from_vertex) == 0 ||
        allowed_indices.count(lane.to_vertex) == 0)
      {
        continue;
      }
      if (lane.from_vertex < 0 || lane.to_vertex < 0 ||
        lane.from_vertex >= static_cast<int>(vertices_.size()) ||
        lane.to_vertex >= static_cast<int>(vertices_.size()))
      {
        continue;
      }
      geometry_msgs::msg::Point p1;
      p1.x = vertices_[static_cast<std::size_t>(lane.from_vertex)].x;
      p1.y = vertices_[static_cast<std::size_t>(lane.from_vertex)].y;
      p1.z = 0.02;
      geometry_msgs::msg::Point p2;
      p2.x = vertices_[static_cast<std::size_t>(lane.to_vertex)].x;
      p2.y = vertices_[static_cast<std::size_t>(lane.to_vertex)].y;
      p2.z = 0.02;
      lanes_marker.points.push_back(p1);
      lanes_marker.points.push_back(p2);
    }
    if (!lanes_marker.points.empty()) {
      markers.markers.push_back(std::move(lanes_marker));
    }

    // White floating text above the robot showing id, current zone, and next zones.
    visualization_msgs::msg::Marker label;
    label.header.frame_id = frame_id;
    label.header.stamp = stamp;
    label.ns = "zone_labels";
    label.id = base_id + 11;
    label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    label.action = visualization_msgs::msg::Marker::ADD;
    label.pose.position.x = view.x;
    label.pose.position.y = view.y;
    label.pose.position.z = 1.0;
    label.pose.orientation.w = 1.0;
    label.scale.z = 0.45;
    label.color = make_color(1.0f, 1.0f, 1.0f, 1.0f);

    std::ostringstream text;
    text << view.robot_id << "\ncurrent: "
         << (view.current_zone.empty() ? "-" : view.current_zone) << "\nnext:";
    if (view.next_zones.empty()) {
      text << " -";
    } else {
      for (std::size_t i = 0; i < view.next_zones.size(); ++i) {
        text << (i == 0 ? " " : ", ") << view.next_zones[i];
      }
    }
    label.text = text.str();
    markers.markers.push_back(std::move(label));

    ++robot_index;
  }

  RCLCPP_DEBUG(
    nav_graph_store_logger(),
    "Built zone markers: marker_count=%zu robot_view_count=%zu frame='%s'",
    markers.markers.size(),
    views.size(),
    frame_id.c_str());

  return markers;
}

}  // namespace monitor
