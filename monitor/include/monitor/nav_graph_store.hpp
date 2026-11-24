#ifndef MONITOR__NAV_GRAPH_STORE_HPP_
#define MONITOR__NAV_GRAPH_STORE_HPP_

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "rmf_building_map_msgs/msg/graph.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace monitor
{

/// Loads nav-graph YAML (vertices, lanes, zones) and builds filtered Graph msgs.
class NavGraphStore
{
public:
  struct Vertex
  {
    double x{0.0};
    double y{0.0};
    std::string name;
    std::vector<std::string> zone_labels;
  };

  struct Lane
  {
    int from_vertex{-1};
    int to_vertex{-1};
  };

  struct Zone
  {
    std::string id;
    std::string name;
    std::vector<int> waypoints;
    std::vector<int> edge;
  };

  /// Per-robot view used for RViz zone markers (current + next ≤3).
  struct ZoneMarkerView
  {
    std::string robot_id;
    double x{0.0};
    double y{0.0};
    std::string current_zone;
    std::vector<std::string> next_zones;
  };

  bool load_map(const std::string & yaml_path, const std::string & level = "L1");

  bool loaded() const { return loaded_; }

  /// Vertices/lanes belonging to the given zone IDs (current + next-N).
  rmf_building_map_msgs::msg::Graph build_filtered_graph(
    const std::unordered_set<std::string> & zone_ids,
    const std::string & graph_name) const;

  /// Waypoints, lanes, and labels for current + next zones (RViz).
  visualization_msgs::msg::MarkerArray build_zone_markers(
    const std::vector<ZoneMarkerView> & views,
    const std::string & frame_id,
    const builtin_interfaces::msg::Time & stamp) const;

  const std::vector<Vertex> & vertices() const { return vertices_; }
  const std::vector<Lane> & lanes() const { return lanes_; }
  const std::unordered_map<std::string, Zone> & zones() const { return zones_by_id_; }

private:
  visualization_msgs::msg::Marker make_zone_waypoints_marker(
    int marker_id,
    const std::string & ns,
    const Zone & zone,
    const std_msgs::msg::ColorRGBA & color,
    const std::string & frame_id,
    const builtin_interfaces::msg::Time & stamp) const;

  std::vector<Vertex> vertices_;
  std::vector<Lane> lanes_;
  std::unordered_map<std::string, Zone> zones_by_id_;
  bool loaded_{false};
};

}  // namespace monitor

#endif  // MONITOR__NAV_GRAPH_STORE_HPP_
