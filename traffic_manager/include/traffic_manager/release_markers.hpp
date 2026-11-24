#ifndef TRAFFIC_MANAGER__RELEASE_MARKERS_HPP_
#define TRAFFIC_MANAGER__RELEASE_MARKERS_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "rmf_fleet_msgs/msg/location.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace traffic_manager
{

/// RViz path markers aligned with PathRequest Location.release:
/// - yellow: release=true  (indices progress_from_index .. release_through_index)
/// - blue:   release=false (indices after release_through_index)
/// - red:    optional traffic hold waypoint (WAIT gate)
/// Markers already passed by the robot (before progress_from_index) are omitted
/// so residual trail segments clear as the robot advances.
void append_path_release_markers(
  visualization_msgs::msg::MarkerArray & array,
  const builtin_interfaces::msg::Time & stamp,
  const std::string & robot_name,
  const std::vector<rmf_fleet_msgs::msg::Location> & path,
  int32_t release_through_index,
  int32_t progress_from_index,
  int32_t hold_index,
  const std::string & default_frame_id,
  bool use_level_name_as_frame);

visualization_msgs::msg::MarkerArray make_path_release_marker_array(
  const builtin_interfaces::msg::Time & stamp,
  const std::string & robot_name,
  const std::vector<rmf_fleet_msgs::msg::Location> & path,
  int32_t release_through_index,
  int32_t progress_from_index,
  int32_t hold_index,
  const std::string & default_frame_id,
  bool use_level_name_as_frame);

}  // namespace traffic_manager

#endif  // TRAFFIC_MANAGER__RELEASE_MARKERS_HPP_
