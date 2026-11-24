#include "traffic_manager/release_markers.hpp"

#include <algorithm>
#include <cmath>
#include <tuple>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace traffic_manager
{
namespace
{

using Point3 = std::tuple<double, double, double>;

std::string marker_frame_for_path(
  const std::vector<rmf_fleet_msgs::msg::Location> & path,
  const std::string & default_frame_id,
  const bool use_level_name_as_frame)
{
  if (!use_level_name_as_frame || path.empty()) {
    return default_frame_id;
  }
  const auto level = path.front().level_name;
  if (level.empty()) {
    return default_frame_id;
  }
  return level;
}

visualization_msgs::msg::Marker make_delete_all(
  const builtin_interfaces::msg::Time & stamp,
  const std::string & frame_id)
{
  visualization_msgs::msg::Marker m;
  m.header.frame_id = frame_id;
  m.header.stamp = stamp;
  m.ns = "";
  m.id = 0;
  m.action = visualization_msgs::msg::Marker::DELETEALL;
  return m;
}

visualization_msgs::msg::Marker make_line_strip(
  const builtin_interfaces::msg::Time & stamp,
  const std::string & frame_id,
  const std::string & ns,
  const int32_t marker_id,
  const std::vector<Point3> & points,
  const float r,
  const float g,
  const float b,
  const float width)
{
  visualization_msgs::msg::Marker m;
  m.header.frame_id = frame_id;
  m.header.stamp = stamp;
  m.ns = ns;
  m.id = marker_id;
  m.type = visualization_msgs::msg::Marker::LINE_STRIP;
  m.action = visualization_msgs::msg::Marker::ADD;
  m.scale.x = width;
  m.color.r = r;
  m.color.g = g;
  m.color.b = b;
  m.color.a = 0.95f;
  m.points.reserve(points.size());
  for (const auto & pt : points) {
    geometry_msgs::msg::Point p;
    p.x = std::get<0>(pt);
    p.y = std::get<1>(pt);
    p.z = std::get<2>(pt);
    m.points.push_back(p);
  }
  return m;
}

visualization_msgs::msg::Marker make_sphere(
  const builtin_interfaces::msg::Time & stamp,
  const std::string & frame_id,
  const std::string & ns,
  const int32_t marker_id,
  const Point3 & point,
  const float r,
  const float g,
  const float b,
  const float scale)
{
  visualization_msgs::msg::Marker m;
  m.header.frame_id = frame_id;
  m.header.stamp = stamp;
  m.ns = ns;
  m.id = marker_id;
  m.type = visualization_msgs::msg::Marker::SPHERE;
  m.action = visualization_msgs::msg::Marker::ADD;
  m.pose.position.x = std::get<0>(point);
  m.pose.position.y = std::get<1>(point);
  m.pose.position.z = std::get<2>(point);
  m.scale.x = scale;
  m.scale.y = scale;
  m.scale.z = scale;
  m.color.r = r;
  m.color.g = g;
  m.color.b = b;
  m.color.a = 0.95f;
  return m;
}

bool index_is_released(const int32_t index, const int32_t release_through_index)
{
  return release_through_index >= 0 && index <= release_through_index;
}

}  // namespace

void append_path_release_markers(
  visualization_msgs::msg::MarkerArray & array,
  const builtin_interfaces::msg::Time & stamp,
  const std::string & robot_name,
  const std::vector<rmf_fleet_msgs::msg::Location> & path,
  const int32_t release_through_index,
  const int32_t progress_from_index,
  const int32_t hold_index,
  const std::string & default_frame_id,
  const bool use_level_name_as_frame)
{
  if (path.empty()) {
    return;
  }

  const std::string frame =
    marker_frame_for_path(path, default_frame_id, use_level_name_as_frame);

  // Drop residual / already-passed waypoints so RViz clears behind the robot.
  const int32_t start_i = std::max(
    0, progress_from_index >= 0 ? progress_from_index : 0);
  if (static_cast<std::size_t>(start_i) >= path.size()) {
    return;
  }

  std::vector<Point3> released_pts;
  std::vector<Point3> held_pts;
  released_pts.reserve(path.size() - static_cast<std::size_t>(start_i));
  held_pts.reserve(path.size() - static_cast<std::size_t>(start_i));

  for (std::size_t i = static_cast<std::size_t>(start_i); i < path.size(); ++i) {
    const Point3 pt(path[i].x, path[i].y, 0.05);
    if (index_is_released(static_cast<int32_t>(i), release_through_index)) {
      // Yellow: matches PathRequest Location.release = true
      released_pts.push_back(pt);
    } else {
      // Blue: matches PathRequest Location.release = false
      held_pts.push_back(pt);
    }
  }

  int32_t marker_id = 1;
  const std::string line_released_ns = "path_line_released_" + robot_name;
  const std::string line_held_ns = "path_line_held_" + robot_name;
  const std::string wp_ns = "path_wp_" + robot_name;
  const std::string hold_ns = "path_hold_" + robot_name;

  if (released_pts.size() >= 2) {
    array.markers.push_back(make_line_strip(
        stamp, frame, line_released_ns, marker_id, released_pts,
        1.0f, 1.0f, 0.0f, 0.12f));
    ++marker_id;
  } else if (released_pts.size() == 1) {
    array.markers.push_back(make_sphere(
        stamp, frame, line_released_ns, marker_id, released_pts.front(),
        1.0f, 1.0f, 0.0f, 0.25f));
    ++marker_id;
  }

  if (held_pts.size() >= 2) {
    std::vector<Point3> held_line;
    if (!released_pts.empty()) {
      held_line.push_back(released_pts.back());
    }
    held_line.insert(held_line.end(), held_pts.begin(), held_pts.end());
    array.markers.push_back(make_line_strip(
        stamp, frame, line_held_ns, marker_id, held_line,
        0.1f, 0.35f, 1.0f, 0.08f));
    ++marker_id;
  } else if (!held_pts.empty()) {
    array.markers.push_back(make_sphere(
        stamp, frame, line_held_ns, marker_id, held_pts.front(),
        0.1f, 0.35f, 1.0f, 0.2f));
    ++marker_id;
  }

  for (std::size_t i = static_cast<std::size_t>(start_i); i < path.size(); ++i) {
    const bool released =
      index_is_released(static_cast<int32_t>(i), release_through_index);
    visualization_msgs::msg::Marker m;
    m.header.frame_id = frame;
    m.header.stamp = stamp;
    m.ns = wp_ns;
    // Stable id by path index so DELETEALL + redraw does not leave ghosts.
    m.id = static_cast<int32_t>(i);
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.position.x = path[i].x;
    m.pose.position.y = path[i].y;
    m.pose.position.z = 0.08;
    m.scale.x = 0.18f;
    m.scale.y = 0.18f;
    m.scale.z = 0.18f;
    if (released) {
      m.color.r = 1.0f;
      m.color.g = 1.0f;
      m.color.b = 0.0f;
      m.color.a = 0.9f;
    } else {
      m.color.r = 0.1f;
      m.color.g = 0.35f;
      m.color.b = 1.0f;
      m.color.a = 0.9f;
    }
    array.markers.push_back(m);
  }

  // Red: traffic hold / WAIT gate (last released node the robot must stop at).
  if (hold_index >= start_i &&
    static_cast<std::size_t>(hold_index) < path.size())
  {
    const Point3 hold_pt(
      path[static_cast<std::size_t>(hold_index)].x,
      path[static_cast<std::size_t>(hold_index)].y,
      0.12);
    array.markers.push_back(make_sphere(
        stamp, frame, hold_ns, 1, hold_pt,
        1.0f, 0.05f, 0.05f, 0.32f));
  }
}

visualization_msgs::msg::MarkerArray make_path_release_marker_array(
  const builtin_interfaces::msg::Time & stamp,
  const std::string & robot_name,
  const std::vector<rmf_fleet_msgs::msg::Location> & path,
  const int32_t release_through_index,
  const int32_t progress_from_index,
  const int32_t hold_index,
  const std::string & default_frame_id,
  const bool use_level_name_as_frame)
{
  visualization_msgs::msg::MarkerArray array;
  if (path.empty()) {
    return array;
  }
  const std::string frame =
    marker_frame_for_path(path, default_frame_id, use_level_name_as_frame);
  array.markers.push_back(make_delete_all(stamp, frame));
  append_path_release_markers(
    array, stamp, robot_name, path, release_through_index,
    progress_from_index, hold_index,
    default_frame_id, use_level_name_as_frame);
  return array;
}

}  // namespace traffic_manager
