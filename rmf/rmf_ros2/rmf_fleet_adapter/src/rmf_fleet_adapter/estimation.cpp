/*
 * Copyright (C) 2020 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include "estimation.hpp"

#include <rmf_traffic_ros2/Time.hpp>

#include <rclcpp/clock.hpp>
#include <rclcpp/logging.hpp>

namespace {
/// Max distance (m) from the plan's final waypoint to accept phase/task completion.
constexpr double kPathFinishToleranceM = 0.6;
/// Beyond this distance, log an error when the driver reports an empty path.
constexpr double kPathFinishErrorDistanceM = 2.0;
/// Max distance (m) at which the robot may be merged onto a plan waypoint that
/// has no lane connecting it to the previous one. Beyond this the merge would
/// be a teleport, so the raw position is reported instead.
constexpr double kMergeOntoWaypointDistanceM = 2.0;
}  // namespace

//==============================================================================
void check_path_finish(
  rclcpp::Node* node,
  const rmf_fleet_msgs::msg::RobotState& state,
  TravelInfo& info)
{
  // The robot believes it has reached the end of its path.
  //
  // waypoints CAN be empty here. follow_new_path() assigns whatever it is given
  // straight into info.waypoints and then installs path_finished_callback, with
  // no check that the plan actually contained any waypoints. update_state()
  // only tests the callback, so an empty plan followed by a robot reporting an
  // empty path lands here — and vector::back() on an empty vector is undefined
  // behaviour, which on this build means a segfault that takes the whole fleet
  // adapter down with no diagnostic at all.
  if (info.waypoints.empty())
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "Robot [%s] reported an empty path while following a plan that has no "
      "waypoints. Treating the leg as finished instead of dereferencing an "
      "empty path.",
      info.robot_name.c_str());

    if (info.path_finished_callback)
    {
      info.path_finished_callback();
      info.path_finished_callback = nullptr;
    }
    info.next_arrival_estimator = nullptr;
    return;
  }

  // A leg whose base has not been released all the way to its destination
  // can never be finished, however close the robot is to the released base
  // end. The driver reports an empty path at every base boundary (the next
  // waypoint is still unreleased), so without this guard reaching a base
  // end is indistinguishable from reaching the destination — observed
  // 2026-10-21: amr2's patrol task completed at (22.83, 34.34), 17.7 m short
  // of its (22.83, 16.65) destination, because waypoints had been truncated
  // to the released prefix and its back() was the base end.
  if (!info.leg_fully_released)
  {
    RCLCPP_WARN_THROTTLE(
      node->get_logger(),
      *node->get_clock(),
      5000,
      "Robot [%s] reported an empty path but its leg is not fully released "
      "to the destination yet (waiting for base extension); not completing "
      "the phase.",
      info.robot_name.c_str());
    estimate_state(node, state.location, info);
    return;
  }

  // Judge completion against the full-leg destination, not the (possibly
  // truncated) released prefix.
  const auto& wp = info.full_leg_destination.has_value() ?
    *info.full_leg_destination : info.waypoints.back();
  const auto& l = state.location;
  const Eigen::Vector2d p{l.x, l.y};
  const double dist = (p - wp.position().block<2, 1>(0, 0)).norm();

  if (wp.graph_index().has_value())
  {
    info.last_known_wp = *wp.graph_index();

    if (dist > kPathFinishErrorDistanceM)
    {
      RCLCPP_ERROR(
        node->get_logger(),
        "Robot named [%s] belonging to fleet [%s] is very far [%fm] from where "
        "it is supposed to be, but its remaining path is empty. This means the "
        "robot believes it is finished, but it is not where it's supposed to be.",
        info.robot_name.c_str(), info.fleet_name.c_str(), dist);
      estimate_state(node, state.location, info);
      return;
    }

    if (dist > kPathFinishToleranceM)
    {
      RCLCPP_WARN_THROTTLE(
        node->get_logger(),
        *node->get_clock(),
        5000,
        "Robot [%s] reported path finished but is [%fm] from goal "
        "(tolerance=%.2fm); waiting until within range before completing phase.",
        info.robot_name.c_str(), dist, kPathFinishToleranceM);
      estimate_state(node, state.location, info);
      return;
    }

    // Within tolerance: snap to the goal waypoint and complete the phase.
    info.updater->update_position(*wp.graph_index(), l.yaw);
  }
  else
  {
    estimate_state(node, state.location, info);
    return;
  }
  assert(info.waypoints.size() >= 2);

  assert(info.path_finished_callback);
  info.path_finished_callback();
  info.path_finished_callback = nullptr;
  info.next_arrival_estimator = nullptr;
}

//==============================================================================
void estimate_path_traveling(
  rclcpp::Node* node,
  const rmf_fleet_msgs::msg::RobotState& state,
  TravelInfo& info)
{
  assert(!state.path.empty());

  const std::size_t remaining_count = state.path.size();

  // Both operands are unsigned, so if the robot reports MORE remaining points
  // than the plan has waypoints this subtraction wraps to ~1.8e19 rather than
  // going negative. That happens in a very ordinary situation: a replan
  // installs a short new plan here while the robot is still driving — and
  // still reporting — the longer previous path. The wrapped index then reaches
  // waypoints.at() (throws, terminating the process) and waypoints[i-1] below,
  // which is an unchecked read at a garbage offset.
  //
  // The robot being ahead of, or behind, our idea of the plan is not fatal; it
  // just means we cannot say which waypoint it is heading for yet. Fall back to
  // estimating from the raw position and let the next state update settle it.
  if (remaining_count > info.waypoints.size())
  {
    // Normal for a few ticks after every release extension: throttled.
    RCLCPP_WARN_THROTTLE(
      node->get_logger(), *node->get_clock(), 5000,
      "Robot [%s] reports [%zu] remaining path points but its current plan has "
      "only [%zu] waypoints — the robot is still following a previous path. "
      "Estimating from position until the two agree.",
      info.robot_name.c_str(), remaining_count, info.waypoints.size());
    return estimate_state(node, state.location, info);
  }

  const std::size_t i_target_wp = info.waypoints.size() - remaining_count;
  if (i_target_wp >= info.waypoints.size())
  {
    // remaining_count == 0 with a non-empty state.path is not reachable, but
    // index-before-use is cheap and this function is on the crash path.
    RCLCPP_WARN(
      node->get_logger(),
      "Robot [%s] target waypoint index [%zu] is out of range for [%zu] "
      "waypoints; estimating from position instead.",
      info.robot_name.c_str(), i_target_wp, info.waypoints.size());
    return estimate_state(node, state.location, info);
  }

  info.target_plan_index = i_target_wp;
  const auto& target_wp = info.waypoints.at(i_target_wp);

  const auto& l = state.location;
  const auto& p = target_wp.position();
  const auto interp = rmf_traffic::agv::Interpolate::positions(
    *info.traits, std::chrono::steady_clock::now(), {{l.x, l.y, l.yaw}, p});
  const auto next_arrival = interp.back().time() - interp.front().time();
  const auto now = rmf_traffic_ros2::convert(node->now());
  if (target_wp.time() < now + next_arrival)
  {
    assert(info.next_arrival_estimator);
    // It seems the robot cannot arrive on time, so we report the earliest that
    // the robot can make it to its next target waypoint.
    info.next_arrival_estimator(i_target_wp, next_arrival);
  }
  else
  {
    assert(info.next_arrival_estimator);
    // It seems the robot will arrive on time, so we'll report that.
    info.next_arrival_estimator(i_target_wp, target_wp.time() - now);
  }

  rmf_utils::optional<std::size_t> lane_start;
  if (i_target_wp > 1)
  {
    lane_start = info.waypoints[i_target_wp-1].graph_index();
    if (lane_start)
      info.last_known_wp = *lane_start;

    return estimate_midlane_state(l, lane_start, i_target_wp, info);
  }

  estimate_state(node, state.location, info);
}

//==============================================================================
void estimate_midlane_state(
  const rmf_fleet_msgs::msg::Location& l,
  rmf_utils::optional<std::size_t> lane_start,
  const std::size_t next_index,
  TravelInfo& info)
{
  assert(0 < next_index && next_index < info.waypoints.size());
  const auto& target_wp = info.waypoints.at(next_index);
  if (!lane_start && info.last_known_wp)
  {
    // Let's see if the current position is reasonably between the last known
    // waypoint and the target waypoint.
    const auto& base_wp = info.graph->get_waypoint(*info.last_known_wp);
    const Eigen::Vector2d p0 = base_wp.get_location();
    const Eigen::Vector2d p1 = target_wp.position().block<2, 1>(0, 0);
    const Eigen::Vector2d p_location{l.x, l.y};

    const double lane_length = (p1 - p0).norm();

    if (lane_length > 1e-8)
    {
      const Eigen::Vector2d pn = (p1 - p0) / lane_length;
      const Eigen::Vector2d p_l = p_location - p0;
      const double p_l_projection = p_l.dot(pn);
      const double lane_dist = (p_l - p_l_projection*pn).norm();

      if (0.0 <= p_l_projection && p_l_projection <= lane_length
        && lane_dist < 2.0)
      {
        lane_start = *info.last_known_wp;
      }
    }
  }

  const std::optional<std::size_t> target_gi = [&]()
    -> std::optional<std::size_t>
    {
      // At least one future waypoint must have a graph index
      if (target_wp.graph_index())
        return *target_wp.graph_index();

      for (std::size_t i = next_index+1; i < info.waypoints.size(); ++i)
      {
        const auto gi = info.waypoints[i].graph_index();
        if (gi)
          return *gi;
      }

      return std::nullopt;
    } ();

  if (target_gi.has_value())
  {
    if (lane_start.has_value())
    {
      const auto last_gi = *lane_start;
      if (last_gi == target_gi)
      {
        // This implies that the robot is either waiting at or rotating on the
        // waypoint.
        return info.updater->update_position(*target_gi, l.yaw);
      }
      else if (const auto* forward_lane =
        info.graph->lane_from(last_gi, *target_gi))
      {
        // This implies that the robot is moving down a lane.
        std::vector<std::size_t> lanes;
        lanes.push_back(forward_lane->index());

        const auto* reverse_lane = info.graph->lane_from(*target_gi, last_gi);
        if (reverse_lane)
        {
          if (!reverse_lane->entry().event())
          {
            // We don't allow the robot to turn back mid-lane if the reverse lane
            // has an entry event, because if that entry event is docking, then it
            // needs to be triggered for the robot to approach the exit.
            //
            // TODO(MXG): This restriction isn't needed for reversing on door or
            // lift events, so with some effort we could loosen this restriction
            // to only apply to docking.
            lanes.push_back(reverse_lane->index());
          }
        }

        return info.updater->update_position({l.x, l.y, l.yaw},
            std::move(lanes));
      }
    }

    // Merging onto target_gi tells RMF the robot is on its way to that waypoint
    // as a graph position, and GoToPlace will treat reaching it as arrival. That
    // is only honest if the robot is actually somewhere near it.
    //
    // We land here when no lane connects lane_start to target_gi. For a normal
    // plan that means a short hop between adjacent waypoints. But a sparse plan
    // — RMF sometimes emits a route that keeps only the start, one intermediate
    // point and the goal — puts two unconnected waypoints tens of metres apart,
    // and merging then teleports the robot onto the goal.
    //
    // Observed 2026-08-21, amr2 on task 8cb52941: a 3-point plan whose middle
    // point was 19 m from its end. The execution-side sweeper popped the middle
    // point at 0.738 m, estimate_path_traveling() concluded the final leg had
    // begun, and this line merged the robot onto Conveyor6. The task completed
    // with the robot 19.3 m away, having never driven there. check_path_finish()
    // — and its distance tolerance — was bypassed entirely.
    //
    // So merge only when the robot is close enough for that claim to hold.
    // Otherwise report the raw position and let the robot actually get there;
    // the next update re-evaluates, and arrival is then decided by
    // check_path_finish() against its tolerance, which is where that decision
    // belongs.
    // graph_index() values come from the plan, so they index this graph; the
    // check keeps a malformed plan from throwing out of get_waypoint().
    if (*target_gi < info.graph->num_waypoints())
    {
      const Eigen::Vector2d target_p =
        info.graph->get_waypoint(*target_gi).get_location();
      const double dist_to_target =
        (Eigen::Vector2d{l.x, l.y} - target_p).norm();

      if (dist_to_target > kMergeOntoWaypointDistanceM)
      {
        // estimate_midlane_state() has no node handle, so throttle against a
        // function-local steady clock rather than the node's.
        static rclcpp::Clock merge_warn_clock(RCL_STEADY_TIME);
        RCLCPP_WARN_THROTTLE(
          rclcpp::get_logger("rmf_fleet_adapter.estimation"),
          merge_warn_clock, 5000,
          "Robot [%s] is [%.2fm] from waypoint [%lu] and no lane connects it to "
          "the previous waypoint; reporting its raw position instead of merging "
          "it onto that waypoint. This usually means the plan skipped "
          "intermediate waypoints.",
          info.robot_name.c_str(), dist_to_target,
          static_cast<unsigned long>(*target_gi));

        info.updater->update_position(l.level_name, {l.x, l.y, l.yaw});
        return;
      }
    }

    return info.updater->update_position({l.x, l.y, l.yaw}, *target_gi);
  }

  info.updater->update_position(l.level_name, {l.x, l.y, l.yaw});
}

//==============================================================================
void estimate_state(
  rclcpp::Node* node,
  const rmf_fleet_msgs::msg::Location& l,
  TravelInfo& info)
{
  std::string last_known_map = l.level_name;
  if (info.last_known_wp)
  {
    const auto& last_known_wp = info.graph->get_waypoint(*info.last_known_wp);
    if (last_known_map.empty())
      last_known_map = last_known_wp.get_map_name();
  }

  if (last_known_map.empty() && l.level_name.empty())
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "Robot named [%s] belonging to fleet [%s] is lost because we cannot "
      "figure out what floor it is on. Please publish the robot's current "
      "floor name in the level_name field of its RobotState.",
      info.robot_name.c_str(), info.fleet_name.c_str());
    return;
  }

  // Always use the robot's current position for planning, not snapped to waypoint.
  // Use small merge distances to force planning from actual position.
  const double max_merge_wp_dist = 0.01;   // meters - very small to prevent snapping
  const double max_merge_lane_dist = 1.0;  // meters - allow lane merges
  const double min_lane_length = 0.05;     // meters

  info.updater->update_position(
    last_known_map, {l.x, l.y, l.yaw},
    max_merge_wp_dist, max_merge_lane_dist, min_lane_length);
}

//==============================================================================
void estimate_waypoint(
  rclcpp::Node* node,
  const rmf_fleet_msgs::msg::Location& l,
  TravelInfo& info)
{
  std::string last_known_map = l.level_name;
  if (last_known_map.empty() && info.last_known_wp)
  {
    last_known_map = info.graph->get_waypoint(*info.last_known_wp)
      .get_map_name();
  }

  const Eigen::Vector2d p(l.x, l.y);
  const rmf_traffic::agv::Graph::Waypoint* closest_wp = nullptr;
  double nearest_dist = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < info.graph->num_waypoints(); ++i)
  {
    const auto& wp = info.graph->get_waypoint(i);
    const Eigen::Vector2d p_wp = wp.get_location();
    const double dist = (p - p_wp).norm();
    if (dist < nearest_dist)
    {
      closest_wp = &wp;
      nearest_dist = dist;
    }
  }

  assert(closest_wp);

  if (nearest_dist > 0.5)
  {
    RCLCPP_WARN(
      node->get_logger(),
      "Robot named [%s] belonging to fleet [%s] is expected to be on a "
      "waypoint, but the nearest waypoint is [%fm] away.",
      info.robot_name.c_str(), info.fleet_name.c_str(), nearest_dist);
  }

  info.updater->update_position(closest_wp->index(), l.yaw);
}
