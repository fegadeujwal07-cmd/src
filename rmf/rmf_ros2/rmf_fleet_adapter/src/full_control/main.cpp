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
#include "../rmf_fleet_adapter/ParseArgs.hpp"

// Internal implementation-specific headers
#include <chrono>
#include <string>
#include <vector>
#include "../rmf_fleet_adapter/load_param.hpp"

#include <vector>

// Public rmf_fleet_adapter API headers
#include <rmf_fleet_adapter/agv/Adapter.hpp>
#include <rmf_fleet_adapter/agv/parse_graph.hpp>

// Standard topic names for communicating with fleet drivers
#include <rmf_fleet_adapter/StandardNames.hpp>

// Fleet driver state/command messages
// added for ECBS — EcbsReplan msg (REPLAN/CLEAR/NO_ALTERNATIVE from evaluate_path_server).
#include <ecbs_planner/msg/ecbs_replan.hpp>
#include <rmf_fleet_msgs/msg/fleet_state.hpp>
#include <rmf_fleet_msgs/msg/path_request.hpp>
#include <rmf_fleet_msgs/msg/mode_request.hpp>
#include <rmf_fleet_msgs/msg/lane_request.hpp>
#include <rmf_fleet_msgs/msg/robot_lane_request.hpp>
#include <rmf_fleet_msgs/msg/closed_lanes.hpp>
#include <rmf_fleet_msgs/msg/speed_limit_request.hpp>
#include <rmf_fleet_msgs/msg/interrupt_request.hpp>

// Zone-manager conflict events (startPause / stopPause per robot)
#include <zone_manager/msg/conflict.hpp>

// Path gate: intent → traffic_manager → release before PathRequest
#include <traffic_manager/msg/path_intent.hpp>
#include <traffic_manager/msg/path_release.hpp>

// RMF Task messages
#include <rmf_task_msgs/msg/task_type.hpp>
#include <rmf_task_msgs/msg/task_profile.hpp>

// ROS2 utilities for rmf_traffic
#include <rmf_traffic_ros2/Time.hpp>

// Utility functions for estimating where a robot is on the graph based on
// the information provided by fleet drivers.
#include "../rmf_fleet_adapter/estimation.hpp"

// Public rmf_task API headers
#include <rmf_task/requests/ChargeBatteryFactory.hpp>
#include <rmf_task/requests/ParkRobotFactory.hpp>

// Public rmf_traffic API headers
#include <rmf_traffic/agv/Interpolate.hpp>
#include <rmf_traffic/Route.hpp>

#include <rmf_battery/agv/BatterySystem.hpp>
#include <rmf_battery/agv/SimpleMotionPowerSink.hpp>
#include <rmf_battery/agv/SimpleDevicePowerSink.hpp>

#include <Eigen/Geometry>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <limits>
#include <mutex>
#include <unordered_map>
#include <unordered_set>
#include <optional>
#include <vector>

//==============================================================================
struct DistanceFromGraph
{
  double value;
  std::size_t index;

  enum Type
  {
    Waypoint = 0,
    Lane
  };
  Type type;
};

//==============================================================================
std::optional<DistanceFromGraph> distance_from_graph(
  const rmf_fleet_msgs::msg::Location& l,
  const rmf_traffic::agv::Graph& graph)
{
  std::optional<DistanceFromGraph> output;
  const Eigen::Vector2d p = {l.x, l.y};
  const std::string& map = l.level_name;

  for (std::size_t i = 0; i < graph.num_waypoints(); ++i)
  {
    const auto& wp = graph.get_waypoint(i);
    if (wp.get_map_name() != map)
      continue;

    const double dist = (wp.get_location() - p).norm();
    if (!output.has_value() || (dist < output->value))
    {
      output = DistanceFromGraph{dist, i, DistanceFromGraph::Waypoint};
    }
  }

  for (std::size_t i = 0; i < graph.num_lanes(); ++i)
  {
    const auto& lane = graph.get_lane(i);
    const auto& wp0 = graph.get_waypoint(lane.entry().waypoint_index());
    const auto& wp1 = graph.get_waypoint(lane.exit().waypoint_index());

    if (map != wp0.get_map_name() && map != wp1.get_map_name())
      continue;

    const Eigen::Vector2d p0 = wp0.get_location();
    const Eigen::Vector2d p1 = wp1.get_location();

    const Eigen::Vector2d dp = p - p0;
    const Eigen::Vector2d dp1 = p1 - p0;

    const double lane_length = dp1.norm();
    if (lane_length < 1e-8)
      continue;

    const double u = dp.dot(dp1)/lane_length;
    if (u < 0 || lane_length < u)
      continue;

    const double dist = (dp - u*dp1/lane_length).norm();
    if (!output.has_value() || (dist < output->value))
    {
      output = DistanceFromGraph{dist, i, DistanceFromGraph::Lane};
    }
  }

  return output;
}

//==============================================================================
/// Fleet-wide live release=true bases. Order publish may only set release=true
/// on points that are not still live-true for another robot (G5/E1).
struct FleetLiveBaseRegistry
{
  struct Point { double x{0.0}; double y{0.0}; };
  mutable std::mutex mutex;
  std::unordered_map<std::string, std::vector<Point>> by_robot;

  void clear(const std::string& robot)
  {
    std::lock_guard<std::mutex> lock(mutex);
    by_robot.erase(robot);
  }

  void clear_robots(const std::vector<std::string>& robots)
  {
    std::lock_guard<std::mutex> lock(mutex);
    for (const auto& robot : robots)
    {
      by_robot.erase(robot);
    }
  }

  /// Drop vacated prefix as the robot advances — PathRequest publish alone
  /// left a frozen corridor footprint that blocked peers after ECBS FREE.
  void shrink_to_pose(
    const std::string& robot,
    double pose_x,
    double pose_y)
  {
    std::lock_guard<std::mutex> lock(mutex);
    auto it = by_robot.find(robot);
    if (it == by_robot.end() || it->second.empty())
    {
      return;
    }
    auto& live = it->second;
    std::size_t start = 0;
    double best = 1e100;
    for (std::size_t i = 0; i < live.size(); ++i)
    {
      const double d = std::hypot(live[i].x - pose_x, live[i].y - pose_y);
      if (d < best)
      {
        best = d;
        start = i;
      }
    }
    if (start > 0)
    {
      live.erase(
        live.begin(),
        live.begin() + static_cast<std::ptrdiff_t>(start));
    }
  }

  void update(
    const std::string& robot,
    const rmf_fleet_msgs::msg::PathRequest& req,
    double pose_x,
    double pose_y)
  {
    std::vector<Point> live;
    int32_t tip = -1;
    for (std::size_t i = 0; i < req.path.size(); ++i)
    {
      if (req.path[i].release)
      {
        tip = static_cast<int32_t>(i);
      }
    }
    if (tip < 0)
    {
      std::lock_guard<std::mutex> lock(mutex);
      by_robot.erase(robot);
      return;
    }
    int32_t start = 0;
    double best = 1e100;
    for (int32_t i = 0; i <= tip; ++i)
    {
      const auto& wp = req.path[static_cast<std::size_t>(i)];
      const double d = std::hypot(wp.x - pose_x, wp.y - pose_y);
      if (d < best)
      {
        best = d;
        start = i;
      }
    }
    live.reserve(static_cast<std::size_t>(tip - start + 1));
    for (int32_t i = start; i <= tip; ++i)
    {
      const auto& wp = req.path[static_cast<std::size_t>(i)];
      live.push_back(Point{wp.x, wp.y});
    }
    std::lock_guard<std::mutex> lock(mutex);
    by_robot[robot] = std::move(live);
  }

  int32_t clamp_through(
    const std::string& self,
    const std::vector<rmf_fleet_msgs::msg::Location>& path,
    int32_t wanted,
    int32_t prior_tip,
    double eps_m = 1.0) const
  {
    if (wanted < 0 || path.empty())
    {
      return wanted;
    }
    const int32_t max_i = static_cast<int32_t>(path.size()) - 1;
    wanted = std::min(wanted, max_i);
    const int32_t grant_start = prior_tip + 1;
    if (grant_start > wanted)
    {
      return wanted;
    }
    std::lock_guard<std::mutex> lock(mutex);
    auto hits = [&](int32_t through) -> bool
      {
        for (int32_t wi = grant_start; wi <= through; ++wi)
        {
          const auto& wp = path[static_cast<std::size_t>(wi)];
          for (const auto& kv : by_robot)
          {
            if (kv.first == self)
            {
              continue;
            }
            for (const auto& p : kv.second)
            {
              if (std::hypot(wp.x - p.x, wp.y - p.y) <= eps_m)
              {
                return true;
              }
            }
          }
        }
        return false;
      };
    while (wanted >= grant_start && hits(wanted))
    {
      --wanted;
    }
    if (wanted < grant_start)
    {
      return prior_tip;
    }
    return wanted;
  }
};

//==============================================================================
class FleetDriverRobotCommandHandle
  : public rmf_fleet_adapter::agv::RobotCommandHandle,
  public std::enable_shared_from_this<FleetDriverRobotCommandHandle>
{
public:

  using PathRequestPub =
    rclcpp::Publisher<rmf_fleet_msgs::msg::PathRequest>::SharedPtr;

  using ModeRequestPub =
    rclcpp::Publisher<rmf_fleet_msgs::msg::ModeRequest>::SharedPtr;

  using PathIntentPub =
    rclcpp::Publisher<traffic_manager::msg::PathIntent>::SharedPtr;
  
    using ActionExecution =
    rmf_fleet_adapter::agv::RobotUpdateHandle::ActionExecution;

  FleetDriverRobotCommandHandle(
    rclcpp::Node& node,
    std::string fleet_name,
    std::string robot_name,
    std::shared_ptr<const rmf_traffic::agv::Graph> graph,
    std::shared_ptr<const rmf_traffic::agv::VehicleTraits> traits,
    PathRequestPub path_request_pub,
    ModeRequestPub mode_request_pub,
    PathIntentPub path_intent_pub,
    std::shared_ptr<std::atomic<uint64_t>> path_request_id_gen,
    std::shared_ptr<FleetLiveBaseRegistry> live_bases)
  : _node(&node),
    _path_request_pub(std::move(path_request_pub)),
    _mode_request_pub(std::move(mode_request_pub)),
    _path_intent_pub(std::move(path_intent_pub)),
    _path_request_id_gen(std::move(path_request_id_gen)),
    _live_bases(std::move(live_bases))
  {
    RCLCPP_INFO(
      _node->get_logger(),
      "Creating FleetDriverRobotCommandHandle for robot [%s] in fleet [%s]",
      robot_name.c_str(), fleet_name.c_str());

    _current_path_request.fleet_name = fleet_name;
    _current_path_request.robot_name = robot_name;

    _current_dock_request.fleet_name = fleet_name;
    _current_dock_request.robot_name = robot_name;
    _current_dock_request.mode.mode = _current_dock_request.mode.MODE_DOCKING;

    rmf_fleet_msgs::msg::ModeParameter p;
    p.name = "docking";
    _current_dock_request.parameters.push_back(std::move(p));

    _travel_info.graph = std::move(graph);
    _travel_info.traits = std::move(traits);
    _travel_info.fleet_name = std::move(fleet_name);
    _travel_info.robot_name = std::move(robot_name);

    RCLCPP_INFO(
      _node->get_logger(),
      "FleetDriverRobotCommandHandle created successfully for robot [%s]",
      _travel_info.robot_name.c_str());
  }

  void follow_new_path(
    const std::vector<rmf_traffic::agv::Plan::Waypoint>& waypoints,
    ArrivalEstimator next_arrival_estimator,
    RequestCompleted path_finished_callback,
    const std::string& dispatch_task_id = "",
    uint32_t phase_index = 0,
    uint32_t path_revision = 0) final
  {
    RCLCPP_INFO(
      _node->get_logger(),
      "Robot [%s] starting to follow new path with [%zu] waypoints",
      _travel_info.robot_name.c_str(), waypoints.size());

    auto lock = _lock();
    // Hold only blocks mid-leg negotiated replans. A new dispatch task (cancel
    // + award) or an explicitly armed ZM/TM replan must always replace the
    // PathIntent — otherwise the old opposite/corridor session never clears
    // and both this robot and its peer stay stuck.
    const bool new_dispatch_task =
      !dispatch_task_id.empty() &&
      dispatch_task_id != _current_dispatch_task_id;
    if (_tm_traffic_hold_active && !_explicit_replan_armed && !new_dispatch_task)
    {
      RCLCPP_WARN(
        _node->get_logger(),
        "[TrafficGate] Robot [%s] ignoring follow_new_path (%zu wps, rev=%u) — "
        "TM traffic hold active; replan only when TM/ZM explicitly arms it",
        _travel_info.robot_name.c_str(),
        waypoints.size(),
        path_revision);
      return;
    }
    if (new_dispatch_task && _tm_traffic_hold_active)
    {
      RCLCPP_INFO(
        _node->get_logger(),
        "[TrafficGate] Robot [%s] accepting new dispatch task [%s] despite TM "
        "hold (clears stale opposite/corridor session)",
        _travel_info.robot_name.c_str(),
        dispatch_task_id.c_str());
    }
    _explicit_replan_armed = false;
    _tm_traffic_hold_active = false;
    _clear_last_command();

    // An empty plan must never be installed alongside a path_finished_callback.
    // update_state() decides what to do purely from that callback, so an empty
    // waypoint list would route the next empty-path report into
    // check_path_finish(), which reads waypoints.back() — undefined behaviour on
    // an empty vector, and a silent segfault of the whole fleet adapter.
    //
    // There is nothing to drive to, so report the leg complete immediately and
    // leave the previous waypoints in place rather than installing an empty set.
    if (waypoints.empty())
    {
      RCLCPP_WARN(
        _node->get_logger(),
        "Robot [%s] was given a plan with no waypoints; reporting the leg as "
        "finished instead of installing an empty path.",
        _travel_info.robot_name.c_str());

      _travel_info.target_plan_index = std::nullopt;
      if (path_finished_callback)
        path_finished_callback();

      return;
    }

    _travel_info.target_plan_index = std::nullopt;
    _full_leg_plan_waypoints = waypoints;
    _travel_info.waypoints = waypoints;
    // The leg starts unreleased: traffic_manager must release the base all
    // the way to waypoints.back() before this leg may ever complete.
    _travel_info.full_leg_destination = waypoints.back();
    _travel_info.leg_fully_released = false;
    _travel_info.next_arrival_estimator = std::move(next_arrival_estimator);
    _travel_info.path_finished_callback = std::move(path_finished_callback);
    _interrupted = false;

    _current_dispatch_task_id = dispatch_task_id;
    _current_phase_index = phase_index;
    _current_path_revision = path_revision;
    _current_path_request.task_id = std::to_string(++_current_task_id);
    _current_path_request.dispatch_task_id = _current_dispatch_task_id;
    _current_path_request.phase_index = _current_phase_index;
    _current_path_request.path_revision = _current_path_revision;
    _current_path_request.path.clear();
    for (const auto& wp : waypoints)
    {
      rmf_fleet_msgs::msg::Location location;
      const Eigen::Vector3d p = wp.position();
      location.t = rmf_traffic_ros2::convert(wp.time());
      location.x = p.x();
      location.y = p.y();
      location.yaw = p.z();

      // The speed limit is set as the minimum of all the approach lanes' limits
      std::optional<double> speed_limit = std::nullopt;
      for (const auto& lane_idx : wp.approach_lanes())
      {
        const auto& lane = _travel_info.graph->get_lane(lane_idx);
        const auto& lane_limit = lane.properties().speed_limit();
        if (lane_limit.has_value())
        {
          if (speed_limit.has_value())
            speed_limit = std::min(speed_limit.value(), lane_limit.value());
          else
            speed_limit = lane_limit.value();
        }
      }
      if (speed_limit.has_value())
      {
        location.obey_approach_speed_limit = true;
        location.approach_speed_limit = speed_limit.value();
      }
      else
      {
        location.obey_approach_speed_limit = false;
      }

      // Note: if the waypoint is not on a graph index, then we'll just leave
      // the level_name blank. That information isn't likely to get used by the
      // fleet driver anyway.
      if (wp.graph_index())
      {
        location.index = *wp.graph_index();
        location.level_name =
          _travel_info.graph->get_waypoint(*wp.graph_index()).get_map_name();
      }

      _current_path_request.path.emplace_back(std::move(location));
    }

    _submit_path_intent();
  }

  void stop() final
  {
    RCLCPP_INFO(
      _node->get_logger(),
      "Robot [%s] stop command requested",
      _travel_info.robot_name.c_str());

    auto lock = _lock();
    // Cancel/stop must always reach traffic_manager so ScheduleCache + conflict
    // session drop this robot; blocking here left peers WAIT forever.
    if (_tm_traffic_hold_active)
    {
      RCLCPP_INFO(
        _node->get_logger(),
        "[TrafficGate] Robot [%s] stop clears TM traffic hold (cancel/replace)",
        _travel_info.robot_name.c_str());
      _tm_traffic_hold_active = false;
      _explicit_replan_armed = false;
    }

    _clear_last_command();

    if (!_last_known_state.has_value())
    {
      RCLCPP_WARN(
        _node->get_logger(),
        "Cannot ask slotcar [%s] to stop because we have never received a "
        "state from it.",
        _travel_info.robot_name.c_str());
      return;
    }

    rmf_fleet_msgs::msg::Location location;
    _current_path_request.task_id = std::to_string(++_current_task_id);
    _current_path_request.dispatch_task_id = _current_dispatch_task_id;
    // _clear_last_command() deliberately leaves the leg counters alone, so a
    // stop marker still reports which leg it interrupted. It stays easy to
    // recognise: one waypoint, and an empty dispatch_task_id.
    _current_path_request.phase_index = _current_phase_index;
    _current_path_request.path_revision = _current_path_revision;
    _current_path_request.path.clear();
    _current_path_request.path.push_back(_last_known_state->location);

    _submit_path_intent();

    RCLCPP_INFO(
      _node->get_logger(),
      "[TrafficGate] Robot [%s] stop path intent submitted (1 waypoint at "
      "current pose); PathRequest withheld until traffic_manager release",
      _travel_info.robot_name.c_str());
   }

  void on_path_release(const traffic_manager::msg::PathRelease& release)
  {
    auto lock = _lock();

    if (release.robot_name != _travel_info.robot_name)
    {
      RCLCPP_DEBUG(
        _node->get_logger(),
        "[TrafficGate] Ignoring PathRelease for robot [%s] (this handle: [%s])",
        release.robot_name.c_str(),
        _travel_info.robot_name.c_str());
      return;
    }

    RCLCPP_INFO(
      _node->get_logger(),
      "[TrafficGate] Robot [%s] PathRelease received: request_id=%lu approved=%s "
      "through_index=%d phase=%u revision=%u task_id=[%s]",
      _travel_info.robot_name.c_str(),
      release.request_id,
      release.approved ? "true" : "false",
      release.release_through_index,
      release.phase_index,
      release.path_revision,
      release.task_id.c_str());

    if (release.request_id != _pending_path_request_id)
    {
      RCLCPP_WARN(
        _node->get_logger(),
        "[TrafficGate] Robot [%s] stale PathRelease request_id=%lu (pending=%lu) — "
        "ignored",
        _travel_info.robot_name.c_str(),
        release.request_id,
        _pending_path_request_id);
      return;
    }

    if (!release.approved || release.release_through_index < 0)
    {
      RCLCPP_WARN(
        _node->get_logger(),
        "[TrafficGate] Robot [%s] path release denied for request_id=%lu "
        "(approved=%s through_index=%d)",
        _travel_info.robot_name.c_str(),
        release.request_id,
        release.approved ? "true" : "false",
        release.release_through_index);
      return;
    }

    // added for ECBS
    if (release.ecbs_status == "WAIT")
    {
      _tm_traffic_hold_active = true;
      _last_wait_blockers = release.blocking_amr_ids;
      RCLCPP_INFO(
        _node->get_logger(),
        "[TrafficGate] Robot [%s] ECBS WAIT — release only through index=%d; "
        "must hold at leg=%d (%.3f, %.3f) up to %.1f s for blocker(s); "
        "unreleased tail withheld from PathRequest/VDA",
        _travel_info.robot_name.c_str(),
        release.release_through_index,
        release.hold_leg_index,
        release.hold_x,
        release.hold_y,
        release.hold_wait_sec);
    }
    else
    {
      // ECBS cleared the wait deps — drop stale blocker footprints so the
      // order interlock cannot keep clamping after TM already said FREE.
      if (_live_bases && !_last_wait_blockers.empty())
      {
        _live_bases->clear_robots(_last_wait_blockers);
      }
      _last_wait_blockers.clear();
      _tm_traffic_hold_active = false;
    }

    _publish_released_path_prefix(release);
  }

  class DockFinder : public rmf_traffic::agv::Graph::Lane::Executor
  {
  public:

    bool is_dock = false;

    DockFinder(const std::string& dock_name)
    : _dock_name(std::move(dock_name))
    {
      // Do nothing
    }

    void execute(const Dock& dock) final
    {
      if (dock.dock_name() == _dock_name)
        is_dock = true;
    }

    void execute(const Wait&) final {}
    void execute(const DoorOpen&) final {}
    void execute(const DoorClose&) final {}
    void execute(const LiftSessionBegin&) final {}
    void execute(const LiftMove&) final {}
    void execute(const LiftDoorOpen&) final {}
    void execute(const LiftSessionEnd&) final {}

  private:
    const std::string& _dock_name;
  };

  void dock(
    const std::string& dock_name,
    RequestCompleted docking_finished_callback) final
  {
    RCLCPP_INFO(
      _node->get_logger(),
      "Robot [%s] docking request to dock [%s]",
      _travel_info.robot_name.c_str(), dock_name.c_str());

    auto lock = _lock();
    _clear_last_command();

    _dock_finished_callback = std::move(docking_finished_callback);
    _current_dock_request.parameters.front().value = dock_name;
    _current_dock_request.task_id = std::to_string(++_current_task_id);

    _dock_requested_time = std::chrono::steady_clock::now();
    _mode_request_pub->publish(_current_dock_request);

    RCLCPP_INFO(
      _node->get_logger(),
      "Robot [%s] dock request published with task_id [%s]",
      _travel_info.robot_name.c_str(), _current_dock_request.task_id.c_str());

    // TODO(MXG): We should come up with a better way to identify the docking
    // lanes.
    _dock_target_wp = rmf_utils::nullopt;
    DockFinder finder(dock_name);
    for (std::size_t i = 0; i < _travel_info.graph->num_lanes(); ++i)
    {
      const auto& lane = _travel_info.graph->get_lane(i);
      const auto entry_event = lane.entry().event();
      if (entry_event)
      {
        entry_event->execute(finder);
        if (finder.is_dock)
        {
          _dock_target_wp = lane.entry().waypoint_index();
          break;
        }
      }
    }

    assert(_dock_target_wp);

    const auto& wp = _travel_info.graph->get_waypoint(*_dock_target_wp);
    const std::string wp_name = wp.name() ?
      *wp.name() : std::to_string(wp.index());

    RCLCPP_INFO(
      _node->get_logger(),
      "Requesting robot [%s] of [%s] to dock into waypoint [%s]",
      _current_path_request.robot_name.c_str(),
      _current_path_request.fleet_name.c_str(),
      wp_name.c_str());
  }

  void update_state(const rmf_fleet_msgs::msg::RobotState& state)
  {
    RCLCPP_DEBUG(
      _node->get_logger(),
      "Robot [%s] state update - mode: [%d], battery: [%.1f]%%, location: [%.2f, %.2f, %.2f], task_id: [%s]",
      state.name.c_str(), state.mode.mode, state.battery_percent,
      state.location.x, state.location.y, state.location.yaw,
      state.task_id.c_str());

    auto lock = _lock();
    _last_known_state = state;

    // Keep peer-visible live base in sync with pose; idle drops it entirely.
    if (_live_bases)
    {
      if (state.mode.mode == state.mode.MODE_IDLE ||
        state.mode.mode == state.mode.MODE_CHARGING)
      {
        _live_bases->clear(_travel_info.robot_name);
      }
      else
      {
        _live_bases->shrink_to_pose(
          _travel_info.robot_name,
          state.location.x,
          state.location.y);
      }
    }

    // Update battery soc
    const double battery_soc = state.battery_percent / 100.0;
    if (battery_soc >= 0.0 && battery_soc <= 1.0)
    {
      _travel_info.updater->update_battery_soc(battery_soc);
      RCLCPP_DEBUG(
        _node->get_logger(),
        "Robot [%s] battery SOC updated to [%.3f]",
        _travel_info.robot_name.c_str(), battery_soc);
    }
    else
    {
      RCLCPP_ERROR(
        _node->get_logger(),
        "Battery percentage reported by the robot is outside of the valid "
        "range [0,100] and hence the battery soc will not be updated. It is "
        "critical to update the battery soc with a valid battery percentage "
        "for task allocation planning.");
    }

    // Charging signal from the robot (powerSupply.charging via execution):
    // 1 = charging, 0 = not charging, -1 = unknown. A charge task waits for
    // this to go 1 -> 0 to complete.
    _travel_info.updater->update_charging_status(state.charging_status);

    // Reset these each time. They will get filled in by the estimation
    // functions as necessary.
    _travel_info.target_plan_index = std::nullopt;

    if (_travel_info.path_finished_callback)
    {
      // If we have a path_finished_callback, then the robot should be
      // following a path. This callback is set once per phase (e.g., pickup phase,
      // drop phase) when RMF commands a new path segment. It will be cleared when
      // that phase completes, so the path finish check happens ONCE per phase.

      // There should not be a docking command happening
      assert(!_dock_finished_callback);

      // The arrival estimator should be available
      assert(_travel_info.next_arrival_estimator);

      if (state.task_id != _current_path_request.task_id)
      {
        RCLCPP_INFO(
          _node->get_logger(),
          "Robot [%s] task_id mismatch - expected: [%s], received: [%s]",
          _travel_info.robot_name.c_str(),
          _current_path_request.task_id.c_str(),
          state.task_id.c_str());

        // The robot has not received our path request yet
        const auto now = std::chrono::steady_clock::now();
        if (std::chrono::milliseconds(200) < now - _path_requested_time)
        {
          if (_path_released_for_pending)
          {
            RCLCPP_WARN(
              _node->get_logger(),
              "[TrafficGate] Robot [%s] task_id mismatch — republishing last "
              "PathRequest ([%zu] waypoints, request_id=%lu)",
              _travel_info.robot_name.c_str(),
              _last_published_path_request.path.size(),
              _pending_path_request_id);
            _path_requested_time = now;
            _path_request_pub->publish(_last_published_path_request);
          }
          else
          {
            RCLCPP_WARN(
              _node->get_logger(),
              "[TrafficGate] Robot [%s] task_id mismatch — no release yet; "
              "resubmitting PathIntent to traffic_manager (request_id=%lu)",
              _travel_info.robot_name.c_str(),
              _pending_path_request_id);
            _path_requested_time = now;
            _submit_path_intent(/* resubmit_pending= */ true);
          }
        }

        return estimate_state(_node, state.location, _travel_info);
      }

      if (state.mode.mode == state.mode.MODE_ADAPTER_ERROR)
      {
        if (_interrupted)
        {
          RCLCPP_INFO(
            _node->get_logger(),
            "Robot [%s] already interrupted, skipping replan",
            _travel_info.robot_name.c_str());
          // This interruption was already noticed
          return;
        }

        RCLCPP_INFO(
          _node->get_logger(),
          "Fleet driver [%s] reported a need to replan for [%s]",
          _current_path_request.fleet_name.c_str(),
          _current_path_request.robot_name.c_str());

        _interrupted = true;
        estimate_state(_node, state.location, _travel_info);

        // ---- STRICT ZONE_MANAGER REPLAN (disabled, not removed) -----------
        // MODE_ADAPTER_ERROR from the fleet driver used to trigger a replan
        // here. updater->replan() is request_replan(), which bumps
        // path_revision — and a bumped revision is what
        // is_replan_of_published_order() treats as a genuine replan: it
        // bypasses the coords gate, sends a cancelOrder and publishes a new
        // VDA5050 order. Indistinguishable from a zone_manager replan at the
        // point of use, so it is filtered here at the source.
        //
        // The robot is still marked _interrupted and its state is still
        // estimated; only the replan is withheld. A driver-reported error now
        // needs an operator to cancel the task.
        //
        // Re-enable by uncommenting the return below.
        RCLCPP_WARN(
          _node->get_logger(),
          "Robot [%s] reported MODE_ADAPTER_ERROR. A replan would normally be "
          "requested here, but replans are restricted to zone_manager — no new "
          "path will be planned for this robot.",
          _travel_info.robot_name.c_str());
        // return _travel_info.updater->replan();
        return;
      }

      if (state.path.empty())
      {
        // Mid-horizon empty path: robot finished the released prefix (or lost
        // it after a retract) while the full leg is still open. Republish the
        // remaining released order from the stored path + live pose so the
        // task does not stall waiting for a base extension that never arrives.
        if (!_travel_info.leg_fully_released &&
          _path_released_for_pending &&
          !_last_published_path_request.path.empty())
        {
          const auto now = std::chrono::steady_clock::now();
          // A robot still reporting MOVING is driving its released order; the
          // empty path is the driver's report, not a stall. Republishing then
          // only floods execution with partial orders it ignores (every 0.5 s).
          const bool driving =
            state.mode.mode == state.mode.MODE_MOVING;
          if (!driving && std::chrono::milliseconds(500) < now - _path_requested_time)
          {
            rmf_fleet_msgs::msg::PathRequest remaining =
              _last_published_path_request;
            std::size_t nearest = 0;
            double best_d = std::numeric_limits<double>::infinity();
            std::size_t last_released = 0;
            for (std::size_t i = 0; i < remaining.path.size(); ++i)
      {
              if (!remaining.path[i].release)
              {
                break;
              }
              last_released = i;
              const double d = std::hypot(
                remaining.path[i].x - state.location.x,
                remaining.path[i].y - state.location.y);
              if (d < best_d)
              {
                best_d = d;
                nearest = i;
              }
            }
            if (nearest < last_released)
            {
              remaining.path.assign(
                _last_published_path_request.path.begin() +
                static_cast<std::ptrdiff_t>(nearest),
                _last_published_path_request.path.begin() +
                static_cast<std::ptrdiff_t>(last_released + 1));
              // Live pose as the first node so VDA/execution do not send the
              // robot backwards to an already-passed waypoint.
              remaining.path.front().x = state.location.x;
              remaining.path.front().y = state.location.y;
              remaining.path.front().yaw = state.location.yaw;
              remaining.path.front().release = true;
              ++remaining.path_revision;
              _path_requested_time = now;
              _path_request_pub->publish(remaining);
              RCLCPP_WARN(
                _node->get_logger(),
                "[TrafficGate] Robot [%s] empty path mid-leg — republished "
                "remaining released order (%zu wps from live pose, through "
                "released index %zu, rev=%u)",
                _travel_info.robot_name.c_str(),
                remaining.path.size(),
                last_released,
                remaining.path_revision);
            }
            else
            {
              // Already at / past the released base end: nudge TM for the
              // next horizon via PathIntent resubmit (BaseRequests may be
              // stuck after a retract / ack thrash).
              _path_requested_time = now;
              _submit_path_intent(/* resubmit_pending= */ true);
              RCLCPP_WARN(
            _node->get_logger(),
                "[TrafficGate] Robot [%s] empty path at released base end — "
                "resubmitting PathIntent for next horizon (request_id=%lu)",
            _travel_info.robot_name.c_str(),
                _pending_path_request_id);
            }
          }
          return estimate_state(_node, state.location, _travel_info);
        }

        RCLCPP_DEBUG(
          _node->get_logger(),
          "Robot [%s] path is empty, checking if path is finished",
          _travel_info.robot_name.c_str());
        // When the state path is empty, that means the robot believes it has
        // arrived at its destination for the current phase.
        //
        // IMPORTANT: This check happens ONCE per phase. When check_path_finish()
        // determines the path is complete, it will call path_finished_callback()
        // and then clear _travel_info.path_finished_callback. After that, even
        // if the fleet driver continues to publish empty paths, this check will
        // not run again until RMF commands a new path segment (new phase) and
        // sets a new path_finished_callback.
        //
        // For delivery tasks, this means:
        // - Pickup phase: When path becomes empty, check if pickup is done
        // - Drop phase: When path becomes empty, check if drop is done
        // Each phase is checked exactly once when its path first becomes empty.
        return check_path_finish(_node, state, _travel_info);
      }

      RCLCPP_DEBUG(
        _node->get_logger(),
        "Robot [%s] estimating path traveling with [%zu] path points",
        _travel_info.robot_name.c_str(), state.path.size());
      return estimate_path_traveling(_node, state, _travel_info);
    }
    else if (_dock_finished_callback)
    {
      const auto now = std::chrono::steady_clock::now();
      // If we have a _dock_finished_callback, then the robot should be docking
      if (state.task_id != _current_dock_request.task_id)
      {
        RCLCPP_INFO(
          _node->get_logger(),
          "Robot [%s] dock task_id mismatch - expected: [%s], received: [%s]",
          _travel_info.robot_name.c_str(),
          _current_dock_request.task_id.c_str(),
          state.task_id.c_str());

        if (std::chrono::milliseconds(200) < now - _dock_requested_time)
        {
          RCLCPP_INFO(
            _node->get_logger(),
            "Robot [%s] resending dock request due to timeout",
            _travel_info.robot_name.c_str());
          // We published the request a while ago, so we'll send it again in
          // case it got dropped.
          _dock_requested_time = now;
          _mode_request_pub->publish(_current_dock_request);
        }

        return;
      }

      if (state.mode.mode != state.mode.MODE_DOCKING)
      {
        RCLCPP_INFO(
          _node->get_logger(),
          "Robot [%s] dock completed, mode changed from DOCKING to [%d]",
          _travel_info.robot_name.c_str(), state.mode.mode);
        estimate_waypoint(_node, state.location, _travel_info);
        _travel_info.last_known_wp = *_dock_target_wp;
        _dock_finished_callback();
        _dock_finished_callback = nullptr;

        return;
      }

      // Update the schedule with the docking path of the robot
      if (!state.path.empty() &&
        std::chrono::seconds(1) < now - _dock_schedule_time)
      {
        RCLCPP_INFO(
          _node->get_logger(),
          "Robot [%s] updating dock schedule with [%zu] path points",
          _travel_info.robot_name.c_str(), state.path.size());

        std::vector<Eigen::Vector3d> positions;
        positions.push_back(
          {state.location.x, state.location.y, state.location.yaw});
        for (const auto& p : state.path)
          positions.push_back({p.x, p.y, p.yaw});

        const rmf_traffic::Trajectory trajectory =
          rmf_traffic::agv::Interpolate::positions(
          *_travel_info.traits,
          rmf_traffic_ros2::convert(state.location.t),
          positions);

        if (trajectory.size() < 2)
        {
          RCLCPP_INFO(
            _node->get_logger(),
            "Robot [%s] trajectory too small, skipping schedule update",
            _travel_info.robot_name.c_str());
          return;
        }

        if (auto participant =
          _travel_info.updater->unstable().get_participant())
        {
          participant->set(
            participant->assign_plan_id(),
            {rmf_traffic::Route{state.location.level_name, trajectory}});
          _dock_schedule_time = now;
          RCLCPP_INFO(
            _node->get_logger(),
            "Robot [%s] dock schedule updated successfully",
            _travel_info.robot_name.c_str());
        }
      }
    }
    else
    {
      RCLCPP_DEBUG(
        _node->get_logger(),
        "Robot [%s] not under command, estimating state only",
        _travel_info.robot_name.c_str());
      // Session end for live-base peers: no active path command → drop
      // footprint so waiters are not clamped on vacated cells.
      if (_live_bases)
      {
        _live_bases->clear(_travel_info.robot_name);
      }
      // If we don't have a finishing callback, then the robot is not under our
      // command. This means no path finish check will occur, even if state.path
      // is empty. The path finish check only happens when path_finished_callback
      // is set (i.e., when a phase is actively being tracked).
      estimate_state(_node, state.location, _travel_info);
    }
  }

  void set_updater(rmf_fleet_adapter::agv::RobotUpdateHandlePtr updater)
  {
    _travel_info.updater = std::move(updater);

    bool enable_responsive_wait = true;
    _node->get_parameter_or(
      "enable_responsive_wait", enable_responsive_wait, true);
    _travel_info.updater->enable_responsive_wait(enable_responsive_wait);

    // Set the action_executor for the robot
    const auto teleop_executioner =
      [w = weak_from_this()](
      const std::string&,
      const nlohmann::json&,
      ActionExecution execution)
      {
        // We do not do anything here. The user can can move the robot by
        // sending PathRequest msgs. Instead we simply store the completed
        // callback which will be called when we receive a RobotModeRequest.
        const auto self = w.lock();
        if (!self)
          return;
        self->set_action_execution(execution);
      };
    _travel_info.updater->set_action_executor(teleop_executioner);
  }

  rmf_fleet_adapter::agv::RobotUpdateHandlePtr get_updater()
  {
    return _travel_info.updater;
  }

  void newly_closed_lanes(const std::unordered_set<std::size_t>& closed_lanes)
  {
    bool need_to_replan = false;

    if (_travel_info.target_plan_index.has_value())
    {
      const auto& target_wp =
        _travel_info.waypoints.at(*_travel_info.target_plan_index);

      const auto& current_lanes = target_wp.approach_lanes();
      for (const auto& l : current_lanes)
      {
        if (closed_lanes.count(l))
        {
          need_to_replan = true;
          const auto& lane = _travel_info.graph->get_lane(l);
          const auto& loc = _last_known_state->location;
          const Eigen::Vector2d p = {loc.x, loc.y};

          const auto& wp0 =
            _travel_info.graph->get_waypoint(lane.entry().waypoint_index());
          const Eigen::Vector2d p0 = wp0.get_location();

          const auto& wp1 =
            _travel_info.graph->get_waypoint(lane.exit().waypoint_index());
          const Eigen::Vector2d p1 = wp1.get_location();

          const bool before_blocked_lane = (p-p0).dot(p1-p0) < 0.0;
          const bool after_blocked_lane = (p-p1).dot(p1-p0) >= 0.0;
          if (!before_blocked_lane && !after_blocked_lane)
          {
            // The robot is currently on a lane that has been closed. We take
            // this to mean that the robot needs to reverse.
            const Eigen::Vector3d position = {p.x(), p.y(), loc.yaw};
            const auto& return_waypoint = wp0.index();
            const auto* reverse_lane =
              _travel_info.graph->lane_from(wp1.index(), wp0.index());

            if (reverse_lane)
            {
              // We know what lane will reverse us back to the beginning of our
              // current lane, so we will update our position by saying that we
              // are on that lane.
              std::vector<std::size_t> lanes;
              lanes.push_back(reverse_lane->index());
              _travel_info.updater->update_position(position, lanes);
            }
            else
            {
              // There isn't an explicit lane for getting back to the beginning of
              // our current lane, so we will update with only our current
              // position and the waypoint index that we intend to return to.
              _travel_info.updater->update_position(position, return_waypoint);
            }
          }
        }
      }
    }

    if (!need_to_replan && _travel_info.target_plan_index.has_value())
    {
      // Check if the remainder of the current plan has been invalidated by the
      // lane closure.
      const auto next_index = *_travel_info.target_plan_index;
      for (std::size_t i = next_index; i < _travel_info.waypoints.size(); ++i)
      {
        for (const auto& lane : _travel_info.waypoints[i].approach_lanes())
        {
          if (closed_lanes.count(lane))
          {
            need_to_replan = true;
            break;
          }
        }

        if (need_to_replan)
          break;
      }
    }

    if (need_to_replan)
      _travel_info.updater->replan();
  }

  void set_action_execution(ActionExecution action_execution)
  {
    _action_execution = action_execution;
  }

  void complete_robot_action()
  {
    if (!_action_execution.has_value())
      return;

    _action_execution->finished();
    _action_execution = std::nullopt;

    RCLCPP_INFO(
      _node->get_logger(),
      "Robot [%s] has completed the action it was performing",
      _travel_info.robot_name.c_str());
  }

  void handle_interrupt_request(
    const rmf_fleet_msgs::msg::InterruptRequest& request)
  {
    const auto it = _interruptions.find(request.interrupt_id);
    if (it == _interruptions.end())
    {
      if (request.type == request.TYPE_RESUME)
        return;

      _interruptions.insert(
        {
          request.interrupt_id,
          _travel_info.updater->interrupt(
            request.labels,
            [id = request.interrupt_id, name = _travel_info.robot_name]()
            {
              std::cout << "[" << name << "] is interrupted for " << id
                        << "!" << std::endl;
            })
        });

      return;
    }
    else
    {
      if (request.type == request.TYPE_INTERRUPT)
        return;

      it->second.resume(request.labels);
      std::cout << "Asking [" << _travel_info.robot_name << "] to resume for "
                << request.interrupt_id << std::endl;

      _interruptions.erase(it);
    }
  }

  // ---------------------------------------------------------------------
  // Zone-manager hold
  //
  // The zone manager stops a robot by publishing a VDA5050 startPause
  // instantAction straight to the vehicle. That path never touches RMF, so
  // without this hook the robot stands still while its itinerary keeps
  // claiming the whole corridor ahead of it — including the ground another
  // robot needs in order to move out of the way. The result is a deadlock
  // that neither side can see: the paused robot waits for the other to move,
  // and the other cannot get a plan because the paused robot still owns the
  // corridor.
  //
  // declare_holding() overrides the schedule with "parked at this pose",
  // which frees the corridor. We deliberately do NOT use
  // RobotUpdateHandle::interrupt() here: it calls command->stop() (which
  // would fight the VDA5050 order already sitting on the robot) and its
  // Resume handler is hard-wired to _find_plan(), i.e. resuming an
  // interruption always replans. The robot already has its full path and
  // simply continues once the obstruction clears, so a replan is exactly
  // what we want to avoid.
  //
  // CAVEAT: RobotUpdateHandle.hpp warns that declare_holding should not be
  // used while a task with automatic schedule updating is running. The
  // running task may overwrite the hold, which is why refresh_zone_hold()
  // re-asserts it on a timer for as long as the pause lasts.
  //
  // Because declare_holding() REPLACES the itinerary (Participant::set drops
  // every previous route), the robot's remaining path vanishes from the
  // schedule — which is precisely what frees the corridor, but it also means
  // RViz stops drawing the robot's path. Nothing puts the route back on its
  // own: the running task only reports progress and delay against an
  // itinerary it assumes is still there. So we snapshot the routes before the
  // first hold and re-assert them, shifted forward by the time spent held,
  // when the hold is released. That is a re-assert, not a replan: no path is
  // recomputed and no new order is sent to the robot.
  // ---------------------------------------------------------------------
  void set_zone_hold(bool hold, const std::string& zone_id)
  {
    auto lock = _lock();
    if (hold == _zone_hold_active)
      return;

    _zone_hold_active = hold;

    if (hold)
    {
      // Snapshot BEFORE the first declare_holding() lands. declare_holding()
      // marshals onto the worker thread, so reading here — synchronously, on
      // the caller's thread — is guaranteed to see the pre-hold itinerary.
      if (_travel_info.updater)
      {
        _zone_hold_saved_itinerary =
          _travel_info.updater->unstable().current_itinerary();
        _zone_hold_saved_checkpoints =
          _travel_info.updater->unstable().reached_checkpoints();
      }
      else
      {
        _zone_hold_saved_itinerary.clear();
        _zone_hold_saved_checkpoints.clear();
      }
      _zone_hold_started_at = rmf_traffic_ros2::convert(_node->now());
      // Start the refresh bookkeeping from scratch so a previous hold's
      // timestamp cannot make refresh_zone_hold() skip the first declaration.
      _zone_hold_declared_at = std::nullopt;
      _zone_hold_plan_id = std::nullopt;
      _zone_hold_preserve_itinerary = false;

      RCLCPP_INFO(
        _node->get_logger(),
        "Robot [%s] held by zone conflict [%s]; releasing its traffic-schedule "
        "reservation (saved %lu route(s) to restore) so other robots can plan "
        "through the corridor.",
        _travel_info.robot_name.c_str(), zone_id.c_str(),
        _zone_hold_saved_itinerary.size());
      refresh_zone_hold();
    }
    else
    {
      restore_zone_hold_itinerary(zone_id);
    }
  }

  /// declare_holding() writes a 1-route, 2-waypoint same-pose trajectory.
  static bool is_parking_hold_itinerary(
    const rmf_traffic::schedule::Itinerary& itinerary)
  {
    if (itinerary.size() != 1)
      return false;
    const auto& traj = itinerary.front().trajectory();
    if (traj.size() != 2)
      return false;
    return (traj.front().position() - traj.back().position()).norm() < 1e-3;
  }

  /// Compare path geometry only (ignore timestamps) so a zone replan can be
  /// told apart from the pre-hold snapshot.
  static bool same_itinerary_geometry(
    const rmf_traffic::schedule::Itinerary& a,
    const rmf_traffic::schedule::Itinerary& b)
  {
    if (a.size() != b.size())
      return false;
    for (std::size_t i = 0; i < a.size(); ++i)
    {
      const auto& ta = a[i].trajectory();
      const auto& tb = b[i].trajectory();
      if (ta.size() != tb.size())
        return false;
      for (std::size_t j = 0; j < ta.size(); ++j)
      {
        if ((ta[j].position() - tb[j].position()).head<2>().norm() > 0.05)
          return false;
      }
    }
    return true;
  }

  /// added for ECBS — used by the /ecbs_replan_candidate handler to skip a replan
  /// when the rest of the leg never touches the keeper's closed corridor.
  /// True if what is left of the current leg passes any graph waypoint in
  /// \p wps.
  bool remaining_leg_visits(const std::unordered_set<int64_t>& wps)
  {
    auto lock = _lock();
    const auto& plan = _full_leg_plan_waypoints.empty() ?
      _travel_info.waypoints : _full_leg_plan_waypoints;
    for (std::size_t i = _travel_info.target_plan_index.value_or(0);
      i < plan.size(); ++i)
    {
      const auto g = plan[i].graph_index();
      if (g && wps.count(static_cast<int64_t>(*g)))
        return true;
    }
    return false;
  }

  /// added for ECBS — stop/hold intents are a single pose and never visit
  /// blocked cells, so they must not count as "detour clear, safe to reopen".
  bool remaining_leg_is_trivial()
  {
    auto lock = _lock();
    const auto& plan = _full_leg_plan_waypoints.empty() ?
      _travel_info.waypoints : _full_leg_plan_waypoints;
    const std::size_t start = _travel_info.target_plan_index.value_or(0);
    return plan.size() <= start + 1;
  }

  /// Zone replan is about to replace the parked hold — stop wiping the
  /// schedule on the refresh timer so the new path can land and stay.
  void arm_zone_replan_preserve()
  {
    auto lock = _lock();
    _explicit_replan_armed = true;
    if (!_zone_hold_active)
    {
      RCLCPP_INFO(
        _node->get_logger(),
        "Robot [%s] armed explicit replan (TM traffic hold may be active).",
        _travel_info.robot_name.c_str());
      return;
    }

    _zone_hold_preserve_itinerary = true;
    RCLCPP_INFO(
      _node->get_logger(),
      "Robot [%s] armed schedule-preserve for an upcoming zone replan.",
      _travel_info.robot_name.c_str());
  }

  /// Put the pre-hold routes back into the schedule, shifted forward by the
  /// duration of the hold so their timestamps are in the future again.
  void restore_zone_hold_itinerary(const std::string& zone_id)
  {
    auto lock = _lock();

    // Zone replans often finish within ~10–100 ms, but stopPause can arrive
    // before the 2 s refresh timer ever sees the overwrite. Restoring the
    // pre-hold snapshot then paints the OLD path over the replan — that is
    // exactly the "shows earlier path not replanned path" bug. Prefer whatever
    // real (non-parking) itinerary is already on the schedule.
    if (_travel_info.updater)
    {
      const auto current =
        _travel_info.updater->unstable().current_itinerary();
      if (!current.empty() && !is_parking_hold_itinerary(current))
      {
        if (_zone_hold_saved_itinerary.empty() ||
          !same_itinerary_geometry(current, _zone_hold_saved_itinerary))
        {
          RCLCPP_INFO(
            _node->get_logger(),
            "Robot [%s] released from zone conflict [%s]; keeping the newer "
            "itinerary already on the schedule (%lu route(s), not the pre-hold "
            "snapshot).",
            _travel_info.robot_name.c_str(), zone_id.c_str(),
            current.size());
          _zone_hold_saved_itinerary.clear();
          _zone_hold_saved_checkpoints.clear();
          _zone_hold_started_at = std::nullopt;
          _zone_hold_declared_at = std::nullopt;
          _zone_hold_plan_id = std::nullopt;
          _zone_hold_preserve_itinerary = false;
          return;
        }
      }
    }

    if (_zone_hold_preserve_itinerary)
    {
      RCLCPP_INFO(
        _node->get_logger(),
        "Robot [%s] released from zone conflict [%s]; leaving the replanned "
        "itinerary already on the schedule (%lu route(s)).",
        _travel_info.robot_name.c_str(), zone_id.c_str(),
        _zone_hold_saved_itinerary.size());
      _zone_hold_saved_itinerary.clear();
      _zone_hold_saved_checkpoints.clear();
      _zone_hold_started_at = std::nullopt;
      _zone_hold_declared_at = std::nullopt;
      _zone_hold_plan_id = std::nullopt;
      _zone_hold_preserve_itinerary = false;
      return;
    }

    if (_zone_hold_saved_itinerary.empty() || !_travel_info.updater)
    {
      RCLCPP_INFO(
        _node->get_logger(),
        "Robot [%s] released from zone conflict [%s]; nothing to restore "
        "(no routes were saved when the hold began).",
        _travel_info.robot_name.c_str(), zone_id.c_str());
      _zone_hold_saved_itinerary.clear();
      _zone_hold_saved_checkpoints.clear();
      _zone_hold_started_at = std::nullopt;
      _zone_hold_declared_at = std::nullopt;
      _zone_hold_plan_id = std::nullopt;
      _zone_hold_preserve_itinerary = false;
      return;
    }

    const auto now = rmf_traffic_ros2::convert(_node->now());
    const auto held_for = _zone_hold_started_at.has_value() ?
      now - *_zone_hold_started_at : rmf_traffic::Duration(0);

    // Adjusting the first waypoint shifts every subsequent waypoint with it,
    // so one call per route moves the whole trajectory.
    auto restored = _zone_hold_saved_itinerary;
    for (auto& route : restored)
    {
      auto& trajectory = route.trajectory();
      if (trajectory.size() > 0)
        trajectory.begin()->adjust_times(held_for);
    }

    _travel_info.updater->unstable().declare_itinerary(
      std::move(restored), _zone_hold_saved_checkpoints);

    RCLCPP_INFO(
      _node->get_logger(),
      "Robot [%s] released from zone conflict [%s] after %.1f s; restored "
      "%lu route(s) to the schedule, shifted forward by the hold duration. "
      "It resumes the path it already has — no replan requested.",
      _travel_info.robot_name.c_str(), zone_id.c_str(),
      std::chrono::duration<double>(held_for).count(),
      _zone_hold_saved_itinerary.size());

    _zone_hold_saved_itinerary.clear();
    _zone_hold_saved_checkpoints.clear();
    _zone_hold_started_at = std::nullopt;
    _zone_hold_declared_at = std::nullopt;
    _zone_hold_plan_id = std::nullopt;
    _zone_hold_preserve_itinerary = false;
  }

  /// Re-assert the hold. Called on a timer while the hold is active, because
  /// declare_holding() expires after a fixed duration and because the running
  /// task may push its own itinerary over the top of ours.
  ///
  /// It does NOT re-declare on every tick. declare_holding() assigns a fresh
  /// plan id each time it runs, so blindly calling it at the tick rate churns
  /// a new plan id every couple of seconds for the whole pause. Instead we
  /// re-declare only when there is a reason to:
  ///
  ///   * the current plan id is no longer the one our hold created, meaning
  ///     the running task overwrote us and the corridor is reserved again; or
  ///   * the existing hold is past its half-life and needs extending before
  ///     it expires and leaves the robot with no reservation at all.
  ///
  /// The plan id is learned one tick after declaring, because declare_holding()
  /// is executed on the robot's worker thread and has not necessarily landed by
  /// the time this function returns.
  void refresh_zone_hold()
  {
    auto lock = _lock();
    if (!_zone_hold_active)
      return;

    if (!_travel_info.updater)
      return;

    if (!_last_known_state.has_value())
    {
      RCLCPP_WARN_THROTTLE(
        _node->get_logger(), *_node->get_clock(), 5000,
        "Cannot hold the schedule for robot [%s]: no RobotState received yet.",
        _travel_info.robot_name.c_str());
      return;
    }

    const auto now = rmf_traffic_ros2::convert(_node->now());
    const auto& unstable = _travel_info.updater->unstable();

    // After a zone replan lands, keep that itinerary on the schedule for the
    // rest of the pause. Re-calling declare_holding() would wipe it again and
    // make /schedule_markers flicker (old path → new path → blank).
    if (_zone_hold_preserve_itinerary)
    {
      const auto current = unstable.current_itinerary();
      if (!current.empty())
      {
        if (!_zone_hold_plan_id.has_value() ||
          unstable.current_plan_id() != *_zone_hold_plan_id)
        {
          _zone_hold_saved_itinerary = current;
          _zone_hold_saved_checkpoints = unstable.reached_checkpoints();
          _zone_hold_plan_id = unstable.current_plan_id();
        }
        return;
      }

      _zone_hold_preserve_itinerary = false;
      RCLCPP_INFO(
        _node->get_logger(),
        "Preserved itinerary for held robot [%s] was cleared; "
        "re-asserting parked hold.",
        _travel_info.robot_name.c_str());
    }

    if (_zone_hold_declared_at.has_value())
    {
      const auto& unstable = _travel_info.updater->unstable();

      if (!_zone_hold_plan_id.has_value())
      {
        // First tick after declaring: whatever plan id is current now is the
        // one our hold produced. Adopt it as the baseline for overwrite
        // detection and leave the hold alone.
        _zone_hold_plan_id = unstable.current_plan_id();
        return;
      }

      const bool overwritten =
        unstable.current_plan_id() != *_zone_hold_plan_id;
      const bool expiring =
        (now - *_zone_hold_declared_at) >= kZoneHoldDuration / 2;

      if (!overwritten && !expiring)
        return;

      if (overwritten)
      {
                // Zone replan replaced our parked hold. Keep the new routes on the
        // schedule so RViz can draw them while the robot stays physically
        // paused. Only re-assert declare_holding() if the overwrite left an
        // empty itinerary (task churn with no usable path).
        const auto current = unstable.current_itinerary();
        if (!current.empty())
        {
          _zone_hold_saved_itinerary = current;
          _zone_hold_saved_checkpoints = unstable.reached_checkpoints();
          _zone_hold_plan_id = unstable.current_plan_id();
          _zone_hold_declared_at = now;
          _zone_hold_preserve_itinerary = true;
          RCLCPP_INFO(
            _node->get_logger(),
            "Hold on robot [%s] was overwritten by a newer plan (%lu route(s)); "
            "leaving it on the schedule so schedule markers stay visible.",
            _travel_info.robot_name.c_str(),
            _zone_hold_saved_itinerary.size());
          return;
        }

        RCLCPP_INFO(
          _node->get_logger(),
          "Hold on robot [%s] was overwritten by its running task; "
          "re-asserting it.",
          _travel_info.robot_name.c_str());
      }
    }

    _zone_hold_declared_at = now;
    _zone_hold_plan_id = std::nullopt;
    _zone_hold_preserve_itinerary = false;

    const auto& l = _last_known_state->location;
    _travel_info.updater->unstable().declare_holding(
      l.level_name,
      Eigen::Vector3d{l.x, l.y, l.yaw},
      kZoneHoldDuration);
  }

private:

  /// How long each declare_holding() call reserves the stationary pose for.
  /// Must comfortably exceed the refresh period set up in main().
  static constexpr std::chrono::seconds kZoneHoldDuration{10};

  /// True while the zone manager has this robot paused via startPause.
  bool _zone_hold_active = false;

  /// The routes that were in the schedule when the hold began, put back on
  /// release so the robot's path reappears instead of staying blank.
  rmf_traffic::schedule::Itinerary _zone_hold_saved_itinerary;

  /// Per-route progress at the moment of the hold. Participant::set() resets
  /// progress, so this is replayed alongside the restored routes.
  std::vector<rmf_traffic::CheckpointId> _zone_hold_saved_checkpoints;

  /// When the hold began, used to shift the restored routes forward.
  std::optional<rmf_traffic::Time> _zone_hold_started_at;

  /// When declare_holding() was last issued, so refresh_zone_hold() can extend
  /// the hold before it expires instead of re-issuing it every tick.
  std::optional<rmf_traffic::Time> _zone_hold_declared_at;

  /// The plan id our most recent hold produced, learned one tick after
  /// declaring it. A different current plan id means the running task
  /// overwrote the hold and it has to be re-asserted.
  std::optional<rmf_traffic::PlanId> _zone_hold_plan_id;

  /// True after a non-empty itinerary (typically a zone replan) overwrote the
  /// parked hold. While set, refresh_zone_hold() must not call
  /// declare_holding() again or /schedule_markers lose the new path.
  bool _zone_hold_preserve_itinerary = false;

  /// Set while PathRelease.ecbs_status == WAIT. Blocks same-task negotiated
  /// follow_new_path unless _explicit_replan_armed (ZM lane close / TM).
  /// stop() and a new dispatch_task_id always clear this and submit PathIntent.
  bool _tm_traffic_hold_active = false;
  bool _explicit_replan_armed = false;
  /// Blockers from the last WAIT PathRelease; cleared from live-base registry
  /// when ECBS reports FREE / dependency cleared.
  std::vector<std::string> _last_wait_blockers;


  rclcpp::Node* _node;

  PathRequestPub _path_request_pub;
  PathIntentPub _path_intent_pub;
  std::shared_ptr<std::atomic<uint64_t>> _path_request_id_gen;
  rmf_fleet_msgs::msg::PathRequest _current_path_request;
  rmf_fleet_msgs::msg::PathRequest _last_published_path_request;
  std::vector<rmf_traffic::agv::Plan::Waypoint> _full_leg_plan_waypoints;
  std::chrono::steady_clock::time_point _path_requested_time;
  uint64_t _pending_path_request_id{0};
  bool _path_released_for_pending{false};
    TravelInfo _travel_info;
  std::optional<rmf_fleet_msgs::msg::RobotState> _last_known_state;
  bool _interrupted = false;
  std::shared_ptr<FleetLiveBaseRegistry> _live_bases;

  rmf_fleet_msgs::msg::ModeRequest _current_dock_request;
  rmf_utils::optional<std::size_t> _dock_target_wp;
  std::chrono::steady_clock::time_point _dock_requested_time;
  std::chrono::steady_clock::time_point _dock_schedule_time =
    std::chrono::steady_clock::now();
  RequestCompleted _dock_finished_callback;
  ModeRequestPub _mode_request_pub;

  using Interruption = rmf_fleet_adapter::agv::RobotUpdateHandle::Interruption;
  std::unordered_map<std::string, Interruption> _interruptions;

  uint32_t _current_task_id = 0;
  std::string _current_dispatch_task_id;

  /// Leg index and replan count of the path currently being followed, as
  /// reported by RobotContext via follow_new_path(). Not cleared by
  /// _clear_last_command(), so a stop marker keeps the leg it interrupted.
  uint32_t _current_phase_index = 0;
  uint32_t _current_path_revision = 0;

  std::recursive_mutex _mutex;

  // ActionExecution for managing teleop action
  std::optional<ActionExecution> _action_execution = std::nullopt;

  std::unique_lock<std::recursive_mutex> _lock()
  {
    std::unique_lock<std::recursive_mutex> lock(_mutex, std::defer_lock);
    while (!lock.try_lock())
    {
      // Intentionally busy wait
    }

    return lock;
  }

  void _clear_last_command()
  {
    _travel_info.next_arrival_estimator = nullptr;
    _travel_info.path_finished_callback = nullptr;
    _dock_finished_callback = nullptr;
    _current_dispatch_task_id.clear();
    _current_path_request.dispatch_task_id.clear();
  }

  void _submit_path_intent(const bool resubmit_pending = false)
  {
    if (!_path_intent_pub || !_path_request_id_gen)
    {
      RCLCPP_ERROR(
        _node->get_logger(),
        "[TrafficGate] Robot [%s] cannot submit PathIntent — publisher or "
        "request_id generator missing",
        _travel_info.robot_name.c_str());
      return;
    }

    if (resubmit_pending)
    {
      if (_pending_path_request_id == 0)
      {
        _pending_path_request_id = ++(*_path_request_id_gen);
      }
    }
    else
    {
      _path_released_for_pending = false;
      _pending_path_request_id = ++(*_path_request_id_gen);
    }

    traffic_manager::msg::PathIntent intent;
    intent.stamp = _node->get_clock()->now();
    intent.fleet_name = _current_path_request.fleet_name;
    intent.robot_name = _current_path_request.robot_name;
    intent.request_id = _pending_path_request_id;
    intent.task_id = _current_path_request.task_id;
    intent.dispatch_task_id = _current_path_request.dispatch_task_id;
    intent.phase_index = _current_path_request.phase_index;
    intent.path_revision = _current_path_request.path_revision;
    intent.cursor_index = 0;
    intent.path = _current_path_request.path;

    _path_intent_pub->publish(intent);
    _path_requested_time = std::chrono::steady_clock::now();

    const char* leg_kind =
      _current_path_request.path_revision == 0 ? "initial" : "replan";
    RCLCPP_INFO(
      _node->get_logger(),
      "[TrafficGate] Robot [%s] published PathIntent → /traffic_manager/path_intent: "
      "request_id=%lu waypoints=%zu phase=%u revision=%u (%s%s) task_id=[%s] "
      "dispatch_task_id=[%s] — PathRequest withheld until PathRelease",
      _travel_info.robot_name.c_str(),
      _pending_path_request_id,
      intent.path.size(),
      intent.phase_index,
      intent.path_revision,
      leg_kind,
      resubmit_pending ? ", resubmit" : "",
      intent.task_id.c_str(),
      intent.dispatch_task_id.c_str());
  }

  /// added for ECBS — RMF schedule estimation matches fleet_state remaining
  /// path length to travel_info.waypoints.size(). Keep the full leg for
  /// extensions, but expose only the ECBS-released prefix so estimation stops
  /// at the hold point.
  void _sync_travel_info_waypoints_to_release(const int32_t through)
  {
    if (_full_leg_plan_waypoints.empty()) {
      _full_leg_plan_waypoints = _travel_info.waypoints;
    }
    if (through < 0 || _full_leg_plan_waypoints.empty()) {
      return;
    }
    const std::size_t released_count = std::min(
      static_cast<std::size_t>(through) + 1,
      _full_leg_plan_waypoints.size());
    _travel_info.waypoints.assign(
      _full_leg_plan_waypoints.begin(),
      _full_leg_plan_waypoints.begin() + static_cast<std::ptrdiff_t>(released_count));
    _travel_info.full_leg_destination = _full_leg_plan_waypoints.back();
    _travel_info.leg_fully_released =
      released_count >= _full_leg_plan_waypoints.size();
  }

  void _publish_released_path_prefix(
    const traffic_manager::msg::PathRelease& release)
  {
    const int32_t release_through_index = release.release_through_index;
    const uint32_t release_update_id = release.release_update_id;
    const auto max_index = static_cast<int32_t>(_current_path_request.path.size()) - 1;
    const int32_t through = std::min(release_through_index, max_index);
    if (through < 0 || _current_path_request.path.empty())
    {
      RCLCPP_WARN(
        _node->get_logger(),
        "[TrafficGate] Robot [%s] cannot publish PathRequest — invalid release "
        "through_index=%d (path size=%zu)",
        _travel_info.robot_name.c_str(),
        release_through_index,
        _current_path_request.path.size());
      return;
    }

    const std::size_t previous_published =
      _last_published_path_request.path.size();
    const bool is_extension =
      _path_released_for_pending && previous_published > 0;

    // X7/G6: FREE tip never shrinks on this pending path. Apex E1/G5: WAIT
    // may retract tip (ownership / opposite) so release=true never covers a
    // foreign base — honor PathRelease through as-is on WAIT.
    int32_t previous_tip = -1;
    if (is_extension)
    {
      for (std::size_t i = 0; i < _last_published_path_request.path.size(); ++i)
      {
        if (_last_published_path_request.path[i].release)
        {
          previous_tip = static_cast<int32_t>(i);
        }
      }
    }
    int32_t effective_through = through;
    if (previous_tip >= 0 && release.ecbs_status != "WAIT")
    {
      effective_through = std::max(through, previous_tip);
    }

    // Order interlock: never set release=true on a cell still live-true for
    // another AMR. Wait (keep prior tip) until that peer clears it.
    // Trust TM when ecbs_status==FREE — TM already cleared peer-base / wait
    // deps; clamping here frozen-stale footprints (amr3 stuck after new base).
    if (_live_bases && release.ecbs_status != "FREE")
    {
      const int32_t wanted = effective_through;
      effective_through = _live_bases->clamp_through(
        _travel_info.robot_name,
        _current_path_request.path,
        effective_through,
        previous_tip);
      if (effective_through < wanted)
      {
        RCLCPP_INFO(
          _node->get_logger(),
          "[TrafficGate] Robot [%s] order interlock — release=true only "
          "through %d (wanted %d); peer live base still owns next cells",
          _travel_info.robot_name.c_str(),
          effective_through,
          wanted);
      }
    }

    _last_published_path_request = _current_path_request;
    _last_published_path_request.path = _current_path_request.path;
    _last_published_path_request.path_revision =
      _current_path_revision + release_update_id;
    for (std::size_t i = 0; i < _last_published_path_request.path.size(); ++i)
    {
      _last_published_path_request.path[i].release =
        static_cast<int32_t>(i) <= through;
      _last_published_path_request.path[i].traffic_hold = false;
      _last_published_path_request.path[i].traffic_wait_sec = 0.0;
      _last_published_path_request.path[i].traffic_blocker_amr.clear();
    }

    // added for ECBS — stamp traffic_hold on the released wait waypoint.
    if (release.ecbs_status == "WAIT" && release.hold_leg_index >= 0 &&
      static_cast<std::size_t>(release.hold_leg_index) <
      _last_published_path_request.path.size())
    {
      auto& hold_wp =
        _last_published_path_request.path[
        static_cast<std::size_t>(release.hold_leg_index)];
      hold_wp.traffic_hold = true;
      hold_wp.traffic_wait_sec = release.hold_wait_sec;
      if (!release.blocking_amr_ids.empty())
      {
        hold_wp.traffic_blocker_amr = release.blocking_amr_ids.front();
      }
    }

    _sync_travel_info_waypoints_to_release(effective_through);
    _path_released_for_pending = true;
    _path_requested_time = std::chrono::steady_clock::now();
    if (_live_bases)
    {
      const double px = _last_known_state ?
        _last_known_state->location.x : 0.0;
      const double py = _last_known_state ?
        _last_known_state->location.y : 0.0;
      _live_bases->update(
        _travel_info.robot_name, _last_published_path_request, px, py);
    }
    _path_request_pub->publish(_last_published_path_request);

    if (release_through_index > max_index)
    {
      RCLCPP_WARN(
        _node->get_logger(),
        "[TrafficGate] Robot [%s] release_through_index=%d clamped to %d "
        "(full leg has %zu waypoints)",
        _travel_info.robot_name.c_str(),
        release_through_index,
        through,
        _current_path_request.path.size());
    }

    std::size_t released_count = 0;
    for (const auto & wp : _last_published_path_request.path)
    {
      if (wp.release)
      {
        ++released_count;
      }
    }

    RCLCPP_INFO(
      _node->get_logger(),
      "[TrafficGate] Robot [%s] published PathRequest → execution: request_id=%lu "
      "full leg=%zu waypoints, release=true on %zu (through_index=%d) (%s) "
      "phase=%u revision=%u task_id=[%s]",
      _travel_info.robot_name.c_str(),
      _pending_path_request_id,
      _last_published_path_request.path.size(),
      released_count,
      through,
      is_extension ? "extended release flags" : "initial release flags",
      _current_path_request.phase_index,
      _current_path_request.path_revision,
      _current_path_request.task_id.c_str());
  }
};

using FleetDriverRobotCommandHandlePtr =
  std::shared_ptr<FleetDriverRobotCommandHandle>;

//==============================================================================
/// This is an RAII class that keeps the connections to the fleet driver alive.
struct Connections : public std::enable_shared_from_this<Connections>
{
  /// Parameter change listener
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
    on_set_param;

  /// The API for adding new robots to the adapter
  rmf_fleet_adapter::agv::FleetUpdateHandlePtr fleet;

  /// The API for running the fleet adapter
  rmf_fleet_adapter::agv::AdapterPtr adapter;

  /// The navigation graph for the robot
  std::shared_ptr<const rmf_traffic::agv::Graph> graph;

  /// The traits of the vehicles
  std::shared_ptr<const rmf_traffic::agv::VehicleTraits> traits;

  /// The topic subscription for responding to new fleet states
  rclcpp::Subscription<rmf_fleet_msgs::msg::FleetState>::SharedPtr
    fleet_state_sub;

  /// The publisher for sending out path requests
  rclcpp::Publisher<rmf_fleet_msgs::msg::PathRequest>::SharedPtr
    path_request_pub;

          /// Path intents for traffic_manager (PathRequest gated on /path_release).
  rclcpp::Publisher<traffic_manager::msg::PathIntent>::SharedPtr
    path_intent_pub;

  rclcpp::Subscription<traffic_manager::msg::PathRelease>::SharedPtr
    path_release_sub;

  std::shared_ptr<std::atomic<uint64_t>> path_request_id_gen =
    std::make_shared<std::atomic<uint64_t>>(1);
  std::shared_ptr<FleetLiveBaseRegistry> live_bases =
    std::make_shared<FleetLiveBaseRegistry>();
  /// The publisher for sending out mode requests
  rclcpp::Publisher<rmf_fleet_msgs::msg::ModeRequest>::SharedPtr
    mode_request_pub;

  /// The topic subscription for ending teleop actions
  rclcpp::Subscription<rmf_fleet_msgs::msg::ModeRequest>::SharedPtr
    mode_request_sub;

  /// The topic subscription for listening for lane closure requests
  rclcpp::Subscription<rmf_fleet_msgs::msg::LaneRequest>::SharedPtr
    lane_closure_request_sub;

  /// The publisher for sending out closed lane statuses
  rclcpp::Publisher<rmf_fleet_msgs::msg::ClosedLanes>::SharedPtr
    closed_lanes_pub;

  /// The topic subscription for listening for speed limit requests
  rclcpp::Subscription<rmf_fleet_msgs::msg::SpeedLimitRequest>::SharedPtr
    speed_limit_request_sub;

  /// The topic subscription for per-robot (private) lane closure requests
  rclcpp::Subscription<rmf_fleet_msgs::msg::RobotLaneRequest>::SharedPtr
    robot_lane_request_sub;

  /// added for ECBS — state for head-on replans (subscription + per-candidate
  /// lane closures and their expiry timer).
  /// Head-on decisions: close the keeper's corridor for the candidate.
  rclcpp::Subscription<ecbs_planner::msg::EcbsReplan>::SharedPtr
    ecbs_replan_sub;

  /// added for ECBS — robots whose private closures currently come from ECBS
  /// (robot → set of keepers / reasons). Later REPLANs merge, never replace.
  std::unordered_map<std::string, std::unordered_set<std::string>> ecbs_closures;
  /// added for ECBS — per-keeper blocked waypoints so CLEAR for one keeper can drop only those.
  std::unordered_map<std::string,
    std::unordered_map<std::string, std::unordered_set<int64_t>>> ecbs_blocked_by_keeper;
  /// added for ECBS — every waypoint closed for the candidate since its first REPLAN.
  std::unordered_map<std::string, std::unordered_set<int64_t>> ecbs_blocked;
  /// added for ECBS — wall-clock time (sec) when the candidate's lane closures expire
  /// (max blocked_until_sec from the conflicted AMR's path times).
  std::unordered_map<std::string, double> ecbs_open_at;
  /// added for ECBS
  rclcpp::TimerBase::SharedPtr ecbs_until_timer;

  /// Container for remembering which lanes are currently closed
  std::unordered_set<std::size_t> closed_lanes;

  /// The topic subscription for robot interruption requests
  rclcpp::Subscription<rmf_fleet_msgs::msg::InterruptRequest>::SharedPtr
    interrupt_request_sub;

  /// Zone-manager conflict events. A startPause means the named robots have
  /// been stopped outside RMF, so we must stop reserving the corridor ahead
  /// of them; a stopPause means they are free to proceed again.
  rclcpp::Subscription<zone_manager::msg::Conflict>::SharedPtr
    zone_conflict_sub;

  /// Re-asserts declare_holding() for every currently held robot, since the
  /// hold expires and the running task can overwrite it.
  rclcpp::TimerBase::SharedPtr zone_hold_refresh_timer;

  /// The container for robot update handles. An entry exists only once the
  /// robot has actually joined the fleet — see try_add_robot().
  std::unordered_map<std::string, FleetDriverRobotCommandHandlePtr> robots;

  /// Robots whose add_robot() is in flight. fleet->add_robot() completes
  /// asynchronously (async_make_participant), so between the call and the
  /// completion callback there is a window with no entry in `robots` yet.
  /// Without this, the retry below would fire a second add during that window
  /// and register a duplicate participant in the traffic schedule.
  std::unordered_set<std::string> pending_add;

  /// When each robot was last attempted, used to throttle retries and to
  /// expire a `pending_add` entry whose completion callback never arrived.
  std::unordered_map<std::string, std::chrono::steady_clock::time_point>
  last_add_attempt;

  /// Last time a failed add was logged for this robot. The failure message is
  /// five lines long; at the fleet-state rate it would bury the log.
  std::unordered_map<std::string, std::chrono::steady_clock::time_point>
  last_add_warn;

  /// How often to retry a robot that could not be placed on the graph.
  static constexpr auto kAddRetryInterval = std::chrono::seconds(1);
  /// How often to log that it is still failing.
  static constexpr auto kAddWarnInterval = std::chrono::seconds(10);
  /// Give up waiting on an in-flight add after this and allow a fresh attempt.
  static constexpr auto kAddPendingTimeout = std::chrono::seconds(30);

  /// Decide whether this robot needs an add attempt, and make it if so.
  ///
  /// Replaces an insert-then-add that could only ever try ONCE: the map entry
  /// was created before add_robot() ran, and add_robot() returns early when
  /// compute_plan_starts() finds nothing, leaving a permanent nullptr behind.
  /// Every later fleet state then saw the key already present and skipped the
  /// add, so a robot rejected on its first state message was excluded for the
  /// life of the process and only a restart could recover it.
  ///
  /// Observed 2026-10-08: amr1 came up 1.51 m off the nav graph while AMCL was
  /// still converging, was rejected on that single attempt, then settled to
  /// 3.6 mm from a lane seconds later — a perfectly valid pose it was never
  /// re-checked at. Restarting the FMS cleared `robots` and it joined at once.
  ///
  /// Three states, not two: never tried -> attempt; in flight -> wait;
  /// previously failed -> retry on the interval.
  void try_add_robot(
    const std::string& fleet_name,
    const rmf_fleet_msgs::msg::RobotState& state)
  {
    auto l = lock();

    // Already in the fleet: nothing to do, ever again.
    if (robots.count(state.name) > 0)
      return;

    const auto now = std::chrono::steady_clock::now();
    const auto attempt_it = last_add_attempt.find(state.name);
    const bool have_attempt = attempt_it != last_add_attempt.end();

    if (pending_add.count(state.name) > 0)
    {
      // An add is in flight. Only reconsider if its completion callback never
      // arrived, which would otherwise strand the robot exactly as before.
      if (have_attempt && now - attempt_it->second < kAddPendingTimeout)
        return;

      RCLCPP_WARN(
        adapter->node()->get_logger(),
        "Robot [%s] add has been pending for over %ld s with no completion; "
        "retrying.",
        state.name.c_str(),
        std::chrono::duration_cast<std::chrono::seconds>(
          kAddPendingTimeout).count());
      pending_add.erase(state.name);
    }
    else if (have_attempt && now - attempt_it->second < kAddRetryInterval)
    {
      // Failed recently. add_robot() allocates a command handle and scans the
      // whole graph twice before it can fail, so do not do that per message.
      return;
    }

    last_add_attempt[state.name] = now;
    pending_add.insert(state.name);
    add_robot(fleet_name, state);
  }

  /// Release the in-flight marker after a failed add so try_add_robot() will
  /// try again once the robot reports a pose that can be placed on the graph.
  void note_add_robot_failed(const std::string& robot_name)
  {
    auto l = lock();
    pending_add.erase(robot_name);
  }

  /// True when the failure for this robot should be logged, so a robot that
  /// cannot be placed does not emit the five-line error at the state rate.
  bool should_warn_add_failure(const std::string& robot_name)
  {
    auto l = lock();
    const auto now = std::chrono::steady_clock::now();
    const auto it = last_add_warn.find(robot_name);
    if (it != last_add_warn.end() && now - it->second < kAddWarnInterval)
      return false;

    last_add_warn[robot_name] = now;
    return true;
  }

  void add_robot(
    const std::string& fleet_name,
    const rmf_fleet_msgs::msg::RobotState& state)
  {
    RCLCPP_INFO(
      adapter->node()->get_logger(),
      "Adding robot [%s] to fleet [%s] at location [%.2f, %.2f, %.2f] on level [%s]",
      state.name.c_str(), fleet_name.c_str(),
      state.location.x, state.location.y, state.location.yaw,
      state.location.level_name.c_str());

    const auto& robot_name = state.name;
    const auto command = std::make_shared<FleetDriverRobotCommandHandle>(
      *adapter->node(), fleet_name, robot_name, graph, traits,
      path_request_pub, mode_request_pub, path_intent_pub,
      path_request_id_gen, live_bases);

    const auto& l = state.location;
    // Force planning to start from the reported pose (not snapped to nearest
    // waypoint) by tightening the merge distance, while still allowing lane
    // merges. This yields multiple candidate starts (forward/right/left) and
    // the planner will pick the best for the goal.
    const double max_merge_wp_dist = 0.01;   // meters
    const double max_merge_lane_dist = 1.0;  // meters
    const double min_lane_length = 0.05;     // meters

    const auto& starts = rmf_traffic::agv::compute_plan_starts(
      *graph, state.location.level_name, {l.x, l.y, l.yaw},
      rmf_traffic_ros2::convert(adapter->node()->now()),
      max_merge_wp_dist,
      max_merge_lane_dist,
      min_lane_length);

    if (starts.empty())
    {
      // Release the in-flight marker so try_add_robot() retries. The robot is
      // NOT recorded in `robots`, so it stays out of the fleet until a later
      // state message puts it somewhere the planner can start from.
      note_add_robot_failed(state.name);

      if (!should_warn_add_failure(state.name))
        return;

      const std::optional<DistanceFromGraph> distance =
        distance_from_graph(state.location, *graph);

      std::string hint;
      if (!distance.has_value())
      {
        hint = "None of the waypoints in the graph are on a map called ["
          + l.level_name + "].";
      }
      else
      {
        const auto to_name = [&](const std::size_t index) -> std::string
          {
            const auto& wp = graph->get_waypoint(index);
            if (wp.name())
              return *wp.name();

            return "#" + std::to_string(index);
          };

        if (distance->type == DistanceFromGraph::Lane)
        {
          const auto& lane = graph->get_lane(distance->index);
          hint = "The closest lane on the navigation graph ["
            + std::to_string(distance->index) + "] connects waypoint ["
            + to_name(lane.entry().waypoint_index()) + "] to ["
            + to_name(lane.exit().waypoint_index()) + "] and is a distance of ["
            + std::to_string(distance->value) + "m] from the robot.";
        }
        else
        {
          hint = "The closest waypoint on the navigation graph ["
            + to_name(distance->index) + "] is a distance of ["
            + std::to_string(distance->value) + "m] from the robot.";
        }
      }

      RCLCPP_ERROR(
        adapter->node()->get_logger(),
        "Unable to compute a StartSet for robot [%s] using level_name [%s] and "
        "location [%f, %f, %f] specified in its RobotState message. This can "
        "happen if the level_name in the RobotState message does not match any "
        "of the map names in the navigation graph supplied or if the location "
        "reported in the RobotState message is far way from the navigation "
        "graph. This robot will not be added to the fleet [%s]. The following "
        "hint may help with debugging: %s",
        state.name.c_str(),
        state.location.level_name.c_str(),
        l.x, l.y, l.yaw,
        fleet_name.c_str(),
        hint.c_str());

      return;
    }

    RCLCPP_INFO(
      adapter->node()->get_logger(),
      "Robot [%s] successfully added to fleet [%s] with [%zu] start positions",
      robot_name.c_str(), fleet_name.c_str(), starts.size());

    fleet->add_robot(
      command, robot_name, traits->profile(),
      starts,
      [c = weak_from_this(), command, robot_name = std::move(robot_name)](
        const rmf_fleet_adapter::agv::RobotUpdateHandlePtr& updater)
      {
        const auto connections = c.lock();
        if (!connections)
          return;

        auto lock = connections->lock();

        command->set_updater(updater);
        connections->robots[robot_name] = command;
        // The add is no longer in flight; `robots` now holds it, so
        // try_add_robot() returns immediately from here on.
        connections->pending_add.erase(robot_name);
        connections->last_add_warn.erase(robot_name);
      });
  }

  std::recursive_mutex _mutex;
  std::unique_lock<std::recursive_mutex> lock()
  {
    std::unique_lock<std::recursive_mutex> l(_mutex, std::defer_lock);
    while (!l.try_lock())
    {
      // Intentionally busy wait
    }

    return l;
  }
};

//==============================================================================
std::shared_ptr<Connections> make_fleet(
  const rmf_fleet_adapter::agv::AdapterPtr& adapter)
{
  RCLCPP_INFO(
    adapter->node()->get_logger(),
    "Starting fleet creation process");

  const auto& node = adapter->node();
  node->declare_parameter("enable_responsive_wait", true);

  std::shared_ptr<Connections> connections = std::make_shared<Connections>();
  connections->adapter = adapter;
  connections->on_set_param =
    node->get_node_parameters_interface()->add_on_set_parameters_callback(
    [w = connections->weak_from_this()](
      const std::vector<rclcpp::Parameter>& params)
    -> rcl_interfaces::msg::SetParametersResult
    {
      const auto self = w.lock();
      if (!self)
        return rcl_interfaces::msg::SetParametersResult();

      for (const auto& p : params)
      {
        if (p.get_name() == "enable_responsive_wait")
        {
          const auto value = p.as_bool();
          for (auto& [_, cmd] : self->robots)
          {
            if (const auto updater = cmd->get_updater())
              updater->enable_responsive_wait(value);
          }
        }
      }

      rcl_interfaces::msg::SetParametersResult r;
      r.successful = true;
      return r;
    });

  const std::string fleet_name_param_name = "fleet_name";
  const std::string fleet_name = node->declare_parameter(
    "fleet_name", std::string());
  if (fleet_name.empty())
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "Missing [%s] parameter", fleet_name_param_name.c_str());

    return nullptr;
  }

  RCLCPP_INFO(
    node->get_logger(),
    "Fleet name configured as: [%s]", fleet_name.c_str());

  connections->traits = std::make_shared<rmf_traffic::agv::VehicleTraits>(
    rmf_fleet_adapter::get_traits_or_default(
      *node, 0.7, 0.3, 0.5, 1.5, 0.5, 1.5));
     //*node, 0.7, 0.3, 0.5, 1.5, 0.5, 1.5));

  const std::string nav_graph_param_name = "nav_graph_file";
  const std::string graph_file =
    node->declare_parameter(nav_graph_param_name, std::string());
  if (graph_file.empty())
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "Missing [%s] parameter", nav_graph_param_name.c_str());

    return nullptr;
  }

  RCLCPP_INFO(
    node->get_logger(),
    "Loading navigation graph from: [%s]", graph_file.c_str());

  connections->graph =
    std::make_shared<rmf_traffic::agv::Graph>(
    rmf_fleet_adapter::agv::parse_graph(graph_file, *connections->traits));

  RCLCPP_INFO(
    node->get_logger(),
    "Navigation graph loaded with [%zu] waypoints and [%zu] lanes",
    connections->graph->num_waypoints(), connections->graph->num_lanes());

  std::cout << "The fleet [" << fleet_name
            << "] has the following named waypoints:\n";
  for (const auto& key : connections->graph->keys())
    std::cout << " -- " << key.first << std::endl;

  std::optional<std::string> server_uri = std::nullopt;
  const std::string uri =
    node->declare_parameter("server_uri", std::string());
  if (!uri.empty())
  {
    RCLCPP_INFO(
      node->get_logger(),
      "API server URI: [%s]", uri.c_str());

    server_uri = uri;
  }

  RCLCPP_INFO(
    node->get_logger(),
    "Creating fleet [%s] with traits and graph", fleet_name.c_str());

  connections->fleet = adapter->add_fleet(
    fleet_name, *connections->traits, *connections->graph, server_uri);

  // Curve ETA timing from eta_robot_params.yaml (via launch parameters).
  // Pads itinerary timestamps on gradual polyline bends so ETA matches
  // slower real curve motion. Disabled when curve_linear_speed_mps <= 0.
  const double curve_linear_speed_mps =
    node->declare_parameter("curve_linear_speed_mps", 0.0);
  const double curve_heading_min_deg =
    node->declare_parameter("curve_heading_min_deg", 3.0);
  const double curve_heading_max_deg =
    node->declare_parameter("curve_heading_max_deg", 25.0);
  const double curve_heading_change_threshold_deg =
    node->declare_parameter("curve_heading_change_threshold_deg", 20.0);
  const double linear_accel_mps2 = node->has_parameter("linear_acceleration")
    ? node->get_parameter("linear_acceleration").as_double()
    : node->declare_parameter("linear_acceleration", 0.5);
  const double linear_decel_mps2 =
    node->declare_parameter("linear_deceleration", 0.6);
  const double approach_start_distance_m =
    node->declare_parameter("approach_start_distance_m", 5.0);
  const double angular_speed_rps = node->has_parameter("angular_velocity")
    ? node->get_parameter("angular_velocity").as_double()
    : node->declare_parameter("angular_velocity", 0.85);
  const double angular_accel_rps2 = node->has_parameter("angular_acceleration")
    ? node->get_parameter("angular_acceleration").as_double()
    : node->declare_parameter("angular_acceleration", 1.0);
  const auto stop_speed_mps = node->declare_parameter(
    "stop_speed_mps", std::vector<double>{0.5, 1.0, 1.5, 2.0});
  const auto stop_distance_m = node->declare_parameter(
    "stop_distance_m", std::vector<double>{3.0, 4.0, 5.0, 7.0});
  connections->fleet->set_curve_timing(
    curve_linear_speed_mps,
    curve_heading_min_deg,
    curve_heading_max_deg,
    curve_heading_change_threshold_deg,
    linear_accel_mps2,
    linear_decel_mps2,
    approach_start_distance_m,
    angular_speed_rps,
    angular_accel_rps2,
    stop_speed_mps,
    stop_distance_m);

  RCLCPP_INFO(
    node->get_logger(),
    "Fleet [%s] created successfully", fleet_name.c_str());

  // We disable fleet state publishing for this fleet adapter because we expect
  // the fleet drivers to publish these messages.
  connections->fleet->fleet_state_topic_publish_period(std::nullopt);

  connections->closed_lanes_pub =
    adapter->node()->create_publisher<rmf_fleet_msgs::msg::ClosedLanes>(
    rmf_fleet_adapter::ClosedLaneTopicName,
    rclcpp::SystemDefaultsQoS().reliable().keep_last(1).transient_local());

  connections->lane_closure_request_sub =
    adapter->node()->create_subscription<rmf_fleet_msgs::msg::LaneRequest>(
    rmf_fleet_adapter::LaneClosureRequestTopicName,
    rclcpp::SystemDefaultsQoS(),
    [w = connections->weak_from_this(), fleet_name](
      rmf_fleet_msgs::msg::LaneRequest::UniquePtr request_msg)
    {
      const auto connections = w.lock();
      if (!connections)
        return;

      if (request_msg->fleet_name != fleet_name ||
      request_msg->fleet_name.empty())
        return;

      connections->fleet->open_lanes(request_msg->open_lanes);
      connections->fleet->close_lanes(request_msg->close_lanes);

      std::unordered_set<std::size_t> newly_closed_lanes;
      for (const auto& l : request_msg->close_lanes)
      {
        if (connections->closed_lanes.count(l) == 0)
          newly_closed_lanes.insert(l);

        connections->closed_lanes.insert(l);
      }

      for (const auto& l : request_msg->open_lanes)
        connections->closed_lanes.erase(l);

      for (auto& [_, robot] : connections->robots)
        robot->newly_closed_lanes(newly_closed_lanes);

      rmf_fleet_msgs::msg::ClosedLanes state_msg;
      state_msg.fleet_name = fleet_name;
      state_msg.closed_lanes.insert(
        state_msg.closed_lanes.begin(),
        connections->closed_lanes.begin(),
        connections->closed_lanes.end());

      connections->closed_lanes_pub->publish(state_msg);
    });

  // Per-robot ("private") lane closures. Unlike LaneRequest above, this affects
  // exactly one robot: it is detached onto its own planner and replanned, while
  // every other robot keeps the shared fleet graph.
  connections->robot_lane_request_sub =
    adapter->node()->create_subscription<
    rmf_fleet_msgs::msg::RobotLaneRequest>(
    "/robot_lane_requests",
    rclcpp::SystemDefaultsQoS(),
    [w = connections->weak_from_this(), fleet_name, node = adapter->node()](
      rmf_fleet_msgs::msg::RobotLaneRequest::UniquePtr request_msg)
    {
      const auto connections = w.lock();
      if (!connections)
        return;

      if (request_msg->fleet_name != fleet_name ||
      request_msg->fleet_name.empty() ||
      request_msg->robot_name.empty())
        return;

      const auto command_it = connections->robots.find(request_msg->robot_name);
      if (command_it == connections->robots.end())
      {
        RCLCPP_WARN(
          node->get_logger(),
          "Ignoring RobotLaneRequest for unknown robot [%s] of fleet [%s]",
          request_msg->robot_name.c_str(), fleet_name.c_str());
        return;
      }

      const auto updater = command_it->second->get_updater();
      if (!updater)
        return;

      // Reopening takes precedence: any open_lanes entry clears ALL private
      // closures for this robot and re-attaches it to the fleet planner.
      if (!request_msg->open_lanes.empty())
      {
        updater->open_lanes_for_robot();
        RCLCPP_INFO(
          node->get_logger(),
          "Cleared private lane closures for robot [%s] of fleet [%s]",
          request_msg->robot_name.c_str(), fleet_name.c_str());
      }
      else if (!request_msg->close_lanes.empty())
      {
        const std::vector<std::size_t> lanes(
          request_msg->close_lanes.begin(), request_msg->close_lanes.end());
        updater->close_lanes_for_robot(lanes);
        // Stop the zone-hold refresh from wiping the schedule while this
        // replan is in flight (and teach restore not to put the old path back).
        command_it->second->arm_zone_replan_preserve();
        RCLCPP_INFO(
          node->get_logger(),
          "Closed %zu lane(s) for robot [%s] of fleet [%s] only",
          lanes.size(), request_msg->robot_name.c_str(), fleet_name.c_str());
      }
      else
      {
        // Empty close_lanes and empty open_lanes: treat as a clear.
        updater->open_lanes_for_robot();
      }

      // Make the robot act on its new graph — unless the publisher asked to
      // keep the current plan (e.g. a 4D drop corridor reopened right after
      // the drop leg was planned on it).
      if (!request_msg->skip_replan)
      {
        updater->replan();
      }
    });

  // added for ECBS — /ecbs_replan_candidate handler + closure-expiry timer below.
  // evaluate_path_server head-on decisions. REPLAN closes, for the candidate
  // only, every lane touching the keeper's corridor until the conflicted AMR's
  // path times (blocked_until_sec) expire or CLEAR arrives.
  // Replan is called once per (robot, keeper) pair — never in a loop.
  connections->ecbs_replan_sub =
    adapter->node()->create_subscription<ecbs_planner::msg::EcbsReplan>(
    "/ecbs_replan_candidate",
    rclcpp::QoS(rclcpp::KeepLast(20)).reliable(),
    [w = connections->weak_from_this(), fleet_name, node = adapter->node()](
      ecbs_planner::msg::EcbsReplan::UniquePtr msg)
    {
      const auto connections = w.lock();
      if (!connections || !connections->graph)
      {
        RCLCPP_WARN(node->get_logger(), "ECBS %s dropped: no nav graph", msg->decision.c_str());
        return;
      }

      // Ignore empty / legacy publishes (EcbsGate FailReplan used to send these).
      if (msg->decision.empty())
        return;

      // added for ECBS — every early return below logs why, so a dropped decision is visible.
      const auto command_it = connections->robots.find(msg->amr_id);
      if (command_it == connections->robots.end())
      {
        RCLCPP_WARN(node->get_logger(), "ECBS %s for [%s] dropped: unknown robot",
          msg->decision.c_str(), msg->amr_id.c_str());
        return;
      }

      const auto updater = command_it->second->get_updater();
      if (!updater)
      {
        RCLCPP_WARN(node->get_logger(), "ECBS %s for [%s] dropped: robot has no updater yet",
          msg->decision.c_str(), msg->amr_id.c_str());
        return;
      }

      const auto& robot = msg->amr_id;
      auto lanes_for = [&](const std::unordered_set<int64_t>& blocked)
        {
          std::vector<std::size_t> lanes;
          const auto& graph = *connections->graph;
          for (std::size_t l = 0; l < graph.num_lanes(); ++l)
          {
            const auto& lane = graph.get_lane(l);
            const auto entry =
              static_cast<int64_t>(lane.entry().waypoint_index());
            const auto exit =
              static_cast<int64_t>(lane.exit().waypoint_index());
            if (blocked.count(entry) || blocked.count(exit))
              lanes.push_back(l);
          }
          return lanes;
        };
      auto rebuild_union = [&]()
        {
          auto& uni = connections->ecbs_blocked[robot];
          uni.clear();
          for (const auto& kv : connections->ecbs_blocked_by_keeper[robot])
            uni.insert(kv.second.begin(), kv.second.end());
        };

      if (msg->decision == msg->DECISION_REPLAN)
      {
        const std::unordered_set<int64_t> incoming(
          msg->blocked_waypoints.begin(), msg->blocked_waypoints.end());
        // Merge keepers: primary keep_amr plus any extras in conflicting_amrs.
        std::unordered_set<std::string> keepers;
        if (!msg->keep_amr.empty())
          keepers.insert(msg->keep_amr);
        for (const auto& a : msg->conflicting_amrs)
        {
          if (!a.empty() && a != robot)
            keepers.insert(a);
        }
        if (keepers.empty())
          keepers.insert(msg->keep_amr.empty() ? "?" : msg->keep_amr);

        // Full union list from ECBS: merge into cumulative set, never replace.
        auto& uni = connections->ecbs_blocked[robot];
        for (const auto wp : incoming)
          uni.insert(wp);
        for (const auto& k : keepers)
        {
          connections->ecbs_closures[robot].insert(k);
          auto& per = connections->ecbs_blocked_by_keeper[robot][k];
          for (const auto wp : incoming)
            per.insert(wp);
        }
        rebuild_union();

        const auto lanes = lanes_for(connections->ecbs_blocked[robot]);
        if (lanes.empty())
        {
          RCLCPP_WARN(node->get_logger(),
            "ECBS REPLAN for [%s] dropped: no nav-graph lane touches the %zu blocked waypoint(s)",
            robot.c_str(), connections->ecbs_blocked[robot].size());
          return;
        }

        double open_at = 0.0;
        for (const double t : msg->blocked_until_sec)
          open_at = std::max(open_at, t);
        if (open_at > 0.0)
          connections->ecbs_open_at[robot] = open_at;
        updater->close_lanes_for_robot(lanes);
        RCLCPP_WARN(
          node->get_logger(),
          "ECBS replan candidate [%s] (keep [%s], rev %u): closed %zu lane(s) "
          "over %zu waypoint(s) until %.1f, replanning once: %s",
          robot.c_str(), msg->keep_amr.c_str(), msg->revision, lanes.size(),
          connections->ecbs_blocked[robot].size(), open_at, msg->reason.c_str());
        command_it->second->arm_zone_replan_preserve();
        updater->replan();
      }
      else if (msg->decision == msg->DECISION_CLEAR)
      {
        if (connections->ecbs_closures.count(robot) == 0)
          return;
        const bool fallback =
          msg->reason.find("using first RMF path") != std::string::npos;
        const bool path_clear =
          msg->reason.find("path clear at iteration") != std::string::npos;
        // Single-keeper leave: drop only that keeper's cells, keep the rest.
        const bool one_keeper =
          !fallback && !path_clear &&
          msg->reason.find("parked AMR left") != std::string::npos &&
          !msg->keep_amr.empty();

        if (one_keeper)
        {
          connections->ecbs_closures[robot].erase(msg->keep_amr);
          connections->ecbs_blocked_by_keeper[robot].erase(msg->keep_amr);
          rebuild_union();
          updater->open_lanes_for_robot();
          const auto& remaining = connections->ecbs_blocked[robot];
          if (!remaining.empty())
          {
            const auto lanes = lanes_for(remaining);
            if (!lanes.empty())
              updater->close_lanes_for_robot(lanes);
          }
          else
          {
            connections->ecbs_closures.erase(robot);
            connections->ecbs_blocked.erase(robot);
            connections->ecbs_blocked_by_keeper.erase(robot);
            connections->ecbs_open_at.erase(robot);
          }
          RCLCPP_INFO(
            node->get_logger(),
            "ECBS CLEAR for [%s] keeper [%s] (rev %u): removed that keeper only: %s",
            robot.c_str(), msg->keep_amr.c_str(), msg->revision, msg->reason.c_str());
          return;
        }

        const auto ever_blocked = connections->ecbs_blocked[robot];
        // added for ECBS — path-clear means the detour is clean *with* these
        // closures. Reopening here lets RMF replan through the parked AMR again
        // (REPLAN↔CLEAR storm). Keep closures; the until-timer opens once a
        // real leg no longer visits the blocked cells.
        if (path_clear)
        {
          connections->ecbs_open_at[robot] = node->now().seconds() + 1.0;
          RCLCPP_INFO(
            node->get_logger(),
            "ECBS path-clear for [%s] (rev %u): keeping lane closures "
            "(open when detour leg is active and clear): %s",
            robot.c_str(), msg->revision, msg->reason.c_str());
          return;
        }
        // Still driving the detour / old leg: keep lanes closed — except
        // fallback which must reopen for the first RMF path.
        if (!fallback &&
          command_it->second->remaining_leg_visits(ever_blocked))
        {
          RCLCPP_INFO(
            node->get_logger(),
            "ECBS CLEAR for [%s] (rev %u) ignored while still on route — "
            "keeping closures, no second replan: %s",
            robot.c_str(), msg->revision, msg->reason.c_str());
          return;
        }
        connections->ecbs_closures.erase(robot);
        connections->ecbs_blocked.erase(robot);
        connections->ecbs_blocked_by_keeper.erase(robot);
        connections->ecbs_open_at.erase(robot);
        updater->open_lanes_for_robot();
        if (fallback)
        {
          command_it->second->arm_zone_replan_preserve();
          updater->replan();
          RCLCPP_INFO(
            node->get_logger(),
            "ECBS fallback for [%s] (rev %u): reopened all lanes and re-submitted: %s",
            robot.c_str(), msg->revision, msg->reason.c_str());
        }
        else
        {
          RCLCPP_INFO(
            node->get_logger(),
            "ECBS cleared closures for [%s] (rev %u), no replan: %s",
            robot.c_str(), msg->revision, msg->reason.c_str());
        }
      }
      else if (msg->decision == msg->DECISION_NO_ALTERNATIVE)
      {
        RCLCPP_WARN(
          node->get_logger(),
          "ECBS: [%s] has no route around [%s] (rev %u): %s",
          robot.c_str(), msg->keep_amr.c_str(), msg->revision,
          msg->reason.c_str());
        if (connections->ecbs_closures.erase(robot) > 0)
        {
          connections->ecbs_blocked.erase(robot);
          connections->ecbs_blocked_by_keeper.erase(robot);
          connections->ecbs_open_at.erase(robot);
          updater->open_lanes_for_robot();
        }
      }
    });

  // added for ECBS — lane closures do not expire in RMF; open them when the
  // conflicted AMR's path time (blocked_until_sec) has passed, if CLEAR has not.
  connections->ecbs_until_timer = adapter->node()->create_wall_timer(
    std::chrono::milliseconds(500),
    [w = connections->weak_from_this(), node = adapter->node()]()
    {
      const auto connections = w.lock();
      if (!connections)
        return;
      const double now = node->now().seconds();
      std::vector<std::string> due;
      for (const auto& kv : connections->ecbs_open_at)
      {
        if (kv.second > 0.0 && now >= kv.second)
          due.push_back(kv.first);
      }
      for (const auto& robot : due)
      {
        const auto command_it = connections->robots.find(robot);
        if (command_it == connections->robots.end())
        {
          connections->ecbs_open_at.erase(robot);
          connections->ecbs_closures.erase(robot);
          connections->ecbs_blocked.erase(robot);
          connections->ecbs_blocked_by_keeper.erase(robot);
          continue;
        }
        // added for ECBS — do not open while the candidate still visits the
        // closed corridor, or while only a stop/hold leg is active (trivial
        // legs never visit blocked cells but are not the detour).
        const auto& blocked = connections->ecbs_blocked[robot];
        if (!blocked.empty() &&
          command_it->second->remaining_leg_visits(blocked))
        {
          continue;
        }
        if (command_it->second->remaining_leg_is_trivial())
        {
          continue;
        }
        const auto updater = command_it->second->get_updater();
        if (!updater)
          continue;
        connections->ecbs_open_at.erase(robot);
        connections->ecbs_closures.erase(robot);
        connections->ecbs_blocked.erase(robot);
        connections->ecbs_blocked_by_keeper.erase(robot);
        updater->open_lanes_for_robot();
        RCLCPP_INFO(
          node->get_logger(),
          "ECBS: opened closures for [%s] — conflicted path time elapsed "
          "(leg finished, no replan)",
          robot.c_str());
      }
    });

  connections->speed_limit_request_sub =
    adapter->node()->create_subscription<
    rmf_fleet_msgs::msg::SpeedLimitRequest>(
    rmf_fleet_adapter::SpeedLimitRequestTopicName,
    rclcpp::SystemDefaultsQoS(),
    [w = connections->weak_from_this(), fleet_name](
      rmf_fleet_msgs::msg::SpeedLimitRequest::ConstSharedPtr request_msg)
    {
      const auto connections = w.lock();
      if (!connections)
        return;

      if (request_msg->fleet_name != fleet_name ||
      request_msg->fleet_name.empty())
        return;

      std::vector<rmf_fleet_adapter::agv::FleetUpdateHandle::SpeedLimitRequest>
      requests;
      for (const auto& limit : request_msg->speed_limits)
      {
        auto request =
        rmf_fleet_adapter::agv::FleetUpdateHandle::SpeedLimitRequest(
          limit.lane_index, limit.speed_limit);
        requests.push_back(std::move(request));
      }
      connections->fleet->limit_lane_speeds(requests);
      connections->fleet->remove_speed_limits(request_msg->remove_limits);
    });

  connections->interrupt_request_sub =
    adapter->node()->create_subscription<rmf_fleet_msgs::msg::InterruptRequest>(
    rmf_fleet_adapter::InterruptRequestTopicName,
    rclcpp::SystemDefaultsQoS(),
    [w = connections->weak_from_this(), fleet_name](
      rmf_fleet_msgs::msg::InterruptRequest::UniquePtr request_msg)
    {
      const auto connections = w.lock();
      if (!connections)
        return;

      if (request_msg->fleet_name != fleet_name)
        return;

      const auto rit = connections->robots.find(request_msg->robot_name);
      if (rit == connections->robots.end())
      {
        RCLCPP_WARN(
          connections->adapter->node()->get_logger(),
          "Could not find robot [%s] in fleet [%s]",
          request_msg->robot_name.c_str(),
          fleet_name.c_str());
        return;
      }

      rit->second->handle_interrupt_request(*request_msg);
    });

  // ---------------------------------------------------------------------
  // /zone_conflicts -> traffic-schedule hold
  //
  // The zone manager pauses a robot by sending it a VDA5050 startPause over
  // MQTT (see execution_full_control's zone_conflict_callback). That path is
  // invisible to RMF, so the paused robot keeps reserving the corridor it is
  // no longer driving down. Mirror those events into the schedule here: hold
  // the robot's pose on startPause, release it on stopPause.
  //
  // QoS must match the zone manager's publisher: KeepLast(200), reliable.
  // ---------------------------------------------------------------------
  connections->zone_conflict_sub =
    adapter->node()->create_subscription<zone_manager::msg::Conflict>(
    "/zone_conflicts",
    rclcpp::QoS(rclcpp::KeepLast(200)).reliable(),
    [w = connections->weak_from_this()](
      zone_manager::msg::Conflict::UniquePtr msg)
    {
      const auto self = w.lock();
      if (!self || !msg)
        return;

      const bool hold =
        msg->type == zone_manager::msg::Conflict::TYPE_DETECTED &&
        msg->action == zone_manager::msg::Conflict::ACTION_START_PAUSE;
      const bool release =
        msg->type == zone_manager::msg::Conflict::TYPE_RESOLVED &&
        msg->action == zone_manager::msg::Conflict::ACTION_STOP_PAUSE;

      if (!hold && !release)
      {
        // Detected/stopPause and Resolved/startPause are not combinations the
        // zone manager produces; ignore rather than guess at the intent.
        RCLCPP_WARN(
          self->adapter->node()->get_logger(),
          "Ignoring /zone_conflicts with mismatched type=%u action=%s zone=%s",
          msg->type, msg->action.c_str(), msg->zone_id.c_str());
        return;
      }

      for (const auto& robot_name : msg->robot_ids)
      {
        if (robot_name.empty())
          continue;

        const auto rit = self->robots.find(robot_name);
        if (rit == self->robots.end())
        {
          RCLCPP_WARN(
            self->adapter->node()->get_logger(),
            "Zone conflict [%s] names robot [%s], which is not in this fleet; "
            "its schedule reservation cannot be held.",
            msg->zone_id.c_str(), robot_name.c_str());
          continue;
        }

        rit->second->set_zone_hold(hold, msg->zone_id);
      }
    });

  // The hold expires and the running task can overwrite it, so re-assert it
  // well inside kZoneHoldDuration. Robots with no active hold are a no-op.
  connections->zone_hold_refresh_timer =
    adapter->node()->create_wall_timer(
    std::chrono::seconds(2),
    [w = connections->weak_from_this()]()
    {
      const auto self = w.lock();
      if (!self)
        return;

      for (auto& [name, command] : self->robots)
      {
        (void)name;
        if (command)
          command->refresh_zone_hold();
      }
    });

  /* *INDENT-OFF* */
  connections->mode_request_sub =
    adapter->node()->create_subscription<rmf_fleet_msgs::msg::ModeRequest>(
    "/action_execution_notice",
    rclcpp::SystemDefaultsQoS(),
    [w = connections->weak_from_this(), fleet_name](
      rmf_fleet_msgs::msg::ModeRequest::UniquePtr msg)
    {
      if (msg->fleet_name.empty() ||
        msg->fleet_name != fleet_name ||
        msg->robot_name.empty())
      {
        return;
      }

      if (msg->mode.mode == msg->mode.MODE_IDLE)
      {
        const auto self = w.lock();
        if (!self)
          return;

        const auto command_it = self->robots.find(msg->robot_name);
        if (command_it == self->robots.end())
          return;

        command_it->second->complete_robot_action();
      }
    });
  /* *INDENT-ON* */

  // Parameters required for task planner
  // Battery system
  auto battery_system_optional = rmf_fleet_adapter::get_battery_system(
    *node, 24.0, 40.0, 8.8);
  if (!battery_system_optional)
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "Invalid values supplied for battery system");

    return nullptr;
  }
  auto battery_system = std::make_shared<rmf_battery::agv::BatterySystem>(
    *battery_system_optional);

  // Mechanical system and motion_sink
  auto mechanical_system_optional = rmf_fleet_adapter::get_mechanical_system(
    *node, 70.0, 40.0, 0.22);
  if (!mechanical_system_optional)
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "Invalid values supplied for mechanical system");

    return nullptr;
  }
  rmf_battery::agv::MechanicalSystem& mechanical_system =
    *mechanical_system_optional;

  std::shared_ptr<rmf_battery::agv::SimpleMotionPowerSink> motion_sink =
    std::make_shared<rmf_battery::agv::SimpleMotionPowerSink>(
    *battery_system, mechanical_system);

  // Ambient power system
  const double ambient_power_drain =
    rmf_fleet_adapter::get_parameter_or_default(
    *node, "ambient_power_drain", 20.0);
  auto ambient_power_system = rmf_battery::agv::PowerSystem::make(
    ambient_power_drain);
  if (!ambient_power_system)
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "Invalid values supplied for ambient power system");

    return nullptr;
  }
  std::shared_ptr<rmf_battery::agv::SimpleDevicePowerSink> ambient_sink =
    std::make_shared<rmf_battery::agv::SimpleDevicePowerSink>(
    *battery_system, *ambient_power_system);

  // Tool power system
  const double tool_power_drain = rmf_fleet_adapter::get_parameter_or_default(
    *node, "tool_power_drain", 10.0);
  auto tool_power_system = rmf_battery::agv::PowerSystem::make(
    tool_power_drain);
  if (!tool_power_system)
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "Invalid values supplied for tool power system");

    return nullptr;
  }
  std::shared_ptr<rmf_battery::agv::SimpleDevicePowerSink> tool_sink =
    std::make_shared<rmf_battery::agv::SimpleDevicePowerSink>(
    *battery_system, *tool_power_system);

  // Drain battery
  const bool drain_battery = rmf_fleet_adapter::get_parameter_or_default(
    *node, "drain_battery", false);
  // Recharge threshold
  const double recharge_threshold = rmf_fleet_adapter::get_parameter_or_default(
    *node, "recharge_threshold", 0.2);
  // Recharge state of charge
  const double recharge_soc = rmf_fleet_adapter::get_parameter_or_default(
    *node, "recharge_soc", 1.0);
  const std::string finishing_request_string =
    node->declare_parameter("finishing_request", "charge");
  rmf_task::ConstRequestFactoryPtr finishing_request = nullptr;
  std::function<rmf_traffic::Time()> get_time =
    [n = std::weak_ptr<rclcpp::Node>(node)]()
    {
      const auto node = n.lock();
      if (!node)
      {
        const auto time_since_epoch =
          std::chrono::system_clock::now().time_since_epoch();
        return rmf_traffic::Time(time_since_epoch);
      }
      return rmf_traffic_ros2::convert(node->now());
    };

  if (finishing_request_string == "charge")
  {
    finishing_request =
      std::make_shared<rmf_task::requests::ChargeBatteryFactory>(
      std::string(node->get_name()),
      std::move(get_time));
    RCLCPP_INFO(
      node->get_logger(),
      "Fleet is configured to perform ChargeBattery as finishing request");
  }
  else if (finishing_request_string == "park")
  {
    finishing_request =
      std::make_shared<rmf_task::requests::ParkRobotFactory>(
      std::string(node->get_name()),
      std::move(get_time));
    RCLCPP_INFO(
      node->get_logger(),
      "Fleet is configured to perform ParkRobot as finishing request");
  }
  else if (finishing_request_string == "nothing")
  {
    RCLCPP_INFO(
      node->get_logger(),
      "Fleet is not configured to perform any finishing request");
  }
  else
  {
    RCLCPP_WARN(
      node->get_logger(),
      "Provided finishing request [%s] is unsupported. The valid "
      "finishing requests are [charge, park, nothing]. The task planner will "
      " default to [nothing].",
      finishing_request_string.c_str());
  }

  if (!connections->fleet->set_task_planner_params(
      battery_system,
      motion_sink,
      ambient_sink,
      tool_sink,
      recharge_threshold,
      recharge_soc,
      drain_battery,
      finishing_request))
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "Failed to initialize task planner parameters");

    return nullptr;
  }

  std::unordered_set<uint8_t> task_types;
  if (node->declare_parameter<bool>("perform_loop", false))
  {
    task_types.insert(rmf_task_msgs::msg::TaskType::TYPE_LOOP);
  }

  // If the perform_deliveries parameter is true, then we just blindly accept
  // all delivery requests.
  if (node->declare_parameter<bool>("perform_deliveries", false))
  {
    task_types.insert(rmf_task_msgs::msg::TaskType::TYPE_DELIVERY);
    connections->fleet->accept_delivery_requests(
      [](const rmf_task_msgs::msg::Delivery&) { return true; });
  }

  connections->fleet->accept_task_requests(
    [task_types](const rmf_task_msgs::msg::TaskProfile& msg)
    {
      if (task_types.find(msg.description.task_type.type) != task_types.end())
        return true;

      return false;
    });

  const auto consider =
    [](const nlohmann::json& /*description*/,
      rmf_fleet_adapter::agv::FleetUpdateHandle::Confirmation& confirm)
    {
      // We accept all actions since full_control may be used for different
      // types of robots.
      confirm.accept();
    };

  // Configure this fleet to perform any kind of teleop action
  connections->fleet->add_performable_action(
    "teleop",
    consider);

  if (node->declare_parameter<bool>("disable_delay_threshold", false))
  {
    connections->fleet->default_maximum_delay(rmf_utils::nullopt);
  }
  else
  {
    connections->fleet->default_maximum_delay(
      rmf_fleet_adapter::get_parameter_or_default_time(
        *node, "delay_threshold", 10.0));
  }

  connections->path_request_pub = node->create_publisher<
    rmf_fleet_msgs::msg::PathRequest>(
    rmf_fleet_adapter::PathRequestTopicName, rclcpp::SystemDefaultsQoS());

      connections->path_intent_pub = node->create_publisher<
    traffic_manager::msg::PathIntent>(
    "/traffic_manager/path_intent", rclcpp::QoS(rclcpp::KeepLast(100)).reliable());

  connections->path_release_sub =
    node->create_subscription<traffic_manager::msg::PathRelease>(
    "/traffic_manager/path_release",
    rclcpp::QoS(rclcpp::KeepLast(100)).reliable(),
    [c = std::weak_ptr<Connections>(connections), node](
      traffic_manager::msg::PathRelease::UniquePtr msg)
    {
      const auto connections = c.lock();
      if (!connections || !msg)
        return;

      const auto rit = connections->robots.find(msg->robot_name);
      if (rit == connections->robots.end() || !rit->second)
      {
        RCLCPP_DEBUG(
          node->get_logger(),
          "[TrafficGate] PathRelease for unknown robot [%s] (not in fleet yet)",
          msg->robot_name.c_str());
        return;
      }

      rit->second->on_path_release(*msg);
    });

  RCLCPP_INFO(
    node->get_logger(),
    "[TrafficGate] Path gate enabled: PathIntent → /traffic_manager/path_intent, "
    "PathRelease ← /traffic_manager/path_release, PathRequest → %s",
    rmf_fleet_adapter::PathRequestTopicName.c_str());
  connections->mode_request_pub = node->create_publisher<
    rmf_fleet_msgs::msg::ModeRequest>(
    rmf_fleet_adapter::ModeRequestTopicName, rclcpp::SystemDefaultsQoS());

  connections->fleet_state_sub = node->create_subscription<
    rmf_fleet_msgs::msg::FleetState>(
    rmf_fleet_adapter::FleetStateTopicName,
    rclcpp::SystemDefaultsQoS(),
    [c = std::weak_ptr<Connections>(connections), fleet_name](
      const rmf_fleet_msgs::msg::FleetState::SharedPtr msg)
    {
      if (msg->name != fleet_name)
        return;

      const auto connections = c.lock();
      if (!connections)
        return;

      for (const auto& state : msg->robots)
      {
        // Attempt the add if this robot is not in the fleet yet. Unlike the
        // previous insert-then-add, a failed attempt leaves no entry behind,
        // so a robot that was briefly un-placeable (localisation still
        // settling, say) joins on a later state message instead of needing
        // the whole fleet adapter restarted.
        connections->try_add_robot(fleet_name, state);

        // Re-look-up rather than reusing an iterator: try_add_robot() and the
        // asynchronous add completion both mutate `robots`.
        FleetDriverRobotCommandHandlePtr command = nullptr;
        {
          const auto it = connections->robots.find(state.name);
          if (it != connections->robots.end())
            command = it->second;
        }

        if (command)
        {
          // We are ready to command this robot, so let's update its state
          command->update_state(state);
        }

        // Forward the payload state from the fleet driver into the fleet
        // adapter so it can gate delivery bids on load_status (2 = unloaded,
        // 4 = loaded, 0 = unknown).
        if (connections->fleet)
        {
          connections->fleet->update_robot_load_status(
            state.name, state.load_status);
          connections->fleet->update_robot_operating_mode(
            state.name, state.operating_mode);
        }
      }
    });

  return connections;
}

//==============================================================================
int main(int argc, char* argv[])
{
  RCLCPP_INFO(
    rclcpp::get_logger("main"),
    "Initializing RMF Fleet Adapter with %d arguments", argc);

  rclcpp::init(argc, argv);
  
  RCLCPP_INFO(
    rclcpp::get_logger("main"),
    "Creating fleet adapter instance");

  const auto adapter = rmf_fleet_adapter::agv::Adapter::make("fleet_adapter");
  if (!adapter)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("main"),
      "Failed to create fleet adapter");
    return 1;
  }

  RCLCPP_INFO(
    rclcpp::get_logger("main"),
    "Fleet adapter created successfully, initializing fleet connections");

  const auto fleet_connections = make_fleet(adapter);
  if (!fleet_connections)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("main"),
      "Failed to create fleet connections");
    return 1;
  }

  RCLCPP_INFO(adapter->node()->get_logger(), "Starting Fleet Adapter");
  RCLCPP_INFO(
    adapter->node()->get_logger(),
    "Fleet adapter initialization completed successfully");

  // Start running the adapter and wait until it gets stopped by SIGINT
  RCLCPP_INFO(
    adapter->node()->get_logger(),
    "Starting fleet adapter main loop");
  adapter->start().wait();

  RCLCPP_INFO(adapter->node()->get_logger(), "Closing Fleet Adapter");
  RCLCPP_INFO(
    adapter->node()->get_logger(),
    "Fleet adapter shutdown initiated");

  rclcpp::shutdown();
  RCLCPP_INFO(
    rclcpp::get_logger("main"),
    "Fleet adapter shutdown completed");
}
