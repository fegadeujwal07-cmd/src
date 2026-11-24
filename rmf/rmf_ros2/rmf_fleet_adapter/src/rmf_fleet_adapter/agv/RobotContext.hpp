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

#ifndef SRC__RMF_FLEET_ADAPTER__AGV__ROBOTCONTEXT_HPP
#define SRC__RMF_FLEET_ADAPTER__AGV__ROBOTCONTEXT_HPP

#include <rmf_fleet_adapter/agv/RobotCommandHandle.hpp>
#include <rmf_fleet_adapter/agv/RobotUpdateHandle.hpp>
#include <rmf_fleet_adapter/agv/FleetUpdateHandle.hpp>

#include <rmf_traffic/schedule/Participant.hpp>
#include <rmf_traffic/schedule/Mirror.hpp>

#include <rmf_task/State.hpp>
#include <rmf_task/Constraints.hpp>
#include <rmf_task/TaskPlanner.hpp>
#include <rmf_task/Activator.hpp>

#include <rclcpp/node.hpp>

#include <rmf_rxcpp/Publisher.hpp>
#include <rmf_rxcpp/Transport.hpp>

#include <rxcpp/rx-observable.hpp>

#include <unordered_set>

#include "Node.hpp"
#include "../Reporting.hpp"

namespace rmf_fleet_adapter {

// Forward declaration
class TaskManager;

namespace agv {

//==============================================================================
class RobotContext
  : public std::enable_shared_from_this<RobotContext>
{
public:

  /// Get a handle to the command interface of the robot. This may return a
  /// nullptr if the robot has disconnected and/or its command API is no longer
  /// available.
  std::shared_ptr<RobotCommandHandle> command();

  /// Create a RobotUpdateHandle that reports to this RobotContext
  std::shared_ptr<RobotUpdateHandle> make_updater();

  /// This is the robot's current (x, y, yaw) position.
  Eigen::Vector3d position() const;

  /// This is the map that the robot is on.
  const std::string& map() const;

  /// Get the current time
  rmf_traffic::Time now() const;

  /// Get a clock that can be used by task loggers
  std::function<rmf_traffic::Time()> clock() const;

  /// This is the current "location" of the robot, which can be used to initiate
  /// a planning job
  const std::vector<rmf_traffic::agv::Plan::Start>& location() const;

  /// Get a mutable reference to the schedule of this robot
  rmf_traffic::schedule::Participant& itinerary();

  /// Get a const-reference to the schedule of this robot
  const rmf_traffic::schedule::Participant& itinerary() const;

  using Mirror = rmf_traffic::schedule::Mirror;
  /// Get a const-reference to an interface that lets you get a snapshot of the
  /// schedule.
  const std::shared_ptr<const Mirror>& schedule() const;

  /// Get the schedule description of this robot
  const rmf_traffic::schedule::ParticipantDescription& description() const;

  /// Get the profile of this robot
  const std::shared_ptr<const rmf_traffic::Profile>& profile() const;

  /// Get the name of this robot
  const std::string& name() const;

  /// Get the group (fleet) of this robot
  const std::string& group() const;

  /// Get the requester ID to use for this robot when sending requests
  const std::string& requester_id() const;

  /// Get the navigation graph used by this robot
  const rmf_traffic::agv::Graph& navigation_graph() const;

  /// Get a mutable reference to the planner for this robot
  const std::shared_ptr<const rmf_traffic::agv::Planner>& planner() const;

  // ===========================================================================
  // Per-robot ("private") lane closures.
  //
  // By default every RobotContext shares one planner pointer with the fleet, so
  // FleetUpdateHandle::close_lanes() affects all robots at once. These methods
  // detach THIS robot onto its own planner instance so a lane can be closed for
  // one robot without disturbing the others (e.g. zone-conflict replanning).
  //
  // Invariant: a private planner always closes (fleet closures) U (private
  // closures) — see set_private_lane_closures(). FleetUpdateHandle re-applies
  // this whenever fleet-wide closures change.
  //
  // NOTE: private closures never expire on their own. The caller is responsible
  // for clearing them (conflict resolved / task ended / re-registration /
  // timeout), otherwise the robot permanently plans on a reduced graph.
  // ===========================================================================

  /// Detach this robot onto its own planner with the given lanes closed, in
  /// addition to any fleet-wide closures. No-op if the set is unchanged,
  /// unless \p force is true (used to re-apply after fleet closures change).
  void set_private_lane_closures(
    const std::unordered_set<std::size_t>& lanes,
    bool force = false);

  /// Re-attach this robot to the shared fleet planner.
  void clear_private_lane_closures();

  /// True while this robot is using a private planner.
  bool has_private_lane_closures() const;

  /// The private closure set (empty when attached to the fleet planner).
  const std::unordered_set<std::size_t>& private_lane_closures() const;

  /// Kept for DockRobot, PerformAction and RobotUpdateHandle::Unstable. Traffic
  /// negotiation has been removed from this fleet adapter, so holding this
  /// handle no longer changes any behavior.
  std::shared_ptr<void> be_stubborn();

  /// If anything is holding onto a be_stubborn handle, this will return true.
  bool is_stubborn() const;

  struct Empty {};
  const rxcpp::observable<Empty>& observe_replan_request() const;

  void request_replan();

  /// Get a reference to the rclcpp node
  const std::shared_ptr<Node>& node();

  /// const-qualified node()
  std::shared_ptr<const Node> node() const;

  /// Get a reference to the worker for this robot. Use this worker to observe
  /// callbacks that can modify the state of the robot.
  const rxcpp::schedulers::worker& worker() const;

  /// Get the maximum allowable delay for this robot
  rmf_utils::optional<rmf_traffic::Duration> maximum_delay() const;

  /// Set the maximum allowable delay for this robot
  RobotContext& maximum_delay(rmf_utils::optional<rmf_traffic::Duration> value);

  /// Get the task activator for this robot
  const rmf_task::ConstActivatorPtr& task_activator() const;

  const rmf_task::ConstParametersPtr& task_parameters() const;

  /// Set the state of this robot at the end of its current task
  RobotContext& current_task_end_state(const rmf_task::State& state);

  /// Get a mutable reference to the state of this robot at the end of its
  // current task
  const rmf_task::State& current_task_end_state() const;

  /// Make a way to get the state for this robot
  std::function<rmf_task::State()> make_get_state();

  /// Get the current task ID of the robot, or a nullptr if the robot is not
  /// performing any task
  const std::string* current_task_id() const;

  /// Set the current task ID of the robot, or give a nullopt if a task is not
  /// being performed.
  RobotContext& current_task_id(std::optional<std::string> id);

  /// 0-based index of the navigation leg that the robot is currently planning
  /// or following, within the active task. Reset when a new task is assigned.
  uint32_t phase_index() const;

  /// Number of replans that have been started for the current navigation leg.
  /// 0 means the path being followed is the leg's original plan.
  uint32_t path_revision() const;

  /// Note that a new navigation leg has begun. Advances phase_index (except
  /// for the first leg of a task, which is index 0) and resets path_revision.
  void begin_new_phase();

  /// Note that a replan has been started for the current navigation leg.
  /// Advances path_revision and leaves phase_index alone.
  void note_replan();

  /// Reset both counters. Called when a new task is assigned to this robot.
  void reset_phase_counters();

  /// Get the current battery state of charge
  double current_battery_soc() const;

  /// Set the current battery state of charge. Note: This function also
  /// publishes the battery soc via _battery_soc_publisher.
  RobotContext& current_battery_soc(const double battery_soc);

  std::size_t dedicated_charger_wp() const;

  // Get a reference to the battery soc observer of this robot.
  const rxcpp::observable<double>& observe_battery_soc() const;

  /// Charging signal reported by the robot (RobotState.charging_status):
  /// 1 = charging, 0 = not charging, -1 = unknown.
  int charging_status() const;

  /// Set the charging signal. Any value other than 0 or 1 is stored as -1.
  /// This also records the time of the update and publishes the value via
  /// _charging_status_publisher.
  RobotContext& charging_status(int status);

  /// Steady-clock time of the last charging_status() update, or nullopt if the
  /// robot has never reported one.
  std::optional<rmf_traffic::Time> charging_status_time() const;

  /// Get a reference to the charging status observer of this robot.
  const rxcpp::observable<int>& observe_charging_status() const;

  /// Get a mutable reference to the task planner for this robot
  const std::shared_ptr<const rmf_task::TaskPlanner>& task_planner() const;

  /// Set the task planner for this robot
  RobotContext& task_planner(
    const std::shared_ptr<const rmf_task::TaskPlanner> task_planner);

  /// Set the current mode of the robot. This mode should correspond to a
  /// constant in the RobotMode message
  [[deprecated]]
  void current_mode(uint32_t mode);

  /// Return the current mode of the robot
  [[deprecated]]
  uint32_t current_mode() const;

  /// Set the current mode of the robot.
  /// Specify a valid string as specified in the robot_state.json schema
  void override_status(std::optional<std::string> status);

  /// Return the current mode of the robot
  std::optional<std::string> override_status() const;

  /// Set the action executor for requesting this robot to execute a
  /// PerformAction activity
  void action_executor(RobotUpdateHandle::ActionExecutor action_executor);

  /// Get the action executor for requesting this robot to execute a
  /// PerformAction activity
  RobotUpdateHandle::ActionExecutor action_executor() const;

  /// Get the task manager for this robot, if it exists.
  std::shared_ptr<TaskManager> task_manager();

  /// Return true if this robot is currently commissioned (available to accept
  /// new tasks).
  bool is_commissioned() const;

  void decommission();

  void recommission();

  Reporting& reporting();

  const Reporting& reporting() const;

private:
  friend class FleetUpdateHandle;
  friend class RobotUpdateHandle;
  friend class rmf_fleet_adapter::TaskManager;

  RobotContext(
    std::shared_ptr<RobotCommandHandle> command_handle,
    std::vector<rmf_traffic::agv::Plan::Start> _initial_location,
    rmf_traffic::schedule::Participant itinerary,
    std::shared_ptr<const Mirror> schedule,
    std::shared_ptr<std::shared_ptr<const rmf_traffic::agv::Planner>> planner,
    rmf_task::ConstActivatorPtr activator,
    rmf_task::ConstParametersPtr parameters,
    std::shared_ptr<Node> node,
    const rxcpp::schedulers::worker& worker,
    rmf_utils::optional<rmf_traffic::Duration> maximum_delay,
    rmf_task::State state,
    std::shared_ptr<const rmf_task::TaskPlanner> task_planner);

  /// Set the task manager for this robot. This should only be called in the
  /// TaskManager::make function.
  void _set_task_manager(std::shared_ptr<TaskManager> mgr);

  std::weak_ptr<RobotCommandHandle> _command_handle;
  std::vector<rmf_traffic::agv::Plan::Start> _location;
  rmf_traffic::schedule::Participant _itinerary;
  std::shared_ptr<const Mirror> _schedule;
  std::shared_ptr<std::shared_ptr<const rmf_traffic::agv::Planner>> _planner;
  // Saved shared (fleet) planner pointer while this robot is detached onto a
  // private planner. Null when attached to the fleet planner.
  std::shared_ptr<std::shared_ptr<const rmf_traffic::agv::Planner>>
  _fleet_planner;
  // Lanes closed for this robot only (empty when not detached).
  std::unordered_set<std::size_t> _private_closures;
  rmf_task::ConstActivatorPtr _task_activator;
  rmf_task::ConstParametersPtr _task_parameters;
  std::shared_ptr<const rmf_traffic::Profile> _profile;

  std::shared_ptr<void> _stubbornness;

  rxcpp::subjects::subject<Empty> _replan_publisher;
  rxcpp::observable<Empty> _replan_obs;

  std::shared_ptr<Node> _node;
  rxcpp::schedulers::worker _worker;
  rmf_utils::optional<rmf_traffic::Duration> _maximum_delay;
  std::string _requester_id;

  /// Always call the current_battery_soc() setter to set a new value
  double _current_battery_soc = 1.0;
  std::size_t _charger_wp;
  rxcpp::subjects::subject<double> _battery_soc_publisher;
  rxcpp::observable<double> _battery_soc_obs;
  /// Always call the charging_status() setter to set a new value
  int _charging_status = -1;
  std::optional<rmf_traffic::Time> _charging_status_time;
  rxcpp::subjects::subject<int> _charging_status_publisher;
  rxcpp::observable<int> _charging_status_obs;
  rmf_task::State _current_task_end_state;
  std::optional<std::string> _current_task_id;

  /// Navigation leg counters, stamped onto each PathRequest so that consumers
  /// can tell an initial path from a replanned one. Maintained by GoToPlace,
  /// which is the only event that knows the difference.
  uint32_t _phase_index = 0;
  uint32_t _path_revision = 0;
  bool _phase_started = false;

  std::shared_ptr<const rmf_task::TaskPlanner> _task_planner;
  std::weak_ptr<TaskManager> _task_manager;

  bool _commissioned = true;

  // Mode value for RobotMode message
  uint32_t _current_mode;
  std::optional<std::string> _override_status;

  RobotUpdateHandle::ActionExecutor _action_executor;
  Reporting _reporting;
};

using RobotContextPtr = std::shared_ptr<RobotContext>;
using ConstRobotContextPtr = std::shared_ptr<const RobotContext>;

//==============================================================================
struct GetContext
{
  RobotContextPtr value;
};

} // namespace agv
} // namespace rmf_fleet_adapter

#endif // SRC__RMF_FLEET_ADAPTER__AGV__ROBOTCONTEXT_HPP
