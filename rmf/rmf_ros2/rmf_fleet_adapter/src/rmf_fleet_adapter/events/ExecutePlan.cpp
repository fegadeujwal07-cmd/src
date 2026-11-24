/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include "ExecutePlan.hpp"
#include "LegacyPhaseShim.hpp"
#include "WaitUntil.hpp"

#include "../phases/MoveRobot.hpp"
#include "../phases/DockRobot.hpp"

#include <rmf_task_sequence/events/Bundle.hpp>

namespace rmf_fleet_adapter {
namespace events {

namespace {
//==============================================================================
using StandbyPtr = rmf_task_sequence::Event::StandbyPtr;
using UpdateFn = std::function<void()>;
using MakeStandby = std::function<StandbyPtr(UpdateFn)>;

//==============================================================================
struct LegacyPhaseWrapper
{
  LegacyPhaseWrapper(
    std::shared_ptr<LegacyTask::PendingPhase> phase_,
    rmf_traffic::Time time_,
    rmf_traffic::Dependencies dependencies_)
  : phase(std::move(phase_)),
    time(time_),
    dependencies(std::move(dependencies_))
  {
    // Do nothing
  }

  std::shared_ptr<LegacyTask::PendingPhase> phase;
  rmf_traffic::Time time;
  rmf_traffic::Dependencies dependencies;
};

using LegacyPhases = std::vector<LegacyPhaseWrapper>;

//==============================================================================
class EventPhaseFactory : public rmf_traffic::agv::Graph::Lane::Executor
{
public:

  using Lane = rmf_traffic::agv::Graph::Lane;

  EventPhaseFactory(
    agv::RobotContextPtr context,
    LegacyPhases& phases,
    rmf_traffic::Time event_start_time,
    const rmf_traffic::Dependencies& dependencies,
    bool& continuous)
  : _context(std::move(context)),
    _phases(phases),
    _event_start_time(event_start_time),
    _dependencies(dependencies),
    _continuous(continuous)
  {
    // Do nothing
  }

  void execute(const Dock& dock) final
  {
    _phases.emplace_back(
      std::make_shared<phases::DockRobot::PendingPhase>(
        _context, dock.dock_name()),
      _event_start_time, _dependencies);
    _continuous = false;
  }

  void execute(const DoorOpen&) final
  {
    _continuous = false;
  }

  void execute(const DoorClose&) final
  {
    _continuous = false;
  }

  void execute(const LiftSessionBegin&) final
  {
    _continuous = false;
  }

  void execute(const LiftMove&) final
  {
    _continuous = false;
  }

  void execute(const LiftDoorOpen&) final
  {
    _continuous = false;
  }

  void execute(const LiftSessionEnd&) final
  {
    _continuous = false;
  }

  void execute(const Wait&) final
  {
    // Do nothing
  }

private:
  agv::RobotContextPtr _context;
  LegacyPhases& _phases;
  rmf_traffic::Time _event_start_time;
  const rmf_traffic::Dependencies& _dependencies;
  bool& _continuous;
};


} // anonymous namespace

//==============================================================================
std::optional<ExecutePlan> ExecutePlan::make(
  agv::RobotContextPtr context,
  rmf_traffic::PlanId plan_id,
  rmf_traffic::agv::Plan plan,
  rmf_traffic::schedule::Itinerary full_itinerary,
  const rmf_task::Event::AssignIDPtr& event_id,
  rmf_task::events::SimpleEventStatePtr state,
  std::function<void()> update,
  std::function<void()> finished,
  std::optional<rmf_traffic::Duration> tail_period)
{
  std::optional<rmf_traffic::Time> finish_time_estimate;
  for (const auto& r : plan.get_itinerary())
  {
    const auto check = r.trajectory().back().time();
    if (!finish_time_estimate.has_value() || *finish_time_estimate < check)
      finish_time_estimate = check;
  }

  if (!finish_time_estimate.has_value())
  {
    // If this is empty then the entire plan is empty... that's not supposed to
    // happen...
    return std::nullopt;
  }

  std::vector<rmf_traffic::agv::Plan::Waypoint> waypoints =
    plan.get_waypoints();

  std::vector<rmf_traffic::agv::Plan::Waypoint> move_through;

  LegacyPhases legacy_phases;
  while (!waypoints.empty())
  {
    auto it = waypoints.begin();
    bool event_occurred = false;
    for (; it != waypoints.end(); ++it)
    {
      move_through.push_back(*it);

      if (it->event())
      {
        if (move_through.size() > 1)
        {
          legacy_phases.emplace_back(
            std::make_shared<phases::MoveRobot::PendingPhase>(
              context, move_through, plan_id, tail_period),
            it->time(), it->dependencies());
        }

        move_through.clear();
        bool continuous = true;
        EventPhaseFactory factory(
          context, legacy_phases, it->time(), it->dependencies(), continuous);
        it->event()->execute(factory);

        if (continuous)
        {
          // Have the next sequence of waypoints begin with the event waypoint
          // of this sequence.
          move_through.push_back(*it);
        }

        waypoints.erase(waypoints.begin(), it+1);
        event_occurred = true;
        break;
      }
    }

    if (move_through.size() > 1)
    {
      // If we have more than one waypoint to move through, then create a
      // moving phase.
      //
      // If we reach this point in the code and move_through is greater than 1,
      // then we have reached the end of the path, so there is definitely no
      // need for any dependencies.
      legacy_phases.emplace_back(
        std::make_shared<phases::MoveRobot::PendingPhase>(
          context, move_through, plan_id, tail_period),
        finish_time_estimate.value(), rmf_traffic::Dependencies{});
    }

    if (!event_occurred)
    {
      // If no event occurred on this loop, then we have reached the end of the
      // waypoint sequence, and we should simply clear it out.
      waypoints.clear();
    }
  }

  // Convert the legacy phases into task events.
  std::vector<MakeStandby> standbys;
  for (const auto& legacy : legacy_phases)
  {
    if (legacy.phase)
    {
      standbys.push_back(
        [legacy = legacy.phase, context, event_id](UpdateFn update)
        {
          return LegacyPhaseShim::Standby::make(
            legacy, context->worker(), context->clock(), event_id, update);
        });
    }
  }

  if (tail_period.has_value() && !legacy_phases.empty())
  {
    // A tail period was requested, so this is actually a ResponsiveWait action.
    // We will ensure that the task doesn't finish until the final time is
    // reached, even if the robot arrives at the final destination early.
    const auto wait_until_time = legacy_phases.back().time;
    standbys.push_back(
      [context, wait_until_time, event_id](UpdateFn update)
      -> rmf_task_sequence::Event::StandbyPtr
      {
        return WaitUntil::Standby::make(
          context, wait_until_time, event_id, std::move(update));
      });
  }

  auto sequence = rmf_task_sequence::events::Bundle::standby(
    rmf_task_sequence::events::Bundle::Type::Sequence,
    standbys, state, std::move(update))->begin([]() {}, std::move(finished));

  std::size_t attempts = 0;
  while (!context->itinerary().set(plan_id, std::move(full_itinerary)))
  {
    // Some mysterious behavior has been happening where plan_ids are invalid.
    // We will attempt to catch that here and try to learn more about what
    // could be causing that, while allowing progress to continue.
    std::string task_id = "<none>";
    if (context->current_task_id())
      task_id = *context->current_task_id();

    RCLCPP_ERROR(
      context->node()->get_logger(),
      "Invalid plan_id [%lu] when current plan_id is [%lu] for [%s] in group "
      "[%s] while performing task [%s]. Please notify an RMF developer.",
      plan_id,
      context->itinerary().current_plan_id(),
      context->name().c_str(),
      context->group().c_str(),
      task_id.c_str());
    state->update_log().error(
      "Invalid plan_id [" + std::to_string(plan_id) + "] when current plan_id "
      "is [" + std::to_string(context->itinerary().current_plan_id()) + "] "
      "Please notify an RMF developer.");

    plan_id = context->itinerary().assign_plan_id();

    if (++attempts > 5)
    {
      RCLCPP_ERROR(
        context->node()->get_logger(),
        "Requesting replan for [%s] in group [%s] because plan is repeatedly "
        "being rejected while performing task [%s]",
        context->name().c_str(),
        context->group().c_str(),
        task_id.c_str());
      return std::nullopt;
    }
  }

  return ExecutePlan{
    std::move(plan),
    plan_id,
    finish_time_estimate.value(),
    std::move(sequence)
  };
}

} // namespace events
} // namespace rmf_fleet_adapter
