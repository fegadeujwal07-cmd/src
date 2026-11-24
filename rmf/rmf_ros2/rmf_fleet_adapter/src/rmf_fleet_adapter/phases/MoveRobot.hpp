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

#ifndef SRC__RMF_FLEET_ADAPTER__PHASES__MOVEROBOT_HPP
#define SRC__RMF_FLEET_ADAPTER__PHASES__MOVEROBOT_HPP

#include "../LegacyTask.hpp"
#include "../agv/RobotContext.hpp"

#include <rmf_traffic_ros2/Time.hpp>

namespace rmf_fleet_adapter {
namespace phases {

namespace {
//==============================================================================
inline std::string destination(
  const rmf_traffic::agv::Plan::Waypoint& wp,
  const rmf_traffic::agv::Graph& graph)
{
  if (wp.graph_index().has_value())
    return rmf_task::standard_waypoint_name(graph, *wp.graph_index());

  std::ostringstream oss;
  oss << "(" << wp.position().block<2, 1>(0, 0).transpose() << ")";
  return oss.str();
}
} // anonymous namespace

struct MoveRobot
{
  class Action;

  class ActivePhase : public LegacyTask::ActivePhase
  {
  public:

    ActivePhase(
      agv::RobotContextPtr context,
      std::vector<rmf_traffic::agv::Plan::Waypoint> waypoints,
      rmf_traffic::PlanId plan_id,
      std::optional<rmf_traffic::Duration> tail_period);

    const rxcpp::observable<LegacyTask::StatusMsg>& observe() const override;

    rmf_traffic::Duration estimate_remaining_time() const override;

    void emergency_alarm(bool on) override;

    void cancel() override;

    const std::string& description() const override;

  private:

    agv::RobotContextPtr _context;
    std::string _description;
    std::shared_ptr<Action> _action;
    rxcpp::observable<LegacyTask::StatusMsg> _obs;
    rxcpp::subjects::subject<bool> _cancel_subject;
    std::optional<rmf_traffic::Duration> _tail_period;
  };

  class PendingPhase : public LegacyTask::PendingPhase
  {
  public:

    PendingPhase(
      agv::RobotContextPtr context,
      std::vector<rmf_traffic::agv::Plan::Waypoint> waypoints,
      rmf_traffic::PlanId plan_id,
      std::optional<rmf_traffic::Duration> tail_period);

    std::shared_ptr<LegacyTask::ActivePhase> begin() override;

    rmf_traffic::Duration estimate_phase_duration() const override;

    const std::string& description() const override;

  private:

    agv::RobotContextPtr _context;
    std::vector<rmf_traffic::agv::Plan::Waypoint> _waypoints;
    rmf_traffic::PlanId _plan_id;
    std::optional<rmf_traffic::Duration> _tail_period;
    std::string _description;
  };

  class Action : public std::enable_shared_from_this<Action>
  {
  public:

    Action(
      agv::RobotContextPtr& context,
      std::vector<rmf_traffic::agv::Plan::Waypoint>& waypoints,
      rmf_traffic::PlanId plan_id,
      std::optional<rmf_traffic::Duration> tail_period);

    template<typename Subscriber>
    void operator()(const Subscriber& s);

  private:

    agv::RobotContextPtr _context;
    std::vector<rmf_traffic::agv::Plan::Waypoint> _waypoints;
    rmf_traffic::PlanId _plan_id;
    std::optional<rmf_traffic::Duration> _tail_period;
    std::optional<rmf_traffic::Time> _last_tail_bump;
    std::size_t _next_path_index = 0;

    rclcpp::TimerBase::SharedPtr _update_timeout_timer;
    rclcpp::Time _last_update_rostime;
    // TODO(MXG): Make this timeout configurable by users
    rmf_traffic::Duration _update_timeout = std::chrono::seconds(10);
  };
};

template<typename Subscriber>
void MoveRobot::Action::operator()(const Subscriber& s)
{
  const auto command = _context->command();
  if (!command)
    return;

  _last_update_rostime = _context->node()->now();
  _update_timeout_timer = _context->node()->try_create_wall_timer(
    _update_timeout, [w = weak_from_this()]()
    {
      const auto self = w.lock();
      if (!self)
        return;

      const auto now = self->_context->node()->now();
      if (now < self->_last_update_rostime + self->_update_timeout)
      {
        // The simulation is paused or running slowly, so we should allow more
        // patience before assuming that there's been a timeout.
        return;
      }

      self->_last_update_rostime = now;

      // added for ECBS — stuck-detector replan disabled so replans come only
      // from ECBS / zone_manager (one-shot), not from this watchdog.
      // The RobotCommandHandle seems to have frozen up. We used to request a
      // replan here (stuck-detector). That is DISABLED: after ECBS closes lanes
      // / traffic_manager holds the robot, progress reports stop and this timer
      // fired every ~10 s → continuous Replanning requested (revision 2, 3, …)
      // even though ECBS only published one REPLAN. Hard rule: replan only from
      // ECBS / zone_manager at planning time, not from this watchdog.
      //
      // Trade-off: a truly dropped first path (old patrol.dispatch-7 case) will
      // not auto-recover here; cancel/retry the task instead.
      RCLCPP_WARN_THROTTLE(
        self->_context->node()->get_logger(),
        (*self->_context->node()->get_clock()),
        30000,
        "Robot [%s] command handle has not reported for %ld ms — stuck-detector "
        "replan suppressed (ECBS/zone one-shot replan only)",
        self->_context->requester_id().c_str(),
        std::chrono::duration_cast<std::chrono::milliseconds>(
          self->_update_timeout).count());
      // self->_context->request_replan();
    });
      // The RobotCommandHandle seems to have frozen up. Perhaps a bug in the
      // user's code has caused the RobotCommandHandle to drop the command. We
      // would normally request a replan.
      //
      // ---- STRICT ZONE_MANAGER REPLAN: this one is RE-ENABLED -------------
      // The other three internal replan triggers stay disabled (MODE_ADAPTER_ERROR
      // in full_control/main.cpp, negotiation failure in FleetUpdateHandle.cpp,
      // resume-after-interruption in GoToPlace.cpp). This one is back on because
      // it is the only recovery for a task whose FIRST path was rejected:
      //
      //   2026-08-25, patrol.dispatch-7 — RMF planned it 174 ms before
      //   patrol.dispatch-6 was marked complete, so the path was anchored where
      //   amr1 was predicted to finish rather than where it was (7.4 m out). The
      //   first-waypoint check rejected it, nothing re-issued a path, and the
      //   task sat in standby for 118 s until it was cancelled.
      //
      // Cost of having it on: request_replan() bumps path_revision, which
      // is_replan_of_published_order() reads as a genuine replan — cancelOrder
      // plus a new order. If the robot is not actually progressing (e.g. a
      // simulator reporting driving:false mid-route, so execution publishes an
      // empty path[]), this fires every 10 s and produces a continuous replan
      // loop with no zone_manager involvement.
    //   RCLCPP_WARN(
    //     self->_context->node()->get_logger(),
    //     "Robot [%s] command handle has not reported for %ld ms — requesting a "
    //     "replan (stuck-detector).",
    //     self->_context->requester_id().c_str(),
    //     std::chrono::duration_cast<std::chrono::milliseconds>(
    //       self->_update_timeout).count());
    //   self->_context->request_replan();
    // });

  std::string dispatch_task_id;
  if (const auto* tid = _context->current_task_id())
    dispatch_task_id = *tid;

  // Which leg of the task this path belongs to, and whether it is that leg's
  // original plan or a replan of it. Maintained by GoToPlace.
  const uint32_t phase_index = _context->phase_index();
  const uint32_t path_revision = _context->path_revision();

  _context->command()->follow_new_path(
    _waypoints,
    [s, w_action = weak_from_this(), r = _context->requester_id()](
      std::size_t path_index, rmf_traffic::Duration estimate)
    {
      const auto action = w_action.lock();
      if (!action)
        return;

      action->_last_update_rostime = action->_context->node()->now();
      action->_update_timeout_timer->reset();

      if (path_index == action->_waypoints.size()-1
      && estimate < std::chrono::seconds(1)
      && action->_tail_period.has_value())
      {
        const auto now = action->_context->now();
        if (!action->_last_tail_bump.has_value()
        || *action->_last_tail_bump + *action->_tail_period < now)
        {
          action->_last_tail_bump = now;
          action->_context->worker().schedule(
            [
              context = action->_context,
              bump = *action->_tail_period,
              plan_id = action->_plan_id
            ](
              const auto&)
            {
              if (const auto c = context->itinerary().cumulative_delay(plan_id))
              {
                context->itinerary().cumulative_delay(plan_id, *c + bump);
              }
            });
        }
      }

      if (path_index != action->_next_path_index)
      {
        action->_next_path_index = path_index;
        LegacyTask::StatusMsg msg;
        msg.state = LegacyTask::StatusMsg::STATE_ACTIVE;

        if (path_index < action->_waypoints.size())
        {
          msg.status = "Heading towards "
          + destination(
            action->_waypoints[path_index],
            action->_context->planner()->get_configuration().graph());
        }
        else
        {
          // TODO(MXG): This should really be a warning, but the legacy phase shim
          // does not have a way for us to specify a warning.
          msg.status = "[Bug] [MoveRobot] Current path index was specified as ["
          + std::to_string(path_index) + "] but that exceeds the limit of ["
          + std::to_string(action->_waypoints.size()-1) + "]";
        }

        // path_index is computed in estimation.cpp against TravelInfo::waypoints,
        // but consumed here against MoveRobot's own _waypoints. Those are the
        // same vector in normal operation — but only because follow_new_path()
        // installs them together, and nothing in this lambda can verify that.
        // An index that outruns this vector reaches
        // Plan::Waypoint::arrival_checkpoints() on garbage, and iterating the
        // returned vector dereferences a junk pointer: a silent SIGSEGV that
        // takes down the whole fleet adapter (observed 2026-08-21, faulting in
        // __normal_iterator<Plan::Checkpoint const*> at address 0x185).
        //
        // The status message above already detects the same condition and
        // reports it as "[Bug] [MoveRobot]", then falls straight through to
        // index anyway. Check before using it instead.
        if (path_index > 0 && path_index - 1 < action->_waypoints.size())
        {
          const auto& arrival =
          action->_waypoints[path_index - 1].arrival_checkpoints();

          for (const auto& c : arrival)
          {
            action->_context->itinerary()
            .reached(action->_plan_id, c.route_id, c.checkpoint_id);
          }
        }

        s.on_next(msg);
      }

      // >= not >: at _next_path_index == size, _waypoints[path_index] below is
      // one past the end. The original > let that exact case through.
      if (action->_next_path_index >= action->_waypoints.size())
      {
        return;
      }

      // Stale-plan guard. This MUST come before the checkpoint loop below.
      //
      // The loop reads action->_waypoints (which belong to THIS action's plan)
      // and uses the route_id / checkpoint_id it finds there to index
      // context->itinerary() (which is the CURRENT plan's itinerary). A replan
      // swaps the itinerary for the new plan's routes while _waypoints still
      // describes the old one, so those ids address a different — usually
      // shorter — itinerary. Trajectory::operator[] is unchecked, so an
      // out-of-range checkpoint_id dereferences a junk pointer and takes the
      // whole fleet adapter down with SIGSEGV.
      //
      // Confirmed from seven crashes on 2026-08-20/21, every one of them at the
      // identical offset 0x30d53 in librmf_traffic.so, which addr2line resolves
      // to rmf_traffic::Trajectory::operator[](unsigned long) const, faulting on
      // `mov 0x50(%rax),%rax` — i.e. *ordering[index].value->myself with index
      // past the end. Every crash landed within ~10 ms of a replan.
      //
      // This check already existed, immediately AFTER the loop, guarding only
      // the delay update. Its own comment states the invariant the loop depends
      // on, so it simply needs to run first: when the plan is stale, the
      // reached() calls below are meaningless anyway because they name routes
      // that are no longer in the itinerary.
      if (action->_plan_id != action->_context->itinerary().current_plan_id())
      {
        // If the current Plan ID of the itinerary does not match the Plan ID
        // of this action, then neither the checkpoints nor the delay below
        // refer to anything in the current itinerary.
        return;
      }

      const auto& target_wp = action->_waypoints[path_index];
      const auto& itinerary = action->_context->itinerary().itinerary();
      for (const auto& progress : target_wp.progress_checkpoints())
      {
        for (const auto& c : progress.checkpoints)
        {
          // Defence in depth: the plan id can still change between the check
          // above and here (the itinerary is written from the robot's worker
          // thread), and a matching plan id does not by itself prove the ids
          // are in range. Both container accesses below are unchecked, so
          // verify before indexing rather than after crashing.
          if (c.route_id >= itinerary.size())
          {
            continue;
          }

          const auto& c_trajectory = itinerary[c.route_id].trajectory();
          if (c.checkpoint_id >= c_trajectory.size())
          {
            continue;
          }

          const auto& c_wp = c_trajectory[c.checkpoint_id];

          if (estimate < target_wp.time() - c_wp.time())
          {
            action->_context->itinerary()
            .reached(action->_plan_id, c.route_id, c.checkpoint_id);
          }
        }
      }

      using namespace std::chrono_literals;
      const rmf_traffic::Time now = action->_context->now();
      const auto planned_time = target_wp.time();
      const auto newly_expected_arrival = now + estimate;
      const auto new_cumulative_delay = newly_expected_arrival - planned_time;
      action->_context->worker().schedule(
        [
          context = action->_context,
          plan_id = action->_plan_id,
          new_cumulative_delay
        ](const auto&)
        {
          context->itinerary().cumulative_delay(
            plan_id, new_cumulative_delay, 500ms);
        });
    },
    [s, w = weak_from_this()]()
    {
      if (const auto self = w.lock())
      {
        for (const auto& c : self->_waypoints.back().arrival_checkpoints())
        {
          self->_context->itinerary().reached(
            self->_plan_id, c.route_id, c.checkpoint_id);
        }

        LegacyTask::StatusMsg msg;
        msg.state = LegacyTask::StatusMsg::STATE_COMPLETED;
        msg.status = "move robot success";
        s.on_next(msg);
        s.on_completed();
      }
    },
    dispatch_task_id,
    phase_index,
    path_revision);
}

} // namespace phases
} // namespace rmf_fleet_adapter


#endif // SRC__RMF_FLEET_ADAPTER__PHASES__MOVEROBOT_HPP
