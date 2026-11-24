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

#include "SearchForPath.hpp"

namespace rmf_fleet_adapter {
namespace jobs {

//==============================================================================
std::function<bool()> make_interrupter(
  std::shared_ptr<std::atomic_bool> interrupt_flag,
  std::optional<rmf_traffic::Time> deadline)
{
  const auto counter = std::make_shared<uint32_t>(0);
  return [interrupt_flag, deadline, counter]()
    {
      ++*counter;
      if (*counter > 20)
      {
        *counter = 0;
        // Only check these once in a while to reduce planning overhead
        if (*interrupt_flag)
          return true;

        const auto now = std::chrono::steady_clock::now();
        if (deadline <= now)
          return true;
      }

      return false;
    };
}

//==============================================================================
SearchForPath::SearchForPath(
  std::shared_ptr<const rmf_traffic::agv::Planner> planner,
  rmf_traffic::agv::Plan::StartSet starts,
  rmf_traffic::agv::Plan::Goal goal,
  std::shared_ptr<const rmf_traffic::schedule::Snapshot> schedule,
  rmf_traffic::schedule::ParticipantId participant_id,
  const std::shared_ptr<const rmf_traffic::Profile>& profile,
  std::optional<rmf_traffic::Duration> planning_time_limit)
: _planner(std::move(planner)),
  _starts(std::move(starts)),
  _goal(std::move(goal)),
  _schedule(std::move(schedule)),
  _participant_id(participant_id),
  _worker(rxcpp::schedulers::make_event_loop().create_worker())
{
  if (planning_time_limit.has_value())
  {
    const auto start_time = std::chrono::steady_clock::now();
    _deadline = start_time + *planning_time_limit;
  }
  auto greedy_options = _planner->get_default_options();
  greedy_options.validator(nullptr);
  greedy_options.interrupter(make_interrupter(_interrupt_flag, _deadline));

  // TODO(MXG): This is a gross hack to side-step the saturation issue that
  // happens when too many start conditions are given. That problem should be
  // fixed after we've improved the planner's heuristic.
  auto greedy_starts = _starts;
  if (greedy_starts.size() > 1)
    greedy_starts.erase(greedy_starts.begin()+1, greedy_starts.end());

  auto greedy_setup = _planner->setup(greedy_starts, _goal, greedy_options);
  const double base_cost = *greedy_setup.cost_estimate();
  greedy_setup.options().maximum_cost_estimate(_greedy_leeway*base_cost);

  auto compliant_options = _planner->get_default_options();

  // ===========================================================================
  // SCHEDULE VALIDATOR DISABLED — planner-internal re-planning turned off.
  // ---------------------------------------------------------------------------
  // With a validator set, DifferentialDrivePlanner runs a validator-driven
  // re-solve loop (kMaxAttempts = 8): every time ScheduleRouteValidator finds
  // the candidate itinerary conflicting with another robot's booked route, it
  // forbids the offending edge and calls cuOpt again. That loop could publish a
  // SECOND, completely different PathRequest for the same task within ~30 ms —
  // observed as amr3 receiving an east-bound 22-waypoint route and then a
  // west-bound 4-waypoint route, both tagged "revision: [0] (initial)".
  //
  // Because the planner re-plans BELOW the GoToPlace layer, note_replan() never
  // runs and path_revision does not increment, so the fleet adapter cannot
  // recognise the second path as a replan: the VDA5050 order kept the first
  // route while RMF's schedule (and RViz) held the second.
  //
  // Passing nullptr makes the compliant search behave like the greedy one: a
  // single cuOpt call, one route, one PathRequest per GoToPlace. Route changes
  // then come only from an explicit replan — i.e. /replan_candidate driving a
  // per-robot lane closure.
  //
  // TRADE-OFF: plans are no longer checked against the traffic schedule at
  // planning time, so robots will plan through each other's booked itineraries.
  // Conflict handling relies entirely on zone_manager + per-robot lane closures.
  //
  // TO RESTORE: delete the nullptr line below and uncomment the original block.
  // ===========================================================================
  compliant_options.validator(nullptr);
  // compliant_options.validator(
  //   rmf_traffic::agv::ScheduleRouteValidator::make(
  //     _schedule, _participant_id, *profile));
  // `profile` is only consumed by the commented-out validator above; silence
  // the unused-parameter warning without changing the constructor signature.
  (void)profile;

  compliant_options.maximum_cost_estimate(_compliant_leeway*base_cost);
  greedy_options.interrupter(make_interrupter(_interrupt_flag, _deadline));
  auto compliant_setup = _planner->setup(_starts, _goal, compliant_options);

  _greedy_job = std::make_shared<Planning>(std::move(greedy_setup));

  // ===========================================================================
  // GREEDY-ONLY MODE — the compliant search is not created.
  // ---------------------------------------------------------------------------
  // SearchForPath normally runs TWO independent planning jobs per GoToPlace:
  // a greedy one (no validator, single start) and a compliant one (all starts,
  // different cost leeway). Both can complete and each can drive a
  // follow_new_path(), which is why one task could produce two DIFFERENT routes
  // carrying the same path_revision — observed as a dense route (15 nodes) and
  // a sparse one (2-6 nodes) published seconds apart for the same dispatch id,
  // and as ~40 cuOpt calls for a single path publication.
  //
  // FindPath (services/detail/impl_FindPath.hpp) already PREFERS the greedy
  // result and only falls back to the compliant one when greedy fails, so not
  // creating the compliant job removes the duplicate plan without changing
  // which plan is normally chosen.
  //
  // TRADE-OFF: if the greedy search fails there is no second attempt to fall
  // back to — FindPath then returns the failed greedy progress and the caller
  // handles the failure. Combined with the disabled schedule validator above,
  // planning is now entirely schedule-unaware; conflict handling rests on
  // zone_manager + per-robot lane closures.
  //
  // TO RESTORE: uncomment the line below (and the validator block above).
  // ===========================================================================
  _compliant_job = nullptr;
  // _compliant_job = std::make_shared<Planning>(std::move(compliant_setup));
  (void)compliant_setup;

  // The two jobs coordinate their COMPLETION: on success the greedy handler
  // only calls s.on_completed() when _compliant_finished is already set (see
  // impl_SearchForPath.hpp, "if (search->_compliant_finished)"), because
  // normally the compliant subscription sets that flag. With no compliant job
  // that flag could never become true, so a SUCCESSFUL greedy plan fell through
  // to a bare `return` without ever completing the rxcpp subscription: FindPath
  // never delivered a result, GoToPlace waited forever, and full_control hung
  // (observed: 1749 cuOpt calls, 0 paths published, process killed with
  // SIGKILL after ignoring SIGINT and SIGTERM).
  //
  // Marking the compliant side as already finished is accurate here — there is
  // nothing left to wait for — and makes greedy success take exactly the same
  // completion branch it takes when the compliant job finishes first.
  //
  // (The greedy FAILURE path already calls s.on_completed() itself, so only
  // success was affected.)
  _compliant_finished = true;
}

//==============================================================================
void SearchForPath::interrupt()
{
  *_interrupt_flag = true;
}

//==============================================================================
Planning& SearchForPath::greedy()
{
  return *_greedy_job;
}

//==============================================================================
const Planning& SearchForPath::greedy() const
{
  return *_greedy_job;
}

//==============================================================================
bool SearchForPath::has_compliant() const
{
  return static_cast<bool>(_compliant_job);
}

//==============================================================================
Planning& SearchForPath::compliant()
{
  return *_compliant_job;
}

//==============================================================================
const Planning& SearchForPath::compliant() const
{
  return *_compliant_job;
}

//==============================================================================
void SearchForPath::set_cost_limit(double cost)
{
  _explicit_cost_limit = cost;
}

} // namespace jobs
} // namespace rmf_fleet_adapter
