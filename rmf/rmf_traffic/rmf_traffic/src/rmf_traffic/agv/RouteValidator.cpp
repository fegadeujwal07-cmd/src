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

#include <rmf_traffic/agv/RouteValidator.hpp>
#include <rmf_traffic/DetectConflict.hpp>

namespace rmf_traffic {
namespace agv {

//==============================================================================
class ScheduleRouteValidator::Implementation
{
public:

  std::shared_ptr<const schedule::Viewer> shared_viewer;
  const schedule::Viewer* viewer;
  schedule::ParticipantId participant;
  Profile profile;

};

//==============================================================================
ScheduleRouteValidator::ScheduleRouteValidator(
  const schedule::Viewer& viewer,
  schedule::ParticipantId participant_id,
  Profile profile)
: _pimpl(rmf_utils::make_impl<Implementation>(
      Implementation{
        nullptr,
        &viewer,
        participant_id,
        std::move(profile)
      }))
{
  // Do nothing
}

//==============================================================================
ScheduleRouteValidator::ScheduleRouteValidator(
  std::shared_ptr<const schedule::Viewer> viewer,
  schedule::ParticipantId participant_id,
  Profile profile)
: _pimpl(rmf_utils::make_impl<Implementation>(
      Implementation{
        viewer,
        viewer.get(),
        participant_id,
        std::move(profile)
      }))
{
  // Do nothing
}

//==============================================================================
ScheduleRouteValidator& ScheduleRouteValidator::schedule_viewer(
  const schedule::Viewer& viewer)
{
  _pimpl->viewer = &viewer;
  return *this;
}

//==============================================================================
const schedule::Viewer& ScheduleRouteValidator::schedule_viewer() const
{
  return *_pimpl->viewer;
}

//==============================================================================
ScheduleRouteValidator& ScheduleRouteValidator::participant(
  const schedule::ParticipantId p)
{
  _pimpl->participant = p;
  return *this;
}

//==============================================================================
schedule::ParticipantId ScheduleRouteValidator::participant() const
{
  return _pimpl->participant;
}

//==============================================================================
std::optional<RouteValidator::Conflict>
ScheduleRouteValidator::find_conflict(const Route& route) const
{
  // TODO(MXG): Should we use a mutable Spacetime instance to avoid the
  // allocation here?
  schedule::Query::Spacetime spacetime;
  spacetime.query_timespan()
  .all_maps(false)
  .add_map(route.map())
  .set_lower_time_bound(*route.trajectory().start_time())
  .set_upper_time_bound(*route.trajectory().finish_time());

  const auto view = _pimpl->viewer->query(
    spacetime, schedule::Query::Participants::make_all());

  for (const auto& v : view)
  {
    if (v.participant == _pimpl->participant)
      continue;

    if (const auto conflict = rmf_traffic::DetectConflict::between(
        _pimpl->profile,
        route.trajectory(),
        route.check_dependencies(v.participant, v.plan_id, v.route_id),
        v.description.profile(),
        v.route->trajectory(),
        nullptr))
    {
      return Conflict{
        Dependency{
          v.participant,
          v.plan_id,
          v.route_id,
          v.route->trajectory().index_after(conflict->time)
        },
        conflict->time,
        v.route
      };
    }
  }

  return std::nullopt;
}

//==============================================================================
std::unique_ptr<RouteValidator> ScheduleRouteValidator::clone() const
{
  return std::make_unique<ScheduleRouteValidator>(*this);
}

} // namespace agv
} // namespace rmf_traffic
