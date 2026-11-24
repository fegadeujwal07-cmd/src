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

#include "../phases/WaitForCharge.hpp"

#include "../events/GoToPlace.hpp"
#include "../events/LegacyPhaseShim.hpp"

#include "../agv/internal_FleetUpdateHandle.hpp"
#include "../TaskManager.hpp"

#include "ChargeBattery.hpp"

#include <rmf_task_sequence/events/Bundle.hpp>
#include <rmf_task_sequence/phases/SimplePhase.hpp>
#include <rmf_task_sequence/events/Placeholder.hpp>

#include <rmf_task_sequence/Task.hpp>

namespace rmf_fleet_adapter {
namespace tasks {

namespace {
//==============================================================================
// Used in place of "go to charger" when no charger is free at the moment the
// charge task starts. It cancels the whole task with the reason. (Failing only
// this step is not enough: the event sequence would still go on to "Wait for
// charging".)
class NoFreeCharger
{
public:
  using StatusMsg = LegacyTask::StatusMsg;

  class Active : public LegacyTask::ActivePhase
  {
  public:
    Active(std::string description)
    : _description(std::move(description))
    {
      StatusMsg initial;
      initial.state = StatusMsg::STATE_ACTIVE;
      initial.status = _description;
      _status_obs = _status_publisher.get_observable().start_with(initial);
    }

    const rxcpp::observable<StatusMsg>& observe() const final
    {
      return _status_obs;
    }

    rmf_traffic::Duration estimate_remaining_time() const final
    {
      return rmf_traffic::Duration(0);
    }

    void emergency_alarm(bool) final
    {
      // Nothing to do: the robot is not moving for this task.
    }

    void cancel() final
    {
      finish(StatusMsg::STATE_CANCELED);
    }

    const std::string& description() const final
    {
      return _description;
    }

    void finish(const uint32_t state)
    {
      if (_finished)
        return;

      _finished = true;
      StatusMsg msg;
      msg.state = state;
      msg.status = _description;
      _status_publisher.get_subscriber().on_next(msg);
      _status_publisher.get_subscriber().on_completed();
    }

  private:
    std::string _description;
    bool _finished = false;
    rxcpp::subjects::subject<StatusMsg> _status_publisher;
    rxcpp::observable<StatusMsg> _status_obs;
  };

  class Pending : public LegacyTask::PendingPhase
  {
  public:
    Pending(agv::RobotContextPtr context, std::string description)
    : _context(std::move(context)),
      _description(std::move(description))
    {
      // Do nothing
    }

    std::shared_ptr<LegacyTask::ActivePhase> begin() final
    {
      auto active = std::make_shared<Active>(_description);

      const std::string task_id = _context->current_task_id() ?
        *_context->current_task_id() : std::string();

      // Scheduled, not called here: this runs while the TaskManager is still
      // starting the task, so the task can only be cancelled afterwards.
      _context->worker().schedule(
        [w_context = std::weak_ptr<agv::RobotContext>(_context),
        w_active = std::weak_ptr<Active>(active),
        task_id,
        reason = _description](const auto&)
        {
          const auto context = w_context.lock();
          const auto active = w_active.lock();
          if (!context || !active)
            return;

          const auto mgr = context->task_manager();
          if (mgr && !task_id.empty() && mgr->cancel_task(task_id, {reason}))
            return;

          RCLCPP_ERROR(
            context->node()->get_logger(),
            "[CHARGE] Could not cancel charge task [%s] of robot [%s] after "
            "finding no free charger. Marking this step as failed.",
            task_id.c_str(), context->name().c_str());
          active->finish(StatusMsg::STATE_FAILED);
        });

      return active;
    }

    rmf_traffic::Duration estimate_phase_duration() const final
    {
      return rmf_traffic::Duration(0);
    }

    const std::string& description() const final
    {
      return _description;
    }

  private:
    agv::RobotContextPtr _context;
    std::string _description;
  };
};

//==============================================================================
// Pick the charger at the moment the charge task starts: the charger held for
// this task if it is still free, otherwise the next free one. The hold is
// updated (or created, for an automatic charge task) so that other robots see
// this charger as taken.
std::optional<std::size_t> choose_charger_at_start(
  const agv::RobotContextPtr& context,
  std::string& busy_detail)
{
  const auto mgr = context->task_manager();
  const auto fleet = mgr ? mgr->fleet_handle() : nullptr;
  if (!fleet)
    return context->dedicated_charger_wp();

  auto& impl = agv::FleetUpdateHandle::Implementation::get(*fleet);
  const auto* task_id = context->current_task_id();

  // Only the current task id is checked (no direct-queue lookup): this runs
  // inside TaskManager::_begin_next_task, which holds that robot's mutex.
  std::optional<std::size_t> held;
  const auto hold_it = impl.charger_holds.find(context->name());
  if (hold_it != impl.charger_holds.end() && task_id
    && *task_id == hold_it->second.task_id)
  {
    held = hold_it->second.charger_wp;
  }

  const auto choice = impl.choose_free_charger(context, held);
  if (!choice.charger_wp.has_value())
  {
    busy_detail = choice.busy_detail;
    RCLCPP_WARN(
      context->node()->get_logger(),
      "[CHARGE] Robot [%s] charge task [%s]: no free charger at start (%s)",
      context->name().c_str(), task_id ? task_id->c_str() : "",
      choice.busy_detail.c_str());
    return std::nullopt;
  }

  if (held.has_value() && *held != *choice.charger_wp)
  {
    RCLCPP_INFO(
      context->node()->get_logger(),
      "[CHARGE] Robot [%s] charge task [%s]: held charger [%s] is not used "
      "(no longer free, or the robot already stands on another free charger) "
      "— going to [%s]",
      context->name().c_str(), task_id ? task_id->c_str() : "",
      impl.waypoint_display_name(*held).c_str(),
      impl.waypoint_display_name(*choice.charger_wp).c_str());
  }
  else
  {
    RCLCPP_INFO(
      context->node()->get_logger(),
      "[CHARGE] Robot [%s] charge task [%s]: going to charger [%s]",
      context->name().c_str(), task_id ? task_id->c_str() : "",
      impl.waypoint_display_name(*choice.charger_wp).c_str());
  }

  if (task_id)
  {
    impl.charger_holds[context->name()] =
      agv::FleetUpdateHandle::Implementation::ChargerHold{
      *task_id, *choice.charger_wp};
  }

  return choice.charger_wp;
}
} // anonymous namespace

//==============================================================================
struct GoToChargerDescription
  : public rmf_task_sequence::events::Placeholder::Description
{
  GoToChargerDescription()
  : rmf_task_sequence::events::Placeholder::Description("Go to charger", "")
  {
    // Do nothing
  }

  static rmf_task_sequence::Event::StandbyPtr standby(
    const rmf_task_sequence::Event::AssignIDPtr& id,
    const std::function<rmf_task::State()>& get_state,
    const rmf_task::ConstParametersPtr& parameters,
    const GoToChargerDescription&,
    std::function<void()> update)
  {
    using GoToPlace = rmf_task_sequence::events::GoToPlace::Description;
    const auto state = get_state();
    const auto context = state.get<agv::GetContext>()->value;

    // Not always the dedicated charger: the charger held for this task if it
    // is still free, otherwise the next free one.
    std::string busy_detail;
    const auto charger_wp = choose_charger_at_start(context, busy_detail);
    if (!charger_wp.has_value())
    {
      return events::LegacyPhaseShim::Standby::make(
        std::make_shared<NoFreeCharger::Pending>(
          context, "All charger points are busy. " + busy_detail),
        context->worker(), context->clock(), id, std::move(update),
        "Go to charger");
    }

    return events::GoToPlace::Standby::make(
      id, get_state, parameters,
      *GoToPlace::make(*charger_wp),
      std::move(update));
  }

  static void add(rmf_task_sequence::Event::Initializer& initializer)
  {
    initializer.add<GoToChargerDescription>(
      [](
        const rmf_task_sequence::Event::AssignIDPtr& id,
        const std::function<rmf_task::State()>& get_state,
        const rmf_task::ConstParametersPtr& parameters,
        const GoToChargerDescription& description,
        std::function<void()> update) -> rmf_task_sequence::Event::StandbyPtr
      {
        return standby(
          id, get_state, parameters, description, std::move(update));
      },
      [](
        const rmf_task_sequence::Event::AssignIDPtr& id,
        const std::function<rmf_task::State()>& get_state,
        const rmf_task::ConstParametersPtr& parameters,
        const GoToChargerDescription& description,
        const nlohmann::json&,
        std::function<void()> update,
        std::function<void()> checkpoint,
        std::function<void()> finished) -> rmf_task_sequence::Event::ActivePtr
      {
        return standby(
          id, get_state, parameters, description, std::move(update))
        ->begin(std::move(checkpoint), std::move(finished));
      });
  }
};

//==============================================================================
struct WaitForChargeDescription
  : public rmf_task_sequence::events::Placeholder::Description
{
  WaitForChargeDescription()
  : rmf_task_sequence::events::Placeholder::Description(
      "Wait for charging", "")
  {
    // Do nothing
  }

  static rmf_task_sequence::Event::StandbyPtr standby(
    const rmf_task_sequence::Event::AssignIDPtr& id,
    const std::function<rmf_task::State()>& get_state,
    const rmf_task::ConstParametersPtr& parameters,
    const WaitForChargeDescription&,
    std::function<void()> update)
  {
    const auto state = get_state();
    const auto context = state.get<agv::GetContext>()->value;

    auto legacy = phases::WaitForCharge::make(
      context,
      parameters->battery_system(),
      context->task_planner()->configuration().constraints().recharge_soc());

    return events::LegacyPhaseShim::Standby::make(
      std::move(legacy), context->worker(), context->clock(), id,
      std::move(update));
  }

  static void add(rmf_task_sequence::Event::Initializer& initializer)
  {
    initializer.add<WaitForChargeDescription>(
      [](
        const rmf_task_sequence::Event::AssignIDPtr& id,
        const std::function<rmf_task::State()>& get_state,
        const rmf_task::ConstParametersPtr& parameters,
        const WaitForChargeDescription& description,
        std::function<void()> update) -> rmf_task_sequence::Event::StandbyPtr
      {
        return standby(
          id, get_state, parameters, description, std::move(update));
      },
      [](
        const rmf_task_sequence::Event::AssignIDPtr& id,
        const std::function<rmf_task::State()>& get_state,
        const rmf_task::ConstParametersPtr& parameters,
        const WaitForChargeDescription& description,
        const nlohmann::json&,
        std::function<void()> update,
        std::function<void()> checkpoint,
        std::function<void()> finished) -> rmf_task_sequence::Event::ActivePtr
      {
        return standby(
          id, get_state, parameters, description, std::move(update))
        ->begin(std::move(checkpoint), std::move(finished));
      });
  }
};

//==============================================================================
// TODO(MXG): Consider making the ChargeBatteryEvent description public so it
// can be incorporated into other task types
struct ChargeBatteryEventDescription
  : public rmf_task_sequence::events::Placeholder::Description
{
  ChargeBatteryEventDescription()
  : rmf_task_sequence::events::Placeholder::Description(
      "Charge Battery", "")
  {
    // Do nothing
  }
};

//==============================================================================
void add_charge_battery(
  rmf_task::Activator& task_activator,
  const rmf_task_sequence::Phase::ConstActivatorPtr& phase_activator,
  rmf_task_sequence::Event::Initializer& event_initializer,
  std::function<rmf_traffic::Time()> clock)
{
  using Bundle = rmf_task_sequence::events::Bundle;
  using Phase = rmf_task_sequence::phases::SimplePhase;
  using ChargeBattery = rmf_task::requests::ChargeBattery;

  auto private_initializer =
    std::make_shared<rmf_task_sequence::Event::Initializer>();

  WaitForChargeDescription::add(*private_initializer);
  GoToChargerDescription::add(*private_initializer);

  auto charge_battery_event_unfolder =
    [](const ChargeBatteryEventDescription&)
    {
      return Bundle::Description({
            std::make_shared<GoToChargerDescription>(),
            std::make_shared<WaitForChargeDescription>()
          }, Bundle::Sequence, "Charge Battery");
    };

  Bundle::unfold<ChargeBatteryEventDescription>(
    std::move(charge_battery_event_unfolder),
    event_initializer, private_initializer);

  auto charge_battery_task_unfolder =
    [](const rmf_task::requests::ChargeBattery::Description&)
    {
      rmf_task_sequence::Task::Builder builder;
      builder
      .add_phase(
        Phase::Description::make(
          std::make_shared<ChargeBatteryEventDescription>(),
          "Charge Battery", ""), {});

      return *builder.build("Charge Battery", "");
    };

  rmf_task_sequence::Task::unfold<ChargeBattery::Description>(
    std::move(charge_battery_task_unfolder),
    task_activator,
    phase_activator,
    std::move(clock));
}

//==============================================================================
void add_charge_requests(agv::TaskDeserialization& deserialization)
{
  // The schema is kept here instead of in schemas/: the schema headers are
  // generated from a file GLOB, so a new schema file would only be picked up
  // after a forced CMake re-configure. The dashboard sends an empty
  // description; RMF chooses the charger.
  static const nlohmann::json charge_schema = nlohmann::json::parse(R"({
    "$schema": "https://json-schema.org/draft/2020-12/schema",
    "$id": "https://raw.githubusercontent.com/open-rmf/rmf_ros2/main/rmf_fleet_adapter/schemas/task_description__charge.json",
    "title": "Charge Task Description",
    "description": "Manual charge task for one robot. It has no fields: the fleet adapter sends the robot to a free charger and waits until the robot reports that charging has finished.",
    "type": "object"
  })");

  deserialization.add_schema(charge_schema);
  auto validate_charge = deserialization.make_validator_shared(charge_schema);

  auto deserialize_charge = [](const nlohmann::json&) -> agv::DeserializedTask
    {
      return {rmf_task::requests::ChargeBattery::Description::make(), {}};
    };

  deserialization.task->add("charge", validate_charge, deserialize_charge);
}

} // namespace task
} // namespace rmf_fleet_adapter
