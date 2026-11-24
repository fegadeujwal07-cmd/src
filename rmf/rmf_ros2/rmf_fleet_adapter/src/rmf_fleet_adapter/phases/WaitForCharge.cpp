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

#include "WaitForCharge.hpp"

#include <algorithm>
#include <cstdio>

namespace rmf_fleet_adapter {
namespace phases {

namespace {
//==============================================================================
std::string battery_text(const double battery_soc)
{
  char buffer[16];
  std::snprintf(buffer, sizeof(buffer), "%.0f%%", 100.0 * battery_soc);
  return buffer;
}
} // anonymous namespace

//==============================================================================
auto WaitForCharge::Active::observe() const
-> const rxcpp::observable<StatusMsg>&
{
  return _status_obs;
}

//==============================================================================
rmf_traffic::Duration WaitForCharge::Active::estimate_remaining_time() const
{
  const double capacity = _battery_system.capacity();
  const double charging_current = _battery_system.charging_current();
  const double time_estimate =
    3600.0 * capacity * (_charge_to_soc - _context->current_battery_soc()) /
    charging_current;

  return rmf_traffic::time::from_seconds(std::max(0.0, time_estimate));
}

//==============================================================================
void WaitForCharge::Active::emergency_alarm(const bool)
{
  // Assume charging station is a holding point
}

//==============================================================================
void WaitForCharge::Active::cancel()
{
  RCLCPP_INFO(
    _context->node()->get_logger(),
    "Robot [%s] charging was cancelled at battery %s.",
    _context->name().c_str(),
    battery_text(_context->current_battery_soc()).c_str());

  _finish(
    StatusMsg::STATE_CANCELED,
    "Charging cancelled for [" + _context->name() + "] at battery "
    + battery_text(_context->current_battery_soc()));
}

//==============================================================================
const std::string& WaitForCharge::Active::description() const
{
  return _description;
}

//==============================================================================
WaitForCharge::Active::Active(
  agv::RobotContextPtr context,
  rmf_battery::agv::BatterySystem battery_system,
  double charge_to_soc,
  rmf_traffic::Time start_time)
: _context(std::move(context)),
  _battery_system(battery_system),
  _charge_to_soc(charge_to_soc),
  _start_time(start_time)
{
  _last_update_time = start_time;
  // No known value seen yet: the unknown_fail_after countdown starts now.
  _last_known_status_time = start_time;
  _initial_battery_soc = _context->current_battery_soc();
  _expected_charging_rate = (100.0 / (
      _battery_system.capacity() /
      _battery_system.charging_current()));

  _description = "Charging [" + _context->requester_id()
    + "] until the robot reports that charging has finished";

  StatusMsg initial_msg;
  initial_msg.state = StatusMsg::STATE_ACTIVE;
  initial_msg.status = "Waiting for [" + _context->name()
    + "] to report charging (battery "
    + battery_text(_context->current_battery_soc()) + ")";
  const auto now = _context->node()->now();
  initial_msg.start_time = now;
  initial_msg.end_time = now + estimate_remaining_time();

  _status_obs = _status_publisher
    .get_observable()
    .start_with(initial_msg);

  _context->current_mode(rmf_fleet_msgs::msg::RobotMode::MODE_CHARGING);
}

//==============================================================================
void WaitForCharge::Active::_publish_status(const std::string& status)
{
  if (_finished)
    return;

  StatusMsg msg;
  msg.state = StatusMsg::STATE_ACTIVE;
  msg.status = status;
  _status_publisher.get_subscriber().on_next(msg);
}

//==============================================================================
void WaitForCharge::Active::_finish(
  const uint32_t state,
  const std::string& status)
{
  if (_finished)
    return;

  _finished = true;
  if (_check_timer)
  {
    _check_timer->cancel();
    _check_timer.reset();
  }

  StatusMsg msg;
  msg.state = state;
  msg.status = status;
  _status_publisher.get_subscriber().on_next(msg);
  _status_publisher.get_subscriber().on_completed();
}

//==============================================================================
void WaitForCharge::Active::_evaluate()
{
  if (_finished)
    return;

  const auto now = std::chrono::steady_clock::now();
  const auto& name = _context->name();
  const int status = _context->charging_status();
  const auto status_time = _context->charging_status_time();
  const bool known = (status == 0 || status == 1)
    && status_time.has_value() && now - *status_time <= status_fresh_for;

  if (known)
  {
    _last_known_status_time = now;
  }
  else
  {
    if (now - _last_known_status_time >= unknown_fail_after)
    {
      RCLCPP_ERROR(
        _context->node()->get_logger(),
        "Robot [%s] charging status has been unknown for %ld seconds "
        "(powerSupply.charging missing, not true/false, or no fresh robot "
        "state). The charge task has failed.",
        name.c_str(),
        static_cast<long>(unknown_fail_after.count()));

      _finish(
        StatusMsg::STATE_FAILED,
        "Charging status of [" + name + "] is unknown for "
        + std::to_string(unknown_fail_after.count())
        + " seconds (robot must send charging true or false)");
    }
    return;
  }

  const double battery_soc = _context->current_battery_soc();
  if (status == 1)
  {
    if (!_seen_charging)
    {
      _seen_charging = true;
      RCLCPP_INFO(
        _context->node()->get_logger(),
        "Robot [%s] has started charging (battery %s).",
        name.c_str(), battery_text(battery_soc).c_str());
      _publish_status(
        "[" + name + "] is charging (started at battery "
        + battery_text(battery_soc) + ")");
    }
    return;
  }

  // status == 0
  if (_seen_charging)
  {
    RCLCPP_INFO(
      _context->node()->get_logger(),
      "Robot [%s] has finished charging (battery %s). The charge task is "
      "complete.",
      name.c_str(), battery_text(battery_soc).c_str());
    _finish(
      StatusMsg::STATE_COMPLETED,
      "[" + name + "] finished charging at battery "
      + battery_text(battery_soc));
    return;
  }

  if (!_warned_not_charging && now - _start_time >= not_charging_warning_after)
  {
    _warned_not_charging = true;
    RCLCPP_WARN(
      _context->node()->get_logger(),
      "Robot [%s] reached the charger %ld seconds ago but is not charging "
      "(powerSupply.charging is false). Still waiting.",
      name.c_str(),
      static_cast<long>(not_charging_warning_after.count()));
    _publish_status(
      "WARNING: [" + name + "] reached the charger but is not charging");
  }
}

//==============================================================================
std::shared_ptr<LegacyTask::ActivePhase> WaitForCharge::Pending::begin()
{
  const auto& now = std::chrono::steady_clock::now();

  auto active =
    std::shared_ptr<Active>(new Active(
        _context, _battery_system, _charge_to_soc, now));

  RCLCPP_INFO(
    _context->node()->get_logger(),
    "Robot [%s] has begun waiting at the charger. The charge task finishes "
    "when the robot reports charging true and then false.",
    _context->name().c_str());

  active->_charging_status_subscription = _context->observe_charging_status()
    .observe_on(rxcpp::identity_same_worker(_context->worker()))
    .subscribe(
    [a = active->weak_from_this()](const int)
    {
      if (const auto active = a.lock())
        active->_evaluate();
    });

  // The robot may stop sending states altogether, so the signal is also
  // checked on a timer, not only when a new value arrives.
  active->_check_timer = _context->node()->try_create_wall_timer(
    std::chrono::seconds(1),
    [a = active->weak_from_this(), worker = _context->worker()]()
    {
      worker.schedule(
        [a](const auto&)
        {
          if (const auto active = a.lock())
            active->_evaluate();
        });
    });

  // Progress log only; the battery level does not finish the phase.
  active->_battery_soc_subscription = _context->observe_battery_soc()
    .observe_on(rxcpp::identity_same_worker(_context->worker()))
    .subscribe(
    [a = active->weak_from_this()](const double battery_soc)
    {
      const auto active = a.lock();

      if (!active || active->_finished)
        return;

      const auto& now = std::chrono::steady_clock::now();
      if (std::chrono::seconds(60) <= now - active->_last_update_time)
      {
        const double delta_soc = battery_soc - active->_initial_battery_soc;
        const double elapsed_seconds =
        (now - active->_start_time).count() / 1e9;
        const double average_charging_rate =
        100.0 * delta_soc / (elapsed_seconds / 3600.0);

        RCLCPP_INFO(
          active->_context->node()->get_logger(),
          "Robot [%s] is still at the charger. Charging status: %d "
          "(1 = charging, 0 = not charging, -1 = unknown). The current "
          "battery percentage is %.1f%%, rising at an average of %.1f %%/hour "
          "(expected %.1f %%/hour).",
          active->_context->name().c_str(),
          active->_context->charging_status(),
          battery_soc * 100,
          average_charging_rate,
          active->_expected_charging_rate);

        active->_last_update_time = now;
      }

    });

  return active;
}

//==============================================================================
rmf_traffic::Duration WaitForCharge::Pending::estimate_phase_duration() const
{
  return rmf_traffic::time::from_seconds(_time_estimate);
}

//==============================================================================
const std::string& WaitForCharge::Pending::description() const
{
  return _description;
}

//==============================================================================
WaitForCharge::Pending::Pending(
  agv::RobotContextPtr context,
  rmf_battery::agv::BatterySystem battery_system,
  double charge_to_soc,
  double time_estimate)
: _context(std::move(context)),
  _battery_system(battery_system),
  _charge_to_soc(charge_to_soc),
  _time_estimate(time_estimate)
{
  _description = "Wait until [" + _context->name()
    + "] reports that charging has finished";
}

//==============================================================================
auto WaitForCharge::make(
  agv::RobotContextPtr context,
  rmf_battery::agv::BatterySystem battery_system,
  double charge_to_soc) -> std::unique_ptr<Pending>
{

  const double capacity = battery_system.capacity();
  const double charging_current = battery_system.charging_current();
  const double time_estimate = std::max(
    0.0,
    3600.0 * capacity * (charge_to_soc - context->current_battery_soc()) /
    charging_current);

  return std::unique_ptr<Pending>(
    new Pending(context, battery_system, charge_to_soc, time_estimate));
}

} // namespace phases
} // namespace rmf_fleet_adapter
