/*
 * Drop task registration for rmf_fleet_adapter.
 *
 * A Drop task is a single-phase task whose only job is to drive a robot that
 * is already loaded (load_status == 4) to a chosen waypoint and execute a
 * DROP action there. The DROP action itself is performed by the fleet's
 * execution layer (e.g. execution_full_control.py) when it observes the
 * resulting path request for a "drop" category task.
 *
 * RMF eligibility (load_status == 4 AND robot truly idle) is gated by the
 * fleet adapter's bid filter in FleetUpdateHandle::bid_notice_cb; the
 * Dispatcher fails the task with HTTP-400-style FailedToAssign when no
 * fleet bids (see Dispatcher.cpp's conclude_bid).
 */

#ifndef SRC__RMF_FLEET_ADAPTER__TASKS__DROP_HPP
#define SRC__RMF_FLEET_ADAPTER__TASKS__DROP_HPP

#include <rmf_task_sequence/Phase.hpp>

#include "../agv/internal_FleetUpdateHandle.hpp"

namespace rmf_fleet_adapter {
namespace tasks {

//==============================================================================
void add_drop(
  agv::TaskDeserialization& deserialization,
  agv::TaskActivation& activation,
  std::function<rmf_traffic::Time()> clock);

} // namespace tasks
} // namespace rmf_fleet_adapter

#endif // SRC__RMF_FLEET_ADAPTER__TASKS__DROP_HPP
