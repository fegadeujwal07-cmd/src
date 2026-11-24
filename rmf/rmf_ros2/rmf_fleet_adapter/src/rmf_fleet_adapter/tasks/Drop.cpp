/*
 * Drop task implementation.
 *
 * A Drop task drives an already-loaded robot (load_status == 4) to a chosen
 * waypoint and releases its pallet there. To make the task completion
 * sequence identical to delivery's drop phase, we build the task as a
 * single phase wrapping an rmf_task_sequence::events::DropOff::Description.
 *
 * The DropOff event is unfolded by add_delivery() into a Bundle of:
 *     [ GoToPlace(place), TransferItems(Unload) ]
 *
 * - GoToPlace          drives the robot to the chosen place.
 * - TransferItems::Unload publishes an rmf_ingestor_msgs::IngestorRequest
 *   for the chosen handler and then waits for a matching IngestorResult.
 *
 * The fleet driver (execution_full_control.py) publishes the IngestorResult
 * on /ingestor_results once the robot reports an empty loads[] array
 * (load_status 4 -> 2). Only then does RMF mark the phase complete and the
 * task as a whole complete - exactly the same sequence delivery uses for
 * its drop phase, eliminating the "robot at waypoint but pallet still on
 * top" race.
 *
 * Eligibility (load_status == 4 AND robot truly idle) is enforced earlier:
 *   - API server pre-validation
 *   - Fleet adapter Gate A + Gate C (in bid_notice_cb)
 *   - Dispatcher fail-fast (in conclude_bid) when no fleet bids
 */

#include "Drop.hpp"

#include <rmf_task/Payload.hpp>

#include <rmf_task_sequence/Task.hpp>
#include <rmf_task_sequence/phases/SimplePhase.hpp>
#include <rmf_task_sequence/events/DropOff.hpp>

#include <rmf_fleet_adapter/schemas/task_description__drop.hpp>

namespace rmf_fleet_adapter {
namespace tasks {

//==============================================================================
void add_drop(
  agv::TaskDeserialization& deserialization,
  agv::TaskActivation& /*activation*/,
  std::function<rmf_traffic::Time()> /*clock*/)
{
  using Phase = rmf_task_sequence::phases::SimplePhase;
  using DropOff = rmf_task_sequence::events::DropOff;

  deserialization.add_schema(schemas::task_description__drop);

  auto validate_drop =
    deserialization.make_validator_shared(schemas::task_description__drop);

  deserialization.consider_drop =
    std::make_shared<agv::FleetUpdateHandle::ConsiderRequest>();

  // Accept drop tasks by default. Operators can override via
  // FleetUpdateHandle::consider_drop_requests(...).
  *deserialization.consider_drop = [](const auto&, auto& confirm)
    {
      confirm.accept();
    };

  auto deserialize_drop =
    [
    place_deser = deserialization.place,
    consider = deserialization.consider_drop
    ](const nlohmann::json& msg) -> agv::DeserializedTask
    {
      if (!(*consider))
      {
        return {nullptr, {"Not accepting drop requests"}};
      }

      const auto place = place_deser(msg.at("place"));
      std::vector<std::string> errors = place.errors;
      if (!place.description.has_value())
      {
        return {nullptr, std::move(errors)};
      }

      agv::FleetUpdateHandle::Confirmation confirm;
      (*consider)(msg, confirm);
      errors.insert(
        errors.end(), confirm.errors().begin(), confirm.errors().end());

      if (!confirm.is_accepted())
      {
        return {nullptr, errors};
      }

      // Optional handler / ingestor target. Defaults to "ingestor1" to
      // match the convention used by the delivery task's dropoff phase.
      std::string handler = "ingestor1";
      const auto handler_it = msg.find("handler");
      if (handler_it != msg.end() && handler_it->is_string())
      {
        handler = handler_it->get<std::string>();
      }

      // A sentinel single-component payload so the unfolded
      // TransferItems(Unload) emits exactly one IngestorRequestItem on
      // /ingestor_requests. SKU/quantity are intentionally not surfaced
      // to the operator: a Drop task only cares about WHERE to drop, not
      // WHAT (the pallet is already on the robot). Downstream listeners
      // (if any) get a stable, predictable payload shape.
      rmf_task::Payload payload({
          rmf_task::Payload::Component("pallet", 1, "")
        });

      auto dropoff = DropOff::Description::make(
        *place.description,
        std::move(handler),
        std::move(payload),
        rmf_traffic::Duration(0));

      rmf_task_sequence::Task::Builder builder;
      builder.add_phase(Phase::Description::make(std::move(dropoff)), {});

      return {builder.build("Drop", ""), std::move(errors)};
    };

  deserialization.task->add("drop", validate_drop, deserialize_drop);
}

} // namespace tasks
} // namespace rmf_fleet_adapter
