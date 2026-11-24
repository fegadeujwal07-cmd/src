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

#include "Node.hpp"
// added for ECBS — needed to construct EcbsGate in Node::make.
#include "EcbsGate.hpp"

#include <rmf_fleet_adapter/StandardNames.hpp>
#include <rmf_traffic_ros2/StandardNames.hpp>

#include <rmf_traffic_ros2/Time.hpp>

namespace rmf_fleet_adapter {
namespace agv {

//==============================================================================
std::shared_ptr<Node> Node::make(
  rxcpp::schedulers::worker worker,
  const std::string& node_name,
  const rclcpp::NodeOptions& options)
{
  auto node = std::shared_ptr<Node>(
    new Node(std::move(worker), node_name, options));

  auto default_qos = rclcpp::SystemDefaultsQoS();
  default_qos.keep_last(100);

  node->_task_summary_pub =
    node->create_publisher<TaskSummary>(
    TaskSummaryTopicName, default_qos);

  node->_dispenser_request_pub =
    node->create_publisher<DispenserRequest>(
    DispenserRequestTopicName, default_qos);

  node->_dispenser_result_obs =
    node->create_observable<DispenserResult>(
    DispenserResultTopicName, default_qos);

  node->_dispenser_state_obs =
    node->create_observable<DispenserState>(
    DispenserStateTopicName, default_qos);

  node->_emergency_notice_obs =
    node->create_observable<EmergencyNotice>(
    rmf_traffic_ros2::EmergencyTopicName, default_qos);

  node->_ingestor_request_pub =
    node->create_publisher<IngestorRequest>(
    IngestorRequestTopicName, default_qos);

  node->_ingestor_result_obs =
    node->create_observable<IngestorResult>(
    IngestorResultTopicName, default_qos);

  node->_ingestor_state_obs =
    node->create_observable<IngestorState>(
    IngestorStateTopicName, default_qos);

  node->_fleet_state_pub =
    node->create_publisher<FleetState>(
    FleetStateTopicName, default_qos);

  // added for ECBS — gate created once with the adapter node.
  // Reads ROS params, builds EcbsGate (schedule cache + wait-only solver +
  // /to_zone_manager and /ecbs_replan_candidate publishers).
  EcbsGate::Config ecbs_cfg;
  // Master switch: false = pass-through (no solve / no publish).
  ecbs_cfg.enabled = node->declare_parameter<bool>("ecbs_gate_enabled", true);
  // Extra seconds after the other AMR clears a cell before we enter.
  ecbs_cfg.safety_margin_sec =
    node->declare_parameter<double>("ecbs_safety_margin_sec", 2.0);
  // Cap on cumulative wait for one AMR during one solve.
  ecbs_cfg.max_wait_sec =
    node->declare_parameter<double>("ecbs_max_wait_sec", 30.0);
  // Max conflict-resolve iterations before FailReplan.
  ecbs_cfg.max_iters =
    node->declare_parameter<int>("ecbs_max_iters", 20);
  // Same-cell match radius (m) when graph index is missing.
  ecbs_cfg.position_epsilon_m =
    node->declare_parameter<double>("ecbs_position_epsilon_m", 0.35);
  ecbs_cfg.external_evaluate_server =
    node->declare_parameter<bool>("ecbs_external_evaluate_server", false);
  // Alias this Transport/Node as an rclcpp::Node for publishers/logger.
  node->_ecbs_gate = EcbsGate::make(
    std::shared_ptr<rclcpp::Node>(node, static_cast<rclcpp::Node*>(node.get())),
    ecbs_cfg);

  auto transient_local_qos = rclcpp::SystemDefaultsQoS()
    .reliable()
    .transient_local()
    .keep_last(100);

  node->_task_api_request_obs =
    node->create_observable<ApiRequest>(
    TaskApiRequests, transient_local_qos);

  node->_task_api_response_pub =
    node->create_publisher<ApiResponse>(
    TaskApiResponses, transient_local_qos);

  return node;
}

//==============================================================================
Node::Node(
  rxcpp::schedulers::worker worker,
  const std::string& node_name,
  const rclcpp::NodeOptions& options)
: rmf_rxcpp::Transport(std::move(worker), node_name, options)
{
  // Do nothing
}

//==============================================================================
std::function<rmf_traffic::Time()> Node::clock() const
{
  return [self = shared_from_this()]()
    {
      return rmf_traffic_ros2::convert(self->now());
    };
}

//==============================================================================
rmf_traffic::Time Node::rmf_now() const
{
  return rmf_traffic_ros2::convert(now());
}

//==============================================================================
auto Node::task_summary() const -> const TaskSummaryPub&
{
  return _task_summary_pub;
}

//==============================================================================
auto Node::dispenser_request() const -> const DispenserRequestPub&
{
  return _dispenser_request_pub;
}

//==============================================================================
auto Node::dispenser_result() const -> const DispenserResultObs&
{
  return _dispenser_result_obs->observe();
}

//==============================================================================
auto Node::dispenser_state() const -> const DispenserStateObs&
{
  return _dispenser_state_obs->observe();
}

//==============================================================================
auto Node::emergency_notice() const -> const EmergencyNoticeObs&
{
  return _emergency_notice_obs->observe();
}

auto Node::ingestor_request() const -> const IngestorRequestPub&
{
  return _ingestor_request_pub;
}

//==============================================================================
auto Node::ingestor_result() const -> const IngestorResultObs&
{
  return _ingestor_result_obs->observe();
}

//==============================================================================
auto Node::ingestor_state() const -> const IngestorStateObs&
{
  return _ingestor_state_obs->observe();
}

//==============================================================================
auto Node::fleet_state() const -> const FleetStatePub&
{
  return _fleet_state_pub;
}

//==============================================================================
/// added for ECBS — shared gate (may be null only if make failed; normally set).
const std::shared_ptr<EcbsGate>& Node::ecbs_gate() const
{
  return _ecbs_gate;
}

//==============================================================================
auto Node::task_api_request() const -> const ApiRequestObs&
{
  return _task_api_request_obs->observe();
}

//==============================================================================
auto Node::task_api_response() const -> const ApiResponsePub&
{
  return _task_api_response_pub;
}

} // namespace agv
} // namespace rmf_fleet_adapter
