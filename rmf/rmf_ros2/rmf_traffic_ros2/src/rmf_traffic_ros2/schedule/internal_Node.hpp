/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#ifndef SRC__RMF_TRAFFIC_SCHEDULE__SCHEDULENODE_HPP
#define SRC__RMF_TRAFFIC_SCHEDULE__SCHEDULENODE_HPP

#include <rmf_traffic/schedule/Database.hpp>

#include <rclcpp/node.hpp>

#include <rmf_traffic_msgs/msg/mirror_update.hpp>
#include <rmf_traffic_msgs/msg/participant.hpp>
#include <rmf_traffic_msgs/msg/participants.hpp>
#include <rmf_traffic_msgs/msg/schedule_queries.hpp>
#include <rmf_traffic_msgs/msg/schedule_identity.hpp>

#include <rmf_traffic_msgs/msg/itinerary_clear.hpp>
#include <rmf_traffic_msgs/msg/itinerary_delay.hpp>
#include <rmf_traffic_msgs/msg/itinerary_extend.hpp>
#include <rmf_traffic_msgs/msg/itinerary_set.hpp>
#include <rmf_traffic_msgs/msg/itinerary_reached.hpp>

#include <rmf_traffic_msgs/msg/schedule_inconsistency.hpp>

#include <rmf_traffic_msgs/msg/heartbeat.hpp>

#include <rmf_traffic_msgs/srv/register_query.hpp>
#include <rmf_traffic_msgs/srv/request_changes.hpp>
#include <rmf_traffic_msgs/srv/register_participant.hpp>
#include <rmf_traffic_msgs/srv/unregister_participant.hpp>

#include <rmf_traffic_ros2/schedule/ParticipantRegistry.hpp>

#include <rmf_utils/Modular.hpp>

#include <mutex>
#include <optional>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace rmf_traffic_ros2 {
namespace schedule {

using namespace std::chrono_literals;

//==============================================================================
class ScheduleNode : public rclcpp::Node
{
public:
  using QueryMap = std::unordered_map<uint64_t, rmf_traffic::schedule::Query>;
  using VersionOpt = std::optional<rmf_traffic::schedule::Version>;

  using ScheduleId = rmf_traffic_msgs::msg::ScheduleIdentity;
  ScheduleId node_id;

  static struct NoAutomaticSetup{} no_automatic_setup;

  ScheduleNode(
    ScheduleId id,
    std::shared_ptr<rmf_traffic::schedule::Database> database_,
    const rclcpp::NodeOptions& options,
    NoAutomaticSetup);

  ScheduleNode(
    std::shared_ptr<rmf_traffic::schedule::Database> database_,
    const rclcpp::NodeOptions& options,
    NoAutomaticSetup);

  ScheduleNode(
    std::shared_ptr<rmf_traffic::schedule::Database> database_,
    QueryMap registered_queries_,
    const rclcpp::NodeOptions& options);

  ScheduleNode(const rclcpp::NodeOptions& options);

  ScheduleNode(
    const rclcpp::NodeOptions& options,
    NoAutomaticSetup);

  ~ScheduleNode();

  virtual void setup(const QueryMap& queries);

  std::chrono::milliseconds heartbeat_period = 10s;
  rclcpp::QoS heartbeat_qos_profile;
  using Heartbeat = rmf_traffic_msgs::msg::Heartbeat;
  using HeartbeatPub = rclcpp::Publisher<Heartbeat>;
  HeartbeatPub::SharedPtr heartbeat_pub;

  virtual void setup_redundancy();
  virtual void start_heartbeat();

  using request_id_ptr = std::shared_ptr<rmw_request_id_t>;

  using RegisterQuery = rmf_traffic_msgs::srv::RegisterQuery;
  using RegisterQueryService = rclcpp::Service<RegisterQuery>;

  virtual void register_query(
    const request_id_ptr& request_header,
    const RegisterQuery::Request::SharedPtr& request,
    const RegisterQuery::Response::SharedPtr& response);

  void register_query(
    uint64_t query_id,
    const rmf_traffic::schedule::Query& query);

  RegisterQueryService::SharedPtr register_query_service;

  // How often we should check the query topics to see if they have lost all
  // their subscribers.
  std::chrono::nanoseconds query_cleanup_period = std::chrono::minutes(5);

  // If a query has no subscribers, we will unregister it, unless it has
  // received a new registration request within this time period.
  std::chrono::nanoseconds query_grace_period = std::chrono::minutes(5);

  rclcpp::TimerBase::SharedPtr query_cleanup_timer;
  void cleanup_queries();

  rclcpp::TimerBase::SharedPtr cull_timer;
  void cull();

  virtual void setup_query_services();

  using RegisterParticipant = rmf_traffic_msgs::srv::RegisterParticipant;
  using RegisterParticipantSrv = rclcpp::Service<RegisterParticipant>;

  virtual void register_participant(
    const request_id_ptr& request_header,
    const RegisterParticipant::Request::SharedPtr& request,
    const RegisterParticipant::Response::SharedPtr& response);

  RegisterParticipantSrv::SharedPtr register_participant_service;

  using UnregisterParticipant = rmf_traffic_msgs::srv::UnregisterParticipant;
  using UnregisterParticipantSrv = rclcpp::Service<UnregisterParticipant>;

  virtual void unregister_participant(
    const request_id_ptr& request_header,
    const UnregisterParticipant::Request::SharedPtr& request,
    const UnregisterParticipant::Response::SharedPtr& response);

  UnregisterParticipantSrv::SharedPtr unregister_participant_service;

  virtual void setup_participant_services();

  using MirrorUpdate = rmf_traffic_msgs::msg::MirrorUpdate;
  using MirrorUpdateTopicPublisher = rclcpp::Publisher<MirrorUpdate>::SharedPtr;

  void add_query_topic(uint64_t query_id);
  void remove_query_topic(uint64_t query_id);

  void make_mirror_update_topics(const QueryMap& queries);

  using SingleParticipantInfo = rmf_traffic_msgs::msg::Participant;
  using ParticipantsInfo = rmf_traffic_msgs::msg::Participants;
  rclcpp::Publisher<ParticipantsInfo>::SharedPtr participants_info_pub;
  virtual void broadcast_participants();

  using ScheduleQuery = rmf_traffic_msgs::msg::ScheduleQuery;
  using ScheduleQueries = rmf_traffic_msgs::msg::ScheduleQueries;
  rclcpp::Publisher<ScheduleQueries>::SharedPtr queries_info_pub;
  virtual void broadcast_queries();

  using RequestChanges = rmf_traffic_msgs::srv::RequestChanges;
  using RequestChangesSrv = rclcpp::Service<RequestChanges>;
  void request_changes(
    const request_id_ptr& request_header,
    const RequestChanges::Request::SharedPtr& request,
    const RequestChanges::Response::SharedPtr& response);
  RequestChangesSrv::SharedPtr request_changes_service;

  virtual void setup_changes_services();

  using ItinerarySet = rmf_traffic_msgs::msg::ItinerarySet;
  void itinerary_set(const ItinerarySet& set);
  rclcpp::Subscription<ItinerarySet>::SharedPtr itinerary_set_sub;

  using ItineraryExtend = rmf_traffic_msgs::msg::ItineraryExtend;
  void itinerary_extend(const ItineraryExtend& extend);
  rclcpp::Subscription<ItineraryExtend>::SharedPtr itinerary_extend_sub;

  using ItineraryDelay = rmf_traffic_msgs::msg::ItineraryDelay;
  void itinerary_delay(const ItineraryDelay& delay);
  rclcpp::Subscription<ItineraryDelay>::SharedPtr itinerary_delay_sub;

  using ItineraryReached = rmf_traffic_msgs::msg::ItineraryReached;
  void itinerary_reached(const ItineraryReached& msg);
  rclcpp::Subscription<ItineraryReached>::SharedPtr itinerary_reached_sub;

  using ItineraryClear = rmf_traffic_msgs::msg::ItineraryClear;
  void itinerary_clear(const ItineraryClear& clear);
  rclcpp::Subscription<ItineraryClear>::SharedPtr itinerary_clear_sub;

  virtual void setup_itinerary_topics();

  using InconsistencyMsg = rmf_traffic_msgs::msg::ScheduleInconsistency;
  rclcpp::Publisher<InconsistencyMsg>::SharedPtr inconsistency_pub;
  void publish_inconsistencies(rmf_traffic::schedule::ParticipantId id);

  rclcpp::Subscription<ScheduleId>::SharedPtr startup_sub;
  rclcpp::Publisher<ScheduleId>::SharedPtr startup_pub;
  void receive_startup_msg(const ScheduleId& msg);

  virtual void setup_incosistency_pub();

  rclcpp::TimerBase::SharedPtr mirror_update_timer;
  void update_mirrors();
  bool update_query(
    const MirrorUpdateTopicPublisher& publisher,
    const rmf_traffic::schedule::Query& query,
    VersionOpt last_sent_version,
    bool is_remedial);

  // TODO(MXG): Consider using libguarded instead of a database_mutex
  std::mutex database_mutex;
  std::shared_ptr<rmf_traffic::schedule::Database> database;

  struct QueryInfo
  {
    rmf_traffic::schedule::Query query;
    MirrorUpdateTopicPublisher publisher;
    VersionOpt last_checked_version;
    VersionOpt last_sent_version;
    std::chrono::steady_clock::time_point last_registration_time;
    std::unordered_set<VersionOpt> remediation_requests;
  };
  using QueryInfoMap = std::unordered_map<uint64_t, QueryInfo>;

  std::size_t last_query_id = 0;
  QueryInfoMap registered_queries;

  std::shared_ptr<ParticipantRegistry> participant_registry;

  void setup_cull_timer();
};

} // namespace schedule
} // namespace rmf_traffic_ros2

#endif // SRC__RMF_TRAFFIC_SCHEDULE__SCHEDULENODE_HPP
