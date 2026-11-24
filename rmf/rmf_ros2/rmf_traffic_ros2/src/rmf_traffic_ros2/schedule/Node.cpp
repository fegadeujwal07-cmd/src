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

#include "internal_Node.hpp"

#include <cstring>

#include <rmf_traffic_ros2/Route.hpp>
#include <rmf_traffic_ros2/StandardNames.hpp>
#include <rmf_traffic_ros2/Time.hpp>
#include <rmf_traffic_ros2/Trajectory.hpp>
#include <rmf_traffic_ros2/schedule/Itinerary.hpp>
#include <rmf_traffic_ros2/schedule/Query.hpp>
#include <rmf_traffic_ros2/schedule/Patch.hpp>
#include <rmf_traffic_ros2/schedule/Writer.hpp>
#include <rmf_traffic_ros2/schedule/ParticipantDescription.hpp>
#include <rmf_traffic_ros2/schedule/Inconsistencies.hpp>
#include <rmf_traffic_ros2/schedule/ScheduleIdentity.hpp>

#include <iostream>
#include <unordered_map>
#include <uuid/uuid.h>

namespace rmf_traffic_ros2 {
namespace schedule {

namespace {
//==============================================================================
ScheduleNode::ScheduleId generate_node_id()
{
  uuid_t raw_uuid;
  uuid_generate(raw_uuid);
  // According to these docs, the size of the uuid string will be
  // 36 bytes + '\n' so we make a buffer of 37.
  char uuid[37];
  uuid_unparse(raw_uuid, uuid);
  const rclcpp::Time time(
    std::chrono::system_clock::now().time_since_epoch().count());

  return rmf_traffic_msgs::build<ScheduleNode::ScheduleId>()
    .node_uuid(std::string(uuid))
    .timestamp(time);
}

//==============================================================================
std::string generate_schedule_node_name(const ScheduleNode::ScheduleId& id)
{
  std::string uuid_underscore = id.node_uuid;
  for (auto& c : uuid_underscore)
  {
    if (c == '-')
      c = '_';
  }

  return "rmf_traffic_schedule_node_" + uuid_underscore;
}
}

//==============================================================================
// This constructor will _not_ automatically call the setup() method to finalise
// construction of the ScheduleNode object. setup() must be called manually.
ScheduleNode::ScheduleNode(
  ScheduleId id,
  std::shared_ptr<rmf_traffic::schedule::Database> database_,
  const rclcpp::NodeOptions& options,
  NoAutomaticSetup)
: Node(generate_schedule_node_name(id), options),
  heartbeat_qos_profile(1),
  database(std::move(database_))
{
  node_id = std::move(id);
  // Period, in milliseconds, for sending out a heartbeat signal to the monitor
  // node in the redundant pair
  declare_parameter<int>("heartbeat_period", 10000);
  heartbeat_period = std::chrono::milliseconds(
    get_parameter("heartbeat_period").as_int());

  // Participant registry location
  declare_parameter<std::string>(
    "log_file_location", ".rmf_schedule_node.yaml");

  // TODO(MXG): Expose a parameter for the update period
  // TODO(MXG): We can probably do something smarter to decide when to update
  // than a simple wall timer
  mirror_update_timer = create_wall_timer(
    std::chrono::milliseconds(10), [this]() { this->update_mirrors(); });
}

//==============================================================================
ScheduleNode::ScheduleNode(
  std::shared_ptr<rmf_traffic::schedule::Database> database_,
  const rclcpp::NodeOptions& options,
  NoAutomaticSetup)
: ScheduleNode(
    generate_node_id(),
    std::move(database_),
    options,
    no_automatic_setup)
{
  // Do nothing
}


//==============================================================================
// This constructor will automatically call the setup() method to finalise
// construction of the ScheduleNode object.
ScheduleNode::ScheduleNode(
  std::shared_ptr<rmf_traffic::schedule::Database> database_,
  QueryMap registered_queries_,
  const rclcpp::NodeOptions& options)
: ScheduleNode(
    generate_node_id(),
    database_,
    options,
    no_automatic_setup)
{
  setup(registered_queries_);
}

//==============================================================================
// This constructor will automatically call the setup() method to finalise
// construction of the ScheduleNode object.
ScheduleNode::ScheduleNode(
  const rclcpp::NodeOptions& options)
: ScheduleNode(  // Call the version that will automatically call setup(...)
    std::make_shared<rmf_traffic::schedule::Database>(),
    QueryMap(),
    options)
{
  // Do nothing
}

//==============================================================================
// This constructor will _not_ automatically call the setup() method to finalise
// construction of the ScheduleNode object. setup() must be called manually.
ScheduleNode::ScheduleNode(
  const rclcpp::NodeOptions& options,
  NoAutomaticSetup)
: ScheduleNode(  // Call the version that does not call setup(...)
    generate_node_id(),
    std::make_shared<rmf_traffic::schedule::Database>(),
    options,
    no_automatic_setup)
{
  // No setup(...) call here; it must be called manually
}

//==============================================================================
ScheduleNode::~ScheduleNode()
{
  // Do nothing
}

//==============================================================================
void ScheduleNode::setup(const QueryMap& queries)
{
  //Attempt to load/create participant registry.
  std::string log_file_name;
  get_parameter_or<std::string>(
    "log_file_location",
    log_file_name,
    ".rmf_schedule_node.yaml");

  // Re-instantiate any query update topics based on received queries
  make_mirror_update_topics(queries);

  try
  {
    auto participant_logger = std::make_unique<YamlLogger>(log_file_name);

    participant_registry =
      std::make_shared<ParticipantRegistry>(
      std::move(participant_logger),
      database);

    RCLCPP_INFO(get_logger(),
      "Successfully loaded logfile %s ",
      log_file_name.c_str());
  }
  catch (const std::exception& e)
  {
    RCLCPP_FATAL(get_logger(),
      "Failed to correctly load participant registry: %s\n",
      e.what());
    throw e;
  }

  setup_redundancy();
  setup_query_services();
  setup_participant_services();
  setup_changes_services();
  setup_itinerary_topics();
  setup_incosistency_pub();
  setup_cull_timer();
}

//==============================================================================
void ScheduleNode::setup_query_services()
{
  register_query_service =
    create_service<RegisterQuery>(
    rmf_traffic_ros2::RegisterQueryServiceName,
    [=](const std::shared_ptr<rmw_request_id_t> request_header,
    const RegisterQuery::Request::SharedPtr request,
    const RegisterQuery::Response::SharedPtr response)
    { this->register_query(request_header, request, response); });

  // TODO(MXG): We could expose the timing parameters to the user so the
  // frequency of cleanups can be customized.
  query_cleanup_timer =
    create_wall_timer(
    query_cleanup_period,
    [this]() { this->cleanup_queries(); });
}

//==============================================================================
void ScheduleNode::setup_participant_services()
{
  register_participant_service =
    create_service<RegisterParticipant>(
    rmf_traffic_ros2::RegisterParticipantSrvName,
    [=](const request_id_ptr request_header,
    const RegisterParticipant::Request::SharedPtr request,
    const RegisterParticipant::Response::SharedPtr response)
    { this->register_participant(request_header, request, response); });

  unregister_participant_service =
    create_service<UnregisterParticipant>(
    rmf_traffic_ros2::UnregisterParticipantSrvName,
    [=](const request_id_ptr request_header,
    const UnregisterParticipant::Request::SharedPtr request,
    const UnregisterParticipant::Response::SharedPtr response)
    { this->unregister_participant(request_header, request, response); });
}

//==============================================================================
void ScheduleNode::setup_changes_services()
{
  request_changes_service =
    create_service<RequestChanges>(
    rmf_traffic_ros2::RequestChangesServiceName,
    [=](const request_id_ptr request_header,
    const RequestChanges::Request::SharedPtr request,
    const RequestChanges::Response::SharedPtr response)
    { this->request_changes(request_header, request, response); });
}

//==============================================================================
void ScheduleNode::setup_itinerary_topics()
{
  const auto itinerary_qos =
    rclcpp::SystemDefaultsQoS()
    .reliable()
    .keep_last(100);

  itinerary_set_sub =
    create_subscription<ItinerarySet>(
    rmf_traffic_ros2::ItinerarySetTopicName,
    itinerary_qos,
    [=](const ItinerarySet::UniquePtr msg)
    {
      this->itinerary_set(*msg);
    });

  itinerary_extend_sub =
    create_subscription<ItineraryExtend>(
    rmf_traffic_ros2::ItineraryExtendTopicName,
    itinerary_qos,
    [=](const ItineraryExtend::UniquePtr msg)
    {
      this->itinerary_extend(*msg);
    });

  itinerary_delay_sub =
    create_subscription<ItineraryDelay>(
    rmf_traffic_ros2::ItineraryDelayTopicName,
    itinerary_qos,
    [=](const ItineraryDelay::UniquePtr msg)
    {
      this->itinerary_delay(*msg);
    });

  itinerary_reached_sub =
    create_subscription<ItineraryReached>(
    rmf_traffic_ros2::ItineraryReachedTopicName,
    itinerary_qos,
    [=](const ItineraryReached::UniquePtr msg)
    {
      this->itinerary_reached(*msg);
    });

  itinerary_clear_sub =
    create_subscription<ItineraryClear>(
    rmf_traffic_ros2::ItineraryClearTopicName,
    itinerary_qos,
    [=](const ItineraryClear::UniquePtr msg)
    {
      this->itinerary_clear(*msg);
    });
}

//==============================================================================
void ScheduleNode::setup_incosistency_pub()
{
  inconsistency_pub =
    create_publisher<InconsistencyMsg>(
    rmf_traffic_ros2::ScheduleInconsistencyTopicName,
    rclcpp::SystemDefaultsQoS().reliable());
}

//==============================================================================
void ScheduleNode::setup_cull_timer()
{
  cull_timer = create_wall_timer(
    std::chrono::minutes(1), [this]() { cull(); });
}

//==============================================================================
void ScheduleNode::setup_redundancy()
{
  start_heartbeat();

  const auto permanent_reliable_single_transient_local =
    rclcpp::SystemDefaultsQoS()
    .reliable()
    .keep_last(1)
    .transient_local();

  participants_info_pub =
    create_publisher<ParticipantsInfo>(
    rmf_traffic_ros2::ParticipantsInfoTopicName,
    permanent_reliable_single_transient_local);

  queries_info_pub =
    create_publisher<ScheduleQueries>(
    rmf_traffic_ros2::QueriesInfoTopicName,
    permanent_reliable_single_transient_local);

  startup_pub =
    create_publisher<ScheduleId>(
    rmf_traffic_ros2::ScheduleStartupTopicName,
    permanent_reliable_single_transient_local);
  startup_pub->publish(node_id);

  const auto reliable_10_transient_local =
    rclcpp::SystemDefaultsQoS()
    .reliable()
    .keep_last(10)
    .transient_local();

  startup_sub =
    create_subscription<ScheduleId>(
    rmf_traffic_ros2::ScheduleStartupTopicName,
    reliable_10_transient_local,
    [=](const ScheduleId::UniquePtr msg)
    {
      receive_startup_msg(*msg);
    });

  broadcast_queries();
  broadcast_participants();
}

//==============================================================================
void ScheduleNode::start_heartbeat()
{
  // Set up liveliness announcements of this node, powered by DDS[tm][r][c][rgb]
  heartbeat_qos_profile
  .liveliness(RMW_QOS_POLICY_LIVELINESS_AUTOMATIC)
  .liveliness_lease_duration(heartbeat_period)
  .deadline(heartbeat_period);

  heartbeat_pub = create_publisher<Heartbeat>(
    rmf_traffic_ros2::HeartbeatTopicName,
    heartbeat_qos_profile);
  RCLCPP_INFO(
    get_logger(),
    "Set up heartbeat on %s with liveliness lease duration of %ld ms "
    "and deadline of %ld ms",
    heartbeat_pub->get_topic_name(),
    heartbeat_period.count(),
    heartbeat_period.count());
}

//==============================================================================
void ScheduleNode::make_mirror_update_topics(const QueryMap& queries)
{
  // Delete any existing topics, just to be sure
  registered_queries.clear();

  for (const auto& [query_id, query] : queries)
  {
    register_query(query_id, query);
    RCLCPP_INFO(get_logger(), "Registering query ID %ld", query_id);
  }
}

//==============================================================================
void ScheduleNode::register_query(
  const std::shared_ptr<rmw_request_id_t>& /*request_header*/,
  const RegisterQuery::Request::SharedPtr& request,
  const RegisterQuery::Response::SharedPtr& response)
{
  rmf_traffic::schedule::Query new_query =
    rmf_traffic_ros2::convert(request->query);

  response->node_id = node_id;

  // Search for an existing query with the same search parameters
  for (auto& [existing_query_id, existing_query] : registered_queries)
  {
    if (existing_query.query == new_query)
    {
      RCLCPP_INFO(
        get_logger(),
        "A new mirror is tracking query ID [%ld]",
        existing_query_id);

      existing_query.last_registration_time = std::chrono::steady_clock::now();
      response->query_id = existing_query_id;
      broadcast_queries();
      return;
    }
  }

  // Find an unused query ID, store the query, and create a topic to publish
  // updates that match it.
  //
  // Note that this search may begin at query ID 0 if this is the first time
  // it is performed on a replacement schedule node. This is because the set
  // of queries will have been filled in from the original schedule node's
  // synchronised data, but last_query_id will have been initialised to zero
  // when the replacement was constructed. This is not a problem because a
  // search for the next available query ID does not need to be performed
  // until we actually need a new query ID, so performing it in the
  // constructor in advance would be unnecessary early optimisation.
  uint64_t query_id = last_query_id;
  uint64_t attempts = 0;
  do
  {
    ++query_id;
    ++attempts;
    if (attempts == std::numeric_limits<uint64_t>::max())
    {
      // I suspect a computer would run out of RAM before we reach this point,
      // but there's no harm in double-checking.
      response->error =
        "No more space for additional queries to be registered";
      RCLCPP_ERROR(
        get_logger(),
        "[ScheduleNode::register_query] %s",
        response->error.c_str());
      return;
    }
  } while (registered_queries.find(query_id) != registered_queries.end());

  response->query_id = query_id;
  register_query(query_id, new_query);
  last_query_id = query_id;
  RCLCPP_INFO(get_logger(), "Registered new query [%ld]", query_id);

  broadcast_queries();
}

//==============================================================================
void ScheduleNode::register_query(
  const uint64_t query_id,
  const rmf_traffic::schedule::Query& query)
{
  MirrorUpdateTopicPublisher update_publisher =
    create_publisher<MirrorUpdate>(
    rmf_traffic_ros2::QueryUpdateTopicNameBase + std::to_string(query_id),
    rclcpp::ServicesQoS().reliable().keep_last(5000));

  registered_queries.emplace(
    query_id,
    QueryInfo{
      query,
      std::move(update_publisher),
      std::nullopt,
      std::nullopt,
      std::chrono::steady_clock::now(),
      {}
    });
}

//==============================================================================
void ScheduleNode::cleanup_queries()
{
  bool any_erased = false;
  const auto now = std::chrono::steady_clock::now();
  auto it = registered_queries.begin();
  while (it != registered_queries.end())
  {
    if (it->second.publisher->get_subscription_count() == 0)
    {
      if (query_grace_period < now - it->second.last_registration_time)
      {
        // This query is considered deprecated, so we should erase it.
        // It's important that we use the post-increment operator here so that
        // we increment the iterator to its next value while erasing the element
        // that it used to point at.
        registered_queries.erase(it++);
        any_erased = true;
        continue;
      }
    }

    ++it;
  }

  if (any_erased)
    broadcast_queries();
}

//==============================================================================
void ScheduleNode::cull()
{
  const auto time = rmf_traffic_ros2::convert(now());
  {
    // Cull unnecessary data from the schedule
    std::lock_guard<std::mutex> lock(database_mutex);
    database->set_current_time(time);
    database->cull(time - std::chrono::hours(2));
  }
}

//==============================================================================
void ScheduleNode::broadcast_queries()
{
  ScheduleQueries msg;
  msg.node_id = node_id;

  for (const auto& registered_query: registered_queries)
  {
    msg.query_ids.push_back(registered_query.first);

    const rmf_traffic::schedule::Query& original =
      registered_queries.at(registered_query.first).query;

    msg.queries.emplace_back(rmf_traffic_ros2::convert(original));
  }

  queries_info_pub->publish(msg);
}

//==============================================================================
void ScheduleNode::register_participant(
  const request_id_ptr& /*request_header*/,
  const RegisterParticipant::Request::SharedPtr& request,
  const RegisterParticipant::Response::SharedPtr& response)
{
  std::unique_lock<std::mutex> lock(database_mutex);

  // TODO(MXG): Use try on every database operation
  try
  {
    const auto registration = participant_registry
      ->add_or_retrieve_participant(
      rmf_traffic_ros2::convert(request->description));

    using Response = rmf_traffic_msgs::srv::RegisterParticipant::Response;

    *response =
      rmf_traffic_msgs::build<Response>()
      .participant_id(registration.id())
      .last_itinerary_version(registration.last_itinerary_version())
      .last_plan_id(registration.last_plan_id())
      .next_storage_base(registration.next_storage_base())
      .error("");

    RCLCPP_INFO(
      get_logger(),
      "Registered participant [%ld] named [%s] owned by [%s]",
      response->participant_id,
      request->description.name.c_str(),
      request->description.owner.c_str());

    broadcast_participants();
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(
      get_logger(),
      "Failed to register participant [%s] owned by [%s]: %s",
      request->description.name.c_str(),
      request->description.owner.c_str(),
      e.what());
    response->error = e.what();
  }
}

//==============================================================================
void ScheduleNode::unregister_participant(
  const request_id_ptr& /*request_header*/,
  const UnregisterParticipant::Request::SharedPtr& request,
  const UnregisterParticipant::Response::SharedPtr& response)
{
  std::unique_lock<std::mutex> lock(database_mutex);

  const auto& p = database->get_participant(request->participant_id);
  if (!p)
  {
    response->error =
      "Failed to unregister participant ["
      + std::to_string(request->participant_id) + "] because no "
      "participant has that ID";
    response->confirmation = false;

    RCLCPP_ERROR(get_logger(), "%s", response->error.c_str());
    return;
  }

  try
  {
    // We need to copy this data before the participant is unregistered, because
    // unregistering it will invalidate the pointer p.
    const std::string name = p->name();
    const std::string owner = p->owner();

    auto version = database->itinerary_version(request->participant_id);
    database->clear(request->participant_id, version);
    response->confirmation = true;

    RCLCPP_INFO(
      get_logger(),
      "Unregistered participant [%ld] named [%s] owned by [%s]",
      request->participant_id,
      name.c_str(),
      owner.c_str());

    broadcast_participants();
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(
      get_logger(),
      "Failed to unregister participant [%ld]: %s",
      request->participant_id,
      e.what());
    response->error = e.what();
    response->confirmation = false;
  }
}

//==============================================================================
void ScheduleNode::broadcast_participants()
{
  ParticipantsInfo msg;
  msg.node_id = node_id;

  for (const auto& id: database->participant_ids())
  {
    SingleParticipantInfo participant;
    participant.id = id;
    participant.description = rmf_traffic_ros2::convert(
      *database->get_participant(id));
    msg.participants.push_back(participant);
  }
  participants_info_pub->publish(msg);
}

//==============================================================================
void ScheduleNode::request_changes(
  [[maybe_unused]] const request_id_ptr& request_header,
  const RequestChanges::Request::SharedPtr& request,
  const RequestChanges::Response::SharedPtr& response)
{
  const auto query = registered_queries.find(request->query_id);
  if (query == registered_queries.end())
  {
    // Missing query update topic; something has gone very wrong.
    RCLCPP_ERROR(
      get_logger(),
      "[ScheduleNode::request_changes] "
      "Could not find a query registered with ID [%ld]",
      request->query_id);
    response->result = RequestChanges::Response::UNKNOWN_QUERY_ID;
  }
  else
  {
    auto& mirror_update_topic_info = query->second;
    // Tell the next update to send the changes since the requested version by
    // resetting the last sent version number to the requested version,
    // which may be std::nullopt if a full update is requested
    if (request->full_update)
    {
      mirror_update_topic_info.remediation_requests.insert(std::nullopt);
    }
    else
    {
      if (mirror_update_topic_info.last_sent_version.has_value())
      {
        try
        {
          if (rmf_utils::modular(request->version).less_than(
              *mirror_update_topic_info.last_sent_version))
          {
            mirror_update_topic_info
            .remediation_requests.insert(request->version);
          }
        }
        catch (const std::exception& e)
        {
          RCLCPP_ERROR(
            get_logger(),
            "[ScheduleNode::request_changes] Received suspicious request for "
            "changes starting at version [%lu]. This request will be ignored."
            "Error message: %s",
            request->version,
            e.what());
          response->result = RequestChanges::Response::ERROR;
          response->error = e.what();
          return;
        }
      }
    }

    response->result = RequestChanges::Response::REQUEST_ACCEPTED;
  }
}

//==============================================================================
void ScheduleNode::itinerary_set(const ItinerarySet& set)
{
  std::unique_lock<std::mutex> lock(database_mutex);
  assert(!set.itinerary.empty());
  try
  {
    database->set(
      set.participant,
      set.plan,
      rmf_traffic_ros2::convert(set.itinerary),
      set.storage_base,
      set.itinerary_version);

    publish_inconsistencies(set.participant);
  }
  catch (const std::exception& e)
  {
    RCLCPP_WARN(get_logger(), "Failed to set itinerary: %s", e.what());
  }
}

//==============================================================================
void ScheduleNode::itinerary_extend(const ItineraryExtend& extend)
{
  std::unique_lock<std::mutex> lock(database_mutex);
  try
  {
    database->extend(
      extend.participant,
      rmf_traffic_ros2::convert(extend.routes),
      extend.itinerary_version);

    publish_inconsistencies(extend.participant);
  }
  catch (const std::exception& e)
  {
    RCLCPP_WARN(get_logger(), "Failed to extend itinerary: %s", e.what());
  }
}

//==============================================================================
void ScheduleNode::itinerary_delay(const ItineraryDelay& delay)
{
  std::unique_lock<std::mutex> lock(database_mutex);
  const auto duration = rmf_traffic::Duration(delay.delay);

  static const auto delay_limit = std::chrono::hours(1);
  if (std::chrono::abs(duration) > delay_limit)
  {
    const auto& desc = database->get_participant(delay.participant);
    // TODO(MXG): Make this limit configurable
    RCLCPP_ERROR(
      get_logger(),
      "Delay of %fs for %s of group %s exceeds maximum delay limit of %fs.",
      rmf_traffic::time::to_seconds(duration),
      desc->name().c_str(),
      desc->owner().c_str(),
      rmf_traffic::time::to_seconds(delay_limit));

    return;
  }

  try
  {
    if (const auto d = database->get_cumulative_delay(delay.participant))
    {
      // TODO(MXG): Consider having this only issue the warning once per
      // new itinerary.
      const auto cumulative = *d + duration;
      if (std::chrono::abs(cumulative) > delay_limit)
      {
        const auto& desc = database->get_participant(delay.participant);
        RCLCPP_WARN(
          get_logger(),
          "Cumulative delay of %fs for %s of group %s exceeds maximum delay "
          "limit of %fs",
          rmf_traffic::time::to_seconds(cumulative),
          desc->name().c_str(),
          desc->owner().c_str(),
          rmf_traffic::time::to_seconds(delay_limit));
      }
    }

    database->delay(
      delay.participant,
      duration,
      delay.itinerary_version);

    publish_inconsistencies(delay.participant);
  }
  catch (const std::exception& e)
  {
    RCLCPP_WARN(get_logger(), "Failed to delay itinerary: %s", e.what());
  }
}

//==============================================================================
void ScheduleNode::itinerary_reached(const ItineraryReached& msg)
{
  std::unique_lock<std::mutex> lock(database_mutex);
  try
  {
    database->reached(
      msg.participant,
      msg.plan,
      msg.reached_checkpoints,
      msg.progress_version);

    // There is no risk of inconsistencies or conflicts occurring due to new
    // progress being reported, so we do not need to check for either.
  }
  catch (const std::exception& e)
  {
    RCLCPP_WARN(
      get_logger(), "Failed to update itinerary progress: %s", e.what());
  }
}

//==============================================================================
void ScheduleNode::itinerary_clear(const ItineraryClear& clear)
{
  std::unique_lock<std::mutex> lock(database_mutex);
  try
  {
    database->clear(clear.participant, clear.itinerary_version);

    publish_inconsistencies(clear.participant);
  }
  catch (const std::exception& e)
  {
    RCLCPP_WARN(get_logger(), "Failed to clear itinerary: %s", e.what());
  }
}

//==============================================================================
void ScheduleNode::publish_inconsistencies(
  rmf_traffic::schedule::ParticipantId id)
{
  // TODO(MXG): This approach is likely to send out a lot of redundant
  // inconsistency reports. We should try to be smarter about how
  // inconsistencies get reported.
  const auto it = database->inconsistencies().find(id);
  assert(it != database->inconsistencies().end());
  if (it->ranges.size() == 0)
    return;

  inconsistency_pub->publish(
    rmf_traffic_ros2::convert(*it, database->get_current_progress_version(id)));
}

//==============================================================================
void ScheduleNode::receive_startup_msg(const ScheduleId& msg)
{
  if (node_id.node_uuid == msg.node_uuid)
  {
    // Ignore the messages that are coming from ourself
    return;
  }

  if (need_reconnection(node_id, msg))
  {
    // Shut down this schedule node if a newer one has appeared. We use
    // std::cout here because I'm not sure if RCLCPP_INFO will get published
    // during a shutdown.
    std::cout << "Shutting down traffic schedule node because a new one ["
              << msg.node_uuid << "] has appeared." << std::endl;
    rclcpp::shutdown(this->get_node_base_interface()->get_context());
  }
  else if (node_id.timestamp == msg.timestamp)
  {
    // Another node has been launched, but its timestamp is tied with ours. We
    // should start racing against it.
    RCLCPP_WARN(
      get_logger(),
      "Two schedule nodes [%s] and [%s], have tied for priority at time %d:%d",
      node_id.node_uuid.c_str(),
      msg.node_uuid.c_str(),
      node_id.timestamp.sec,
      node_id.timestamp.nanosec);

    node_id.timestamp = now();
    startup_pub->publish(node_id);
  }
}

//==============================================================================
void ScheduleNode::update_mirrors()
{
  for (auto& [query_id, query_info] : registered_queries)
  {
    for (const auto request : query_info.remediation_requests)
    {
      const auto updated = update_query(
        query_info.publisher,
        query_info.query,
        request,
        true);

      if (updated)
      {
        const std::string starting_from = request.has_value() ?
          "version " + std::to_string(*request) : "the beginning";

        RCLCPP_INFO(
          get_logger(),
          "[ScheduleNode::update_mirrors] Sending remedial update starting "
          "from %s going to %lu for query %ld",
          starting_from.c_str(),
          database->latest_version(),
          query_id);
      }
    }
    query_info.remediation_requests.clear();

    if (query_info.last_checked_version == database->latest_version())
      continue;

    query_info.last_checked_version = database->latest_version();
    const auto updated = update_query(
      query_info.publisher,
      query_info.query,
      query_info.last_sent_version,
      false);

    if (updated)
    {
      // Update the latest version sent to this topic
      query_info.last_sent_version = database->latest_version();

      RCLCPP_DEBUG(
        get_logger(),
        "[ScheduleNode::update_mirrors] Updated query [%ld]",
        query_id);
    }
  }
}

//==============================================================================
bool ScheduleNode::update_query(
  const MirrorUpdateTopicPublisher& publisher,
  const rmf_traffic::schedule::Query& query,
  VersionOpt last_sent_version,
  bool is_remedial)
{
  const auto patch = database->changes(query, last_sent_version);

  if (!is_remedial && patch.size() == 0 && !patch.cull())
    return false;

  rmf_traffic_msgs::msg::MirrorUpdate msg;
  msg.node_id = node_id;
  msg.database_version = database->latest_version();
  msg.patch = rmf_traffic_ros2::convert(patch);
  msg.is_remedial_update = is_remedial;
  publisher->publish(msg);

  return true;
}

std::shared_ptr<rclcpp::Node> make_node(const rclcpp::NodeOptions& options)
{
  return std::make_shared<ScheduleNode>(options);
}

} // namespace schedule
} // namespace rmf_traffic_ros2
