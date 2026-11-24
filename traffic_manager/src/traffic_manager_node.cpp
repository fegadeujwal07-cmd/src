#include "traffic_manager/traffic_manager_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <set>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "ecbs_planner/msg/evaluate_path_waypoint.hpp"
#include "ecbs_planner/msg/wait_dependency.hpp"
#include "ecbs_planner/srv/evaluate_path.hpp"
#include "rmf_fleet_msgs/msg/robot_mode.hpp"
#include "rmf_fleet_msgs/msg/robot_state.hpp"
#include "traffic_manager/path_horizon.hpp"
#include "traffic_manager/release_markers.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace traffic_manager
{
namespace
{
std::vector<ecbs_planner::msg::EvaluatePathWaypoint> to_ecbs_waypoints(
  const std::vector<rmf_fleet_msgs::msg::Location> & locations)
{
  std::vector<ecbs_planner::msg::EvaluatePathWaypoint> out;
  out.reserve(locations.size());
  for (std::size_t i = 0; i < locations.size(); ++i) {
    const auto & loc = locations[i];
    ecbs_planner::msg::EvaluatePathWaypoint wp;
    wp.t_sec =
      static_cast<double>(loc.t.sec) + 1e-9 * static_cast<double>(loc.t.nanosec);
    wp.x = loc.x;
    wp.y = loc.y;
    wp.yaw = loc.yaw;
    wp.level_name = loc.level_name;
    // ECBS cache indices are nav-graph vertices. Location.index == 0 means
    // "not set" here; -1 makes ECBS match this waypoint by x/y instead.
    wp.index = loc.index > 0 ? static_cast<int64_t>(loc.index) : int64_t{-1};
    out.push_back(wp);
  }
  return out;
}

/// Live peer base start: pose/leg_cursor, not path[0] (historical prefix
/// falsely looked like a base after amr3 had already left the shared leg).
int32_t live_base_start_index(const PathReleaseSession & peer)
{
  const auto & path = peer.intent.path;
  if (path.empty()) {
    return 0;
  }
  const int32_t last = static_cast<int32_t>(path.size()) - 1;
  int32_t start = 0;
  if (peer.leg_cursor_index >= 0) {
    start = peer.leg_cursor_index;
  } else {
    double best = 1e100;
    for (std::size_t i = 0; i < path.size(); ++i) {
      const double d = std::hypot(
        path[i].x - peer.last_eval_x, path[i].y - peer.last_eval_y);
      if (d < best) {
        best = d;
        start = static_cast<int32_t>(i);
      }
    }
  }
  if (start < 0) {
    start = 0;
  }
  if (start > last) {
    start = last;
  }
  return start;
}

/// Full-leg session whose cursor is at the tip — corridor cells vacated.
bool peer_live_base_finished(const PathReleaseSession & peer)
{
  if (peer.intent.path.empty() || peer.released_through_index < 0) {
    return true;
  }
  const int32_t last = static_cast<int32_t>(peer.intent.path.size()) - 1;
  if (peer.released_through_index < last) {
    return false;
  }
  return live_base_start_index(peer) >= last;
}

/// True when path indices [w_start..w_end] hit any peer's live allotted base.
bool grant_hits_peer_live_base(
  const PathReleaseSession & self,
  const int32_t w_start,
  const int32_t w_end,
  const PathGate & gate,
  const std::string & self_id,
  const double eps_m = 1.0,
  const double arrived_standoff_m = 2.5)
{
  if (self.intent.path.empty() || w_start > w_end || w_start < 0) {
    return false;
  }
  const auto & wpath = self.intent.path;
  const int32_t wlast = static_cast<int32_t>(wpath.size()) - 1;
  const int32_t end = std::min(w_end, wlast);
  for (const auto & peer_id : gate.active_robot_ids()) {
    if (peer_id == self_id) {
      continue;
    }
    const PathReleaseSession * peer = gate.session(peer_id);
    if (!peer || peer->intent.path.empty() ||
      peer->released_through_index < 0)
    {
      continue;
    }
    const auto & ppath = peer->intent.path;
    const int32_t plast = static_cast<int32_t>(ppath.size()) - 1;
    // Arrived / idle-at-tip still occupies tip + live pose. Skipping these
    // released B2B followers into the leader's body (working→idle).
    if (peer_live_base_finished(*peer)) {
      const double bx = peer->last_eval_x;
      const double by = peer->last_eval_y;
      const auto & tip = ppath[static_cast<std::size_t>(plast)];
      for (int32_t wi = w_start; wi <= end; ++wi) {
        const auto & w = wpath[static_cast<std::size_t>(wi)];
        if (std::hypot(w.x - bx, w.y - by) <= arrived_standoff_m ||
          std::hypot(w.x - tip.x, w.y - tip.y) <= arrived_standoff_m)
        {
          return true;
        }
      }
      continue;
    }
    const int32_t pend = std::min(peer->released_through_index, plast);
    const int32_t pstart = live_base_start_index(*peer);
    if (pend < pstart) {
      continue;
    }
    for (int32_t wi = w_start; wi <= end; ++wi) {
      for (int32_t pi = pstart; pi <= pend; ++pi) {
        if (std::hypot(
            wpath[static_cast<std::size_t>(wi)].x -
            ppath[static_cast<std::size_t>(pi)].x,
            wpath[static_cast<std::size_t>(wi)].y -
            ppath[static_cast<std::size_t>(pi)].y) <= eps_m)
        {
          return true;
        }
      }
    }
  }
  return false;
}

void clear_frozen_hold(PathReleaseSession & session)
{
  session.held_at_wait = false;
  session.hold_through_index = -1;
  session.junction_only_hold = false;
  session.hold_frozen = false;
  session.frozen_hold_leg = -1;
  session.frozen_hold_x = 0.0;
  session.frozen_hold_y = 0.0;
  session.last_hold_published = false;
  session.last_pub_blocking_amrs.clear();
}

/// Commit the first WAIT hold; robot↔robot / occupancy re-evals keep it.
void freeze_wait_hold(
  PathReleaseSession & session,
  const int32_t hold_index,
  const std::vector<rmf_fleet_msgs::msg::Location> & path)
{
  if (session.hold_frozen || hold_index < 0) {
    return;
  }
  session.hold_frozen = true;
  session.held_at_wait = true;
  session.hold_through_index = hold_index;
  session.frozen_hold_leg = hold_index;
  if (static_cast<std::size_t>(hold_index) < path.size()) {
    session.frozen_hold_x = path[static_cast<std::size_t>(hold_index)].x;
    session.frozen_hold_y = path[static_cast<std::size_t>(hold_index)].y;
  }
}

/// True when observed progress (not grant) has reached the end of the leg.
bool observed_leg_complete(const PathReleaseSession & session)
{
  if (session.intent.path.empty()) {
    return true;
  }
  const int32_t last = static_cast<int32_t>(session.intent.path.size()) - 1;
  const int32_t progress = session.observed_progress_index >= 0 ?
    session.observed_progress_index : session.leg_cursor_index;
  return progress >= last;
}}  // namespace

TrafficManagerNode::TrafficManagerNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("traffic_manager", options)
{
  declare_parameter<std::string>("path_intent_topic", "/traffic_manager/path_intent");
  declare_parameter<std::string>("path_release_topic", "/traffic_manager/path_release");
  declare_parameter<std::string>("fleet_state_topic", "/fleet_states");
  declare_parameter<std::string>("evaluate_path_service", "/evaluate_path_service");
  declare_parameter<double>("query_horizon_m", 20.0);
  declare_parameter<double>("reevaluate_travel_m", 12.0);
  declare_parameter<bool>("send_task_id_to_ecbs", false);
  declare_parameter<double>("held_reevaluate_sec", 0.0);
  declare_parameter<bool>("use_new_base_request_trigger", true);
  declare_parameter<std::string>("base_extension_request_topic", "/BaseRequests");
  declare_parameter<std::string>("to_zone_manager_topic", "/to_zone_manager");
  declare_parameter<bool>("reevaluate_on_to_zone", true);
  declare_parameter<double>("initial_ecbs_defer_ms", 400.0);
  declare_parameter<bool>("publish_path_release_markers", true);
  declare_parameter<std::string>(
    "path_release_marker_topic", "/traffic_manager/path_release_markers");
  declare_parameter<std::string>("marker_frame_id", "map");
  declare_parameter<bool>("use_level_name_as_marker_frame", false);
  declare_parameter<double>("marker_republish_hz", 2.0);
  declare_parameter<double>("amr_length_m", 1.5);
  declare_parameter<double>("clearance_uncertainty_m", 0.5);
  declare_parameter<double>("authority_stop_confirm_sec", 1.0);
  declare_parameter<std::string>(
    "base_extension_query_service", "/base_extension_query");
  declare_parameter<std::string>(
    "commit_proposal_service", "/commit_proposal");
  declare_parameter<std::string>(
    "grant_event_topic", "/traffic_manager/grant_event");
  declare_parameter<std::string>(
    "clearance_event_topic", "/traffic_manager/clearance_event");
  declare_parameter<bool>("ecbs_authoritative", true);
  declare_parameter<double>("recovery_min_interval_sec", 1.0);
  declare_parameter<std::string>("ecbs_release_event_topic", "/ecbs/release_event");

  const auto intent_topic = get_parameter("path_intent_topic").as_string();
  const auto release_topic = get_parameter("path_release_topic").as_string();
  fleet_state_topic_ = get_parameter("fleet_state_topic").as_string();
  base_extension_request_topic_ =
    get_parameter("base_extension_request_topic").as_string();
  const auto ecbs_service = get_parameter("evaluate_path_service").as_string();
  query_horizon_m_ = get_parameter("query_horizon_m").as_double();
  reevaluate_travel_m_ = get_parameter("reevaluate_travel_m").as_double();
  send_task_id_to_ecbs_ = get_parameter("send_task_id_to_ecbs").as_bool();
  held_reevaluate_sec_ = get_parameter("held_reevaluate_sec").as_double();
  use_new_base_request_trigger_ = get_parameter("use_new_base_request_trigger").as_bool();
  to_zone_manager_topic_ = get_parameter("to_zone_manager_topic").as_string();
  reevaluate_on_to_zone_ = get_parameter("reevaluate_on_to_zone").as_bool();
  initial_ecbs_defer_ms_ = get_parameter("initial_ecbs_defer_ms").as_double();
  publish_path_release_markers_ = get_parameter("publish_path_release_markers").as_bool();
  path_release_marker_topic_ = get_parameter("path_release_marker_topic").as_string();
  marker_frame_id_ = get_parameter("marker_frame_id").as_string();
  use_level_name_as_marker_frame_ =
    get_parameter("use_level_name_as_marker_frame").as_bool();
  marker_republish_hz_ = get_parameter("marker_republish_hz").as_double();
  amr_length_m_ = get_parameter("amr_length_m").as_double();
  clearance_uncertainty_m_ = get_parameter("clearance_uncertainty_m").as_double();
  authority_stop_confirm_sec_ =
    get_parameter("authority_stop_confirm_sec").as_double();
  ecbs_authoritative_ = get_parameter("ecbs_authoritative").as_bool();
  recovery_min_interval_sec_ = get_parameter("recovery_min_interval_sec").as_double();

  auto qos = rclcpp::QoS(rclcpp::KeepLast(100)).reliable();

  intent_sub_ = create_subscription<traffic_manager::msg::PathIntent>(
    intent_topic,
    qos,
    std::bind(&TrafficManagerNode::on_path_intent, this, std::placeholders::_1));

  fleet_state_sub_ = create_subscription<rmf_fleet_msgs::msg::FleetState>(
    fleet_state_topic_,
    qos,
    std::bind(&TrafficManagerNode::on_fleet_state, this, std::placeholders::_1));

  base_request_sub_ = create_subscription<traffic_manager::msg::BaseExtensionRequest>(
    base_extension_request_topic_,
    qos,
    std::bind(
      &TrafficManagerNode::on_base_extension_request, this, std::placeholders::_1));

  // Always subscribe: wait markers come from /to_zone_manager directly.
  // Re-evaluate-on-wait remains gated by reevaluate_on_to_zone_.
  to_zone_sub_ = create_subscription<ecbs_planner::msg::ToZoneManager>(
    to_zone_manager_topic_,
    rclcpp::QoS(rclcpp::KeepLast(50)).reliable(),
    std::bind(&TrafficManagerNode::on_to_zone_manager, this, std::placeholders::_1));

  release_event_sub_ = create_subscription<ecbs_planner::msg::ClearanceEvent>(
    get_parameter("ecbs_release_event_topic").as_string(),
    rclcpp::QoS(rclcpp::KeepLast(50)).reliable(),
    std::bind(&TrafficManagerNode::on_ecbs_release_event, this, std::placeholders::_1));

  release_pub_ = create_publisher<traffic_manager::msg::PathRelease>(release_topic, qos);
  grant_event_pub_ = create_publisher<ecbs_planner::msg::GrantEvent>(
    get_parameter("grant_event_topic").as_string(), qos);
  clearance_event_pub_ = create_publisher<ecbs_planner::msg::ClearanceEvent>(
    get_parameter("clearance_event_topic").as_string(), qos);
  base_query_client_ = create_client<ecbs_planner::srv::BaseExtensionQuery>(
    get_parameter("base_extension_query_service").as_string());
  commit_client_ = create_client<ecbs_planner::srv::CommitProposal>(
    get_parameter("commit_proposal_service").as_string());

  if (publish_path_release_markers_) {
    auto marker_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local();
    path_release_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      path_release_marker_topic_, marker_qos);
    if (marker_republish_hz_ > 0.0) {
      const auto period_ms = static_cast<int64_t>(1000.0 / marker_republish_hz_);
      marker_republish_timer_ = create_wall_timer(
        std::chrono::milliseconds(std::max<int64_t>(1, period_ms)),
        std::bind(&TrafficManagerNode::on_marker_republish_timer, this));
    }
  }

  ecbs_client_ = create_client<ecbs_planner::srv::EvaluatePath>(ecbs_service);

  eval_timer_ = create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&TrafficManagerNode::on_eval_timer, this));

  if (use_new_base_request_trigger_) {
    RCLCPP_INFO(
      get_logger(),
      "[traffic_manager] Ready — subscribe PathIntent [%s], publish PathRelease [%s], "
      "to_zone re-eval [%s on %s], BaseRequests [%s], "
      "initial_ecbs_defer=%.0f ms, fleet states [%s], ECBS service [%s], "
      "horizon=%.1f m, base_request_trigger=on (no travel gate), "
      "held_reeval=%.1f s, send_task_id=%s, "
      "release_markers=%s on [%s]",
      intent_topic.c_str(),
      release_topic.c_str(),
      reevaluate_on_to_zone_ ? "enabled" : "disabled",
      to_zone_manager_topic_.c_str(),
      base_extension_request_topic_.c_str(),
      initial_ecbs_defer_ms_,
      fleet_state_topic_.c_str(),
      ecbs_service.c_str(),
      query_horizon_m_,
      held_reevaluate_sec_,
      send_task_id_to_ecbs_ ? "true" : "false",
      publish_path_release_markers_ ? "enabled" : "disabled",
      path_release_marker_topic_.c_str());
  } else {
    RCLCPP_INFO(
      get_logger(),
      "[traffic_manager] Ready — subscribe PathIntent [%s], publish PathRelease [%s], "
      "to_zone re-eval [%s on %s], BaseRequests [%s], "
      "initial_ecbs_defer=%.0f ms, fleet states [%s], ECBS service [%s], "
      "horizon=%.1f m, base_request_trigger=off, reeval_travel=%.1f m, "
      "held_reeval=%.1f s, send_task_id=%s, "
      "release_markers=%s on [%s]",
      intent_topic.c_str(),
      release_topic.c_str(),
      reevaluate_on_to_zone_ ? "enabled" : "disabled",
      to_zone_manager_topic_.c_str(),
      base_extension_request_topic_.c_str(),
      initial_ecbs_defer_ms_,
      fleet_state_topic_.c_str(),
      ecbs_service.c_str(),
      query_horizon_m_,
      reevaluate_travel_m_,
      held_reevaluate_sec_,
      send_task_id_to_ecbs_ ? "true" : "false",
      publish_path_release_markers_ ? "enabled" : "disabled",
      path_release_marker_topic_.c_str());
  }
}

void TrafficManagerNode::cancel_initial_ecbs_timer(const std::string & robot_id)
{
  const auto it = initial_eval_timers_.find(robot_id);
  if (it != initial_eval_timers_.end()) {
    it->second->cancel();
    initial_eval_timers_.erase(it);
  }
}

void TrafficManagerNode::run_pending_initial_ecbs_eval(const std::string & robot_id)
{
  bool run = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    cancel_initial_ecbs_timer(robot_id);
    PathReleaseSession * session = gate_.session(robot_id);
    if (session && session->pending_initial_ecbs) {
      session->pending_initial_ecbs = false;
      run = true;
    }
  }
  if (run) {
    RCLCPP_INFO(
      get_logger(),
      "[traffic_manager] Initial ECBS evaluate robot=[%s] (defer elapsed or /to_zone_manager)",
      robot_id.c_str());
    request_ecbs_evaluation(robot_id, true);
  }
}

void TrafficManagerNode::schedule_pending_initial_ecbs_eval(const std::string & robot_id)
{
  bool eval_immediately = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    cancel_initial_ecbs_timer(robot_id);
    const auto defer_ms = std::max(0.0, initial_ecbs_defer_ms_);
    if (defer_ms <= 0.0) {
      if (PathReleaseSession * session = gate_.session(robot_id)) {
        session->pending_initial_ecbs = false;
      }
      eval_immediately = true;
    } else {
      const auto period = std::chrono::milliseconds(static_cast<int64_t>(defer_ms));
      initial_eval_timers_[robot_id] = create_wall_timer(
        period,
        [this, robot_id]() { run_pending_initial_ecbs_eval(robot_id); });
    }
  }
  if (eval_immediately) {
    request_ecbs_evaluation(robot_id, true);
  }
}

void TrafficManagerNode::on_to_zone_manager(
  const ecbs_planner::msg::ToZoneManager::SharedPtr msg)
{
  if (!msg) {
    return;
  }

  bool has_wait = false;
  int32_t wait_i = -1;
  double wait_x = 0.0;
  double wait_y = 0.0;
  std::vector<std::string> blockers;
  for (std::size_t i = 0; i < msg->path.size(); ++i) {
    const auto & pt = msg->path[i];
    if (pt.wait_sec <= 1e-6 && pt.waiting_for.empty()) {
      continue;
    }
    has_wait = true;
    if (wait_i < 0) {
      wait_i = static_cast<int32_t>(i);
      wait_x = pt.x;
      wait_y = pt.y;
      for (const auto & dep : pt.waiting_for) {
        if (std::abs(dep.wait_point.x) > 1e-9 ||
          std::abs(dep.wait_point.y) > 1e-9)
        {
          wait_x = dep.wait_point.x;
          wait_y = dep.wait_point.y;
        }
        if (!dep.amr_id.empty() &&
          std::find(blockers.begin(), blockers.end(), dep.amr_id) ==
          blockers.end())
        {
          blockers.push_back(dep.amr_id);
        }
      }
    }
  }

  bool pending_initial = false;
  bool should_reeval = false;
  bool refresh_markers = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    PathReleaseSession * session = gate_.session(msg->amr_id);
    if (!session || session->intent.path.empty()) {
      return;
    }
    // Store waits from /to_zone_manager for path_release markers (direct viz).
    const bool wait_changed =
      session->to_zone_has_wait != has_wait ||
      session->to_zone_wait_index != wait_i ||
      session->to_zone_blockers != blockers ||
      std::hypot(session->to_zone_wait_x - wait_x, session->to_zone_wait_y - wait_y) >
      1e-3;
    session->to_zone_has_wait = has_wait;
    session->to_zone_wait_index = wait_i;
    session->to_zone_wait_x = wait_x;
    session->to_zone_wait_y = wait_y;
    session->to_zone_blockers = blockers;
    refresh_markers = wait_changed;

    if (reevaluate_on_to_zone_) {
      should_reeval = has_wait;
      if (session->pending_initial_ecbs) {
        pending_initial = true;
        should_reeval = true;
      }
      if (session->held_at_wait) {
        should_reeval = false;
        pending_initial = false;
      }
    }
  }

  if (refresh_markers) {
    rebuild_path_release_markers();
  }

  if (!reevaluate_on_to_zone_ || !should_reeval) {
    return;
  }

  if (pending_initial) {
    run_pending_initial_ecbs_eval(msg->amr_id);
    return;
  }

  RCLCPP_INFO(
    get_logger(),
    "[traffic_manager] /to_zone_manager wait deps for robot=[%s] — "
    "re-running ECBS evaluate (after register settle)",
    msg->amr_id.c_str());
  const std::string robot_id = msg->amr_id;
  to_zone_reeval_timers_[robot_id] = create_wall_timer(
    std::chrono::milliseconds(80),
    [this, robot_id]()
    {
      to_zone_reeval_timers_.erase(robot_id);
      request_ecbs_evaluation(robot_id, true);
    });
}

void TrafficManagerNode::on_ecbs_release_event(
  const ecbs_planner::msg::ClearanceEvent::SharedPtr msg)
{
  if (!msg || msg->amr_id.empty()) {
    return;
  }
  bool query = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    PathReleaseSession * session = gate_.session(msg->amr_id);
    if (!session || session->intent.path.empty() || session->pending_initial_ecbs) {
      return;
    }
    const int32_t last = static_cast<int32_t>(session->intent.path.size()) - 1;
    // Wake only a robot that is waiting for base: held at an ECBS wait point,
    // or whose newBaseRequest is still unanswered (latched). A robot already
    // driving its awarded base gets the next query_horizon_m slice only when
    // it asks — an ECBS decision change must not extend its base unasked.
    const bool waiting_for_base = session->held_at_wait || session->pending_extension;
    query = waiting_for_base && session->released_through_index < last &&
      !session->ecbs_eval_in_flight && session->pending_commit_grant_id == 0;
    if (!query) {
      RCLCPP_DEBUG(
        get_logger(),
        "[traffic_manager] ECBS release event robot=[%s] (%s) — not waiting for "
        "base (held=%d pending_extension=%d), no re-query",
        msg->amr_id.c_str(), msg->reason.c_str(), session->held_at_wait ? 1 : 0,
        session->pending_extension ? 1 : 0);
    }
  }
  if (query) {
    RCLCPP_INFO(
      get_logger(),
      "[traffic_manager] ECBS release event robot=[%s] (%s) — re-query now",
      msg->amr_id.c_str(), msg->reason.c_str());
    request_base_extension_query(msg->amr_id, false, true);
  }
}

void TrafficManagerNode::on_path_intent(
  const traffic_manager::msg::PathIntent::SharedPtr msg)
{
  if (!msg) {
    return;
  }

  std::string robot_to_schedule;
  std::string robot_to_clear;
  std::string robot_tip_extend;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto result = gate_.accept_intent(*msg);
    if (result == IntentAcceptResult::Invalid) {
      RCLCPP_WARN(
        get_logger(),
        "[traffic_manager] Rejected PathIntent: empty robot_name (fleet=[%s])",
        msg->fleet_name.c_str());
      return;
    }
    if (result == IntentAcceptResult::Duplicate) {
      // full_control resubmits the same PathIntent at released tip end when
      // BaseRequests latch was burned by a premature newBaseRequest. Treat as
      // tip-extension request (G6/T12) instead of silence.
      PathReleaseSession * session = gate_.session(msg->robot_name);
      const int32_t last = session ?
        static_cast<int32_t>(session->intent.path.size()) - 1 : -1;
      if (session && !session->intent.path.empty() &&
        !session->ecbs_eval_in_flight && !session->awaiting_robot_ack &&
        !session->held_at_wait &&
        session->released_through_index >= 0 &&
        session->released_through_index < last)
      {
        RCLCPP_INFO_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "[traffic_manager] Duplicate PathIntent robot=[%s] request_id=%lu "
          "at tip=%d/%d — ECBS extend from awarded tip",
          msg->robot_name.c_str(),
          msg->request_id,
          session->released_through_index,
          last);
        robot_tip_extend = msg->robot_name;
      } else {
        RCLCPP_DEBUG(
          get_logger(),
          "[traffic_manager] Duplicate PathIntent robot=[%s] request_id=%lu — skipped",
          msg->robot_name.c_str(),
          msg->request_id);
      }
      // Keep lock scope tidy; eval runs after unlock.
    } else {
      robot_to_clear = msg->robot_name;

      if (msg->path.empty()) {
        RCLCPP_INFO(
          get_logger(),
          "[traffic_manager] Session cleared robot=[%s] request_id=%lu (empty path / stop)",
          msg->robot_name.c_str(),
          msg->request_id);
      } else {
        RCLCPP_INFO(
          get_logger(),
          "[traffic_manager] Session started robot=[%s] fleet=[%s] request_id=%lu "
          "waypoints=%zu phase=%u revision=%u task_id=[%s] dispatch=[%s]",
          msg->robot_name.c_str(),
          msg->fleet_name.c_str(),
          msg->request_id,
          msg->path.size(),
          msg->phase_index,
          msg->path_revision,
          msg->task_id.c_str(),
          msg->dispatch_task_id.c_str());

        robot_to_schedule = msg->robot_name;
        PathReleaseSession * staged_session = gate_.session(msg->robot_name);
        if (staged_session && staged_session->has_staged_intent &&
          staged_session->staged_intent.request_id == msg->request_id)
        {
          // New task staged behind the current session. If the current leg is
          // already done (robot idle at its destination, nothing outstanding)
          // switch now; otherwise keep it staged and do NOT seed ECBS with the
          // old route (that overrode the RMF handover of the new task).
          const bool done = observed_leg_complete(*staged_session) &&
            staged_session->committed_grant_id == 0 &&
            staged_session->pending_commit_grant_id == 0;
          if (done && gate_.activate_staged_intent(msg->robot_name)) {
            RCLCPP_INFO(
              get_logger(),
              "[traffic_manager] Activated staged intent robot=[%s] (path_intent) — "
              "previous leg already complete",
              msg->robot_name.c_str());
          } else {
            // full_control resubmits the pending PathIntent ~2 Hz until released.
            RCLCPP_INFO_THROTTLE(
              get_logger(), *get_clock(), 5000,
              "[traffic_manager] Next task staged robot=[%s] request_id=%lu — activates "
              "when the current leg completes",
              msg->robot_name.c_str(), msg->request_id);
            robot_to_schedule.clear();
            robot_to_clear.clear();  // not a stop: keep ECBS route / handover intact
          }
        }
        if (!robot_to_schedule.empty()) {
          double x = msg->path.front().x;
          double y = msg->path.front().y;
          const auto state_it = robot_states_.find(robot_to_schedule);
          if (state_it != robot_states_.end()) {
            x = state_it->second.location.x;
            y = state_it->second.location.y;
          }
          gate_.mark_eval_pose(robot_to_schedule, x, y);
        }
      }
    }
  }

  // New path: seed PathIntent (upsert). Do not clear-then-seed async race —
  // clear alone is for empty/stop. /from_zone_manager sampling diverges by
  // index and caused fake WAIT_CLEARANCE with zero blockers at the tip.
  if (!robot_to_clear.empty() && robot_to_schedule.empty()) {
    send_ecbs_clear(robot_to_clear);
  }
  if (!robot_to_schedule.empty()) {
    send_ecbs_seed_path_intent(robot_to_schedule, true);
  }
  if (!robot_tip_extend.empty()) {
    request_ecbs_evaluation(robot_tip_extend, false, true);
  }
  // New / cleared PathIntent: wipe residual markers immediately.
  rebuild_path_release_markers();
}

void TrafficManagerNode::on_fleet_state(
  const rmf_fleet_msgs::msg::FleetState::SharedPtr msg)
{
  if (!msg) {
    return;
  }

  bool markers_need_refresh = false;
  std::vector<std::string> activated_robots;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto & robot : msg->robots) {
      robot_states_[robot.name] = robot;
      PathReleaseSession * session = gate_.session(robot.name);
      if (session && !session->intent.path.empty()) {
        // Keep live-base start tied to pose between ECBS evals.
        gate_.mark_eval_pose(
          robot.name, robot.location.x, robot.location.y);
        // Never search earlier than a small lookback from the last cursor —
        // return-leg geometry must not snap the robot back to outbound indices.
        const int32_t min_start = session->leg_cursor_index >= 0 ?
          std::max(0, session->leg_cursor_index - 2) : 0;
        const int32_t cursor = nearest_leg_index(
          session->intent.path,
          robot.location.x,
          robot.location.y,
          0.5,
          min_start);
        if (cursor >= 0) {
          if (cursor != session->leg_cursor_index) {
            session->leg_cursor_index = cursor;
            session->intent.cursor_index = static_cast<uint32_t>(cursor);
            markers_need_refresh = true;
          }
          // Observed progress is monotonic along the route occurrence.
          if (cursor > session->observed_progress_index) {
            session->observed_progress_index = cursor;
          }
          session->last_telemetry_sec = now().seconds();
          session->telemetry_age_sec = 0.0;
          // Progress → per-overlap clearance / tip complete.
          evaluate_grant_clearance_from_progress(robot.name);
          // STOP_ACCEPTED → STOP_CONFIRMED after dwell with near-zero motion.
          if (session->authority_conflict_active &&
            session->auth_phase == PathReleaseSession::AuthPhase::StopAccepted)
          {
            const double speed = std::hypot(
              robot.location.x - session->last_eval_x,
              robot.location.y - session->last_eval_y);
            const bool idle_mode =
              robot.mode.mode == rmf_fleet_msgs::msg::RobotMode::MODE_IDLE ||
              robot.mode.mode == rmf_fleet_msgs::msg::RobotMode::MODE_CHARGING;
            if (session->authority_stop_confirm_since_sec < 0.0) {
              session->authority_stop_confirm_since_sec = now().seconds();
            }
            if ((idle_mode || speed < 0.05) &&
              now().seconds() - session->authority_stop_confirm_since_sec >=
                authority_stop_confirm_sec_)
            {
              session->auth_phase =
                PathReleaseSession::AuthPhase::StopConfirmed;
              gate_.latch_pending_extension(robot.name);
              RCLCPP_INFO(
                get_logger(),
                "[traffic_manager] AUTHORITY STOP_CONFIRMED robot=[%s]",
                robot.name.c_str());
            } else if (speed >= 0.05 && !idle_mode) {
              session->authority_stop_confirm_since_sec = now().seconds();
            }
          }
        }
        // Task leg complete: retain destination occupancy; do not erase session.
        const int32_t last =
          static_cast<int32_t>(session->intent.path.size()) - 1;
        const bool idle =
          robot.mode.mode == rmf_fleet_msgs::msg::RobotMode::MODE_IDLE ||
          robot.mode.mode == rmf_fleet_msgs::msg::RobotMode::MODE_CHARGING;
        if (idle && observed_leg_complete(*session) && last >= 0) {
          evaluate_grant_clearance_from_progress(robot.name);
          session->destination_occupied = true;
          session->task_leg_complete = true;
          if (session->has_staged_intent &&
            session->committed_grant_id == 0 &&
            session->pending_commit_grant_id == 0)
          {
            if (gate_.activate_staged_intent(robot.name)) {
              markers_need_refresh = true;
              activated_robots.push_back(robot.name);
            }
          }
        }
      }
    }
  }

  // Back-to-back task: the staged next leg is now the session — register it
  // with ECBS and start its first query (previously this activation was
  // silent and the next task never got a release).
  for (const auto & robot_id : activated_robots) {
    complete_staged_activation(robot_id, "leg_complete");
  }
  if (markers_need_refresh) {
    rebuild_path_release_markers();
  }
}

void TrafficManagerNode::complete_staged_activation(
  const std::string & robot_id, const char * where)
{
  bool clear_after = false;
  std::size_t wps = 0;
  std::string dispatch;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    PathReleaseSession * s = gate_.session(robot_id);
    if (!s) {
      return;
    }
    if (s->intent.path.empty()) {
      clear_after = true;
      gate_.clear_session(robot_id);
    } else {
      wps = s->intent.path.size();
      dispatch = s->intent.dispatch_task_id;
    }
  }
  if (clear_after) {
    RCLCPP_INFO(
      get_logger(),
      "[traffic_manager] Activated staged intent robot=[%s] (%s) — empty path, session cleared",
      robot_id.c_str(), where);
    send_ecbs_clear(robot_id);
    return;
  }
  RCLCPP_INFO(
    get_logger(),
    "[traffic_manager] Activated staged intent robot=[%s] (%s) — next task dispatch=[%s] "
    "waypoints=%zu: registering with ECBS, then first query",
    robot_id.c_str(), where, dispatch.c_str(), wps);
  send_ecbs_seed_path_intent(robot_id, true);
}

void TrafficManagerNode::on_base_extension_request(
  const traffic_manager::msg::BaseExtensionRequest::SharedPtr msg)
{
  // Entry log before all guards — every /BaseRequests message hits this line.
  RCLCPP_DEBUG(
    get_logger(),
    "[traffic_manager] /BaseRequests ENTER robot=[%s] order_id=[%s] "
    "task_id=[%s] dispatch=[%s] order_update_id=%u newBaseRequest=%d "
    "trigger_enabled=%d",
    msg ? msg->robot_name.c_str() : "(null)",
    msg ? msg->order_id.c_str() : "",
    msg ? msg->task_id.c_str() : "",
    msg ? msg->dispatch_task_id.c_str() : "",
    msg ? msg->order_update_id : 0u,
    msg && msg->new_base_request ? 1 : 0,
    use_new_base_request_trigger_ ? 1 : 0);

  if (!msg) {
    RCLCPP_WARN(
      get_logger(),
      "[traffic_manager] /BaseRequests RETURN reason=null_msg");
    return;
  }
  if (!use_new_base_request_trigger_) {
    RCLCPP_INFO(
      get_logger(),
      "[traffic_manager] /BaseRequests RETURN robot=[%s] "
      "reason=trigger_disabled",
      msg->robot_name.c_str());
    return;
  }

  std::string robot_to_eval;
  bool extend_from_tip = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    PathReleaseSession * session = gate_.session(msg->robot_name);
    if (!session) {
      RCLCPP_DEBUG_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "[traffic_manager] /BaseRequests RETURN robot=[%s] "
        "reason=missing_session (idle / pre-PathIntent expected)",
        msg->robot_name.c_str());
      return;
    }
    if (session->intent.path.empty()) {
      RCLCPP_WARN(
        get_logger(),
        "[traffic_manager] /BaseRequests RETURN robot=[%s] "
        "reason=empty_session_path awaiting_ack=%d pending_ext=%d "
        "eval_in_flight=%d released_through=%d last_pub_release=%u",
        msg->robot_name.c_str(),
        session->awaiting_robot_ack ? 1 : 0,
        session->pending_extension ? 1 : 0,
        session->ecbs_eval_in_flight ? 1 : 0,
        session->released_through_index,
        session->last_published_release_update_id);
      return;
    }
    if (!msg->dispatch_task_id.empty() &&
      !session->intent.dispatch_task_id.empty() &&
      msg->dispatch_task_id != session->intent.dispatch_task_id)
    {
      RCLCPP_WARN(
        get_logger(),
        "[traffic_manager] /BaseRequests RETURN robot=[%s] "
        "reason=dispatch_mismatch msg=[%s] session=[%s]",
        msg->robot_name.c_str(),
        msg->dispatch_task_id.c_str(),
        session->intent.dispatch_task_id.c_str());
      return;
    }
    if (!msg->task_id.empty() && msg->task_id != session->intent.task_id) {
      RCLCPP_WARN(
        get_logger(),
        "[traffic_manager] /BaseRequests RETURN robot=[%s] "
        "reason=task_mismatch msg=[%s] session=[%s]",
        msg->robot_name.c_str(),
        msg->task_id.c_str(),
        session->intent.task_id.c_str());
      return;
    }

    // execution: VDA orderUpdateId = phase_base + PathRequest.path_revision
    // (path_revision = RMF leg revision + release_update_id; phase_base is 1
    // on drop legs). execution strips its phase_base before publishing
    // /BaseRequests (publish_base_extension_from_mqtt), so it is 0 here for
    // every leg type — do not add a drop offset on this side as well.
    const uint32_t phase_base = 0;
    const uint32_t reported = msg->order_update_id;
    if (reported < phase_base + session->intent.path_revision) {
      RCLCPP_WARN(
        get_logger(),
        "[traffic_manager] /BaseRequests RETURN robot=[%s] "
        "reason=stale_update reported=%u intent_path_revision=%u "
        "phase_base=%u",
        msg->robot_name.c_str(),
        reported,
        session->intent.path_revision,
        phase_base);
      return;
    }
    const uint32_t release_slot =
      reported - phase_base - session->intent.path_revision;

    auto log_ack_state = [this, session, release_slot, msg](
        const char * phase) {
        RCLCPP_DEBUG(
          get_logger(),
          "[traffic_manager] /BaseRequests ACK_%s robot=[%s] "
          "awaiting_robot_ack=%d received_release_slot=%u "
          "expected_release_update_id=%u pending_extension=%d "
          "ecbs_eval_in_flight=%d released_through=%d "
          "committed_grant_id=%lu",
          phase,
          msg->robot_name.c_str(),
          session->awaiting_robot_ack ? 1 : 0,
          release_slot,
          session->last_published_release_update_id,
          session->pending_extension ? 1 : 0,
          session->ecbs_eval_in_flight ? 1 : 0,
          session->released_through_index,
          static_cast<unsigned long>(session->committed_grant_id));
      };

    auto on_ack = [this, session]() {
        // Order ACK = STOP_ACCEPTED only — not standstill / reconcile permission.
        // Do not reset committed base here.
        if (session->authority_conflict_active &&
          session->auth_phase == PathReleaseSession::AuthPhase::StopRequested)
        {
          session->auth_phase = PathReleaseSession::AuthPhase::StopAccepted;
          session->authority_stop_confirm_since_sec = now().seconds();
          RCLCPP_INFO(
            get_logger(),
            "[traffic_manager] AUTHORITY STOP_ACCEPTED robot=[%s] — awaiting "
            "standstill confirmation",
            session->intent.robot_name.c_str());
        }
      };

    if (!msg->new_base_request) {
      log_ack_state("BEFORE");
      const bool acked =
        gate_.acknowledge_release(msg->robot_name, release_slot);
      log_ack_state("AFTER");
      if (acked) {
        session->last_new_base_request = false;
        release_markers_dirty_ = true;  // newly allowed stretch turns yellow
        on_ack();
        RCLCPP_INFO(
          get_logger(),
          "[traffic_manager] /BaseRequests robot=[%s] order_id=[%s] ack "
          "order_update_id=%u (release_update_id=%u)",
          msg->robot_name.c_str(),
          msg->order_id.c_str(),
          reported,
          release_slot);
      } else if (session->awaiting_robot_ack) {
        RCLCPP_WARN(
          get_logger(),
          "[traffic_manager] /BaseRequests RETURN robot=[%s] "
          "reason=ack_mismatch received_slot=%u expected=%u "
          "(ACK validation kept; committed base unchanged)",
          msg->robot_name.c_str(),
          release_slot,
          session->last_published_release_update_id);
      }
    }

    if (msg->new_base_request) {
      log_ack_state("BEFORE_IMPLICIT");
      const bool acked =
        gate_.acknowledge_release(msg->robot_name, release_slot);
      log_ack_state("AFTER_IMPLICIT");
      if (acked) {
        session->last_new_base_request = false;
        release_markers_dirty_ = true;  // newly allowed stretch turns yellow
        on_ack();
        RCLCPP_INFO(
          get_logger(),
          "[traffic_manager] /BaseRequests robot=[%s] order_id=[%s] implicit ack "
          "via newBaseRequest order_update_id=%u (release_update_id=%u)",
          msg->robot_name.c_str(),
          msg->order_id.c_str(),
          reported,
          release_slot);
      } else if (session->awaiting_robot_ack) {
        RCLCPP_WARN(
          get_logger(),
          "[traffic_manager] /BaseRequests robot=[%s] "
          "reason=ack_mismatch_on_newBaseRequest received_slot=%u expected=%u "
          "(continuing; ACK validation kept; committed base unchanged)",
          msg->robot_name.c_str(),
          release_slot,
          session->last_published_release_update_id);
      }
    }

    const bool rising_edge =
      msg->new_base_request && !session->last_new_base_request;
    session->last_new_base_request = msg->new_base_request;

    // Latch extension until fulfilled/cancelled/superseded — do not depend
    // only on a rising edge that can be missed while awaiting ack.
    if (msg->new_base_request) {
      gate_.latch_pending_extension(msg->robot_name);
    }
    if (!session->pending_extension && (!rising_edge || !msg->new_base_request)) {
      RCLCPP_DEBUG(
        get_logger(),
        "[traffic_manager] /BaseRequests RETURN robot=[%s] "
        "reason=no_pending_extension_and_no_rising_edge "
        "newBaseRequest=%d rising_edge=%d pending_extension=%d "
        "awaiting_ack=%d eval_in_flight=%d",
        msg->robot_name.c_str(),
        msg->new_base_request ? 1 : 0,
        rising_edge ? 1 : 0,
        session->pending_extension ? 1 : 0,
        session->awaiting_robot_ack ? 1 : 0,
        session->ecbs_eval_in_flight ? 1 : 0);
      return;
    }
    if (!session->pending_extension) {
      RCLCPP_INFO(
        get_logger(),
        "[traffic_manager] /BaseRequests RETURN robot=[%s] "
        "reason=pending_extension_false_after_latch_check "
        "awaiting_ack=%d eval_in_flight=%d",
        msg->robot_name.c_str(),
        session->awaiting_robot_ack ? 1 : 0,
        session->ecbs_eval_in_flight ? 1 : 0);
      return;
    }
    if (session->awaiting_robot_ack) {
      // Do not consume the rising edge: ack may clear on the next poll, and
      // MQTT often keeps newBaseRequest=true, so leaving the latch high would
      // drop this request forever. Reset so the next poll re-triggers.
      // Do not bypass ACK validation or reset committed base.
      session->last_new_base_request = false;
      RCLCPP_DEBUG(
        get_logger(),
        "[traffic_manager] /BaseRequests RETURN robot=[%s] "
        "reason=awaiting_robot_ack expected_release_update_id=%u "
        "received_slot=%u pending_extension=%d eval_in_flight=%d "
        "released_through=%d — will re-trigger on next poll",
        msg->robot_name.c_str(),
        session->last_published_release_update_id,
        release_slot,
        session->pending_extension ? 1 : 0,
        session->ecbs_eval_in_flight ? 1 : 0,
        session->released_through_index);
      return;
    }
    if (session->held_at_wait && !session->junction_only_hold) {
      // Re-check immediately: entry clear / ownership trim may FREE. Floor at
      // awarded tip so a FREE opens the next query_horizon_m (G6) — do not
      // burn the NBR latch on a pose-only same-tip FREE.
      session->last_new_base_request = false;
      if (session->ecbs_eval_in_flight) {
        RCLCPP_INFO(
          get_logger(),
          "[traffic_manager] /BaseRequests RETURN robot=[%s] "
          "reason=evaluation_already_in_flight_while_held "
          "hold_through=%d pending_extension=%d released_through=%d",
          msg->robot_name.c_str(),
          session->hold_through_index,
          session->pending_extension ? 1 : 0,
          session->released_through_index);
        return;
      }
      robot_to_eval = msg->robot_name;
      extend_from_tip = true;
      RCLCPP_INFO(
        get_logger(),
        "[traffic_manager] /BaseRequests SCHEDULE robot=[%s] "
        "reason=held_reeval_from_tip hold_through=%d released_through=%d "
        "pending_extension=%d eval_in_flight=%d",
        msg->robot_name.c_str(),
        session->hold_through_index,
        session->released_through_index,
        session->pending_extension ? 1 : 0,
        session->ecbs_eval_in_flight ? 1 : 0);
    } else {
      if (release_slot != session->last_published_release_update_id) {
        RCLCPP_WARN(
          get_logger(),
          "[traffic_manager] /BaseRequests RETURN robot=[%s] "
          "reason=ack_slot_mismatch_for_extension received_slot=%u "
          "expected_release_update_id=%u order_update_id=%u "
          "pending_extension=%d awaiting_ack=%d eval_in_flight=%d "
          "(ACK validation kept; committed base unchanged)",
          msg->robot_name.c_str(),
          release_slot,
          session->last_published_release_update_id,
          reported,
          session->pending_extension ? 1 : 0,
          session->awaiting_robot_ack ? 1 : 0,
          session->ecbs_eval_in_flight ? 1 : 0);
        return;
      }
      if (session->ecbs_eval_in_flight) {
        session->last_new_base_request = false;
        RCLCPP_INFO(
          get_logger(),
          "[traffic_manager] /BaseRequests RETURN robot=[%s] "
          "reason=evaluation_already_in_flight pending_extension=%d "
          "awaiting_ack=%d released_through=%d expected_release_update_id=%u",
          msg->robot_name.c_str(),
          session->pending_extension ? 1 : 0,
          session->awaiting_robot_ack ? 1 : 0,
          session->released_through_index,
          session->last_published_release_update_id);
        return;
      }

      // Rolling horizon: one extension of query_horizon_m per base request,
      // asked at the MIDPOINT of the released stretch. The robot's
      // newBaseRequest can still be set in the state that acknowledges the
      // extension just sent (it was waiting at the old boundary), which used
      // to trigger a second and third extension within 400 ms (amr3: 32→39→
      // 45→51, 18 m in one go). Extend only when the robot really is within
      // half a horizon of its released end, and never past the route end.
      {
        const auto & path = session->intent.path;
        const int32_t last = static_cast<int32_t>(path.size()) - 1;
        if (last >= 0 && session->released_through_index >= last) {
          gate_.clear_pending_extension(msg->robot_name);
          RCLCPP_DEBUG(
            get_logger(),
            "[traffic_manager] /BaseRequests RETURN robot=[%s] "
            "reason=route_fully_released released_through=%d",
            msg->robot_name.c_str(), session->released_through_index);
          return;
        }
        const int32_t from = session->leg_cursor_index;
        const int32_t to = std::min(session->released_through_index, last);
        if (from >= 0 && to > from) {
          double remaining_m = 0.0;
          for (int32_t i = from; i < to; ++i) {
            remaining_m += std::hypot(
              path[i + 1].x - path[i].x, path[i + 1].y - path[i].y);
          }
          const double request_zone_m = 0.5 * query_horizon_m_;
          if (remaining_m > request_zone_m + 0.5) {
            gate_.clear_pending_extension(msg->robot_name);
            RCLCPP_DEBUG(
              get_logger(),
              "[traffic_manager] /BaseRequests RETURN robot=[%s] "
              "reason=stale_base_request remaining_released=%.1fm > %.1fm "
              "(half of query_horizon_m) cursor=%d released_through=%d",
              msg->robot_name.c_str(), remaining_m, request_zone_m, from,
              session->released_through_index);
            return;
          }
        }
      }

      RCLCPP_INFO(
        get_logger(),
        "[traffic_manager] /BaseRequests SCHEDULE robot=[%s] order_id=[%s] "
        "order_update_id=%u release_slot=%u released_through=%d "
        "pending_extension=%d → ECBS re-eval from awarded tip (G6) "
        "(no waiter/dep here — order update only if FREE tip advances "
        "and Commit ACCEPTED publishes PathRelease)",
        msg->robot_name.c_str(),
        msg->order_id.c_str(),
        reported,
        release_slot,
        session->released_through_index,
        session->pending_extension ? 1 : 0);
      robot_to_eval = msg->robot_name;
      extend_from_tip = true;
    }
  }

  if (!robot_to_eval.empty()) {
    RCLCPP_DEBUG(
      get_logger(),
      "[traffic_manager] /BaseRequests DISPATCH_EVAL robot=[%s] "
      "extend_from_tip=%d",
      robot_to_eval.c_str(),
      extend_from_tip ? 1 : 0);
    request_ecbs_evaluation(robot_to_eval, false, extend_from_tip);
  } else {
    RCLCPP_INFO(
      get_logger(),
      "[traffic_manager] /BaseRequests EXIT robot=[%s] "
      "reason=no_eval_scheduled_after_guards",
      msg->robot_name.c_str());
  }
}

void TrafficManagerNode::on_eval_timer()
{
  std::vector<std::string> robots_to_eval;
  std::vector<std::string> held_to_eval;
  std::vector<std::string> pending_ext_to_eval;
  std::vector<std::string> staged_to_activate;
  std::vector<std::pair<std::string, uint64_t>> recovery_seeds;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const double now_sec = now().seconds();
    for (const auto & robot_id : gate_.active_robot_ids()) {
      auto * session = gate_.session(robot_id);
      if (!session) {
        continue;
      }
      // Activate staged replacement once outstanding grant is reconciled.
      if (session->has_staged_intent &&
        (session->released_through_index < 0 ||
        observed_leg_complete(*session)))
      {
        staged_to_activate.push_back(robot_id);
      }
      // Lost-response recovery: clear stuck in-flight and re-latch demand.
      constexpr double kEvalTimeoutSec = 5.0;
      if (session->ecbs_eval_in_flight &&
        now_sec - session->last_ecbs_request_sec >= kEvalTimeoutSec)
      {
        RCLCPP_WARN(
          get_logger(),
          "[traffic_manager] ECBS eval timeout robot=[%s] — clear in-flight",
          robot_id.c_str());
        gate_.set_ecbs_in_flight(robot_id, false);
        gate_.latch_pending_extension(robot_id);
      }
      // Commit-then-publish: a lost CommitProposal ACCEPTED leaves
      // pending_commit_grant_id set and blocks every BaseExtensionQuery —
      // no PathRelease ever reaches the robot despite FREE_PROPOSAL.
      constexpr double kCommitTimeoutSec = 5.0;
      if (session->pending_commit_grant_id != 0 &&
        now_sec - session->last_ecbs_request_sec >= kCommitTimeoutSec)
      {
        RCLCPP_WARN(
          get_logger(),
          "[traffic_manager] Commit timeout robot=[%s] grant=%lu — drop "
          "pending commit and re-latch extension",
          robot_id.c_str(),
          static_cast<unsigned long>(session->pending_commit_grant_id));
        session->pending_commit_grant_id = 0;
        session->pending_commit_release_index = -1;
        session->authority_txn_phase =
          PathReleaseSession::AuthorityTxnPhase::None;
        gate_.latch_pending_extension(robot_id);
      }
      // Bounded route-recovery seed retry — one txn per robot; seed outside lock.
      if (session->route_recovery_active &&
        !session->route_recovery_seed_in_flight &&
        !session->ecbs_eval_in_flight &&
        now_sec >= session->route_recovery_next_retry_sec)
      {
        if (begin_or_refresh_route_recovery_locked(
            *session, "eval_timer_backoff"))
        {
          recovery_seeds.emplace_back(
            robot_id, session->route_recovery_txn_id);
        }
      }
      // Independently service latched extension demand (no new robot message).
      // Skip while a route-recovery seed is in flight — one recovery txn only.
      if (session->pending_extension && !session->ecbs_eval_in_flight &&
        !session->awaiting_robot_ack &&
        !(session->route_recovery_active &&
          session->route_recovery_seed_in_flight) &&
        (!session->authority_conflict_active ||
          session->auth_phase == PathReleaseSession::AuthPhase::StopConfirmed ||
          session->auth_phase ==
            PathReleaseSession::AuthPhase::AuthorityReconciled) &&
        !session->pending_initial_ecbs)
      {
        pending_ext_to_eval.push_back(robot_id);
      }
      if (session->held_at_wait) {
        if (held_reevaluate_sec_ > 0.0 && !session->ecbs_eval_in_flight &&
          now_sec - session->last_ecbs_request_sec >= held_reevaluate_sec_)
        {
          held_to_eval.push_back(robot_id);
        }
        continue;
      }
      // With base-request trigger: rolling grants come only from
      // on_base_extension_request — never from travelled distance
      // (pending_extension retry above is the exception).
      if (use_new_base_request_trigger_) {
        continue;
      }
      if (session->awaiting_robot_ack) {
        continue;
      }
      const auto state_it = robot_states_.find(robot_id);
      if (state_it == robot_states_.end()) {
        continue;
      }
      if (robot_needs_reevaluation(
          *session,
          state_it->second.location.x,
          state_it->second.location.y))
      {
        robots_to_eval.push_back(robot_id);
      }
    }
  }

  for (const auto & robot_id : staged_to_activate) {
    bool activated = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      activated = gate_.activate_staged_intent(robot_id);
    }
    // seed / clear must not run under mutex_ (async helpers lock it).
    if (activated) {
      complete_staged_activation(robot_id, "eval_timer");
    }
  }

  // Seed outside mutex_ — send_ecbs_seed_path_intent acquires the lock.
  for (const auto & item : recovery_seeds) {
    send_ecbs_seed_path_intent(item.first, false, item.second);
  }

  for (const auto & robot_id : pending_ext_to_eval) {
    request_ecbs_evaluation(robot_id, false, true);
  }

  for (const auto & robot_id : robots_to_eval) {
    RCLCPP_INFO(
      get_logger(),
      "[traffic_manager] Re-evaluation triggered robot=[%s] (moved >= %.1f m since last eval)",
      robot_id.c_str(),
      reevaluate_travel_m_);
    request_ecbs_evaluation(robot_id, false);
  }

  for (const auto & robot_id : held_to_eval) {
    RCLCPP_DEBUG(
      get_logger(),
      "[traffic_manager] Held robot=[%s] — re-checking wait dependency with ECBS",
      robot_id.c_str());
    request_ecbs_evaluation(robot_id, true);
  }
}

bool TrafficManagerNode::robot_needs_reevaluation(
  const PathReleaseSession & session,
  const double current_x,
  const double current_y) const
{
  if (session.ecbs_eval_in_flight) {
    return false;
  }

  if (session.pending_initial_ecbs) {
    return false;
  }

  const auto & path = session.intent.path;
  if (path.empty()) {
    return false;
  }

  const int32_t last_index = static_cast<int32_t>(path.size()) - 1;
  if (session.released_through_index >= last_index) {
    return false;
  }

  const double dx = current_x - session.last_eval_x;
  const double dy = current_y - session.last_eval_y;
  return std::hypot(dx, dy) >= reevaluate_travel_m_;
}

void TrafficManagerNode::request_ecbs_evaluation(
  const std::string & robot_id,
  const bool force,
  const bool extend_from_awarded_tip)
{
  // Single release transaction: ECBS proposal → TM validate/commit → PathRelease.
  request_base_extension_query(robot_id, force, extend_from_awarded_tip);
}

void TrafficManagerNode::request_base_extension_query(
  const std::string & robot_id,
  const bool force,
  const bool extend_from_awarded_tip)
{
  PathReleaseSession * session = gate_.session(robot_id);
  if (!session || session->intent.path.empty()) {
    return;
  }
  if (session->ecbs_eval_in_flight) {
    return;
  }

  const int32_t last_index =
    static_cast<int32_t>(session->intent.path.size()) - 1;
  if (!force && session->released_through_index >= last_index) {
    return;
  }

  double current_x = session->intent.path.front().x;
  double current_y = session->intent.path.front().y;
  const auto state_it = robot_states_.find(robot_id);
  if (state_it != robot_states_.end()) {
    current_x = state_it->second.location.x;
    current_y = state_it->second.location.y;
  }

  int32_t horizon_floor = 0;
  if (session->leg_cursor_index >= 0) {
    horizon_floor = std::max(horizon_floor, session->leg_cursor_index - 2);
  }
  if (extend_from_awarded_tip && session->released_through_index >= 0) {
    horizon_floor = std::max(horizon_floor, session->released_through_index);
  }
  const PathBaseRange horizon = base_range_from_pose(
    session->intent.path, current_x, current_y, query_horizon_m_,
    horizon_floor, extend_from_awarded_tip);
  if (!horizon.valid) {
    return;
  }
  const int32_t horizon_end_index = static_cast<int32_t>(horizon.end_index);

  if (!base_query_client_ || !base_query_client_->service_is_ready()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "[traffic_manager] BaseExtensionQuery not ready — robot=[%s] fail-closed",
      robot_id.c_str());
    return;
  }

  if (session->pending_commit_grant_id != 0 || session->ecbs_eval_in_flight ||
    (session->route_recovery_active && session->route_recovery_seed_in_flight))
  {
    RCLCPP_DEBUG_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "[traffic_manager] BaseExtensionQuery deferred — robot=[%s] open txn "
      "pending_commit=%lu eval_in_flight=%d recovery_seed=%d",
      robot_id.c_str(),
      static_cast<unsigned long>(session->pending_commit_grant_id),
      session->ecbs_eval_in_flight ? 1 : 0,
      (session->route_recovery_active &&
        session->route_recovery_seed_in_flight) ? 1 : 0);
    return;
  }

  auto req = std::make_shared<ecbs_planner::srv::BaseExtensionQuery::Request>();
  req->request_id = session->intent.request_id;
  req->amr_id = robot_id;
  req->fleet_name = session->intent.fleet_name;
  // Canonical = dispatch/planner UUID when present; execution order is separate.
  const std::string canonical = !session->intent.dispatch_task_id.empty() ?
    session->intent.dispatch_task_id : session->intent.task_id;
  // ECBS authoritative: always send the route identity so ECBS can tell a
  // stale session from the route RMF handed over.
  req->task_id = (send_task_id_to_ecbs_ || ecbs_authoritative_) ? canonical : std::string();
  req->execution_order_id = session->intent.task_id;
  req->phase_index = static_cast<int32_t>(session->intent.phase_index);
  req->path_revision = session->intent.path_revision;
  req->current_grant_through_index = session->released_through_index;
  req->current_x = current_x;
  req->current_y = current_y;
  req->desired_extension_m = query_horizon_m_;
  req->distance_origin = extend_from_awarded_tip ? "GRANT_TIP" : "POSE";
  if (session->observed_progress_index >= 0) {
    req->observed_progress.route_occurrence =
      static_cast<uint32_t>(session->observed_progress_index);
    req->observed_progress.path_revision = session->intent.path_revision;
  }

  if (session->authority_conflict_active &&
    session->auth_phase != PathReleaseSession::AuthPhase::StopConfirmed &&
    session->auth_phase != PathReleaseSession::AuthPhase::AuthorityReconciled)
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "[traffic_manager] BaseExtensionQuery blocked — robot=[%s] AUTHORITY "
      "phase=%d id=%s (need STOP_CONFIRMED)",
      robot_id.c_str(), static_cast<int>(session->auth_phase),
      session->authority_conflict_id.c_str());
    return;
  }

  const uint64_t evaluation_id = session->next_evaluation_id++;
  session->pending_evaluation_id = evaluation_id;
  session->ecbs_eval_in_flight = true;
  session->last_ecbs_request_sec = now().seconds();
  // Do not clear pending_extension on send — only on fulfilled commit / cancel.
  gate_.mark_eval_pose(robot_id, current_x, current_y);

  const std::size_t session_hash = session->content_hash;
  RCLCPP_INFO(
    get_logger(),
    "[traffic_manager] BaseExtensionQuery robot=[%s] eval_id=%lu "
    "grant_through=%d desired_m=%.1f origin=%s",
    robot_id.c_str(), static_cast<unsigned long>(evaluation_id),
    session->released_through_index, query_horizon_m_,
    req->distance_origin.c_str());

  base_query_client_->async_send_request(
    req,
    [this, robot_id, horizon_end_index, session_hash, evaluation_id](
      rclcpp::Client<ecbs_planner::srv::BaseExtensionQuery>::SharedFuture future)
    {
      handle_base_extension_query_response(
        robot_id, future, horizon_end_index, session_hash, evaluation_id);
    });
}

bool TrafficManagerNode::begin_or_refresh_route_recovery_locked(
  PathReleaseSession & session,
  const std::string & reason)
{
  // Preserve committed tip / grant / pending extension for the whole txn.
  gate_.latch_pending_extension(session.intent.robot_name);
  const double now_sec = now().seconds();
  const std::string canonical = !session.intent.dispatch_task_id.empty() ?
    session.intent.dispatch_task_id : session.intent.task_id;

  if (session.route_recovery_active) {
    if (session.route_recovery_seed_in_flight) {
      RCLCPP_INFO(
        get_logger(),
        "[traffic_manager] Route recovery already in flight robot=[%s] "
        "txn=%lu tip=%d grant=%lu — skip duplicate seed (%s)",
        session.intent.robot_name.c_str(),
        static_cast<unsigned long>(session.route_recovery_txn_id),
        session.released_through_index,
        static_cast<unsigned long>(session.committed_grant_id),
        reason.c_str());
      return false;
    }
    if (now_sec < session.route_recovery_next_retry_sec) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "[traffic_manager] Route recovery backoff robot=[%s] txn=%lu "
        "next_retry_in=%.2fs tip=%d (%s)",
        session.intent.robot_name.c_str(),
        static_cast<unsigned long>(session.route_recovery_txn_id),
        session.route_recovery_next_retry_sec - now_sec,
        session.released_through_index,
        reason.c_str());
      return false;
    }
    // Refresh identity snapshot for the same txn; tip/grant unchanged.
    session.route_recovery_session_hash = session.content_hash;
    session.route_recovery_request_id = session.intent.request_id;
    session.route_recovery_canonical_task = canonical;
    session.route_recovery_execution_order = session.intent.task_id;
    session.route_recovery_path_revision = session.intent.path_revision;
    session.route_recovery_preserved_tip = session.released_through_index;
    session.route_recovery_preserved_grant = session.committed_grant_id;
    session.route_recovery_had_pending_extension = true;
    session.route_recovery_seed_in_flight = true;
    session.route_recovery_next_retry_sec = now_sec + 0.5;
    return true;
  }

  session.route_recovery_active = true;
  session.route_recovery_seed_in_flight = true;
  session.route_recovery_txn_id = next_route_recovery_txn_id_++;
  session.route_recovery_session_hash = session.content_hash;
  session.route_recovery_request_id = session.intent.request_id;
  session.route_recovery_canonical_task = canonical;
  session.route_recovery_execution_order = session.intent.task_id;
  session.route_recovery_path_revision = session.intent.path_revision;
  session.route_recovery_preserved_tip = session.released_through_index;
  session.route_recovery_preserved_grant = session.committed_grant_id;
  session.route_recovery_had_pending_extension = session.pending_extension;
  session.route_recovery_started_sec = now_sec;
  session.route_recovery_next_retry_sec = now_sec + 0.5;
  session.route_recovery_attempt = 0;
  // Rate limit: a robot whose route keeps coming back STALE must not spin a
  // seed→query→STALE loop (observed 2.6M recovery txns in 170 s). Arm the
  // eval-timer backoff retry instead of seeding immediately.
  {
    auto & last_start = last_recovery_start_sec_[session.intent.robot_name];
    if (now_sec - last_start < recovery_min_interval_sec_) {
      session.route_recovery_seed_in_flight = false;
      session.route_recovery_next_retry_sec = last_start + recovery_min_interval_sec_;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "[traffic_manager] Route recovery rate-limited robot=[%s] — retry in %.2fs (%s)",
        session.intent.robot_name.c_str(),
        session.route_recovery_next_retry_sec - now_sec, reason.c_str());
      return false;
    }
    last_start = now_sec;
  }
  RCLCPP_WARN(
    get_logger(),
    "[traffic_manager] Route recovery START robot=[%s] txn=%lu tip=%d/%zu "
    "grant=%lu pending_ext=%d path_rev=%u request_id=%lu reason=[%s]",
    session.intent.robot_name.c_str(),
    static_cast<unsigned long>(session.route_recovery_txn_id),
    session.released_through_index,
    session.intent.path.size(),
    static_cast<unsigned long>(session.committed_grant_id),
    session.pending_extension ? 1 : 0,
    session.intent.path_revision,
    static_cast<unsigned long>(session.intent.request_id),
    reason.c_str());
  return true;
}

bool TrafficManagerNode::complete_route_recovery_seed_locked(
  PathReleaseSession & session,
  const uint64_t recovery_txn_id,
  const bool seed_ok,
  const std::string & seed_message)
{
  if (recovery_txn_id == 0) {
    return false;
  }
  if (!session.route_recovery_active ||
    session.route_recovery_txn_id != recovery_txn_id)
  {
    RCLCPP_WARN(
      get_logger(),
      "[traffic_manager] Ignore stale route-recovery seed robot=[%s] "
      "txn=%lu active=%d current_txn=%lu",
      session.intent.robot_name.c_str(),
      static_cast<unsigned long>(recovery_txn_id),
      session.route_recovery_active ? 1 : 0,
      static_cast<unsigned long>(session.route_recovery_txn_id));
    return false;
  }

  const std::string canonical = !session.intent.dispatch_task_id.empty() ?
    session.intent.dispatch_task_id : session.intent.task_id;
  if (session.content_hash != session.route_recovery_session_hash ||
    session.intent.request_id != session.route_recovery_request_id ||
    session.intent.path_revision != session.route_recovery_path_revision ||
    session.intent.task_id != session.route_recovery_execution_order ||
    canonical != session.route_recovery_canonical_task)
  {
    RCLCPP_WARN(
      get_logger(),
      "[traffic_manager] Ignore route-recovery seed — identity mismatch "
      "robot=[%s] txn=%lu (session replaced or task/route changed)",
      session.intent.robot_name.c_str(),
      static_cast<unsigned long>(recovery_txn_id));
    session.route_recovery_seed_in_flight = false;
    return false;
  }

  // Never erase tip/grant during recovery — restore if something drifted.
  if (session.released_through_index < session.route_recovery_preserved_tip) {
    session.released_through_index = session.route_recovery_preserved_tip;
  }
  if (session.committed_grant_id == 0 &&
    session.route_recovery_preserved_grant != 0)
  {
    session.committed_grant_id = session.route_recovery_preserved_grant;
  }
  gate_.latch_pending_extension(session.intent.robot_name);
  session.route_recovery_seed_in_flight = false;

  if (seed_ok) {
    session.route_recovery_active = false;
    session.route_recovery_attempt = 0;
    RCLCPP_INFO(
      get_logger(),
      "[traffic_manager] Route recovery SEED_OK robot=[%s] txn=%lu tip=%d "
      "grant=%lu — tip-extend retained pending_extension",
      session.intent.robot_name.c_str(),
      static_cast<unsigned long>(recovery_txn_id),
      session.released_through_index,
      static_cast<unsigned long>(session.committed_grant_id));
    return true;
  }

  ++session.route_recovery_attempt;
  constexpr int kMaxAttempts = 8;
  const double backoff = std::min(
    10.0, 0.5 * static_cast<double>(1 << std::min(session.route_recovery_attempt, 4)));
  session.route_recovery_next_retry_sec = now().seconds() + backoff;
  if (session.route_recovery_attempt >= kMaxAttempts) {
    session.fault_reason = "route_recovery_exhausted";
    RCLCPP_ERROR(
      get_logger(),
      "[traffic_manager] Route recovery EXHAUSTED robot=[%s] txn=%lu "
      "attempts=%d tip=%d msg=[%s] — demand retained",
      session.intent.robot_name.c_str(),
      static_cast<unsigned long>(recovery_txn_id),
      session.route_recovery_attempt,
      session.released_through_index,
      seed_message.c_str());
  } else {
    RCLCPP_WARN(
      get_logger(),
      "[traffic_manager] Route recovery SEED_FAIL robot=[%s] txn=%lu "
      "attempt=%d backoff=%.1fs tip=%d msg=[%s]",
      session.intent.robot_name.c_str(),
      static_cast<unsigned long>(recovery_txn_id),
      session.route_recovery_attempt,
      backoff,
      session.released_through_index,
      seed_message.c_str());
  }
  return false;
}

void TrafficManagerNode::handle_base_extension_query_response(
  const std::string & robot_id,
  rclcpp::Client<ecbs_planner::srv::BaseExtensionQuery>::SharedFuture future,
  const int32_t horizon_end_index,
  const std::size_t session_hash,
  const uint64_t evaluation_id)
{
  // Decide recovery under the lock; never call send_ecbs_seed_path_intent
  // (which acquires mutex_) while holding mutex_ — nested non-recursive lock.
  bool seed_after_unlock = false;
  uint64_t seed_recovery_txn = 0;
  {
  std::lock_guard<std::mutex> lock(mutex_);
  PathReleaseSession * session = gate_.session(robot_id);
  if (!session) {
    return;
  }
  if (session->pending_evaluation_id != evaluation_id) {
    RCLCPP_WARN(
      get_logger(),
      "[traffic_manager] Stale BaseExtensionQuery response robot=[%s] "
      "eval_id=%lu pending=%lu — reject",
      robot_id.c_str(), static_cast<unsigned long>(evaluation_id),
      static_cast<unsigned long>(session->pending_evaluation_id));
    return;
  }
  gate_.set_ecbs_in_flight(robot_id, false);
  if (session->content_hash != session_hash) {
    return;
  }

  std::shared_ptr<ecbs_planner::srv::BaseExtensionQuery::Response> response;
  try {
    response = future.get();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      get_logger(),
      "[traffic_manager] BaseExtensionQuery failed robot=[%s]: %s",
      robot_id.c_str(), e.what());
    gate_.latch_pending_extension(robot_id);
    return;
  }
  if (!response || !response->success) {
    gate_.latch_pending_extension(robot_id);
    return;
  }

  const std::string & status = response->status;
  if (status == "ROUTE_COMPLETE") {
    // Idle robot / whole route already released: nothing to release, no
    // reseed. (Previously STALE_ROUTE → recovery → seed → STALE hot loop.)
    gate_.clear_pending_extension(robot_id);
    session->held_at_wait = false;
    return;
  }
  if (status == "STALE_ROUTE" || status == "RECOVERY_REQUIRED" ||
    status == "WAIT" || status == "AUTHORITY_CONFLICT")
  {
    // Keep extension demand pending until fulfilled/cancelled/superseded.
    gate_.latch_pending_extension(robot_id);
  }
  if (status == "STALE_ROUTE") {
    RCLCPP_WARN(
      get_logger(),
      "[traffic_manager] STALE_ROUTE robot=[%s] tip=%d/%zu grant=%lu "
      "msg=[%s]",
      robot_id.c_str(),
      session->released_through_index,
      session->intent.path.size(),
      static_cast<unsigned long>(session->committed_grant_id),
      response->message.c_str());
    seed_after_unlock = begin_or_refresh_route_recovery_locked(
      *session, response->message);
    if (seed_after_unlock) {
      seed_recovery_txn = session->route_recovery_txn_id;
    }
    // Exit lock scope below — do not retain session* past unlock.
  } else {
  if (status == "AUTHORITY_CONFLICT") {
    RCLCPP_ERROR(
      get_logger(),
      "[traffic_manager] AUTHORITY_CONFLICT robot=[%s] conflict=%s grant=%lu "
      "— preserve tip; stop/reconcile required before extension",
      robot_id.c_str(),
      response->authority_conflict.conflict_id.c_str(),
      static_cast<unsigned long>(response->authority_conflict.grant_id));
    session->authority_conflict_active = true;
    session->authority_conflict_id = response->authority_conflict.conflict_id;
    session->auth_phase = PathReleaseSession::AuthPhase::StopRequested;
    session->authority_stop_confirm_since_sec = -1.0;
    session->held_at_wait = true;
    session->fault_reason = "AUTHORITY_CONFLICT";
    // Preserve committed tip; stop constraint is separate (hold_*).
    const int32_t tip = std::max(0, session->released_through_index);
    int32_t stop_at = tip;
    // Prefer hazard wait point when present and inside the issued base.
    for (const auto & rec : response->relevant_conflicts) {
      if (rec.conflict_id != session->authority_conflict_id) {
        continue;
      }
      for (std::size_t i = 0; i < rec.participant_amrs.size(); ++i) {
        if (rec.participant_amrs[i] != robot_id) {
          continue;
        }
        if (i < rec.wait_occurrence.size()) {
          stop_at = static_cast<int32_t>(
            rec.wait_occurrence[i].route_occurrence);
        }
        break;
      }
    }
    stop_at = std::min(
      stop_at, static_cast<int32_t>(session->intent.path.size()) - 1);
    if (stop_at < 0) {
      stop_at = 0;
    }
    freeze_wait_hold(*session, stop_at, session->intent.path);
    auto release = make_release_msg(*session, true, tip);
    release.ecbs_status = "WAIT";
    release.hold_leg_index = session->frozen_hold_leg;
    release.hold_x = session->frozen_hold_x;
    release.hold_y = session->frozen_hold_y;
    // Name conflict peers so markers / PathRelease show blockers (not []).
    for (const auto & rec : response->relevant_conflicts) {
      if (rec.conflict_id != session->authority_conflict_id &&
        !session->authority_conflict_id.empty())
      {
        continue;
      }
      for (const auto & peer : rec.participant_amrs) {
        if (peer.empty() || peer == robot_id) {
          continue;
        }
        if (std::find(
            release.blocking_amr_ids.begin(), release.blocking_amr_ids.end(),
            peer) == release.blocking_amr_ids.end())
        {
          release.blocking_amr_ids.push_back(peer);
        }
      }
    }
    // Same-tip AUTHORITY WAIT must NOT burn a new release_update_id.
    // Execution keeps orderUpdateId from the last PathRequest (tip unchanged),
    // so bumping 0→1 left TM expecting slot 1 while the robot ACK'd 0 forever
    // (no further BaseExtensionQuery). Reuse last id; still await ACK so
    // STOP_ACCEPTED can arm from the matching slot.
    const bool same_tip_reuse =
      session->next_release_update_id > 0 && tip >= 0 &&
      tip == session->released_through_index;
    if (same_tip_reuse) {
      release.release_update_id = session->last_published_release_update_id;
      RCLCPP_INFO(
        get_logger(),
        "[traffic_manager] AUTHORITY same-tip WAIT robot=[%s] tip=%d — "
        "reuse release_update_id=%u (do not bump ACK chain)",
        robot_id.c_str(), tip, release.release_update_id);
    } else {
      release.release_update_id = session->next_release_update_id;
      session->last_published_release_update_id = release.release_update_id;
      session->next_release_update_id += 1;
    }
    session->awaiting_robot_ack = true;
    session->last_hold_published = true;
    session->last_pub_hold_through = tip;
    session->last_pub_hold_leg = release.hold_leg_index;
    session->last_pub_hold_x = release.hold_x;
    session->last_pub_hold_y = release.hold_y;
    session->last_pub_blocking_amrs = release.blocking_amr_ids;
    publish_release(release);
    return;
  }
  if (status == "RECOVERY_REQUIRED") {
    session->fault_reason = response->message;
    session->held_at_wait = true;
    return;
  }
  if (status == "WAIT") {
    // Only retain overlaps this robot participates in (waiter bookkeeping).
    retain_participant_overlaps(*session, *response, session->committed_grant_id);
    session->held_at_wait = true;
    const int32_t tip = session->released_through_index;
    // Empty waiting_for + no blocker: do not invent hold@wp0 or bump
    // release_update_id (that burned orderUpdateId=3 with same tip).
    if (response->waiting_for.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "[traffic_manager] WAIT robot=[%s] tip=%d waiters=0 reason=[%s] — "
        "no PathRelease (keep tip; re-query on BaseRequest)",
        robot_id.c_str(), tip, response->wait_reason.c_str());
      return;
    }
    int32_t stop_at = tip >= 0 ? tip : 0;
    if (response->waiting_for.front().wait_point.index >= 0) {
      stop_at = static_cast<int32_t>(
        response->waiting_for.front().wait_point.index);
    }
    stop_at = std::min(
      stop_at, static_cast<int32_t>(session->intent.path.size()) - 1);
    if (stop_at < 0) {
      stop_at = 0;
    }
    // Pure WAIT never invents a new tip from stop_at — approach needs a grant.
    if (tip >= 0 && stop_at < tip) {
      session->fault_reason = "stop_inside_issued_base";
    }
    freeze_wait_hold(*session, stop_at, session->intent.path);
    auto release = make_release_msg(*session, true, tip);
    release.ecbs_status = "WAIT";
    release.hold_x = session->frozen_hold_x;
    release.hold_y = session->frozen_hold_y;
    release.hold_leg_index = session->frozen_hold_leg;
    for (const auto & d : response->waiting_for) {
      if (!d.amr_id.empty()) {
        release.blocking_amr_ids.push_back(d.amr_id);
      }
    }
    const bool hold_changed =
      !session->last_hold_published ||
      session->last_pub_hold_through != tip ||
      session->last_pub_hold_leg != release.hold_leg_index;
    if (hold_changed) {
      // Tip unchanged → reuse release_update_id (same ACK rule as AUTHORITY).
      const bool same_tip_reuse =
        session->next_release_update_id > 0 && tip >= 0 &&
        tip == session->released_through_index &&
        session->last_pub_hold_through == tip;
      if (same_tip_reuse) {
        release.release_update_id = session->last_published_release_update_id;
      } else {
        release.release_update_id = session->next_release_update_id;
        session->last_published_release_update_id = release.release_update_id;
        session->next_release_update_id += 1;
      }
      session->awaiting_robot_ack = true;
      session->last_hold_published = true;
      session->last_pub_hold_through = tip;
      session->last_pub_hold_leg = release.hold_leg_index;
      session->last_pub_hold_x = release.hold_x;
      session->last_pub_hold_y = release.hold_y;
      session->last_pub_blocking_amrs = release.blocking_amr_ids;
      publish_release(release);
    }
    return;
  }

  if (status != "FREE_PROPOSAL") {
    RCLCPP_WARN(
      get_logger(),
      "[traffic_manager] Unexpected BaseExtensionQuery status=[%s] robot=[%s]",
      status.c_str(), robot_id.c_str());
    gate_.latch_pending_extension(robot_id);
    return;
  }

  if (response->proposal.proposal_id == 0) {
    RCLCPP_WARN(
      get_logger(),
      "[traffic_manager] Reject FREE_PROPOSAL robot=[%s] — missing proposal_id",
      robot_id.c_str());
    gate_.latch_pending_extension(robot_id);
    return;
  }

  // Commit proposed endpoint, but never open more than query_horizon_m of *new*
  // path from the committed tip (yaml). ECBS approach-to-wait used to propose
  // 20–50 m tips and PathRelease accepted them verbatim.
  int32_t release_index = static_cast<int32_t>(
    response->proposal.to_occurrence.route_occurrence);
  if (release_index < 0 ||
    static_cast<std::size_t>(release_index) >= session->intent.path.size())
  {
    RCLCPP_WARN(
      get_logger(),
      "[traffic_manager] Reject FREE_PROPOSAL robot=[%s] — to_occurrence %d "
      "out of path range",
      robot_id.c_str(), release_index);
    commit_proposal_to_ecbs(
      *session, response->proposal.proposal_id, 0,
      session->released_through_index, "REJECTED", "invalid_to_occurrence");
    gate_.latch_pending_extension(robot_id);
    return;
  }
  {
    const int32_t origin = session->released_through_index >= 0 ?
      session->released_through_index : 0;
    const int32_t proposed = release_index;
    double travel = 0.0;
    int32_t capped = origin;
    for (int32_t i = origin + 1;
      i <= proposed &&
      static_cast<std::size_t>(i) < session->intent.path.size(); ++i)
    {
      const auto & a = session->intent.path[static_cast<std::size_t>(i - 1)];
      const auto & b = session->intent.path[static_cast<std::size_t>(i)];
      travel += std::hypot(b.x - a.x, b.y - a.y);
      capped = i;
      if (travel >= query_horizon_m_) {
        break;
      }
    }
    // Also respect the horizon slice computed when the query was sent.
    if (horizon_end_index >= 0) {
      capped = std::min(capped, horizon_end_index);
    }
    if (capped < release_index) {
      RCLCPP_WARN(
        get_logger(),
        "[traffic_manager] FREE_PROPOSAL robot=[%s] clamp tip %d→%d "
        "(query_horizon_m=%.1f travel=%.1f horizon_end=%d) — enforce yaml cap",
        robot_id.c_str(), release_index, capped, query_horizon_m_, travel,
        horizon_end_index);
      release_index = capped;
    }
  }
  // Dual-graph: ECBS proposes on ScheduleCache (/from_zone_manager); TM awards
  // PathIntent indices that full_control executes. Same occurrence can diverge
  // past 0.75 m from sampling — index + path_revision are the identity; snap XY.
  const auto & tip = session->intent.path[
    static_cast<std::size_t>(release_index)];
  {
    const double dx = tip.x - response->proposal.to_occurrence.x;
    const double dy = tip.y - response->proposal.to_occurrence.y;
    const double delta = std::hypot(dx, dy);
    if (delta > 0.75) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "[traffic_manager] FREE_PROPOSAL robot=[%s] occurrence %d XY delta=%.2fm "
        "(ecbs=%.2f,%.2f intent=%.2f,%.2f) — accepting index, snapping to PathIntent",
        robot_id.c_str(), release_index, delta,
        response->proposal.to_occurrence.x, response->proposal.to_occurrence.y,
        tip.x, tip.y);
    }
  }
  // Never expand past proposed tip; subset of prior tip is a no-op.
  if (session->released_through_index >= 0 &&
    release_index > session->released_through_index)
  {
    // ok — monotonic advance within proposal
  } else if (release_index < session->released_through_index) {
    RCLCPP_WARN(
      get_logger(),
      "[traffic_manager] Reject FREE_PROPOSAL robot=[%s] — would expand/shrink "
      "inconsistently (proposed=%d committed=%d)",
      robot_id.c_str(), release_index, session->released_through_index);
    commit_proposal_to_ecbs(
      *session, response->proposal.proposal_id, 0,
      session->released_through_index, "REJECTED", "inconsistent_range");
    gate_.latch_pending_extension(robot_id);
    return;
  }

  const int32_t w_start = std::max(
    0, static_cast<int32_t>(response->proposal.from_occurrence.route_occurrence));
  const int32_t w_from = session->released_through_index >= 0 ?
    std::max(session->released_through_index, w_start) : w_start;
  // ECBS is the single traffic authority: TM never second-guesses a proposal
  // (TM's own geometry check vetoed ECBS 10x/s and caused standoffs).
  if (!ecbs_authoritative_ && grant_hits_peer_live_base(
      *session, w_from, release_index, gate_, robot_id))
  {
    RCLCPP_WARN(
      get_logger(),
      "[traffic_manager] Reject FREE_PROPOSAL robot=[%s] — peer live base",
      robot_id.c_str());
    commit_proposal_to_ecbs(
      *session, response->proposal.proposal_id, 0, session->released_through_index,
      "REJECTED", "peer_live_base");
    gate_.latch_pending_extension(robot_id);
    return;
  }
  // Provisional multi-robot reserve: also reject against peer pending commits.
  // (Legacy TM arbitration — disabled while ECBS is authoritative.)
  if (!ecbs_authoritative_) {
    bool peer_pending_hit = false;
    for (const auto & peer_id : gate_.active_robot_ids()) {
      if (peer_id == robot_id) {
        continue;
      }
      const PathReleaseSession * peer = gate_.session(peer_id);
      if (!peer || peer->pending_commit_grant_id == 0 ||
        peer->pending_commit_release_index < 0)
      {
        continue;
      }
      const int32_t p_from = peer->released_through_index >= 0 ?
        peer->released_through_index : 0;
      const int32_t p_to = peer->pending_commit_release_index;
      if (grant_hits_peer_live_base(
          *session, w_from, release_index, gate_, robot_id))
      {
        peer_pending_hit = true;
        break;
      }
      // Geometric interval check against peer's pending [from,to] tips.
      if (!peer->intent.path.empty() && !session->intent.path.empty()) {
        const auto & sp = session->intent.path;
        const auto & pp = peer->intent.path;
        const int32_t slast = static_cast<int32_t>(sp.size()) - 1;
        const int32_t plast = static_cast<int32_t>(pp.size()) - 1;
        const int32_t s1 = std::min(release_index, slast);
        const int32_t p1 = std::min(p_to, plast);
        for (int32_t i = std::max(0, w_from); i <= s1; ++i) {
          for (int32_t j = std::max(0, p_from); j <= p1; ++j) {
            if (std::hypot(
                sp[static_cast<std::size_t>(i)].x -
                pp[static_cast<std::size_t>(j)].x,
                sp[static_cast<std::size_t>(i)].y -
                pp[static_cast<std::size_t>(j)].y) <= 1.0)
            {
              peer_pending_hit = true;
              break;
            }
          }
          if (peer_pending_hit) {
            break;
          }
        }
      }
    }
    if (peer_pending_hit) {
      RCLCPP_WARN(
        get_logger(),
        "[traffic_manager] Reject FREE_PROPOSAL robot=[%s] — peer pending "
        "provisional reservation",
        robot_id.c_str());
      commit_proposal_to_ecbs(
        *session, response->proposal.proposal_id, 0,
        session->released_through_index, "REJECTED", "peer_pending_reserve");
      gate_.latch_pending_extension(robot_id);
      return;
    }
  }

  session->pending_proposal_id = response->proposal.proposal_id;
  session->proposal_route_version = response->proposal.route_version;
  session->proposal_occupancy_version = response->proposal.occupancy_version;
  session->proposal_commitment_version = response->proposal.commitment_version;
  // Persist exact proposal identity for commit (never rebuild from session alone).
  session->pending_canonical_task_id = response->proposal.task_id;
  session->pending_execution_order_id = response->proposal.execution_order_id;
  session->pending_action_id = response->proposal.action_id;
  session->pending_path_revision = response->proposal.path_revision;
  {
    const uint32_t from_occ =
      response->proposal.from_occurrence.route_occurrence;
    if (from_occ < session->intent.path.size()) {
      const auto & from_tip = session->intent.path[from_occ];
      session->pending_from_x = from_tip.x;
      session->pending_from_y = from_tip.y;
    } else {
      session->pending_from_x = response->proposal.from_occurrence.x;
      session->pending_from_y = response->proposal.from_occurrence.y;
    }
    session->pending_from_occurrence = from_occ;
  }
  // PathIntent geometry is what PathRelease ships; commit tip = clamped index
  // (subset of proposal is allowed; to_occurrence must match release_through).
  session->pending_to_x = tip.x;
  session->pending_to_y = tip.y;
  session->pending_to_occurrence = static_cast<uint32_t>(release_index);
  session->authority_txn_phase =
    PathReleaseSession::AuthorityTxnPhase::Proposed;
  session->held_at_wait = false;
  // Authority resume only after STOP_CONFIRMED (not mere order ACK).
  if (session->authority_conflict_active) {
    if (session->auth_phase != PathReleaseSession::AuthPhase::StopConfirmed &&
      session->auth_phase != PathReleaseSession::AuthPhase::AuthorityReconciled)
    {
      gate_.latch_pending_extension(robot_id);
      return;
    }
    session->auth_phase = PathReleaseSession::AuthPhase::AuthorityReconciled;
    session->authority_conflict_active = false;
    session->authority_conflict_id.clear();
    session->authority_stop_confirm_since_sec = -1.0;
    session->fault_reason.clear();
  }
  clear_frozen_hold(*session);

  if (release_index <= session->released_through_index) {
    // T11/T12: same-tip FREE must not burn the BaseRequests rising edge —
    // sticky newBaseRequest=true otherwise never re-triggers after this clear.
    gate_.clear_pending_extension(robot_id);
    session->last_new_base_request = false;
    RCLCPP_INFO(
      get_logger(),
      "[traffic_manager] FREE_PROPOSAL robot=[%s] same tip=%d — no PathRelease "
      "(re-armed BaseRequests; extend when pose/horizon can advance)",
      robot_id.c_str(), release_index);
    return;
  }

  // Commit-then-publish: do not advance tip or PathRelease until ECBS ACCEPTED.
  const uint64_t grant_id = next_grant_id_++;
  session->pending_commit_grant_id = grant_id;
  session->pending_commit_release_index = release_index;
  session->authority_txn_phase =
    PathReleaseSession::AuthorityTxnPhase::LocallyReserved;
  retain_participant_overlaps(*session, *response, grant_id);
  session->authority_txn_phase =
    PathReleaseSession::AuthorityTxnPhase::CommitPending;
  commit_proposal_to_ecbs(
    *session, response->proposal.proposal_id, grant_id, release_index,
    "COMMITTED", "tm_validated_and_reserved", true);
  RCLCPP_INFO(
    get_logger(),
    "[traffic_manager] Commit pending acceptance proposal=%lu grant=%lu "
    "robot=[%s] through=%d (PathRelease deferred)",
    static_cast<unsigned long>(response->proposal.proposal_id),
    static_cast<unsigned long>(grant_id),
    robot_id.c_str(), release_index);
  }  // !STALE_ROUTE
  }  // mutex_ — session* must not be used after this

  if (seed_after_unlock) {
    // Nested-lock fix: seed acquires mutex_ itself.
    send_ecbs_seed_path_intent(robot_id, false, seed_recovery_txn);
  }
}

void TrafficManagerNode::send_ecbs_clear(const std::string & robot_id)
{
  if (robot_id.empty() || !ecbs_client_->service_is_ready()) {
    return;
  }
  auto request = std::make_shared<ecbs_planner::srv::EvaluatePath::Request>();
  request->mode = "clear";
  request->amr_id = robot_id;
  request->query_horizon_m = 0.0;
  RCLCPP_INFO(
    get_logger(),
    "[TrafficArbitrate] ECBS clear ScheduleCache robot=[%s]",
    robot_id.c_str());
  ecbs_client_->async_send_request(
    request,
    [this, robot_id](
      rclcpp::Client<ecbs_planner::srv::EvaluatePath>::SharedFuture future)
    {
      const auto response = future.get();
      if (!response || !response->success) {
        RCLCPP_WARN(
          get_logger(),
          "[TrafficArbitrate] ECBS clear failed robot=[%s]: %s",
          robot_id.c_str(),
          response ? response->message.c_str() : "null");
        return;
      }
      RCLCPP_INFO(
        get_logger(),
        "[TrafficArbitrate] ECBS clear ok robot=[%s] msg=[%s]",
        robot_id.c_str(),
        response->message.c_str());
    });
}

void TrafficManagerNode::send_ecbs_seed_path_intent(
  const std::string & robot_id,
  const bool then_schedule_initial_eval,
  const uint64_t recovery_txn_id)
{
  // LOCKING: this function acquires mutex_. Callers must not already hold it.
  if (robot_id.empty() || !ecbs_client_->service_is_ready()) {
    if (recovery_txn_id != 0) {
      std::lock_guard<std::mutex> lock(mutex_);
      if (PathReleaseSession * session = gate_.session(robot_id)) {
        complete_route_recovery_seed_locked(
          *session, recovery_txn_id, false, "ecbs_client_not_ready");
      }
    }
    if (then_schedule_initial_eval) {
      schedule_pending_initial_ecbs_eval(robot_id);
    }
    return;
  }
  auto request = std::make_shared<ecbs_planner::srv::EvaluatePath::Request>();
  int32_t preserved_tip = -1;
  uint64_t preserved_grant = 0;
  std::size_t full_route_wps = 0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    PathReleaseSession * session = gate_.session(robot_id);
    if (!session || session->intent.path.empty()) {
      return;
    }
    // Horizon seed must never erase tip/grant — copy full-route geometry only.
    preserved_tip = session->released_through_index;
    preserved_grant = session->committed_grant_id;
    full_route_wps = session->intent.path.size();
    request->mode = "evaluate";
    request->amr_id = robot_id;
    request->fleet_name = session->intent.fleet_name;
    const std::string canonical = !session->intent.dispatch_task_id.empty() ?
      session->intent.dispatch_task_id : session->intent.task_id;
    request->task_id = send_task_id_to_ecbs_ ? canonical : std::string();
    if (ecbs_authoritative_) {
      // Register exactly which PathIntent session this route is: a repeated
      // identical leg (loop / patrol) is a NEW registration, a recovery
      // reseed of the same session is not.
      request->task_id = canonical + "#req" + std::to_string(session->intent.request_id);
    }
    request->phase_index = static_cast<int32_t>(session->intent.phase_index);
    request->path_revision = static_cast<int32_t>(session->intent.path_revision);
    request->waypoints = to_ecbs_waypoints(session->intent.path);
    request->query_horizon_m = query_horizon_m_;
    request->current_x = session->intent.path.front().x;
    request->current_y = session->intent.path.front().y;
    const auto state_it = robot_states_.find(robot_id);
    if (state_it != robot_states_.end()) {
      request->current_x = state_it->second.location.x;
      request->current_y = state_it->second.location.y;
    }
  }
  RCLCPP_INFO(
    get_logger(),
    "[traffic_manager] ECBS seed PathIntent robot=[%s] waypoints=%zu "
    "preserved_tip=%d preserved_grant=%lu recovery_txn=%lu",
    robot_id.c_str(), request->waypoints.size(),
    preserved_tip,
    static_cast<unsigned long>(preserved_grant),
    static_cast<unsigned long>(recovery_txn_id));
  ecbs_client_->async_send_request(
    request,
    [this, robot_id, then_schedule_initial_eval, recovery_txn_id,
      preserved_tip, preserved_grant, full_route_wps](
      rclcpp::Client<ecbs_planner::srv::EvaluatePath>::SharedFuture future)
    {
      std::shared_ptr<ecbs_planner::srv::EvaluatePath::Response> response;
      try {
        response = future.get();
      } catch (const std::exception & e) {
        RCLCPP_ERROR(
          get_logger(),
          "[traffic_manager] ECBS seed PathIntent exception robot=[%s]: %s",
          robot_id.c_str(), e.what());
      }
      const bool seed_ok = response && response->success;
      const std::string seed_msg = response ?
        response->message : "null_or_exception";
      if (!seed_ok) {
        RCLCPP_WARN(
          get_logger(),
          "[traffic_manager] ECBS seed PathIntent failed robot=[%s]: %s",
          robot_id.c_str(), seed_msg.c_str());
      } else {
        RCLCPP_INFO(
          get_logger(),
          "[traffic_manager] ECBS seed PathIntent ok robot=[%s] status=[%s] "
          "full_route_wps=%zu preserved_tip=%d",
          robot_id.c_str(),
          response->status.c_str(),
          full_route_wps,
          preserved_tip);
      }

      bool tip_extend = false;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        PathReleaseSession * session = gate_.session(robot_id);
        if (!session) {
          return;
        }
        // Identity-gated recovery completion; ignore stale async responses.
        if (recovery_txn_id != 0) {
          tip_extend = complete_route_recovery_seed_locked(
            *session, recovery_txn_id, seed_ok, seed_msg);
        } else if (seed_ok) {
          // Initial / non-recovery seed: still never shrink tip or drop grant.
          if (session->released_through_index < preserved_tip) {
            session->released_through_index = preserved_tip;
          }
          if (session->committed_grant_id == 0 && preserved_grant != 0) {
            session->committed_grant_id = preserved_grant;
          }
        }
      }

      if (then_schedule_initial_eval) {
        schedule_pending_initial_ecbs_eval(robot_id);
      } else if (tip_extend) {
        // Recovery seed OK — rolling extension from existing committed tip.
        request_base_extension_query(robot_id, true, true);
      } else if (recovery_txn_id == 0 && seed_ok) {
        // Non-recovery reseed (e.g. explicit caller): tip-extend without new task.
        request_base_extension_query(robot_id, true, true);
      }
      // Failed recovery: demand retained; eval-timer backoff retries seed.
    });
}

traffic_manager::msg::PathRelease TrafficManagerNode::make_release_msg(
  const PathReleaseSession & session,
  const bool approved,
  const int32_t release_through_index) const
{
  traffic_manager::msg::PathRelease release;
  release.stamp = now();
  release.fleet_name = session.intent.fleet_name;
  release.robot_name = session.intent.robot_name;
  release.request_id = session.intent.request_id;
  release.task_id = session.intent.task_id;
  release.dispatch_task_id = session.intent.dispatch_task_id;
  release.phase_index = session.intent.phase_index;
  release.path_revision = session.intent.path_revision;
  release.approved = approved;
  release.release_through_index = release_through_index;
  return release;
}

void TrafficManagerNode::commit_proposal_to_ecbs(
  const PathReleaseSession & session,
  const uint64_t proposal_id,
  const uint64_t grant_id,
  const int32_t release_through_index,
  const std::string & status,
  const std::string & reason,
  const bool publish_on_accept)
{
  ecbs_planner::msg::GrantEvent ev;
  ev.stamp = now();
  ev.grant_id = grant_id;
  ev.proposal_id = proposal_id;
  ev.amr_id = session.intent.robot_name;
  ev.fleet_name = session.intent.fleet_name;
  ev.task_id = session.intent.task_id;
  ev.phase_index = static_cast<int32_t>(session.intent.phase_index);
  ev.path_revision = session.intent.path_revision;
  ev.status = status;
  ev.reason = reason;
  ev.release_through_index = release_through_index;
  ev.route_version = session.proposal_route_version;
  ev.occupancy_version = session.proposal_occupancy_version;
  ev.commitment_version = session.proposal_commitment_version;
  if (release_through_index >= 0 &&
    static_cast<std::size_t>(release_through_index) < session.intent.path.size())
  {
    const auto & loc = session.intent.path[
      static_cast<std::size_t>(release_through_index)];
    ev.to_occurrence.path_revision = session.intent.path_revision;
    ev.to_occurrence.route_occurrence =
      static_cast<uint32_t>(release_through_index);
    ev.to_occurrence.x = loc.x;
    ev.to_occurrence.y = loc.y;
    ev.to_occurrence.level_name = loc.level_name;
  }
  // Publish GrantEvent after acceptance when publish_on_accept; otherwise now
  // (REJECTED / COMPLETED notifications).
  if (grant_event_pub_ && !publish_on_accept) {
    grant_event_pub_->publish(ev);
  }

  if (!commit_client_) {
    // Callers already hold mutex_ (FREE_PROPOSAL / EvaluatePath tip paths).
    // Do not re-acquire — nested non-recursive lock deadlocks the node.
    if (publish_on_accept) {
      gate_.latch_pending_extension(session.intent.robot_name);
    }
    return;
  }
  auto req = std::make_shared<ecbs_planner::srv::CommitProposal::Request>();
  req->proposal_id = proposal_id;
  req->grant_id = grant_id;
  req->amr_id = session.intent.robot_name;
  req->fleet_name = session.intent.fleet_name;
  // Exact proposal identity — never reconstruct order id as route task_id.
  req->task_id = !session.pending_canonical_task_id.empty() ?
    session.pending_canonical_task_id :
    (!session.intent.dispatch_task_id.empty() ?
      session.intent.dispatch_task_id : session.intent.task_id);
  req->execution_order_id = !session.pending_execution_order_id.empty() ?
    session.pending_execution_order_id : session.intent.task_id;
  req->action_id = session.pending_action_id;
  req->phase_index = static_cast<int32_t>(session.intent.phase_index);
  req->path_revision = session.pending_path_revision != 0 ?
    session.pending_path_revision : session.intent.path_revision;
  req->status = status;
  req->reason = reason;
  req->release_through_index = release_through_index;
  req->from_occurrence.path_revision = req->path_revision;
  req->from_occurrence.route_occurrence = session.pending_from_occurrence;
  req->from_occurrence.x = session.pending_from_x;
  req->from_occurrence.y = session.pending_from_y;
  req->to_occurrence = ev.to_occurrence;
  if (session.pending_to_occurrence != 0 || session.pending_to_x != 0.0 ||
    session.pending_to_y != 0.0)
  {
    req->to_occurrence.path_revision = req->path_revision;
    req->to_occurrence.route_occurrence = session.pending_to_occurrence;
    req->to_occurrence.x = session.pending_to_x;
    req->to_occurrence.y = session.pending_to_y;
  }
  req->route_version = session.proposal_route_version;
  req->occupancy_version = session.proposal_occupancy_version;
  req->commitment_version = session.proposal_commitment_version;
  if (status == "COMMITTED") {
    // Const session — phase bump happens on PathReleaseSession via caller.
  }
  const std::string robot_id = session.intent.robot_name;
  const auto grant_ev = ev;
  commit_client_->async_send_request(
    req,
    [this, robot_id, status, publish_on_accept, grant_id,
      release_through_index, grant_ev](
      rclcpp::Client<ecbs_planner::srv::CommitProposal>::SharedFuture future)
    {
      // Never call rebuild_path_release_markers() while holding mutex_ —
      // that function locks mutex_ again (non-recursive) and deadlocks.
      // Update authority under the lock, copy PathRelease, then publish outside.
      bool publish_release_msg = false;
      bool refresh_markers = false;
      traffic_manager::msg::PathRelease release_copy;
      std::string action_id;
      std::string canonical_task;
      std::string execution_order;
      try {
        const auto res = future.get();
        {
          std::lock_guard<std::mutex> lock(mutex_);
          PathReleaseSession * session = gate_.session(robot_id);
          if (!res || !res->success) {
            if (session && session->pending_commit_grant_id == grant_id) {
              session->pending_commit_grant_id = 0;
              session->pending_commit_release_index = -1;
            }
            gate_.latch_pending_extension(robot_id);
            RCLCPP_WARN(
              get_logger(),
              "[traffic_manager] CommitProposal %s rejected robot=[%s] result=%s "
              "— no PathRelease; latch retry",
              status.c_str(), robot_id.c_str(),
              res ? res->result.c_str() : "null");
            return;
          }
          if (publish_on_accept && session &&
            session->pending_commit_grant_id == grant_id)
          {
            gate_.set_released_through(robot_id, release_through_index);
            session->committed_grant_id = grant_id;
            session->pending_commit_grant_id = 0;
            session->pending_commit_release_index = -1;
            session->authority_txn_phase =
              PathReleaseSession::AuthorityTxnPhase::Committed;
            gate_.clear_pending_extension(robot_id);
            release_copy = make_release_msg(
              *session, true, release_through_index);
            release_copy.ecbs_status = "FREE";
            release_copy.release_update_id = session->next_release_update_id;
            session->last_published_release_update_id =
              release_copy.release_update_id;
            session->next_release_update_id += 1;
            session->awaiting_robot_ack = true;
            session->authority_txn_phase =
              PathReleaseSession::AuthorityTxnPhase::Published;
            action_id = session->pending_action_id;
            canonical_task = session->pending_canonical_task_id;
            execution_order = session->pending_execution_order_id;
            publish_release_msg = true;
            refresh_markers = true;
          }
        }  // release mutex_ before publish / marker rebuild

        if (publish_release_msg) {
          if (grant_event_pub_) {
            grant_event_pub_->publish(grant_ev);
          }
          // Commit-before-release ordering preserved: tip/update id already
          // advanced under the lock; PathRelease publish is after ACCEPTED.
          publish_release(release_copy);
          if (refresh_markers) {
            rebuild_path_release_markers();
          }
          RCLCPP_INFO(
            get_logger(),
            "[traffic_manager] Commit ACCEPTED grant=%lu robot=[%s] through=%d "
            "action=[%s] task=[%s] order=[%s] — PathRelease published",
            static_cast<unsigned long>(grant_id), robot_id.c_str(),
            release_through_index,
            action_id.c_str(),
            canonical_task.c_str(),
            execution_order.c_str());
        }
      } catch (const std::exception & e) {
        // Timeout / transport failure: retain provisional reservation and retry
        // the same transaction — never release potentially authorized space.
        std::lock_guard<std::mutex> lock(mutex_);
        gate_.latch_pending_extension(robot_id);
        RCLCPP_ERROR(
          get_logger(),
          "[traffic_manager] CommitProposal failed robot=[%s]: %s "
          "(retaining pending_commit grant=%lu for retry)",
          robot_id.c_str(), e.what(),
          static_cast<unsigned long>(grant_id));
      }
    });
}

void TrafficManagerNode::publish_clearance_event(
  const std::string & amr_id,
  const std::string & conflict_id,
  const bool physically_cleared,
  const std::string & reason)
{
  if (!clearance_event_pub_) {
    return;
  }
  ecbs_planner::msg::ClearanceEvent ev;
  ev.stamp = now();
  ev.amr_id = amr_id;
  ev.conflict_id = conflict_id;
  ev.physically_cleared = physically_cleared;
  ev.reason = reason;
  const auto it = robot_states_.find(amr_id);
  if (it != robot_states_.end()) {
    ev.observed_x = it->second.location.x;
    ev.observed_y = it->second.location.y;
  }
  clearance_event_pub_->publish(ev);
  // Event-driven wake: retain/retry extension demand for the clearer and peers.
  if (physically_cleared) {
    gate_.latch_pending_extension(amr_id);
    if (auto * s = gate_.session(amr_id)) {
      for (const auto & t : s->tracked_overlaps) {
        if (t.conflict_id == conflict_id && !t.peer_amr.empty()) {
          gate_.latch_pending_extension(t.peer_amr);
        }
      }
    }
  }
}

void TrafficManagerNode::retain_participant_overlaps(
  PathReleaseSession & session,
  const ecbs_planner::srv::BaseExtensionQuery::Response & response,
  const uint64_t grant_id)
{
  const std::string & self = session.intent.robot_name;
  std::unordered_set<std::string> keep;
  for (const auto & id : response.proposal.required_conflict_ids) {
    if (!id.empty()) {
      keep.insert(id);
    }
  }

  for (const auto & rec : response.relevant_conflicts) {
    if (rec.conflict_id.empty() || rec.physically_cleared) {
      continue;
    }
    std::size_t self_i = rec.participant_amrs.size();
    for (std::size_t i = 0; i < rec.participant_amrs.size(); ++i) {
      if (rec.participant_amrs[i] == self) {
        self_i = i;
        break;
      }
    }
    if (self_i >= rec.participant_amrs.size()) {
      continue;  // not a participant — never track foreign overlaps
    }
    if (!keep.empty() && keep.count(rec.conflict_id) == 0 &&
      !(rec.mandatory && rec.blocker_amr == self))
    {
      continue;
    }
    // Preserve prior clearance for the same conflict/traversal.
    bool was_cleared = false;
    for (const auto & cur : session.tracked_overlaps) {
      if (cur.conflict_id == rec.conflict_id &&
        cur.path_revision == session.intent.path_revision &&
        cur.cleared)
      {
        was_cleared = true;
        break;
      }
    }
    PathReleaseSession::TrackedOverlapClearance t;
    t.conflict_id = rec.conflict_id;
    t.grant_id = grant_id;
    t.path_revision = session.intent.path_revision;
    if (self_i < rec.entry_occurrence.size()) {
      t.entry_x = rec.entry_occurrence[self_i].x;
      t.entry_y = rec.entry_occurrence[self_i].y;
    }
    if (self_i < rec.exit_occurrence.size()) {
      t.exit_x = rec.exit_occurrence[self_i].x;
      t.exit_y = rec.exit_occurrence[self_i].y;
      t.exit_occurrence = rec.exit_occurrence[self_i].route_occurrence;
    }
    for (const auto & peer : rec.participant_amrs) {
      if (peer != self) {
        t.peer_amr = peer;
        break;
      }
    }
    // Blocker obligation only — waiters do not clear this overlap.
    t.as_blocker = rec.blocker_amr == self ||
      (rec.blocker_amr.empty() && !rec.mandatory && grant_id > 0);
    t.cleared = was_cleared;
    bool exists = false;
    for (auto & cur : session.tracked_overlaps) {
      if (cur.conflict_id == t.conflict_id) {
        const bool keep_cleared = cur.cleared;
        cur = t;
        cur.cleared = keep_cleared || was_cleared;
        exists = true;
        break;
      }
    }
    if (!exists) {
      session.tracked_overlaps.push_back(std::move(t));
    }
  }
}

bool TrafficManagerNode::footprint_cleared_exit(
  const PathReleaseSession & session,
  const PathReleaseSession::TrackedOverlapClearance & t,
  const double px,
  const double py,
  const int32_t progress) const
{
  if (t.path_revision != 0 &&
    t.path_revision != session.intent.path_revision)
  {
    return false;
  }
  if (session.telemetry_age_sec > 2.0) {
    return false;  // stale pose — not evidence
  }
  const double rear_m = 0.5 * amr_length_m_ + clearance_uncertainty_m_;
  // Direction of travel through the overlap (entry → exit).
  double dx = t.exit_x - t.entry_x;
  double dy = t.exit_y - t.entry_y;
  const double seg_len = std::hypot(dx, dy);
  if (seg_len < 1e-3) {
    // Degenerate: require progress past exit AND pose beyond exit by rear_m
    // along the remaining path direction if available.
    if (progress < 0 ||
      static_cast<uint32_t>(progress) <= t.exit_occurrence)
    {
      return false;
    }
    return std::hypot(px - t.exit_x, py - t.exit_y) >= rear_m;
  }
  dx /= seg_len;
  dy /= seg_len;
  // Longitudinal distance of centre past the exit along travel direction.
  const double past = (px - t.exit_x) * dx + (py - t.exit_y) * dy;
  // Centre past exit by rear envelope ⇒ rear/load outside the overlap.
  if (past < rear_m) {
    return false;
  }
  // Also require route occurrence past exit (matching revision identity).
  if (progress < 0 ||
    static_cast<uint32_t>(progress) < t.exit_occurrence)
  {
    return false;
  }
  return true;
}

void TrafficManagerNode::evaluate_grant_clearance_from_progress(
  const std::string & robot_id)
{
  PathReleaseSession * session = gate_.session(robot_id);
  if (!session) {
    return;
  }
  const int32_t progress = session->observed_progress_index >= 0 ?
    session->observed_progress_index : session->leg_cursor_index;
  const auto state_it = robot_states_.find(robot_id);
  if (state_it == robot_states_.end()) {
    return;
  }
  const double px = state_it->second.location.x;
  const double py = state_it->second.location.y;

  // 1) Clear individual overlap constraints with footprint evidence only.
  for (auto & t : session->tracked_overlaps) {
    if (t.cleared || !t.as_blocker) {
      continue;
    }
    if (!footprint_cleared_exit(*session, t, px, py, progress)) {
      continue;
    }
    publish_clearance_event(
      robot_id, t.conflict_id, true, "footprint_cleared_exit");
    t.cleared = true;
  }

  // 2) Grant completion is independent: entire issued tip reconciled.
  if (session->committed_grant_id == 0) {
    return;
  }
  const int32_t tip = session->released_through_index;
  if (tip < 0) {
    return;
  }
  const bool tip_reached = progress >= tip;
  double tip_x = 0.0;
  double tip_y = 0.0;
  if (static_cast<std::size_t>(tip) < session->intent.path.size()) {
    tip_x = session->intent.path[static_cast<std::size_t>(tip)].x;
    tip_y = session->intent.path[static_cast<std::size_t>(tip)].y;
  }
  const double rear_m = 0.5 * amr_length_m_ + clearance_uncertainty_m_;
  // At tip: centre within a short radius OR idle at destination.
  const bool at_tip_pose = std::hypot(px - tip_x, py - tip_y) <= rear_m;
  const bool idle =
    state_it->second.mode.mode == rmf_fleet_msgs::msg::RobotMode::MODE_IDLE ||
    state_it->second.mode.mode == rmf_fleet_msgs::msg::RobotMode::MODE_CHARGING;
  if (tip_reached && (at_tip_pose || idle || session->task_leg_complete)) {
    publish_grant_completed(
      *session, session->committed_grant_id, "grant_tip_reconciled");
    session->committed_grant_id = 0;
    // Destination occupancy remains until a new task takes over.
    session->destination_occupied = true;
  }
}

void TrafficManagerNode::publish_grant_completed(
  const PathReleaseSession & session,
  const uint64_t grant_id,
  const std::string & reason)
{
  if (grant_id == 0) {
    return;
  }
  ecbs_planner::msg::GrantEvent ev;
  ev.stamp = now();
  ev.grant_id = grant_id;
  ev.proposal_id = session.pending_proposal_id;
  ev.amr_id = session.intent.robot_name;
  ev.fleet_name = session.intent.fleet_name;
  ev.task_id = session.intent.task_id;
  ev.phase_index = static_cast<int32_t>(session.intent.phase_index);
  ev.path_revision = session.intent.path_revision;
  ev.status = "COMPLETED";
  ev.reason = reason;
  ev.release_through_index = session.released_through_index;
  if (grant_event_pub_) {
    grant_event_pub_->publish(ev);
  }
  RCLCPP_INFO(
    get_logger(),
    "[traffic_manager] Grant COMPLETED robot=[%s] grant=%lu reason=%s",
    session.intent.robot_name.c_str(),
    static_cast<unsigned long>(grant_id), reason.c_str());
}

void TrafficManagerNode::publish_release(
  const traffic_manager::msg::PathRelease & release)
{
  release_pub_->publish(release);
  if (release.approved && release.release_through_index >= 0) {
    RCLCPP_INFO(
      get_logger(),
      "[traffic_manager] Published PathRelease robot=[%s] request_id=%lu "
      "through_index=%d (%d waypoints) release_update_id=%u ecbs_status=[%s] "
      "hold_leg=%d phase=%u revision=%u → full_control",
      release.robot_name.c_str(),
      release.request_id,
      release.release_through_index,
      release.release_through_index + 1,
      release.release_update_id,
      release.ecbs_status.c_str(),
      release.hold_leg_index,
      release.phase_index,
      release.path_revision);
    if (release.ecbs_status == "WAIT") {
      RCLCPP_INFO(
        get_logger(),
        "[traffic_manager] PathRelease WAIT robot=[%s] hold@(%.3f, %.3f) "
        "wait_sec=%.1f blocking=[%s]",
        release.robot_name.c_str(),
        release.hold_x,
        release.hold_y,
        release.hold_wait_sec,
        release.blocking_amr_ids.empty() ? "" : release.blocking_amr_ids.front().c_str());
    }
  } else {
    RCLCPP_WARN(
      get_logger(),
      "[traffic_manager] Published PathRelease DENIED robot=[%s] request_id=%lu",
      release.robot_name.c_str(),
      release.request_id);
  }
}

void TrafficManagerNode::rebuild_path_release_markers()
{
  // LOCKING CONTRACT: do not call while mutex_ is held. This acquires mutex_
  // with a non-recursive std::mutex; nested calls deadlock the node (no further
  // /BaseRequests, Commit ACCEPTED log missing, SIGKILL on shutdown).
  if (!publish_path_release_markers_ || !path_release_marker_pub_) {
    return;
  }

  const auto stamp = now();
  visualization_msgs::msg::MarkerArray array;
  // No DELETEALL: RViz draws it before the re-adds, so every rebuild (each
  // cursor step while robots drive) blinked. Markers are replaced in place
  // by (ns, id); ids the previous frame had and this one lacks are deleted
  // below.

  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto & robot_id : gate_.active_robot_ids()) {
      const PathReleaseSession * session = gate_.session(robot_id);
      if (session == nullptr || session->intent.path.empty()) {
        continue;
      }
      // Yellow/blue split matches PathRequest Location.release flags.
      // Red hold from /to_zone_manager wait XY (direct), not PathRelease-only.
      int32_t hold_index = -1;
      if (session->to_zone_has_wait) {
        double best = 1e100;
        for (std::size_t i = 0; i < session->intent.path.size(); ++i) {
          const double d = std::hypot(
            session->intent.path[i].x - session->to_zone_wait_x,
            session->intent.path[i].y - session->to_zone_wait_y);
          if (d < best) {
            best = d;
            hold_index = static_cast<int32_t>(i);
          }
        }
      } else if (session->held_at_wait) {
        hold_index = session->hold_through_index >= 0 ?
          session->hold_through_index : session->released_through_index;
      }
      // Yellow = ALLOWED: only the stretch the robot has acknowledged in its
      // state (order update published AND ACKed). A committed extension that
      // is still awaiting the robot's ACK stays blue (planned, not allowed).
      const int32_t allowed_through = session->awaiting_robot_ack ?
        session->acknowledged_grant : session->released_through_index;
      append_path_release_markers(
        array,
        stamp,
        robot_id,
        session->intent.path,
        allowed_through,
        session->leg_cursor_index,
        hold_index,
        marker_frame_id_,
        use_level_name_as_marker_frame_);
    }
  }

  {
    std::set<std::pair<std::string, int32_t>> current;
    for (const auto & m : array.markers) {
      current.emplace(m.ns, m.id);
    }
    for (const auto & m : last_path_release_markers_.markers) {
      if (m.action != visualization_msgs::msg::Marker::ADD ||
        current.count({m.ns, m.id}) > 0)
      {
        continue;
      }
      visualization_msgs::msg::Marker del;
      del.header.frame_id = m.header.frame_id;
      del.header.stamp = stamp;
      del.ns = m.ns;
      del.id = m.id;
      del.action = visualization_msgs::msg::Marker::DELETE;
      array.markers.push_back(del);
    }
  }

  last_path_release_markers_ = array;
  path_release_marker_pub_->publish(array);
}

void TrafficManagerNode::on_marker_republish_timer()
{
  if (!publish_path_release_markers_ || !path_release_marker_pub_) {
    return;
  }
  // A robot ACKed an order update: redraw so its new stretch turns yellow.
  if (release_markers_dirty_.exchange(false)) {
    rebuild_path_release_markers();
    return;
  }
  if (last_path_release_markers_.markers.empty()) {
    return;
  }
  const auto stamp = now();
  for (auto & marker : last_path_release_markers_.markers) {
    marker.header.stamp = stamp;
  }
  path_release_marker_pub_->publish(last_path_release_markers_);
}

}  // namespace traffic_manager
