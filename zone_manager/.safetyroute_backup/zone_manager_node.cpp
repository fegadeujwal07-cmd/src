// =============================================================================
// File: zone_manager/src/zone_manager_node.cpp
// =============================================================================
// OBJECTIVE
//   Zone conflict detection + /zone_conflicts handshake (startPause/stopPause).
//   On Detected: classify direction/topology (C1–C5) and pause accordingly.
//   Opposite (C2/C4): pause both and publish /replan_candidate; arm a watch for
//   the candidate's new /Monitored_task_details plan, then re-decide without
//   waiting for an occupancy edge.
//
// INPUTS
//   /zone_catalog, /robot_zone_status (ETA, zone_sequence), /fleet_states,
//   /Monitored_task_details (priority + plan fingerprint for post-replan)
// OUTPUTS
//   /zone_conflicts  type, zone_id, robot_ids[], action
//   /replan_candidate  zone_id, robot_id (opposite conflicts only; once per arm)
// =============================================================================

#include "zone_manager/zone_manager_node.hpp"

#include <algorithm>
#include <sstream>
#include <utility>
#include <vector>

namespace zone_manager
{
namespace
{
constexpr const char * kActionStartPause = "startPause";
constexpr const char * kActionStopPause = "stopPause";

std::string join_ids(const std::vector<std::string> & ids)
{
  std::ostringstream oss;
  for (std::size_t i = 0; i < ids.size(); ++i) {
    if (i) {
      oss << ", ";
    }
    oss << ids[i];
  }
  return oss.str();
}
}  // namespace

ZoneManagerNode::ZoneManagerNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("zone_manager", options)
{
  declare_parameter<std::string>("fleet_states_topic", "/fleet_states");
  declare_parameter<std::string>("robot_zone_status_topic", "/robot_zone_status");
  declare_parameter<std::string>("zone_catalog_topic", "/zone_catalog");
  declare_parameter<std::string>("monitored_task_topic", "/Monitored_task_details");
  declare_parameter<std::string>("conflict_topic", "/zone_conflicts");
  declare_parameter<std::string>("replan_candidate_topic", "/replan_candidate");
  declare_parameter<double>("process_period_sec", 0.04);  // 25 Hz
  declare_parameter<double>("robot_stale_timeout_sec", 5.0);
  declare_parameter<double>("replan_plan_timeout_sec", 3.0);
  declare_parameter<std::string>("level", "L1");
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ZoneManagerNode::on_configure(const rclcpp_lifecycle::State &)
{
  process_period_sec_ = std::max(get_parameter("process_period_sec").as_double(), 0.01);
  stale_timeout_sec_ = std::max(get_parameter("robot_stale_timeout_sec").as_double(), 0.0);
  replan_plan_timeout_sec_ =
    std::max(get_parameter("replan_plan_timeout_sec").as_double(), 0.0);
  level_ = get_parameter("level").as_string();

  const auto conflict_topic = get_parameter("conflict_topic").as_string();
  const auto replan_topic = get_parameter("replan_candidate_topic").as_string();
  conflict_pub_ = create_publisher<zone_manager::msg::Conflict>(
    conflict_topic, rclcpp::QoS(rclcpp::KeepLast(200)).reliable());
  replan_candidate_pub_ = create_publisher<zone_manager::msg::ReplanCandidate>(
    replan_topic, rclcpp::QoS(rclcpp::KeepLast(50)).reliable());

  auto latched = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();

  catalog_sub_ = create_subscription<monitor::msg::ZoneCatalog>(
    get_parameter("zone_catalog_topic").as_string(),
    latched,
    std::bind(&ZoneManagerNode::on_zone_catalog, this, std::placeholders::_1));

  fleet_sub_ = create_subscription<rmf_fleet_msgs::msg::FleetState>(
    get_parameter("fleet_states_topic").as_string(),
    rclcpp::QoS(rclcpp::KeepLast(50)),
    std::bind(&ZoneManagerNode::on_fleet_state, this, std::placeholders::_1));

  zone_status_sub_ = create_subscription<monitor::msg::RobotZoneStatus>(
    get_parameter("robot_zone_status_topic").as_string(),
    rclcpp::QoS(rclcpp::KeepLast(200)),
    std::bind(&ZoneManagerNode::on_robot_zone_status, this, std::placeholders::_1));

  task_sub_ = create_subscription<monitor::msg::MonitoredTaskDetails>(
    get_parameter("monitored_task_topic").as_string(),
    rclcpp::QoS(rclcpp::KeepLast(100)),
    std::bind(&ZoneManagerNode::on_monitored_task, this, std::placeholders::_1));

  RCLCPP_INFO(
    get_logger(),
    "zone_manager configured (period=%.0f ms, replan_plan_timeout=%.1f s, "
    "conflict_topic=%s, replan_topic=%s)",
    process_period_sec_ * 1000.0,
    replan_plan_timeout_sec_,
    conflict_topic.c_str(),
    replan_topic.c_str());
  return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ZoneManagerNode::on_activate(const rclcpp_lifecycle::State &)
{
  conflict_pub_->on_activate();
  replan_candidate_pub_->on_activate();
  timer_ = create_wall_timer(
    std::chrono::duration<double>(process_period_sec_),
    std::bind(&ZoneManagerNode::on_timer, this));
  RCLCPP_INFO(get_logger(), "zone_manager activated");
  return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ZoneManagerNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  timer_.reset();
  conflict_pub_->on_deactivate();
  replan_candidate_pub_->on_deactivate();
  RCLCPP_INFO(get_logger(), "zone_manager deactivated");
  return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ZoneManagerNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  timer_.reset();
  fleet_sub_.reset();
  zone_status_sub_.reset();
  catalog_sub_.reset();
  task_sub_.reset();
  conflict_pub_.reset();
  replan_candidate_pub_.reset();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    robots_by_conflict_zone_.clear();
    pause_refcount_by_robot_.clear();
    last_zone_update_.clear();
    robot_info_.clear();
    pending_replan_by_candidate_.clear();
  }
  RCLCPP_INFO(get_logger(), "zone_manager cleaned up");
  return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ZoneManagerNode::on_shutdown(const rclcpp_lifecycle::State &)
{
  timer_.reset();
  return CallbackReturn::SUCCESS;
}

ZoneManagerNode::RobotRuntimeInfo & ZoneManagerNode::ensure_robot_info(
  const std::string & robot_id)
{
  auto & info = robot_info_[robot_id];
  if (!info.has_first_request) {
    info.first_request_time = std::chrono::steady_clock::now();
    info.has_first_request = true;
  }
  return info;
}

void ZoneManagerNode::on_zone_catalog(const monitor::msg::ZoneCatalog::SharedPtr msg)
{
  if (!msg) {
    return;
  }
  std::vector<std::string> ids(msg->zone_ids.begin(), msg->zone_ids.end());
  std::lock_guard<std::mutex> lock(mutex_);
  world_.register_zones(ids);
  RCLCPP_INFO(
    get_logger(),
    "WorldModel registered %zu zones from /zone_catalog (no YAML reload)",
    ids.size());
}

void ZoneManagerNode::on_fleet_state(const rmf_fleet_msgs::msg::FleetState::SharedPtr msg)
{
  if (!msg) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto & robot : msg->robots) {
    if (robot.name.empty()) {
      continue;
    }
    if (!robot.location.level_name.empty() && robot.location.level_name != level_) {
      continue;
    }
    world_.update_pose(robot.name, robot.location.x, robot.location.y);
    auto & info = ensure_robot_info(robot.name);
    info.fleet_load_status = robot.load_status;
    info.fleet_mode = robot.mode.mode;
    info.mode_known = true;
    info.operating_mode = robot.operating_mode;
  }
}

void ZoneManagerNode::on_robot_zone_status(
  const monitor::msg::RobotZoneStatus::SharedPtr msg)
{
  if (!msg || msg->amr_id.empty()) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  last_zone_update_[msg->amr_id] = std::chrono::steady_clock::now();
  auto & info = ensure_robot_info(msg->amr_id);
  info.eta_minutes = parse_eta_minutes(msg->eta);

  if (msg->current_zone.empty() && msg->next_zone.empty()) {
    world_.clear_robot(msg->amr_id);
    return;
  }

  world_.update_zones(
    msg->amr_id,
    msg->current_zone,
    std::vector<std::string>(msg->next_zone.begin(), msg->next_zone.end()),
    std::vector<std::string>(msg->zone_sequence.begin(), msg->zone_sequence.end()));
}

PlanSnapshot ZoneManagerNode::snapshot_from_world(const std::string & robot_id) const
{
  PlanSnapshot snap;
  const auto * st = world_.robot(robot_id);
  if (!st) {
    return snap;
  }
  snap.current_zone = st->current_zone;
  snap.next_zones.reserve(st->next_count);
  for (std::size_t i = 0; i < st->next_count; ++i) {
    snap.next_zones.push_back(st->next_zones[i]);
  }
  snap.zone_sequence = st->zone_sequence;
  return snap;
}

PlanSnapshot ZoneManagerNode::snapshot_from_monitored(
  const monitor::msg::MonitoredTaskDetails & msg)
{
  PlanSnapshot snap;
  snap.current_zone = msg.current_zone;
  snap.next_zones.assign(msg.next_zone.begin(), msg.next_zone.end());
  snap.zone_sequence.assign(msg.zone_sequence.begin(), msg.zone_sequence.end());
  snap.planned_path.assign(msg.planned_path.begin(), msg.planned_path.end());
  return snap;
}

void ZoneManagerNode::on_monitored_task(
  const monitor::msg::MonitoredTaskDetails::SharedPtr msg)
{
  if (!msg || msg->amr_id.empty()) {
    return;
  }

  std::vector<PostReplanPublish> posts;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto & info = ensure_robot_info(msg->amr_id);
    info.task_priority = msg->priority;
    info.task_load_status = msg->load_status;

    const auto watch_it = pending_replan_by_candidate_.find(msg->amr_id);
    if (watch_it == pending_replan_by_candidate_.end() ||
      watch_it->second.empty())
    {
      return;
    }

    const PlanSnapshot incoming = snapshot_from_monitored(*msg);
    // Any watch for this candidate shares the arm-time snapshot for change detect.
    if (!is_new_plan_for_candidate(
        watch_it->second.front().snapshot_at_arm, incoming))
    {
      return;
    }

    // Refresh candidate zones so re-decision uses the new sequence immediately.
    if (!msg->current_zone.empty() || !msg->next_zone.empty() ||
      !msg->zone_sequence.empty())
    {
      world_.update_zones(
        msg->amr_id,
        msg->current_zone,
        std::vector<std::string>(msg->next_zone.begin(), msg->next_zone.end()),
        std::vector<std::string>(msg->zone_sequence.begin(), msg->zone_sequence.end()));
    }

    std::vector<PendingReplanWatch> watches = std::move(watch_it->second);
    pending_replan_by_candidate_.erase(watch_it);
    posts.reserve(watches.size());

    for (auto & watch : watches) {
      RCLCPP_INFO(
        get_logger(),
        "PostReplan: new plan for candidate=%s zone=%s; re-analyzing",
        watch.candidate_robot_id.c_str(),
        watch.zone_id.c_str());
      posts.push_back(handle_post_replan_reanalysis(std::move(watch)));
    }
  }

  for (const auto & post : posts) {
    flush_post_replan_publish(post);
  }
}

bool ZoneManagerNode::is_robot_held_for_conflict(const std::string & robot_id) const
{
  const auto it = pause_refcount_by_robot_.find(robot_id);
  return it != pause_refcount_by_robot_.end() && it->second > 0;
}

std::vector<std::string> ZoneManagerNode::note_conflict_detected(
  const std::string & zone_id,
  const std::vector<std::string> & robot_ids)
{
  std::unordered_set<std::string> zone_set;
  std::vector<std::string> newly_held;
  newly_held.reserve(robot_ids.size());

  for (const auto & robot_id : robot_ids) {
    if (robot_id.empty()) {
      continue;
    }
    zone_set.insert(robot_id);
    const int prev = pause_refcount_by_robot_[robot_id];
    pause_refcount_by_robot_[robot_id] = prev + 1;
    if (prev == 0) {
      newly_held.push_back(robot_id);
    }
  }
  if (!zone_set.empty()) {
    robots_by_conflict_zone_[zone_id] = std::move(zone_set);
  }
  return newly_held;
}

void ZoneManagerNode::ensure_robots_held_for_zone(
  const std::string & zone_id,
  const std::vector<std::string> & robot_ids)
{
  auto & zone_set = robots_by_conflict_zone_[zone_id];
  for (const auto & robot_id : robot_ids) {
    if (robot_id.empty()) {
      continue;
    }
    if (zone_set.count(robot_id) > 0) {
      continue;  // already counted for this zone — no double-bump
    }
    zone_set.insert(robot_id);
    pause_refcount_by_robot_[robot_id] += 1;
  }
}

std::vector<std::string> ZoneManagerNode::release_robots_from_zone(
  const std::string & zone_id,
  const std::vector<std::string> & robot_ids)
{
  std::vector<std::string> released;
  auto zone_it = robots_by_conflict_zone_.find(zone_id);
  if (zone_it == robots_by_conflict_zone_.end()) {
    return released;
  }

  for (const auto & robot_id : robot_ids) {
    if (robot_id.empty() || zone_it->second.count(robot_id) == 0) {
      continue;
    }
    zone_it->second.erase(robot_id);
    auto ref_it = pause_refcount_by_robot_.find(robot_id);
    if (ref_it == pause_refcount_by_robot_.end()) {
      continue;
    }
    const int next = ref_it->second - 1;
    if (next <= 0) {
      pause_refcount_by_robot_.erase(ref_it);
      released.push_back(robot_id);
    } else {
      ref_it->second = next;
    }
  }
  if (zone_it->second.empty()) {
    robots_by_conflict_zone_.erase(zone_it);
  }
  return released;
}

std::vector<std::string> ZoneManagerNode::note_conflict_resolved(const std::string & zone_id)
{
  std::vector<std::string> released;
  const auto it = robots_by_conflict_zone_.find(zone_id);
  if (it == robots_by_conflict_zone_.end()) {
    return released;
  }

  for (const auto & robot_id : it->second) {
    auto ref_it = pause_refcount_by_robot_.find(robot_id);
    if (ref_it == pause_refcount_by_robot_.end()) {
      continue;
    }
    const int next = ref_it->second - 1;
    if (next <= 0) {
      pause_refcount_by_robot_.erase(ref_it);
      released.push_back(robot_id);
    } else {
      ref_it->second = next;
    }
  }
  robots_by_conflict_zone_.erase(it);
  return released;
}

ConflictDecision ZoneManagerNode::decide_for_zone(
  const std::string & zone_id,
  const std::vector<std::string> & robot_ids) const
{
  std::vector<ConflictRobotInput> inputs;
  inputs.reserve(robot_ids.size());

  for (const auto & robot_id : robot_ids) {
    if (robot_id.empty()) {
      continue;
    }
    ConflictRobotInput in;
    in.robot_id = robot_id;
    const auto * st = world_.robot(robot_id);
    if (st) {
      in.reserved_zones = st->reserved_zones();
      in.zone_sequence = st->zone_sequence;
    }
    const auto it = robot_info_.find(robot_id);
    if (it != robot_info_.end()) {
      in.fleet_mode = it->second.fleet_mode;
      in.mode_known = it->second.mode_known;
      in.operating_mode = it->second.operating_mode;
      in.priority.robot_id = robot_id;
      in.priority.task_priority = it->second.task_priority;
      in.priority.loaded = is_loaded_status(it->second.fleet_load_status);
      in.priority.eta_minutes = it->second.eta_minutes;
      in.priority.first_request_time = it->second.has_first_request ?
        it->second.first_request_time : std::chrono::steady_clock::now();
    } else {
      in.priority.robot_id = robot_id;
      in.priority.first_request_time = std::chrono::steady_clock::now();
    }
    inputs.push_back(std::move(in));
  }

  return decide_conflict_action(zone_id, inputs);
}

void ZoneManagerNode::arm_pending_replan_watch(
  const std::string & zone_id,
  const std::string & candidate_robot_id,
  const std::vector<std::string> & conflicting_robots)
{
  if (candidate_robot_id.empty() || zone_id.empty()) {
    return;
  }

  PendingReplanWatch watch;
  watch.zone_id = zone_id;
  watch.candidate_robot_id = candidate_robot_id;
  watch.conflicting_robots = conflicting_robots;
  watch.snapshot_at_arm = snapshot_from_world(candidate_robot_id);
  watch.armed_at = std::chrono::steady_clock::now();

  auto & watches = pending_replan_by_candidate_[candidate_robot_id];
  for (auto & existing : watches) {
    if (existing.zone_id != zone_id) {
      continue;
    }
    // Same zone: refresh conflicting set; keep earliest arm + snapshot.
    std::unordered_set<std::string> merged(
      existing.conflicting_robots.begin(), existing.conflicting_robots.end());
    for (const auto & id : conflicting_robots) {
      if (!id.empty()) {
        merged.insert(id);
      }
    }
    existing.conflicting_robots.assign(merged.begin(), merged.end());
    std::sort(
      existing.conflicting_robots.begin(), existing.conflicting_robots.end());
    RCLCPP_INFO(
      get_logger(),
      "PostReplan: refreshed watch candidate=%s zone=%s timeout=%.1f s",
      candidate_robot_id.c_str(),
      zone_id.c_str(),
      replan_plan_timeout_sec_);
    return;
  }

  watches.push_back(std::move(watch));
  RCLCPP_INFO(
    get_logger(),
    "PostReplan: armed watch candidate=%s zone=%s timeout=%.1f s",
    candidate_robot_id.c_str(),
    zone_id.c_str(),
    replan_plan_timeout_sec_);
}

void ZoneManagerNode::publish_replan_candidate(
  const std::string & zone_id,
  const ConflictDecision & decision,
  const std::vector<std::string> & conflicting_robots)
{
  if (!replan_candidate_pub_ || !replan_candidate_pub_->is_activated()) {
    return;
  }
  if (decision.replan_candidate_id.empty()) {
    return;
  }

  zone_manager::msg::ReplanCandidate msg;
  msg.stamp = now();
  msg.zone_id = zone_id;
  msg.robot_id = decision.replan_candidate_id;
  msg.conflicting_robots = conflicting_robots;
  msg.reason = decision.replan_reason;
  replan_candidate_pub_->publish(msg);

  {
    std::lock_guard<std::mutex> lock(mutex_);
    arm_pending_replan_watch(
      zone_id, decision.replan_candidate_id, conflicting_robots);
  }

  RCLCPP_WARN(
    get_logger(),
    "ReplanCandidate zone=%s robot=%s reason=%s conflicting=[%s]",
    zone_id.c_str(),
    decision.replan_candidate_id.c_str(),
    decision.replan_reason.c_str(),
    join_ids(conflicting_robots).c_str());
}

ZoneManagerNode::PostReplanPublish ZoneManagerNode::handle_post_replan_reanalysis(
  PendingReplanWatch watch)
{
  PostReplanPublish out;
  out.zone_id = watch.zone_id;

  // 1) Safety: ensure both original robots are held for this zone (no double-bump).
  ensure_robots_held_for_zone(watch.zone_id, watch.conflicting_robots);
  out.safety_start_pause = watch.conflicting_robots;

  // 2) Re-decide with latest sequences (no occupancy edge required).
  out.decision = decide_for_zone(watch.zone_id, watch.conflicting_robots);
  RCLCPP_WARN(
    get_logger(),
    "PostReplanDecision zone=%s %s",
    watch.zone_id.c_str(),
    out.decision.debug_summary.c_str());

  const bool opposite =
    out.decision.conflict_class == ConflictClass::StraightOpposite ||
    out.decision.conflict_class == ConflictClass::JunctionOpposite;
  out.still_opposite = opposite;

  if (opposite) {
    // Keep both paused; do not republish /replan_candidate (loop avoidance).
    RCLCPP_ERROR(
      get_logger(),
      "PostReplan: still opposite on zone=%s after new plan; keeping both paused "
      "(no second replan_candidate)",
      watch.zone_id.c_str());
    return out;
  }

  // 3) Branch / same / through: resume anyone not in the new pause set.
  const auto to_resume = robots_to_resume(
    watch.conflicting_robots, out.decision.pause_robot_ids);
  out.selective_stop_pause = release_robots_from_zone(watch.zone_id, to_resume);

  // Ensure decision pause targets remain held for this zone.
  if (!out.decision.pause_robot_ids.empty()) {
    ensure_robots_held_for_zone(watch.zone_id, out.decision.pause_robot_ids);
  }

  RCLCPP_INFO(
    get_logger(),
    "PostReplan: zone=%s resume=[%s] keep_paused=[%s]",
    watch.zone_id.c_str(),
    join_ids(out.selective_stop_pause).c_str(),
    join_ids(out.decision.pause_robot_ids).c_str());

  return out;
}

void ZoneManagerNode::publish_conflict_action(
  const std::string & zone_id,
  const std::string & action,
  const std::vector<std::string> & robot_ids)
{
  if (!conflict_pub_ || !conflict_pub_->is_activated() || robot_ids.empty()) {
    return;
  }
  zone_manager::msg::Conflict msg;
  msg.stamp = now();
  msg.type = static_cast<uint8_t>(
    action == kActionStartPause ?
    ConflictEventType::Detected : ConflictEventType::Resolved);
  msg.zone_id = zone_id;
  msg.action = action;
  msg.robot_ids = robot_ids;
  conflict_pub_->publish(msg);

  RCLCPP_INFO(
    get_logger(),
    "[Publish] PostReplan /zone_conflicts zone=%s action=%s robots=[%s]",
    zone_id.c_str(), action.c_str(), join_ids(robot_ids).c_str());
}

void ZoneManagerNode::flush_post_replan_publish(const PostReplanPublish & out)
{
  // Safety startPause first (adapter may treat as idempotent reaffirm).
  publish_conflict_action(out.zone_id, kActionStartPause, out.safety_start_pause);
  if (!out.selective_stop_pause.empty()) {
    publish_conflict_action(out.zone_id, kActionStopPause, out.selective_stop_pause);
  }
}

void ZoneManagerNode::check_pending_replan_timeouts(
  std::chrono::steady_clock::time_point now)
{
  if (pending_replan_by_candidate_.empty()) {
    return;
  }

  for (auto cand_it = pending_replan_by_candidate_.begin();
    cand_it != pending_replan_by_candidate_.end(); )
  {
    auto & watches = cand_it->second;
    watches.erase(
      std::remove_if(
        watches.begin(), watches.end(),
        [&](const PendingReplanWatch & watch) {
          if (!is_replan_watch_timed_out(watch, now, replan_plan_timeout_sec_)) {
            return false;
          }
          RCLCPP_ERROR(
            get_logger(),
            "PostReplan: timeout waiting for new plan candidate=%s zone=%s "
            "(%.1f s); leaving robots paused",
            watch.candidate_robot_id.c_str(),
            watch.zone_id.c_str(),
            replan_plan_timeout_sec_);
          return true;
        }),
      watches.end());

    if (watches.empty()) {
      cand_it = pending_replan_by_candidate_.erase(cand_it);
    } else {
      ++cand_it;
    }
  }
}

void ZoneManagerNode::publish_conflicts(const std::vector<ConflictEvent> & events)
{
  if (!conflict_pub_ || !conflict_pub_->is_activated()) {
    return;
  }

  for (const auto & ev : events) {
    zone_manager::msg::Conflict msg;
    msg.stamp = now();
    msg.type = static_cast<uint8_t>(ev.type);
    msg.zone_id = ev.zone_id;

    // Staged replan candidate for this event; published only after the
    // Conflict message has gone out (see the interlock note below).
    ConflictDecision staged_replan_decision;
    std::vector<std::string> staged_replan_conflicting;
    bool has_staged_replan = false;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (ev.type == ConflictEventType::Detected) {
        msg.action = kActionStartPause;

        const auto decision = decide_for_zone(ev.zone_id, ev.robot_ids);
        RCLCPP_WARN(
          get_logger(),
          "ConflictDecision zone=%s %s",
          ev.zone_id.c_str(), decision.debug_summary.c_str());

        if (decision.pause_robot_ids.empty()) {
          RCLCPP_WARN(
            get_logger(),
            "ConflictDetected zone=%s no pause target; skipping startPause",
            ev.zone_id.c_str());
          continue;
        }

        note_conflict_detected(ev.zone_id, decision.pause_robot_ids);
        msg.robot_ids = decision.pause_robot_ids;

        // ---- Publish-order interlock (part 1 of 2) -----------------------
        // The replan candidate MUST reach the execution adapter AFTER the
        // startPause it belongs to: the adapter only honours a candidate for a
        // robot it has already recorded as paused. Publishing it here, inside
        // the mutex block, put it on the wire ~0.45 ms BEFORE the Conflict
        // message and the adapter rejected every candidate with
        //   "robot is not currently paused by a zone conflict"
        // So only stage it here; it is published after conflict_pub_->publish()
        // below.
        const bool opposite =
          decision.conflict_class == ConflictClass::StraightOpposite ||
          decision.conflict_class == ConflictClass::JunctionOpposite;
        if (opposite) {
          staged_replan_decision = decision;
          staged_replan_conflicting = ev.robot_ids;
          has_staged_replan = true;
        }
      } else {
        msg.action = kActionStopPause;
        msg.robot_ids = note_conflict_resolved(ev.zone_id);
      }
    }

    if (msg.robot_ids.empty()) {
      if (ev.type == ConflictEventType::Resolved) {
        RCLCPP_INFO(
          get_logger(),
          "ConflictResolved zone=%s (no stopPause; none paused or still held)",
          ev.zone_id.c_str());
      }
      continue;
    }

    if (ev.type == ConflictEventType::Detected) {
      RCLCPP_INFO(
        get_logger(),
        "[Publish] Sending pause signal(s) on /zone_conflicts zone=%s action=%s robots=[%s]",
        ev.zone_id.c_str(), msg.action.c_str(), join_ids(msg.robot_ids).c_str());
      RCLCPP_WARN(
        get_logger(),
        "ConflictDetected zone=%s action=%s robots=[%s]",
        ev.zone_id.c_str(), msg.action.c_str(), join_ids(msg.robot_ids).c_str());
    } else {
      RCLCPP_INFO(
        get_logger(),
        "ConflictResolved zone=%s action=%s robots=[%s]",
        ev.zone_id.c_str(), msg.action.c_str(), join_ids(msg.robot_ids).c_str());
    }
    conflict_pub_->publish(msg);

    // ---- Publish-order interlock (part 2 of 2) ---------------------------
    // Conflict first, then the replan candidate that depends on it.
    if (has_staged_replan) {
      RCLCPP_INFO(
        get_logger(),
        "[Publish] Publishing replan candidate after pause for zone=%s selected=%s reason=%s",
        ev.zone_id.c_str(),
        staged_replan_decision.replan_candidate_id.c_str(),
        staged_replan_decision.replan_reason.c_str());
      publish_replan_candidate(
        ev.zone_id, staged_replan_decision, staged_replan_conflicting);
    }
  }
}

void ZoneManagerNode::on_timer()
{
  std::vector<ConflictEvent> events;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto now_tp = std::chrono::steady_clock::now();

    check_pending_replan_timeouts(now_tp);

    if (stale_timeout_sec_ > 0.0) {
      std::vector<std::string> stale;
      for (const auto & [robot_id, last] : last_zone_update_) {
        const double age = std::chrono::duration<double>(now_tp - last).count();
        if (age > stale_timeout_sec_) {
          stale.push_back(robot_id);
        }
      }
      for (const auto & robot_id : stale) {
        if (is_robot_held_for_conflict(robot_id)) {
          continue;
        }
        world_.clear_robot(robot_id);
        last_zone_update_.erase(robot_id);
      }
    }

    const auto changed = world_.sync_dirty_occupancy();
    events = detector_.evaluate(world_, changed);
  }

  publish_conflicts(events);
}

}  // namespace zone_manager
