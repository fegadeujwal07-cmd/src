#include "traffic_manager/path_gate.hpp"

#include <algorithm>
#include <functional>

namespace traffic_manager
{
namespace
{
std::size_t hash_combine(std::size_t seed, std::size_t value)
{
  return seed ^ (value + 0x9e3779b9 + (seed << 6) + (seed >> 2));
}
}  // namespace

std::size_t PathGate::hash_route_identity(const traffic_manager::msg::PathIntent & intent)
{
  std::size_t h = 0;
  h = hash_combine(h, std::hash<std::string>{}(intent.dispatch_task_id));
  h = hash_combine(h, intent.phase_index);
  h = hash_combine(h, intent.path_revision);
  h = hash_combine(h, intent.path.size());
  for (const auto & loc : intent.path) {
    h = hash_combine(h, std::hash<double>{}(loc.x));
    h = hash_combine(h, std::hash<double>{}(loc.y));
    h = hash_combine(h, std::hash<std::string>{}(loc.level_name));
  }
  return h;
}

std::size_t PathGate::hash_schedule_revision(const traffic_manager::msg::PathIntent & intent)
{
  std::size_t h = 0;
  for (const auto & loc : intent.path) {
    h = hash_combine(h, std::hash<int32_t>{}(loc.t.sec));
    h = hash_combine(h, std::hash<uint32_t>{}(loc.t.nanosec));
    h = hash_combine(h, std::hash<double>{}(loc.yaw));
    h = hash_combine(h, std::hash<std::string>{}(loc.level_name));
  }
  return h;
}

std::size_t PathGate::hash_intent(const traffic_manager::msg::PathIntent & intent)
{
  // Route identity + schedule revision — cursor/progress excluded so pose
  // updates never reset the session as a "new plan".
  return hash_combine(hash_route_identity(intent), hash_schedule_revision(intent));
}

IntentAcceptResult PathGate::accept_intent(const traffic_manager::msg::PathIntent & intent)
{
  if (intent.robot_name.empty()) {
    return IntentAcceptResult::Invalid;
  }

  if (intent.path.empty()) {
    // Cancellation/empty intent: preserve outstanding reservations until
    // reconciled — do not erase the session while a grant is live.
    const auto it = by_robot_.find(intent.robot_name);
    if (it != by_robot_.end() && it->second.released_through_index >= 0) {
      it->second.staged_intent = intent;
      it->second.staged_route_identity_hash = 0;
      it->second.staged_schedule_revision_hash = 0;
      it->second.has_staged_intent = true;
      it->second.intent.request_id = intent.request_id;
      return IntentAcceptResult::NewOrUpdatedSession;
    }
    clear_session(intent.robot_name);
    return IntentAcceptResult::NewOrUpdatedSession;
  }

  const std::size_t route_hash = hash_route_identity(intent);
  const std::size_t schedule_hash = hash_schedule_revision(intent);
  const std::size_t content_hash = hash_combine(route_hash, schedule_hash);
  const auto it = by_robot_.find(intent.robot_name);
  if (it != by_robot_.end() && it->second.content_hash == content_hash)
  {
    // Same route+schedule — resubmits must not reset ECBS defer / session.
    // Cursor / request_id alone are execution progress, not a new plan.
    if (it->second.intent.request_id != intent.request_id) {
      it->second.intent.request_id = intent.request_id;
    }
    it->second.intent.cursor_index = intent.cursor_index;
    return IntentAcceptResult::Duplicate;
  }

  // Same route geometry, schedule-only change: preserve committed authority.
  if (it != by_robot_.end() && it->second.route_identity_hash == route_hash) {
    it->second.intent = intent;
    it->second.schedule_revision_hash = schedule_hash;
    it->second.content_hash = content_hash;
    it->second.has_staged_intent = false;
    return IntentAcceptResult::ScheduleOnlyUpdate;
  }

  // New route while a grant is outstanding: stage replacement; do not erase
  // committed authority until activate_staged_intent after reconciliation.
  if (it != by_robot_.end() && it->second.released_through_index >= 0) {
    it->second.staged_intent = intent;
    it->second.staged_route_identity_hash = route_hash;
    it->second.staged_schedule_revision_hash = schedule_hash;
    it->second.has_staged_intent = true;
    it->second.intent.request_id = intent.request_id;
    return IntentAcceptResult::NewOrUpdatedSession;
  }

  PathReleaseSession session;
  session.intent = intent;
  session.content_hash = content_hash;
  session.route_identity_hash = route_hash;
  session.schedule_revision_hash = schedule_hash;
  session.released_through_index = -1;
  session.acknowledged_grant = -1;
  session.observed_progress_index = -1;
  session.pending_evaluation_id = 0;
  session.pending_extension = false;
  session.ecbs_eval_in_flight = false;
  session.pending_initial_ecbs = true;
  session.leg_cursor_index = -1;
  session.stalled = false;
  by_robot_[intent.robot_name] = std::move(session);
  return IntentAcceptResult::NewOrUpdatedSession;
}

PathReleaseSession * PathGate::session(const std::string & robot_id)
{
  const auto it = by_robot_.find(robot_id);
  if (it == by_robot_.end()) {
    return nullptr;
  }
  return &it->second;
}

const PathReleaseSession * PathGate::session(const std::string & robot_id) const
{
  const auto it = by_robot_.find(robot_id);
  if (it == by_robot_.end()) {
    return nullptr;
  }
  return &it->second;
}

void PathGate::set_released_through(const std::string & robot_id, const int32_t index)
{
  if (auto * s = session(robot_id)) {
    // Committed grant is monotonic: never retract without stop/cancel protocol.
    if (index > s->released_through_index) {
      s->released_through_index = index;
    }
  }
}

void PathGate::set_ecbs_in_flight(const std::string & robot_id, const bool in_flight)
{
  if (auto * s = session(robot_id)) {
    s->ecbs_eval_in_flight = in_flight;
    if (!in_flight) {
      s->pending_evaluation_id = 0;
    }
  }
}

void PathGate::mark_eval_pose(
  const std::string & robot_id, const double x, const double y)
{
  if (auto * s = session(robot_id)) {
    s->last_eval_x = x;
    s->last_eval_y = y;
  }
}

bool PathGate::acknowledge_release(
  const std::string & robot_id, const uint32_t order_update_id)
{
  auto * s = session(robot_id);
  if (!s || !s->awaiting_robot_ack) {
    return false;
  }
  // Matching published release clears pending ACK state.
  if (order_update_id != s->last_published_release_update_id) {
    // Same-tip WAIT once bumped release_update_id while execution kept the
    // prior PathRequest revision (orderUpdateId). Accept the prior slot and
    // roll the expected id back so the ACK chain can recover.
    const bool same_tip =
      s->released_through_index >= 0 &&
      s->acknowledged_grant == s->released_through_index;
    if (!(same_tip &&
        order_update_id + 1u == s->last_published_release_update_id))
    {
      return false;
    }
    s->last_published_release_update_id = order_update_id;
    // Never roll the NEXT id back: the same-tip update may still reach the
    // robot later (execution holds it until the robot is idle), and reusing
    // its id makes the following extension look like a duplicate that
    // execution suppresses — the robot then waits at its release boundary
    // for good (amr2, 2026-11-20: hold update 2 delivered late, final FREE
    // release re-sent as update 2 and dropped). Gaps are fine: execution only
    // needs a higher revision.
    s->next_release_update_id =
      std::max(s->next_release_update_id, order_update_id + 1u);
  }
  s->awaiting_robot_ack = false;
  s->acknowledged_grant = s->released_through_index;
  s->last_acked_grant_id = s->committed_grant_id;
  return true;
}

void PathGate::latch_pending_extension(const std::string & robot_id)
{
  if (auto * s = session(robot_id)) {
    s->pending_extension = true;
  }
}

void PathGate::clear_pending_extension(const std::string & robot_id)
{
  if (auto * s = session(robot_id)) {
    s->pending_extension = false;
  }
}

bool PathGate::activate_staged_intent(const std::string & robot_id)
{
  auto * s = session(robot_id);
  if (!s || !s->has_staged_intent) {
    return false;
  }
  const auto staged = s->staged_intent;
  const auto route_hash = s->staged_route_identity_hash;
  const auto schedule_hash = s->staged_schedule_revision_hash;
  // Activate replacement: reset release tip for the new route identity.
  s->intent = staged;
  s->route_identity_hash = route_hash;
  s->schedule_revision_hash = schedule_hash;
  s->content_hash = hash_combine(route_hash, schedule_hash);
  s->released_through_index = -1;
  s->acknowledged_grant = -1;
  s->observed_progress_index = -1;
  // The previous leg's cursor must not carry over: it made the new task look
  // already progressed (false Grant COMPLETED / cursor search from old index).
  s->leg_cursor_index = -1;
  s->intent.cursor_index = 0;
  s->pending_evaluation_id = 0;
  s->pending_extension = false;
  s->ecbs_eval_in_flight = false;
  s->pending_initial_ecbs = true;
  s->held_at_wait = false;
  s->hold_frozen = false;
  s->awaiting_robot_ack = false;
  s->has_staged_intent = false;
  s->pending_proposal_id = 0;
  s->committed_grant_id = 0;
  s->last_acked_grant_id = 0;
  s->tracked_overlaps.clear();
  s->proposal_route_version = 0;
  s->proposal_occupancy_version = 0;
  s->proposal_commitment_version = 0;
  s->pending_commit_grant_id = 0;
  s->pending_commit_release_index = -1;
  s->auth_phase = PathReleaseSession::AuthPhase::None;
  s->authority_conflict_active = false;
  s->authority_conflict_id.clear();
  s->authority_stop_confirm_since_sec = -1.0;
  s->destination_occupied = false;
  s->task_leg_complete = false;
  s->stalled = false;
  s->fault_reason.clear();
  s->route_recovery_active = false;
  s->route_recovery_seed_in_flight = false;
  s->route_recovery_txn_id = 0;
  s->route_recovery_attempt = 0;
  return true;
}

void PathGate::clear_session(const std::string & robot_id)
{
  by_robot_.erase(robot_id);
}

std::vector<std::string> PathGate::active_robot_ids() const
{
  std::vector<std::string> ids;
  ids.reserve(by_robot_.size());
  for (const auto & [robot_id, session] : by_robot_) {
    if (!session.intent.path.empty()) {
      ids.push_back(robot_id);
    }
  }
  std::sort(ids.begin(), ids.end());
  return ids;
}

}  // namespace traffic_manager
