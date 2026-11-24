#ifndef TRAFFIC_MANAGER__PATH_GATE_HPP_
#define TRAFFIC_MANAGER__PATH_GATE_HPP_

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

#include "traffic_manager/msg/path_intent.hpp"

namespace traffic_manager
{

enum class IntentAcceptResult
{
  Invalid,
  Duplicate,
  /// Same route geometry; schedule (t/yaw) changed — grants preserved.
  ScheduleOnlyUpdate,
  NewOrUpdatedSession,
};

/// Per-robot path release session until leg completes or a new plan replaces it.
struct PathReleaseSession
{
  traffic_manager::msg::PathIntent intent;
  /// Combined hash used for Duplicate vs NewOrUpdatedSession (route + schedule).
  std::size_t content_hash{0};
  /// Immutable route geometry identity (XY + level); excludes cursor/progress.
  std::size_t route_identity_hash{0};
  /// Schedule revision (waypoint t, yaw, level) for the current route.
  std::size_t schedule_revision_hash{0};
  /// Authority issued / published tip (committed grant). Monotonic until stop/cancel.
  int32_t released_through_index{-1};
  /// Alias semantics: committed_grant == released_through_index.
  int32_t committed_grant() const { return released_through_index; }
  /// Last tip the robot acknowledged (NBR falling edge / matching update id).
  int32_t acknowledged_grant{-1};
  /// Bounded pose→leg progress (not interchangeable with committed_grant).
  int32_t observed_progress_index{-1};
  /// Outstanding EvaluatePath transaction; only matching response may clear in-flight.
  uint64_t pending_evaluation_id{0};
  uint64_t next_evaluation_id{1};
  /// Latched extension request until fulfilled, cancelled, or superseded.
  bool pending_extension{false};
  double telemetry_age_sec{0.0};
  double last_telemetry_sec{0.0};
  bool ecbs_eval_in_flight{false};
  double last_eval_x{0.0};
  double last_eval_y{0.0};
  /// Next PathRelease.release_update_id to publish (0, 1, 2, …).
  uint32_t next_release_update_id{0};
  /// Last release_update_id sent to full_control (for robot ack correlation).
  uint32_t last_published_release_update_id{0};
  /// ECBS WAIT with dependency: release capped at hold_through_index; no
  /// extension on base requests for robot↔robot / occupancy losers.
  /// Junction-only holds still accept newBaseRequest and may advance the
  /// freeze when a nearer T/+ gate clears. Re-evaluated every
  /// held_reevaluate_sec; cleared on FREE (or a new PathIntent session).
  bool held_at_wait{false};
  int32_t hold_through_index{-1};
  /// Wait geometry from /to_zone_manager (direct viz — not PathRelease-only).
  bool to_zone_has_wait{false};
  int32_t to_zone_wait_index{-1};
  double to_zone_wait_x{0.0};
  double to_zone_wait_y{0.0};
  std::vector<std::string> to_zone_blockers;
  /// True when the active hold is precautionary T/+ only (no robot deps).
  /// Allows newBaseRequest → evaluate and forward hold advance.
  bool junction_only_hold{false};
  /// Committed WAIT hold (leg + xy). Robot↔robot / occupancy: never advances.
  /// Junction-only: may advance when ECBS wait_cap moves further along the leg.
  bool hold_frozen{false};
  int32_t frozen_hold_leg{-1};
  double frozen_hold_x{0.0};
  double frozen_hold_y{0.0};
  /// Last WAIT hold sent on PathRelease. Identical re-evals must not bump
  /// release_update_id (that looks like a replan to execution/full_control).
  bool last_hold_published{false};
  int32_t last_pub_hold_through{-1};
  int32_t last_pub_hold_leg{-1};
  double last_pub_hold_x{0.0};
  double last_pub_hold_y{0.0};
  std::vector<std::string> last_pub_blocking_amrs;
  /// Node clock seconds of the last evaluate sent for this session.
  double last_ecbs_request_sec{0.0};
  /// Waiting for robot new_base_request=0 after last order update.
  bool awaiting_robot_ack{false};
  /// Last seen new_base_request from robot (edge detect).
  bool last_new_base_request{false};
  /// First ECBS eval deferred until /to_zone_manager or timer (Phase 2).
  bool pending_initial_ecbs{true};
  /// Nearest leg index from last fleet pose (rolling horizon cursor).
  int32_t leg_cursor_index{-1};
  /// Last conflict session version we force-evaluated peers for (debounce).
  uint64_t last_peer_force_session_version{0};
  /// After first evaluate post-PathIntent, force-eval peers once so they see
  /// this path in ScheduleCache (mode=clear wiped the from_zone seed).
  bool peers_force_eval_after_seed{false};
  /// Stalled GO: retain occupancy; do not resolve as free.
  bool stalled{false};
  /// Explicit fault / recovery reason when retries exhaust.
  std::string fault_reason;
  /// Proposal/commit txn: only matching ids may clear pending state.
  uint64_t pending_proposal_id{0};
  uint64_t committed_grant_id{0};
  uint64_t last_acked_grant_id{0};
  /// Per-overlap clearance bookkeeping (participant + exit + grant).
  struct TrackedOverlapClearance
  {
    std::string conflict_id;
    uint64_t grant_id{0};
    uint32_t path_revision{0};
    std::string peer_amr;
    double entry_x{0.0};
    double entry_y{0.0};
    double exit_x{0.0};
    double exit_y{0.0};
    uint32_t exit_occurrence{0};
    /// This robot's rear/load must clear the exit (blocker obligation).
    bool as_blocker{false};
    bool cleared{false};
  };
  std::vector<TrackedOverlapClearance> tracked_overlaps;
  /// Proposal versions at FREE_PROPOSAL (revalidated on commit).
  uint64_t proposal_route_version{0};
  uint64_t proposal_occupancy_version{0};
  uint64_t proposal_commitment_version{0};
  /// Commit-then-publish: tip advanced only after ECBS ACCEPTED.
  uint64_t pending_commit_grant_id{0};
  int32_t pending_commit_release_index{-1};
  /// Exact identity from FREE_PROPOSAL — commit these, do not reconstruct.
  std::string pending_canonical_task_id;
  std::string pending_execution_order_id;
  std::string pending_action_id;
  uint32_t pending_path_revision{0};
  double pending_from_x{0.0};
  double pending_from_y{0.0};
  uint32_t pending_from_occurrence{0};
  double pending_to_x{0.0};
  double pending_to_y{0.0};
  uint32_t pending_to_occurrence{0};
  /// Authority txn: Proposed→LocallyReserved→CommitPending→…→Cleared
  enum class AuthorityTxnPhase
  {
    None = 0,
    Proposed,
    LocallyReserved,
    CommitPending,
    Committed,
    Published,
    Acknowledged,
    Executing,
    Cleared,
  };
  AuthorityTxnPhase authority_txn_phase{AuthorityTxnPhase::None};
  /// AUTHORITY_CONFLICT stop/reconcile state machine.
  enum class AuthPhase
  {
    None = 0,
    StopRequested,
    StopAccepted,
    StopConfirmed,
    AuthorityReconciled,
  };
  AuthPhase auth_phase{AuthPhase::None};
  bool authority_conflict_active{false};
  std::string authority_conflict_id;
  double authority_stop_confirm_since_sec{-1.0};
  /// Destination occupancy retained after task leg completes.
  bool destination_occupied{false};
  bool task_leg_complete{false};
  /// Staged replacement route while reconciling outstanding grants.
  bool has_staged_intent{false};
  traffic_manager::msg::PathIntent staged_intent;
  std::size_t staged_route_identity_hash{0};
  std::size_t staged_schedule_revision_hash{0};

  /// At most one STALE_ROUTE reseed txn per robot. Tip/grant/pending_extension
  /// are preserved for the whole txn — recovery must not require a new task.
  bool route_recovery_active{false};
  bool route_recovery_seed_in_flight{false};
  uint64_t route_recovery_txn_id{0};
  std::size_t route_recovery_session_hash{0};
  uint64_t route_recovery_request_id{0};
  std::string route_recovery_canonical_task;
  std::string route_recovery_execution_order;
  uint32_t route_recovery_path_revision{0};
  int32_t route_recovery_preserved_tip{-1};
  uint64_t route_recovery_preserved_grant{0};
  bool route_recovery_had_pending_extension{true};
  double route_recovery_started_sec{0.0};
  double route_recovery_next_retry_sec{0.0};
  int route_recovery_attempt{0};
};

/// Stores full-leg intents and session state (ECBS decides release prefix).
class PathGate
{
public:
  IntentAcceptResult accept_intent(const traffic_manager::msg::PathIntent & intent);

  PathReleaseSession * session(const std::string & robot_id);
  const PathReleaseSession * session(const std::string & robot_id) const;

  void set_released_through(const std::string & robot_id, int32_t index);
  void set_ecbs_in_flight(const std::string & robot_id, bool in_flight);
  void mark_eval_pose(const std::string & robot_id, double x, double y);

  /// Acknowledge a published release only when order_update_id matches.
  bool acknowledge_release(
    const std::string & robot_id, uint32_t order_update_id);
  /// Latch extension until fulfilled, cancelled, or superseded.
  void latch_pending_extension(const std::string & robot_id);
  void clear_pending_extension(const std::string & robot_id);
  /// Activate staged replacement after grant reconciliation (or immediately
  /// when no committed grant exists).
  bool activate_staged_intent(const std::string & robot_id);

  void clear_session(const std::string & robot_id);

  std::vector<std::string> active_robot_ids() const;

  std::size_t robot_count() const { return by_robot_.size(); }

  static std::size_t hash_route_identity(const traffic_manager::msg::PathIntent & intent);
  static std::size_t hash_schedule_revision(const traffic_manager::msg::PathIntent & intent);

private:
  /// Route + schedule only (excludes cursor / execution progress).
  static std::size_t hash_intent(const traffic_manager::msg::PathIntent & intent);

  std::unordered_map<std::string, PathReleaseSession> by_robot_;
};

}  // namespace traffic_manager

#endif  // TRAFFIC_MANAGER__PATH_GATE_HPP_
