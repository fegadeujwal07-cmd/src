#ifndef ECBS_PLANNER__PROPOSAL_ENGINE_HPP_
#define ECBS_PLANNER__PROPOSAL_ENGINE_HPP_

#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "ecbs_planner/authority_ledger.hpp"
#include "ecbs_planner/conflict_group_builder.hpp"
#include "ecbs_planner/conflict_store.hpp"
#include "ecbs_planner/ecbs_solver.hpp"
#include "ecbs_planner/group_passage_scheduler.hpp"
#include "ecbs_planner/passage_arbitration.hpp"
#include "ecbs_planner/route_occurrence.hpp"
#include "ecbs_planner/schedule_cache.hpp"
#include "ecbs_planner/types.hpp"

namespace ecbs_planner
{

struct CommittedGrantView
{
  std::string amr_id;
  uint64_t grant_id{0};
  uint64_t proposal_id{0};
  int32_t release_through_index{-1};
  RouteOccurrence from_occurrence;
  RouteOccurrence to_occurrence;
  uint32_t path_revision{0};
  std::string task_id;
  std::string execution_order_id;
  std::string action_id;
  std::vector<std::string> required_conflict_ids;
  uint64_t commitment_version{0};
  /// Watermarks at commit time — AUTHORITY_CONFLICT only for newer sources.
  uint64_t committed_at_route_version{0};
  uint64_t committed_at_occupancy_version{0};
};

/// Route identity of one robot at proposal time: path_revision plus
/// (route_occurrence, xy/level hash) per waypoint. Compared by occurrence so a
/// rolling prefix erase (robot moving) does not look like a route change.
struct RouteStamp
{
  uint32_t path_revision{0};
  std::vector<std::pair<uint32_t, std::size_t>> points;
};

RouteStamp make_route_stamp(const TimedPath & path);
bool route_stamp_matches(const RouteStamp & stamp, const TimedPath & path);

struct PendingProposal
{
  uint64_t proposal_id{0};
  uint64_t decision_id{0};
  std::string amr_id;
  /// Canonical route/task identity (ScheduleCache / planner).
  std::string task_id;
  /// PathRequest / execution order id — not used for route match.
  std::string execution_order_id;
  std::string action_id;
  /// Informational only: groups merge/split. apply_commit resolves the group
  /// from amr_id, never from this field.
  uint64_t group_id{0};
  uint32_t path_revision{0};
  int32_t from_index{0};
  int32_t to_index{0};
  RouteOccurrence from_occurrence;
  RouteOccurrence to_occurrence;
  std::vector<std::string> required_conflict_ids;
  uint64_t route_version{0};
  uint64_t occupancy_version{0};
  uint64_t commitment_version{0};
  uint64_t group_structure_version{0};
  uint64_t route_geometry_version{0};
  uint64_t authority_version{0};
  uint64_t occupancy_clearance_version{0};
  uint64_t policy_priority_version{0};

  /// The exact action approved (fairness / compatibility at commit).
  PassageAction action;
  /// Stale scope (global route/occupancy counters are for echo/logging only):
  /// the proposal is stale only if one of these changed, or the robot has an
  /// unresolved AUTHORITY_CONFLICT.
  std::unordered_map<std::string, uint64_t> conflict_versions;
  std::unordered_map<std::string, RouteStamp> route_stamps;
  uint64_t group_authority_stamp{0};
};

struct ProposalEngineState
{
  ConflictStore conflicts;
  AuthorityLedger ledger;
  PassageGroupRegistry groups;
  GroupPassageScheduler scheduler;
  std::unordered_map<std::string, RobotPriorityFacts> priorities;
  std::unordered_map<std::string, CommittedGrantView> commitments;
  std::unordered_set<std::string> active_go;
  std::unordered_map<uint64_t, PendingProposal> pending_proposals;
  std::unordered_map<uint64_t, CommittedGrantView> grants_by_id;
  std::unordered_map<std::string, uint64_t> latest_proposal_by_amr;
  std::unordered_map<std::string, std::string> current_task_by_robot;
  std::unordered_map<std::string, std::size_t> last_path_content_hash;
  /// Retained until clearance / retire / explicit clear_authority_conflict.
  std::unordered_map<std::string, std::string> unresolved_authority;
  /// Event-driven wake: robots with retained extension demand.
  std::unordered_set<std::string> pending_demand;
  uint64_t next_proposal_id{1};
  uint64_t next_decision_id{1};
  uint64_t route_version{0};
  uint64_t occupancy_version{0};
  uint64_t commitment_version{0};
  uint64_t priority_version{0};
  int scheduler_max_nodes{256};
  int max_bypass_count{3};
};

/// Wake scheduling for AMRs affected by clearance / commit / occupancy / priority.
std::vector<std::string> wake_pending_demand(
  ProposalEngineState & state,
  const std::vector<std::string> & affected_amrs);

struct ProposalQueryInput
{
  uint64_t request_id{0};
  std::string amr_id;
  std::string fleet_name;
  /// Hint for canonical task; empty → use ScheduleCache path.task_id.
  std::string task_id;
  std::string execution_order_id;
  int32_t phase_index{0};
  uint32_t path_revision{0};
  std::string route_hash;
  int32_t current_grant_through_index{-1};
  RouteOccurrence observed_progress;
  double current_x{0.0};
  double current_y{0.0};
  double desired_extension_m{20.0};
  std::string distance_origin;
  uint64_t route_version{0};
  uint64_t occupancy_version{0};
  uint64_t commitment_version{0};
  uint64_t priority_version{0};
};

struct ProposalQueryResult
{
  bool success{false};
  std::string message;
  std::string status;
  uint64_t proposal_id{0};
  uint64_t decision_id{0};
  double proposed_horizon_m{0.0};
  RouteOccurrence from_occurrence;
  RouteOccurrence to_occurrence;
  int32_t release_through_index{-1};
  std::vector<WaitDependency> waiting_for;
  std::string wait_reason;
  /// WAIT_CLEARANCE | WAIT_SEARCH | WAIT_HARDHOLD when waiting.
  std::string wait_reason_code;
  std::vector<ConflictRecord> relevant_conflicts;
  std::vector<std::string> required_conflict_ids;
  std::string decision_reason;
  bool authority_conflict{false};
  std::string authority_conflict_id;
  uint64_t grant_id_hit{0};
  PassageDecision decision;
  /// Canonical route task identity returned with the proposal.
  std::string task_id;
  std::string execution_order_id;
  std::string action_id;
  uint64_t group_id{0};
  uint64_t group_structure_version{0};
  uint64_t route_geometry_version{0};
  uint64_t authority_version{0};
  uint64_t occupancy_clearance_version{0};
  uint64_t policy_priority_version{0};
};

/// \p detect_distance_m should be SolverConfig::clearance_m.
std::vector<std::string> refresh_conflicts_from_cache(
  ProposalEngineState & state,
  const ScheduleCache & cache,
  double detect_distance_m,
  double position_epsilon_m,
  double safety_distance_m = SolverConfig{}.safety_distance_m);

ProposalQueryResult query_base_extension(
  ProposalEngineState & state,
  const ScheduleCache & cache,
  const ProposalQueryInput & in,
  double detect_distance_m,
  double position_epsilon_m,
  double safety_distance_m = SolverConfig{}.safety_distance_m);

/// Idempotent commit. Optionally updates ScheduleCache granted base.
std::string apply_commit(
  ProposalEngineState & state,
  uint64_t proposal_id,
  uint64_t grant_id,
  const std::string & amr_id,
  const std::string & status,
  int32_t release_through_index,
  const RouteOccurrence & from_occ,
  const RouteOccurrence & to_occ,
  uint32_t path_revision,
  const std::string & task_id,
  ScheduleCache * cache = nullptr);

/// Clear only if \p reporter_amr participates in \p conflict_id.
bool apply_clearance(
  ProposalEngineState & state,
  const std::string & reporter_amr,
  const std::string & conflict_id,
  bool physically_cleared);

/// Retire a grant. Clears current authority only when \p grant_id is current.
bool retire_grant(
  ProposalEngineState & state,
  const std::string & amr_id,
  uint64_t grant_id,
  ScheduleCache * cache = nullptr);

/// Explicit reconcile after stop protocol resolves an AUTHORITY_CONFLICT.
bool clear_authority_conflict(
  ProposalEngineState & state,
  const std::string & amr_id);

}  // namespace ecbs_planner

#endif  // ECBS_PLANNER__PROPOSAL_ENGINE_HPP_
