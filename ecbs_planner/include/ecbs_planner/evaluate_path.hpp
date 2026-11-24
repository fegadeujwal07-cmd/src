#ifndef ECBS_PLANNER__EVALUATE_PATH_HPP_
#define ECBS_PLANNER__EVALUATE_PATH_HPP_

#include <vector>

#include <ecbs_planner/conflict_store.hpp>
#include <ecbs_planner/ecbs_solver.hpp>
#include <ecbs_planner/nav_graph_junctions.hpp>
#include <ecbs_planner/schedule_cache.hpp>
#include <ecbs_planner/msg/from_zone_manager.hpp>
#include <ecbs_planner/msg/to_zone_manager.hpp>
#include <ecbs_planner/srv/evaluate_path.hpp>

namespace ecbs_planner
{

/// Overwrite waypoint t_sec with monitor::path_point_reach_times_sec
/// (eta_path_calculator). Keeps waits/deps. Returns motion ETA (s).
/// t_sec[i] = t0 + reach[i] where t0 is the previous front time (or 0).
double stamp_timed_path_with_eta(
  TimedPath & path, double start_speed_mps = 0.0);

/// Convert service request waypoints into an internal TimedPath.
/// Motion times come from stamp_timed_path_with_eta (not RMF planner times).
TimedPath timed_path_from_evaluate_request(
  const srv::EvaluatePath::Request & req);

/// TimedPath → lean ToZoneManager message (waits + waiting_for included).
msg::ToZoneManager to_zone_manager_msg(const TimedPath & path);

/// TimedPath → FromZoneManager path seed (/from_zone_manager).
msg::FromZoneManager from_zone_manager_msg(const TimedPath & path);

/// Dispatch EvaluatePath:
///   mode "evaluate"  → no solve; erase behind pose; lean FREE/WAIT
///                      FREE = no register; WAIT = ZM must register decision
///                      The robot's cached path gets its waits replaced by
///                      this answer (saved + put in \p committed_out), so
///                      /to_zone_manager matches the response. Not cached →
///                      WAIT is published view-only.
///   mode "register"  → process #2 after WAIT: updates[] only (waiters;
///                      GO implied). Exact ZM minimum payload — no waypoints.
///
/// On register and evaluate, \p committed_out gets paths to publish
/// on /to_zone_manager.
void evaluate_path(
  const srv::EvaluatePath::Request & req,
  srv::EvaluatePath::Response & res,
  ScheduleCache & cache,
  const EcbsSolver & solver,
  std::vector<TimedPath> * committed_out = nullptr,
  const NavGraphJunctions * junctions = nullptr,
  /// When false, FREE does not call set_granted_base — TM CommitProposal owns
  /// committed authority (avoids dual grant representations).
  bool record_granted_base = true,
  /// register mode: when set, every wait is validated against the active
  /// ConflictRecords (partner, before entry, not behind robot / on T/+ centre)
  /// and the whole batch is rejected, cache untouched, on the first failure.
  const ConflictStore * conflicts = nullptr);

/// After a successful register: mark each (waiter, blocker) pair mandatory on
/// the overlap its wait belongs to. T/+ sentinels are skipped. Returns the
/// number of records changed.
std::size_t persist_register_dependencies(
  const srv::EvaluatePath::Request & req,
  const ScheduleCache & cache,
  ConflictStore & conflicts);

/// Seed full path into ScheduleCache (/from_zone_manager): runs ECBS solve
/// against the shared cache so waiting points/times exist at planning time
/// (for replan wait-vs-detour compare). Not the ZM wait-decision register
/// (that is mode=register + updates[]).
void register_from_zone_manager(
  const msg::FromZoneManager & msg,
  srv::EvaluatePath::Response & res,
  ScheduleCache & cache,
  const EcbsSolver & solver,
  std::vector<TimedPath> * committed_out = nullptr);

}  // namespace ecbs_planner

#endif  // ECBS_PLANNER__EVALUATE_PATH_HPP_
