#ifndef ECBS_PLANNER__TYPES_HPP_
#define ECBS_PLANNER__TYPES_HPP_

#include <cstdint>   // uint32_t, uint64_t
#include <optional>  // optional graph index
#include <string>
#include <vector>

namespace ecbs_planner
{

/// Map point the blocker must leave before the waiter proceeds.
struct ClearPoint
{
  double x{0.0};  // metres, map frame
  double y{0.0};  // metres, map frame
};

/// Where / how long to hold for one overlap (safe wait point).
struct PathWaitPoint
{
  double x{0.0};
  double y{0.0};
  std::optional<uint64_t> index;
  double wait_duration_sec{0.0};
};

/// Who we wait for, safe hold for this overlap, and cells they must clear.
struct WaitDependency
{
  std::string amr_id;                    // Blocker robot id
  PathWaitPoint wait_point;              // Safe waiting point for THIS overlap
  std::vector<ClearPoint> clear_points;  // Cells that must be vacated
  /// Geometry for TM (wait_reason.hpp kTopo*). Empty if unknown.
  std::string topology;
  bool self_blocks_other{false};         // easiness: self on peer's remaining
  bool other_blocks_self{false};         // easiness: peer on self's remaining
};

/// One timed waypoint on an AMR path (internal solve type, not ROS msg).
struct TimedPoint
{
  double t_sec{0.0};                     // Planned arrival time (seconds)
  double x{0.0};                         // Position x
  double y{0.0};                         // Position y
  double yaw{0.0};                       // Heading
  std::string level_name;                // Building level / map name
  std::optional<uint64_t> index;         // Optional nav-graph vertex id
  /// Immutable index on the task/revision route (survives pose-prefix erase).
  /// TM grant tips and conflict occurrences refer to this, not vector index.
  uint32_t route_occurrence{0};
  double wait_duration_sec{0.0};         // ECBS-inserted dwell at this wp
  std::vector<WaitDependency> waiting_for;  // Why we wait (who + clear pts)
};

/// Full timed path for one AMR (candidate or committed cache entry).
struct TimedPath
{
  std::string amr_id;            // Robot name
  std::string fleet_name;        // Fleet / group name
  std::string task_id;           // Current task id
  std::string dispatch_task_id;  // Dispatch / parent task id
  uint32_t phase_index{0};       // GoToPlace / path phase
  uint32_t path_revision{0};     // Bumps on genuine replan
  std::vector<TimedPoint> waypoints;  // Ordered space-time path
};

/// End of the base ECBS last answered FREE for (robot owns path up to here).
struct GrantedBase
{
  std::string task_id;            // Task the base was given for
  double x{0.0};                  // Base end, map frame
  double y{0.0};
  std::optional<uint64_t> index;  // Base end nav-graph index, if known
};

/// Earliest vertex conflict found between two TimedPaths.
/// Also stores the spatially overlapped corridor around that conflict so the
/// waiter can hold *before* the overlapped path (not only one step before
/// the time-conflict cell) — important for opposite / head-on traffic.
struct ConflictHit
{
  std::string other_amr;         // Conflicting AMR id
  std::size_t self_wp{0};        // Time-conflict waypoint on self path
  std::size_t other_wp{0};       // Time-conflict waypoint on other path
  double overlap_start_sec{0.0}; // When occupancy intervals first overlap
  double other_clear_sec{0.0};   // When other finishes clearing overlapped corridor
  double self_clear_sec{0.0};    // When self would finish clearing overlapped corridor

  // Contiguous shared cells around the conflict (inclusive indices).
  std::size_t self_overlap_lo{0};
  std::size_t self_overlap_hi{0};
  std::size_t other_overlap_lo{0};
  std::size_t other_overlap_hi{0};

  // Hold here: waypoint just before overlapped path start (or 0).
  std::size_t self_wait_wp{0};
  std::size_t other_wait_wp{0};

  bool opposite{false};  // true if paths traverse the shared cells in opposite order
};

/// Outcome of EcbsSolver::solve.
enum class SolveStatus
{
  Ok,            // Candidate OK (maybe with waits on itself)
  UpdatedOther,  // Had to delay an earlier/cached AMR (feasibility fallback)
  FailReplan     // Waiting cannot resolve within limits
};

/// Full solve output returned to EcbsGate / optional EcbsNode.
struct SolveResult
{
  SolveStatus status{SolveStatus::FailReplan};  // Overall result
  TimedPath path;                               // Resolved candidate path
  std::vector<TimedPath> updated_others;        // Cached AMRs we delayed
  std::string reason;                           // Human-readable status text
  std::vector<std::string> conflicting_amrs;    // AMRs involved in conflicts
};

}  // namespace ecbs_planner

#endif  // ECBS_PLANNER__TYPES_HPP_
