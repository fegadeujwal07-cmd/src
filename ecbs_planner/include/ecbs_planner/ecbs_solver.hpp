#ifndef ECBS_PLANNER__ECBS_SOLVER_HPP_
#define ECBS_PLANNER__ECBS_SOLVER_HPP_

#include <cmath>     // used by same_cell (hypot) in .cpp
#include <optional>
#include <string>
#include <vector>

#include "ecbs_planner/schedule_cache.hpp"
#include "ecbs_planner/types.hpp"

namespace ecbs_planner
{

/// Tunables for wait-only conflict resolution.
/// config/ecbs_params.yaml is the single source of truth for the running
/// system: evaluate_path_server reads it with load_solver_config_yaml(). The
/// member defaults below are only the fallback for keys missing from the YAML
/// and the fixed geometry the unit tests are written against.
struct SolverConfig
{
  double safety_margin_sec{2.0};   // Extra clearance after other clears cell
  double max_wait_sec{120.0};       // Cap on cumulative wait for one AMR
  int max_iters{20};               // Max resolve-loop iterations
  double position_epsilon_m{0.35}; // XY same-cell match if no graph index
  double safety_distance_m{2.5};   // Hold at least this far (along path) before overlap
  double evaluate_lookahead_margin_m{5.0};  // Evaluate checks clashes up to horizon + this
  /// Centre-to-centre distance at which two robots touch (robot width +
  /// margin): closer path points conflict, wait points keep this far from
  /// the other path, reserved stretches and standing robots.
  double clearance_m{2.0};
  /// AMR physical body dimensions (metres) used for body-overlap check at
  /// the candidate wait point. The wait is placed at safety_distance_m
  /// before the conflict; if the AMR body would still overlap the blocker's
  /// body, it is stepped further back until bodies are clear.
  double amr_length_m{1.5};   // AMR body length (metres)
  double amr_width_m{1.5};    // AMR body width  (metres)
  /// Drop precautionary T/+ junction waits once the AMR is this close to the
  /// approach stop and no real conflict remains.
  double junction_release_m{8.0};
};

/// Read evaluate_path_server.ros__parameters from \p yaml_path (the format of
/// config/ecbs_params.yaml). Keys missing from the file keep their value from
/// \p fallback. Throws std::runtime_error if the file cannot be read or has no
/// evaluate_path_server.ros__parameters block.
SolverConfig load_solver_config_yaml(
  const std::string & yaml_path, const SolverConfig & fallback = SolverConfig{});

/// Wait-only ECBS (waiting_for on both AMRs; no FIFO):
/// - On conflict: attach waiting_for + clear_points on BOTH paths.
/// - Time-shift the candidate when it fits, then the cached AMR using the
///   candidate's updated clear time (not the pre-delay clear time).
/// - Re-scan every cached AMR after each pass. Commit once when none conflict.
/// - FailReplan if a conflict cannot fit in max_wait, or max_iters is hit.
/// - Wait dwell (when applied) is placed safety_distance_m before the
///   overlapped corridor (or at path start). A cached AMR whose given base
///   already covers the overlap start holds at its base end and is not
///   time-shifted (base owner priority).
class EcbsSolver
{
public:
  /// Construct with config (defaults = SolverConfig{}).
  explicit EcbsSolver(SolverConfig config = {});

  /// Resolve candidate vs cache by inserting waits only (no geometric replan).
  /// Updates cache on success (candidate + any delayed others); FailReplan else.
  SolveResult solve(const TimedPath & candidate, ScheduleCache & cache) const;

  const SolverConfig & config() const {return config_;}

  /// True if two TimedPoints occupy the same cell (index or XY+level).
  static bool same_cell(const TimedPoint & a, const TimedPoint & b, double eps_m);

  /// Earliest overlapping same-cell conflict between paths a and b,
  /// enriched with overlapped-corridor wait indices.
  static std::optional<ConflictHit> earliest_conflict(
    const TimedPath & a,
    const TimedPath & b,
    double eps_m);

  /// Clear points for blocker corridor [lo, hi] inclusive.
  static std::vector<ClearPoint> clear_points_from_range(
    const TimedPath & blocker,
    std::size_t lo,
    std::size_t hi);

private:
  SolverConfig config_;  // Stored margins / limits

  /// Occupancy end time at waypoint i (arrive+wait or next arrival).
  static double depart_sec(const TimedPath & path, std::size_t i);

  /// Expand contiguous shared cells around (i,j); set wait_wp before corridor.
  static void enrich_overlap_and_wait(
    const TimedPath & a,
    const TimedPath & b,
    ConflictHit & hit,
    double eps_m);

  /// Add wait at wp_index and shift later waypoint times (legacy helper).
  static void apply_wait(TimedPath & path, std::size_t wp_index, double delta_t);
};

/// Where a path runs into another AMR's reserved stretch.
struct ReservedHit
{
  std::size_t wait_i{0};            // Hold index (safety distance before it)
  std::size_t hit_i{0};             // First waypoint too close to the stretch
  std::vector<ClearPoint> cells;    // Reserved cells that must be left first
  /// Exact wait XY along the path (safety_m before first reach of a cell).
  /// When set, prefer this over waypoints[wait_i] so discrete spacing does
  /// not push the wait ~one cell further back (~5 m before a crossing).
  bool has_exact_wait{false};
  double wait_x{0.0};
  double wait_y{0.0};
};

/// \p path from \p from_i (robot) to \p to_i first comes within
/// \p footprint_m of \p reserved (cells already that close to the robot are
/// ignored: it is there already). Also detects body-touch (L×W) conflicts.
/// Wait is placed \p safety_m before the path first *reaches* a reserved
/// cell (not before the clearance edge or a nearest-wp snap). Then, only if
/// L×W bodies would still dash (\p amr_half_diag), the wait steps further back.
std::optional<ReservedHit> wait_before_reserved(
  const TimedPath & path,
  std::size_t from_i,
  std::size_t to_i,
  const std::vector<ClearPoint> & reserved,
  const TimedPath & owner_path,
  double footprint_m,
  double safety_m,
  double amr_half_diag = 0.0);

/// Wait before every other AMR's reserved stretch on \p path (no hold: the
/// wait ends by itself once the stretch is out of the way). Records those
/// AMRs as \p path's reserve waits; upserts \p path if a wait was added.
void attach_reserved_waits(
  TimedPath & path, ScheduleCache & cache, const SolverConfig & config);

}  // namespace ecbs_planner

#endif  // ECBS_PLANNER__ECBS_SOLVER_HPP_
