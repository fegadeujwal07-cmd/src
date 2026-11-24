#ifndef ECBS_PLANNER__REPLAN_CANDIDATE_HPP_
#define ECBS_PLANNER__REPLAN_CANDIDATE_HPP_

#include <cstdint>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "ecbs_planner/nav_graph.hpp"
#include "ecbs_planner/schedule_cache.hpp"
#include "ecbs_planner/types.hpp"

namespace ecbs_planner
{

/// What the fleet reports about one robot.
struct RobotFacts
{
  bool idle{false};            // No task: others wait for it, never replan
};

struct ReplanConfig
{
  double clearance_m{2.0};         // Waypoints this close are the same place
  /// Parked-AMR "on the path" radius (m): distance to a path *segment*, not
  /// solver cell clearance. Keep separate from clearance_m (often ~0.05).
  double parked_block_radius_m{1.0};
  /// Other-AMR goal "on the path" radius (m): same segment rule as parked.
  double goal_block_radius_m{1.0};
  /// Max parked/goal REPLAN rounds per candidate per task before fallback.
  int replan_max_iterations{5};
  double replan_timeout_sec{15.0}; // Candidate must come back with a new path
  double blocked_margin_sec{2.0};  // Base half-width of the keeper time window
  /// Extra margin per second until the waypoint: margin = base + growth * dt.
  double blocked_margin_growth{0.1};
  /// Drop this much path (m) at the shared stretch end nearest the keeper.
  /// Related to safety_distance_m / base_tail_m (both 2.0): opening more than
  /// the reserved tail can let the candidate into the keeper's rear envelope.
  double blocked_tail_ignore_m{2.5};
  double safety_distance_m{2.0};   // For the tail-trim warning only
  double base_tail_m{2.0};         // For the tail-trim warning only
  double default_speed_mps{0.5};   // Speed for the ETA of a path with no usable times
  double reopen_guard_sec{10.0};   // No new replan for a pair this soon after its CLEAR
  double planning_window_sec{5.0}; // A new REPLAN only this soon after a goal's path is first seen
  int replan_max_retries{5};       // Alternatives tried before NO_ALTERNATIVE
};

enum class ReplanDecisionKind { Replan, NoAlternative, Clear };

/// One message for /ecbs_replan_candidate.
struct ReplanDecision
{
  ReplanDecisionKind kind{ReplanDecisionKind::Replan};
  std::string candidate;  // AMR that replans (closures are for it only)
  std::string keeper;     // AMR that keeps its path
  std::string task_id;    // Candidate's task
  std::string reason;
  std::vector<std::string> conflicting;
  std::vector<int64_t> blocked_waypoints;  // Keeper's remaining shared corridor
  std::vector<double> blocked_until_sec;   // Keeper leaves each one by then
  uint32_t revision{0};
};

/// True if \p a and \p b share at least two places (within \p eps_m) and
/// pass them in opposite order (head-on along a corridor).
bool opposite_overlap(const TimedPath & a, const TimedPath & b, double eps_m);

/// First come, first served: of a head-on pair, the AMR whose task was noted
/// later (ScheduleCache::note_path / upsert task_id) replans. The earlier
/// one never does. Unknown order: later robot id.
std::string choose_replan_candidate(
  const std::string & a, const std::string & b,
  const ScheduleCache & cache,
  std::string * why = nullptr);

/// Resolves head-on deadlocks (each AMR holds for the other), first come,
/// first served: the AMR that got its task first keeps its path and never
/// replans (task_id order in the cache; path_revision does not flip this).
/// The later AMR replans only if that gets it to its goal sooner than waiting
/// (ETA of its path with the wait vs ETA of the shortest route around the
/// first one's corridor; a tie keeps the wait). Then the first one's corridor
/// is closed for it until the first has passed it, so RMF routes it another
/// way. If waiting is sooner, CLEAR is published and both keep their paths.
/// A pair with an idle AMR (no task) is never replanned: the other waits.
/// With a nav graph, the later AMR replans only if it still reaches its goal
/// with that corridor closed; the shortest stretch of the corridor it must
/// drive first to get out (its exit) stays open. If it cannot, both keep
/// waiting. The candidate holds where it stands (no wait point ahead); the
/// keeper drives on and holds only just before the candidate's exit until the
/// candidate has left it.
/// Not thread-safe; call under the server's cache lock.
class ReplanCoordinator
{
public:
  explicit ReplanCoordinator(ReplanConfig config = {});

  void set_facts(const std::string & amr, const RobotFacts & facts);

  /// Path seeded but not yet /ecbs_path_approval'd: treat as a live candidate
  /// even if /fleet_states still shows idle (empty task_id).
  void set_awaiting(const std::string & amr, bool awaiting);

  /// Graph for the route check; without one every candidate is assumed to
  /// have a route.
  void set_graph(NavGraph graph);

  /// Detects new deadlocks, follows active ones and returns the decisions to
  /// publish. Clears the pair's holds in \p cache once the candidate's new
  /// path no longer runs head-on into the keeper.
  std::vector<ReplanDecision> update(ScheduleCache & cache, double now_sec);

  /// True if the cached path for \p amr still hits parked AMRs / goals.
  bool path_still_conflicted(
    const ScheduleCache & cache, const std::string & amr, double now_sec) const;

  /// Gate timeout: restore first RMF path + CLEAR. True if a session existed
  /// (CLEAR emitted, or already fallen back). False if there is no session.
  bool force_obstacle_fallback(
    ScheduleCache & cache, const std::string & amr,
    std::vector<ReplanDecision> & out);

  /// True if \p amr is in a pair being resolved.
  bool involved(const std::string & amr) const;

  /// AMRs whose cached path (waits) update() changed since the last call;
  /// republish them on /to_zone_manager.
  std::vector<std::string> take_changed();

  /// Why the wait-vs-detour check skipped or decided a pair (one line each,
  /// repeated lines suppressed). Log them.
  std::vector<std::string> take_notes();

private:
  void note(const std::string & pair_key, const std::string & text) const;
  mutable std::vector<std::string> notes_;
  mutable std::map<std::string, std::string> last_note_;

  struct Pair
  {
    std::string candidate;
    std::string keeper;
    TimedPath candidate_path;  // Candidate's path when it was chosen
    TimedPath keeper_path;     // Keeper's path when given up (re-detect on change)
    std::vector<int64_t> blocked;
    std::vector<double> until;
    std::vector<int64_t> exit;   // Keeper corridor the candidate leaves through
    std::vector<TimedPoint> exit_cells;  // Candidate's spot + exit (keeper's clear points)
    bool keeper_released{false};
    double decided_at{0.0};
    bool resolved{false};      // Candidate has a path clear of the keeper
    bool gave_up{false};       // No route around: wait for new paths
  };

  /// ETA (s) = candidate travel + wait. Prefer wait_sec from planned
  /// waiting_for on the path; else max(0, keeper_clear − now) + margin.
  /// nullopt if waiting cannot solve (candidate sits on the keeper's path)
  /// or there is no shared stretch / path wait.
  std::optional<double> eta_waiting(
    const TimedPath & candidate, const TimedPath & keeper, double now_sec) const;
  /// ETA (s) of the shortest route for \p candidate that stays out of the
  /// \p closed waypoints; nullopt if it cannot be computed.
  std::optional<double> eta_replanned(
    const TimedPath & candidate, const std::vector<int64_t> & closed) const;
  /// True if the geometric detour around \p closed still comes within
  /// clearance of \p keeper's path (excluding candidate start/goal). That
  /// means zone logic would re-attach waiting_for the same keeper — do not
  /// claim the detour is faster.
  bool detour_still_meets_keeper(
    const TimedPath & candidate, const TimedPath & keeper,
    const std::vector<int64_t> & closed) const;
  double speed_mps(const TimedPath & path) const;
  /// Wall-clock arrival of the candidate at (x,y), using speed_mps and waits.
  double candidate_arrival_at(
    const TimedPath & candidate, double x, double y, double now_sec) const;

  static std::string key(const std::string & a, const std::string & b);
  bool idle(const std::string & amr) const;
  /// Idle for CANDIDATE skips only: awaiting-approval AMRs are still movers.
  bool candidate_idle(const std::string & amr) const;
  bool awaiting(const std::string & amr) const;
  bool path_changed(const TimedPath & now, const TimedPath & then) const;
  /// Time-overlap + tail-trim corridor. \p attempt varies openness (0 = strict).
  void blocked_corridor(
    const TimedPath & keeper, const TimedPath & candidate_now,
    const TimedPath & candidate_orig,
    std::vector<int64_t> & wps, std::vector<double> & until,
    double now_sec, int attempt = 0) const;
  bool keeper_passed_blocked(
    const TimedPath & keeper, const Pair & p, const ScheduleCache & cache) const;
  struct RoutePlan
  {
    bool ok{false};
    std::vector<int64_t> closed;
    std::vector<double> until;
    std::vector<int64_t> exit;
    int attempt{-1};
  };
  RoutePlan plan_route(
    const TimedPath & candidate, const TimedPath & keeper, double now_sec = 0.0,
    const ScheduleCache * cache = nullptr) const;
  /// false: every route is dirty (idle AMR / another AMR's goal on it).
  bool avoid_idle_and_goals(
    RoutePlan & r, const TimedPath & candidate, const ScheduleCache & cache,
    double now_sec) const;
  bool has_route_around(
    const std::string & candidate, const std::string & keeper,
    const ScheduleCache & cache) const;
  void start(
    Pair & p, ScheduleCache & cache, double now_sec,
    const std::string & why, std::vector<ReplanDecision> & out);
  void hold_in_place(ScheduleCache & cache, const Pair & p) const;
  void hold_keeper_before_exit(ScheduleCache & cache, Pair & p) const;
  bool left_exit(const TimedPath & candidate, const Pair & p) const;
  ReplanDecision make(
    ReplanDecisionKind kind, const Pair & p, const std::string & reason,
    const ScheduleCache & cache);
  void drop_hold(
    ScheduleCache & cache, const std::string & waiter, const std::string & blocker) const;
  void clear_pair_holds(ScheduleCache & cache, const Pair & p) const;

  /// Parked AMR and/or other AMR goals on a path: iterative REPLAN with a
  /// cumulative blocked union; fallback to the first RMF path when exhausted.
  void replan_around_parked(
    ScheduleCache & cache, double now_sec, std::vector<ReplanDecision> & out);
  /// A later AMR's goal lies on an earlier AMR's remaining path: the later one
  /// replans only if the detour beats waiting for the earlier one to pass.
  void replan_goal_on_path(
    ScheduleCache & cache, double now_sec, std::vector<ReplanDecision> & out);

  /// True while \p path is still in its planning moment (first seen for its
  /// goal less than planning_window_sec ago). Never replan after that.
  bool planning(const TimedPath & path, double now_sec) const;

  struct BlockTag
  {
    int64_t wp{0};
    std::string owner;
    std::string reason;  // "parked amrX" or "goal of amrY"
  };
  struct IterSession
  {
    TimedPath first_path;
    TimedPath last_path;
    std::string task_id;
    int iteration{0};
    bool done{false};
    bool fallback{false};
    /// Last path_revision we already counted / acted on (avoid timer spam).
    uint32_t last_rev_seen{0};
    /// Cumulative blocked waypoints (never drop except when owner leaves).
    std::vector<BlockTag> union_blocks;
    /// Active keepers still holding closures for this candidate.
    std::map<std::string, Pair> keepers;  // owner → Pair slice
  };
  void reset_session(const std::string & amr);
  bool path_has_obstacles(
    const TimedPath & path, const ScheduleCache & cache, double now_sec,
    std::vector<BlockTag> * hits) const;
  /// True if at least one new waypoint was added to the union.
  bool merge_union(IterSession & s, const std::vector<BlockTag> & hits);
  void prune_stale_owners(IterSession & s, const ScheduleCache & cache);
  std::vector<int64_t> union_waypoints(const IterSession & s) const;
  void emit_fallback(
    IterSession & s, ScheduleCache & cache, const std::string & amr,
    const std::string & why, std::vector<ReplanDecision> & out);
  void emit_path_clear(
    IterSession & s, ScheduleCache & cache, const std::string & amr,
    int iteration, int max_iter, std::vector<ReplanDecision> & out);
  void emit_replan(
    IterSession & s, ScheduleCache & cache, const TimedPath & path,
    const std::vector<BlockTag> & hits, double now_sec, int max_iter,
    std::vector<ReplanDecision> & out);
  ReplanDecision make_obstacle(
    ReplanDecisionKind kind, const std::string & candidate,
    const IterSession & s, const std::string & reason,
    const ScheduleCache & cache);

  ReplanConfig config_;
  /// amr → (goal key, time its path for that goal was first seen)
  std::map<std::string, std::pair<std::string, double>> first_seen_;
  std::map<std::string, Pair> parked_;  // key(candidate, parked) → open REPLAN
  std::map<std::string, IterSession> sessions_;  // candidate → iterative state
  NavGraph graph_;
  std::unordered_map<std::string, RobotFacts> facts_;
  std::unordered_set<std::string> awaiting_;  // seeded, waiting for /ecbs_path_approval
  std::map<std::string, Pair> pairs_;
  std::unordered_map<std::string, double> cleared_at_;  // pair key → last CLEAR
  uint32_t revision_{0};
  mutable std::set<std::string> changed_;
  mutable bool warned_tail_{false};
};

}  // namespace ecbs_planner

#endif  // ECBS_PLANNER__REPLAN_CANDIDATE_HPP_
