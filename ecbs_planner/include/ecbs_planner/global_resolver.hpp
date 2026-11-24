#ifndef ECBS_PLANNER__GLOBAL_RESOLVER_HPP_
#define ECBS_PLANNER__GLOBAL_RESOLVER_HPP_

// Global conflict resolver — ECBS is the single traffic authority.
//
// Decides every conflict of a route the moment the route is registered
// (TM PathIntent seed), over the FULL remaining routes of all robots — not a
// query horizon. Decisions are sticky: they are keyed by immutable route
// geometry and never re-decided because a robot moved. Robots are released
// past a decided wait point only after the robot passing first has physically
// cleared the overlap (progress from /fleet_states).
//
// Rulebook (per overlap of two robots; strongest first):
//   R1 feasibility   A robot that cannot stop safely outside the overlap
//                    (starts inside it / its granted base already reaches the
//                    approach) passes first. Neither can → unresolvable.
//   R2 blocking      A robot whose route ENDS inside the overlap would park on
//                    the other's path → it passes second.
//   R3 same way      Same-direction overlap: the robot ahead leads; the one
//                    behind follows at safety distance (robot from behind waits).
//   R4 opposite/crossing:
//        health      healthy robot first (an unhealthy one may never clear)
//        dependency  a robot already held for another robot before it reaches
//                    this overlap does not go first (no chain wait / standoff)
//        near        crossing only: the robot already at its approach first
//        free pass   a robot that clears the overlap before the other even
//                    reaches its wait point goes first (nobody slows down)
//        priority    task priority (high > low > none)
//        load        loaded robot first
//        eta         closer to the overlap first
//        fifo        route registered earlier first, then robot id
//   Group  All pairwise orders form a passage dependency graph (conflict
//          edges + each robot's route order). A cycle is a deadlock: the
//          weakest non-forced decision in it is flipped; if none can be,
//          the cycle is reported unresolvable (replan candidate).
//
// Safe wait point: the furthest route waypoint at least safety_distance_m
// (along the waiter's route) before the overlap entry, never behind the
// robot's committed position (pose / granted tip), off every other robot's
// remaining route (fallback: off the blocker's route) and off T/+ junctions.
//
// No foreign base: a release never comes within conflict_distance_m of any
// other robot's occupied stretch (body behind pose → granted tip); the tip
// is pulled back safety_distance_m before it.

#include <cstdint>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "ecbs_planner/types.hpp"

namespace ecbs_planner
{

struct ResolverConfig
{
  /// Centre-to-centre distance at which two route points conflict.
  double conflict_distance_m{1.0};
  /// Wait this far (along the route) before an overlap; release the waiter
  /// once the first robot is this far past the overlap exit.
  double safety_distance_m{2.0};
  /// Route sampling step for geometry.
  double sample_step_m{0.25};
  /// Occupancy kept behind a robot's pose (body rear).
  double body_back_m{0.5};
  /// Overlap shorter than this on BOTH routes is a crossing.
  double crossing_max_len_m{3.0};
  /// Two overlaps of the same pair closer than this are one block
  /// (no safe room to wait between them). <= 0 → 2*safety + conflict.
  double merge_gap_m{-1.0};
  /// Free-pass margin (distance ratio). 0 disables the free-pass rule.
  double free_pass_ratio{1.5};
  /// Ignore poses farther than this from the route.
  double max_offroute_m{3.0};
  /// Keep wait points this far from T/+ junction centres. <= 0 → safety.
  double junction_clear_m{-1.0};
  /// Conflicts are decided ONLY on the RMF handover (GoToPlace → EcbsGate).
  /// A TM PathIntent arriving first is parked (robot holds) until its
  /// handover arrives; decide_pending_intent() is the timeout fallback.
  bool decide_on_handover_only{false};
};

enum class OverlapKind { Opposite, SameDirection, Crossing };

const char * overlap_kind_name(OverlapKind k);

struct ResolverFacts
{
  int priority_rank{0};  // zone_manager task_priority_rank (2 high, 1 low, 0 none)
  bool loaded{false};
  bool healthy{true};
};

struct ResolverRoutePoint
{
  double x{0.0};
  double y{0.0};
};

struct ResolverParticipant
{
  std::string amr_id;
  double entry_s{0.0};
  double exit_s{0.0};
  int32_t entry_index{0};   // waypoint at/before entry
  int32_t exit_index{0};    // waypoint at/after exit
  bool parks_inside{false}; // route ends inside the overlap
};

struct ResolverDecision
{
  std::string conflict_id;
  OverlapKind kind{OverlapKind::Opposite};
  ResolverParticipant p[2];  // sorted by amr_id
  std::string first;         // passes the overlap first
  std::string second;        // waits at wait point
  std::string rule;          // human-readable rule that decided
  int strength{0};           // rule strength (cycle breaking)
  bool forced{false};
  bool unresolvable{false};
  bool relaxed_wait{false};  // wait point only clear of the blocker
  // Waiter hold (route occurrence on the waiter's route).
  int32_t wait_index{-1};
  double wait_s{0.0};
  double wait_x{0.0};
  double wait_y{0.0};
  // Release condition on the first robot's route.
  double release_s{0.0};
  double release_x{0.0};
  double release_y{0.0};
  bool cleared{false};
  uint64_t decided_seq{0};

  const ResolverParticipant & of(const std::string & amr) const
  {
    return p[0].amr_id == amr ? p[0] : p[1];
  }
  bool involves(const std::string & amr) const
  {
    return p[0].amr_id == amr || p[1].amr_id == amr;
  }
  const std::string & other(const std::string & amr) const
  {
    return p[0].amr_id == amr ? p[1].amr_id : p[0].amr_id;
  }
};

enum class ResolverStatus { Free, Wait, RouteComplete, UnknownRoute, Recovery };

const char * resolver_status_name(ResolverStatus s);

struct ResolverAnswer
{
  ResolverStatus status{ResolverStatus::UnknownRoute};
  int32_t from_index{-1};
  int32_t through_index{-1};
  double through_x{0.0};
  double through_y{0.0};
  double from_x{0.0};
  double from_y{0.0};
  double proposed_m{0.0};
  uint64_t proposal_id{0};
  std::vector<WaitDependency> waiting_for;
  std::vector<std::string> conflict_ids;
  std::string reason;
};

/// Something TM should act on (waiter released, decision changed).
struct ResolverEvent
{
  std::string amr_id;       // robot to re-query
  std::string blocker;
  std::string conflict_id;
  std::string text;
};

/// Replan suggestion for an unresolvable conflict.
struct ResolverReplan
{
  std::string conflict_id;
  std::string candidate;
  std::vector<std::string> robots;
  std::string reason;
};

class GlobalResolver
{
public:
  explicit GlobalResolver(ResolverConfig cfg = {});

  const ResolverConfig & config() const {return cfg_;}

  /// T/+ junction centres (map frame) for wait-point placement.
  void set_junctions(std::vector<std::pair<double, double>> centres);

  /// RMF handover (GoToPlace → EcbsGate → /from_zone_manager): the moment
  /// RMF hands the planned waypoints over, the route is stored and every
  /// conflict involving it is decided. \p base_key = fleet|task|phase|rev.
  /// Returns "decided" | "unchanged" | "ignored_stale" | "ignored".
  std::string register_handover(
    const std::string & amr_id,
    const std::string & base_key,
    const std::vector<ResolverRoutePoint> & points);

  /// TM PathIntent registration (what the robot will execute). Same
  /// identity + same geometry as the RMF handover → "rebound": decisions
  /// are kept, only waypoint numbering is remapped to the PathIntent.
  /// Otherwise "decided" (re-decided on the PathIntent) or "unchanged".
  std::string register_intent(
    const std::string & amr_id,
    const std::string & base_key,
    const std::string & session,
    const std::vector<ResolverRoutePoint> & points);

  /// Timeout fallback: no RMF handover arrived for the parked PathIntent —
  /// decide on the PathIntent itself. Returns false if nothing was parked.
  bool decide_pending_intent(const std::string & amr_id);
  bool has_pending_intent(const std::string & amr_id) const;

  /// Generic registration (tests / no handover): identical key + geometry →
  /// no-op (returns false); otherwise stored and decided immediately.
  bool upsert_route(
    const std::string & amr_id,
    const std::string & route_key,
    const std::vector<ResolverRoutePoint> & points);

  /// Robot without a task route: a standing obstacle at (x, y).
  void set_standing(const std::string & amr_id, double x, double y);

  bool has_route(const std::string & amr_id) const;
  /// Route key last stored (empty if none).
  std::string route_key(const std::string & amr_id) const;

  void set_facts(const std::string & amr_id, const ResolverFacts & facts);

  /// Live pose. Returns robots to re-query (waiters released).
  std::vector<ResolverEvent> update_pose(
    const std::string & amr_id, double x, double y);

  /// Next release for \p amr_id beyond \p current_tip (route occurrence).
  /// \p base_key (optional): the TM session identity; a mismatch with the
  /// registered route (newer RMF handover not yet bound) → WAIT at the tip.
  ResolverAnswer query(
    const std::string & amr_id, int32_t current_tip, double desired_m,
    const std::string & base_key = std::string());

  /// TM commit of a FREE proposal. Returns ACCEPTED | DUPLICATE |
  /// REJECTED_STALE | REJECTED_MISMATCH.
  std::string commit(
    const std::string & amr_id, uint64_t proposal_id,
    int32_t through_index, const std::string & status);

  /// Re-evaluate unresolvable conflicts with the current poses/grants
  /// (recovery; normal decisions are made on the RMF handover). Returns the
  /// conflict ids that became resolvable.
  std::vector<std::string> retry_unresolvable();

  /// Move every pending wait to the nearest safe waypoint before its overlap
  /// with the current decisions / progress (order is never changed, a wait
  /// never moves behind the robot's granted tip). Returns true if any moved.
  bool refine_waits(const std::set<std::string> & quiet_ids = {});

  std::vector<ResolverDecision> decisions() const;
  std::vector<ResolverDecision> decisions_for(const std::string & amr_id) const;

  int32_t granted_index(const std::string & amr_id) const;
  double progress_s(const std::string & amr_id) const;
  std::optional<ResolverRoutePoint> waypoint(
    const std::string & amr_id, int32_t index) const;
  std::vector<ResolverRoutePoint> route_points(const std::string & amr_id) const;

  /// Explanations since the last call (decisions, flips, clearances).
  std::vector<std::string> take_log();
  /// Unresolvable conflicts reported since the last call.
  std::vector<ResolverReplan> take_replans();
  /// Robots whose decisions changed since the last call (for viz / wake-up).
  std::vector<std::string> take_changed();

private:
  struct Sample
  {
    double s{0.0};
    double x{0.0};
    double y{0.0};
  };

  enum class Source { Generic, Handover, Intent };

  struct Route
  {
    std::string amr_id;
    std::string key;
    std::string base_key;
    std::string session;
    Source source{Source::Generic};
    std::size_t geo_hash{0};
    uint64_t reg_seq{0};
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> s;
    std::vector<Sample> samples;
    double length{0.0};
    double progress_s{0.0};
    bool progress_valid{false};
    double pose_x{0.0};
    double pose_y{0.0};
    bool pose_valid{false};
    int32_t granted{-1};
    uint64_t pending_proposal{0};
    int32_t pending_to{-1};
    bool standing() const {return x.size() <= 1 || length < 1e-6;}
  };

  struct Overlap
  {
    double a_in{0}, a_out{0}, b_in{0}, b_out{0};
    double b_at_a_in{0}, b_at_a_out{0};  // B arc at A's first / last sample
  };

  struct WaitChoice
  {
    int32_t index{-1};
    double s{0.0};
    int tier{0};  // 2 strict, 1 off other routes, 0 off blocker only
  };

  // registration
  void store_and_decide(
    const std::string & amr_id, const std::string & key, const std::string & base_key,
    const std::string & session, Source source,
    const std::vector<ResolverRoutePoint> & points, std::size_t geo_hash);
  static std::size_t geometry_hash(const std::vector<ResolverRoutePoint> & points);
  bool geometry_matches(const Route & cur, const std::vector<ResolverRoutePoint> & points) const;
  double map_s(const Route & from, const Route & to, double s) const;
  void rebind(
    const std::string & amr_id, const std::string & session,
    const std::vector<ResolverRoutePoint> & points);

  // geometry
  static void build_geometry(Route & r, double step);
  static int32_t index_at_or_before(const Route & r, double s);
  static int32_t index_at_or_after(const Route & r, double s);
  static Sample point_at(const Route & r, double s);
  double committed_s(const Route & r) const;
  double remaining_from(const Route & r) const;
  bool parked(const Route & r) const;

  std::vector<Overlap> find_overlaps(const Route & a, const Route & b) const;

  // decisions
  void resolve_for(const std::string & amr_id);
  void decide(ResolverDecision & d);
  std::optional<WaitChoice> find_wait(
    const Route & waiter, const ResolverDecision & d) const;
  void apply_order(
    ResolverDecision & d, const std::string & first, const std::string & rule,
    int strength, bool forced, bool require_wait = true);
  /// x's body (pose) stands on y's remaining path ahead of y.
  bool sits_on(const Route & x, const Route & y) const;
  bool assign_wait(ResolverDecision & d);
  void enforce_waiter_occupancy();
  void break_cycles();
  std::vector<std::string> find_cycle() const;
  /// Robot \p amr is already held (second in an active decision) at a wait
  /// point at/before \p entry_s on its route. Returns that blocker or "".
  std::string held_before(
    const std::string & amr, double entry_s, const std::string & skip_id) const;
  /// Arc length the blockers of \p amr's holds at/before \p entry_s still
  /// have to drive (on their own routes) until the LAST of those holds is
  /// released. 0 if there is none. Used to order two robots that are both
  /// already held: the one released sooner passes first.
  double hold_release_remaining(
    const std::string & amr, double entry_s, const std::string & skip_id) const;
  /// Same-direction leader still inside the shared section (follower trails).
  bool leader_in_section(const ResolverDecision & d) const;
  void report_unresolvable(ResolverDecision & d);
  std::string loser_by_priority(const std::string & a, const std::string & b) const;

  // release computation (no side effects)
  struct Cap
  {
    int32_t index{-1};
    std::vector<WaitDependency> deps;
    std::vector<std::string> conflict_ids;
    std::string reason;
    bool recovery{false};
  };
  Cap compute_cap(const Route & r, int32_t tip, int32_t target) const;

  void log(const std::string & line);
  void mark_changed(const std::string & amr_id);

  ResolverConfig cfg_;
  std::map<std::string, Route> routes_;
  std::unordered_map<std::string, ResolverFacts> facts_;
  std::map<std::string, ResolverDecision> decisions_;  // by conflict_id
  std::vector<std::pair<double, double>> junctions_;
  std::vector<std::string> log_;
  std::vector<ResolverReplan> replans_;
  std::vector<std::string> changed_;
  std::map<std::string, bool> replan_reported_;
  std::map<std::string, std::set<std::string>> seen_handover_keys_;
  struct PendingIntent
  {
    std::string base_key;
    std::string session;
    std::vector<ResolverRoutePoint> points;
  };
  std::map<std::string, PendingIntent> pending_intents_;
  uint64_t next_reg_seq_{1};
  uint64_t next_decided_seq_{1};
  uint64_t next_proposal_id_{1};
};

}  // namespace ecbs_planner

#endif  // ECBS_PLANNER__GLOBAL_RESOLVER_HPP_
