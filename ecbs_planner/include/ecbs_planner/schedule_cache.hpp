#ifndef ECBS_PLANNER__SCHEDULE_CACHE_HPP_
#define ECBS_PLANNER__SCHEDULE_CACHE_HPP_

#include <mutex>           // Thread-safe access
#include <optional>        // get() may miss
#include <string>
#include <unordered_map>   // amr_id → TimedPath
#include <unordered_set>
#include <utility>
#include <vector>

#include "ecbs_planner/types.hpp"

namespace ecbs_planner
{

/// One waypoint of a reserved stretch; \p since orders reservations (the
/// AMR that reserved a place first keeps it).
struct ReservedCell
{
  double x{0.0};
  double y{0.0};
  uint64_t since{0};
};

/// Cells of \p other that block the AMR owning \p own: all except cells that
/// are the *same place* as one \p own reserved earlier (\p dist_m should be a
/// same-cell epsilon, e.g. 0.35 m — not clearance footprint, which would
/// absorb a peer's next corridor waypoint ~2 m ahead).
inline std::vector<ClearPoint> blocking_cells(
  const std::vector<ReservedCell> & other,
  const std::vector<ReservedCell> & own,
  double dist_m)
{
  const double d2 = dist_m * dist_m;
  std::vector<ClearPoint> out;
  for (const auto & c : other) {
    bool mine_first = false;
    for (const auto & o : own) {
      const double dx = o.x - c.x;
      const double dy = o.y - c.y;
      if (o.since < c.since && dx * dx + dy * dy <= d2) {
        mine_first = true;
        break;
      }
    }
    if (!mine_first) {
      out.push_back({c.x, c.y});
    }
  }
  return out;
}

/// In-memory view of committed timed paths (schedule database mirror for ECBS).
/// Primary source: paths ECBS itself commits after a successful solve.
/// Secondary: optional itinerary_set seeding (standalone EcbsNode only).
class ScheduleCache
{
public:
  /// Insert or replace the path for path.amr_id (no-op if amr_id empty).
  void upsert(const TimedPath & path);

  /// Remove one AMR from the cache.
  void erase(const std::string & amr_id);

  /// Remove all AMRs.
  void clear();

  /// Copy of one AMR's path, or nullopt if unknown.
  std::optional<TimedPath> get(const std::string & amr_id) const;

  /// All committed paths except `exclude_amr` (empty exclude → return all).
  std::vector<TimedPath> all_except(const std::string & exclude_amr) const;

  /// Number of AMRs currently in the cache.
  std::size_t size() const;

  /// Drop waypoints strictly before the nearest to (x,y) (rolling view).
  /// Keeps from that waypoint onward. route_occurrence stamps are preserved —
  /// do not treat the new vector index 0 as the original route start.
  /// Uses last pose hint for amortized nearest lookup.
  /// Returns true if an entry existed and was modified.
  bool erase_before_pose(const std::string & amr_id, double x, double y);

  /// Remember the base end last given (evaluate FREE) to this AMR.
  void set_granted_base(const std::string & amr_id, const GrantedBase & base);

  /// Base end given to this AMR, or nullopt if none.
  std::optional<GrantedBase> granted_base(const std::string & amr_id) const;

  /// Forget the given base (new plan, or robot passed it).
  void clear_granted_base(const std::string & amr_id);

  /// Record that \p waiter must hold for dep.amr_id until ZM registers the
  /// pair. Ignored for a pair already registered (either direction).
  /// \p replace_wait_point: update the wait point of an existing hold that
  /// is not pinned yet (a pinned wait point never changes).
  void add_hold(
    const std::string & waiter, const WaitDependency & dep,
    bool replace_wait_point = false);

  /// Hold every wait already on \p path (planner / seeded waits). A new
  /// task_id for path.amr_id first forgets that AMR's *outgoing* holds and
  /// registers (inbound holds from other waiters are kept).
  /// Planner wait points replace unpinned holds.
  void add_plan_holds(const TimedPath & path);

  /// Fix the wait point of \p waiter's hold for dep.amr_id for good
  /// (no-op if there is no such hold or it is already pinned).
  void pin_hold(const std::string & waiter, const WaitDependency & dep);

  /// Overwrite the wait point of an existing hold even if it is pinned
  /// (junction hard-rule corrections).
  void force_hold_wait_point(
    const std::string & waiter, const WaitDependency & dep);

  /// True if \p waiter's hold for \p blocker has a pinned wait point.
  bool hold_pinned(const std::string & waiter, const std::string & blocker) const;

  /// Unregistered holds of \p waiter (one per blocker).
  std::vector<WaitDependency> holds(const std::string & waiter) const;

  /// True if \p waiter holds for \p blocker.
  bool has_hold(const std::string & waiter, const std::string & blocker) const;

  /// AMRs holding for \p blocker.
  std::vector<std::string> waiters_for(const std::string & blocker) const;

  /// Drop \p waiter's hold for \p blocker only (the pair is not registered;
  /// \p blocker keeps any hold it has for \p waiter).
  void remove_hold(const std::string & waiter, const std::string & blocker);

  /// \p yielder waits for \p goer (decided, e.g. by a head-on replan):
  /// \p goer never holds or waits for \p yielder until cleared.
  void set_right_of_way(const std::string & goer, const std::string & yielder);
  void clear_right_of_way(const std::string & goer, const std::string & yielder);
  bool has_right_of_way(const std::string & goer, const std::string & other) const;

  /// ZM registered \p waiter waiting for \p blocker: both holds of the pair
  /// end (blocker is GO for this pair).
  void mark_registered(const std::string & waiter, const std::string & blocker);

  /// Blocker cleared the shared stretch: drop the registered pair so evaluate
  /// can FREE the waiter.
  void unmark_registered(const std::string & waiter, const std::string & blocker);

  /// Blockers ZM registered \p waiter waiting for.
  std::vector<std::string> registered_blockers(const std::string & waiter) const;

  /// True if ZM registered \p waiter waiting for \p blocker.
  bool registered_waiter(
    const std::string & waiter, const std::string & blocker) const;

  /// Stretch \p amr_id was last given (robot → base end on FREE, robot →
  /// wait point on WAIT). No other AMR may enter it, whatever the times.
  /// Cells it already had keep their reservation order.
  void set_reserved(const std::string & amr_id, const std::vector<ClearPoint> & cells);

  /// Reserved stretch of \p amr_id (empty if none).
  std::vector<ReservedCell> reserved(const std::string & amr_id) const;

  /// Reserved stretches of every AMR except \p exclude_amr, plus the live
  /// position of every standing robot (see standing) as a cell reserved
  /// before all others.
  std::unordered_map<std::string, std::vector<ReservedCell>> reserved_except(
    const std::string & exclude_amr) const;

  /// AMRs \p waiter waits for only because of their reserved stretch (no
  /// hold, no register needed: FREE once the stretch is out of the way).
  void set_reserve_waits(
    const std::string & waiter, std::unordered_set<std::string> blockers);
  std::unordered_set<std::string> reserve_waits(const std::string & waiter) const;

  /// Where \p waiter's path first met \p blocker's reserved stretch, kept
  /// while the stretch still touches its path (keep_reserve_conflicts).
  void set_reserve_conflict(
    const std::string & waiter, const std::string & blocker, const ClearPoint & at);
  std::optional<ClearPoint> reserve_conflict(
    const std::string & waiter, const std::string & blocker) const;
  void keep_reserve_conflicts(
    const std::string & waiter, const std::unordered_set<std::string> & blockers);

  /// Live position of \p amr_id (e.g. from /fleet_states).
  void set_pose(const std::string & amr_id, double x, double y);

  /// Default stretch kept reserved behind a moving robot (metres). Footprint
  /// (clearance_m) is applied separately around cells — not included here.
  static constexpr double kDefaultBaseTailM = 2.5;

  /// Length of a robot's reserved stretch kept behind it: not freed as it
  /// moves, and given with each stretch. Others are kept a footprint
  /// (clearance_m) away from those cells on top.
  void set_base_tail_m(double tail_m);
  double base_tail_m() const;

  /// set_pose, and free what the robot has passed: its cached path behind it
  /// is dropped, and its reserved stretch behind it except the last
  /// base_tail_m. The rest keeps its reservation order.
  void update_pose(const std::string & amr_id, double x, double y);

  /// Robot state from /fleet_states: \p idle = no task. An idle robot stands
  /// at its live pose wherever its cached path ended (it may have been
  /// pushed off it). When it turns idle after a task, that task's path,
  /// base, reserved stretch and holds are dropped — unless a new task was
  /// seeded in the meantime.
  void set_idle(const std::string & amr_id, bool idle);

  /// True if \p amr_id has a known position and stands there: idle (no
  /// task, no path, or at its path end) or waiting (hold, register or
  /// reserve wait).
  bool standing(const std::string & amr_id) const;

  /// Position of \p amr_id if it stands (see standing), else nullopt.
  std::optional<ClearPoint> standing_pose(const std::string & amr_id) const;

  /// Last known position of \p amr_id, moving or not.
  std::optional<ClearPoint> pose(const std::string & amr_id) const;

  /// \p amr_id got a path for \p task_key. A new key (or an AMR not seen
  /// yet) puts it behind every AMR noted so far; an empty key only notes an
  /// AMR not seen yet. First come, first served.
  void note_path(const std::string & amr_id, const std::string & task_key);

  /// True if \p a got its current path before \p b (both noted).
  bool accepted_before(const std::string & a, const std::string & b) const;

private:
  static constexpr double kArrivedM = 1.0;  // Robot this close to its path end = idle
  bool standing_locked(const std::string & amr_id, const ClearPoint & pose) const;
  bool erase_before_pose_locked(const std::string & amr_id, double x, double y);
  bool pair_registered_locked(const std::string & a, const std::string & b) const;
  bool pinned_locked(const std::string & waiter, const std::string & blocker) const;
  void unpin_locked(const std::string & waiter, const std::string & blocker);
  void add_hold_locked(
    const std::string & waiter, const WaitDependency & dep, bool replace_wait_point);
  /// Drop this AMR's outgoing holds/registers/pins and stale ZM registers
  /// naming it as blocker. Does **not** erase other waiters' holds for it.
  void forget_outgoing_holds_locked(const std::string & amr_id);
  /// Full bilateral wipe (robot gone): outgoing + inbound holds/registers/pins.
  void forget_holds_locked(const std::string & amr_id);

  mutable std::mutex mutex_;                         // Guards by_amr_
  std::unordered_map<std::string, TimedPath> by_amr_;  // Key = amr_id
  std::unordered_map<std::string, GrantedBase> granted_;  // Key = amr_id
  /// waiter → blocker → hold (wait point); kept until ZM register or until
  /// the blocker has passed.
  std::unordered_map<std::string,
    std::unordered_map<std::string, WaitDependency>> holds_;
  /// waiter → blockers whose hold wait point is pinned.
  std::unordered_map<std::string, std::unordered_set<std::string>> pinned_;
  /// waiter → blockers ZM registered it waiting for.
  std::unordered_map<std::string, std::unordered_set<std::string>> registered_;
  /// amr → reserved stretch (see set_reserved).
  std::unordered_map<std::string, std::vector<ReservedCell>> reserved_;
  uint64_t reserve_seq_{0};
  double base_tail_m_{kDefaultBaseTailM};
  /// waiter → blockers it waits for only because of their reserved stretch.
  std::unordered_map<std::string, std::unordered_set<std::string>> reserve_waits_;
  /// waiter → blocker → first place its path met the blocker's stretch.
  std::unordered_map<std::string,
    std::unordered_map<std::string, ClearPoint>> reserve_conflicts_;
  /// amr → last known position.
  std::unordered_map<std::string, ClearPoint> poses_;
  /// AMRs /fleet_states reports with no task.
  std::unordered_set<std::string> idle_;
  /// amr → task_id of its cached path while it was last seen busy.
  std::unordered_map<std::string, std::string> busy_task_;
  /// Task the holds / registers of an AMR belong to.
  std::unordered_map<std::string, std::string> hold_task_;
  /// amr → (task key, order its current path was accepted in).
  std::unordered_map<std::string, std::pair<std::string, uint64_t>> accepted_;
  /// goer → AMRs that yield to it (see set_right_of_way).
  std::unordered_map<std::string, std::unordered_set<std::string>> right_of_way_;
  uint64_t accept_seq_{0};
  /// Last nearest waypoint index per AMR (hint for erase_before_pose).
  mutable std::unordered_map<std::string, std::size_t> last_pose_wp_;
};

/// True if \p waiter is waiting for \p blocker (hold, register or reserved
/// stretch). Its reserved stretch must then not block \p blocker in turn.
inline bool waits_for(
  const ScheduleCache & cache, const std::string & waiter, const std::string & blocker)
{
  return cache.has_hold(waiter, blocker) || cache.registered_waiter(waiter, blocker) ||
         cache.reserve_waits(waiter).count(blocker) > 0;
}

}  // namespace ecbs_planner

#endif  // ECBS_PLANNER__SCHEDULE_CACHE_HPP_

