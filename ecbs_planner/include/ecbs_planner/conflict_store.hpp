#ifndef ECBS_PLANNER__CONFLICT_STORE_HPP_
#define ECBS_PLANNER__CONFLICT_STORE_HPP_

#include <cmath>
#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "ecbs_planner/route_occurrence.hpp"
#include "ecbs_planner/types.hpp"

namespace ecbs_planner
{

/// Per-robot geometry and easiness on one overlap (never inferred from sort order).
struct ParticipantOverlap
{
  std::string amr_id;
  std::string task_id;
  int32_t phase_index{0};
  uint32_t path_revision{0};
  RouteOccurrence entry;
  RouteOccurrence exit;
  RouteOccurrence wait;
  /// This robot's remaining path blocks the peer.
  bool blocks_other{false};
  /// Peer remaining path blocks this robot.
  bool blocked_by_other{false};
};

enum class ConflictLifecycle
{
  Active = 0,
  Held,
  Cleared,
  Retired,
};

/// Persistent overlap / dependency. conflict_id is bookkeeping, not a zone.
/// IDs use route revision + segment occurrence (not raw float offsets).
struct ConflictRecord
{
  std::string conflict_id;
  std::vector<ParticipantOverlap> participants;
  std::string fleet_name;
  std::string topology;
  std::string wait_reason;
  std::string waiter_amr;
  std::string blocker_amr;
  bool mandatory{false};
  std::string clearance_condition;
  uint64_t source_route_version{0};
  uint64_t source_occupancy_version{0};
  uint64_t source_priority_version{0};
  uint64_t source_commitment_version{0};
  uint64_t record_version{0};
  bool tentative{false};
  bool physically_cleared{false};
  ConflictLifecycle lifecycle{ConflictLifecycle::Active};
  uint64_t group_id{0};
  /// Retain past query-horizon until clearance / authority settles.
  bool retain_beyond_horizon{true};

  std::vector<std::string> participant_amrs() const;
  const ParticipantOverlap * participant(const std::string & amr_id) const;
  ParticipantOverlap * participant(const std::string & amr_id);
};

/// Offset match tolerance (metres) for geometric updates without new IDs.
inline constexpr double kSegmentOffsetMatchTol_m = 0.25;

inline bool offsets_match(double a, double b, double tol = kSegmentOffsetMatchTol_m)
{
  return std::fabs(a - b) <= tol;
}

struct DirectedDependency
{
  std::string waiter;
  std::string blocker;
  std::string conflict_id;
  std::string reason;
  uint64_t source_version{0};
};

struct ConflictCluster
{
  uint64_t id{0};
  std::unordered_set<std::string> members;
  std::vector<DirectedDependency> edges;
  std::vector<std::string> proposed_precedence;
  uint64_t version{0};
  bool has_cycle{false};
};

/// Symmetric ID from stable route_occurrence stamps. Topology is deliberately
/// not part of the ID (reclassification must update the record, not fork it).
std::string make_overlap_id(
  const std::string & amr_a,
  const std::string & amr_b,
  uint32_t rev_a,
  uint32_t occ_a,
  uint32_t rev_b,
  uint32_t occ_b);

std::string make_conflict_id(
  const std::string & waiter,
  const std::string & blocker,
  uint32_t waiter_rev,
  uint32_t waiter_occ,
  uint32_t blocker_rev,
  uint32_t blocker_occ);

/// True if segments AB and CD properly intersect or come within eps_m.
bool segments_conflict(
  double ax, double ay, double bx, double by,
  double cx, double cy, double dx, double dy,
  double eps_m);

/// Minimum distance from point P to segment AB.
double point_to_segment_distance(
  double px, double py,
  double ax, double ay, double bx, double by);

class ConflictStore
{
public:
  bool upsert(const ConflictRecord & record);
  bool mark_physically_cleared(const std::string & conflict_id);
  bool erase(const std::string & conflict_id);
  void erase_participant(const std::string & amr_id);
  bool set_mandatory_dependency(
    const std::string & conflict_id,
    const std::string & waiter,
    const std::string & blocker,
    bool mandatory);

  std::optional<ConflictRecord> get(const std::string & conflict_id) const;
  std::vector<ConflictRecord> all() const;
  std::vector<ConflictRecord> involving(const std::string & amr_id) const;
  std::vector<ConflictRecord> for_cluster_members(
    const std::unordered_set<std::string> & members) const;
  std::vector<DirectedDependency> outbound_deps(const std::string & waiter) const;
  std::vector<ConflictCluster> build_clusters() const;
  std::vector<ConflictRecord> active_for_robot(const std::string & amr_id) const;
  uint64_t store_version() const { return store_version_; }
  void clear();
  std::size_t size() const { return by_id_.size(); }

private:
  static bool content_equal(const ConflictRecord & a, const ConflictRecord & b);
  static uint64_t stable_cluster_id(const std::unordered_set<std::string> & members);

  std::unordered_map<std::string, ConflictRecord> by_id_;
  uint64_t store_version_{0};
};

/// Which overlap a register wait belongs to: the waiter's next overlap with
/// \p blocker whose entry is after \p wait_occurrence. \p error is set (and
/// conflict_id empty) if the pair has no active record, the wait is inside an
/// overlap, or it is at/after the entry of every overlap.
struct RegisterMatch
{
  std::string conflict_id;
  std::string error;
};

RegisterMatch match_register_wait(
  const ConflictStore & store,
  const std::string & waiter,
  const std::string & blocker,
  uint32_t wait_occurrence);

/// \p detect_distance_m: clearance used for point/segment proximity and
/// topology/easiness (must match evaluate_path). \p position_epsilon_m: same
/// graph cell when nav index is missing (not a detection radius).
std::vector<ConflictRecord> discover_route_overlaps(
  const TimedPath & self,
  const std::vector<TimedPath> & others,
  double detect_distance_m,
  double position_epsilon_m,
  double safety_distance_m,
  uint64_t route_version,
  uint64_t occupancy_version);

}  // namespace ecbs_planner

#endif  // ECBS_PLANNER__CONFLICT_STORE_HPP_
