#ifndef ECBS_PLANNER__CONFLICT_GROUP_BUILDER_HPP_
#define ECBS_PLANNER__CONFLICT_GROUP_BUILDER_HPP_

#include <cstdint>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "ecbs_planner/conflict_store.hpp"
#include "ecbs_planner/dependency_graph.hpp"
#include "ecbs_planner/passage_action.hpp"

namespace ecbs_planner
{

struct FairnessState
{
  double waiting_age_sec{0.0};
  int bypass_count{0};
};

struct PassageGroup
{
  uint64_t group_id{0};
  uint64_t group_structure_version{0};
  uint64_t route_geometry_version{0};
  uint64_t authority_version{0};
  uint64_t occupancy_clearance_version{0};
  uint64_t policy_priority_version{0};

  std::unordered_set<std::string> member_amrs;
  std::vector<std::string> conflict_ids;
  /// conflict_id → topology (ids no longer embed it; scheduler B2B needs it).
  std::unordered_map<std::string, std::string> conflict_topology;
  std::vector<PassageAction> actions;
  std::vector<DependencyEdge> mandatory_deps;
  std::vector<DependencyEdge> selected_precedences;
  /// Grant ids referenced — lifetime owned by AuthorityLedger.
  std::vector<uint64_t> referenced_grant_ids;
  std::unordered_map<std::string, bool> pending_extension_by_amr;
  std::unordered_map<std::string, FairnessState> fairness;
  std::unordered_map<std::string, std::string> clearance_obligations;
};

class PassageGroupRegistry
{
public:
  /// Rebuild connected components; merge/split preserves fairness + grant refs.
  void rebuild_from_conflicts(
    const ConflictStore & store,
    uint64_t route_geometry_version,
    uint64_t authority_version,
    uint64_t occupancy_clearance_version,
    uint64_t policy_priority_version);

  std::vector<PassageGroup> & groups() { return groups_; }
  const std::vector<PassageGroup> & groups() const { return groups_; }

  PassageGroup * group_for_amr(const std::string & amr_id);
  const PassageGroup * group_for_amr(const std::string & amr_id) const;

  void set_pending_extension(const std::string & amr_id, bool pending);
  std::vector<std::string> amrs_with_pending_extension() const;

  /// Increment bypass only when eligible for exclusive passage and peer committed.
  void on_exclusive_passage_committed(
    uint64_t group_id,
    const std::string & winner_amr,
    const std::vector<std::string> & eligible_losers);

  void on_passage_committed_reset_bypass(
    uint64_t group_id, const std::string & amr_id);

  void bump_occupancy_version(uint64_t group_id);
  void bump_policy_version(uint64_t group_id);

private:
  std::vector<PassageGroup> groups_;
  static uint64_t stable_id(const std::unordered_set<std::string> & members);
};

}  // namespace ecbs_planner

#endif  // ECBS_PLANNER__CONFLICT_GROUP_BUILDER_HPP_
