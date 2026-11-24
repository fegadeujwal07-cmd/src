#ifndef ECBS_PLANNER__GROUP_PASSAGE_SCHEDULER_HPP_
#define ECBS_PLANNER__GROUP_PASSAGE_SCHEDULER_HPP_

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

#include "ecbs_planner/action_compatibility.hpp"
#include "ecbs_planner/conflict_group_builder.hpp"
#include "ecbs_planner/dependency_graph.hpp"
#include "ecbs_planner/passage_action.hpp"
#include "ecbs_planner/passage_arbitration.hpp"

namespace ecbs_planner
{

inline constexpr const char * kWaitClearance = "WAIT_CLEARANCE";
inline constexpr const char * kWaitSearch = "WAIT_SEARCH";
inline constexpr const char * kWaitHardHold = "WAIT_HARDHOLD";

struct SchedulerBudget
{
  int max_nodes{256};
  /// Optional wall clock; node budget is the deterministic primary limit.
  double max_wall_ms{5.0};
  bool use_wall_clock{false};
};

struct SchedulerSnapshotKey
{
  uint64_t group_structure_version{0};
  uint64_t route_geometry_version{0};
  uint64_t authority_version{0};
  uint64_t occupancy_clearance_version{0};
  uint64_t policy_priority_version{0};

  bool operator==(const SchedulerSnapshotKey & o) const
  {
    return group_structure_version == o.group_structure_version &&
           route_geometry_version == o.route_geometry_version &&
           authority_version == o.authority_version &&
           occupancy_clearance_version == o.occupancy_clearance_version &&
           policy_priority_version == o.policy_priority_version;
  }
};

struct SearchFrontier
{
  SchedulerSnapshotKey key;
  std::vector<DependencyEdge> undecided_choices;
  std::vector<DependencyEdge> cached_valid_order;
  int nodes_expanded{0};
  bool search_complete{false};
  bool has_verified_incumbent{false};
  std::vector<std::string> incumbent_action_ids;
};

struct ScheduleResult
{
  std::vector<PassageAction> eligible_actions;
  std::string wait_reason_code;  // empty if has eligible for requester
  std::string debug_summary;
  bool search_in_progress{false};
  bool hard_hold{false};
  std::vector<std::string> forced_cycle_action_ids;
};

struct ScheduleRequest
{
  PassageGroup * group{nullptr};
  std::string requester_amr;
  CompatibilitySnapshot snapshot;
  std::unordered_map<std::string, RobotPriorityFacts> facts;
  /// Per-AMR cluster easiness (from conflict_store flags via cluster_easiness_for).
  std::unordered_map<std::string, int> easiness_by_amr;
  SchedulerBudget budget;
  int max_bypass_count{3};
};

/// Resumable PBS-inspired precedence search over passage actions.
class GroupPassageScheduler
{
public:
  ScheduleResult schedule(ScheduleRequest & req);

  SearchFrontier & frontier_for(uint64_t group_id) { return frontiers_[group_id]; }
  void invalidate(uint64_t group_id) { frontiers_.erase(group_id); }
  void clear() { frontiers_.clear(); }

private:
  std::unordered_map<uint64_t, SearchFrontier> frontiers_;

  ScheduleResult run_search(ScheduleRequest & req, SearchFrontier & frontier);
  bool prefer_action(
    const PassageAction & a,
    const PassageAction & b,
    const ScheduleRequest & req) const;
};

}  // namespace ecbs_planner

#endif  // ECBS_PLANNER__GROUP_PASSAGE_SCHEDULER_HPP_
