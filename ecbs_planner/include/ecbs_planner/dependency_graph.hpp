#ifndef ECBS_PLANNER__DEPENDENCY_GRAPH_HPP_
#define ECBS_PLANNER__DEPENDENCY_GRAPH_HPP_

#include <cstdint>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "ecbs_planner/passage_action.hpp"

namespace ecbs_planner
{

/// A → B means B may start only after A satisfies its clearance condition.
struct DependencyEdge
{
  std::string from_action_id;  // A
  std::string to_action_id;    // B
  std::string clearance_condition;
  bool compulsory{true};
  bool from_scheduling_choice{false};
  /// External blocker (idle destination) — not an SCC membership requirement.
  bool external_blocker{false};
  std::string reason;
};

struct SccResult
{
  std::vector<std::vector<std::string>> components;
  /// Components with size > 1.
  std::vector<std::vector<std::string>> nontrivial;
};

class DependencyGraph
{
public:
  void clear();
  void add_node(const std::string & action_id);
  void add_edge(const DependencyEdge & e);

  const std::vector<std::string> & nodes() const { return nodes_; }
  const std::vector<DependencyEdge> & edges() const { return edges_; }

  /// Tarjan SCC.
  SccResult strongly_connected_components() const;

  /// Forced cycle: nontrivial SCC using only compulsory / non-choice edges.
  std::vector<std::vector<std::string>> forced_cycles() const;

  /// Scheduling cycle: nontrivial SCC on full candidate edges.
  std::vector<std::vector<std::string>> scheduling_cycles() const;

  /// External blockers that hold an action without forming a cycle.
  std::vector<DependencyEdge> external_blockers_for(
    const std::string & action_id) const;

private:
  std::vector<std::string> nodes_;
  std::unordered_set<std::string> node_set_;
  std::vector<DependencyEdge> edges_;
};

/// Build mandatory graph: same-robot sequence, physical, committed, B2B front→rear.
DependencyGraph build_mandatory_graph(
  const std::vector<PassageAction> & actions,
  const std::vector<DependencyEdge> & compulsory_extra);

/// Candidate = mandatory + tentative ordering choices.
DependencyGraph build_candidate_graph(
  const DependencyGraph & mandatory,
  const std::vector<DependencyEdge> & tentative_choices);

}  // namespace ecbs_planner

#endif  // ECBS_PLANNER__DEPENDENCY_GRAPH_HPP_
