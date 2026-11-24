#include "ecbs_planner/dependency_graph.hpp"

#include <algorithm>
#include <functional>
#include <stack>

namespace ecbs_planner
{

void DependencyGraph::clear()
{
  nodes_.clear();
  node_set_.clear();
  edges_.clear();
}

void DependencyGraph::add_node(const std::string & action_id)
{
  if (node_set_.insert(action_id).second) {
    nodes_.push_back(action_id);
  }
}

void DependencyGraph::add_edge(const DependencyEdge & e)
{
  add_node(e.from_action_id);
  add_node(e.to_action_id);
  edges_.push_back(e);
}

SccResult DependencyGraph::strongly_connected_components() const
{
  SccResult out;
  std::unordered_map<std::string, int> index;
  std::unordered_map<std::string, int> lowlink;
  std::unordered_map<std::string, bool> onstack;
  std::stack<std::string> st;
  int idx = 0;

  std::unordered_map<std::string, std::vector<std::string>> adj;
  for (const auto & n : nodes_) {
    adj[n] = {};
  }
  for (const auto & e : edges_) {
    if (e.external_blocker) {
      continue;  // external blockers are not cycle edges
    }
    adj[e.from_action_id].push_back(e.to_action_id);
  }

  std::function<void(const std::string &)> strongconnect;
  strongconnect = [&](const std::string & v) {
      index[v] = idx;
      lowlink[v] = idx;
      ++idx;
      st.push(v);
      onstack[v] = true;
      for (const auto & w : adj[v]) {
        if (!index.count(w)) {
          strongconnect(w);
          lowlink[v] = std::min(lowlink[v], lowlink[w]);
        } else if (onstack[w]) {
          lowlink[v] = std::min(lowlink[v], index[w]);
        }
      }
      if (lowlink[v] == index[v]) {
        std::vector<std::string> comp;
        while (true) {
          const auto w = st.top();
          st.pop();
          onstack[w] = false;
          comp.push_back(w);
          if (w == v) {
            break;
          }
        }
        std::sort(comp.begin(), comp.end());
        out.components.push_back(comp);
        if (comp.size() > 1) {
          out.nontrivial.push_back(comp);
        }
      }
    };

  std::vector<std::string> sorted = nodes_;
  std::sort(sorted.begin(), sorted.end());
  for (const auto & v : sorted) {
    if (!index.count(v)) {
      strongconnect(v);
    }
  }
  return out;
}

std::vector<std::vector<std::string>> DependencyGraph::forced_cycles() const
{
  DependencyGraph mand;
  for (const auto & n : nodes_) {
    mand.add_node(n);
  }
  for (const auto & e : edges_) {
    if (e.compulsory && !e.from_scheduling_choice && !e.external_blocker) {
      mand.add_edge(e);
    }
  }
  return mand.strongly_connected_components().nontrivial;
}

std::vector<std::vector<std::string>> DependencyGraph::scheduling_cycles() const
{
  return strongly_connected_components().nontrivial;
}

std::vector<DependencyEdge> DependencyGraph::external_blockers_for(
  const std::string & action_id) const
{
  std::vector<DependencyEdge> out;
  for (const auto & e : edges_) {
    if (e.external_blocker && e.to_action_id == action_id) {
      out.push_back(e);
    }
  }
  return out;
}

DependencyGraph build_mandatory_graph(
  const std::vector<PassageAction> & actions,
  const std::vector<DependencyEdge> & compulsory_extra)
{
  DependencyGraph g;
  std::unordered_map<std::string, std::vector<const PassageAction *>> by_amr;
  for (const auto & a : actions) {
    g.add_node(a.action_id);
    by_amr[a.amr_id].push_back(&a);
  }
  // Same-robot sequence: earlier → later (later waits for earlier clearance).
  for (auto & kv : by_amr) {
    auto & list = kv.second;
    std::sort(list.begin(), list.end(),
      [](const PassageAction * x, const PassageAction * y) {
        return x->from_index < y->from_index;
      });
    for (std::size_t i = 1; i < list.size(); ++i) {
      DependencyEdge e;
      e.from_action_id = list[i - 1]->action_id;
      e.to_action_id = list[i]->action_id;
      e.clearance_condition = "same_robot_prior_action";
      e.compulsory = true;
      e.reason = "same_robot_sequence";
      g.add_edge(e);
    }
  }
  for (const auto & e : compulsory_extra) {
    g.add_edge(e);
  }
  return g;
}

DependencyGraph build_candidate_graph(
  const DependencyGraph & mandatory,
  const std::vector<DependencyEdge> & tentative_choices)
{
  DependencyGraph g = mandatory;
  for (const auto & e : tentative_choices) {
    DependencyEdge c = e;
    c.from_scheduling_choice = true;
    c.compulsory = false;
    g.add_edge(c);
  }
  return g;
}

}  // namespace ecbs_planner
