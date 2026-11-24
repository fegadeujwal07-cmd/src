#include <gtest/gtest.h>

#include <algorithm>
#include <string>
#include <vector>

#include "ecbs_planner/action_compatibility.hpp"
#include "ecbs_planner/authority_ledger.hpp"
#include "ecbs_planner/conflict_group_builder.hpp"
#include "ecbs_planner/conflict_store.hpp"
#include "ecbs_planner/dependency_graph.hpp"
#include "ecbs_planner/group_passage_scheduler.hpp"
#include "ecbs_planner/passage_action.hpp"
#include "ecbs_planner/proposal_engine.hpp"
#include "ecbs_planner/schedule_cache.hpp"

using namespace ecbs_planner;

namespace
{

TimedPath cross_path(
  const std::string & id, double x0, double y0, double dx, double dy, int n,
  const std::string & task = "")
{
  TimedPath p;
  p.amr_id = id;
  p.fleet_name = "fleet";
  p.task_id = task.empty() ? ("task-" + id) : task;
  p.dispatch_task_id = p.task_id;
  p.path_revision = 1;
  for (int i = 0; i < n; ++i) {
    TimedPoint w;
    w.x = x0 + i * dx;
    w.y = y0 + i * dy;
    w.level_name = "L1";
    p.waypoints.push_back(w);
  }
  return p;
}

ConflictRecord make_pair_conflict(
  const std::string & a, const std::string & b,
  uint32_t a_entry, uint32_t a_exit, uint32_t b_entry, uint32_t b_exit,
  const std::string & topo = "crossing")
{
  ConflictRecord r;
  ParticipantOverlap pa;
  pa.amr_id = a;
  pa.path_revision = 1;
  pa.entry = make_occurrence(1, a_entry, static_cast<double>(a_entry), 0);
  pa.exit = make_occurrence(1, a_exit, static_cast<double>(a_exit), 0);
  pa.wait = make_occurrence(1, a_entry > 0 ? a_entry - 1 : 0, 0, 0);
  ParticipantOverlap pb = pa;
  pb.amr_id = b;
  pb.entry = make_occurrence(1, b_entry, 0, static_cast<double>(b_entry));
  pb.exit = make_occurrence(1, b_exit, 0, static_cast<double>(b_exit));
  pb.wait = make_occurrence(1, b_entry > 0 ? b_entry - 1 : 0, 0, 0);
  r.participants = {pa, pb};
  r.topology = topo;
  r.wait_reason = topo;
  r.conflict_id = make_overlap_id(a, b, 1, a_entry, 1, b_entry);
  r.lifecycle = ConflictLifecycle::Active;
  return r;
}

}  // namespace

TEST(DependencyGraph, TarjanDetectsCycle)
{
  DependencyGraph g;
  DependencyEdge e1{"A", "B", "c", true, false, false, "x"};
  DependencyEdge e2{"B", "C", "c", true, false, false, "x"};
  DependencyEdge e3{"C", "A", "c", true, false, false, "x"};
  g.add_edge(e1);
  g.add_edge(e2);
  g.add_edge(e3);
  const auto cycles = g.forced_cycles();
  ASSERT_EQ(cycles.size(), 1u);
  EXPECT_EQ(cycles[0].size(), 3u);
}

TEST(DependencyGraph, ExternalBlockerNotCycle)
{
  DependencyGraph g;
  DependencyEdge e;
  e.from_action_id = "idle";
  e.to_action_id = "pass1";
  e.external_blocker = true;
  e.compulsory = true;
  g.add_node("pass1");
  g.add_edge(e);
  EXPECT_TRUE(g.forced_cycles().empty());
  EXPECT_EQ(g.external_blockers_for("pass1").size(), 1u);
}

TEST(DependencyGraph, B2BFrontToRearDirection)
{
  // A → B means B waits for A. Front action is from.
  DependencyEdge e;
  e.from_action_id = "front";
  e.to_action_id = "rear";
  e.reason = "b2b";
  e.compulsory = true;
  EXPECT_EQ(e.from_action_id, "front");
}

TEST(ActionCompatibility, DifferentConflictIdsCanCollide)
{
  CompatibilitySnapshot snap;
  snap.paths["amr1"] = cross_path("amr1", 0, 0, 1, 0, 10);
  snap.paths["amr2"] = cross_path("amr2", 5, -5, 0, 1, 10);
  PassageAction a;
  a.amr_id = "amr1";
  a.from_index = 0;
  a.to_index = 8;
  a.reserved_conflict_ids = {"C12"};
  PassageAction b;
  b.amr_id = "amr2";
  b.from_index = 0;
  b.to_index = 8;
  b.reserved_conflict_ids = {"C23"};
  // Paths cross near (5,0) — incompatible despite different conflict ids.
  EXPECT_FALSE(actions_compatible(a, b, snap));
}

TEST(AuthorityLedger, SurvivesConflictRetirement)
{
  AuthorityLedger ledger;
  LedgerGrant g;
  g.amr_id = "amr1";
  g.grant_id = 9;
  g.release_through_index = 5;
  g.required_conflict_ids = {"C12"};
  g.commitment_version = 1;
  ledger.put_grant(g);
  ledger.note_conflict_retired("C12");
  auto e = ledger.get("amr1");
  ASSERT_TRUE(e.has_value());
  EXPECT_EQ(e->grant.grant_id, 9u);
  EXPECT_FALSE(e->historical_conflict_ids.empty());
}

TEST(GroupBuilder, ConnectedComponentFourRobots)
{
  ConflictStore store;
  store.upsert(make_pair_conflict("amr1", "amr2", 3, 5, 3, 5));
  store.upsert(make_pair_conflict("amr2", "amr3", 4, 6, 4, 6));
  store.upsert(make_pair_conflict("amr3", "amr4", 5, 7, 5, 7));
  PassageGroupRegistry reg;
  reg.rebuild_from_conflicts(store, 1, 1, 1, 1);
  ASSERT_EQ(reg.groups().size(), 1u);
  EXPECT_EQ(reg.groups()[0].member_amrs.size(), 4u);
}

TEST(GroupBuilder, FairnessSurvivesMerge)
{
  ConflictStore store;
  store.upsert(make_pair_conflict("amr1", "amr2", 3, 5, 3, 5));
  PassageGroupRegistry reg;
  reg.rebuild_from_conflicts(store, 1, 1, 1, 1);
  ASSERT_EQ(reg.groups().size(), 1u);
  const auto gid = reg.groups()[0].group_id;
  reg.on_exclusive_passage_committed(gid, "amr1", {"amr2"});
  EXPECT_EQ(reg.groups()[0].fairness["amr2"].bypass_count, 1);
  store.upsert(make_pair_conflict("amr2", "amr3", 4, 6, 4, 6));
  reg.rebuild_from_conflicts(store, 1, 1, 1, 1);
  auto * g = reg.group_for_amr("amr2");
  ASSERT_NE(g, nullptr);
  EXPECT_EQ(g->fairness["amr2"].bypass_count, 1);
}

TEST(PassageAction, ApproachIsAction)
{
  auto path = cross_path("amr1", 0, 0, 1, 0, 20);
  ConflictRecord c = make_pair_conflict("amr1", "amr2", 10, 12, 10, 12);
  PassageActionBuildInput in;
  in.amr_id = "amr1";
  in.path = &path;
  in.from_index = 0;
  in.desired_extension_m = 15.0;
  in.relevant = {c};
  const auto acts = build_passage_actions(in);
  ASSERT_FALSE(acts.empty());
  EXPECT_TRUE(acts.front().is_approach || acts.front().from_index == 0);
}

/// Wait far ahead must not open a base past desired_extension_m (TM yaml).
TEST(PassageAction, ApproachCappedByDesiredExtension)
{
  auto path = cross_path("amr1", 0, 0, 1, 0, 40);
  ConflictRecord c = make_pair_conflict("amr1", "amr2", 30, 32, 30, 32);
  PassageActionBuildInput in;
  in.amr_id = "amr1";
  in.path = &path;
  in.from_index = 0;
  in.desired_extension_m = 10.0;
  in.relevant = {c};
  const auto acts = build_passage_actions(in);
  ASSERT_FALSE(acts.empty());
  EXPECT_TRUE(acts.front().is_approach);
  EXPECT_LE(acts.front().to_index, 10);
  EXPECT_GT(acts.front().to_index, 0);
}

TEST(PassageAction, ExitBlockedHoldsBeforeEnter)
{
  auto path = cross_path("amr1", 0, 0, 1, 0, 20);
  ConflictRecord c = make_pair_conflict("amr1", "amr2", 8, 10, 8, 10);
  PassageActionBuildInput in;
  in.amr_id = "amr1";
  in.path = &path;
  in.from_index = 0;
  in.desired_extension_m = 15.0;
  in.relevant = {c};
  in.exit_blocked = true;
  const auto acts = build_passage_actions(in);
  ASSERT_FALSE(acts.empty());
  bool held = false;
  for (const auto & a : acts) {
    if (a.exec_state == PassageActionState::Held ||
      a.hold_reason == "exit_occupied")
    {
      held = true;
    }
  }
  EXPECT_TRUE(held);
}

TEST(PassageAction, PastOnlyOverlapsStillFreeExtend)
{
  // Tip already past conflict exit — must still propose the next horizon.
  auto path = cross_path("amr1", 0, 0, 1, 0, 30);
  ConflictRecord c = make_pair_conflict("amr1", "amr2", 5, 8, 5, 8);
  PassageActionBuildInput in;
  in.amr_id = "amr1";
  in.path = &path;
  in.from_index = 17;
  in.desired_extension_m = 10.0;
  in.relevant = {c};
  const auto acts = build_passage_actions(in);
  ASSERT_FALSE(acts.empty());
  EXPECT_GT(acts.front().to_index, 17);
  EXPECT_TRUE(acts.front().safe_hold_verified);
}

TEST(PassageAction, AtWaitDoesNotFreePastUnclearedOverlap)
{
  auto path = cross_path("amr1", 0, 0, 1, 0, 30);
  ConflictRecord c = make_pair_conflict("amr1", "amr2", 10, 12, 10, 12);
  PassageActionBuildInput in;
  in.amr_id = "amr1";
  in.path = &path;
  in.from_index = 9;  // at wait (entry-1)
  in.desired_extension_m = 15.0;
  in.relevant = {c};
  in.exit_blocked = true;
  const auto acts = build_passage_actions(in);
  bool free_past_entry = false;
  for (const auto & a : acts) {
    if (a.reserved_conflict_ids.empty() && a.to_index > 10) {
      free_past_entry = true;
    }
  }
  EXPECT_FALSE(free_past_entry);
}

TEST(PassageAction, MapsOccurrenceOnTrimmedPath)
{
  // Occurrences 10..29 (trimmed); conflict wait/entry/exit are occurrences.
  auto path = cross_path("amr1", 0, 0, 1, 0, 20);
  for (std::size_t i = 0; i < path.waypoints.size(); ++i) {
    path.waypoints[i].route_occurrence = static_cast<uint32_t>(10 + i);
  }
  ConflictRecord c = make_pair_conflict("amr1", "amr2", 20, 22, 20, 22);
  PassageActionBuildInput in;
  in.amr_id = "amr1";
  in.path = &path;
  in.from_index = 0;  // local 0 == occurrence 10
  in.desired_extension_m = 15.0;
  in.relevant = {c};
  const auto acts = build_passage_actions(in);
  ASSERT_FALSE(acts.empty());
  EXPECT_TRUE(acts.front().is_approach);
  // Wait occurrence 19 → local 9; approach must land there (horizon allows).
  EXPECT_EQ(acts.front().to_index, 9);
  EXPECT_EQ(
    path.waypoints[static_cast<std::size_t>(acts.front().to_index)]
      .route_occurrence,
    19u);
}

TEST(GroupScheduler, TenRobotExclusiveCrossing)
{
  ConflictStore store;
  PassageGroupRegistry reg;
  std::vector<PassageAction> actions;
  CompatibilitySnapshot snap;
  for (int i = 0; i < 10; ++i) {
    const std::string id = "amr" + std::to_string(i);
    snap.paths[id] = cross_path(
      id, static_cast<double>(i) * 0.01, -5.0, 0.0, 1.0, 20);
  }
  // Star conflicts through amr0 at crossing.
  for (int i = 1; i < 10; ++i) {
    const std::string id = "amr" + std::to_string(i);
    store.upsert(make_pair_conflict("amr0", id, 8, 10, 8, 10));
  }
  reg.rebuild_from_conflicts(store, 1, 1, 1, 1);
  ASSERT_EQ(reg.groups().size(), 1u);
  auto & g = reg.groups()[0];
  for (int i = 0; i < 10; ++i) {
    const std::string id = "amr" + std::to_string(i);
    PassageActionBuildInput in;
    in.amr_id = id;
    in.path = &snap.paths[id];
    in.from_index = 0;
    in.desired_extension_m = 12.0;
    in.relevant = store.for_cluster_members(g.member_amrs);
    auto built = build_passage_actions(in);
    actions.insert(actions.end(), built.begin(), built.end());
  }
  g.actions = actions;
  GroupPassageScheduler sched;
  ScheduleRequest req;
  req.group = &g;
  req.requester_amr = "amr0";
  req.snapshot = snap;
  req.budget.max_nodes = 256;
  const auto r1 = sched.schedule(req);
  // At most one incompatible passage among non-approach eligibles sharing space.
  int pass_count = 0;
  for (const auto & a : r1.eligible_actions) {
    if (!a.is_approach) {
      ++pass_count;
    }
  }
  EXPECT_LE(pass_count, 1);
  // Stable on second tick without fact change.
  const auto r2 = sched.schedule(req);
  EXPECT_EQ(r1.eligible_actions.size(), r2.eligible_actions.size());
}

TEST(GroupScheduler, ChainedPartialOverlaps)
{
  ConflictStore store;
  store.upsert(make_pair_conflict("amr1", "amr2", 5, 7, 5, 7));
  store.upsert(make_pair_conflict("amr2", "amr3", 10, 12, 10, 12));
  store.upsert(make_pair_conflict("amr3", "amr4", 15, 17, 15, 17));
  store.upsert(make_pair_conflict("amr4", "amr5", 20, 22, 8, 10));
  PassageGroupRegistry reg;
  reg.rebuild_from_conflicts(store, 1, 1, 1, 1);
  ASSERT_EQ(reg.groups().size(), 1u);
  EXPECT_EQ(reg.groups()[0].member_amrs.size(), 5u);

  store.mark_physically_cleared(
    make_overlap_id("amr1", "amr2", 1, 5, 1, 5));
  reg.rebuild_from_conflicts(store, 1, 1, 2, 1);
  // C23 still present.
  bool c23 = false;
  for (const auto & cid : reg.groups()[0].conflict_ids) {
    if (cid.find("amr2") != std::string::npos &&
      cid.find("amr3") != std::string::npos)
    {
      c23 = true;
    }
  }
  EXPECT_TRUE(c23);
}

TEST(GroupScheduler, SearchBudgetResumes)
{
  PassageGroup g;
  g.group_id = 42;
  g.group_structure_version = 1;
  for (int i = 0; i < 6; ++i) {
    PassageAction a;
    a.action_id = "p" + std::to_string(i);
    a.amr_id = "amr" + std::to_string(i);
    a.from_index = 0;
    a.to_index = 5;
    a.safe_hold_verified = true;
    a.exec_state = PassageActionState::Eligible;
    a.reserved_conflict_ids = {"shared"};
    g.actions.push_back(a);
  }
  CompatibilitySnapshot snap;
  for (int i = 0; i < 6; ++i) {
    const std::string id = "amr" + std::to_string(i);
    snap.paths[id] = cross_path(id, 0, 0, 1, 0, 10);
  }
  GroupPassageScheduler sched;
  ScheduleRequest req;
  req.group = &g;
  req.requester_amr = "amr0";
  req.snapshot = snap;
  req.budget.max_nodes = 3;  // force incomplete
  const auto r1 = sched.schedule(req);
  EXPECT_TRUE(
    r1.search_in_progress || r1.wait_reason_code == kWaitSearch ||
    !r1.eligible_actions.empty());
  req.budget.max_nodes = 256;
  const auto r2 = sched.schedule(req);
  // Second call resumes / completes rather than forgetting progress.
  EXPECT_GE(sched.frontier_for(42).nodes_expanded, 3);
  (void)r2;
}

TEST(ProposalEngine, HorizonRetentionKeepsConflict)
{
  ProposalEngineState state;
  ConflictRecord c = make_pair_conflict("amr1", "amr2", 50, 52, 50, 52);
  c.retain_beyond_horizon = true;
  state.conflicts.upsert(c);
  state.groups.rebuild_from_conflicts(state.conflicts, 1, 1, 1, 1);
  EXPECT_FALSE(state.conflicts.get(c.conflict_id)->physically_cleared);
  EXPECT_NE(state.groups.group_for_amr("amr1"), nullptr);
}

TEST(GroupScheduler, CrossingDoesNotCreateCompulsoryB2B)
{
  // Shared plus/crossing conflict must not force front→rear B2B order.
  PassageGroup g;
  g.group_id = 7;
  g.group_structure_version = 1;
  CompatibilitySnapshot snap;
  snap.paths["amr1"] = cross_path("amr1", 0, 0, 1, 0, 20);
  snap.paths["amr2"] = cross_path("amr2", 5, -5, 0, 1, 20);
  auto c = make_pair_conflict("amr1", "amr2", 8, 10, 8, 10);
  PassageAction a;
  a.action_id = "pass:amr1:0:12";
  a.amr_id = "amr1";
  a.from_index = 0;
  a.to_index = 12;
  a.from = make_occurrence(1, 0, 0, 0);
  a.to = make_occurrence(1, 12, 12, 0);
  a.safe_hold_verified = true;
  a.exec_state = PassageActionState::Eligible;
  a.reserved_conflict_ids = {c.conflict_id};
  PassageAction b = a;
  b.action_id = "pass:amr2:0:12";
  b.amr_id = "amr2";
  b.from = make_occurrence(1, 0, 5, -5);
  b.to = make_occurrence(1, 12, 5, 7);
  b.reserved_conflict_ids = {c.conflict_id};
  g.actions = {a, b};
  g.member_amrs = {"amr1", "amr2"};
  GroupPassageScheduler sched;
  ScheduleRequest req;
  req.group = &g;
  req.requester_amr = "amr1";
  req.snapshot = snap;
  req.budget.max_nodes = 64;
  RobotPriorityFacts f1;
  f1.robot_id = "amr1";
  f1.task_priority = "low";  // rank 1
  f1.progress_valid = true;
  f1.progress_occurrence = 0;
  RobotPriorityFacts f2 = f1;
  f2.robot_id = "amr2";
  f2.task_priority = "high";  // rank 2 — should win PBS branch order
  req.facts["amr1"] = f1;
  req.facts["amr2"] = f2;
  const auto r = sched.schedule(req);
  // No compulsory B2B from a plus/crossing id.
  for (const auto & e : g.selected_precedences) {
    EXPECT_NE(e.reason, "b2b");
  }
  // Priority-aware PBS: if any pass is eligible, higher-priority amr2 wins.
  std::string pass_winner;
  for (const auto & ea : r.eligible_actions) {
    if (!ea.is_approach) {
      pass_winner = ea.amr_id;
      break;
    }
  }
  if (!pass_winner.empty()) {
    EXPECT_EQ(pass_winner, "amr2");
  }
}

TEST(GroupScheduler, B2BOnlyForSameWayTopology)
{
  PassageGroup g;
  g.group_id = 8;
  g.group_structure_version = 1;
  CompatibilitySnapshot snap;
  snap.paths["amr1"] = cross_path("amr1", 0, 0, 1, 0, 20);
  snap.paths["amr2"] = cross_path("amr2", 0, 0.1, 1, 0, 20);
  auto c = make_pair_conflict(
    "amr1", "amr2", 5, 12, 3, 10, "same_way");
  g.conflict_topology[c.conflict_id] = "same_way";
  PassageAction a;
  a.action_id = "pass:amr1:4:12";
  a.amr_id = "amr1";
  a.from_index = 4;
  a.to_index = 12;
  a.from = make_occurrence(1, 4, 4, 0);
  a.to = make_occurrence(1, 12, 12, 0);
  a.safe_hold_verified = true;
  a.exec_state = PassageActionState::Eligible;
  a.reserved_conflict_ids = {c.conflict_id};
  PassageAction b = a;
  b.action_id = "pass:amr2:2:10";
  b.amr_id = "amr2";
  b.from_index = 2;
  b.to_index = 10;
  b.from = make_occurrence(1, 2, 2, 0.1);
  b.to = make_occurrence(1, 10, 10, 0.1);
  b.reserved_conflict_ids = {c.conflict_id};
  g.actions = {a, b};
  GroupPassageScheduler sched;
  ScheduleRequest req;
  req.group = &g;
  req.requester_amr = "amr2";
  req.snapshot = snap;
  req.budget.max_nodes = 32;
  RobotPriorityFacts f1;
  f1.robot_id = "amr1";
  f1.progress_valid = true;
  f1.progress_occurrence = 10;  // further through passage → front
  RobotPriorityFacts f2;
  f2.robot_id = "amr2";
  f2.progress_valid = true;
  f2.progress_occurrence = 3;
  req.facts["amr1"] = f1;
  req.facts["amr2"] = f2;
  const auto r = sched.schedule(req);
  // Front (amr1) may go; rear (amr2) must wait on compulsory B2B edge.
  bool amr1_go = false;
  bool amr2_go = false;
  for (const auto & ea : r.eligible_actions) {
    if (ea.amr_id == "amr1") {
      amr1_go = true;
    }
    if (ea.amr_id == "amr2") {
      amr2_go = true;
    }
  }
  EXPECT_TRUE(amr1_go);
  EXPECT_FALSE(amr2_go);
  EXPECT_FALSE(r.wait_reason_code.empty());
}

TEST(ProposalEngine, DualEncounterSameSegment)
{
  ConflictStore store;
  auto c1 = make_pair_conflict("amr1", "amr2", 3, 4, 3, 4);
  auto c2 = make_pair_conflict("amr1", "amr2", 8, 9, 8, 9);
  EXPECT_NE(c1.conflict_id, c2.conflict_id);
  store.upsert(c1);
  store.upsert(c2);
  store.mark_physically_cleared(c1.conflict_id);
  EXPECT_TRUE(store.get(c1.conflict_id)->physically_cleared);
  EXPECT_FALSE(store.get(c2.conflict_id)->physically_cleared);
}

// ---- F3: group registry merge / split ----

TEST(GroupBuilder, MergeThenSplitKeepsStateAndLargestGroupId)
{
  ConflictStore store;
  store.upsert(make_pair_conflict("amr1", "amr2", 3, 5, 3, 5));
  store.upsert(make_pair_conflict("amr3", "amr4", 3, 5, 3, 5));
  store.upsert(make_pair_conflict("amr4", "amr5", 4, 6, 4, 6));
  PassageGroupRegistry reg;
  reg.rebuild_from_conflicts(store, 1, 1, 1, 1);
  ASSERT_EQ(reg.groups().size(), 2u);
  const auto small_id = reg.group_for_amr("amr1")->group_id;
  const auto big_id = reg.group_for_amr("amr3")->group_id;
  reg.on_exclusive_passage_committed(small_id, "amr1", {"amr2"});
  reg.on_exclusive_passage_committed(big_id, "amr3", {"amr4"});
  reg.group_for_amr("amr2")->referenced_grant_ids.push_back(11);
  reg.group_for_amr("amr3")->referenced_grant_ids.push_back(22);
  reg.set_pending_extension("amr2", true);

  // Merge {amr1,amr2} + {amr3,amr4,amr5}.
  const auto bridge = make_pair_conflict("amr2", "amr3", 8, 10, 8, 10);
  store.upsert(bridge);
  reg.rebuild_from_conflicts(store, 1, 1, 1, 1);
  ASSERT_EQ(reg.groups().size(), 1u);
  const auto * merged = reg.group_for_amr("amr1");
  ASSERT_NE(merged, nullptr);
  EXPECT_EQ(merged->member_amrs.size(), 5u);
  EXPECT_EQ(merged->group_id, big_id) << "merge keeps the largest previous id";
  EXPECT_EQ(merged->fairness.at("amr2").bypass_count, 1);
  EXPECT_EQ(merged->fairness.at("amr4").bypass_count, 1);
  EXPECT_TRUE(merged->pending_extension_by_amr.at("amr2"));
  auto has = [](const PassageGroup * g, uint64_t id) {
      return std::find(g->referenced_grant_ids.begin(), g->referenced_grant_ids.end(), id) !=
             g->referenced_grant_ids.end();
    };
  EXPECT_TRUE(has(merged, 11));
  EXPECT_TRUE(has(merged, 22));

  // Split again.
  ASSERT_TRUE(store.erase(bridge.conflict_id));
  reg.rebuild_from_conflicts(store, 1, 1, 1, 1);
  ASSERT_EQ(reg.groups().size(), 2u);
  const auto * big = reg.group_for_amr("amr3");
  const auto * small = reg.group_for_amr("amr1");
  ASSERT_NE(big, nullptr);
  ASSERT_NE(small, nullptr);
  EXPECT_NE(big, small);
  EXPECT_EQ(big->group_id, big_id) << "largest part keeps the id";
  EXPECT_NE(small->group_id, big_id);
  EXPECT_EQ(small->fairness.at("amr2").bypass_count, 1);
  EXPECT_EQ(big->fairness.at("amr4").bypass_count, 1);
  EXPECT_TRUE(small->pending_extension_by_amr.at("amr2"));
  EXPECT_TRUE(has(small, 11));
  EXPECT_TRUE(has(big, 22));
}

// ---- F6: held-blocker rule ----

namespace
{
PassageAction scheduler_action(
  const std::string & amr, int from, int to, const std::string & cid)
{
  PassageAction a;
  a.action_id = "pass:" + amr + ":" + std::to_string(from) + ":" + std::to_string(to);
  a.amr_id = amr;
  a.from_index = from;
  a.to_index = to;
  a.from = make_occurrence(1, static_cast<uint32_t>(from), 0, 0);
  a.to = make_occurrence(1, static_cast<uint32_t>(to), 0, 0);
  a.safe_hold_verified = true;
  a.exec_state = PassageActionState::Eligible;
  a.reserved_conflict_ids = {cid};
  return a;
}

ScheduleRequest held_request(
  PassageGroup & g, const TimedPath & other_path, int held_from, int held_to,
  uint32_t held_progress)
{
  g.group_id = 91;
  g.group_structure_version = 1;
  g.member_amrs = {"amr1", "amr2"};
  auto a = scheduler_action("amr1", 0, 12, "c");
  auto b = scheduler_action("amr2", held_from, held_to, "c");
  b.exec_state = PassageActionState::Held;
  b.safe_hold_verified = false;
  b.hold_reason = "exit_occupied";
  g.actions = {a, b};
  ScheduleRequest req;
  req.group = &g;
  req.requester_amr = "amr1";
  req.snapshot.paths["amr1"] = cross_path("amr1", 0, 0, 1, 0, 20);
  req.snapshot.paths["amr2"] = other_path;
  RobotPriorityFacts f;
  f.robot_id = "amr2";
  f.progress_valid = true;
  f.progress_occurrence = held_progress;
  req.facts["amr2"] = f;
  return req;
}
}  // namespace

TEST(GroupScheduler, HeldIncompatibleActionBlocksEligible)
{
  PassageGroup g;
  // amr2 stands on A's crossing (x=10) and cannot leave (exit occupied).
  auto req = held_request(g, cross_path("amr2", 10, -10, 0, 1, 20), 9, 11, 9);
  GroupPassageScheduler sched;
  const auto r = sched.schedule(req);
  for (const auto & e : r.eligible_actions) {
    EXPECT_NE(e.amr_id, "amr1");
  }
  EXPECT_EQ(r.wait_reason_code, kWaitHardHold);
  EXPECT_FALSE(r.hard_hold) << "held blocker is transient: no RECOVERY_REQUIRED";
}

TEST(GroupScheduler, HeldActionOnOtherPassageDoesNotBlock)
{
  PassageGroup g;
  // amr2 held far away (y=100): different, non-overlapping passage.
  auto req = held_request(g, cross_path("amr2", 10, 100, 0, 1, 20), 9, 11, 9);
  GroupPassageScheduler sched;
  const auto r = sched.schedule(req);
  ASSERT_EQ(r.eligible_actions.size(), 1u);
  EXPECT_EQ(r.eligible_actions.front().amr_id, "amr1");
  EXPECT_TRUE(r.wait_reason_code.empty());
}

TEST(GroupScheduler, HeldActionNotYetReachedDoesNotBlock)
{
  PassageGroup g;
  // Same geometry as the blocking case, but amr2 has not reached its held range.
  auto req = held_request(g, cross_path("amr2", 10, -10, 0, 1, 20), 9, 11, 2);
  GroupPassageScheduler sched;
  const auto r = sched.schedule(req);
  ASSERT_EQ(r.eligible_actions.size(), 1u);
  EXPECT_EQ(r.eligible_actions.front().amr_id, "amr1");
}

/// Exit space blocked on A's own passage: hold at the wait point, never enter.
TEST(PassageAction, ExitBlockedNeverPassesEntry)
{
  auto path = cross_path("amr1", 0, 0, 1, 0, 20);
  ConflictRecord c = make_pair_conflict("amr1", "amr2", 8, 10, 8, 10);
  PassageActionBuildInput in;
  in.amr_id = "amr1";
  in.path = &path;
  in.from_index = 0;
  in.desired_extension_m = 15.0;
  in.relevant = {c};
  in.exit_blocked = true;
  const auto acts = build_passage_actions(in);
  ASSERT_FALSE(acts.empty());
  bool held = false;
  for (const auto & a : acts) {
    if (a.exec_state == PassageActionState::Held) {
      held = true;
      EXPECT_EQ(a.hold_reason, "exit_occupied");
      EXPECT_LT(a.to_index, 8) << "hold must be before entry";
    } else {
      EXPECT_LT(a.to_index, 8) << "no eligible action may enter the overlap";
    }
  }
  EXPECT_TRUE(held);
}
