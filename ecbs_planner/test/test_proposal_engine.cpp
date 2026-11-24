#include <gtest/gtest.h>

#include "ecbs_planner/action_compatibility.hpp"
#include "ecbs_planner/ecbs_solver.hpp"
#include "ecbs_planner/evaluate_path.hpp"
#include "ecbs_planner/nav_graph_junctions.hpp"
#include "ecbs_planner/path_utils.hpp"
#include "ecbs_planner/proposal_engine.hpp"
#include "ecbs_planner/schedule_cache.hpp"
#include "ecbs_planner/types.hpp"

using ecbs_planner::ProposalEngineState;
using ecbs_planner::ProposalQueryInput;
using ecbs_planner::ScheduleCache;
using ecbs_planner::TimedPath;
using ecbs_planner::TimedPoint;
using ecbs_planner::WaitDependency;
using ecbs_planner::apply_commit;
using ecbs_planner::apply_clearance;
using ecbs_planner::kGrantCommitted;
using ecbs_planner::kStatusFreeProposal;
using ecbs_planner::kStatusStaleRoute;
using ecbs_planner::kStatusWait;
using ecbs_planner::make_occurrence;
using ecbs_planner::query_base_extension;
using ecbs_planner::refresh_conflicts_from_cache;
using ecbs_planner::retire_grant;

TimedPath line_path(
  const std::string & id, int n, double x0, double dx, uint32_t rev = 1)
{
  TimedPath p;
  p.amr_id = id;
  p.fleet_name = "fleet";
  p.task_id = "task-" + id;
  p.path_revision = rev;
  for (int i = 0; i < n; ++i) {
    TimedPoint w;
    w.x = x0 + i * dx;
    w.y = 0.0;
    w.level_name = "L1";
    p.waypoints.push_back(w);
  }
  return p;
}

TEST(ProposalEngine, StaleRouteWhenNotSeeded)
{
  ProposalEngineState state;
  ScheduleCache cache;
  ProposalQueryInput in;
  in.amr_id = "amr1";
  in.path_revision = 1;
  in.desired_extension_m = 10.0;
  in.distance_origin = "GRANT_TIP";
  const auto r = query_base_extension(state, cache, in, 0.5, 0.35);
  EXPECT_EQ(r.status, kStatusStaleRoute);
}

/// Tip at last cached occurrence (e.g. grant=17 on an 18-wp cache while
/// PathIntent still has 27) must STALE with diagnostic sizes — not silent WAIT.
TEST(ProposalEngine, StaleRouteAtCacheEndLogsContinuationDiagnostics)
{
  ProposalEngineState state;
  ScheduleCache cache;
  // 18 waypoints → last index 17 (matches "stuck at tip 17" reports).
  cache.upsert(line_path("amr1", 18, 0.0, 1.0));
  ProposalQueryInput in;
  in.amr_id = "amr1";
  in.path_revision = 1;
  in.current_grant_through_index = 17;
  in.desired_extension_m = 20.0;
  in.distance_origin = "GRANT_TIP";
  const auto r = query_base_extension(state, cache, in, 0.5, 0.35);
  EXPECT_EQ(r.status, kStatusStaleRoute);
  EXPECT_NE(r.message.find("cache_wps=18"), std::string::npos);
  EXPECT_NE(r.message.find("from_idx=17"), std::string::npos);
  EXPECT_NE(r.message.find("cache_last=17"), std::string::npos);
  EXPECT_NE(r.message.find("grant_through=17"), std::string::npos);
}

TEST(ProposalEngine, FreeProposalWithoutConflict)
{
  ProposalEngineState state;
  ScheduleCache cache;
  cache.upsert(line_path("amr1", 20, 0.0, 1.0));
  ProposalQueryInput in;
  in.amr_id = "amr1";
  in.path_revision = 1;
  in.current_grant_through_index = -1;
  in.desired_extension_m = 10.0;
  in.distance_origin = "GRANT_TIP";
  const auto r = query_base_extension(state, cache, in, 0.5, 0.35);
  EXPECT_EQ(r.status, kStatusFreeProposal);
  EXPECT_GT(r.proposal_id, 0u);
  EXPECT_TRUE(state.pending_proposals.count(r.proposal_id) > 0);
}

TEST(ProposalEngine, CommitIdempotentDuplicate)
{
  ProposalEngineState state;
  ScheduleCache cache;
  cache.upsert(line_path("amr1", 20, 0.0, 1.0));
  ProposalQueryInput in;
  in.amr_id = "amr1";
  in.path_revision = 1;
  in.desired_extension_m = 8.0;
  in.distance_origin = "GRANT_TIP";
  const auto r = query_base_extension(state, cache, in, 0.5, 0.35);
  ASSERT_EQ(r.status, kStatusFreeProposal);
  const auto a1 = apply_commit(
    state, r.proposal_id, 10, "amr1", kGrantCommitted, r.release_through_index,
    r.from_occurrence, r.to_occurrence, 1, "task-amr1", &cache);
  EXPECT_EQ(a1, "ACCEPTED");
  const auto ver = state.commitment_version;
  const auto a2 = apply_commit(
    state, r.proposal_id, 10, "amr1", kGrantCommitted, r.release_through_index,
    r.from_occurrence, r.to_occurrence, 1, "task-amr1", &cache);
  EXPECT_EQ(a2, "DUPLICATE");
  EXPECT_EQ(state.commitment_version, ver);
  EXPECT_TRUE(cache.granted_base("amr1").has_value());
}

TEST(ProposalEngine, StaleCommitRejected)
{
  ProposalEngineState state;
  state.latest_proposal_by_amr["amr1"] = 5;
  const auto result = apply_commit(
    state, 3, 10, "amr1", kGrantCommitted, 4,
    make_occurrence(1, 0, 0, 0), make_occurrence(1, 4, 4, 0),
    1, "task");
  EXPECT_EQ(result, "REJECTED_STALE");
}

TEST(ProposalEngine, ClearanceDoesNotClearSiblingDeps)
{
  ProposalEngineState state;
  ecbs_planner::ConflictRecord a;
  a.conflict_id = "c1";
  ecbs_planner::ParticipantOverlap p1;
  p1.amr_id = "amr1";
  ecbs_planner::ParticipantOverlap p2;
  p2.amr_id = "amr2";
  a.participants = {p1, p2};
  a.mandatory = true;
  a.waiter_amr = "amr1";
  a.blocker_amr = "amr2";
  ecbs_planner::ConflictRecord b = a;
  b.conflict_id = "c2";
  state.conflicts.upsert(a);
  state.conflicts.upsert(b);
  EXPECT_TRUE(apply_clearance(state, "amr2", "c1", true));
  EXPECT_EQ(state.conflicts.outbound_deps("amr1").size(), 1u);
  // Waiter / foreign / empty cannot clear blocker's obligation.
  EXPECT_FALSE(apply_clearance(state, "amr1", "c2", true));
  EXPECT_FALSE(apply_clearance(state, "amr9", "c2", true));
  EXPECT_FALSE(apply_clearance(state, "", "c2", true));
  EXPECT_TRUE(apply_clearance(state, "amr2", "c2", true));
  // Duplicate clearance is idempotent (no crash / still ok).
  EXPECT_TRUE(apply_clearance(state, "amr2", "c2", true));
}

TEST(ProposalEngine, CommitRejectsExpandedEndpoint)
{
  ProposalEngineState state;
  ScheduleCache cache;
  cache.upsert(line_path("amr1", 20, 0.0, 1.0));
  ProposalQueryInput in;
  in.amr_id = "amr1";
  in.path_revision = 1;
  in.desired_extension_m = 5.0;
  in.distance_origin = "GRANT_TIP";
  const auto r = query_base_extension(state, cache, in, 0.5, 0.35);
  ASSERT_EQ(r.status, kStatusFreeProposal);
  const auto bad = apply_commit(
    state, r.proposal_id, 11, "amr1", kGrantCommitted,
    r.release_through_index + 5,
    r.from_occurrence, make_occurrence(1, r.release_through_index + 5, 0, 0),
    1, "task-amr1", &cache);
  EXPECT_EQ(bad, "REJECTED_MISMATCH");
}

TEST(ProposalEngine, RetireOldGrantKeepsNewerAuthority)
{
  ProposalEngineState state;
  ScheduleCache cache;
  cache.upsert(line_path("amr1", 20, 0.0, 1.0));
  ProposalQueryInput in;
  in.amr_id = "amr1";
  in.path_revision = 1;
  in.desired_extension_m = 4.0;
  in.distance_origin = "GRANT_TIP";
  const auto r1 = query_base_extension(state, cache, in, 0.5, 0.35);
  ASSERT_EQ(r1.status, kStatusFreeProposal);
  apply_commit(
    state, r1.proposal_id, 1, "amr1", kGrantCommitted, r1.release_through_index,
    r1.from_occurrence, r1.to_occurrence, 1, "task-amr1", &cache);
  in.current_grant_through_index = r1.release_through_index;
  in.desired_extension_m = 8.0;
  const auto r2 = query_base_extension(state, cache, in, 0.5, 0.35);
  ASSERT_EQ(r2.status, kStatusFreeProposal);
  apply_commit(
    state, r2.proposal_id, 2, "amr1", kGrantCommitted, r2.release_through_index,
    r2.from_occurrence, r2.to_occurrence, 1, "task-amr1", &cache);
  EXPECT_TRUE(retire_grant(state, "amr1", 1, &cache));
  EXPECT_TRUE(state.active_go.count("amr1") > 0);
  EXPECT_EQ(state.commitments["amr1"].grant_id, 2u);
  EXPECT_TRUE(cache.granted_base("amr1").has_value());
}

TEST(ProposalEngine, RetireGrantRemovesActiveGo)
{
  ProposalEngineState state;
  ScheduleCache cache;
  cache.upsert(line_path("amr1", 10, 0.0, 1.0));
  ProposalQueryInput in;
  in.amr_id = "amr1";
  in.path_revision = 1;
  in.desired_extension_m = 5.0;
  in.distance_origin = "GRANT_TIP";
  const auto r = query_base_extension(state, cache, in, 0.5, 0.35);
  ASSERT_EQ(r.status, kStatusFreeProposal);
  apply_commit(
    state, r.proposal_id, 77, "amr1", kGrantCommitted, r.release_through_index,
    r.from_occurrence, r.to_occurrence, 1, "task-amr1", &cache);
  EXPECT_TRUE(state.active_go.count("amr1") > 0);
  EXPECT_TRUE(retire_grant(state, "amr1", 77, &cache));
  EXPECT_TRUE(state.active_go.count("amr1") == 0);
  EXPECT_FALSE(cache.granted_base("amr1").has_value());
}

TEST(ProposalEngine, FromIndexBounded)
{
  ProposalEngineState state;
  ScheduleCache cache;
  cache.upsert(line_path("amr1", 5, 0.0, 1.0));
  ProposalQueryInput in;
  in.amr_id = "amr1";
  in.path_revision = 1;
  in.current_grant_through_index = 999;
  in.desired_extension_m = 10.0;
  in.distance_origin = "GRANT_TIP";
  const auto r = query_base_extension(state, cache, in, 0.5, 0.35);
  // Tip past cached route end must not clamp onto a different local waypoint.
  EXPECT_EQ(r.status, kStatusStaleRoute);
}

TEST(ProposalEngine, RouteVersionStableWhenUnchanged)
{
  ProposalEngineState state;
  ScheduleCache cache;
  cache.upsert(line_path("amr1", 10, 0.0, 1.0));
  refresh_conflicts_from_cache(state, cache, 0.5, 0.35);
  const auto v1 = state.route_version;
  refresh_conflicts_from_cache(state, cache, 0.5, 0.35);
  EXPECT_EQ(state.route_version, v1);
}

TEST(ProposalEngine, MixedFleetScenarioSmoke)
{
  for (const int n : {10, 20, 30}) {
    ProposalEngineState state;
    ScheduleCache cache;
    for (int i = 0; i < n; ++i) {
      cache.upsert(line_path("amr" + std::to_string(i), 20, i * 0.1, 1.0));
    }
    refresh_conflicts_from_cache(state, cache, 0.35, 0.35);
    for (int i = 0; i < n; ++i) {
      ProposalQueryInput in;
      in.amr_id = "amr" + std::to_string(i);
      in.path_revision = 1;
      in.desired_extension_m = 10.0;
      in.distance_origin = "GRANT_TIP";
      const auto r = query_base_extension(state, cache, in, 0.35, 0.35);
      EXPECT_TRUE(
        r.status == kStatusFreeProposal || r.status == kStatusWait ||
        r.status == kStatusStaleRoute);
    }
  }
}

/// Planner UUID (dispatch) ≠ execution order id "1" — commit uses proposal identity.
TEST(ProposalEngine, CanonicalTaskIdDiffersFromExecutionOrder)
{
  ProposalEngineState state;
  ScheduleCache cache;
  TimedPath p = line_path("amr1", 20, 0.0, 1.0);
  p.task_id = "planner-uuid-aaaa";
  p.dispatch_task_id = "planner-uuid-aaaa";
  cache.upsert(p);

  ProposalQueryInput in;
  in.amr_id = "amr1";
  in.path_revision = 1;
  in.task_id = "planner-uuid-aaaa";
  in.execution_order_id = "1";
  in.desired_extension_m = 8.0;
  in.distance_origin = "GRANT_TIP";
  const auto r = query_base_extension(state, cache, in, 0.5, 0.35);
  ASSERT_EQ(r.status, kStatusFreeProposal);
  EXPECT_EQ(r.task_id, "planner-uuid-aaaa");
  EXPECT_EQ(r.execution_order_id, "1");
  EXPECT_EQ(state.pending_proposals[r.proposal_id].task_id, "planner-uuid-aaaa");
  EXPECT_EQ(
    state.pending_proposals[r.proposal_id].execution_order_id, "1");

  // Commit with exact proposal canonical id (not order id "1").
  const auto ok = apply_commit(
    state, r.proposal_id, 42, "amr1", kGrantCommitted, r.release_through_index,
    r.from_occurrence, r.to_occurrence, 1, r.task_id, &cache);
  EXPECT_EQ(ok, "ACCEPTED");
  EXPECT_EQ(state.commitments["amr1"].task_id, "planner-uuid-aaaa");
  EXPECT_EQ(state.commitments["amr1"].execution_order_id, "1");

  // Reconstructing with execution order id must not accept against pending.
  ProposalQueryInput in2 = in;
  in2.current_grant_through_index = r.release_through_index;
  const auto r2 = query_base_extension(state, cache, in2, 0.5, 0.35);
  if (r2.status == kStatusFreeProposal) {
    const auto bad = apply_commit(
      state, r2.proposal_id, 43, "amr1", kGrantCommitted,
      r2.release_through_index, r2.from_occurrence, r2.to_occurrence,
      1, "1", &cache);
    EXPECT_EQ(bad, "REJECTED_MISMATCH");
  }
}

/// At evaluate wait tip with a hold and no conflict group → WAIT + waiting_for.
TEST(ProposalEngine, WaitAtHoldPublishesWaitingFor)
{
  ProposalEngineState state;
  ScheduleCache cache;
  TimedPath p = line_path("amr1", 20, 0.0, 1.0);
  WaitDependency dep;
  dep.amr_id = "amr2";
  dep.topology = "crossing";
  dep.wait_point.x = 5.0;
  dep.wait_point.y = 0.0;
  dep.wait_point.index = 5;
  dep.clear_points.push_back({10.0, 0.0});
  p.waypoints[5].waiting_for.push_back(dep);
  p.waypoints[5].wait_duration_sec = 1.0;
  cache.upsert(p);
  cache.add_plan_holds(p);
  // Peer not on a conflicting route → no passage group to schedule against.

  ProposalQueryInput in;
  in.amr_id = "amr1";
  in.path_revision = 1;
  in.current_grant_through_index = 5;
  in.desired_extension_m = 10.0;
  in.distance_origin = "GRANT_TIP";
  const auto r = query_base_extension(state, cache, in, 0.5, 0.35);
  EXPECT_EQ(r.status, kStatusWait);
  ASSERT_FALSE(r.waiting_for.empty());
  EXPECT_EQ(r.waiting_for.front().amr_id, "amr2");
}

/// After pose-prefix erase, TM grant tip (original occurrence) must still
/// extend past that tip — not clamp onto the trimmed vector's last index.
TEST(ProposalEngine, GrantTipSurvivesPoseTrim)
{
  ProposalEngineState state;
  ScheduleCache cache;
  cache.upsert(line_path("amr1", 27, 0.0, 1.0));
  // Robot at original wp 10 → erase leaves occurrences 10..26 (17 wps).
  ASSERT_TRUE(cache.erase_before_pose("amr1", 10.0, 0.0));
  const auto trimmed = cache.get("amr1");
  ASSERT_TRUE(trimmed.has_value());
  ASSERT_EQ(trimmed->waypoints.size(), 17u);
  EXPECT_EQ(trimmed->waypoints.front().route_occurrence, 10u);
  EXPECT_EQ(trimmed->waypoints.back().route_occurrence, 26u);

  ProposalQueryInput in;
  in.amr_id = "amr1";
  in.path_revision = 1;
  in.current_grant_through_index = 17;  // original tip, not local 17
  in.desired_extension_m = 10.0;
  in.distance_origin = "GRANT_TIP";
  const auto r = query_base_extension(state, cache, in, 0.5, 0.35);
  EXPECT_EQ(r.status, kStatusFreeProposal);
  // Continuation past occurrence 17 toward ~27.
  EXPECT_GT(r.release_through_index, 17);
  EXPECT_EQ(
    r.to_occurrence.route_occurrence,
    static_cast<uint32_t>(r.release_through_index));
}

// ---- F4: commit staleness scoped to the proposal's own conflicts ----

namespace
{
TimedPath xy_path(
  const std::string & id, double x0, double y0, double dx, double dy, int n,
  uint32_t rev = 1)
{
  TimedPath p;
  p.amr_id = id;
  p.fleet_name = "fleet";
  p.task_id = "task-" + id;
  p.path_revision = rev;
  for (int i = 0; i < n; ++i) {
    TimedPoint w;
    w.x = x0 + i * dx;
    w.y = y0 + i * dy;
    w.level_name = "L1";
    p.waypoints.push_back(w);
  }
  return p;
}

ecbs_planner::ProposalQueryInput query_for(const std::string & id)
{
  ProposalQueryInput in;
  in.amr_id = id;
  in.path_revision = 1;
  in.desired_extension_m = 10.0;
  in.distance_origin = "GRANT_TIP";
  return in;
}

std::string commit(
  ProposalEngineState & state, ScheduleCache & cache,
  const ecbs_planner::ProposalQueryResult & r, const std::string & amr,
  uint64_t grant_id)
{
  return apply_commit(
    state, r.proposal_id, grant_id, amr, kGrantCommitted, r.release_through_index,
    r.from_occurrence, r.to_occurrence, 1, r.task_id, &cache);
}

/// amr1 (x axis) crosses amr2 (y axis at x=10); amr3 / amr4 cross far away.
struct StaleFixture
{
  ProposalEngineState state;
  ScheduleCache cache;
  ecbs_planner::ProposalQueryResult a;

  StaleFixture()
  {
    cache.upsert(xy_path("amr1", 0, 0, 1, 0, 20));
    cache.upsert(xy_path("amr2", 10, -10, 0, 1, 20));
    cache.upsert(xy_path("amr3", 0, 100, 1, 0, 20));
    cache.upsert(xy_path("amr4", 10, 90, 0, 1, 20));
    a = query_base_extension(state, cache, query_for("amr1"), 0.5, 0.35);
  }
};
}  // namespace

/// Foreign reserved base → ownership WAIT must name the owner (not waiters=0).
TEST(ProposalEngine, OwnershipWaitNamesForeignBaseOwner)
{
  ProposalEngineState state;
  ScheduleCache cache;
  cache.upsert(xy_path("owner", 0, 0, 1, 0, 20));
  auto q = query_for("owner");
  q.amr_id = "owner";
  q.desired_extension_m = 10.0;
  const auto go = query_base_extension(state, cache, q, 0.5, 0.35);
  ASSERT_EQ(go.status, kStatusFreeProposal);
  ASSERT_EQ(commit(state, cache, go, "owner", 1), "ACCEPTED");

  // Challenger crosses the owner's reserved base.
  cache.upsert(xy_path("challenger", 5, -5, 0, 1, 20));
  auto qc = query_for("challenger");
  qc.amr_id = "challenger";
  qc.desired_extension_m = 15.0;
  const auto w = query_base_extension(state, cache, qc, 0.5, 0.35);
  if (w.status == kStatusWait) {
    ASSERT_FALSE(w.waiting_for.empty()) << "ownership must name blocker";
    bool named = false;
    for (const auto & d : w.waiting_for) {
      named = named || d.amr_id == "owner";
    }
    EXPECT_TRUE(named);
    EXPECT_EQ(w.wait_reason, "ownership");
  }
}

/// Mutual wait at hold tips: group scheduler picks one GO (priority), other WAIT.
TEST(ProposalEngine, MutualWaitAtHoldTipsSchedulesByPriority)
{
  ProposalEngineState state;
  ScheduleCache cache;
  cache.upsert(xy_path("amr1", 0, 0, 1, 0, 20));
  cache.upsert(xy_path("amr2", 10, -10, 0, 1, 20));
  refresh_conflicts_from_cache(state, cache, 0.5, 0.35);
  ASSERT_FALSE(state.conflicts.all().empty());

  state.priorities["amr1"].robot_id = "amr1";
  state.priorities["amr1"].task_id = "task-amr1";
  state.priorities["amr1"].task_priority = "low";
  state.priorities["amr2"].robot_id = "amr2";
  state.priorities["amr2"].task_id = "task-amr2";
  state.priorities["amr2"].task_priority = "high";

  auto q1 = query_for("amr1");
  q1.desired_extension_m = 20.0;
  auto a1 = query_base_extension(state, cache, q1, 0.5, 0.35);
  ASSERT_EQ(a1.status, kStatusFreeProposal);
  ASSERT_EQ(commit(state, cache, a1, "amr1", 1), "ACCEPTED");

  auto q2 = query_for("amr2");
  q2.desired_extension_m = 20.0;
  auto a2 = query_base_extension(state, cache, q2, 0.5, 0.35);
  ASSERT_EQ(a2.status, kStatusFreeProposal);
  ASSERT_EQ(commit(state, cache, a2, "amr2", 2), "ACCEPTED");

  q1.current_grant_through_index = a1.release_through_index;
  q2.current_grant_through_index = a2.release_through_index;
  // Re-apply priorities after approach queries (task_id touch can clear them).
  state.priorities["amr1"].task_priority = "low";
  state.priorities["amr2"].task_priority = "high";
  const auto r1 = query_base_extension(state, cache, q1, 0.5, 0.35);
  const auto r2 = query_base_extension(state, cache, q2, 0.5, 0.35);

  const bool a1_go = r1.status == kStatusFreeProposal;
  const bool a2_go = r2.status == kStatusFreeProposal;
  EXPECT_TRUE(a1_go || a2_go) << "scheduler must break mutual WAIT"
    << " r1=" << r1.status << " " << r1.message
    << " r2=" << r2.status << " " << r2.message;
  EXPECT_FALSE(a1_go && a2_go) << "only one exclusive passage";
  if (a2_go) {
    EXPECT_GT(r2.release_through_index, a2.release_through_index);
    EXPECT_EQ(r1.status, kStatusWait);
  } else {
    EXPECT_EQ(r2.status, kStatusWait);
    ADD_FAILURE() << "expected high-priority amr2 to win exclusive passage";
  }
}

TEST(ProposalStale, UnrelatedPathChangeDoesNotStaleCommit)
{
  StaleFixture f;
  ASSERT_EQ(f.a.status, kStatusFreeProposal);
  ASSERT_FALSE(f.state.pending_proposals.at(f.a.proposal_id).conflict_versions.empty());
  const auto route_v = f.state.route_version;
  f.cache.upsert(xy_path("amr3", 0, 200, 1, 0, 20));  // unrelated robot replans
  refresh_conflicts_from_cache(f.state, f.cache, 0.5, 0.35);
  ASSERT_NE(f.state.route_version, route_v) << "global counter still bumps";
  EXPECT_EQ(commit(f.state, f.cache, f.a, "amr1", 1), "ACCEPTED");
}

TEST(ProposalStale, ParticipantRouteChangeStalesCommit)
{
  StaleFixture f;
  ASSERT_EQ(f.a.status, kStatusFreeProposal);
  f.cache.upsert(xy_path("amr2", 10, -20, 0, 1, 30));  // participant B replans
  refresh_conflicts_from_cache(f.state, f.cache, 0.5, 0.35);
  EXPECT_EQ(commit(f.state, f.cache, f.a, "amr1", 1), "REJECTED_STALE");
}

TEST(ProposalStale, ParticipantRouteChangeStalesEvenWithoutRefresh)
{
  StaleFixture f;
  ASSERT_EQ(f.a.status, kStatusFreeProposal);
  f.cache.upsert(xy_path("amr2", 10, -20, 0, 1, 30));
  EXPECT_EQ(commit(f.state, f.cache, f.a, "amr1", 1), "REJECTED_STALE");
}

TEST(ProposalStale, UnrelatedClearanceDoesNotStaleCommit)
{
  StaleFixture f;
  ASSERT_EQ(f.a.status, kStatusFreeProposal);
  // Make the far group's conflict known, then clear it.
  refresh_conflicts_from_cache(f.state, f.cache, 0.5, 0.35);
  std::string far_id;
  for (const auto & r : f.state.conflicts.all()) {
    if (r.participant("amr3") && r.participant("amr4")) {
      far_id = r.conflict_id;
    }
  }
  ASSERT_FALSE(far_id.empty());
  const auto occ = f.state.occupancy_version;
  ASSERT_TRUE(apply_clearance(f.state, "amr4", far_id, true));
  ASSERT_NE(f.state.occupancy_version, occ);
  EXPECT_EQ(commit(f.state, f.cache, f.a, "amr1", 1), "ACCEPTED");
}

/// A rolling prefix erase (robot moved on) is not a route change; a real
/// geometry or revision change is.
TEST(ProposalStale, RouteStampIgnoresPrefixEraseButNotReplan)
{
  ScheduleCache cache;
  cache.upsert(xy_path("amr1", 0, 0, 1, 0, 20));
  const auto stamp = ecbs_planner::make_route_stamp(*cache.get("amr1"));
  ASSERT_TRUE(cache.erase_before_pose("amr1", 3.0, 0.0));
  EXPECT_TRUE(ecbs_planner::route_stamp_matches(stamp, *cache.get("amr1")));

  auto stamped = [](TimedPath p) {
      ecbs_planner::ensure_route_occurrences(p);
      return p;
    };
  auto moved = xy_path("amr1", 0, 0, 1, 0, 20);
  moved.waypoints[15].y = 4.0;
  EXPECT_FALSE(ecbs_planner::route_stamp_matches(stamp, stamped(moved)));
  EXPECT_FALSE(
    ecbs_planner::route_stamp_matches(stamp, stamped(xy_path("amr1", 0, 0, 1, 0, 20, 2))));
  EXPECT_FALSE(
    ecbs_planner::route_stamp_matches(stamp, stamped(xy_path("amr1", 0, 0, 1, 0, 25))));
  EXPECT_TRUE(
    ecbs_planner::route_stamp_matches(stamp, stamped(xy_path("amr1", 0, 0, 1, 0, 20))));
}

TEST(ProposalStale, ConflictRecordChangeStalesCommit)
{
  StaleFixture f;
  ASSERT_EQ(f.a.status, kStatusFreeProposal);
  const auto cid = f.state.pending_proposals.at(f.a.proposal_id)
    .conflict_versions.begin()->first;
  ASSERT_TRUE(f.state.conflicts.mark_physically_cleared(cid));
  EXPECT_EQ(commit(f.state, f.cache, f.a, "amr1", 1), "REJECTED_STALE");
}

TEST(ProposalStale, UnresolvedAuthorityStalesCommit)
{
  StaleFixture f;
  ASSERT_EQ(f.a.status, kStatusFreeProposal);
  f.state.unresolved_authority["amr1"] = "x";
  EXPECT_EQ(commit(f.state, f.cache, f.a, "amr1", 1), "REJECTED_STALE");
}

TEST(ProposalStale, StaleProposalIsNotReoffered)
{
  StaleFixture f;
  ASSERT_EQ(f.a.status, kStatusFreeProposal);
  f.cache.upsert(xy_path("amr2", 10, -20, 0, 1, 30));
  const auto again = query_base_extension(f.state, f.cache, query_for("amr1"), 0.5, 0.35);
  if (again.status == kStatusFreeProposal) {
    EXPECT_NE(again.proposal_id, f.a.proposal_id);
    EXPECT_EQ(commit(f.state, f.cache, again, "amr1", 2), "ACCEPTED");
  }
}

TEST(ProposalStale, ConflictVersionStableAfterOneSideCommits)
{
  StaleFixture f;
  ASSERT_EQ(f.a.status, kStatusFreeProposal);
  ASSERT_EQ(commit(f.state, f.cache, f.a, "amr1", 1), "ACCEPTED");
  refresh_conflicts_from_cache(f.state, f.cache, 0.5, 0.35);
  const auto before = f.state.conflicts.all();
  refresh_conflicts_from_cache(f.state, f.cache, 0.5, 0.35);
  refresh_conflicts_from_cache(f.state, f.cache, 0.5, 0.35);
  const auto after = f.state.conflicts.all();
  ASSERT_EQ(before.size(), after.size());
  for (std::size_t i = 0; i < before.size(); ++i) {
    EXPECT_EQ(before[i].record_version, after[i].record_version) << before[i].conflict_id;
  }
}

// ---- F5: fairness counters only for real losers ----

namespace
{
using ecbs_planner::PassageAction;

PassageAction fairness_action(const std::string & amr, int from, int to)
{
  PassageAction a;
  a.action_id = "pass:" + amr + ":" + std::to_string(from) + ":" + std::to_string(to);
  a.amr_id = amr;
  a.from_index = from;
  a.to_index = to;
  a.from = make_occurrence(1, static_cast<uint32_t>(from), 0, 0);
  a.to = make_occurrence(1, static_cast<uint32_t>(to), 0, 0);
  a.safe_hold_verified = true;
  return a;
}

ecbs_planner::ConflictRecord star_conflict(const std::string & a, const std::string & b)
{
  ecbs_planner::ConflictRecord r;
  ecbs_planner::ParticipantOverlap pa;
  pa.amr_id = a;
  ecbs_planner::ParticipantOverlap pb;
  pb.amr_id = b;
  r.participants = {pa, pb};
  r.conflict_id = ecbs_planner::make_overlap_id(a, b, 1, 8, 1, 8);
  return r;
}

/// amr1 commits [8,12]. amr2 crosses it (waiting); amr3 crosses it but is not
/// waiting; amr4 waits but is on a parallel far lane (compatible).
struct FairnessFixture
{
  ProposalEngineState state;
  ScheduleCache cache;

  explicit FairnessFixture(const TimedPath & winner_path, const TimedPath & loser_path)
  {
    cache.upsert(winner_path);
    cache.upsert(loser_path);
    cache.upsert(xy_path("amr3", 10, -10, 0, 1, 20));
    cache.upsert(xy_path("amr4", 0, 50, 1, 0, 20));
    for (const auto & m : {"amr2", "amr3", "amr4"}) {
      state.conflicts.upsert(star_conflict("amr1", m));
    }
    state.groups.rebuild_from_conflicts(state.conflicts, 1, 0, 1, 1);
    auto * g = state.groups.group_for_amr("amr1");
    g->actions = {
      fairness_action("amr1", 8, 12), fairness_action("amr2", 5, 12),
      fairness_action("amr3", 5, 12), fairness_action("amr4", 5, 12)};
    for (const auto & a : g->actions) {
      g->fairness[a.amr_id];
    }
    state.groups.set_pending_extension("amr2", true);
    state.groups.set_pending_extension("amr4", true);

    ecbs_planner::PendingProposal p;
    p.proposal_id = 1;
    p.amr_id = "amr1";
    p.task_id = "task-amr1";
    p.path_revision = 1;
    p.action = g->actions[0];
    p.from_occurrence = p.action.from;
    p.to_occurrence = p.action.to;
    state.pending_proposals[1] = p;
  }

  std::string commit_winner()
  {
    return apply_commit(
      state, 1, 100, "amr1", kGrantCommitted, 12,
      make_occurrence(1, 8, 0, 0), make_occurrence(1, 12, 0, 0), 1, "task-amr1", &cache);
  }

  int bypass(const std::string & amr) const
  {
    return state.groups.group_for_amr(amr)->fairness.at(amr).bypass_count;
  }
};
}  // namespace

TEST(ProposalFairness, ExclusiveCrossingIncrementsOnlyWaitingLoser)
{
  FairnessFixture f(xy_path("amr1", 0, 0, 1, 0, 20), xy_path("amr2", 10, -10, 0, 1, 20));
  ASSERT_EQ(f.commit_winner(), "ACCEPTED");
  EXPECT_EQ(f.bypass("amr2"), 1) << "waiting + incompatible";
  EXPECT_EQ(f.bypass("amr3"), 0) << "incompatible but not waiting";
  EXPECT_EQ(f.bypass("amr4"), 0) << "waiting but compatible";
  EXPECT_EQ(f.bypass("amr1"), 0);
}

TEST(ProposalFairness, SameDirectionFollowerCommitIncrementsNobody)
{
  // amr2 follows amr1 on the same line, behind it: compatible with the winner.
  FairnessFixture f(xy_path("amr1", 0, 0, 1, 0, 20), xy_path("amr2", 0, 0, 1, 0, 20));
  f.state.groups.group_for_amr("amr1")->actions[1] = fairness_action("amr2", 0, 4);
  ASSERT_EQ(f.commit_winner(), "ACCEPTED");
  EXPECT_EQ(f.bypass("amr2"), 0);
  EXPECT_EQ(f.bypass("amr3"), 0);
  EXPECT_EQ(f.bypass("amr4"), 0);
}

TEST(ProposalFairness, WinnerBypassResetsWhenNobodyQualifies)
{
  FairnessFixture f(xy_path("amr1", 0, 0, 1, 0, 20), xy_path("amr2", 0, 70, 1, 0, 20));
  auto * g = f.state.groups.group_for_amr("amr1");
  g->fairness["amr1"].bypass_count = 2;
  const auto policy = g->policy_priority_version;
  ASSERT_EQ(f.commit_winner(), "ACCEPTED");
  EXPECT_EQ(f.bypass("amr1"), 0);
  EXPECT_EQ(f.state.groups.group_for_amr("amr1")->policy_priority_version, policy)
    << "reset-only path does not bump policy version";
}

// ---- F7: register validated against conflicts, persisted as mandatory ----

namespace
{
using ecbs_planner::msg::RegisterUpdate;

RegisterUpdate wait_update(
  const std::string & waiter, const std::string & blocker, double x, double y)
{
  RegisterUpdate u;
  u.amr_id = waiter;
  u.task_id = "task-" + waiter;
  u.path_revision = 1;
  ecbs_planner::msg::WaitDependency d;
  d.amr_id = blocker;
  d.wait_point.x = x;
  d.wait_point.y = y;
  d.wait_point.index = -1;
  d.wait_point.wait_sec = 5.0;
  u.waiting_for.push_back(d);
  return u;
}

/// amr1 (x axis) crosses amr2 (y axis) at x=10: amr1 entry occ 9, exit 11, wait 7.
/// amr3 is far from both.
struct RegisterFixture
{
  ProposalEngineState state;
  ScheduleCache cache;
  ecbs_planner::EcbsSolver solver;
  ecbs_planner::srv::EvaluatePath::Request req;
  ecbs_planner::srv::EvaluatePath::Response res;

  RegisterFixture()
  {
    cache.upsert(xy_path("amr1", 0, 0, 1, 0, 20));
    cache.upsert(xy_path("amr2", 10, -10, 0, 1, 20));
    cache.upsert(xy_path("amr3", 0, 100, 1, 0, 20));
    refresh_conflicts_from_cache(state, cache, 0.5, 0.35);
    req.mode = "register";
    req.fleet_name = "fleet";
  }

  void run(const ecbs_planner::NavGraphJunctions * junctions = nullptr)
  {
    res = decltype(res)();
    ecbs_planner::evaluate_path(
      req, res, cache, solver, nullptr, junctions, false, &state.conflicts);
  }

  bool has_waits(const std::string & amr) const
  {
    for (const auto & w : cache.get(amr)->waypoints) {
      if (w.wait_duration_sec > 1e-9 || !w.waiting_for.empty()) {
        return true;
      }
    }
    return false;
  }

  const ecbs_planner::ConflictRecord & only_record() const
  {
    static ecbs_planner::ConflictRecord rec;
    const auto all = state.conflicts.all();
    EXPECT_EQ(all.size(), 1u);
    rec = all.front();
    return rec;
  }
};
}  // namespace

TEST(RegisterValidation, ValidWaitIsPersistedAsMandatory)
{
  RegisterFixture f;
  ASSERT_EQ(f.state.conflicts.size(), 1u);
  const auto v0 = f.only_record().record_version;
  f.req.updates = {wait_update("amr1", "amr2", 7, 0)};
  f.run();
  ASSERT_TRUE(f.res.success) << f.res.message;
  EXPECT_TRUE(f.has_waits("amr1"));
  EXPECT_EQ(ecbs_planner::persist_register_dependencies(f.req, f.cache, f.state.conflicts), 1u);
  refresh_conflicts_from_cache(f.state, f.cache, 0.5, 0.35);
  const auto rec = f.only_record();
  EXPECT_TRUE(rec.mandatory);
  EXPECT_EQ(rec.waiter_amr, "amr1");
  EXPECT_EQ(rec.blocker_amr, "amr2");
  EXPECT_GT(rec.record_version, v0);
  ASSERT_EQ(f.state.conflicts.outbound_deps("amr1").size(), 1u);
}

TEST(RegisterValidation, WaitInsideOverlapRejectedCacheUnchanged)
{
  RegisterFixture f;
  const auto before = f.cache.get("amr1");
  f.req.updates = {wait_update("amr1", "amr2", 10, 0)};  // occ 10 in [9, 11]
  f.run();
  EXPECT_FALSE(f.res.success);
  EXPECT_NE(f.res.message.find("inside overlap"), std::string::npos) << f.res.message;
  EXPECT_FALSE(f.has_waits("amr1"));
  EXPECT_EQ(f.cache.get("amr1")->waypoints.size(), before->waypoints.size());
}

TEST(RegisterValidation, WaitAfterOverlapRejected)
{
  RegisterFixture f;
  f.req.updates = {wait_update("amr1", "amr2", 15, 0)};
  f.run();
  EXPECT_FALSE(f.res.success);
  EXPECT_FALSE(f.has_waits("amr1"));
}

TEST(RegisterValidation, UnrelatedBlockerRejected)
{
  RegisterFixture f;
  f.req.updates = {wait_update("amr1", "amr3", 7, 0)};
  f.run();
  EXPECT_FALSE(f.res.success);
  EXPECT_NE(f.res.message.find("amr3"), std::string::npos) << f.res.message;
  EXPECT_FALSE(f.has_waits("amr1"));
}

TEST(RegisterValidation, SecondInvalidUpdateLeavesFirstUnapplied)
{
  RegisterFixture f;
  f.req.updates = {
    wait_update("amr1", "amr2", 7, 0),     // valid
    wait_update("amr2", "amr1", 10, 0)};   // amr2 wait on the crossing itself
  f.run();
  EXPECT_FALSE(f.res.success);
  EXPECT_FALSE(f.has_waits("amr1")) << "all-or-nothing";
  EXPECT_FALSE(f.has_waits("amr2"));
  EXPECT_FALSE(f.cache.has_hold("amr1", "amr2"));
}

TEST(RegisterValidation, WaitBehindRobotRejected)
{
  RegisterFixture f;
  ASSERT_TRUE(f.cache.erase_before_pose("amr1", 5.0, 0.0));  // robot at x=5
  f.cache.set_pose("amr1", 5.0, 0.0);
  f.req.updates = {wait_update("amr1", "amr2", 2, 0)};
  f.run();
  EXPECT_FALSE(f.res.success);
  EXPECT_NE(f.res.message.find("behind the robot"), std::string::npos) << f.res.message;
  EXPECT_FALSE(f.has_waits("amr1"));
}

TEST(RegisterValidation, WaitOnJunctionCentreRejected)
{
  RegisterFixture f;
  ecbs_planner::NavGraphJunctions junctions;
  // T junction (degree 3) centred on amr1's route at (7, 0).
  junctions.set_from_edges(
    {{7, 0}, {0, 0}, {7, 5}, {7, -5}}, {{0, 1}, {0, 2}, {0, 3}});
  f.req.updates = {wait_update("amr1", "amr2", 7, 0)};
  f.run(&junctions);
  EXPECT_FALSE(f.res.success);
  EXPECT_NE(f.res.message.find("junction centre"), std::string::npos) << f.res.message;
  EXPECT_FALSE(f.has_waits("amr1"));
}
