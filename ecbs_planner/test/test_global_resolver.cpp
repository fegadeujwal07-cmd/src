#include <gtest/gtest.h>

#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include "ecbs_planner/global_resolver.hpp"

using ecbs_planner::GlobalResolver;
using ecbs_planner::OverlapKind;
using ecbs_planner::ResolverConfig;
using ecbs_planner::ResolverDecision;
using ecbs_planner::ResolverFacts;
using ecbs_planner::ResolverRoutePoint;
using ecbs_planner::ResolverStatus;

namespace
{

using Pts = std::vector<ResolverRoutePoint>;

/// Straight leg from (x0,y0) to (x1,y1) with waypoints every `step` metres.
void leg(Pts & out, double x0, double y0, double x1, double y1, double step = 2.0)
{
  const double L = std::hypot(x1 - x0, y1 - y0);
  const int n = std::max(1, static_cast<int>(std::round(L / step)));
  const int start = out.empty() ? 0 : 1;
  for (int i = start; i <= n; ++i) {
    const double t = static_cast<double>(i) / n;
    out.push_back({x0 + t * (x1 - x0), y0 + t * (y1 - y0)});
  }
}

Pts nodes(std::initializer_list<std::pair<double, double>> pts)
{
  Pts out;
  for (const auto & p : pts) {
    out.push_back({p.first, p.second});
  }
  return out;
}

ResolverConfig cfg()
{
  ResolverConfig c;
  c.conflict_distance_m = 1.0;
  c.safety_distance_m = 2.0;
  return c;
}

std::vector<ResolverDecision> between(
  const GlobalResolver & r, const std::string & a, const std::string & b)
{
  std::vector<ResolverDecision> out;
  for (const auto & d : r.decisions_for(a)) {
    if (d.involves(b)) {
      out.push_back(d);
    }
  }
  return out;
}

}  // namespace

// User scenario: amr1 31,32,33,34,35,6,5,4,3,2,1 granted till 33; amr2 1..10.
// amr2 starts inside the overlap (node 1) → amr2 passes first; amr1 holds on
// its approach (off amr2's path) and is released once amr2 clears node 6.
TEST(GlobalResolver, UserScenarioAmr2PassesAmr1HoldsOnApproach)
{
  GlobalResolver r(cfg());
  // Corridor nodes 1..10 on y=0, x = 0,3,...,27. Node 6 = (15,0).
  const auto amr1 = nodes(
    {{15, 20}, {15, 16}, {15, 12}, {15, 8}, {15, 4},      // 31..35
      {15, 0}, {12, 0}, {9, 0}, {6, 0}, {3, 0}, {0, 0}});  // 6..1
  Pts amr2;
  for (int i = 0; i < 10; ++i) {
    amr2.push_back({3.0 * i, 0.0});
  }
  r.update_pose("amr1", 15, 20);
  ASSERT_TRUE(r.upsert_route("amr1", "t1", amr1));
  // amr1 already holds base till node 33 (index 2).
  auto a1 = r.query("amr1", -1, 10.0);
  ASSERT_EQ(a1.status, ResolverStatus::Free);
  ASSERT_EQ(r.commit("amr1", a1.proposal_id, 2, "COMMITTED"), "ACCEPTED");
  r.update_pose("amr1", 15, 19);

  r.update_pose("amr2", 0, 0);
  ASSERT_TRUE(r.upsert_route("amr2", "t2", amr2));
  const auto ds = between(r, "amr1", "amr2");
  ASSERT_EQ(ds.size(), 1u);
  const auto & d = ds.front();
  EXPECT_EQ(d.kind, OverlapKind::Opposite);
  EXPECT_EQ(d.first, "amr2");
  EXPECT_EQ(d.second, "amr1");
  EXPECT_TRUE(d.forced) << d.rule;
  // Furthest safe hold before the corridor: node 35 (index 4), 4 m off it.
  EXPECT_EQ(d.wait_index, 4) << d.rule;
  EXPECT_GE(d.wait_index, 2);  // never behind the granted base (33)

  // amr2 is free to go now — no waiting on amr1.
  const auto a2 = r.query("amr2", -1, 10.0);
  ASSERT_EQ(a2.status, ResolverStatus::Free);
  ASSERT_EQ(r.commit("amr2", a2.proposal_id, a2.through_index, "COMMITTED"), "ACCEPTED");

  // amr1 may approach its wait point, never beyond it.
  auto q1 = r.query("amr1", 2, 10.0);
  ASSERT_EQ(q1.status, ResolverStatus::Free);
  EXPECT_EQ(q1.through_index, 4);
  ASSERT_EQ(r.commit("amr1", q1.proposal_id, 4, "COMMITTED"), "ACCEPTED");
  r.update_pose("amr1", 15, 4);
  q1 = r.query("amr1", 4, 10.0);
  ASSERT_EQ(q1.status, ResolverStatus::Wait);
  ASSERT_FALSE(q1.waiting_for.empty());
  EXPECT_EQ(q1.waiting_for.front().amr_id, "amr2");
  EXPECT_EQ(static_cast<int>(*q1.waiting_for.front().wait_point.index), 4);

  // amr2 drives through; still inside → amr1 keeps waiting.
  r.update_pose("amr2", 6, 0);
  EXPECT_EQ(r.query("amr1", 4, 10.0).status, ResolverStatus::Wait);
  auto a2b = r.query("amr2", a2.through_index, 10.0);
  ASSERT_EQ(a2b.status, ResolverStatus::Free);
  ASSERT_EQ(r.commit("amr2", a2b.proposal_id, a2b.through_index, "COMMITTED"), "ACCEPTED");
  r.update_pose("amr2", 12, 0);
  EXPECT_EQ(r.query("amr1", 4, 10.0).status, ResolverStatus::Wait);
  auto a2c = r.query("amr2", a2b.through_index, 10.0);
  ASSERT_EQ(a2c.status, ResolverStatus::Free);
  ASSERT_EQ(r.commit("amr2", a2c.proposal_id, a2c.through_index, "COMMITTED"), "ACCEPTED");

  // amr2 passes node 6 + safety → amr1 released (event names amr1).
  const auto ev = r.update_pose("amr2", 18.5, 0);
  ASSERT_EQ(ev.size(), 1u);
  EXPECT_EQ(ev.front().amr_id, "amr1");
  const auto rel = r.query("amr1", 4, 10.0);
  ASSERT_EQ(rel.status, ResolverStatus::Free);
  EXPECT_GT(rel.through_index, 4);
  for (const auto & line : r.take_log()) {
    std::cout << line << std::endl;
  }
}

// The decision must not flip while robots move (stable conflict identity).
TEST(GlobalResolver, DecisionIsStableUnderProgress)
{
  GlobalResolver r(cfg());
  Pts a;
  leg(a, 15, 20, 15, 0);
  leg(a, 15, 0, 0, 0);
  Pts b;
  leg(b, 0, 0, 27, 0);
  r.update_pose("amr1", 15, 20);
  r.upsert_route("amr1", "t1", a);
  r.update_pose("amr2", 0, 0);
  r.upsert_route("amr2", "t2", b);
  const auto before = between(r, "amr1", "amr2");
  ASSERT_EQ(before.size(), 1u);
  int32_t tip = -1;
  for (double x = 0.5; x < 14.0; x += 0.5) {
    const auto q = r.query("amr2", tip, 10.0);
    if (q.status == ResolverStatus::Free) {
      ASSERT_EQ(r.commit("amr2", q.proposal_id, q.through_index, "COMMITTED"), "ACCEPTED");
      tip = q.through_index;
    }
    r.update_pose("amr2", x, 0);
    const auto now = between(r, "amr1", "amr2");
    ASSERT_EQ(now.size(), 1u);
    EXPECT_EQ(now.front().conflict_id, before.front().conflict_id);
    EXPECT_EQ(now.front().first, "amr2");
    EXPECT_NE(r.query("amr2", tip, 10.0).status, ResolverStatus::Recovery);
  }
  // Re-seeding the identical route is a no-op (no re-decision).
  EXPECT_FALSE(r.upsert_route("amr2", "t2", b));
  EXPECT_EQ(between(r, "amr1", "amr2").front().first, "amr2");
}

// Same direction: robot ahead leads; follower trails at safety distance.
TEST(GlobalResolver, SameDirectionFollowerTrails)
{
  GlobalResolver r(cfg());
  Pts lead;
  leg(lead, 10, 0, 40, 0);
  Pts follow;
  leg(follow, 0, 0, 40, 0);
  r.update_pose("lead", 10, 0);
  r.upsert_route("lead", "a", lead);
  r.update_pose("follow", 0, 0);
  r.upsert_route("follow", "b", follow);
  const auto ds = between(r, "lead", "follow");
  ASSERT_EQ(ds.size(), 1u);
  EXPECT_EQ(ds.front().kind, OverlapKind::SameDirection);
  EXPECT_EQ(ds.front().first, "lead");
  // Follower holds before the leader's tail.
  auto qf = r.query("follow", -1, 20.0);
  ASSERT_EQ(qf.status, ResolverStatus::Free);
  const auto tip_pt = r.waypoint("follow", qf.through_index);
  ASSERT_TRUE(tip_pt.has_value());
  EXPECT_LE(tip_pt->x, 10.0 - 0.5 - 2.0 + 1e-6);
  // Leader moves on → follower released and trails, never into its base.
  auto ql = r.query("lead", -1, 10.0);
  ASSERT_EQ(r.commit("lead", ql.proposal_id, ql.through_index, "COMMITTED"), "ACCEPTED");
  r.update_pose("lead", 16, 0);
  ASSERT_EQ(r.commit("follow", qf.proposal_id, qf.through_index, "COMMITTED"), "ACCEPTED");
  r.update_pose("follow", tip_pt->x, 0);
  const auto q2 = r.query("follow", qf.through_index, 20.0);
  ASSERT_EQ(q2.status, ResolverStatus::Free);
  const auto tip2 = r.waypoint("follow", q2.through_index);
  ASSERT_TRUE(tip2.has_value());
  EXPECT_LE(tip2->x, 16.0 - 0.5 - 1.0);
}

// Normal crossing, equal distances: zone_manager priority rule decides.
TEST(GlobalResolver, CrossingDecidedByPriority)
{
  ResolverConfig c = cfg();
  GlobalResolver r(c);
  Pts a;
  leg(a, -10, 0, 10, 0);
  Pts b;
  leg(b, 0, -10, 0, 10);
  ResolverFacts hi;
  hi.priority_rank = 2;
  r.set_facts("amr2", hi);
  r.update_pose("amr1", -10, 0);
  r.upsert_route("amr1", "a", a);
  r.update_pose("amr2", 0, -10);
  r.upsert_route("amr2", "b", b);
  const auto ds = between(r, "amr1", "amr2");
  ASSERT_EQ(ds.size(), 1u);
  EXPECT_EQ(ds.front().kind, OverlapKind::Crossing);
  EXPECT_EQ(ds.front().first, "amr2") << ds.front().rule;
  EXPECT_EQ(ds.front().rule.rfind("priority", 0), 0u) << ds.front().rule;
}

// Free pass: the near robot clears the crossing before the far (high
// priority) robot even reaches its wait point → near robot goes, nobody stops.
TEST(GlobalResolver, FreePassBeatsPriorityWhenNobodyStops)
{
  GlobalResolver r(cfg());
  Pts a;
  leg(a, -6, 0, 10, 0);
  Pts b;
  leg(b, 0, -40, 0, 10);
  ResolverFacts hi;
  hi.priority_rank = 2;
  r.set_facts("amr2", hi);
  r.update_pose("amr1", -6, 0);
  r.upsert_route("amr1", "a", a);
  r.update_pose("amr2", 0, -40);
  r.upsert_route("amr2", "b", b);
  const auto ds = between(r, "amr1", "amr2");
  ASSERT_EQ(ds.size(), 1u);
  EXPECT_EQ(ds.front().first, "amr1") << ds.front().rule;
  EXPECT_EQ(ds.front().rule.rfind("free_pass", 0), 0u) << ds.front().rule;
}

// Who-blocks-whom: a robot ending its route inside the overlap goes second,
// even with higher priority (it would park on the other's path).
TEST(GlobalResolver, ParkingInsideOverlapGoesSecond)
{
  GlobalResolver r(cfg());
  Pts a;  // amr1 comes down at x=30 then west, ends at (10,0) inside corridor
  leg(a, 30, 6, 30, 0);
  leg(a, 30, 0, 10, 0);
  Pts b;  // amr2 drives the corridor west→east from x=0
  leg(b, 0, 0, 40, 0);
  ResolverFacts hi;
  hi.priority_rank = 2;
  r.set_facts("amr1", hi);
  r.update_pose("amr1", 30, 6);
  r.upsert_route("amr1", "a", a);
  r.update_pose("amr2", 0, 0);
  r.upsert_route("amr2", "b", b);
  const auto ds = between(r, "amr1", "amr2");
  ASSERT_EQ(ds.size(), 1u);
  EXPECT_EQ(ds.front().kind, OverlapKind::Opposite);
  EXPECT_EQ(ds.front().first, "amr2") << ds.front().rule;
  EXPECT_EQ(ds.front().rule.rfind("blocking", 0), 0u) << ds.front().rule;
}

// Group: two robots in a corridor (same way) and one entering against them
// from a branch. The branch robot waits for BOTH, released one by one.
TEST(GlobalResolver, GroupBranchRobotWaitsForWholeConvoy)
{
  GlobalResolver r(cfg());
  Pts a;
  leg(a, 6, 0, 40, 0);   // A ahead
  Pts b;
  leg(b, 0, 0, 40, 0);   // B behind A
  Pts c;                 // C enters at x=30 from the north, heads west
  leg(c, 30, 8, 30, 0);
  leg(c, 30, 0, -10, 0);
  r.update_pose("A", 6, 0);
  r.upsert_route("A", "a", a);
  r.update_pose("B", 0, 0);
  r.upsert_route("B", "b", b);
  r.update_pose("C", 30, 8);
  r.upsert_route("C", "c", c);
  for (const auto & d : between(r, "A", "C")) {
    EXPECT_EQ(d.first, "A") << d.rule;
  }
  for (const auto & d : between(r, "B", "C")) {
    EXPECT_EQ(d.first, "B") << d.rule;
  }
  // C approaches its wait point (off the corridor) and holds there.
  const auto qc = r.query("C", -1, 10.0);
  ASSERT_EQ(qc.status, ResolverStatus::Free);
  ASSERT_GE(qc.conflict_ids.size(), 2u);
  const auto c_hold = r.waypoint("C", qc.through_index);
  ASSERT_TRUE(c_hold.has_value());
  EXPECT_GE(c_hold->y, 1.0 + 2.0 - 1e-6);
  ASSERT_EQ(r.commit("C", qc.proposal_id, qc.through_index, "COMMITTED"), "ACCEPTED");
  r.update_pose("C", c_hold->x, c_hold->y);
  const int32_t c_tip = qc.through_index;
  EXPECT_EQ(r.query("C", c_tip, 10.0).status, ResolverStatus::Wait);

  // Drive A and B east past the junction; C waits until both cleared.
  auto drive = [&](const std::string & id, int32_t & tip, double x) {
      const auto q = r.query(id, tip, 40.0);
      if (q.status == ResolverStatus::Free) {
        EXPECT_EQ(r.commit(id, q.proposal_id, q.through_index, "COMMITTED"), "ACCEPTED");
        tip = q.through_index;
      }
      return r.update_pose(id, x, 0);
    };
  int32_t ta = -1;
  int32_t tb = -1;
  for (double x = 6.5; x <= 36.0; x += 1.0) {
    drive("A", ta, x);
  }
  // A cleared, B has not → C still waiting.
  EXPECT_EQ(r.query("C", c_tip, 10.0).status, ResolverStatus::Wait);
  bool released = false;
  for (double x = 0.5; x <= 36.0; x += 1.0) {
    for (const auto & ev : drive("B", tb, x)) {
      released = released || ev.amr_id == "C";
    }
  }
  EXPECT_TRUE(released);
  const auto go = r.query("C", c_tip, 10.0);
  EXPECT_EQ(go.status, ResolverStatus::Free);
  EXPECT_GT(go.through_index, c_tip);
}

// A proposal issued before a new decision must not be committed past it.
TEST(GlobalResolver, StaleProposalRejectedAfterNewDecision)
{
  GlobalResolver r(cfg());
  Pts a;
  leg(a, 15, 30, 15, 0);
  leg(a, 15, 0, 0, 0);
  r.update_pose("amr1", 15, 30);
  r.upsert_route("amr1", "a", a);
  const auto q = r.query("amr1", -1, 40.0);
  ASSERT_EQ(q.status, ResolverStatus::Free);
  // amr2 appears inside the corridor before the commit lands.
  Pts b;
  leg(b, 0, 0, 27, 0);
  r.update_pose("amr2", 0, 0);
  r.upsert_route("amr2", "b", b);
  ASSERT_EQ(between(r, "amr1", "amr2").front().second, "amr1");
  EXPECT_EQ(r.commit("amr1", q.proposal_id, q.through_index, "COMMITTED"), "REJECTED_STALE");
  const auto q2 = r.query("amr1", -1, 40.0);
  ASSERT_EQ(q2.status, ResolverStatus::Free);
  const auto tip = r.waypoint("amr1", q2.through_index);
  ASSERT_TRUE(tip.has_value());
  EXPECT_GE(tip->y, 1.0 + 2.0 - 1e-6);  // stops ≥ safety before the corridor
}

// Idle / single waypoint routes: nothing to release, no reseed loop.
TEST(GlobalResolver, SingleWaypointRouteIsComplete)
{
  GlobalResolver r(cfg());
  r.upsert_route("amr3", "idle", nodes({{67.25, 42.9}}));
  EXPECT_EQ(r.query("amr3", -1, 10.0).status, ResolverStatus::RouteComplete);
  EXPECT_EQ(r.query("ghost", -1, 10.0).status, ResolverStatus::UnknownRoute);
}

// A standing robot on the route is an obstacle: hold safety before it.
TEST(GlobalResolver, StandingRobotOnRouteHolds)
{
  GlobalResolver r(cfg());
  r.update_pose("amr9", 20, 0);  // unknown robot → standing obstacle
  Pts a;
  leg(a, 0, 0, 40, 0);
  r.update_pose("amr1", 0, 0);
  r.upsert_route("amr1", "a", a);
  int32_t tip = -1;
  for (int i = 0; i < 6; ++i) {
    const auto q = r.query("amr1", tip, 10.0);
    if (q.status != ResolverStatus::Free) {
      ASSERT_EQ(q.status, ResolverStatus::Wait);
      ASSERT_FALSE(q.waiting_for.empty());
      EXPECT_EQ(q.waiting_for.front().amr_id, "amr9");
      break;
    }
    ASSERT_EQ(r.commit("amr1", q.proposal_id, q.through_index, "COMMITTED"), "ACCEPTED");
    tip = q.through_index;
    const auto p = r.waypoint("amr1", tip);
    ASSERT_TRUE(p.has_value());
    EXPECT_LE(p->x, 20.0 - 1.0 - 2.0 + 1e-6);
    r.update_pose("amr1", p->x, 0);
  }
}

// A repeated identical leg (loop task) registered under a new session key
// starts fresh: grant and progress of the previous run do not leak.
TEST(GlobalResolver, RepeatedLegNewSessionStartsFresh)
{
  GlobalResolver r(cfg());
  Pts a;
  leg(a, 0, 0, 20, 0);
  r.update_pose("amr1", 0, 0);
  r.upsert_route("amr1", "f|t#req1|0|0", a);
  int32_t tip = -1;
  for (int i = 0; i < 5; ++i) {
    const auto q = r.query("amr1", tip, 10.0);
    if (q.status != ResolverStatus::Free) {
      break;
    }
    ASSERT_EQ(r.commit("amr1", q.proposal_id, q.through_index, "COMMITTED"), "ACCEPTED");
    tip = q.through_index;
  }
  r.update_pose("amr1", 20, 0);
  EXPECT_EQ(r.query("amr1", tip, 10.0).status, ResolverStatus::RouteComplete);
  // Robot back at the start, same geometry, new PathIntent session.
  r.update_pose("amr1", 0, 0);
  EXPECT_TRUE(r.upsert_route("amr1", "f|t#req2|0|0", a));
  const auto q = r.query("amr1", -1, 10.0);
  EXPECT_EQ(q.status, ResolverStatus::Free);
  // Same-session reseed (route recovery) is a no-op.
  EXPECT_FALSE(r.upsert_route("amr1", "f|t#req2|0|0", a));
}

// ---------------------------------------------------------------------------
// RMF handover: ECBS evaluates when RMF hands the waypoints over (EcbsGate
// /from_zone_manager, before ExecutePlan::make); TM's PathIntent only binds.
// ---------------------------------------------------------------------------
namespace
{
Pts user_amr1_sparse()
{
  return nodes(
    {{15, 20}, {15, 16}, {15, 12}, {15, 8}, {15, 4},
      {15, 0}, {12, 0}, {9, 0}, {6, 0}, {3, 0}, {0, 0}});
}
Pts user_amr2()
{
  Pts out;
  for (int i = 0; i < 10; ++i) {
    out.push_back({3.0 * i, 0.0});
  }
  return out;
}
}  // namespace

TEST(GlobalResolverHandover, DecidesOnHandoverIntentBindsKeepingDecision)
{
  GlobalResolver r(cfg());
  r.update_pose("amr1", 15, 20);
  r.update_pose("amr2", 0, 0);
  EXPECT_EQ(r.register_handover("amr1", "f|task1|0|0", user_amr1_sparse()), "decided");
  EXPECT_EQ(r.register_handover("amr2", "f|task2|0|0", user_amr2()), "decided");
  // Decided at handover — no PathIntent exists yet.
  auto ds = between(r, "amr1", "amr2");
  ASSERT_EQ(ds.size(), 1u);
  const auto decided = ds.front();
  EXPECT_EQ(decided.first, "amr2");
  EXPECT_DOUBLE_EQ(decided.wait_x, 15.0);
  EXPECT_DOUBLE_EQ(decided.wait_y, 4.0);

  // PathIntent with denser numbering (every 2 m) for the same task/phase/rev.
  Pts dense;
  leg(dense, 15, 20, 15, 0);
  leg(dense, 15, 0, 0, 0);
  EXPECT_EQ(r.register_intent("amr1", "f|task1|0|0", "7", dense), "rebound");
  EXPECT_EQ(r.register_intent("amr2", "f|task2|0|0", "8", user_amr2()), "rebound");
  ds = between(r, "amr1", "amr2");
  ASSERT_EQ(ds.size(), 1u);
  EXPECT_EQ(ds.front().conflict_id, decided.conflict_id);
  EXPECT_EQ(ds.front().first, "amr2");
  EXPECT_EQ(ds.front().wait_index, 8);  // (15,4) in the PathIntent numbering
  EXPECT_DOUBLE_EQ(ds.front().wait_x, 15.0);
  EXPECT_DOUBLE_EQ(ds.front().wait_y, 4.0);
  // Release on PathIntent numbering stops at the wait point.
  const auto q = r.query("amr1", -1, 30.0, "f|task1|0|0");
  ASSERT_EQ(q.status, ResolverStatus::Free);
  EXPECT_EQ(q.through_index, 8);
  // Same-session reseed is a no-op.
  EXPECT_EQ(r.register_intent("amr1", "f|task1|0|0", "7", dense), "unchanged");
}

TEST(GlobalResolverHandover, StaleHandoverCopyIgnored)
{
  GlobalResolver r(cfg());
  r.update_pose("amr2", 0, 0);
  EXPECT_EQ(r.register_handover("amr2", "f|task2|0|0", user_amr2()), "decided");
  // Re-sent identical handover → unchanged.
  EXPECT_EQ(r.register_handover("amr2", "f|task2|0|0", user_amr2()), "unchanged");
  Pts next;
  leg(next, 27, 0, 27, 20);
  r.update_pose("amr2", 27, 0);
  EXPECT_EQ(r.register_handover("amr2", "f|task3|0|0", next), "decided");
  // Old copy of task2 arriving late (another robot's stale updated_others).
  EXPECT_EQ(r.register_handover("amr2", "f|task2|0|0", user_amr2()), "ignored_stale");
  EXPECT_EQ(r.route_points("amr2").size(), next.size());
}

TEST(GlobalResolverHandover, IntentBeforeHandoverMakesHandoverNoop)
{
  GlobalResolver r(cfg());
  r.update_pose("amr2", 0, 0);
  EXPECT_EQ(r.register_intent("amr2", "f|task2|0|0", "3", user_amr2()), "decided");
  EXPECT_EQ(r.register_handover("amr2", "f|task2|0|0", user_amr2()), "unchanged");
}

TEST(GlobalResolverHandover, IntentGeometryDiffersIsRedecided)
{
  GlobalResolver r(cfg());
  r.update_pose("amr1", 15, 20);
  r.update_pose("amr2", 0, 0);
  r.register_handover("amr1", "f|task1|0|0", user_amr1_sparse());
  r.register_handover("amr2", "f|task2|0|0", user_amr2());
  // Executed PathIntent goes elsewhere (does not enter the corridor).
  Pts other;
  leg(other, 15, 20, 30, 20);
  EXPECT_EQ(r.register_intent("amr1", "f|task1|0|0", "9", other), "decided");
  EXPECT_TRUE(between(r, "amr1", "amr2").empty());
}

TEST(GlobalResolverHandover, OldSessionHeldWhenRmfHandsOverNewRoute)
{
  GlobalResolver r(cfg());
  r.update_pose("amr2", 0, 0);
  r.register_handover("amr2", "f|task2|0|0", user_amr2());
  r.register_intent("amr2", "f|task2|0|0", "1", user_amr2());
  auto q = r.query("amr2", -1, 10.0, "f|task2|0|0");
  ASSERT_EQ(q.status, ResolverStatus::Free);
  ASSERT_EQ(r.commit("amr2", q.proposal_id, q.through_index, "COMMITTED"), "ACCEPTED");
  // Replan: RMF hands over revision 1.
  r.register_handover("amr2", "f|task2|0|1", user_amr2());
  const auto old = r.query("amr2", q.through_index, 10.0, "f|task2|0|0");
  EXPECT_EQ(old.status, ResolverStatus::Wait);
  EXPECT_NE(old.reason.find("superseded"), std::string::npos);
  r.register_intent("amr2", "f|task2|0|1", "2", user_amr2());
  EXPECT_EQ(r.query("amr2", -1, 10.0, "f|task2|0|1").status, ResolverStatus::Free);
}

// Strict mode: resolution happens ONLY on the RMF handover.
TEST(GlobalResolverHandover, StrictPathIntentFirstIsParkedUntilHandover)
{
  ResolverConfig c = cfg();
  c.decide_on_handover_only = true;
  GlobalResolver r(c);
  r.update_pose("amr1", 15, 20);
  r.update_pose("amr2", 0, 0);
  EXPECT_EQ(r.register_handover("amr1", "f|task1|0|0", user_amr1_sparse()), "decided");
  // amr2's PathIntent races ahead of its handover: parked, no decision yet.
  EXPECT_EQ(r.register_intent("amr2", "f|task2|0|0", "4", user_amr2()), "awaiting_handover");
  EXPECT_TRUE(between(r, "amr1", "amr2").empty());
  const auto held = r.query("amr2", -1, 10.0, "f|task2|0|0");
  EXPECT_EQ(held.status, ResolverStatus::Wait);
  EXPECT_NE(held.reason.find("awaiting RMF handover"), std::string::npos);
  // Handover arrives → decided there, parked PathIntent binds.
  EXPECT_EQ(r.register_handover("amr2", "f|task2|0|0", user_amr2()), "decided");
  EXPECT_FALSE(r.has_pending_intent("amr2"));
  const auto ds = between(r, "amr1", "amr2");
  ASSERT_EQ(ds.size(), 1u);
  EXPECT_EQ(ds.front().first, "amr2");
  EXPECT_EQ(r.route_key("amr2"), "f|task2|0|0#4");
  EXPECT_EQ(r.query("amr2", -1, 10.0, "f|task2|0|0").status, ResolverStatus::Free);
}

TEST(GlobalResolverHandover, StrictTimeoutFallbackDecidesOnPathIntent)
{
  ResolverConfig c = cfg();
  c.decide_on_handover_only = true;
  GlobalResolver r(c);
  r.update_pose("amr2", 0, 0);
  EXPECT_EQ(r.register_intent("amr2", "f|task2|0|0", "4", user_amr2()), "awaiting_handover");
  EXPECT_TRUE(r.decide_pending_intent("amr2"));
  EXPECT_EQ(r.query("amr2", -1, 10.0, "f|task2|0|0").status, ResolverStatus::Free);
  // A late handover for the same identity is then a no-op.
  EXPECT_EQ(r.register_handover("amr2", "f|task2|0|0", user_amr2()), "unchanged");
}

TEST(GlobalResolverHandover, StrictIdleIntentIsStandingImmediately)
{
  ResolverConfig c = cfg();
  c.decide_on_handover_only = true;
  GlobalResolver r(c);
  EXPECT_EQ(r.register_intent("amr3", "f|1|0|0", "2", nodes({{67.25, 42.9}})), "decided");
  EXPECT_EQ(r.query("amr3", -1, 10.0, "f|1|0|0").status, ResolverStatus::RouteComplete);
}

// Back-to-back tasks: RMF hands over task B while TM still re-seeds the
// finished task A. A must not park, override B, or trigger re-decisions.
TEST(GlobalResolverHandover, BackToBackOldSessionCannotOverrideNextTask)
{
  ResolverConfig c = cfg();
  c.decide_on_handover_only = true;
  GlobalResolver r(c);
  r.update_pose("amr2", 0, 0);
  ASSERT_EQ(r.register_handover("amr2", "f|taskA|0|0", user_amr2()), "decided");
  ASSERT_EQ(r.register_intent("amr2", "f|taskA|0|0", "3", user_amr2()), "rebound");
  r.update_pose("amr2", 27, 0);  // task A finished at its goal
  Pts b;
  leg(b, 27, 0, 27, 20);
  ASSERT_EQ(r.register_handover("amr2", "f|taskB|0|0", b), "decided");
  // TM re-seeds the finished task A (staged session) — ignored.
  EXPECT_EQ(r.register_intent("amr2", "f|taskA|0|0", "3", user_amr2()), "stale_session");
  EXPECT_FALSE(r.has_pending_intent("amr2"));
  EXPECT_EQ(r.route_points("amr2").size(), b.size());
  // Old session's query holds; the new session binds and runs.
  EXPECT_EQ(r.query("amr2", 9, 10.0, "f|taskA|0|0").status, ResolverStatus::Wait);
  EXPECT_EQ(r.register_intent("amr2", "f|taskB|0|0", "6", b), "rebound");
  EXPECT_EQ(r.query("amr2", -1, 10.0, "f|taskB|0|0").status, ResolverStatus::Free);
}

// ---------------------------------------------------------------------------
// Drive every robot (query → commit → move → pose) until all reach their goal.
// ---------------------------------------------------------------------------
namespace
{
struct SimBot
{
  std::string id;
  Pts route;
  std::vector<double> s;
  int32_t tip{-1};
  double at{0.0};
  bool done() const {return at >= s.back() - 1e-6;}
};

SimBot make_bot(const std::string & id, const Pts & route)
{
  SimBot b{id, route, {0.0}};
  for (std::size_t i = 1; i < route.size(); ++i) {
    b.s.push_back(b.s.back() + std::hypot(
        route[i].x - route[i - 1].x, route[i].y - route[i - 1].y));
  }
  return b;
}

ResolverRoutePoint pos_at(const SimBot & b, double s)
{
  for (std::size_t i = 1; i < b.s.size(); ++i) {
    if (s <= b.s[i] + 1e-9) {
      const double L = b.s[i] - b.s[i - 1];
      const double t = L > 1e-9 ? (s - b.s[i - 1]) / L : 0.0;
      return {b.route[i - 1].x + t * (b.route[i].x - b.route[i - 1].x),
        b.route[i - 1].y + t * (b.route[i].y - b.route[i - 1].y)};
    }
  }
  return b.route.back();
}

/// Returns number of steps until all bots finished, or -1 on standstill.
int drive(GlobalResolver & r, std::vector<SimBot *> bots, int max_steps)
{
  for (int step = 0; step < max_steps; ++step) {
    bool all = true;
    for (auto * b : bots) {
      if (b->done()) {
        continue;
      }
      all = false;
      const auto q = r.query(b->id, b->tip, 10.0);
      if (q.status == ResolverStatus::Free &&
        r.commit(b->id, q.proposal_id, q.through_index, "COMMITTED") == "ACCEPTED")
      {
        b->tip = q.through_index;
      }
      const double target = b->tip >= 0 ? b->s[static_cast<std::size_t>(b->tip)] : 0.0;
      b->at = std::min(target, b->at + 0.5);
      const auto p = pos_at(*b, b->at);
      r.update_pose(b->id, p.x, p.y);
    }
    if (all) {
      return step;
    }
  }
  return -1;
}
}  // namespace

// Field standstill (2026-11-18 12:5x): amr2 held for amr1; amr1 follows amr5;
// amr5's new route meets amr2 and ETA/FIFO made amr2 "first" → circular wait
// amr5→amr2→amr1→amr5. The dependency rule must put amr5 first and every
// robot must reach its goal.
TEST(GlobalResolverGroup, NoChainDeadlockWhenHeldRobotMeetsNewRoute)
{
  GlobalResolver r(cfg());
  Pts r1;
  leg(r1, 20, 0, 50, 0);
  leg(r1, 50, 0, 50, 20);   // amr1 east along the corridor, then north at x=50
  Pts r2;
  leg(r2, 80, 0, 38, 0);
  leg(r2, 38, 0, 38, -20);  // amr2 west along the corridor, then south at x=38
  Pts r5;
  leg(r5, 24, 0, 40, 0);
  leg(r5, 40, 0, 40, 20);   // amr5 (ahead of amr1) east, then north at x=40
  auto b1 = make_bot("amr1", r1);
  auto b2 = make_bot("amr2", r2);
  auto b5 = make_bot("amr5", r5);

  r.update_pose("amr5", 24, 0);  // amr5 parked on the corridor (previous task)
  r.update_pose("amr1", 20, 0);
  r.upsert_route("amr1", "t1", r1);
  r.update_pose("amr2", 80, 0);
  r.upsert_route("amr2", "t2", r2);
  ASSERT_EQ(between(r, "amr1", "amr2").front().first, "amr1");
  // amr2 approaches and holds for amr1; amr1 is stuck behind parked amr5.
  ASSERT_EQ(drive(r, {&b1, &b2}, 120), -1);
  EXPECT_EQ(r.query("amr2", b2.tip, 10.0).status, ResolverStatus::Wait);

  // amr5 gets its next task.
  r.upsert_route("amr5", "t5", r5);
  const auto d25 = between(r, "amr2", "amr5");
  ASSERT_FALSE(d25.empty());
  for (const auto & d : d25) {
    EXPECT_EQ(d.first, "amr5") << d.rule;
  }
  for (const auto & d : r.decisions()) {
    EXPECT_FALSE(d.unresolvable) << d.conflict_id << " " << d.rule;
  }
  EXPECT_GE(drive(r, {&b1, &b2, &b5}, 4000), 0) << "robots stood still (deadlock)";
  EXPECT_TRUE(b1.done() && b2.done() && b5.done());
}

// Field freeze (2026-11-18 15:1x): five robots parked nose-to-tail ~2.1 m
// apart in one lane get new tasks leaving the lane northwards. The rear robot
// cannot hold 2 m before the overlap, but it can stay where it is. Must not be
// "unresolvable"; front robot first, everyone reaches its goal.
TEST(GlobalResolverGroup, ParkedColumnLeavesOneByOne)
{
  GlobalResolver r(cfg());
  const double ys[5] = {1.10, 3.19, 5.29, 8.45, 10.55};  // amr5, amr2, amr1, amr4, amr3
  const char * ids[5] = {"amr5", "amr2", "amr1", "amr4", "amr3"};
  std::vector<SimBot> bots;
  for (int i = 0; i < 5; ++i) {
    r.update_pose(ids[i], 72.5, ys[i]);
  }
  for (int i = 4; i >= 0; --i) {  // front robot handed over first (as in the field)
    Pts route;
    leg(route, 72.5, ys[i], 72.5, 30.0, 1.05);
    leg(route, 72.5, 30.0, 72.5 + (i - 2) * 8.0, 40.0);  // fan out at the top
    bots.push_back(make_bot(ids[i], route));
    r.upsert_route(ids[i], std::string("t") + ids[i], route);
  }
  for (const auto & d : r.decisions()) {
    EXPECT_FALSE(d.unresolvable) << d.conflict_id << " " << d.rule;
  }
  std::vector<SimBot *> ptrs;
  for (auto & b : bots) {
    ptrs.push_back(&b);
  }
  EXPECT_GE(drive(r, ptrs, 6000), 0) << "column froze";
  for (const auto & b : bots) {
    EXPECT_TRUE(b.done()) << b.id;
  }
}

// True face-to-face stays unresolvable, and recovers once one robot is moved
// away (retry_unresolvable re-evaluates with live poses).
TEST(GlobalResolverGroup, FaceToFaceUnresolvableRecoversWhenCleared)
{
  GlobalResolver r(cfg());
  Pts a;
  leg(a, 0, 0, 20, 0);
  Pts b;
  leg(b, 1, 0, -20, 0);
  r.update_pose("amr1", 0, 0);
  r.update_pose("amr2", 1, 0);
  r.upsert_route("amr1", "a", a);
  r.upsert_route("amr2", "b", b);
  auto ds = between(r, "amr1", "amr2");
  ASSERT_EQ(ds.size(), 1u);
  EXPECT_TRUE(ds.front().unresolvable) << ds.front().rule;
  EXPECT_EQ(r.query("amr1", -1, 10.0).status, ResolverStatus::Recovery);
  EXPECT_TRUE(r.retry_unresolvable().empty());
  // Operator / zone_manager moves amr2 aside: off amr1's path.
  r.set_standing("amr2", 1, 5);
  Pts b2;
  leg(b2, 1, 5, -20, 5);
  r.upsert_route("amr2", "b2", b2);
  EXPECT_NE(r.query("amr1", -1, 10.0).status, ResolverStatus::Recovery);
}

// Field (2026-11-18 16:5x): "amr4 holds at wp2 before its overlap wp17..wp25",
// "amr5 holds at wp25 before wp48..wp60" — waits picked far back because every
// corridor waypoint in front was on some other robot's route, although those
// robots were already ordered against the waiter (leader ahead / follower held
// behind). The wait must be the NEAREST safe waypoint before the overlap.
TEST(GlobalResolverGroup, WaitIsNearestSafeWaypointOnSharedCorridor)
{
  GlobalResolver r(cfg());
  Pts lead;   // amr3 already on the corridor, heading east
  leg(lead, 16, 0, 60, 0);
  leg(lead, 60, 0, 60, 10);
  Pts mid;    // amr4 joins from its station spur, then east
  leg(mid, 10, 6, 10, 0);
  leg(mid, 10, 0, 52, 0);
  leg(mid, 52, 0, 52, -10);
  Pts back;   // amr5 joins behind amr4
  leg(back, 4, 6, 4, 0);
  leg(back, 4, 0, 46, 0);
  leg(back, 46, 0, 46, 10);
  Pts cross;  // amr1 crosses the corridor at x=40 (north → south)
  leg(cross, 40, 16, 40, -16);
  r.update_pose("amr3", 16, 0);
  r.update_pose("amr4", 10, 6);
  r.update_pose("amr5", 4, 6);
  r.update_pose("amr1", 40, 16);
  ResolverFacts hi;
  hi.priority_rank = 2;
  r.set_facts("amr1", hi);
  r.upsert_route("amr3", "t3", lead);
  r.upsert_route("amr4", "t4", mid);
  r.upsert_route("amr5", "t5", back);
  r.upsert_route("amr1", "t1", cross);

  const double slack = cfg().safety_distance_m + 2.0 + 0.5;  // + one waypoint gap
  int checked = 0;
  for (const auto & d : r.decisions()) {
    EXPECT_FALSE(d.unresolvable) << d.conflict_id << " " << d.rule;
    if (d.first != "amr1" || d.cleared) {
      continue;  // the crossing robot's waiters are the corridor robots
    }
    const auto & ps = d.of(d.second);
    EXPECT_GE(d.wait_s, ps.entry_s - slack)
      << d.second << " waits at wp" << d.wait_index << " (" << d.wait_x << "," << d.wait_y
      << ") far from its overlap wp" << ps.entry_index << " — " << d.rule;
    EXPECT_FALSE(d.relaxed_wait) << d.conflict_id;
    ++checked;
  }
  EXPECT_GE(checked, 2);

  auto b3 = make_bot("amr3", lead);
  auto b4 = make_bot("amr4", mid);
  auto b5 = make_bot("amr5", back);
  auto b1 = make_bot("amr1", cross);
  const int res = drive(r, {&b3, &b4, &b5, &b1}, 6000);
  EXPECT_GE(res, 0) << "robots stood still";
  EXPECT_TRUE(b3.done() && b4.done() && b5.done() && b1.done());
}

// Late base allocation after dependency clearance (field log 2026-11-19,
// amr1/amr2/amr3): p and q are BOTH already held for k before they reach
// their own crossing. q has the higher task priority, but k releases p long
// before it releases q. p must pass first — otherwise p's own dependency
// clears and it keeps waiting behind q, which is still held far longer.
//
//            p ─────────────┼────────── (60,20)      p: (0,-10) up, then east
//            │              │ q
//   k ───────┼──────────────┼──────────────────▶     k: y=0, x -10..100
//            │(0,0)         │(30,0)
//            p              q (30,-40)
TEST(GlobalResolver, BothHeldReleasedSoonerPassesFirst)
{
  GlobalResolver r(cfg());
  Pts k;
  leg(k, -10, 0, 100, 0);
  Pts p;
  leg(p, 0, -10, 0, 20);
  leg(p, 0, 20, 60, 20);
  Pts q;
  leg(q, 30, -40, 30, 40);
  ResolverFacts top;
  top.priority_rank = 2;
  ResolverFacts mid;
  mid.priority_rank = 1;
  r.set_facts("k", top);
  r.set_facts("q", mid);  // p keeps rank 0: priority alone would pick q

  r.update_pose("k", -10, 0);
  r.upsert_route("k", "k", k);
  r.update_pose("p", 0, -10);
  r.upsert_route("p", "p", p);
  r.update_pose("q", 30, -40);
  r.upsert_route("q", "q", q);

  // Preconditions: k passes first at both crossings (p and q are held).
  const auto kp = between(r, "k", "p");
  const auto kq = between(r, "k", "q");
  ASSERT_EQ(kp.size(), 1u);
  ASSERT_EQ(kq.size(), 1u);
  ASSERT_EQ(kp.front().first, "k") << kp.front().rule;
  ASSERT_EQ(kq.front().first, "k") << kq.front().rule;

  const auto pq = between(r, "p", "q");
  ASSERT_EQ(pq.size(), 1u);
  EXPECT_EQ(pq.front().first, "p") << pq.front().rule;
  EXPECT_EQ(pq.front().rule.rfind("dependency: both held", 0), 0u) << pq.front().rule;
}
