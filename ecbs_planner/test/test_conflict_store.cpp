#include <algorithm>
#include <gtest/gtest.h>

#include "ecbs_planner/conflict_store.hpp"
#include "ecbs_planner/types.hpp"

using ecbs_planner::ConflictRecord;
using ecbs_planner::ConflictStore;
using ecbs_planner::ParticipantOverlap;
using ecbs_planner::TimedPath;
using ecbs_planner::TimedPoint;
using ecbs_planner::discover_route_overlaps;
using ecbs_planner::make_occurrence;
using ecbs_planner::make_overlap_id;
using ecbs_planner::point_to_segment_distance;
using ecbs_planner::segments_conflict;

TimedPath make_line(
  const std::string & id, double x0, double y0, double x1, double y1,
  uint32_t rev = 0)
{
  TimedPath p;
  p.amr_id = id;
  p.fleet_name = "fleet";
  p.task_id = "t-" + id;
  p.path_revision = rev;
  for (int i = 0; i <= 5; ++i) {
    TimedPoint m;
    const double t = i / 5.0;
    m.x = x0 + t * (x1 - x0);
    m.y = y0 + t * (y1 - y0);
    m.level_name = "L1";
    p.waypoints.push_back(m);
  }
  return p;
}

ParticipantOverlap part(const std::string & id)
{
  ParticipantOverlap p;
  p.amr_id = id;
  return p;
}

TEST(ConflictStore, UpsertVersionsOnlyOnChange)
{
  ConflictStore store;
  ConflictRecord r;
  r.conflict_id = "c1";
  r.participants = {part("amr1"), part("amr2")};
  r.topology = "opposite";
  EXPECT_TRUE(store.upsert(r));
  EXPECT_EQ(store.get("c1")->record_version, 1u);
  EXPECT_FALSE(store.upsert(r));
  r.wait_reason = "opposite";
  EXPECT_TRUE(store.upsert(r));
  EXPECT_EQ(store.get("c1")->record_version, 2u);
}

TEST(ConflictStore, ClearancePreservedOnRediscovery)
{
  ConflictStore store;
  ConflictRecord a;
  a.conflict_id = "c-a";
  a.participants = {part("amr1"), part("amr2")};
  a.topology = "cross";
  store.upsert(a);
  EXPECT_TRUE(store.mark_physically_cleared("c-a"));
  ConflictRecord again = a;
  again.physically_cleared = false;
  store.upsert(again);
  EXPECT_TRUE(store.get("c-a")->physically_cleared);
}

TEST(ConflictStore, ClearOneOverlapKeepsOtherDeps)
{
  ConflictStore store;
  ConflictRecord a;
  a.conflict_id = "c-a";
  a.participants = {part("amr1"), part("amr2")};
  a.mandatory = true;
  a.waiter_amr = "amr1";
  a.blocker_amr = "amr2";
  ConflictRecord b;
  b.conflict_id = "c-b";
  b.participants = {part("amr1"), part("amr2")};
  b.mandatory = true;
  b.waiter_amr = "amr1";
  b.blocker_amr = "amr2";
  store.upsert(a);
  store.upsert(b);
  EXPECT_TRUE(store.mark_physically_cleared("c-a"));
  EXPECT_EQ(store.outbound_deps("amr1").size(), 1u);
  EXPECT_EQ(store.outbound_deps("amr1").front().conflict_id, "c-b");
}

TEST(ConflictStore, GeometricOverlapIsNotMandatoryWait)
{
  auto a = make_line("amr1", 0, 0, 10, 0);
  auto b = make_line("amr2", 5, 0, 15, 0);
  const auto found = discover_route_overlaps(a, {b}, 0.5, 0.35, 2.5, 1, 1);
  ASSERT_FALSE(found.empty());
  EXPECT_FALSE(found.front().mandatory);
  EXPECT_TRUE(found.front().waiter_amr.empty());
}

TEST(ConflictStore, SegmentCrossingDetectedWithoutSharedEndpoints)
{
  // Proper interior intersection: endpoints are 5 m from the other segment.
  EXPECT_TRUE(segments_conflict(-5, 0, 5, 0, 0, -5, 0, 5, 0.1));
  auto a = make_line("amr1", -5, 0, 5, 0);
  auto b = make_line("amr2", 0, -5, 0, 5);
  const auto found = discover_route_overlaps(a, {b}, 0.1, 0.35, 2.5, 1, 1);
  EXPECT_FALSE(found.empty());
}

TEST(ConflictStore, IdleMidSegmentDetected)
{
  auto a = make_line("amr1", 0, 0, 10, 0);
  TimedPath idle;
  idle.amr_id = "amr2";
  idle.path_revision = 0;
  TimedPoint p;
  // Midway on segment between densified waypoints (x=1 and x=3 → x=2).
  p.x = 2.0;
  p.y = 0.0;
  p.level_name = "L1";
  idle.waypoints = {p};
  EXPECT_LE(point_to_segment_distance(2, 0, 0, 0, 10, 0), 0.01);
  const auto found = discover_route_overlaps(a, {idle}, 0.5, 0.35, 2.5, 1, 1);
  EXPECT_FALSE(found.empty());
}

TEST(ConflictStore, IdleSinglePointFootprintDetected)
{
  auto a = make_line("amr1", 0, 0, 10, 0);
  TimedPath idle;
  idle.amr_id = "amr2";
  idle.path_revision = 0;
  TimedPoint p;
  p.x = 4.0;
  p.y = 0.0;
  p.level_name = "L1";
  idle.waypoints = {p};
  const auto found = discover_route_overlaps(a, {idle}, 0.5, 0.35, 2.5, 1, 1);
  EXPECT_FALSE(found.empty());
}

TEST(ConflictStore, DisjointOverlapsStaySeparate)
{
  TimedPath a;
  a.amr_id = "amr1";
  a.path_revision = 1;
  for (int i = 0; i <= 20; ++i) {
    TimedPoint w;
    w.x = static_cast<double>(i);
    w.y = 0;
    w.level_name = "L1";
    a.waypoints.push_back(w);
  }
  TimedPath b = a;
  b.amr_id = "amr2";
  for (auto & w : b.waypoints) {
    w.y = 100;
  }
  b.waypoints[1].y = 0;
  b.waypoints[1].x = 1;
  b.waypoints[19].y = 0;
  b.waypoints[19].x = 19;
  const auto found = discover_route_overlaps(a, {b}, 0.5, 0.35, 2.5, 1, 1);
  EXPECT_GE(found.size(), 2u);
}

TEST(ConflictStore, SafeWaitUsesSafetyDistanceNotPrevWaypoint)
{
  auto a = make_line("amr1", 0, 0, 20, 0);
  auto b = make_line("amr2", 10, 0, 20, 0);
  const auto found = discover_route_overlaps(a, {b}, 0.5, 0.35, 5.0, 1, 1);
  ASSERT_FALSE(found.empty());
  const auto * self = found.front().participant("amr1");
  ASSERT_NE(self, nullptr);
  EXPECT_LT(self->wait.route_occurrence, self->entry.route_occurrence);
}

TEST(ConflictStore, ParticipantGeometryBoundByIdentity)
{
  auto a = make_line("AMR2", 0, 0, 10, 0, 1);
  auto b = make_line("AMR1", 5, -5, 5, 5, 1);
  const auto found = discover_route_overlaps(a, {b}, 0.5, 0.35, 2.5, 1, 1);
  ASSERT_FALSE(found.empty());
  const auto * p2 = found.front().participant("AMR2");
  const auto * p1 = found.front().participant("AMR1");
  ASSERT_NE(p2, nullptr);
  ASSERT_NE(p1, nullptr);
  EXPECT_EQ(p2->task_id, "t-AMR2");
  EXPECT_EQ(p1->task_id, "t-AMR1");
  EXPECT_EQ(p2->path_revision, 1u);
  EXPECT_EQ(p1->path_revision, 1u);
}

TEST(ConflictStore, ClusterUsesGeometricParticipants)
{
  ConflictStore store;
  ConflictRecord r;
  r.conflict_id = "geo";
  r.participants = {part("amr1"), part("amr2")};
  r.mandatory = false;
  store.upsert(r);
  const auto clusters = store.build_clusters();
  ASSERT_EQ(clusters.size(), 1u);
  EXPECT_EQ(clusters.front().members.size(), 2u);
  EXPECT_TRUE(clusters.front().edges.empty());
}

TEST(ConflictStore, ChainPrecedenceWithMandatoryEdges)
{
  ConflictStore store;
  auto add = [&](const std::string & w, const std::string & b, const std::string & id) {
      ConflictRecord r;
      r.conflict_id = id;
      r.participants = {part(w), part(b)};
      r.mandatory = true;
      r.waiter_amr = w;
      r.blocker_amr = b;
      store.upsert(r);
    };
  add("amr1", "amr2", "d12");
  add("amr2", "amr3", "d23");
  add("amr4", "amr1", "d41");
  const auto clusters = store.build_clusters();
  ASSERT_EQ(clusters.size(), 1u);
  const auto & prec = clusters.front().proposed_precedence;
  const auto i3 = std::find(prec.begin(), prec.end(), "amr3");
  const auto i2 = std::find(prec.begin(), prec.end(), "amr2");
  const auto i1 = std::find(prec.begin(), prec.end(), "amr1");
  const auto i4 = std::find(prec.begin(), prec.end(), "amr4");
  EXPECT_LT(i3, i2);
  EXPECT_LT(i2, i1);
  EXPECT_LT(i1, i4);
}

TEST(ConflictStore, VertexZeroAndOriginValid)
{
  auto a = make_line("amr1", 0, 0, 5, 0);
  a.waypoints.front().index = 0;
  auto b = make_line("amr2", 0, 0, 0, 5);
  b.waypoints.front().index = 0;
  const auto found = discover_route_overlaps(a, {b}, 0.5, 0.35, 2.5, 1, 1);
  EXPECT_FALSE(found.empty());
  const auto * p = found.front().participant("amr1");
  ASSERT_NE(p, nullptr);
  EXPECT_DOUBLE_EQ(p->entry.x, 0.0);
}

TEST(ConflictStore, OverlapIdStableAcrossDiscoveryOrder)
{
  const auto id_ab = make_overlap_id("AMR2", "AMR1", 1, 0, 1, 2);
  const auto id_ba = make_overlap_id("AMR1", "AMR2", 1, 2, 1, 0);
  EXPECT_EQ(id_ab, id_ba);
}

/// F1: detection radius must be clearance_m, not position_epsilon_m.
TEST(ConflictStore, ParallelRoutesDetectedAtClearanceNotEpsilon)
{
  constexpr double clearance_m = 2.0;
  constexpr double position_epsilon_m = 0.35;
  auto a = make_line("amr1", 0, 0, 10, 0);
  auto near = make_line("amr2", 0, 1.2, 10, 1.2);
  auto far = make_line("amr3", 0, 2.5, 10, 2.5);

  const auto hit = discover_route_overlaps(
    a, {near}, clearance_m, position_epsilon_m, 2.5, 1, 1);
  EXPECT_FALSE(hit.empty()) << "1.2 m lane gap must detect at clearance 2.0";

  const auto miss = discover_route_overlaps(
    a, {far}, clearance_m, position_epsilon_m, 2.5, 1, 1);
  EXPECT_TRUE(miss.empty()) << "2.5 m lane gap must not detect at clearance 2.0";

  // Old bug: epsilon-sized detect missed the same 1.2 m gap.
  const auto epsilon_miss = discover_route_overlaps(
    a, {near}, position_epsilon_m, position_epsilon_m, 2.5, 1, 1);
  EXPECT_TRUE(epsilon_miss.empty());
}

TEST(ConflictStore, IdleRobotDetectedAtClearanceDistance)
{
  constexpr double clearance_m = 2.0;
  constexpr double position_epsilon_m = 0.35;
  auto a = make_line("amr1", 0, 0, 10, 0);
  TimedPath idle;
  idle.amr_id = "amr2";
  TimedPoint p;
  p.x = 5.0;
  p.y = 1.5;
  p.level_name = "L1";
  idle.waypoints = {p};

  const auto found = discover_route_overlaps(
    a, {idle}, clearance_m, position_epsilon_m, 2.5, 1, 1);
  EXPECT_FALSE(found.empty()) << "idle 1.5 m off route must detect at clearance 2.0";
}

// ---- F3: stable conflict ids ----

namespace
{
/// n points, route_occurrence stamped 0..n-1 (as ScheduleCache stores them).
TimedPath stamped_line(
  const std::string & id, double x0, double y0, double x1, double y1, int n)
{
  TimedPath p;
  p.amr_id = id;
  p.fleet_name = "fleet";
  p.task_id = "t-" + id;
  p.path_revision = 1;
  for (int i = 0; i < n; ++i) {
    TimedPoint m;
    const double t = static_cast<double>(i) / (n - 1);
    m.x = x0 + t * (x1 - x0);
    m.y = y0 + t * (y1 - y0);
    m.level_name = "L1";
    m.route_occurrence = static_cast<uint32_t>(i);
    p.waypoints.push_back(m);
  }
  return p;
}
}  // namespace

TEST(ConflictStore, SameOverlapKeepsIdAfterPrefixErase)
{
  auto a = stamped_line("amr1", 0, 0, 20, 0, 21);
  const auto b = stamped_line("amr2", 10, -10, 10, 10, 21);
  const auto before = discover_route_overlaps(a, {b}, 0.5, 0.35, 2.5, 1, 1);
  ASSERT_EQ(before.size(), 1u);

  // Cache rolling erase: robot passed first 3 waypoints; stamps are kept.
  a.waypoints.erase(a.waypoints.begin(), a.waypoints.begin() + 3);
  const auto after = discover_route_overlaps(a, {b}, 0.5, 0.35, 2.5, 1, 1);
  ASSERT_EQ(after.size(), 1u);
  EXPECT_EQ(before.front().conflict_id, after.front().conflict_id);

  // Symmetric: discovered from the other robot's side → same id.
  const auto mirror = discover_route_overlaps(b, {a}, 0.5, 0.35, 2.5, 1, 1);
  ASSERT_EQ(mirror.size(), 1u);
  EXPECT_EQ(before.front().conflict_id, mirror.front().conflict_id);
}

TEST(ConflictStore, ConflictIdExcludesTopology)
{
  const auto a = stamped_line("amr1", 0, 0, 20, 0, 21);
  const auto b = stamped_line("amr2", 10, -10, 10, 10, 21);
  const auto found = discover_route_overlaps(a, {b}, 0.5, 0.35, 2.5, 1, 1);
  ASSERT_EQ(found.size(), 1u);
  ASSERT_FALSE(found.front().topology.empty());
  EXPECT_EQ(found.front().conflict_id.find(found.front().topology), std::string::npos);
}

TEST(ConflictStore, TopologyReclassificationUpdatesSameId)
{
  ConflictStore store;
  ConflictRecord r;
  r.conflict_id = make_overlap_id("amr1", "amr2", 1, 9, 1, 10);
  r.participants = {part("amr1"), part("amr2")};
  r.topology = "plus";
  ASSERT_TRUE(store.upsert(r));
  ASSERT_EQ(store.get(r.conflict_id)->record_version, 1u);

  r.topology = "cross";  // topology only; everything else equal
  EXPECT_TRUE(store.upsert(r));
  EXPECT_EQ(store.size(), 1u);
  EXPECT_EQ(store.get(r.conflict_id)->record_version, 2u);
  EXPECT_EQ(store.get(r.conflict_id)->topology, "cross");
}
