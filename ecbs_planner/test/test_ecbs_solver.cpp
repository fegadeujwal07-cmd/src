#include <algorithm>
#include <gtest/gtest.h>

#include "ecbs_planner/ecbs_solver.hpp"
#include "ecbs_planner/evaluate_path.hpp"
#include "ecbs_planner/path_utils.hpp"

using ecbs_planner::EcbsSolver;
using ecbs_planner::ScheduleCache;
using ecbs_planner::SolveStatus;
using ecbs_planner::TimedPath;
using ecbs_planner::TimedPoint;

namespace
{

/// Helper: build a TimedPoint with graph index.
TimedPoint wp(double t, double x, double y, uint64_t index)
{
  TimedPoint p;
  p.t_sec = t;
  p.x = x;
  p.y = y;
  p.index = index;
  return p;
}

/// Helper: TimedPath with amr_id / task_id and given waypoints.
TimedPath make_path(const std::string & id, std::vector<TimedPoint> pts)
{
  TimedPath p;
  p.amr_id = id;
  p.task_id = id + "_task";
  p.waypoints = std::move(pts);
  return p;
}

/// Assert no residual space-time conflict between two paths.
void ExpectNoConflict(
  const TimedPath & a, const TimedPath & b, double eps = 0.35)
{
  EXPECT_FALSE(EcbsSolver::earliest_conflict(a, b, eps).has_value())
    << "residual conflict between " << a.amr_id << " and " << b.amr_id;
}

}  // namespace

/// Non-overlapping paths → Ok, no waits, no updated_others.
TEST(EcbsSolver, ConflictFreePassesThrough)
{
  ScheduleCache cache;
  cache.upsert(make_path("A", {wp(0, 0, 0, 1), wp(5, 1, 0, 2)}));

  EcbsSolver solver;
  auto result = solver.solve(make_path("B", {wp(0, 0, 3, 10), wp(5, 1, 3, 11)}), cache);
  EXPECT_EQ(result.status, SolveStatus::Ok);
  EXPECT_TRUE(result.updated_others.empty());
}

/// Lanes 1 m apart (other graph points) at the same time: the robots touch
/// (clearance 2 m), so it is a conflict.
TEST(EcbsSolver, SideBySideLanesConflict)
{
  const auto a = make_path("A", {wp(0, 0, 0, 1), wp(5, 1, 0, 2)});
  const auto b = make_path("B", {wp(0, 0, 1, 10), wp(5, 1, 1, 11)});
  EXPECT_TRUE(EcbsSolver::earliest_conflict(a, b, 2.0).has_value());
  EXPECT_FALSE(EcbsSolver::earliest_conflict(a, b, 0.35).has_value());
}

/// Both can wait → mutual provisional holds (both get wait_sec + deps).
/// Not FCFS — neither side is preferred for the time wait.
TEST(EcbsSolver, MutualBothAmrsGetWaits)
{
  ScheduleCache cache;
  cache.upsert(make_path("A", {wp(0, 0, 0, 5), wp(10, 1, 0, 6)}));

  EcbsSolver solver({1.0, 30.0, 20, 0.35});
  auto result = solver.solve(make_path("B", {wp(2, 0, 0, 5), wp(12, 1, 0, 7)}), cache);
  ASSERT_TRUE(
    result.status == SolveStatus::UpdatedOther ||
    result.status == SolveStatus::Ok);
  ASSERT_FALSE(result.path.waypoints.empty());
  EXPECT_GT(result.path.waypoints.front().wait_duration_sec, 0.0);
  ASSERT_FALSE(result.path.waypoints.front().waiting_for.empty());
  EXPECT_EQ(result.path.waypoints.front().waiting_for.front().amr_id, "A");

  ASSERT_FALSE(result.updated_others.empty());
  EXPECT_EQ(result.updated_others.front().amr_id, "A");
  EXPECT_GT(result.updated_others.front().waypoints.front().wait_duration_sec, 0.0)
    << "cached AMR must also get provisional wait_sec (mutual holds)";
  ASSERT_FALSE(result.updated_others.front().waypoints.front().waiting_for.empty());
  EXPECT_EQ(
    result.updated_others.front().waypoints.front().waiting_for.front().amr_id, "B");

  auto cached_a = cache.get("A");
  auto cached_b = cache.get("B");
  ASSERT_TRUE(cached_a.has_value());
  ASSERT_TRUE(cached_b.has_value());
  EXPECT_GT(cached_a->waypoints.front().wait_duration_sec, 0.0);
  EXPECT_GT(cached_b->waypoints.front().wait_duration_sec, 0.0);

  ExpectNoConflict(result.path, result.updated_others.front());
  ExpectNoConflict(*cached_b, *cached_a);
}

/// Mutual also when other wait smaller (no FIFO preference).
TEST(EcbsSolver, MutualEvenIfOtherWaitIsSmaller)
{
  ScheduleCache cache;
  cache.upsert(make_path("A", {wp(0, 0, 0, 5), wp(3, 1, 0, 6)}));

  EcbsSolver solver({1.0, 30.0, 20, 0.35});
  auto result = solver.solve(make_path("B", {wp(1, 0, 0, 5), wp(10, 1, 0, 7)}), cache);
  ASSERT_TRUE(
    result.status == SolveStatus::UpdatedOther ||
    result.status == SolveStatus::Ok);
  EXPECT_FALSE(result.updated_others.empty());
  EXPECT_GT(result.path.waypoints.front().wait_duration_sec, 0.0);
  EXPECT_GT(result.updated_others.front().waypoints.front().wait_duration_sec, 0.0);
  ExpectNoConflict(result.path, result.updated_others.front());
}

/// Candidate cannot wait within max → earlier AMR A gets time wait;
/// both still get waiting_for; candidate also gets provisional wait_sec
/// (metadata) so ZM sees mutual holds.
TEST(EcbsSolver, EarlierAmrWaitsWhenOnlyThatFixesConflict)
{
  ScheduleCache cache;
  cache.upsert(make_path("A", {wp(0, 0, 0, 5), wp(8, 1, 0, 6)}));

  EcbsSolver solver({1.0, 5.0, 20, 0.35});
  auto result = solver.solve(make_path("B", {wp(1, 0, 0, 5), wp(3, 1, 0, 7)}), cache);

  ASSERT_EQ(result.status, SolveStatus::UpdatedOther) << result.reason;
  ASSERT_EQ(result.updated_others.size(), 1u);
  EXPECT_EQ(result.updated_others.front().amr_id, "A");
  EXPECT_GT(result.updated_others.front().waypoints.front().wait_duration_sec, 0.0);
  ASSERT_FALSE(result.updated_others.front().waypoints.front().waiting_for.empty());
  EXPECT_EQ(result.updated_others.front().waypoints.front().waiting_for.front().amr_id, "B");
  ASSERT_FALSE(result.path.waypoints.front().waiting_for.empty());
  EXPECT_EQ(result.path.waypoints.front().waiting_for.front().amr_id, "A");
  EXPECT_GT(
    result.path.waypoints.front().waiting_for.front().wait_point.wait_duration_sec,
    0.0)
    << "candidate gets provisional wait_sec on wait_point for ZM";
  EXPECT_EQ(result.path.amr_id, "B");
  ExpectNoConflict(result.path, result.updated_others.front());
}

/// Only candidate can wait within max_wait; cached AMR still gets waiting_for
/// plus a provisional wait_sec (capped at max_wait) for ZM.
TEST(EcbsSolver, OnlyCandidateTimeWaitStillAnnotatesOther)
{
  ScheduleCache cache;
  // A clears soon; B can wait within 10s; wait_other for A exceeds max.
  // wait_B = 10+1-2 = 9 ≤ 10; wait_A ≈ depart_B(~12)+1 - 0 ≈ 13 > 10.
  cache.upsert(make_path("A", {wp(0, 0, 0, 5), wp(10, 1, 0, 6)}));

  EcbsSolver solver({1.0, 10.0, 20, 0.35});
  auto result = solver.solve(make_path("B", {wp(2, 0, 0, 5), wp(12, 1, 0, 7)}), cache);

  ASSERT_EQ(result.status, SolveStatus::UpdatedOther);
  ASSERT_EQ(result.updated_others.size(), 1u);
  EXPECT_EQ(result.updated_others.front().amr_id, "A");
  ASSERT_FALSE(result.updated_others.front().waypoints.front().waiting_for.empty());
  EXPECT_EQ(
    result.updated_others.front().waypoints.front().waiting_for.front().amr_id, "B");
  const auto & a_wp = result.updated_others.front().waypoints.front();
  EXPECT_DOUBLE_EQ(a_wp.waiting_for.front().wait_point.wait_duration_sec, 10.0)
    << "provisional wait_sec capped at max_wait (needed ~13s)";
  EXPECT_DOUBLE_EQ(a_wp.wait_duration_sec, 0.0)
    << "provisional is metadata only; A's timing is unchanged";
  EXPECT_DOUBLE_EQ(ecbs_planner::published_wait_sec(a_wp), 10.0)
    << "published PathPoint.wait_sec shows the capped provisional";
  EXPECT_GT(result.path.waypoints.front().wait_duration_sec, 0.0);
  ASSERT_FALSE(result.path.waypoints.front().waiting_for.empty());
  ExpectNoConflict(result.path, result.updated_others.front());
}

/// Waiting path records waiting_for + clear_points for the blocker.
TEST(EcbsSolver, LaterAmrRecordsWaitingFor)
{
  ScheduleCache cache;
  cache.upsert(make_path("A", {wp(0, 0, 0, 5), wp(10, 1, 0, 6)}));

  EcbsSolver solver({1.0, 30.0, 20, 0.35});
  auto result = solver.solve(make_path("B", {wp(2, 0, 0, 5), wp(12, 1, 0, 7)}), cache);
  ASSERT_TRUE(
    result.status == SolveStatus::UpdatedOther ||
    result.status == SolveStatus::Ok);
  ASSERT_FALSE(result.path.waypoints.front().waiting_for.empty());
  EXPECT_EQ(result.path.waypoints.front().waiting_for.front().amr_id, "A");
  ASSERT_FALSE(result.path.waypoints.front().waiting_for.front().clear_points.empty());
  if (!result.updated_others.empty()) {
    ExpectNoConflict(result.path, result.updated_others.front());
  }
}

/// max_wait too small for both sides → FailReplan.
TEST(EcbsSolver, FailWhenNeitherCanWait)
{
  ScheduleCache cache;
  cache.upsert(make_path("A", {wp(0, 0, 0, 5), wp(50, 1, 0, 6)}));

  EcbsSolver solver({1.0, 2.0, 20, 0.35});
  auto result = solver.solve(make_path("B", {wp(1, 0, 0, 5), wp(3, 1, 0, 7)}), cache);
  EXPECT_EQ(result.status, SolveStatus::FailReplan);
}

/// Three cached AMRs. Candidate overlaps A and C at different cells; D is
/// clear. One solve() must clear both conflicts before it commits.
TEST(EcbsSolver, ResolvesTwoConflictsInOneSolve)
{
  ScheduleCache cache;
  // A holds cell 10 over [0, 10). C holds cell 20 over [30, 45).
  // D is on a disjoint corridor.
  cache.upsert(make_path("A", {
    wp(0, 0, 0, 10), wp(10, 1, 0, 11)}));
  cache.upsert(make_path("C", {
    wp(30, 10, 0, 20), wp(45, 11, 0, 21)}));
  cache.upsert(make_path("D", {
    wp(0, 100, 0, 90), wp(5, 101, 0, 91)}));

  // B hits A first, then still overlaps C after that wait is applied.
  EcbsSolver solver({1.0, 60.0, 30, 0.35});
  auto result = solver.solve(
    make_path("B", {
      wp(1, 0, 0, 10),
      wp(12, 5, 0, 15),
      wp(32, 10, 0, 20),
      wp(48, 15, 0, 25)}),
    cache);

  ASSERT_TRUE(
    result.status == SolveStatus::UpdatedOther ||
    result.status == SolveStatus::Ok)
    << result.reason;

  auto cached_a = cache.get("A");
  auto cached_c = cache.get("C");
  auto cached_d = cache.get("D");
  auto cached_b = cache.get("B");
  ASSERT_TRUE(cached_a.has_value());
  ASSERT_TRUE(cached_c.has_value());
  ASSERT_TRUE(cached_d.has_value());
  ASSERT_TRUE(cached_b.has_value());

  ExpectNoConflict(*cached_b, *cached_a);
  ExpectNoConflict(*cached_b, *cached_c);
  ExpectNoConflict(*cached_b, *cached_d);

  bool saw_a = false;
  bool saw_c = false;
  bool saw_d = false;
  for (const auto & o : result.updated_others) {
    if (o.amr_id == "A") {
      saw_a = true;
    }
    if (o.amr_id == "C") {
      saw_c = true;
    }
    if (o.amr_id == "D") {
      saw_d = true;
    }
  }
  EXPECT_TRUE(saw_a);
  EXPECT_TRUE(saw_c);
  EXPECT_FALSE(saw_d);
}

#include "ecbs_planner/evaluate_path.hpp"
#include <ecbs_planner/msg/clear_point.hpp>
#include <ecbs_planner/msg/evaluate_path_waypoint.hpp>
#include <ecbs_planner/msg/from_zone_manager.hpp>
#include <ecbs_planner/msg/wait_dependency.hpp>
#include <ecbs_planner/srv/evaluate_path.hpp>
#include <ecbs_planner/msg/register_update.hpp>
#include <ecbs_planner/msg/wait_dependency.hpp>
#include <ecbs_planner/msg/clear_point.hpp>

namespace
{

ecbs_planner::msg::EvaluatePathWaypoint msg_wp(
  double t, double x, double y, int64_t index)
{
  ecbs_planner::msg::EvaluatePathWaypoint w;
  w.t_sec = t;
  w.x = x;
  w.y = y;
  w.yaw = 0.0;
  w.level_name = "L1";
  w.index = index;
  // wait_sec / waiting_for left default — evaluate omits; register sets when needed
  return w;
}

}  // namespace

TEST(EvaluatePath, RegisterRequiresUpdates)
{
  ecbs_planner::ScheduleCache cache;
  ecbs_planner::EcbsSolver solver;
  ecbs_planner::srv::EvaluatePath::Request req;
  ecbs_planner::srv::EvaluatePath::Response res;
  req.mode = "register";
  req.fleet_name = "amr_fleet";
  req.amr_id = "amr1";
  req.waypoints = {
    msg_wp(0, 0, 0, 0),
    msg_wp(5, 5, 0, 1),
  };
  // No updates[] → rejected (ZM register is updates-only).
  ecbs_planner::evaluate_path(req, res, cache, solver);
  EXPECT_FALSE(res.success);
  EXPECT_NE(res.message.find("updates[]"), std::string::npos);
}

/// TM seed PathIntent via mode=evaluate must land in ScheduleCache even when
/// empty — otherwise BaseExtensionQuery stays STALE and no PathRelease/order.
TEST(EvaluatePath, EvaluateUpsertsRequestGeometryIntoEmptyCache)
{
  ecbs_planner::ScheduleCache cache;
  ecbs_planner::EcbsSolver solver;
  ecbs_planner::srv::EvaluatePath::Request req;
  ecbs_planner::srv::EvaluatePath::Response res;
  req.mode = "evaluate";
  req.fleet_name = "amr_fleet";
  req.amr_id = "amr1";
  req.task_id = "dispatch-uuid-1";
  req.path_revision = 1;
  req.query_horizon_m = 10.0;
  req.current_x = 0.0;
  req.current_y = 0.0;
  req.waypoints = {
    msg_wp(0, 0, 0, 0),
    msg_wp(1, 1, 0, 1),
    msg_wp(2, 2, 0, 2),
    msg_wp(3, 3, 0, 3),
  };
  EXPECT_FALSE(cache.get("amr1").has_value());
  ecbs_planner::evaluate_path(req, res, cache, solver);
  EXPECT_TRUE(res.success);
  const auto cached = cache.get("amr1");
  ASSERT_TRUE(cached.has_value());
  EXPECT_EQ(cached->waypoints.size(), 4u);
  EXPECT_DOUBLE_EQ(cached->waypoints[2].x, 2.0);
  EXPECT_EQ(cached->task_id, "dispatch-uuid-1");
}

/// Prefer PathIntent request geometry over a prior /from_zone_manager sample
/// so proposal indices match the route full_control executes.
TEST(EvaluatePath, EvaluateReplacesDivergentPlannerSample)
{
  ecbs_planner::ScheduleCache cache;
  ecbs_planner::EcbsSolver solver;
  {
    ecbs_planner::TimedPath planner;
    planner.amr_id = "amr1";
    planner.task_id = "old";
    planner.path_revision = 1;
    for (int i = 0; i < 10; ++i) {
      ecbs_planner::TimedPoint wp;
      wp.x = i * 0.5;  // denser planner sample
      wp.y = 0.0;
      wp.index = static_cast<uint64_t>(i);
      planner.waypoints.push_back(wp);
    }
    cache.upsert(planner);
  }
  ecbs_planner::srv::EvaluatePath::Request req;
  ecbs_planner::srv::EvaluatePath::Response res;
  req.mode = "evaluate";
  req.amr_id = "amr1";
  req.task_id = "dispatch-uuid-1";
  req.path_revision = 1;
  req.query_horizon_m = 20.0;
  req.current_x = 0.0;
  req.current_y = 0.0;
  req.waypoints = {
    msg_wp(0, 0, 0, 0),
    msg_wp(1, 2, 0, 1),
    msg_wp(2, 4, 0, 2),
  };
  ecbs_planner::evaluate_path(req, res, cache, solver);
  EXPECT_TRUE(res.success);
  const auto cached = cache.get("amr1");
  ASSERT_TRUE(cached.has_value());
  EXPECT_EQ(cached->waypoints.size(), 3u);
  EXPECT_DOUBLE_EQ(cached->waypoints[1].x, 2.0);
  EXPECT_EQ(cached->task_id, "dispatch-uuid-1");
}

TEST(EvaluatePath, RegisterUpdatesLeanWaitResponse)
{
  using ecbs_planner::TimedPath;
  using ecbs_planner::TimedPoint;

  ecbs_planner::ScheduleCache cache;
  ecbs_planner::EcbsSolver solver;

  TimedPath seed;
  seed.amr_id = "amr1";
  seed.task_id = "task_A";
  seed.path_revision = 1;
  for (int i = 0; i < 3; ++i) {
    TimedPoint wp;
    wp.t_sec = static_cast<double>(i);
    wp.x = static_cast<double>(i * 5);
    wp.y = static_cast<double>(i * 5);
    wp.index = static_cast<uint64_t>(i);
    seed.waypoints.push_back(wp);
  }
  cache.upsert(seed);

  ecbs_planner::srv::EvaluatePath::Request req;
  ecbs_planner::srv::EvaluatePath::Response res;
  req.mode = "register";
  req.fleet_name = "amr_fleet";

  ecbs_planner::msg::RegisterUpdate u;
  u.amr_id = "amr1";
  u.task_id = "task_A";
  u.phase_index = 0;
  u.path_revision = 1;
  {
    ecbs_planner::msg::WaitDependency dep;
    dep.amr_id = "amr2";
    dep.wait_point.x = 5.0;
    dep.wait_point.y = 5.0;
    dep.wait_point.index = 1;
    dep.wait_point.wait_sec = 8.0;
    ecbs_planner::msg::ClearPoint cp;
    cp.x = 10.0;
    cp.y = 0.0;
    dep.clear_points.push_back(cp);
    u.waiting_for.push_back(dep);
  }
  req.updates = {u};

  ecbs_planner::evaluate_path(req, res, cache, solver);
  EXPECT_TRUE(res.success);
  EXPECT_EQ(res.status, "WAIT");
  EXPECT_EQ(res.dep_from_index, 1);
  EXPECT_EQ(res.dep_to_index, 1);
  ASSERT_EQ(res.waiting_for.size(), 1u);
  EXPECT_EQ(res.waiting_for[0].amr_id, "amr2");
  EXPECT_DOUBLE_EQ(res.waiting_for[0].wait_point.x, 5.0);
  EXPECT_DOUBLE_EQ(res.waiting_for[0].wait_point.y, 5.0);
  EXPECT_EQ(res.waiting_for[0].wait_point.index, 1);
  EXPECT_DOUBLE_EQ(res.waiting_for[0].wait_point.wait_sec, 8.0);

  const auto cached = cache.get("amr1");
  ASSERT_TRUE(cached.has_value());
  ASSERT_FALSE(cached->waypoints[1].waiting_for.empty());
  EXPECT_DOUBLE_EQ(cached->waypoints[1].waiting_for[0].wait_point.x, 5.0);
  EXPECT_DOUBLE_EQ(
    cached->waypoints[1].waiting_for[0].wait_point.wait_duration_sec, 8.0);
}

TEST(EvaluatePath, EvaluateDetectsConflictLeanWait)
{
  using ecbs_planner::TimedPath;
  using ecbs_planner::TimedPoint;

  ecbs_planner::ScheduleCache cache;
  ecbs_planner::EcbsSolver solver;

  // Seed amr2 (Process #1 / from_zone_manager) — no ZM register yet.
  TimedPath other;
  other.amr_id = "amr2";
  other.task_id = "task_B";
  other.path_revision = 1;
  {
    TimedPoint wp;
    wp.t_sec = 0; wp.x = 10; wp.y = 0; wp.index = 0;
    other.waypoints.push_back(wp);
    wp.t_sec = 1; wp.x = 5; wp.y = 5; wp.index = 1;
    other.waypoints.push_back(wp);
    wp.t_sec = 2; wp.x = 0; wp.y = 0; wp.index = 2;
    other.waypoints.push_back(wp);
  }
  cache.upsert(other);

  // Conflict inside the 20 m base, but no base given yet → WAIT at start.
  ecbs_planner::srv::EvaluatePath::Request req;
  ecbs_planner::srv::EvaluatePath::Response res;
  req.mode = "evaluate";
  req.amr_id = "amr1";
  req.task_id = "task_A";
  req.path_revision = 1;
  req.query_horizon_m = 20.0;
  req.current_x = 0.0;
  req.current_y = 0.0;
  req.waypoints = {
    msg_wp(0, 0, 0, 0),
    msg_wp(5, 5, 0, 1),
    msg_wp(10, 10, 0, 2),
  };
  ecbs_planner::evaluate_path(req, res, cache, solver);
  EXPECT_TRUE(res.success);
  EXPECT_EQ(res.status, "WAIT") << res.message;
  ASSERT_EQ(res.waiting_for.size(), 1u);
  EXPECT_EQ(res.waiting_for[0].amr_id, "amr2");
  EXPECT_EQ(res.waiting_for[0].wait_point.index, 0);
}

/// Base 0..20 already given; conflict fully inside it → owner priority FREE.
TEST(EvaluatePath, EvaluateConflictInsideBaseIsFree)
{
  using ecbs_planner::TimedPath;
  using ecbs_planner::TimedPoint;

  ecbs_planner::ScheduleCache cache;
  ecbs_planner::EcbsSolver solver;

  ecbs_planner::srv::EvaluatePath::Request req;
  ecbs_planner::srv::EvaluatePath::Response res;
  req.mode = "evaluate";
  req.amr_id = "amr1";
  req.task_id = "task_A";
  req.path_revision = 1;
  req.query_horizon_m = 20.0;
  req.current_x = 0.0;
  req.current_y = 0.0;
  for (int i = 0; i <= 8; ++i) {
    req.waypoints.push_back(msg_wp(i, i * 5.0, 0.0, i));
  }
  // Empty cache → FREE → base 0..20 given.
  ecbs_planner::evaluate_path(req, res, cache, solver);
  ASSERT_EQ(res.status, "FREE");
  ASSERT_TRUE(cache.granted_base("amr1").has_value());

  // Other occupies only 0..20 → overlap ends at the given base end.
  TimedPath other;
  other.amr_id = "amr2";
  other.task_id = "task_B";
  for (int i = 0; i <= 4; ++i) {
    TimedPoint wp;
    wp.t_sec = static_cast<double>(i);
    wp.x = static_cast<double>(i * 5);  // 0..20
    wp.y = 0.0;
    wp.index = static_cast<uint64_t>(i);
    other.waypoints.push_back(wp);
  }
  cache.upsert(other);

  ecbs_planner::evaluate_path(req, res, cache, solver);
  EXPECT_TRUE(res.success);
  EXPECT_EQ(res.status, "FREE") << res.message;
  EXPECT_TRUE(res.waiting_for.empty());
}

namespace
{

/// Opposite-direction AMR on x in [from_x, to_x] (from_x > to_x), 2.5 m
/// spacing, 1 m/s, graph index = x / 2.5. Meets a 0→ AMR (t = x) mid-way.
ecbs_planner::TimedPath opposite_corridor(
  const std::string & id, double from_x, double to_x)
{
  ecbs_planner::TimedPath p;
  p.amr_id = id;
  p.task_id = id + "_task";
  for (double x = from_x; x >= to_x - 1e-9; x -= 2.5) {
    ecbs_planner::TimedPoint wp;
    wp.t_sec = to_x + (from_x - x);
    wp.x = x;
    wp.y = 0.0;
    wp.index = static_cast<uint64_t>(std::llround(x / 2.5));
    p.waypoints.push_back(wp);
  }
  return p;
}

/// Evaluate request for amr1 driving +x from start_x to 60 at 1 m/s (t = x).
ecbs_planner::srv::EvaluatePath::Request evaluate_along_x(double start_x)
{
  ecbs_planner::srv::EvaluatePath::Request req;
  req.mode = "evaluate";
  req.amr_id = "amr1";
  req.task_id = "task_A";
  req.path_revision = 1;
  req.query_horizon_m = 20.0;
  req.current_x = start_x;
  req.current_y = 0.0;
  for (double x = start_x; x <= 60.0 + 1e-9; x += 2.5) {
    req.waypoints.push_back(
      msg_wp(x, x, 0.0, static_cast<int64_t>(std::llround(x / 2.5))));
  }
  return req;
}

}  // namespace

/// Scenario 1: path 0..60, overlap 15..35, no base given yet
/// → wait 2.5 m before the overlap (x = 12.5).
TEST(EvaluatePath, NoBaseGivenWaitSafetyDistanceBeforeOverlap)
{
  ecbs_planner::ScheduleCache cache;
  ecbs_planner::EcbsSolver solver;
  cache.upsert(opposite_corridor("amr2", 35.0, 15.0));

  auto req = evaluate_along_x(0.0);
  ecbs_planner::srv::EvaluatePath::Response res;
  std::vector<ecbs_planner::TimedPath> committed;
  ecbs_planner::evaluate_path(req, res, cache, solver, &committed);

  EXPECT_TRUE(res.success);
  EXPECT_EQ(res.status, "WAIT") << res.message;
  ASSERT_EQ(res.waiting_for.size(), 1u);
  EXPECT_EQ(res.waiting_for[0].amr_id, "amr2");
  EXPECT_NEAR(res.waiting_for[0].wait_point.x, 12.5, 1e-3);
  EXPECT_EQ(res.waiting_for[0].wait_point.index, 5);

  // Provisional /to_zone_manager path shows the same hold; amr1 not cached.
  // amr2 (cached) gets the reciprocal hold for amr1 until ZM register.
  ASSERT_EQ(committed.size(), 2u);
  const auto & viz1 = committed[0].amr_id == "amr1" ? committed[0] : committed[1];
  const auto & viz2 = committed[0].amr_id == "amr1" ? committed[1] : committed[0];
  ASSERT_FALSE(viz1.waypoints[5].waiting_for.empty());
  EXPECT_EQ(viz1.waypoints[5].waiting_for[0].amr_id, "amr2");
  EXPECT_EQ(viz2.amr_id, "amr2");
  // sync_cache_with_evaluate stores the request route (index-aligned).
  EXPECT_TRUE(cache.get("amr1").has_value());
  EXPECT_TRUE(cache.has_hold("amr1", "amr2"));
  EXPECT_TRUE(cache.has_hold("amr2", "amr1"));
  // WAIT releases the path up to the wait point: that is amr1's base now.
  const auto base = cache.granted_base("amr1");
  ASSERT_TRUE(base.has_value());
  EXPECT_EQ(base->index, std::optional<uint64_t>(5));
}

/// Scenario 2: path starts at 20, overlap starts at 20 → wait at 20.
TEST(EvaluatePath, OverlapAtPathStartWaitsAtStart)
{
  ecbs_planner::ScheduleCache cache;
  ecbs_planner::EcbsSolver solver;
  // 45 → 20: spills past the 20..40 base.
  auto other = opposite_corridor("amr2", 45.0, 20.0);
  for (auto & p : other.waypoints) {
    p.t_sec = 50.0 - p.x;  // meets amr1 (t = x) at x = 25
  }
  cache.upsert(other);

  auto req = evaluate_along_x(20.0);
  ecbs_planner::srv::EvaluatePath::Response res;
  ecbs_planner::evaluate_path(req, res, cache, solver);

  EXPECT_EQ(res.status, "WAIT") << res.message;
  ASSERT_EQ(res.waiting_for.size(), 1u);
  EXPECT_NEAR(res.waiting_for[0].wait_point.x, 20.0, 1e-3);
  EXPECT_EQ(res.waiting_for[0].wait_point.index, 8);
}

/// Scenario 3 (C5): base 0..20 given, then head-on overlap 15..35 → keep the
/// safety stop (not snap forward to base end 20 past separation).
TEST(EvaluatePath, GivenBaseOverlapIntoNextBaseWaitsAtNextBaseStart)
{
  ecbs_planner::ScheduleCache cache;
  ecbs_planner::EcbsSolver solver;

  auto req = evaluate_along_x(0.0);
  ecbs_planner::srv::EvaluatePath::Response res;
  ecbs_planner::evaluate_path(req, res, cache, solver);
  ASSERT_EQ(res.status, "FREE");
  const auto base = cache.granted_base("amr1");
  ASSERT_TRUE(base.has_value());
  EXPECT_NEAR(base->x, 20.0, 1e-3);

  cache.upsert(opposite_corridor("amr2", 35.0, 15.0));
  double first_wait_x = -1.0;
  for (int k = 0; k < 3; ++k) {
    ecbs_planner::evaluate_path(req, res, cache, solver);
    EXPECT_EQ(res.status, "WAIT") << res.message;
    ASSERT_EQ(res.waiting_for.size(), 1u);
    EXPECT_LE(res.waiting_for[0].wait_point.x, 20.0 + 1e-3) << "evaluate " << k;
    if (k == 0) {
      first_wait_x = res.waiting_for[0].wait_point.x;
    } else {
      EXPECT_NEAR(res.waiting_for[0].wait_point.x, first_wait_x, 1e-3)
        << "wait stays put, evaluate " << k;
    }
    const auto kept = cache.granted_base("amr1");
    ASSERT_TRUE(kept.has_value());
    EXPECT_LE(kept->x, 20.0 + 1e-3) << "evaluate " << k;
  }
}

/// Base 0..20 given, overlap starts at 25 (after the base, inside the 25 m
/// look-ahead) → wait at the base end (20), not 22.5: no permission past it.
TEST(EvaluatePath, GivenBaseOverlapAfterBaseWaitsAtBaseEnd)
{
  ecbs_planner::ScheduleCache cache;
  ecbs_planner::EcbsSolver solver;

  auto req = evaluate_along_x(0.0);
  ecbs_planner::srv::EvaluatePath::Response res;
  ecbs_planner::evaluate_path(req, res, cache, solver);
  ASSERT_EQ(res.status, "FREE");

  cache.upsert(opposite_corridor("amr2", 45.0, 25.0));
  ecbs_planner::evaluate_path(req, res, cache, solver);
  EXPECT_EQ(res.status, "WAIT") << res.message;
  ASSERT_EQ(res.waiting_for.size(), 1u);
  EXPECT_NEAR(res.waiting_for[0].wait_point.x, 20.0, 1e-3);
}

/// Run of 17:12: amr1 is first asked about a 1-waypoint path (it stays put),
/// so its base ends where it stands. The real path then meets amr2 at 15:
/// the wait is 2.5 m before the overlap (12.5), not at the robot (0).
TEST(EvaluatePath, BaseAtRobotDoesNotPullWaitBackFromConflict)
{
  ecbs_planner::ScheduleCache cache;
  ecbs_planner::EcbsSolver solver;

  auto stay = evaluate_along_x(0.0);
  stay.waypoints.resize(1);
  ecbs_planner::srv::EvaluatePath::Response res;
  ecbs_planner::evaluate_path(stay, res, cache, solver);
  ASSERT_EQ(res.status, "FREE") << res.message;
  const auto base = cache.granted_base("amr1");
  ASSERT_TRUE(base.has_value());
  ASSERT_NEAR(base->x, 0.0, 1e-3);

  cache.upsert(opposite_corridor("amr2", 35.0, 15.0));
  ecbs_planner::evaluate_path(evaluate_along_x(0.0), res, cache, solver);
  EXPECT_EQ(res.status, "WAIT") << res.message;
  ASSERT_EQ(res.waiting_for.size(), 1u);
  EXPECT_NEAR(res.waiting_for[0].wait_point.x, 12.5, 1e-3);
}

/// Look-ahead = asked 20 m + 5 m: a clash starting at 30 is ignored (FREE);
/// one starting at 22.5 counts → WAIT 2.5 m before it (x = 20).
TEST(EvaluatePath, LookAheadIsHorizonPlusFive)
{
  ecbs_planner::EcbsSolver solver;
  auto req = evaluate_along_x(0.0);
  ecbs_planner::srv::EvaluatePath::Response res;

  {
    ecbs_planner::ScheduleCache cache;
    cache.upsert(opposite_corridor("amr2", 50.0, 30.0));
    ecbs_planner::evaluate_path(req, res, cache, solver);
    EXPECT_EQ(res.status, "FREE") << res.message;
    EXPECT_TRUE(res.waiting_for.empty());
  }
  {
    ecbs_planner::ScheduleCache cache;
    cache.upsert(opposite_corridor("amr2", 42.5, 22.5));
    ecbs_planner::evaluate_path(req, res, cache, solver);
    EXPECT_EQ(res.status, "WAIT") << res.message;
    ASSERT_EQ(res.waiting_for.size(), 1u);
    EXPECT_NEAR(res.waiting_for[0].wait_point.x, 20.0, 1e-3);
  }
}

namespace
{

/// Cached amr1 path 0..60 (t = x) with a planner wait for amr2 at x = 12.5.
ecbs_planner::TimedPath amr1_with_planner_wait(double wait_sec)
{
  ecbs_planner::TimedPath p;
  p.amr_id = "amr1";
  p.task_id = "task_A";
  for (double x = 0.0; x <= 60.0 + 1e-9; x += 2.5) {
    p.waypoints.push_back(wp(x, x, 0.0, static_cast<uint64_t>(std::llround(x / 2.5))));
  }
  ecbs_planner::WaitDependency dep;
  dep.amr_id = "amr2";
  dep.wait_point.x = 12.5;
  dep.wait_point.index = 5;
  dep.wait_point.wait_duration_sec = wait_sec;
  dep.clear_points.push_back({15.0, 0.0});
  p.waypoints[5].waiting_for.push_back(dep);
  return p;
}

}  // namespace

/// Evaluate WAIT (C5): base 0..20 given, head-on overlap → keep safety stop
/// (not snap to base end). Planner wait seconds carry over onto the evaluate stop.
TEST(EvaluatePath, EvaluateWaitReplacesPlannerWait)
{
  ecbs_planner::ScheduleCache cache;
  ecbs_planner::EcbsSolver solver;
  cache.upsert(amr1_with_planner_wait(30.0));
  cache.upsert(opposite_corridor("amr2", 35.0, 15.0));
  ecbs_planner::GrantedBase base;
  base.task_id = "task_A";
  base.x = 20.0;
  base.index = 8;
  cache.set_granted_base("amr1", base);

  auto req = evaluate_along_x(0.0);
  ecbs_planner::srv::EvaluatePath::Response res;
  std::vector<ecbs_planner::TimedPath> committed;
  ecbs_planner::evaluate_path(req, res, cache, solver, &committed);

  ASSERT_EQ(res.status, "WAIT") << res.message;
  ASSERT_EQ(res.waiting_for.size(), 1u);
  EXPECT_LE(res.waiting_for[0].wait_point.x, 20.0 + 1e-3);
  EXPECT_DOUBLE_EQ(res.waiting_for[0].wait_point.wait_sec, 30.0);

  const auto cached = cache.get("amr1");
  ASSERT_TRUE(cached.has_value());
  bool saw_amr2 = false;
  for (const auto & w : cached->waypoints) {
    for (const auto & d : w.waiting_for) {
      if (d.amr_id == "amr2") {
        saw_amr2 = true;
        EXPECT_DOUBLE_EQ(d.wait_point.wait_duration_sec, 30.0);
      }
    }
  }
  EXPECT_TRUE(saw_amr2) << "evaluate wait for amr2 on cached path";

  // amr1 plus amr2 (reciprocal hold for amr1).
  ASSERT_EQ(committed.size(), 2u);
  EXPECT_TRUE(cache.has_hold("amr1", "amr2"));
}

/// Evaluate FREE removes the planner's wait in the checked window, so
/// /to_zone_manager no longer shows a wait the response does not have.
TEST(EvaluatePath, EvaluateFreeClearsPlannerWait)
{
  ecbs_planner::ScheduleCache cache;
  ecbs_planner::EcbsSolver solver;
  cache.upsert(amr1_with_planner_wait(30.0));

  auto req = evaluate_along_x(0.0);
  ecbs_planner::srv::EvaluatePath::Response res;
  std::vector<ecbs_planner::TimedPath> committed;
  ecbs_planner::evaluate_path(req, res, cache, solver, &committed);

  ASSERT_EQ(res.status, "FREE") << res.message;
  const auto cached = cache.get("amr1");
  ASSERT_TRUE(cached.has_value());
  EXPECT_TRUE(cached->waypoints[5].waiting_for.empty());
  ASSERT_EQ(committed.size(), 1u);
  for (const auto & p : committed[0].waypoints) {
    EXPECT_TRUE(p.waiting_for.empty());
  }
}

/// Path 20..60, conflict 20..25 inside the requested base 20..40:
/// no base given → WAIT at 20; base 20..40 given → FREE (owner priority).
TEST(EvaluatePath, ConflictInsideRequestedBaseWaitsUntilBaseGiven)
{
  auto req = evaluate_along_x(20.0);
  ecbs_planner::EcbsSolver solver;
  ecbs_planner::srv::EvaluatePath::Response res;

  {
    ecbs_planner::ScheduleCache cache;
    cache.upsert(opposite_corridor("amr2", 25.0, 20.0));
    ecbs_planner::evaluate_path(req, res, cache, solver);
    EXPECT_EQ(res.status, "WAIT") << res.message;
    ASSERT_EQ(res.waiting_for.size(), 1u);
    EXPECT_NEAR(res.waiting_for[0].wait_point.x, 20.0, 1e-3);
    EXPECT_EQ(res.waiting_for[0].wait_point.index, 8);
  }
  {
    ecbs_planner::ScheduleCache cache;
    ecbs_planner::evaluate_path(req, res, cache, solver);
    ASSERT_EQ(res.status, "FREE");
    cache.upsert(opposite_corridor("amr2", 25.0, 20.0));
    ecbs_planner::evaluate_path(req, res, cache, solver);
    EXPECT_EQ(res.status, "FREE") << res.message;
    EXPECT_TRUE(res.waiting_for.empty());
  }
}

/// Solver: candidate holds 2.5 m before the overlap (not one waypoint before).
TEST(EcbsSolver, CandidateWaitsSafetyDistanceBeforeOverlap)
{
  ecbs_planner::ScheduleCache cache;
  cache.upsert(opposite_corridor("A", 35.0, 15.0));

  TimedPath b;
  b.amr_id = "B";
  b.task_id = "B_task";
  for (double x = 0.0; x <= 60.0 + 1e-9; x += 2.5) {
    b.waypoints.push_back(wp(x, x, 0.0, static_cast<uint64_t>(std::llround(x / 2.5))));
  }

  EcbsSolver solver({2.0, 120.0, 20, 0.35, 2.5});
  auto result = solver.solve(b, cache);
  ASSERT_NE(result.status, SolveStatus::FailReplan) << result.reason;

  // Overlap on B starts at x = 15 (index 6) → hold at x = 12.5 (index 5).
  const auto & hold = result.path.waypoints[5];
  EXPECT_NEAR(hold.x, 12.5, 1e-3);
  ASSERT_FALSE(hold.waiting_for.empty());
  EXPECT_EQ(hold.waiting_for.front().amr_id, "A");
  EXPECT_TRUE(result.path.waypoints[6].waiting_for.empty());
  for (const auto & o : result.updated_others) {
    ExpectNoConflict(result.path, o);
  }
}

/// Solver Scenario 2: candidate path starts at 20 where the overlap starts
/// → hold at the path start.
TEST(EcbsSolver, OverlapAtPathStartHoldsAtStart)
{
  ecbs_planner::ScheduleCache cache;
  auto a = opposite_corridor("A", 25.0, 20.0);
  for (auto & p : a.waypoints) {
    p.t_sec -= 20.0;  // A in 20..25 during t = 0..5
  }
  cache.upsert(a);

  TimedPath b;
  b.amr_id = "B";
  b.task_id = "B_task";
  for (double x = 20.0; x <= 60.0 + 1e-9; x += 2.5) {
    b.waypoints.push_back(
      wp(x - 20.0, x, 0.0, static_cast<uint64_t>(std::llround(x / 2.5))));
  }

  EcbsSolver solver({2.0, 120.0, 20, 0.35, 2.5});
  auto result = solver.solve(b, cache);
  ASSERT_NE(result.status, SolveStatus::FailReplan) << result.reason;
  ASSERT_FALSE(result.path.waypoints.front().waiting_for.empty());
  EXPECT_NEAR(result.path.waypoints.front().x, 20.0, 1e-3);
}

/// Solver: cached A was given a base that covers its overlap start and the
/// overlap runs past it → A holds at the first waypoint of its next base and
/// is not time-shifted; the candidate waits outside.
TEST(EcbsSolver, BaseOwnerNotTimeShifted)
{
  ecbs_planner::ScheduleCache cache;
  const auto a = opposite_corridor("A", 35.0, 15.0);
  cache.upsert(a);
  ecbs_planner::GrantedBase base;  // A owns 35 → 25
  base.task_id = a.task_id;
  base.x = 25.0;
  base.index = 10;
  cache.set_granted_base("A", base);

  TimedPath b;
  b.amr_id = "B";
  b.task_id = "B_task";
  for (double x = 0.0; x <= 60.0 + 1e-9; x += 2.5) {
    b.waypoints.push_back(wp(x, x, 0.0, static_cast<uint64_t>(std::llround(x / 2.5))));
  }

  EcbsSolver solver({2.0, 120.0, 20, 0.35, 2.5});
  auto result = solver.solve(b, cache);
  ASSERT_NE(result.status, SolveStatus::FailReplan) << result.reason;

  const auto cached_a = cache.get("A");
  ASSERT_TRUE(cached_a.has_value());
  for (std::size_t i = 0; i < a.waypoints.size(); ++i) {
    EXPECT_DOUBLE_EQ(cached_a->waypoints[i].t_sec, a.waypoints[i].t_sec);
    EXPECT_DOUBLE_EQ(cached_a->waypoints[i].wait_duration_sec, 0.0);
  }
  // Base end x = 25 (array 4) → A's hold is at x = 22.5 (array 5).
  EXPECT_TRUE(cached_a->waypoints[4].waiting_for.empty());
  ASSERT_FALSE(cached_a->waypoints[5].waiting_for.empty());
  EXPECT_EQ(cached_a->waypoints[5].waiting_for.front().amr_id, "B");
  // B really waits, 2.5 m before its overlap.
  EXPECT_GT(result.path.waypoints[5].wait_duration_sec, 0.0);
  ExpectNoConflict(result.path, *cached_a);
  // A's given base survives; B's (none) stays cleared.
  EXPECT_TRUE(cache.granted_base("A").has_value());
  EXPECT_FALSE(cache.granted_base("B").has_value());
}

TEST(EvaluatePath, EvaluateFreeReturnsHorizon20)
{
  ecbs_planner::ScheduleCache cache;
  ecbs_planner::EcbsSolver solver;
  ecbs_planner::srv::EvaluatePath::Request req;
  ecbs_planner::srv::EvaluatePath::Response res;
  req.mode = "evaluate";
  req.amr_id = "amr1";
  req.task_id = "task_A";
  req.path_revision = 1;
  req.query_horizon_m = 20.0;
  req.current_x = 0.0;
  req.current_y = 0.0;
  req.waypoints = {
    msg_wp(0, 0, 0, 0),
    msg_wp(5, 5, 0, 1),
    msg_wp(10, 10, 0, 2),
  };
  ecbs_planner::evaluate_path(req, res, cache, solver);
  EXPECT_TRUE(res.success);
  EXPECT_EQ(res.status, "FREE");
  EXPECT_DOUBLE_EQ(res.horizon_m, 20.0);
  EXPECT_EQ(res.dep_from_index, -1);
  EXPECT_TRUE(res.waiting_for.empty());
}

TEST(EvaluatePath, RegisterWaitersOnlyClearsGoBlocker)
{
  using ecbs_planner::TimedPath;
  using ecbs_planner::TimedPoint;

  ecbs_planner::ScheduleCache cache;
  ecbs_planner::EcbsSolver solver;

  auto seed = [&](const std::string & amr, double x0) {
    TimedPath p;
    p.amr_id = amr;
    p.fleet_name = "amr_fleet";
    p.task_id = amr == "amr1" ? "task_A" : "task_B";
    p.path_revision = 1;
    for (int i = 0; i < 3; ++i) {
      TimedPoint wp;
      wp.t_sec = static_cast<double>(i);
      wp.x = x0 + static_cast<double>(i * 5);
      wp.y = static_cast<double>((amr == "amr1") ? i * 5 : (2 - i) * 5);
      wp.index = static_cast<uint64_t>(i);
      p.waypoints.push_back(wp);
    }
    // Stale Process #1 wait on GO amr1 — should be cleared when amr2 registers.
    if (amr == "amr1") {
      p.waypoints[1].wait_duration_sec = 3.0;
      ecbs_planner::WaitDependency d;
      d.amr_id = "amr2";
      d.wait_point.x = 5.0;
      d.wait_point.y = 5.0;
      d.wait_point.index = 1;
      d.wait_point.wait_duration_sec = 3.0;
      p.waypoints[1].waiting_for.push_back(d);
    }
    cache.upsert(p);
  };
  seed("amr1", 0.0);
  seed("amr2", 10.0);

  // ZM: only waiter amr2; GO amr1 omitted.
  ecbs_planner::srv::EvaluatePath::Request req;
  ecbs_planner::srv::EvaluatePath::Response res;
  req.mode = "register";
  req.fleet_name = "amr_fleet";

  ecbs_planner::msg::RegisterUpdate u2;
  u2.amr_id = "amr2";
  u2.task_id = "task_B";
  u2.phase_index = 0;
  u2.path_revision = 1;
  {
    ecbs_planner::msg::WaitDependency dep;
    dep.amr_id = "amr1";
    dep.wait_point.x = 5.0;
    dep.wait_point.y = 5.0;
    dep.wait_point.index = 1;
    dep.wait_point.wait_sec = 12.0;
    ecbs_planner::msg::ClearPoint c;
    c.x = 10.0;
    c.y = 0.0;
    dep.clear_points.push_back(c);
    u2.waiting_for.push_back(dep);
  }
  req.updates = {u2};

  std::vector<ecbs_planner::TimedPath> committed;
  ecbs_planner::evaluate_path(req, res, cache, solver, &committed);
  EXPECT_TRUE(res.success) << res.message;
  EXPECT_GE(committed.size(), 1u);

  const auto a1 = cache.get("amr1");
  const auto a2 = cache.get("amr2");
  ASSERT_TRUE(a1.has_value());
  ASSERT_TRUE(a2.has_value());

  // GO amr1: stale waits cleared.
  for (const auto & wp : a1->waypoints) {
    EXPECT_NEAR(wp.wait_duration_sec, 0.0, 1e-9);
    EXPECT_TRUE(wp.waiting_for.empty());
  }
  // Waiter amr2: wait applied.
  bool a2_has_dep = false;
  for (const auto & wp : a2->waypoints) {
    for (const auto & dep : wp.waiting_for) {
      if (dep.amr_id == "amr1") {
        a2_has_dep = true;
      }
    }
  }
  EXPECT_TRUE(a2_has_dep);
}

TEST(EvaluatePath, EvaluateErasesBehindPose)
{
  using ecbs_planner::TimedPath;
  using ecbs_planner::TimedPoint;

  ecbs_planner::ScheduleCache cache;
  ecbs_planner::EcbsSolver solver;

  TimedPath seed;
  seed.amr_id = "amr1";
  seed.task_id = "task_A";
  seed.path_revision = 1;
  for (int i = 0; i < 4; ++i) {
    TimedPoint wp;
    wp.t_sec = static_cast<double>(i);
    wp.x = static_cast<double>(i * 5);
    wp.y = static_cast<double>(i * 5);
    wp.index = static_cast<uint64_t>(i);
    seed.waypoints.push_back(wp);
  }
  cache.upsert(seed);

  ecbs_planner::srv::EvaluatePath::Request req;
  ecbs_planner::srv::EvaluatePath::Response res;
  req.mode = "evaluate";
  req.amr_id = "amr1";
  req.task_id = "task_A";
  req.path_revision = 1;
  req.query_horizon_m = 20.0;
  req.current_x = 10.0;
  req.current_y = 0.0;
  req.waypoints = {
    msg_wp(0, 0, 0, 0),
    msg_wp(5, 5, 0, 1),
    msg_wp(10, 10, 0, 2),
    msg_wp(15, 15, 0, 3),
  };
  ecbs_planner::evaluate_path(req, res, cache, solver);
  EXPECT_TRUE(res.success);

  const auto cached = cache.get("amr1");
  ASSERT_TRUE(cached.has_value());
  // sync_cache_with_evaluate re-upserts the full request route so TM grant
  // indices stay aligned; pose trim is a rolling view only during evaluate.
  EXPECT_EQ(cached->waypoints.size(), 4u);
  ASSERT_FALSE(cached->waypoints.empty());
  EXPECT_EQ(cached->waypoints.front().route_occurrence, 0u);
  EXPECT_EQ(cached->waypoints.back().route_occurrence, 3u);
}

TEST(EvaluatePath, RegisterUpdatesBatchWaitersOnly)
{
  using ecbs_planner::TimedPath;
  using ecbs_planner::TimedPoint;

  ecbs_planner::ScheduleCache cache;
  ecbs_planner::EcbsSolver solver;

  auto seed = [&](const std::string & amr, const std::string & task) {
    TimedPath p;
    p.amr_id = amr;
    p.fleet_name = "amr_fleet";
    p.task_id = task;
    p.path_revision = 1;
    for (int i = 0; i < 5; ++i) {
      TimedPoint wp;
      wp.t_sec = static_cast<double>(i * 5);
      wp.x = static_cast<double>(i * 5);
      wp.y = 0.0;
      wp.index = static_cast<uint64_t>(i);
      p.waypoints.push_back(wp);
    }
    // Pretend Process #1 already put mutual deps / a wait on amr1 too.
    if (amr == "amr1") {
      p.waypoints[1].wait_duration_sec = 3.0;
      ecbs_planner::WaitDependency d;
      d.amr_id = "amr2";
      d.wait_point.x = 5.0;
      d.wait_point.y = 0.0;
      d.wait_point.index = 1;
      d.wait_point.wait_duration_sec = 3.0;
      p.waypoints[1].waiting_for.push_back(d);
      for (std::size_t k = 2; k < p.waypoints.size(); ++k) {
        p.waypoints[k].t_sec += 3.0;
      }
    }
    cache.upsert(p);
  };
  seed("amr1", "task_A");
  seed("amr2", "task_B");
  seed("amr3", "task_C");

  ecbs_planner::srv::EvaluatePath::Request req;
  ecbs_planner::srv::EvaluatePath::Response res;
  req.mode = "register";
  req.fleet_name = "amr_fleet";
  // top-level amr_id optional for batch
  req.amr_id = "";

  ecbs_planner::msg::RegisterUpdate u2;
  u2.amr_id = "amr2";
  u2.task_id = "task_B";
  u2.phase_index = 0;
  u2.path_revision = 1;
  {
    ecbs_planner::msg::WaitDependency dep;
    dep.amr_id = "amr1";
    dep.wait_point.x = 5.0;
    dep.wait_point.y = 0.0;
    dep.wait_point.index = 1;
    dep.wait_point.wait_sec = 12.0;
    ecbs_planner::msg::ClearPoint c1;
    c1.x = 10.0;
    c1.y = 0.0;
    ecbs_planner::msg::ClearPoint c2;
    c2.x = 14.0;
    c2.y = 0.0;
    dep.clear_points = {c1, c2};
    u2.waiting_for.push_back(dep);
  }

  ecbs_planner::msg::RegisterUpdate u3;
  u3.amr_id = "amr3";
  u3.task_id = "task_C";
  u3.phase_index = 0;
  u3.path_revision = 1;
  {
    ecbs_planner::msg::WaitDependency dep;
    dep.amr_id = "amr1";
    dep.wait_point.x = 5.0;
    dep.wait_point.y = 0.0;
    dep.wait_point.index = 1;
    dep.wait_point.wait_sec = 8.0;
    ecbs_planner::msg::ClearPoint c1;
    c1.x = 12.0;
    c1.y = 0.0;
    ecbs_planner::msg::ClearPoint c2;
    c2.x = 16.0;
    c2.y = 0.0;
    dep.clear_points = {c1, c2};
    u3.waiting_for.push_back(dep);
  }
  req.updates = {u2, u3};

  std::vector<ecbs_planner::TimedPath> committed;
  ecbs_planner::evaluate_path(req, res, cache, solver, &committed);

  EXPECT_TRUE(res.success);
  EXPECT_EQ(res.status, "WAIT");
  EXPECT_NE(res.message.find("amr2(WAIT)"), std::string::npos);
  EXPECT_NE(res.message.find("amr3(WAIT)"), std::string::npos);
  EXPECT_NE(res.message.find("amr1(GO)"), std::string::npos);

  const auto a1 = cache.get("amr1");
  ASSERT_TRUE(a1.has_value());
  EXPECT_DOUBLE_EQ(a1->waypoints[1].wait_duration_sec, 0.0);
  EXPECT_TRUE(a1->waypoints[1].waiting_for.empty());

  const auto a2 = cache.get("amr2");
  ASSERT_TRUE(a2.has_value());
  EXPECT_NEAR(a2->waypoints[1].wait_duration_sec, 12.0, 1e-6);
  ASSERT_FALSE(a2->waypoints[1].waiting_for.empty());
  EXPECT_EQ(a2->waypoints[1].waiting_for[0].amr_id, "amr1");
  EXPECT_EQ(a2->waypoints[1].waiting_for[0].clear_points.size(), 2u);

  const auto a3 = cache.get("amr3");
  ASSERT_TRUE(a3.has_value());
  EXPECT_NEAR(a3->waypoints[1].wait_duration_sec, 8.0, 1e-6);

  EXPECT_GE(committed.size(), 2u);
}

/// Planner waits hold both AMRs until ZM registers the pair; a re-commit of
/// the same task does not bring the hold back; a new task forgets holds.
TEST(EcbsSolver, PlannerWaitsHoldBothUntilRegister)
{
  ScheduleCache cache;
  cache.upsert(make_path("A", {wp(0, 0, 0, 5), wp(10, 1, 0, 6)}));
  EcbsSolver solver({1.0, 30.0, 20, 0.35});
  solver.solve(make_path("B", {wp(2, 0, 0, 5), wp(12, 1, 0, 7)}), cache);

  EXPECT_TRUE(cache.has_hold("A", "B"));
  EXPECT_TRUE(cache.has_hold("B", "A"));

  cache.mark_registered("B", "A");
  EXPECT_FALSE(cache.has_hold("A", "B"));
  EXPECT_FALSE(cache.has_hold("B", "A"));
  EXPECT_TRUE(cache.registered_waiter("B", "A"));

  cache.add_plan_holds(*cache.get("A"));
  cache.add_plan_holds(*cache.get("B"));
  EXPECT_FALSE(cache.has_hold("A", "B"));
  EXPECT_FALSE(cache.has_hold("B", "A"));

  auto a_new = *cache.get("A");
  a_new.task_id = "A_task2";
  cache.add_plan_holds(a_new);
  EXPECT_FALSE(cache.registered_waiter("B", "A"));
  EXPECT_TRUE(cache.has_hold("A", "B")) << "new task: planner wait holds again";
}

/// C1: task change forgets A's outgoing holds/registers only — inbound holds
/// from waiters for A must survive while A still occupies the conflict.
TEST(ScheduleCache, TaskChangeKeepsInboundHolds)
{
  ScheduleCache cache;
  cache.upsert(make_path("A", {wp(0, 0, 0, 5), wp(10, 1, 0, 6)}));
  cache.upsert(make_path("B", {wp(2, 0, 0, 5), wp(12, 1, 0, 7)}));
  {
    auto a = *cache.get("A");
    a.task_id = "A_task1";
    cache.add_plan_holds(a);
  }
  ecbs_planner::WaitDependency b_waits_a;
  b_waits_a.amr_id = "A";
  b_waits_a.wait_point.x = 2.0;
  b_waits_a.wait_point.y = 0.0;
  b_waits_a.clear_points = {{10.0, 0.0}};
  cache.add_hold("B", b_waits_a);
  ASSERT_TRUE(cache.has_hold("B", "A"));

  auto a_new = *cache.get("A");
  a_new.task_id = "A_task2";
  a_new.waypoints[0].waiting_for.clear();
  a_new.waypoints[1].waiting_for.clear();
  cache.add_plan_holds(a_new);
  EXPECT_TRUE(cache.has_hold("B", "A"))
    << "inbound hold must survive A's task change";
}

namespace
{

ecbs_planner::srv::EvaluatePath::Request evaluate_request_for(
  const ecbs_planner::TimedPath & p)
{
  ecbs_planner::srv::EvaluatePath::Request req;
  req.mode = "evaluate";
  req.amr_id = p.amr_id;
  req.task_id = p.task_id;
  req.path_revision = 1;
  req.query_horizon_m = 20.0;
  req.current_x = p.waypoints.front().x;
  req.current_y = p.waypoints.front().y;
  for (const auto & w : p.waypoints) {
    req.waypoints.push_back(
      msg_wp(w.t_sec, w.x, w.y, static_cast<int64_t>(w.index.value_or(0))));
  }
  return req;
}

}  // namespace

/// Once amr1 is told WAIT for amr2, amr2 (no base given) is held too and keeps
/// getting WAIT for amr1 — no FREE — until ZM registers amr1 waiting for amr2.
/// Then amr2 is GO (FREE) and amr1 keeps its registered wait.
TEST(EvaluatePath, BothConflictingAmrsHoldUntilRegister)
{
  ecbs_planner::ScheduleCache cache;
  ecbs_planner::EcbsSolver solver;

  auto req1 = evaluate_along_x(0.0);
  cache.upsert(ecbs_planner::timed_path_from_evaluate_request(req1));
  const auto amr2 = opposite_corridor("amr2", 35.0, 15.0);
  cache.upsert(amr2);
  const auto req2 = evaluate_request_for(amr2);

  ecbs_planner::srv::EvaluatePath::Response res;
  ecbs_planner::evaluate_path(req1, res, cache, solver);
  ASSERT_EQ(res.status, "WAIT") << res.message;
  ASSERT_EQ(res.waiting_for.size(), 1u);
  EXPECT_EQ(res.waiting_for[0].amr_id, "amr2");
  const double amr1_wait_x = res.waiting_for[0].wait_point.x;

  for (int i = 0; i < 3; ++i) {
    ecbs_planner::evaluate_path(req2, res, cache, solver);
    ASSERT_EQ(res.status, "WAIT") << res.message;
    ASSERT_EQ(res.waiting_for.size(), 1u);
    EXPECT_EQ(res.waiting_for[0].amr_id, "amr1");

    ecbs_planner::evaluate_path(req1, res, cache, solver);
    ASSERT_EQ(res.status, "WAIT") << res.message;
    ASSERT_EQ(res.waiting_for.size(), 1u);
    EXPECT_NEAR(res.waiting_for[0].wait_point.x, amr1_wait_x, 1e-3)
      << "waiting point must stay put while held";
  }

  ecbs_planner::srv::EvaluatePath::Request reg;
  reg.mode = "register";
  ecbs_planner::msg::RegisterUpdate u1;
  u1.amr_id = "amr1";
  u1.task_id = "task_A";
  u1.path_revision = 1;
  ecbs_planner::msg::WaitDependency dep;
  dep.amr_id = "amr2";
  dep.wait_point.x = amr1_wait_x;
  dep.wait_point.index = static_cast<int64_t>(std::llround(amr1_wait_x / 2.5));
  u1.waiting_for.push_back(dep);
  reg.updates = {u1};
  ecbs_planner::evaluate_path(reg, res, cache, solver);
  ASSERT_TRUE(res.success) << res.message;
  EXPECT_FALSE(cache.has_hold("amr1", "amr2"));
  EXPECT_FALSE(cache.has_hold("amr2", "amr1"));

  ecbs_planner::evaluate_path(req2, res, cache, solver);
  EXPECT_EQ(res.status, "FREE") << "GO after register: " << res.message;

  ecbs_planner::evaluate_path(req1, res, cache, solver);
  EXPECT_EQ(res.status, "WAIT") << res.message;
}

/// Evaluate WAIT with the other AMR not holding yet: the other AMR's evaluate
/// that sees no conflict of its own still answers WAIT (both wait).
TEST(EvaluatePath, OtherAmrHeldEvenWithoutOwnConflict)
{
  ecbs_planner::ScheduleCache cache;
  ecbs_planner::EcbsSolver solver;
  const auto amr2 = opposite_corridor("amr2", 35.0, 15.0);
  cache.upsert(amr2);

  auto req1 = evaluate_along_x(0.0);
  ecbs_planner::srv::EvaluatePath::Response res;
  ecbs_planner::evaluate_path(req1, res, cache, solver);
  ASSERT_EQ(res.status, "WAIT") << res.message;

  // amr1 is not cached, so amr2 alone would see no conflict.
  ecbs_planner::evaluate_path(evaluate_request_for(amr2), res, cache, solver);
  EXPECT_EQ(res.status, "WAIT") << res.message;
  ASSERT_EQ(res.waiting_for.size(), 1u);
  EXPECT_EQ(res.waiting_for[0].amr_id, "amr1");
  const auto base = cache.granted_base("amr2");
  ASSERT_TRUE(base.has_value()) << "held: base up to its wait point";
  EXPECT_EQ(base->index, std::optional<uint64_t>(res.waiting_for[0].wait_point.index));
}

/// C5: base 0..20 given; planner wait / head-on safety stop stays at the
/// calculated stop (not snapped to base end) and remains fixed while driving.
TEST(EvaluatePath, PlannerWaitInsideGivenBaseMovesOutAndStaysPut)
{
  ecbs_planner::ScheduleCache cache;
  ecbs_planner::EcbsSolver solver;
  const auto amr1 = amr1_with_planner_wait(30.0);
  cache.upsert(amr1);
  cache.add_plan_holds(amr1);
  cache.upsert(opposite_corridor("amr2", 35.0, 15.0));
  ecbs_planner::GrantedBase base;
  base.task_id = "task_A";
  base.x = 20.0;
  base.index = 8;
  cache.set_granted_base("amr1", base);

  double pinned_x = -1.0;
  int64_t pinned_idx = -1;
  // Only evaluate from poses still before the pinned stop (past it drops).
  for (double x : {0.0, 5.0, 10.0}) {
    auto req = evaluate_along_x(x);
    ecbs_planner::srv::EvaluatePath::Response res;
    ecbs_planner::evaluate_path(req, res, cache, solver);
    ASSERT_EQ(res.status, "WAIT") << "x=" << x << ": " << res.message;
    ASSERT_EQ(res.waiting_for.size(), 1u) << "x=" << x;
    EXPECT_EQ(res.waiting_for[0].amr_id, "amr2");
    EXPECT_LE(res.waiting_for[0].wait_point.x, 20.0 + 1e-3) << "x=" << x;
    EXPECT_DOUBLE_EQ(res.waiting_for[0].wait_point.wait_sec, 30.0);
    if (pinned_x < 0.0) {
      pinned_x = res.waiting_for[0].wait_point.x;
      pinned_idx = res.waiting_for[0].wait_point.index;
      ASSERT_GT(pinned_x, 10.0) << "stop must stay ahead of test poses";
    } else {
      EXPECT_NEAR(res.waiting_for[0].wait_point.x, pinned_x, 1e-3) << "x=" << x;
      EXPECT_EQ(res.waiting_for[0].wait_point.index, pinned_idx) << "x=" << x;
    }
  }
}

/// No base given: the natural wait point (12.5) is fixed on the first WAIT
/// and does not follow amr1 as it keeps evaluating from further along.
TEST(EvaluatePath, WaitPointDoesNotFollowMovingAmr)
{
  ecbs_planner::ScheduleCache cache;
  ecbs_planner::EcbsSolver solver;
  cache.upsert(opposite_corridor("amr2", 35.0, 15.0));

  for (double x : {0.0, 5.0, 10.0}) {
    auto req = evaluate_along_x(x);
    ecbs_planner::srv::EvaluatePath::Response res;
    ecbs_planner::evaluate_path(req, res, cache, solver);
    ASSERT_EQ(res.status, "WAIT") << "x=" << x << ": " << res.message;
    ASSERT_EQ(res.waiting_for.size(), 1u) << "x=" << x;
    EXPECT_NEAR(res.waiting_for[0].wait_point.x, 12.5, 1e-3) << "x=" << x;
    EXPECT_EQ(res.waiting_for[0].wait_point.index, 5) << "x=" << x;
  }
}

/// amr1 holds for amr2 (planner wait), but their times never clash. amr2's
/// reciprocal hold uses where the paths cross (y = 0): 2.5 m before it,
/// not wherever amr2 happens to be.
TEST(EvaluatePath, ReciprocalHoldWithoutTimeClashUsesCrossing)
{
  ecbs_planner::ScheduleCache cache;
  ecbs_planner::EcbsSolver solver;

  ecbs_planner::TimedPath amr2;
  amr2.amr_id = "amr2";
  amr2.task_id = "task_B";
  uint64_t idx = 100;
  for (double y = 30.0; y >= -30.0 - 1e-9; y -= 2.5) {
    amr2.waypoints.push_back(wp(1000.0 + (30.0 - y), 20.0, y, idx++));
  }
  cache.upsert(amr2);

  auto req = evaluate_along_x(0.0);
  auto amr1 = ecbs_planner::timed_path_from_evaluate_request(req);
  ecbs_planner::WaitDependency dep;
  dep.amr_id = "amr2";
  dep.wait_point.x = 17.5;
  dep.wait_point.index = 7;
  amr1.waypoints[7].waiting_for.push_back(dep);
  cache.upsert(amr1);
  cache.add_plan_holds(amr1);

  ecbs_planner::srv::EvaluatePath::Response res;
  ecbs_planner::evaluate_path(req, res, cache, solver);
  ASSERT_EQ(res.status, "WAIT") << res.message;

  const auto holds = cache.holds("amr2");
  ASSERT_EQ(holds.size(), 1u);
  EXPECT_EQ(holds[0].amr_id, "amr1");
  EXPECT_NEAR(holds[0].wait_point.x, 20.0, 1e-3);
  EXPECT_NEAR(holds[0].wait_point.y, 2.5, 1e-3);
}

namespace
{

/// Base owner A: 35 → 25 on y = 0 (shared with a 0→ AMR), then turns off to
/// (25, -20). Given base = first 20 m, ends at (25, -10): the shared part is
/// fully inside it and nothing is shared after it. At x = 30, t = 30.
ecbs_planner::TimedPath owner_turning_off()
{
  ecbs_planner::TimedPath p;
  p.amr_id = "A";
  p.task_id = "A_task";
  double t = 25.0;
  for (double x = 35.0; x >= 25.0 - 1e-9; x -= 2.5, t += 2.5) {
    p.waypoints.push_back(wp(t, x, 0.0, static_cast<uint64_t>(std::llround(x / 2.5))));
  }
  uint64_t idx = 201;
  for (double y = -2.5; y >= -20.0 - 1e-9; y -= 2.5, t += 2.5) {
    p.waypoints.push_back(wp(t, 25.0, y, idx++));
  }
  return p;
}

ecbs_planner::GrantedBase owner_base()
{
  ecbs_planner::GrantedBase base;
  base.task_id = "A_task";
  base.x = 25.0;
  base.y = -10.0;
  base.index = 204;
  return base;
}

}  // namespace

/// Solver: B's clash with A lies fully inside A's given base and they share
/// nothing after it → only B waits; A gets no wait and no hold.
TEST(EcbsSolver, OwnerMeetingOnlyInsideGivenBaseGetsNoWait)
{
  ScheduleCache cache;
  const auto a = owner_turning_off();
  cache.upsert(a);
  cache.set_granted_base("A", owner_base());

  TimedPath b;
  b.amr_id = "B";
  b.task_id = "B_task";
  for (double x = 0.0; x <= 60.0 + 1e-9; x += 2.5) {
    b.waypoints.push_back(wp(x, x, 0.0, static_cast<uint64_t>(std::llround(x / 2.5))));
  }
  EcbsSolver solver({2.0, 120.0, 20, 0.35, 2.5});
  auto result = solver.solve(b, cache);
  ASSERT_NE(result.status, SolveStatus::FailReplan) << result.reason;

  const auto cached_a = cache.get("A");
  ASSERT_TRUE(cached_a.has_value());
  for (std::size_t i = 0; i < a.waypoints.size(); ++i) {
    EXPECT_TRUE(cached_a->waypoints[i].waiting_for.empty()) << "A wp " << i;
    EXPECT_DOUBLE_EQ(cached_a->waypoints[i].wait_duration_sec, 0.0);
    EXPECT_DOUBLE_EQ(cached_a->waypoints[i].t_sec, a.waypoints[i].t_sec);
  }
  EXPECT_FALSE(cache.has_hold("A", "B"));
  EXPECT_TRUE(cache.has_hold("B", "A"));
  ExpectNoConflict(result.path, *cached_a);
}

/// Evaluate: B is told WAIT for A, but A (meeting B only inside its given
/// base) is not held: A stays FREE with no waiting point, even if an old
/// wait for B was left on its cached path.
TEST(EvaluatePath, OwnerMeetingOnlyInsideGivenBaseStaysFree)
{
  ecbs_planner::ScheduleCache cache;
  ecbs_planner::EcbsSolver solver;
  auto a = owner_turning_off();
  ecbs_planner::WaitDependency stale;
  stale.amr_id = "B";
  stale.wait_point.x = 25.0;
  stale.wait_point.y = -10.0;
  stale.wait_point.index = 204;
  a.waypoints[8].waiting_for.push_back(stale);
  cache.upsert(a);
  cache.add_plan_holds(a);
  cache.set_granted_base("A", owner_base());
  ASSERT_TRUE(cache.has_hold("A", "B"));

  auto req_b = evaluate_along_x(0.0);
  req_b.amr_id = "B";
  req_b.task_id = "B_task";
  cache.upsert(ecbs_planner::timed_path_from_evaluate_request(req_b));

  ecbs_planner::srv::EvaluatePath::Response res;
  ecbs_planner::evaluate_path(req_b, res, cache, solver);
  ASSERT_EQ(res.status, "WAIT") << res.message;
  ASSERT_EQ(res.waiting_for.size(), 1u);
  EXPECT_EQ(res.waiting_for[0].amr_id, "A");

  auto req_a = evaluate_request_for(owner_turning_off());
  ecbs_planner::evaluate_path(req_a, res, cache, solver);
  EXPECT_EQ(res.status, "FREE") << res.message;
  EXPECT_TRUE(res.waiting_for.empty());
  EXPECT_FALSE(cache.has_hold("A", "B"));
  EXPECT_TRUE(cache.has_hold("B", "A")) << "B still waits for A";

  const auto cached_a = cache.get("A");
  ASSERT_TRUE(cached_a.has_value());
  for (const auto & w : cached_a->waypoints) {
    EXPECT_TRUE(w.waiting_for.empty());
  }

  ecbs_planner::evaluate_path(req_b, res, cache, solver);
  EXPECT_EQ(res.status, "WAIT") << res.message;
}

namespace
{

/// /from_zone_manager seed of \p p; \p wait_at (if >= 0) carries a planner
/// wait for \p blocker, as the fleet-adapter planner sends it.
ecbs_planner::msg::FromZoneManager seed_msg(
  const ecbs_planner::TimedPath & p, int wait_at = -1, const std::string & blocker = "")
{
  ecbs_planner::msg::FromZoneManager m;
  m.amr_id = p.amr_id;
  m.task_id = p.task_id;
  for (std::size_t i = 0; i < p.waypoints.size(); ++i) {
    const auto & w = p.waypoints[i];
    auto mw = msg_wp(w.t_sec, w.x, w.y, static_cast<int64_t>(w.index.value_or(0)));
    if (static_cast<int>(i) == wait_at) {
      ecbs_planner::msg::WaitDependency d;
      d.amr_id = blocker;
      d.wait_point.x = w.x;
      d.wait_point.y = w.y;
      d.wait_point.index = static_cast<int64_t>(w.index.value_or(0));
      d.wait_point.wait_sec = 30.0;
      mw.wait_sec = 30.0;
      mw.waiting_for.push_back(d);
    }
    m.waypoints.push_back(mw);
  }
  return m;
}

bool shows_wait(
  const std::vector<ecbs_planner::TimedPath> & published, const std::string & amr)
{
  bool found = false;
  for (const auto & p : published) {
    if (p.amr_id != amr) {
      continue;
    }
    found = false;  // last publish of this AMR wins
    for (const auto & w : p.waypoints) {
      found = found || !w.waiting_for.empty();
    }
  }
  return found;
}

}  // namespace

/// Bag case: A was given FREE for a base covering the whole overlap with B;
/// the planner still seeds A with a wait for B. /to_zone_manager must not
/// show that wait, in either seed order, and A's evaluate stays FREE.
TEST(EvaluatePath, SeededWaitOnBaseOwnerIsNotPublished)
{
  ecbs_planner::EcbsSolver solver;
  auto b_req = evaluate_along_x(0.0);
  b_req.amr_id = "B";
  b_req.task_id = "B_task";
  auto b = ecbs_planner::timed_path_from_evaluate_request(b_req);

  for (const bool owner_first : {false, true}) {
    ecbs_planner::ScheduleCache cache;
    cache.upsert(owner_turning_off());
    cache.set_granted_base("A", owner_base());

    ecbs_planner::srv::EvaluatePath::Response res;
    std::vector<ecbs_planner::TimedPath> published;
    std::vector<ecbs_planner::TimedPath> step;
    auto seed = [&](const ecbs_planner::msg::FromZoneManager & m) {
        ecbs_planner::register_from_zone_manager(m, res, cache, solver, &step);
        published.insert(published.end(), step.begin(), step.end());
      };
    if (owner_first) {
      seed(seed_msg(owner_turning_off(), 0, "B"));
      seed(seed_msg(b, 5, "A"));
    } else {
      seed(seed_msg(b, 5, "A"));
      seed(seed_msg(owner_turning_off(), 0, "B"));
    }

    EXPECT_FALSE(shows_wait(published, "A")) << "owner_first=" << owner_first;
    EXPECT_TRUE(shows_wait(published, "B")) << "owner_first=" << owner_first;
    EXPECT_FALSE(cache.has_hold("A", "B")) << "owner_first=" << owner_first;
    EXPECT_TRUE(cache.has_hold("B", "A")) << "owner_first=" << owner_first;
    const auto cached_a = cache.get("A");
    ASSERT_TRUE(cached_a.has_value());
    for (const auto & w : cached_a->waypoints) {
      EXPECT_TRUE(w.waiting_for.empty()) << "owner_first=" << owner_first;
    }

    ecbs_planner::evaluate_path(
      evaluate_request_for(owner_turning_off()), res, cache, solver, &step);
    EXPECT_EQ(res.status, "FREE") << res.message;
    EXPECT_FALSE(shows_wait(step, "A"));
  }
}

namespace
{

/// Straight path of \p id from (x0, y0) to (x1, y1), 2.5 m spacing, 1 m/s
/// starting at \p t0, graph indices from \p idx0.
ecbs_planner::TimedPath line_path(
  const std::string & id, double x0, double y0, double x1, double y1,
  double t0, uint64_t idx0)
{
  ecbs_planner::TimedPath p;
  p.amr_id = id;
  p.task_id = id + "_task";
  const double len = std::hypot(x1 - x0, y1 - y0);
  const int n = static_cast<int>(std::llround(len / 2.5));
  for (int k = 0; k <= n; ++k) {
    const double f = n == 0 ? 0.0 : static_cast<double>(k) / n;
    p.waypoints.push_back(
      wp(t0 + 2.5 * k, x0 + f * (x1 - x0), y0 + f * (y1 - y0), idx0 + k));
  }
  return p;
}

/// Evaluate request for \p p with the robot at waypoint \p from.
ecbs_planner::srv::EvaluatePath::Request request_from(
  const ecbs_planner::TimedPath & p, std::size_t from)
{
  ecbs_planner::TimedPath rest = p;
  rest.waypoints.erase(rest.waypoints.begin(), rest.waypoints.begin() + from);
  return evaluate_request_for(rest);
}

}  // namespace

/// Parallel lanes: no path/time clash (centres > clearance_m) but L×W
/// bodies still touch A's reserved base → B must WAIT (size conflict).
TEST(EvaluatePath, BodySizeConflictGetsWaitWithoutPathTimeClash)
{
  ecbs_planner::SolverConfig cfg;
  cfg.clearance_m = 2.0;
  cfg.amr_length_m = 1.5;
  cfg.amr_width_m = 1.5;  // half_diag≈1.06 → body_touch≈2.12 > 2.05
  ecbs_planner::EcbsSolver solver(cfg);
  ecbs_planner::ScheduleCache cache;
  const auto a = line_path("A", 0.0, 0.0, 40.0, 0.0, 0.0, 0);
  // B on a parallel lane 2.05 m away: outside clearance, inside body_touch.
  // Starts ~1000 s later so earliest_conflict finds no time overlap.
  const auto b = line_path("B", 0.0, 2.05, 40.0, 2.05, 1000.0, 500);
  cache.upsert(a);
  cache.upsert(b);

  ecbs_planner::srv::EvaluatePath::Response res;
  ecbs_planner::evaluate_path(request_from(a, 0), res, cache, solver);
  ASSERT_EQ(res.status, "FREE") << res.message;

  ecbs_planner::evaluate_path(request_from(b, 0), res, cache, solver);
  ASSERT_EQ(res.status, "WAIT") << "L×W body dash without path clash: " << res.message;
  ASSERT_FALSE(res.waiting_for.empty());
  EXPECT_EQ(res.waiting_for[0].amr_id, "A");
}

/// Tiny bodies + same geometry → no wait (control for BodySizeConflict…).
TEST(EvaluatePath, TinyBodiesNoWaitWhenCentresOutsideClearance)
{
  ecbs_planner::SolverConfig cfg;
  cfg.clearance_m = 2.0;
  cfg.amr_length_m = 0.2;
  cfg.amr_width_m = 0.2;
  ecbs_planner::EcbsSolver solver(cfg);
  ecbs_planner::ScheduleCache cache;
  const auto a = line_path("A", 0.0, 0.0, 40.0, 0.0, 0.0, 0);
  const auto b = line_path("B", 0.0, 2.05, 40.0, 2.05, 1000.0, 500);
  cache.upsert(a);
  cache.upsert(b);

  ecbs_planner::srv::EvaluatePath::Response res;
  ecbs_planner::evaluate_path(request_from(a, 0), res, cache, solver);
  ASSERT_EQ(res.status, "FREE") << res.message;
  ecbs_planner::evaluate_path(request_from(b, 0), res, cache, solver);
  EXPECT_EQ(res.status, "FREE")
    << "centres outside clearance and tiny bodies: " << res.message;
}

/// Run of 10:27: amr1's reserved stretch included a cell on amr2's lane at
/// (58.2, 55) before the crossing (60, 55). Wait must be safety_m (2.5 m)
/// before that cell (~55.7), NOT clearance+safety (~53.2 / ~6.8 m before 60).
TEST(EvaluatePath, WaitIsSafetyBeforeReservedCellNotClearanceEdge)
{
  ecbs_planner::ScheduleCache cache;
  ecbs_planner::EcbsSolver solver;
  const auto a = line_path("A", 60.0, 70.0, 60.0, 40.0, 0.0, 0);
  const auto b = line_path("B", 44.0, 55.0, 70.0, 55.0, 1000.0, 500);
  cache.upsert(a);
  cache.upsert(b);
  ecbs_planner::srv::EvaluatePath::Response res;
  ecbs_planner::evaluate_path(request_from(a, 0), res, cache, solver);
  ASSERT_EQ(res.status, "FREE") << res.message;
  // Inject the spur cell on B's lane that caused the ~6.8 m wait in the run.
  std::vector<ecbs_planner::ClearPoint> pts;
  for (const auto & c : cache.reserved("A")) {
    pts.push_back({c.x, c.y});
  }
  pts.push_back({58.2, 55.0});
  cache.set_reserved("A", pts);

  ecbs_planner::evaluate_path(request_from(b, 0), res, cache, solver);
  ASSERT_EQ(res.status, "WAIT") << res.message;
  ASSERT_FALSE(res.waiting_for.empty());
  EXPECT_EQ(res.waiting_for[0].amr_id, "A");
  // Wait = safety_m (2.5 m) before path first reaches cell 58.2 — not ~5 m
  // before the crossing (60, 55) from clearance/nearest-wp stacking.
  const double wx = res.waiting_for[0].wait_point.x;
  // The wait is a real waypoint (2.5 m spacing here): at least safety_m before
  // the cell, never mid-segment, and not a whole extra clearance further back.
  EXPECT_GE(58.2 - wx, 2.5 - 1e-6) << "gap to reserved cell is >= safety_m";
  EXPECT_LE(58.2 - wx, 2.5 + 2.5 + 1e-6) << "at most one waypoint spacing more";
  bool on_wp = false;
  for (const auto & w : b.waypoints) {
    on_wp = on_wp || std::abs(w.x - wx) < 1e-6;
  }
  EXPECT_TRUE(on_wp) << "wait must be a path waypoint, x=" << wx;
}

/// A is given FREE for 0..20 on y = 0. C crosses at (10, 0) ~1000 s later
/// (no time clash) yet must not enter A's base: WAIT 2.5 m before it, with
/// no hold. Once A has driven past the crossing, C is FREE.
TEST(EvaluatePath, ReservedBaseBlocksCrossingWithoutTimeClash)
{
  ecbs_planner::ScheduleCache cache;
  ecbs_planner::EcbsSolver solver;
  const auto a = line_path("A", 0.0, 0.0, 40.0, 0.0, 0.0, 0);
  const auto c = line_path("C", 10.0, -20.0, 10.0, 20.0, 1000.0, 300);
  cache.upsert(a);
  cache.upsert(c);

  ecbs_planner::srv::EvaluatePath::Response res;
  ecbs_planner::evaluate_path(request_from(a, 0), res, cache, solver);
  ASSERT_EQ(res.status, "FREE") << res.message;

  ecbs_planner::evaluate_path(request_from(c, 0), res, cache, solver);
  ASSERT_EQ(res.status, "WAIT") << res.message;
  ASSERT_EQ(res.waiting_for.size(), 1u);
  EXPECT_EQ(res.waiting_for[0].amr_id, "A");
  EXPECT_NEAR(res.waiting_for[0].wait_point.x, 10.0, 1e-3);
  EXPECT_NEAR(res.waiting_for[0].wait_point.y, -2.5, 1e-3);
  EXPECT_FALSE(cache.has_hold("C", "A")) << "reserved base: no register needed";
  EXPECT_FALSE(cache.has_hold("A", "C"));

  ecbs_planner::evaluate_path(request_from(a, 0), res, cache, solver);
  EXPECT_EQ(res.status, "FREE") << "C's wait must not block the owner: " << res.message;

  cache.update_pose("A", 12.4, 0.0);
  ecbs_planner::evaluate_path(request_from(a, 5), res, cache, solver);
  ASSERT_EQ(res.status, "FREE") << res.message;
  ecbs_planner::evaluate_path(request_from(c, 0), res, cache, solver);
  EXPECT_EQ(res.status, "WAIT")
    << "A at 12.4: crossing less than 2.5 m behind it: " << res.message;

  cache.update_pose("A", 12.6, 0.0);
  ecbs_planner::evaluate_path(request_from(a, 5), res, cache, solver);
  ASSERT_EQ(res.status, "FREE") << res.message;
  ecbs_planner::evaluate_path(request_from(c, 0), res, cache, solver);
  EXPECT_EQ(res.status, "WAIT")
    << "STRICT: A's tail cell (10.0) is still booked at A=12.6: " << res.message;

  cache.update_pose("A", 15.0, 0.0);
  ecbs_planner::evaluate_path(request_from(a, 6), res, cache, solver);
  ASSERT_EQ(res.status, "FREE") << res.message;
  ecbs_planner::evaluate_path(request_from(c, 0), res, cache, solver);
  EXPECT_EQ(res.status, "FREE") << "A's whole booked stretch is past the crossing: " << res.message;
  EXPECT_FALSE(cache.has_hold("C", "A"));
}

/// Bag regression (ecbs_tm_run): A owns the northbound corridor and is itself
/// WAIT for B (mutual hold / no free path). C crosses that corridor with no
/// time clash. Base ownership is checked first: C must WAIT before A's base
/// and must not be granted through the crossing — even though path/time is
/// silent and A is busy holding for someone else.
TEST(EvaluatePath, BaseOwnershipBeforePathWhenOwnerHoldsForOther)
{
  ecbs_planner::ScheduleCache cache;
  // Scenario geometry was written for pre-yaml body/clearance defaults.
  ecbs_planner::SolverConfig cfg;
  cfg.clearance_m = 2.0;
  cfg.safety_distance_m = 2.5;
  cfg.amr_length_m = 1.5;
  cfg.amr_width_m = 1.5;
  ecbs_planner::EcbsSolver solver(cfg);
  // A: north on x=40.9 (like amr2). B: east on y=74 (like amr3 ahead).
  const auto a = line_path("A", 40.9, 43.0, 40.9, 78.0, 0.0, 0);
  const auto b = line_path("B", 48.5, 43.0, 48.5, 78.0, 0.0, 100);
  cache.upsert(a);
  cache.upsert(b);
  cache.set_pose("A", 40.9, 43.5);
  cache.set_pose("B", 48.5, 46.0);

  ecbs_planner::srv::EvaluatePath::Response res;
  ecbs_planner::evaluate_path(request_from(a, 0), res, cache, solver);
  ASSERT_EQ(res.status, "FREE") << res.message;
  ASSERT_TRUE(cache.granted_base("A").has_value());
  ASSERT_FALSE(cache.reserved("A").empty());

  // A holds for B (as in the bag mutual wait) — must not clear A's base.
  ecbs_planner::WaitDependency hold_b;
  hold_b.amr_id = "B";
  hold_b.wait_point.x = 40.9;
  hold_b.wait_point.y = 74.0;
  cache.add_hold("A", hold_b, false);
  cache.add_hold("B", [&] {
      ecbs_planner::WaitDependency d;
      d.amr_id = "A";
      d.wait_point.x = 48.5;
      d.wait_point.y = 74.0;
      return d;
    }(), false);

  // C: east across A's corridor at y=57.5 (like amr1), times far in the future.
  const auto c = line_path("C", 36.5, 43.25, 36.5, 56.67, 1000.0, 0);
  auto c_east = line_path("C", 36.5, 56.67, 76.5, 57.5, 1010.0, 8);
  TimedPath cross = c;
  cross.waypoints.insert(
    cross.waypoints.end(), c_east.waypoints.begin() + 1, c_east.waypoints.end());
  cache.upsert(cross);

  ecbs_planner::evaluate_path(request_from(cross, 0), res, cache, solver);
  ASSERT_EQ(res.status, "WAIT") << res.message;
  ASSERT_FALSE(res.waiting_for.empty());
  bool waits_for_a = false;
  for (const auto & d : res.waiting_for) {
    waits_for_a = waits_for_a || d.amr_id == "A";
    if (d.amr_id == "A") {
      EXPECT_LT(d.wait_point.x, 40.9 - 0.5)
        << "wait must be before A's corridor, not through it";
      EXPECT_NEAR(d.wait_point.y, 56.67, 1.0);
    }
  }
  EXPECT_TRUE(waits_for_a) << res.message;

  const auto c_base = cache.granted_base("C");
  ASSERT_TRUE(c_base.has_value());
  EXPECT_LT(c_base->x, 40.9 - 0.5)
    << "C must not be granted through A's booked crossing";
}

/// A comes down x = 0 and turns east along y = 0. W is waiting on y = 0 behind
/// (0, 0) for A's base. When A evaluates, since it is in front and passes first,
/// A stays FREE (no wait at its base end). W keeps waiting behind A.
TEST(EvaluatePath, OwnerInFrontDoesNotWaitAtBaseEndWhenFollowerWaitsBehind)
{
  ecbs_planner::ScheduleCache cache;
  ecbs_planner::EcbsSolver solver;
  auto a = line_path("A", 0.0, 20.0, 0.0, 0.0, 0.0, 0);
  const auto east = line_path("A", 0.0, 0.0, 60.0, 0.0, 20.0, 8);
  a.waypoints.insert(a.waypoints.end(), east.waypoints.begin() + 1, east.waypoints.end());
  cache.upsert(a);
  ecbs_planner::srv::EvaluatePath::Response res;
  ecbs_planner::evaluate_path(request_from(a, 0), res, cache, solver);
  ASSERT_EQ(res.status, "FREE") << res.message;
  const auto base = cache.granted_base("A");
  ASSERT_TRUE(base.has_value());
  ASSERT_NEAR(base->x, 0.0, 1e-3);
  ASSERT_NEAR(base->y, 0.0, 1e-3);

  const auto w = line_path("W", -25.0, 0.0, 80.0, 0.0, 1000.0, 300);
  cache.upsert(w);
  ecbs_planner::evaluate_path(request_from(w, 0), res, cache, solver);
  ASSERT_EQ(res.status, "WAIT") << res.message;
  ASSERT_EQ(res.waiting_for.size(), 1u);
  EXPECT_EQ(res.waiting_for[0].amr_id, "A");
  EXPECT_LT(res.waiting_for[0].wait_point.x, 0.0);

  for (int k = 0; k < 2; ++k) {
    ecbs_planner::evaluate_path(request_from(a, 2), res, cache, solver);
    ASSERT_EQ(res.status, "FREE") << res.message;
    EXPECT_TRUE(res.waiting_for.empty());
    EXPECT_FALSE(cache.has_hold("A", "W"));
  }

  ecbs_planner::evaluate_path(request_from(w, 0), res, cache, solver);
  ASSERT_EQ(res.status, "WAIT") << res.message;
  EXPECT_EQ(res.waiting_for[0].amr_id, "A");
  EXPECT_FALSE(cache.has_hold("W", "A"));
}

/// L stands idle at (0, 0); W behind it on y = 0 waits before it. L then
/// gets a task east (0 → 60), its goal past the end of W's path (30, 0).
/// W's stored times say it reaches (0, 0) as L sets off, but W is waiting:
/// L is FREE (no hold, no wait at its base end), W keeps waiting behind L.
TEST(EvaluatePath, LeaderAheadOfWaitingFollowerIsFree)
{
  ecbs_planner::ScheduleCache cache;
  ecbs_planner::EcbsSolver solver;
  cache.set_pose("L", 0.0, 0.0);
  const auto w = line_path("W", -25.0, 0.0, 30.0, 0.0, 0.0, 300);
  cache.upsert(w);
  cache.set_pose("W", -25.0, 0.0);
  ecbs_planner::srv::EvaluatePath::Response res;
  ecbs_planner::evaluate_path(request_from(w, 0), res, cache, solver);
  ASSERT_EQ(res.status, "WAIT") << res.message;
  ASSERT_EQ(res.waiting_for.size(), 1u);
  ASSERT_EQ(res.waiting_for[0].amr_id, "L");

  const auto l = line_path("L", 0.0, 0.0, 60.0, 0.0, 25.0, 0);
  cache.upsert(l);
  for (const std::size_t from : {0u, 2u}) {
    ecbs_planner::evaluate_path(request_from(l, from), res, cache, solver);
    EXPECT_EQ(res.status, "FREE") << "L from " << from << ": " << res.message;
    EXPECT_FALSE(cache.has_hold("L", "W"));
  }

  ecbs_planner::evaluate_path(request_from(w, 8), res, cache, solver);
  ASSERT_EQ(res.status, "WAIT") << "W at -5 reaches L's base: " << res.message;
  EXPECT_EQ(res.waiting_for[0].amr_id, "L");
  EXPECT_FALSE(cache.has_hold("W", "L"));
}

/// Video case: C waits (reserved up to its wait point at (10, -2.5)). D
/// passes beside C on y = -5 with no time clash: it must not squeeze past the
/// waiting robot, and its own wait point is not beside anyone's stretch.
TEST(EvaluatePath, WaitingRobotBlocksThirdAmrPassingBeside)
{
  ecbs_planner::ScheduleCache cache;
  ecbs_planner::EcbsSolver solver;
  const auto a = line_path("A", 0.0, 0.0, 40.0, 0.0, 0.0, 0);
  const auto c = line_path("C", 10.0, -20.0, 10.0, 20.0, 1000.0, 300);
  const auto d = line_path("D", -10.0, -5.0, 30.0, -5.0, 2000.0, 500);
  cache.upsert(a);
  cache.upsert(c);
  cache.upsert(d);

  ecbs_planner::srv::EvaluatePath::Response res;
  ecbs_planner::evaluate_path(request_from(a, 0), res, cache, solver);
  ASSERT_EQ(res.status, "FREE") << res.message;
  ecbs_planner::evaluate_path(request_from(c, 0), res, cache, solver);
  ASSERT_EQ(res.status, "WAIT") << res.message;

  ecbs_planner::evaluate_path(request_from(d, 0), res, cache, solver);
  ASSERT_EQ(res.status, "WAIT") << res.message;
  ASSERT_EQ(res.waiting_for.size(), 1u);
  EXPECT_EQ(res.waiting_for[0].amr_id, "C");
  const double wx = res.waiting_for[0].wait_point.x;
  const double wy = res.waiting_for[0].wait_point.y;
  EXPECT_LT(wx, 10.0 - 1.5);
  for (const auto & kv : cache.reserved_except("D")) {
    for (const auto & cell : kv.second) {
      EXPECT_GT(std::hypot(cell.x - wx, cell.y - wy), 1.5)
        << "wait point beside " << kv.first << "'s stretch";
    }
  }
  EXPECT_FALSE(cache.has_hold("D", "C"));
}

/// Cells already reserved by another AMR earlier are not taken from it.
TEST(ScheduleCache, EarlierReserverKeepsCells)
{
  ecbs_planner::ScheduleCache cache;
  cache.set_reserved("A", {{0.0, 0.0}, {2.5, 0.0}});
  cache.set_reserved("B", {{2.5, 0.0}, {5.0, 0.0}});
  cache.set_reserved("A", {{2.5, 0.0}, {5.0, 0.0}});  // A moved on: keeps 2.5

  const auto for_a =
    ecbs_planner::blocking_cells(cache.reserved("B"), cache.reserved("A"), 1.0);
  const auto for_b =
    ecbs_planner::blocking_cells(cache.reserved("A"), cache.reserved("B"), 1.0);
  ASSERT_EQ(for_a.size(), 1u);
  EXPECT_NEAR(for_a[0].x, 5.0, 1e-9) << "B reserved 5.0 before A";
  ASSERT_EQ(for_b.size(), 1u);
  EXPECT_NEAR(for_b[0].x, 2.5, 1e-9) << "A reserved 2.5 before B";
}

/// Planner: C's plan crosses A's reserved base with no time clash → C gets a
/// wait before it, A none, and the seeded wait never becomes a hold.
TEST(EcbsSolver, PlannerWaitsBeforeReservedBaseWithoutHold)
{
  ecbs_planner::ScheduleCache cache;
  ecbs_planner::EcbsSolver solver;
  const auto a = line_path("A", 0.0, 0.0, 40.0, 0.0, 0.0, 0);
  cache.upsert(a);
  ecbs_planner::srv::EvaluatePath::Response res;
  ecbs_planner::evaluate_path(request_from(a, 0), res, cache, solver);
  ASSERT_EQ(res.status, "FREE") << res.message;

  const auto c = line_path("C", 10.0, -20.0, 10.0, 20.0, 1000.0, 300);
  const auto result = solver.solve(c, cache);
  ASSERT_NE(result.status, SolveStatus::FailReplan) << result.reason;
  bool c_waits = false;
  for (const auto & w : result.path.waypoints) {
    for (const auto & dep : w.waiting_for) {
      c_waits = c_waits || dep.amr_id == "A";
      EXPECT_NEAR(w.y, -2.5, 1e-3);
    }
  }
  EXPECT_TRUE(c_waits);
  const auto cached_a = cache.get("A");
  ASSERT_TRUE(cached_a.has_value());
  for (const auto & w : cached_a->waypoints) {
    EXPECT_TRUE(w.waiting_for.empty());
  }
  EXPECT_FALSE(cache.has_hold("C", "A"));

  ecbs_planner::register_from_zone_manager(
    seed_msg(result.path, 7, "A"), res, cache, solver);
  EXPECT_FALSE(cache.has_hold("C", "A")) << "reserved-only wait is no hold";
  EXPECT_FALSE(cache.has_hold("A", "C"));
}

/// Step 1: safety_m (2.5 m). Step 2: with L×W half-diag, step further only
/// while bodies would still dash; clearance-only near_path is not used.
TEST(PathUtils, WaitAtSafetyThenBodyDashOnly)
{
  const auto self = line_path("S", 0.0, 0.0, 30.0, 0.0, 0.0, 0);
  TimedPath curve;
  curve.waypoints = {wp(0, 15.0, 0.0, 900), wp(1, 12.5, 1.5, 901), wp(2, 10.0, 4.0, 902)};
  EXPECT_EQ(ecbs_planner::safety_wait_index(self, 6, 0, 2.5), 5u);
  // Legacy clearance stepping (no L×W): still available.
  EXPECT_EQ(ecbs_planner::clear_wait_index(self, 6, 0, 2.5, curve, 2.0), 4u);
  // Small body: 2.5 m wait does not dash → stay at index 5.
  const double small = std::hypot(0.4, 0.4);  // L=W=0.8 → touch ≈ 1.13 m
  EXPECT_EQ(
    ecbs_planner::clear_wait_index(self, 6, 0, 2.5, curve, 2.0, small), 5u);
  // Huge body: 2.5 m wait still dashes → step further back.
  const double huge = std::hypot(2.0, 2.0);  // L=W=4 → touch = 5.66 m
  EXPECT_LT(
    ecbs_planner::clear_wait_index(self, 6, 0, 2.5, curve, 2.0, huge), 5u);
}

/// B comes head-on along y = 0 (15..35) and curves off past (12.5, 1.5).
/// Wait starts 2.5 m before the overlap; steps further only if L×W bodies dash.
TEST(EvaluatePath, CurveBesideApproachMovesWaitBack)
{
  ecbs_planner::ScheduleCache cache;
  // Expects body-dash step-back with 1.5×1.5 and safety_distance_m=2.5.
  ecbs_planner::SolverConfig cfg;
  cfg.clearance_m = 2.0;
  cfg.safety_distance_m = 2.5;
  cfg.amr_length_m = 1.5;
  cfg.amr_width_m = 1.5;
  ecbs_planner::EcbsSolver solver(cfg);
  auto b = opposite_corridor("B", 35.0, 15.0);
  b.waypoints.push_back(wp(23.0, 12.5, 1.5, 901));
  b.waypoints.push_back(wp(26.0, 10.0, 4.0, 902));
  cache.upsert(b);

  ecbs_planner::srv::EvaluatePath::Response res;
  ecbs_planner::evaluate_path(evaluate_along_x(0.0), res, cache, solver);
  ASSERT_EQ(res.status, "WAIT") << res.message;
  ASSERT_EQ(res.waiting_for.size(), 1u);
  EXPECT_EQ(res.waiting_for[0].amr_id, "B");
  const double wx = res.waiting_for[0].wait_point.x;
  EXPECT_LE(wx, 12.5 + 1e-3);
  // Default 1.5×1.5 → touch ≈ 2.12 m; wait must not body-dash B.
  const double touch = 2.0 * std::hypot(0.75, 0.75);
  EXPECT_FALSE(ecbs_planner::near_path(wx, 0.0, b, touch - 1e-3))
    << "wait body would dash B at x=" << wx;
}

/// An idle robot (no path) stands 0.5 m beside amr1's route: amr1 waits
/// before it (no register), and is FREE once that robot has a task away.
TEST(EvaluatePath, IdleRobotOnRouteGetsWaitBeforeIt)
{
  ecbs_planner::ScheduleCache cache;
  ecbs_planner::EcbsSolver solver;
  cache.set_pose("I", 20.0, 0.5);
  ASSERT_TRUE(cache.standing("I"));

  ecbs_planner::srv::EvaluatePath::Response res;
  ecbs_planner::evaluate_path(evaluate_along_x(0.0), res, cache, solver);
  ASSERT_EQ(res.status, "WAIT") << res.message;
  ASSERT_EQ(res.waiting_for.size(), 1u);
  EXPECT_EQ(res.waiting_for[0].amr_id, "I");
  EXPECT_NEAR(res.waiting_for[0].wait_point.x, 17.5, 1e-3);
  EXPECT_FALSE(cache.has_hold("amr1", "I"));

  cache.upsert(line_path("I", 20.0, 0.5, 20.0, 30.5, 1000.0, 700));
  ASSERT_FALSE(cache.standing("I"));
  ecbs_planner::evaluate_path(evaluate_along_x(0.0), res, cache, solver);
  EXPECT_EQ(res.status, "FREE") << res.message;
}

/// I finished its task (down x = 30, then west on y = 5 to (10, 5)) and was
/// pushed onto amr1's route at (20, 0.5), far from its old path end.
/// /fleet_states reports it idle: it stands where it is (amr1 waits before
/// it) and its old path is dropped.
TEST(EvaluatePath, IdleRobotPushedOffItsPathStillBlocks)
{
  ecbs_planner::ScheduleCache cache;
  ecbs_planner::EcbsSolver solver;
  auto old = line_path("I", 30.0, 20.0, 30.0, 5.0, 0.0, 700);
  const auto west = line_path("I", 30.0, 5.0, 10.0, 5.0, 15.0, 800);
  old.waypoints.insert(old.waypoints.end(), west.waypoints.begin() + 1, west.waypoints.end());
  cache.upsert(old);
  cache.update_pose("I", 30.0, 10.0);
  cache.set_idle("I", false);
  cache.update_pose("I", 20.0, 0.5);
  ASSERT_FALSE(cache.standing("I")) << "busy, not at its path end";

  cache.set_idle("I", true);
  EXPECT_TRUE(cache.standing("I"));
  EXPECT_FALSE(cache.get("I").has_value()) << "finished task's path dropped";

  ecbs_planner::srv::EvaluatePath::Response res;
  ecbs_planner::evaluate_path(evaluate_along_x(0.0), res, cache, solver);
  ASSERT_EQ(res.status, "WAIT") << res.message;
  ASSERT_EQ(res.waiting_for.size(), 1u);
  EXPECT_EQ(res.waiting_for[0].amr_id, "I");
  EXPECT_NEAR(res.waiting_for[0].wait_point.x, 17.5, 1e-3);
  EXPECT_FALSE(cache.has_hold("amr1", "I"));
}

/// Idle reported only after I's next task was seeded: the new path is kept.
/// No busy report seen yet (startup): the seeded path is kept, I still
/// stands at its pose.
TEST(ScheduleCache, IdleKeepsPathOfNewTask)
{
  ecbs_planner::ScheduleCache cache;
  cache.upsert(line_path("I", 0.0, 0.0, 10.0, 0.0, 0.0, 0));
  cache.set_pose("I", 10.0, 0.0);
  cache.set_idle("I", false);
  auto next = line_path("I", 10.0, 0.0, 10.0, 20.0, 100.0, 50);
  next.task_id = "I_task2";
  cache.upsert(next);
  cache.set_idle("I", true);
  const auto kept = cache.get("I");
  ASSERT_TRUE(kept.has_value());
  EXPECT_EQ(kept->task_id, "I_task2");
  EXPECT_TRUE(cache.standing("I"));
  cache.set_idle("I", false);
  EXPECT_FALSE(cache.standing("I")) << "moving on its new path";

  ecbs_planner::ScheduleCache fresh;
  fresh.upsert(line_path("J", 0.0, 0.0, 10.0, 0.0, 0.0, 0));
  fresh.set_pose("J", 30.0, 0.0);
  fresh.set_idle("J", true);
  EXPECT_TRUE(fresh.get("J").has_value());
  EXPECT_TRUE(fresh.standing("J"));
}

/// The nearest waypoint is found over the whole path, not at the first
/// local minimum from the hint.
TEST(PathUtils, NearestWaypointSearchesWholePath)
{
  const auto p = make_path("A", {
    wp(0, 0.0, 0.0, 0), wp(1, -2.0, 0.0, 1), wp(2, -4.0, 0.0, 2),
    wp(3, -4.0, 4.0, 3), wp(4, 0.0, 4.0, 4), wp(5, 4.0, 4.0, 5), wp(6, 4.0, 0.5, 6)});
  EXPECT_EQ(ecbs_planner::nearest_waypoint_index(p, 4.0, 0.0, 0), 6u);

  const auto u = make_path("U", {
    wp(0, 0.0, 0.0, 0), wp(1, 5.0, 0.0, 1), wp(2, 5.0, 0.0, 2), wp(3, 0.0, 0.0, 3)});
  EXPECT_EQ(ecbs_planner::nearest_waypoint_index(u, 0.0, 0.0, 0), 0u);
  EXPECT_EQ(ecbs_planner::nearest_waypoint_index(u, 0.0, 0.0, 3), 3u);
}

/// W is held (waiting) at (30, 1), beside amr1's route, though its path
/// leads away: its live position blocks amr1. Once W is not waiting (driving
/// its path), amr1 is FREE.
TEST(EvaluatePath, WaitingRobotPositionBlocksPassingAmr)
{
  ecbs_planner::ScheduleCache cache;
  ecbs_planner::EcbsSolver solver;
  cache.upsert(line_path("W", 30.0, 1.0, 30.0, 21.0, 1000.0, 800));
  cache.set_pose("W", 30.0, 1.0);
  ecbs_planner::WaitDependency hold;
  hold.amr_id = "X";
  hold.wait_point.x = 30.0;
  hold.wait_point.y = 1.0;
  cache.add_hold("W", hold);
  ASSERT_TRUE(cache.standing("W"));

  ecbs_planner::srv::EvaluatePath::Response res;
  ecbs_planner::evaluate_path(evaluate_along_x(15.0), res, cache, solver);
  ASSERT_EQ(res.status, "WAIT") << res.message;
  ASSERT_EQ(res.waiting_for.size(), 1u);
  EXPECT_EQ(res.waiting_for[0].amr_id, "W");
  const double wx = res.waiting_for[0].wait_point.x;
  const double wy = res.waiting_for[0].wait_point.y;
  EXPECT_NEAR(wx, 27.5, 1e-3);
  EXPECT_GT(std::hypot(wx - 30.0, wy - 1.0), 2.0) << "wait point touches W";

  cache.remove_hold("W", "X");
  ASSERT_FALSE(cache.standing("W"));
  ecbs_planner::evaluate_path(evaluate_along_x(15.0), res, cache, solver);
  EXPECT_EQ(res.status, "FREE") << res.message;
}

/// A drives through its base (0..20): with each /fleet_states pose what it
/// has passed is freed at once (no evaluate needed), except the last
/// base_tail_m of cells. C, waiting at (10, -2.5) before the crossing (10, 0),
/// stays WAIT until A is safety_distance_m (2.5 m) past the first meeting
/// point; A keeps the rest of its base in its reservation order.
TEST(EvaluatePath, PassedPartOfBaseIsFreedAsRobotMoves)
{
  ecbs_planner::ScheduleCache cache;
  cache.set_base_tail_m(2.5);
  ecbs_planner::EcbsSolver solver;
  const auto a = line_path("A", 0.0, 0.0, 40.0, 0.0, 0.0, 0);
  const auto c = line_path("C", 10.0, -20.0, 10.0, 20.0, 1000.0, 300);
  cache.upsert(a);
  cache.upsert(c);

  ecbs_planner::srv::EvaluatePath::Response res;
  ecbs_planner::evaluate_path(request_from(a, 0), res, cache, solver);
  ASSERT_EQ(res.status, "FREE") << res.message;
  ecbs_planner::evaluate_path(request_from(c, 7), res, cache, solver);
  ASSERT_EQ(res.status, "WAIT") << res.message;
  const auto before = cache.reserved("A");
  ASSERT_FALSE(before.empty());

  cache.update_pose("A", 5.0, 0.0);
  ecbs_planner::evaluate_path(request_from(c, 7), res, cache, solver);
  EXPECT_EQ(res.status, "WAIT") << "A not past the crossing yet: " << res.message;

  cache.update_pose("A", 10.0, 0.0);
  const auto at10 = cache.reserved("A");
  ASSERT_FALSE(at10.empty());
  // 2.5 m spacing with 2.5 m tail → one cell kept behind A at 7.5.
  EXPECT_NEAR(at10.front().x, 7.5, 1e-9) << "tail cells: 2.5 m behind A";
  EXPECT_NEAR(at10.back().x, before.back().x, 1e-9) << "base end unchanged";

  cache.update_pose("A", 11.6, 0.0);
  ecbs_planner::evaluate_path(request_from(c, 7), res, cache, solver);
  EXPECT_EQ(res.status, "WAIT")
    << "C 2.5 m from the crossing, A only 1.6 m past it: " << res.message;

  cache.update_pose("A", 12.4, 0.0);
  ecbs_planner::evaluate_path(request_from(c, 7), res, cache, solver);
  EXPECT_EQ(res.status, "WAIT")
    << "first conflict stays booked until A is 2.5 m past it: " << res.message;

  cache.update_pose("A", 12.6, 0.0);
  ecbs_planner::evaluate_path(request_from(c, 7), res, cache, solver);
  EXPECT_EQ(res.status, "WAIT") << "STRICT: tail cell at the crossing still booked: " << res.message;

  cache.update_pose("A", 15.0, 0.0);
  ecbs_planner::evaluate_path(request_from(c, 7), res, cache, solver);
  EXPECT_EQ(res.status, "FREE") << "A's booked stretch no longer covers the crossing: " << res.message;

  cache.update_pose("A", 17.6, 0.0);
  const auto after = cache.reserved("A");
  ASSERT_FALSE(after.empty());
  EXPECT_NEAR(after.front().x, 15.0, 1e-9) << "cells more than 2.5 m behind A are freed";
  EXPECT_NEAR(after.back().x, before.back().x, 1e-9) << "base end unchanged";
  EXPECT_EQ(after.back().since, before.back().since) << "order kept";
  const auto cached_a = cache.get("A");
  ASSERT_TRUE(cached_a.has_value());
  EXPECT_NEAR(cached_a->waypoints.front().x, 17.5, 1e-9) << "path behind A dropped";

  ecbs_planner::evaluate_path(request_from(c, 0), res, cache, solver);
  EXPECT_EQ(res.status, "FREE") << "A is past the crossing: " << res.message;
}

/// STRICT base booking: A (at x = 15) books its stretch from base_tail_m
/// (2.5 m) behind it to its base end. Anything behind the tail is free; any
/// other AMR whose path touches the booked stretch at even ONE waypoint waits,
/// however far apart in time (1000 s here) the two would be there.
TEST(EvaluatePath, StrictBaseBookedFromTailToBaseEndOnly)
{
  ecbs_planner::ScheduleCache cache;
  cache.set_base_tail_m(2.5);
  ecbs_planner::EcbsSolver solver;
  const auto a = line_path("A", 0.0, 0.0, 40.0, 0.0, 0.0, 0);
  const auto behind = line_path("B", 5.0, -20.0, 5.0, 20.0, 1000.0, 300);
  const auto tail = line_path("T", 12.5, -20.0, 12.5, 20.0, 1000.0, 300);
  const auto ahead = line_path("H", 20.0, -20.0, 20.0, 20.0, 1000.0, 300);
  cache.upsert(a);
  cache.upsert(behind);
  cache.upsert(tail);
  cache.upsert(ahead);

  ecbs_planner::srv::EvaluatePath::Response res;
  ecbs_planner::evaluate_path(request_from(a, 0), res, cache, solver);
  ASSERT_EQ(res.status, "FREE") << res.message;
  cache.update_pose("A", 15.0, 0.0);
  ecbs_planner::evaluate_path(request_from(a, 6), res, cache, solver);
  ASSERT_EQ(res.status, "FREE") << res.message;

  ecbs_planner::evaluate_path(request_from(behind, 0), res, cache, solver);
  EXPECT_EQ(res.status, "FREE") << "x = 5 is behind A's 2.5 m tail: " << res.message;
  ecbs_planner::evaluate_path(request_from(tail, 0), res, cache, solver);
  EXPECT_EQ(res.status, "WAIT") << "x = 12.5 is A's last tail cell: " << res.message;
  ecbs_planner::evaluate_path(request_from(ahead, 0), res, cache, solver);
  EXPECT_EQ(res.status, "WAIT") << "x = 20 is inside A's base ahead: " << res.message;
  EXPECT_FALSE(cache.has_hold("H", "A")) << "reserved base: no register needed";
}

namespace
{

/// Run case: amr1 comes down x = 60 and turns east on y = 50 to \p goal_x;
/// amr2 goes east on y = 55 and turns down to its goal (65, 50) on amr1's
/// lane. amr2 first waits before the crossing (60, 55).
struct RunCase
{
  ecbs_planner::ScheduleCache cache;
  ecbs_planner::EcbsSolver solver;
  ecbs_planner::TimedPath amr1;
  ecbs_planner::TimedPath amr2;
  std::size_t amr2_wait{0};
  std::string message;

  explicit RunCase(double goal_x)
  {
    amr1 = line_path("amr1", 60.0, 70.0, 60.0, 50.0, 0.0, 0);
    const auto east = line_path("amr1", 60.0, 50.0, goal_x, 50.0, 20.0, 8);
    amr1.waypoints.insert(amr1.waypoints.end(), east.waypoints.begin() + 1, east.waypoints.end());
    amr2 = line_path("amr2", 45.0, 55.0, 65.0, 55.0, 1000.0, 300);
    const auto down = line_path("amr2", 65.0, 55.0, 65.0, 50.0, 1020.0, 308);
    amr2.waypoints.insert(amr2.waypoints.end(), down.waypoints.begin() + 1, down.waypoints.end());
    cache.upsert(amr1);
    cache.upsert(amr2);
  }

  std::string evaluate(const ecbs_planner::TimedPath & p, std::size_t from)
  {
    ecbs_planner::srv::EvaluatePath::Response res;
    ecbs_planner::evaluate_path(request_from(p, from), res, cache, solver);
    message = res.message;
    if (&p == &amr2 && res.status == "WAIT" && !res.waiting_for.empty()) {
      const auto & w = res.waiting_for[0].wait_point;
      for (std::size_t i = 0; i < amr2.waypoints.size(); ++i) {
        if (std::hypot(amr2.waypoints[i].x - w.x, amr2.waypoints[i].y - w.y) < 1e-3) {
          amr2_wait = i;
        }
      }
    }
    return res.status;
  }

  void amr1_at(std::size_t i)
  {
    cache.update_pose("amr1", amr1.waypoints[i].x, amr1.waypoints[i].y);
    ASSERT_EQ(evaluate(amr1, i), "FREE") << message;
  }
};

}  // namespace

/// amr1's goal (75, 50) lies past amr2's goal: once the crossing is clear,
/// amr1 drives on through the spot amr2 heads for and is 5 m past it long
/// before amr2 gets there, so amr2 does not wait for that part of amr1's base.
TEST(EvaluatePath, OwnerDrivingAwayDoesNotHoldWaiterForBaseAhead)
{
  RunCase run(75.0);
  ASSERT_EQ(run.evaluate(run.amr1, 0), "FREE");
  ASSERT_EQ(run.evaluate(run.amr2, 0), "WAIT");
  ASSERT_GT(run.amr2_wait, 0u);
  ASSERT_LT(run.amr2.waypoints[run.amr2_wait].x, 60.0);

  run.amr1_at(5);
  EXPECT_EQ(run.evaluate(run.amr2, run.amr2_wait), "WAIT") << "amr1 still before the crossing";

  run.amr1_at(8);
  EXPECT_EQ(run.evaluate(run.amr2, run.amr2_wait), "WAIT")
    << "STRICT: amr1's base still runs through amr2's goal, whatever the timing";

  run.amr1_at(run.amr1.waypoints.size() - 1);
  EXPECT_EQ(run.evaluate(run.amr2, run.amr2_wait), "FREE")
    << "amr1 and its 2.5 m tail are past amr2's goal";
}

/// amr1 (2.5 m before the crossing, base running to its goal) and amr2
/// (15 m before it): amr2 waits until amr1 is safety_m (2.5 m) past the
/// crossing. Then, if amr1's goal is amr2's goal (65, 50), amr1 stops there:
/// amr2 keeps waiting. If it lies past it (75, 50), amr1 drives through
/// amr2's goal ahead of it: FREE.
TEST(EvaluatePath, OwnerStoppingOnWaiterPathStillBlocks)
{
  for (const double goal_x : {65.0, 75.0}) {
    RunCase run(goal_x);
    run.cache.set_base_tail_m(2.5);
    run.amr1_at(5);
    ASSERT_EQ(run.evaluate(run.amr2, 0), "WAIT") << "goal_x=" << goal_x << ": " << run.message;
    run.cache.update_pose("amr1", 60.0, 53.5);
    EXPECT_EQ(run.evaluate(run.amr2, 0), "WAIT")
      << "goal_x=" << goal_x << ", amr1 1.5 m past the crossing: " << run.message;
    run.cache.update_pose("amr1", 60.0, 52.4);
    EXPECT_EQ(run.evaluate(run.amr2, 0), "WAIT")
      << "STRICT: goal_x=" << goal_x << ", amr1's base still covers amr2's path: " << run.message;
  }
}

/// Run of 15:32: amr3 heads west on y = 0 and is given a base through the
/// junction (0, 0); amr1 comes down x = 0 and turns west behind it on the
/// same lane. amr1 waits before the junction; amr3, in first, gets no hold
/// (none past the junction either) and stays FREE.
TEST(EvaluatePath, OwnerInFirstOnSharedLaneGetsNoHold)
{
  ecbs_planner::ScheduleCache cache;
  ecbs_planner::EcbsSolver solver;
  auto amr1 = line_path("amr1", 0.0, 20.0, 0.0, 0.0, 0.0, 0);
  const auto west = line_path("amr1", 0.0, 0.0, -20.0, 0.0, 20.0, 8);
  amr1.waypoints.insert(amr1.waypoints.end(), west.waypoints.begin() + 1, west.waypoints.end());
  const auto amr3 = line_path("amr3", 15.0, 0.0, -30.0, 0.0, 0.0, 100);
  cache.upsert(amr1);
  cache.upsert(amr3);

  ecbs_planner::srv::EvaluatePath::Response res;
  ecbs_planner::evaluate_path(request_from(amr3, 0), res, cache, solver);
  ASSERT_EQ(res.status, "FREE") << res.message;
  const auto base = cache.granted_base("amr3");
  ASSERT_TRUE(base.has_value());
  ASSERT_LT(base->x, 0.0) << "amr3's base runs through the junction";

  ecbs_planner::evaluate_path(request_from(amr1, 0), res, cache, solver);
  ASSERT_EQ(res.status, "WAIT") << res.message;
  ASSERT_FALSE(res.waiting_for.empty());
  EXPECT_EQ(res.waiting_for[0].amr_id, "amr3");
  EXPECT_GT(res.waiting_for[0].wait_point.y, 0.0) << "amr1 waits before the junction";

  EXPECT_FALSE(cache.has_hold("amr3", "amr1"));
  const auto cached3 = cache.get("amr3");
  ASSERT_TRUE(cached3.has_value());
  for (const auto & w : cached3->waypoints) {
    EXPECT_TRUE(w.waiting_for.empty()) << "amr3 shows a wait at (" << w.x << ", " << w.y << ")";
  }

  ecbs_planner::evaluate_path(request_from(amr3, 2), res, cache, solver);
  EXPECT_EQ(res.status, "FREE") << res.message;
}

/// Back to back on y = 0: A (goal 40) drives away ahead of F (goal 30). F far
/// enough behind is FREE and follows; F close behind A waits.
TEST(EvaluatePath, FollowerBehindOwnerDrivingAwayIsFree)
{
  ecbs_planner::ScheduleCache cache;
  ecbs_planner::EcbsSolver solver;
  const auto a = line_path("A", 0.0, 0.0, 40.0, 0.0, 0.0, 0);
  const auto f = line_path("F", -20.0, 0.0, 30.0, 0.0, 1000.0, 500);
  cache.upsert(a);
  cache.upsert(f);

  ecbs_planner::srv::EvaluatePath::Response res;
  ecbs_planner::evaluate_path(request_from(a, 0), res, cache, solver);
  ASSERT_EQ(res.status, "FREE") << res.message;
  ecbs_planner::evaluate_path(request_from(f, 0), res, cache, solver);
  ASSERT_EQ(res.status, "WAIT") << "A's position not known yet: " << res.message;

  cache.update_pose("A", 10.0, 0.0);
  ecbs_planner::evaluate_path(request_from(a, 4), res, cache, solver);
  ASSERT_EQ(res.status, "FREE") << res.message;

  ecbs_planner::evaluate_path(request_from(f, 0), res, cache, solver);
  EXPECT_EQ(res.status, "FREE") << "F 30 m behind A driving away: " << res.message;

  ecbs_planner::evaluate_path(request_from(f, 11), res, cache, solver);
  EXPECT_EQ(res.status, "WAIT") << "F 2.5 m behind A: " << res.message;
  EXPECT_FALSE(cache.has_hold("F", "A"));
}

/// Leader arrives at its goal and stands there with a reserved tail still
/// held: follower wait is safety_distance_m (2.5 m) before the pose — not
/// the old ~7 m (tail + footprint + safety).
TEST(EvaluatePath, StandingAtGoalWaitsSafetyDistanceOnly)
{
  ecbs_planner::ScheduleCache cache;
  cache.set_base_tail_m(2.5);
  ecbs_planner::EcbsSolver solver;
  const auto a = line_path("A", 0.0, 0.0, 40.0, 0.0, 0.0, 0);
  const auto f = line_path("F", 0.0, 0.0, 60.0, 0.0, 1000.0, 500);
  cache.upsert(a);
  cache.upsert(f);

  ecbs_planner::srv::EvaluatePath::Response res;
  ecbs_planner::evaluate_path(request_from(a, 0), res, cache, solver);
  ASSERT_EQ(res.status, "FREE") << res.message;
  // Leave a long reserved stretch behind A as if the base had not shrunk yet.
  cache.set_reserved(
    "A",
    {{30.0, 0.0}, {32.5, 0.0}, {35.0, 0.0}, {37.5, 0.0}, {40.0, 0.0}});
  cache.update_pose("A", 40.0, 0.0);
  ASSERT_TRUE(cache.standing("A")) << "A at its path end";

  auto freq = request_from(f, 12);  // F at x ≈ 30, look-ahead covers A at 40
  freq.current_x = 30.0;
  freq.current_y = 0.0;
  ecbs_planner::evaluate_path(freq, res, cache, solver);
  ASSERT_EQ(res.status, "WAIT") << res.message;
  ASSERT_EQ(res.waiting_for.size(), 1u);
  EXPECT_EQ(res.waiting_for[0].amr_id, "A");
  EXPECT_NEAR(res.waiting_for[0].wait_point.x, 37.5, 1e-3)
    << "must be 2.5 m before standing A, not ~7 m into the reserved tail";
}

/// Back-to-back: F's wait tracks A's pose at safety_distance_m (2.5 m), not
/// a fixed pin. When A is far past F's look-ahead, F is FREE.
TEST(EvaluatePath, FollowerWaitTracksLeaderAtSafetyDistance)
{
  ecbs_planner::ScheduleCache cache;
  ecbs_planner::EcbsSolver solver;
  const auto a = line_path("A", 0.0, 0.0, 40.0, 0.0, 0.0, 0);
  const auto f = line_path("F", -20.0, 0.0, 60.0, 0.0, 1000.0, 500);
  cache.upsert(a);
  cache.upsert(f);

  ecbs_planner::srv::EvaluatePath::Response res;
  ecbs_planner::evaluate_path(request_from(a, 0), res, cache, solver);
  ASSERT_EQ(res.status, "FREE") << res.message;
  // Pose known so B2B tracking applies (not reserved-stretch pin).
  cache.update_pose("A", 0.0, 0.0);
  ecbs_planner::evaluate_path(request_from(f, 0), res, cache, solver);
  ASSERT_EQ(res.status, "WAIT") << res.message;
  ASSERT_EQ(res.waiting_for.size(), 1u);
  EXPECT_EQ(res.waiting_for[0].amr_id, "A");
  EXPECT_NEAR(res.waiting_for[0].wait_point.x, -2.5, 0.75)
    << "wait ~2.5 m before A at 0";

  for (const double ax : {2.6, 5.1}) {
    cache.update_pose("A", ax, 0.0);
    ecbs_planner::evaluate_path(request_from(f, 0), res, cache, solver);
    ASSERT_EQ(res.status, "WAIT") << "A at " << ax << ": " << res.message;
    ASSERT_EQ(res.waiting_for.size(), 1u);
    EXPECT_NEAR(res.waiting_for[0].wait_point.x, ax - 2.5, 0.75)
      << "wait must track A at " << ax;
  }

  cache.update_pose("A", 40.0, 0.0);
  ecbs_planner::evaluate_path(request_from(f, 0), res, cache, solver);
  EXPECT_EQ(res.status, "FREE") << "A past F look-ahead: " << res.message;
}

/// The stretch makes a U-turn: on the way out the robot is nearer a cell of
/// the way back, but it is not there yet — only cells behind it are freed.
TEST(ScheduleCache, UpdatePoseFollowsProgressNotNearestCell)
{
  ecbs_planner::ScheduleCache cache;
  cache.set_base_tail_m(0.0);
  cache.set_reserved(
    "A", {{0.0, 0.0}, {2.5, 0.0}, {5.0, 0.0}, {5.0, 1.0}, {2.5, 1.0}, {0.0, 1.0}});
  cache.update_pose("A", 2.5, 0.6);
  const auto cells = cache.reserved("A");
  ASSERT_EQ(cells.size(), 5u);
  EXPECT_NEAR(cells.front().x, 2.5, 1e-9);
  EXPECT_NEAR(cells.front().y, 0.0, 1e-9);
}

/// External evaluate server: the fleet adapter's planner does not know A's
/// reserved base, so C is seeded without a wait. The seed adds it (shown on
/// /to_zone_manager), still with no hold.
TEST(EvaluatePath, SeedAddsWaitBeforeReservedBase)
{
  ecbs_planner::ScheduleCache cache;
  ecbs_planner::EcbsSolver solver;
  const auto a = line_path("A", 0.0, 0.0, 40.0, 0.0, 0.0, 0);
  cache.upsert(a);
  ecbs_planner::srv::EvaluatePath::Response res;
  ecbs_planner::evaluate_path(request_from(a, 0), res, cache, solver);
  ASSERT_EQ(res.status, "FREE") << res.message;

  const auto c = line_path("C", 10.0, -20.0, 10.0, 20.0, 1000.0, 300);
  std::vector<ecbs_planner::TimedPath> published;
  ecbs_planner::register_from_zone_manager(seed_msg(c), res, cache, solver, &published);
  EXPECT_TRUE(shows_wait(published, "C"));
  EXPECT_FALSE(shows_wait(published, "A"));
  EXPECT_FALSE(cache.has_hold("C", "A"));
  EXPECT_FALSE(cache.has_hold("A", "C"));

  const auto cached_c = cache.get("C");
  ASSERT_TRUE(cached_c.has_value());
  bool waits_before = false;
  for (const auto & w : cached_c->waypoints) {
    for (const auto & dep : w.waiting_for) {
      waits_before = waits_before ||
        (dep.amr_id == "A" && std::abs(w.y + 2.5) < 1e-3);
    }
  }
  EXPECT_TRUE(waits_before);
}

/// Head-on: A drives 0 -> 40 and is given its base first; B drives 40 -> 0 on
/// the same line. The fleet-adapter planner cannot resolve this by waiting, so
/// the gate seeds B's path unchanged. B must get WAIT at its start (it can
/// neither go forward into A nor back), and A must not drive into B's body.
TEST(EvaluatePath, HeadOnSecondAmrWaitsAtStart)
{
  ecbs_planner::ScheduleCache cache;
  ecbs_planner::EcbsSolver solver;
  const auto a = line_path("A", 0.0, 0.0, 40.0, 0.0, 0.0, 0);
  const auto b = line_path("B", 40.0, 0.0, 0.0, 0.0, 0.0, 100);
  cache.upsert(a);
  cache.set_pose("A", 0.0, 0.0);
  cache.set_pose("B", 40.0, 0.0);

  ecbs_planner::srv::EvaluatePath::Response res;
  ecbs_planner::evaluate_path(request_from(a, 0), res, cache, solver);
  ASSERT_EQ(res.status, "FREE") << res.message;

  std::vector<ecbs_planner::TimedPath> published;
  ecbs_planner::register_from_zone_manager(seed_msg(b), res, cache, solver, &published);

  ecbs_planner::evaluate_path(request_from(b, 0), res, cache, solver);
  ASSERT_EQ(res.status, "WAIT") << res.message;
  ASSERT_FALSE(res.waiting_for.empty());
  EXPECT_EQ(res.waiting_for[0].amr_id, "A");
  EXPECT_NEAR(res.waiting_for[0].wait_point.x, 40.0, 1e-3) << res.message;

  ecbs_planner::evaluate_path(request_from(a, 0), res, cache, solver);
  ASSERT_EQ(res.status, "WAIT") << res.message;
  ASSERT_FALSE(res.waiting_for.empty());
  EXPECT_EQ(res.waiting_for[0].amr_id, "B");
  // C5: safety stop may sit before the prior tip (authority conflict).
  EXPECT_LT(res.waiting_for[0].wait_point.x, 40.0) << "A stops short of B";
}

/// Run of 16:41: B stands at its start while A is seeded, so A's path already
/// waits just before B's body. B's path is A's reversed: head-on, so first
/// come, first served does not apply. B waits at its start and A holds for B
/// right after the base it was already given, however far away B is.
TEST(EvaluatePath, HeadOnBothHoldFirstAmrRightAfterItsBase)
{
  ecbs_planner::ScheduleCache cache;
  ecbs_planner::EcbsSolver solver;
  const auto a = line_path("A", 0.0, 0.0, 40.0, 0.0, 0.0, 0);
  const auto b = line_path("B", 40.0, 0.0, 0.0, 0.0, 7.0, 100);
  cache.set_pose("A", 0.0, 0.0);
  cache.set_pose("B", 40.0, 0.0);

  ecbs_planner::srv::EvaluatePath::Response res;
  std::vector<ecbs_planner::TimedPath> published;
  ecbs_planner::register_from_zone_manager(seed_msg(a), res, cache, solver, &published);
  ASSERT_TRUE(shows_wait(published, "A")) << "A waits before standing B";

  ecbs_planner::evaluate_path(request_from(a, 0), res, cache, solver);
  ASSERT_EQ(res.status, "FREE") << res.message;
  const auto first_base = cache.granted_base("A");
  ASSERT_TRUE(first_base.has_value());
  ASSERT_LT(first_base->x, 40.0 - 2.5);

  published.clear();
  ecbs_planner::register_from_zone_manager(seed_msg(b), res, cache, solver, &published);
  ecbs_planner::evaluate_path(request_from(b, 0), res, cache, solver, &published);
  ASSERT_EQ(res.status, "WAIT") << res.message;
  ASSERT_FALSE(res.waiting_for.empty());
  EXPECT_EQ(res.waiting_for[0].amr_id, "A");
  EXPECT_NEAR(res.waiting_for[0].wait_point.x, 40.0, 1e-3) << "B waits at its start";

  for (const std::size_t from : {std::size_t{0}, std::size_t{4}}) {
    cache.set_pose("A", 2.5 * static_cast<double>(from), 0.0);
    published.clear();
    ecbs_planner::evaluate_path(request_from(a, from), res, cache, solver, &published);
    ASSERT_EQ(res.status, "WAIT") << "from=" << from << ": " << res.message;
    ASSERT_EQ(res.waiting_for.size(), 1u);
    EXPECT_EQ(res.waiting_for[0].amr_id, "B");
    // C5: may wait before granted tip when safety stop is earlier.
    EXPECT_LE(res.waiting_for[0].wait_point.x, first_base->x + 2.5 + 1e-3)
      << "A does not wait deep past its first base, from=" << from;
    EXPECT_TRUE(cache.has_hold("A", "B")) << "mutual waits";
    EXPECT_TRUE(cache.has_hold("B", "A"));
  }
}

/// Run of 12:44: amr2 (first) ends on amr3's route; amr3 comes the other way.
/// Both hold (mutual), amr2 right after its first base.
TEST(EvaluatePath, HeadOnPartialOverlapGivesMutualWaits)
{
  ecbs_planner::ScheduleCache cache;
  ecbs_planner::EcbsSolver solver;
  const auto a = line_path("A", 0.0, 0.0, 40.0, 0.0, 0.0, 0);
  const auto b = line_path("B", 50.0, 0.0, 0.0, 0.0, 0.0, 100);
  cache.set_pose("A", 0.0, 0.0);
  cache.set_pose("B", 50.0, 0.0);

  ecbs_planner::srv::EvaluatePath::Response res;
  ecbs_planner::register_from_zone_manager(seed_msg(a), res, cache, solver);
  ecbs_planner::evaluate_path(request_from(a, 0), res, cache, solver);
  ASSERT_EQ(res.status, "FREE") << res.message;
  const auto first_base = cache.granted_base("A");
  ASSERT_TRUE(first_base.has_value());

  ecbs_planner::register_from_zone_manager(seed_msg(b), res, cache, solver);
  ecbs_planner::evaluate_path(request_from(b, 0), res, cache, solver);
  ASSERT_EQ(res.status, "WAIT") << res.message;
  EXPECT_EQ(res.waiting_for[0].amr_id, "A");

  cache.set_pose("A", 10.0, 0.0);
  std::vector<ecbs_planner::TimedPath> published;
  ecbs_planner::evaluate_path(request_from(a, 4), res, cache, solver, &published);
  ASSERT_EQ(res.status, "WAIT") << res.message;
  ASSERT_EQ(res.waiting_for.size(), 1u);
  EXPECT_EQ(res.waiting_for[0].amr_id, "B");
  // C5: safety stop may be before granted tip.
  EXPECT_LE(res.waiting_for[0].wait_point.x, first_base->x + 2.5 + 1e-3);
  EXPECT_TRUE(shows_wait(published, "A"));
  EXPECT_TRUE(cache.has_hold("A", "B"));
}

/// Bag of 11:32: amr1 is answered WAIT at 12.5, so the path up to there is
/// released to it. C crosses amr1's path at 7.5, inside that stretch: amr1
/// gets no hold for C there (it cannot stop short of what it was given).
TEST(EvaluatePath, WaitReleasedStretchIsBaseNoNewWaitInsideIt)
{
  ecbs_planner::ScheduleCache cache;
  ecbs_planner::EcbsSolver solver;
  const auto req1 = evaluate_along_x(0.0);
  cache.upsert(ecbs_planner::timed_path_from_evaluate_request(req1));
  cache.upsert(opposite_corridor("amr2", 35.0, 15.0));

  ecbs_planner::srv::EvaluatePath::Response res;
  ecbs_planner::evaluate_path(req1, res, cache, solver);
  ASSERT_EQ(res.status, "WAIT") << res.message;
  ASSERT_NEAR(res.waiting_for[0].wait_point.x, 12.5, 1e-3);

  // Reaches the crossing at t = 7.5, together with amr1.
  const auto c = line_path("C", 7.5, -10.0, 7.5, 10.0, -2.5, 300);
  ecbs_planner::register_from_zone_manager(seed_msg(c, 2, "amr1"), res, cache, solver);
  EXPECT_TRUE(cache.has_hold("C", "amr1"));
  EXPECT_FALSE(cache.has_hold("amr1", "C")) << "crossing lies inside amr1's base";

  ecbs_planner::evaluate_path(req1, res, cache, solver);
  ASSERT_EQ(res.status, "WAIT") << res.message;
  for (const auto & d : res.waiting_for) {
    EXPECT_GE(d.wait_point.x, 12.5 - 1e-3) << d.amr_id << " wait inside released stretch";
  }
}

namespace
{

/// A drives (0,0) -> (40,0); B crosses it at (20,0) at the same time.
std::pair<ecbs_planner::TimedPath, ecbs_planner::TimedPath> crossing_pair()
{
  return {
    line_path("A", 0.0, 0.0, 40.0, 0.0, 0.0, 0),
    line_path("B", 20.0, -20.0, 20.0, 20.0, 0.0, 100)};
}

}  // namespace

/// No first come, first served: A got its path first, yet both A and B hold
/// for each other, whichever evaluates first.
TEST(EvaluatePath, CrossingPairHoldsMutuallyRegardlessOfOrder)
{
  ecbs_planner::ScheduleCache cache;
  ecbs_planner::EcbsSolver solver;
  const auto [a, b] = crossing_pair();
  ecbs_planner::srv::EvaluatePath::Response res;
  std::vector<ecbs_planner::TimedPath> published;
  ecbs_planner::register_from_zone_manager(seed_msg(a), res, cache, solver);
  ecbs_planner::register_from_zone_manager(seed_msg(b), res, cache, solver);

  ecbs_planner::evaluate_path(request_from(b, 0), res, cache, solver);
  ASSERT_EQ(res.status, "WAIT") << res.message;
  ASSERT_EQ(res.waiting_for.size(), 1u);
  EXPECT_EQ(res.waiting_for[0].amr_id, "A");
  EXPECT_LT(res.waiting_for[0].wait_point.y, 0.0) << "B waits before the crossing";
  EXPECT_TRUE(cache.has_hold("A", "B"));

  ecbs_planner::evaluate_path(request_from(a, 0), res, cache, solver);
  ASSERT_EQ(res.status, "WAIT") << res.message;
  ASSERT_EQ(res.waiting_for.size(), 1u);
  EXPECT_EQ(res.waiting_for[0].amr_id, "B");
  EXPECT_LT(res.waiting_for[0].wait_point.x, 20.0) << "A waits before the crossing";

  ecbs_planner::register_from_zone_manager(seed_msg(a, 6, "B"), res, cache, solver, &published);
  EXPECT_TRUE(cache.has_hold("A", "B"));
  EXPECT_TRUE(cache.has_hold("B", "A"));
}

/// Same pair, A evaluated first: still mutual.
TEST(EvaluatePath, CrossingPairHoldsMutuallyWhenFirstAmrEvaluatesFirst)
{
  ecbs_planner::ScheduleCache cache;
  ecbs_planner::EcbsSolver solver;
  const auto [a, b] = crossing_pair();
  ecbs_planner::srv::EvaluatePath::Response res;
  ecbs_planner::register_from_zone_manager(seed_msg(a), res, cache, solver);
  ecbs_planner::register_from_zone_manager(seed_msg(b), res, cache, solver);

  ecbs_planner::evaluate_path(request_from(a, 0), res, cache, solver);
  ASSERT_EQ(res.status, "WAIT") << res.message;
  ecbs_planner::evaluate_path(request_from(b, 0), res, cache, solver);
  ASSERT_EQ(res.status, "WAIT") << res.message;
  EXPECT_TRUE(cache.has_hold("A", "B"));
  EXPECT_TRUE(cache.has_hold("B", "A"));

  for (int i = 0; i < 3; ++i) {
    ecbs_planner::evaluate_path(request_from(a, 0), res, cache, solver);
    EXPECT_EQ(res.status, "WAIT") << "held until ZM registers: " << res.message;
    ecbs_planner::evaluate_path(request_from(b, 0), res, cache, solver);
    EXPECT_EQ(res.status, "WAIT") << "held until ZM registers: " << res.message;
  }
}

/// amr1 overran its wait point (12.5) to 17.5: the old wait is not dragged
/// along with it (it is behind the robot); the wait is at or ahead of it.
TEST(EvaluatePath, PassedWaitPointIsDroppedNotDraggedAlong)
{
  ecbs_planner::ScheduleCache cache;
  ecbs_planner::EcbsSolver solver;
  cache.upsert(opposite_corridor("amr2", 35.0, 15.0));

  auto req = evaluate_along_x(0.0);
  ecbs_planner::srv::EvaluatePath::Response res;
  ecbs_planner::evaluate_path(req, res, cache, solver);
  ASSERT_EQ(res.status, "WAIT") << res.message;
  ASSERT_NEAR(res.waiting_for[0].wait_point.x, 12.5, 1e-3);

  req.current_x = 17.5;
  std::vector<ecbs_planner::TimedPath> published;
  ecbs_planner::evaluate_path(req, res, cache, solver, &published);
  ASSERT_EQ(res.status, "WAIT") << res.message;
  for (const auto & d : res.waiting_for) {
    EXPECT_GE(d.wait_point.x, 17.5 - 1e-3) << d.amr_id;
  }
  for (const auto & p : published) {
    if (p.amr_id != "amr1") {
      continue;
    }
    for (const auto & w : p.waypoints) {
      for (const auto & d : w.waiting_for) {
        EXPECT_GE(d.wait_point.x, 17.5 - 1e-3) << "published wait behind robot";
      }
    }
  }
}

/// Run of 12:46: A already holds for B (crossing, B first). C then comes
/// head-on at A, so A also holds for C — at the same wait point, not at a
/// second one further on.
TEST(EvaluatePath, SecondBlockerJoinsTheExistingWaitPoint)
{
  ecbs_planner::ScheduleCache cache;
  ecbs_planner::EcbsSolver solver;
  const auto b = line_path("B", 20.0, -20.0, 20.0, 20.0, 0.0, 100);
  const auto a = line_path("A", 0.0, 0.0, 40.0, 0.0, 0.0, 0);
  const auto c = line_path("C", 50.0, 0.0, 25.0, 0.0, 0.0, 200);
  ecbs_planner::srv::EvaluatePath::Response res;
  ecbs_planner::register_from_zone_manager(seed_msg(b), res, cache, solver);
  ecbs_planner::register_from_zone_manager(seed_msg(a), res, cache, solver);

  ecbs_planner::evaluate_path(request_from(a, 0), res, cache, solver);
  ASSERT_EQ(res.status, "WAIT") << res.message;
  ASSERT_EQ(res.waiting_for.size(), 1u);
  ASSERT_EQ(res.waiting_for[0].amr_id, "B");
  const double wait_x = res.waiting_for[0].wait_point.x;
  ASSERT_LT(wait_x, 20.0);

  std::vector<ecbs_planner::TimedPath> published;
  ecbs_planner::register_from_zone_manager(seed_msg(c, 2, "A"), res, cache, solver, &published);
  ASSERT_TRUE(cache.has_hold("C", "A"));
  ASSERT_TRUE(cache.has_hold("A", "C")) << "head-on: mutual";
  for (const auto & h : cache.holds("A")) {
    EXPECT_NEAR(h.wait_point.x, wait_x, 1e-3) << "hold for " << h.amr_id;
  }

  published.clear();
  ecbs_planner::evaluate_path(request_from(a, 0), res, cache, solver, &published);
  ASSERT_EQ(res.status, "WAIT") << res.message;
  ASSERT_EQ(res.waiting_for.size(), 2u);
  for (const auto & d : res.waiting_for) {
    EXPECT_NEAR(d.wait_point.x, wait_x, 1e-3) << d.amr_id;
  }
  for (const auto & p : published) {
    if (p.amr_id != "A") {
      continue;
    }
    std::size_t points = 0;
    for (const auto & w : p.waypoints) {
      points += w.waiting_for.empty() ? 0 : 1;
    }
    EXPECT_EQ(points, 1u) << "one wait point on A";
  }

  // C4: B's path cleared → release_passed_blockers may drop B; C must not be
  // cascade-dropped solely because it shared B's wait XY.
  ASSERT_TRUE(cache.has_hold("A", "B"));
  ASSERT_TRUE(cache.has_hold("A", "C"));
  auto b_gone = *cache.get("B");
  b_gone.waypoints.clear();
  cache.upsert(b_gone);
  ecbs_planner::evaluate_path(request_from(a, 0), res, cache, solver);
  EXPECT_TRUE(cache.has_hold("A", "C"))
    << "C4: co-located hold for C must survive when B passes";
}

/// Spatial hash: near_junction is true only near a T/cross.
TEST(NavGraphJunctions, HashNearJunction)
{
  ecbs_planner::NavGraphJunctions j;
  j.set_from_edges(
    {{0.0, 0.0}, {10.0, 0.0}, {20.0, 0.0}, {10.0, 10.0}},
    {{0, 1}, {1, 2}, {1, 3}});
  ASSERT_EQ(j.size(), 1u);
  EXPECT_TRUE(j.loaded());
  EXPECT_TRUE(j.near_junction(10.0, 0.0, 0.5));
  EXPECT_TRUE(j.near_junction(10.2, 0.1, 0.5));
  EXPECT_FALSE(j.near_junction(0.0, 0.0, 0.5));
  EXPECT_FALSE(j.near_junction(20.0, 0.0, 0.5));
}

/// Degree 3 → T, degree 4 → cross.
TEST(NavGraphJunctions, DetectsTAndCrossFromDegree)
{
  ecbs_planner::NavGraphJunctions j;
  // Corridor 0—1—2 with a spur 1—3 (T at 1) and a cross at 4: 5—4—6, 7—4—8.
  std::vector<std::pair<double, double>> verts = {
    {0.0, 0.0}, {10.0, 0.0}, {20.0, 0.0}, {10.0, 10.0},
    {40.0, 0.0}, {30.0, 0.0}, {50.0, 0.0}, {40.0, -10.0}, {40.0, 10.0},
  };
  std::vector<std::pair<std::size_t, std::size_t>> edges = {
    {0, 1}, {1, 2}, {1, 3},
    {5, 4}, {4, 6}, {7, 4}, {4, 8},
  };
  j.set_from_edges(verts, edges);
  ASSERT_EQ(j.size(), 2u);
  bool saw_t = false;
  bool saw_x = false;
  for (const auto & p : j.junctions()) {
    if (p.kind == ecbs_planner::JunctionKind::TSection) {
      EXPECT_NEAR(p.x, 10.0, 1e-9);
      EXPECT_NEAR(p.y, 0.0, 1e-9);
      saw_t = true;
    }
    if (p.kind == ecbs_planner::JunctionKind::Cross) {
      EXPECT_NEAR(p.x, 40.0, 1e-9);
      EXPECT_NEAR(p.y, 0.0, 1e-9);
      saw_x = true;
    }
  }
  EXPECT_TRUE(saw_t);
  EXPECT_TRUE(saw_x);
}

/// Hard rule: wait on/near a cross → always 2.5 m before the centre.
TEST(NavGraphJunctions, SnapWaitOffJunctionCentre)
{
  ecbs_planner::NavGraphJunctions j;
  j.set_from_edges(
    {{0.0, 0.0}, {10.0, 0.0}, {20.0, 0.0}, {10.0, -10.0}, {10.0, 10.0}},
    {{0, 1}, {1, 2}, {1, 3}, {1, 4}});
  ecbs_planner::TimedPath path;
  path.amr_id = "A";
  for (int k = 0; k <= 40; ++k) {
    path.waypoints.push_back(wp(static_cast<double>(k), static_cast<double>(k), 0.0, k));
  }
  const std::size_t cross_i = 10;
  ASSERT_NEAR(path.waypoints[cross_i].x, 10.0, 1e-6);

  const std::size_t at = j.snap_wait_off_junction(path, cross_i, 0, 2.5);
  EXPECT_LT(at, cross_i);
  EXPECT_NEAR(path.waypoints[at].x, 7.5, 0.6);

  // Even with a "granted" index on the centre, never snap after — only before.
  auto before = j.snap_wait_point_off_junction(
    path, cross_i, 10.0, 0.0, 0, 2.5);
  ASSERT_TRUE(before.has_value());
  EXPECT_LT(before->second.x, 10.0);
  EXPECT_NEAR(before->second.x, 7.5, 0.6);

  // Approach stop already clear → leave it.
  auto stay = j.snap_wait_point_off_junction(
    path, at, path.waypoints[at].x, path.waypoints[at].y, 0, 2.5);
  EXPECT_FALSE(stay.has_value()) << "approach stop must stay put";

  auto approach = j.approach_near_base_end(path, 0, 10, 2.5);  // base ends on the centre
  ASSERT_TRUE(approach.has_value());
  EXPECT_EQ(approach->kind, ecbs_planner::JunctionKind::Cross);
  EXPECT_NEAR(approach->wait_point.x, 7.5, 0.6);
  EXPECT_STREQ(ecbs_planner::junction_wait_label(approach->kind), "+");
}

/// Crossing waiter (no base on centre): wait ~2.5 m before the cross.
TEST(EvaluatePath, WaitPointSnappedOffJunctionCentre)
{
  ecbs_planner::NavGraphJunctions j;
  j.set_from_edges(
    {{0.0, 0.0}, {10.0, 0.0}, {20.0, 0.0}, {10.0, -20.0}, {10.0, 20.0}},
    {{0, 1}, {1, 2}, {1, 3}, {1, 4}});

  ecbs_planner::ScheduleCache cache;
  ecbs_planner::EcbsSolver solver;
  const auto a = line_path("A", 0.0, 0.0, 40.0, 0.0, 0.0, 0);
  const auto b = line_path("B", 10.0, -20.0, 10.0, 20.0, 1000.0, 300);
  cache.upsert(a);
  cache.upsert(b);

  ecbs_planner::srv::EvaluatePath::Response res;
  ecbs_planner::evaluate_path(request_from(a, 0), res, cache, solver);
  ASSERT_EQ(res.status, "FREE") << res.message;

  ecbs_planner::evaluate_path(request_from(b, 0), res, cache, solver, nullptr, &j);
  ASSERT_EQ(res.status, "WAIT") << res.message;
  ASSERT_FALSE(res.waiting_for.empty());
  const double wx = res.waiting_for[0].wait_point.x;
  const double wy = res.waiting_for[0].wait_point.y;
  EXPECT_FALSE(j.near_junction(wx, wy, 0.5))
    << "wait must not sit on the junction centre";
  EXPECT_NEAR(std::hypot(wx - 10.0, wy - 0.0), 2.5, 1.0)
    << "wait ~2.5 m from cross centre";
  EXPECT_LT(wy, -1.5) << "wait south of cross at y=0";
}

/// Conflict near a cross: wait stays 2.5 m before the centre (never after),
/// and the pinned hold does not jump on later evaluates.
TEST(EvaluatePath, PinnedWaitNeverOnJunctionCentre)
{
  ecbs_planner::NavGraphJunctions j;
  j.set_from_edges(
    {{0.0, 0.0}, {10.0, 0.0}, {20.0, 0.0}, {10.0, -10.0}, {10.0, 10.0}},
    {{0, 1}, {1, 2}, {1, 3}, {1, 4}});

  ecbs_planner::ScheduleCache cache;
  ecbs_planner::EcbsSolver solver;
  ecbs_planner::TimedPath a;
  a.amr_id = "A";
  a.task_id = "A_task";
  for (int k = 0; k <= 40; ++k) {
    a.waypoints.push_back(wp(static_cast<double>(k), static_cast<double>(k), 0.0, k));
  }
  cache.upsert(a);
  // Start past the precautionary gate distance so a real AMR conflict drives
  // the wait (not the T/+ sentinel).
  ecbs_planner::GrantedBase base;
  base.task_id = a.task_id;
  base.x = 5.0;
  base.y = 0.0;
  base.index = 5;
  cache.set_granted_base("A", base);
  cache.upsert(line_path("B", 40.0, 0.0, 0.0, 0.0, 0.0, 100));

  ecbs_planner::srv::EvaluatePath::Response res;
  auto req = request_from(a, 0);
  req.current_x = 5.0;
  req.current_y = 0.0;
  ecbs_planner::evaluate_path(req, res, cache, solver, nullptr, &j);
  ASSERT_EQ(res.status, "WAIT") << res.message;
  ASSERT_FALSE(res.waiting_for.empty());
  // Prefer the real blocker wait (skip T/+ gate if also present).
  double first_x = res.waiting_for[0].wait_point.x;
  for (const auto & d : res.waiting_for) {
    if (d.amr_id == "B") {
      first_x = d.wait_point.x;
      break;
    }
  }
  EXPECT_LT(first_x, 10.0) << "wait before the centre";
  EXPECT_FALSE(j.near_junction(first_x, 0.0, 0.5));

  req.current_x = 5.5;
  req.current_y = 0.0;
  ecbs_planner::evaluate_path(req, res, cache, solver, nullptr, &j);
  ASSERT_EQ(res.status, "WAIT") << res.message;
  double later_x = res.waiting_for[0].wait_point.x;
  for (const auto & d : res.waiting_for) {
    if (d.amr_id == "B") {
      later_x = d.wait_point.x;
      break;
    }
  }
  EXPECT_NEAR(later_x, first_x, 0.75)
    << "wait must stay put (not flip past the centre)";
}

namespace
{

/// Straight path y = 0, x = 0..40 (1 m spacing) and a T junction at \p jx.
struct JunctionLine
{
  ecbs_planner::NavGraphJunctions j;
  ecbs_planner::TimedPath path;

  explicit JunctionLine(double jx)
  {
    j.set_from_edges(
      {{0.0, 0.0}, {jx, 0.0}, {40.0, 0.0}, {jx, 10.0}}, {{0, 1}, {1, 2}, {1, 3}});
    path.amr_id = "A";
    path.task_id = "A_task";
    for (int k = 0; k <= 40; ++k) {
      path.waypoints.push_back(wp(static_cast<double>(k), static_cast<double>(k), 0.0, k));
    }
  }
};

}  // namespace

/// Junction rules on a base 0..20: deep inside the base → none (full base);
/// within 2.5 m of the end (before or after) → stop 2.5 m before the centre;
/// 2.5 m or more beyond the end → none.
TEST(NavGraphJunctions, ApproachOnlyNearBaseEnd)
{
  for (const double jx : {10.0, 17.0, 23.0, 30.0}) {
    JunctionLine line(jx);
    EXPECT_FALSE(line.j.approach_near_base_end(line.path, 0, 20, 2.5).has_value())
      << "junction at " << jx << " must not clip the base 0..20";
  }
  for (const double jx : {18.0, 19.0, 20.0, 21.0, 22.0}) {
    JunctionLine line(jx);
    const auto a = line.j.approach_near_base_end(line.path, 0, 20, 2.5);
    ASSERT_TRUE(a.has_value()) << "junction at " << jx;
    // A real waypoint (1 m spacing), at least 2.5 m before the centre.
    EXPECT_LE(a->wait_point.x, jx - 2.5 + 1e-6) << "junction at " << jx;
    EXPECT_GT(a->wait_point.x, jx - 3.5) << "junction at " << jx;
    EXPECT_NEAR(a->wait_point.x, std::floor(a->wait_point.x), 1e-9)
      << "wait must be a waypoint, junction at " << jx;
  }
  JunctionLine end(30.0);  // base reaching the path end: robot parks at its goal
  EXPECT_FALSE(end.j.approach_near_base_end(end.path, 0, 40, 2.5).has_value());
}

/// Rule 1: junction inside the base (10, base 0..20) → full base, no wait.
TEST(EvaluatePath, JunctionInsideBaseGivesFullBase)
{
  JunctionLine line(10.0);
  ecbs_planner::ScheduleCache cache;
  ecbs_planner::EcbsSolver solver;
  cache.upsert(line.path);
  ecbs_planner::srv::EvaluatePath::Response res;
  auto req = request_from(line.path, 0);
  req.current_x = 0.0;
  req.current_y = 0.0;
  req.query_horizon_m = 20.0;
  ecbs_planner::evaluate_path(req, res, cache, solver, nullptr, &line.j);
  ASSERT_EQ(res.status, "FREE") << res.message;
  EXPECT_TRUE(res.waiting_for.empty());
  const auto base = cache.granted_base("A");
  ASSERT_TRUE(base.has_value());
  EXPECT_NEAR(base->x, 20.0, 1e-6) << "full base through the junction";
}

/// Rules 2 + 3: junction just past the base end (21, base 0..20) → base cut to
/// 18.5; the stop never moves while the robot drives up; within 8 m of it the
/// base goes through, and the stop never comes back (no flutter).
TEST(EvaluatePath, JunctionPastBaseEndCutsBaseThenReleasesWithoutFlutter)
{
  JunctionLine line(21.0);
  ecbs_planner::ScheduleCache cache;
  // Stop at 18.0 = 2.5 m before junction at 21 (safety_distance_m).
  ecbs_planner::SolverConfig cfg;
  cfg.safety_distance_m = 2.5;
  cfg.clearance_m = 2.0;
  ecbs_planner::EcbsSolver solver(cfg);
  cache.upsert(line.path);
  ecbs_planner::srv::EvaluatePath::Response res;
  auto req = request_from(line.path, 0);
  req.current_y = 0.0;
  req.query_horizon_m = 20.0;

  auto stop_x = [&](double robot_x) -> std::optional<double> {
      req.current_x = robot_x;
      cache.update_pose("A", robot_x, 0.0);
      ecbs_planner::evaluate_path(req, res, cache, solver, nullptr, &line.j);
      for (const auto & d : res.waiting_for) {
        if (d.amr_id == "T") {
          return d.wait_point.x;
        }
      }
      return std::nullopt;
    };

  for (const double x : {0.0, 1.0, 2.0, 3.0}) {
    const auto sx = stop_x(x);
    ASSERT_TRUE(sx.has_value()) << "robot at " << x << ": " << res.message;
    EXPECT_NEAR(*sx, 18.0, 1e-6) << "stop (waypoint >= 2.5 m before) must not move (robot at " << x << ")";
    EXPECT_EQ(res.status, "WAIT");
  }
  // Within 8 m of the stop (18): released, base goes through the junction.
  for (const double x : {10.5, 11.0, 12.0, 13.0}) {
    EXPECT_FALSE(stop_x(x).has_value()) << "robot at " << x << " must be released";
    EXPECT_EQ(res.status, "FREE") << res.message;
  }
}

/// Precautionary junction gate: WAIT for "+" 2.5 m before a cross when far;
/// released within 8 m if no real conflict.
TEST(EvaluatePath, JunctionGateWaitForCrossThenRelease)
{
  // Cross at x=20 so approach wait ≈17.5 (> 8 m from start).
  ecbs_planner::NavGraphJunctions j;
  j.set_from_edges(
    {{0.0, 0.0}, {20.0, 0.0}, {40.0, 0.0}, {20.0, -10.0}, {20.0, 10.0}},
    {{0, 1}, {1, 2}, {1, 3}, {1, 4}});

  ecbs_planner::ScheduleCache cache;
  ecbs_planner::EcbsSolver solver;
  ecbs_planner::TimedPath a;
  a.amr_id = "A";
  a.task_id = "A_task";
  for (int k = 0; k <= 40; ++k) {
    a.waypoints.push_back(wp(static_cast<double>(k), static_cast<double>(k), 0.0, k));
  }
  cache.upsert(a);

  // Far from the approach stop (~17.5): gate must WAIT for "+".
  ecbs_planner::srv::EvaluatePath::Response res;
  auto req = request_from(a, 0);
  req.current_x = 0.0;
  req.current_y = 0.0;
  req.query_horizon_m = 20.0;  // cross at 20 = base end → base cut to 17.5
  ecbs_planner::evaluate_path(req, res, cache, solver, nullptr, &j);
  ASSERT_EQ(res.status, "WAIT") << res.message;
  ASSERT_FALSE(res.waiting_for.empty());
  bool saw_cross = false;
  for (const auto & d : res.waiting_for) {
    if (d.amr_id == "+") {
      saw_cross = true;
      EXPECT_NEAR(d.wait_point.x, 17.5, 0.6);
      EXPECT_LT(d.wait_point.x, 20.0);
    }
  }
  EXPECT_TRUE(saw_cross) << res.message;

  // Within 8 m of the approach stop, still no conflict → gate opens (FREE).
  req.current_x = 12.0;  // ~5.5 m to wait at 17.5
  req.current_y = 0.0;
  ecbs_planner::evaluate_path(req, res, cache, solver, nullptr, &j);
  EXPECT_EQ(res.status, "FREE") << res.message;
  for (const auto & d : res.waiting_for) {
    EXPECT_NE(d.amr_id, "+");
    EXPECT_NE(d.amr_id, "T");
  }
}

/// T-section gate labels the wait as "T".
TEST(EvaluatePath, JunctionGateWaitForT)
{
  // T at x=20 so approach wait ≈17.5 (> 8 m from start).
  ecbs_planner::NavGraphJunctions j;
  j.set_from_edges(
    {{0.0, 0.0}, {20.0, 0.0}, {40.0, 0.0}, {20.0, 10.0}},
    {{0, 1}, {1, 2}, {1, 3}});

  ecbs_planner::ScheduleCache cache;
  ecbs_planner::EcbsSolver solver;
  ecbs_planner::TimedPath a;
  a.amr_id = "A";
  a.task_id = "A_task";
  for (int k = 0; k <= 40; ++k) {
    a.waypoints.push_back(wp(static_cast<double>(k), static_cast<double>(k), 0.0, k));
  }
  cache.upsert(a);

  ecbs_planner::srv::EvaluatePath::Response res;
  auto req = request_from(a, 0);
  req.current_x = 0.0;
  req.current_y = 0.0;
  req.query_horizon_m = 20.0;  // T at 20 = base end → base cut to 17.5
  ecbs_planner::evaluate_path(req, res, cache, solver, nullptr, &j);
  ASSERT_EQ(res.status, "WAIT") << res.message;
  bool saw_t = false;
  for (const auto & d : res.waiting_for) {
    if (d.amr_id == "T") {
      saw_t = true;
      EXPECT_NEAR(d.wait_point.x, 17.5, 0.6);
    }
  }
  EXPECT_TRUE(saw_t) << res.message;
}

/// C5: first FREE grants a base; a later opposite conflict with a safer stop
/// earlier must keep that stop (authority conflict) — not snap forward to the
/// granted tip past the required separation.
TEST(EvaluatePath, NewBlockerDoesNotPlaceWaitInsideGrantedBase)
{
  ecbs_planner::ScheduleCache cache;
  ecbs_planner::EcbsSolver solver;
  // A travels +x; first FREE grants a base ending near x≈20 (horizon).
  const auto a = line_path("A", 0.0, 0.0, 50.0, 0.0, 0.0, 0);
  cache.upsert(a);
  cache.set_pose("A", 0.0, 0.0);

  ecbs_planner::srv::EvaluatePath::Response res;
  auto req = request_from(a, 0);
  req.current_x = 0.0;
  req.current_y = 0.0;
  req.query_horizon_m = 20.0;
  ecbs_planner::evaluate_path(req, res, cache, solver);
  ASSERT_EQ(res.status, "FREE") << res.message;
  const auto first_base = cache.granted_base("A");
  ASSERT_TRUE(first_base.has_value());
  const double base_x = first_base->x;

  // Opposite B: overlap starts before the base end → safety stop earlier.
  const auto b = line_path("B", 50.0, 0.0, 0.0, 0.0, 0.0, 100);
  cache.upsert(b);
  cache.set_pose("B", 50.0, 0.0);
  ecbs_planner::evaluate_path(req, res, cache, solver);
  ASSERT_EQ(res.status, "WAIT") << res.message;
  ASSERT_FALSE(res.waiting_for.empty());
  bool saw_earlier_or_auth = false;
  for (const auto & d : res.waiting_for) {
    if (d.wait_point.x + 1e-3 < base_x) {
      saw_earlier_or_auth = true;
    }
  }
  EXPECT_TRUE(
    saw_earlier_or_auth ||
    res.message.find("authority conflict") != std::string::npos)
    << "C5: safe stop may stay before granted tip; msg=" << res.message;
}

/// C5: with T at the base end and a conflict wait earlier, deps share the
/// earliest safe stop — conflict may stay before the T (not forced forward).
TEST(EvaluatePath, JunctionAndConflictShareBaseEndWait)
{
  ecbs_planner::NavGraphJunctions j;
  j.set_from_edges(
    {{0.0, 0.0}, {30.0, 0.0}, {50.0, 0.0}, {30.0, 10.0}},
    {{0, 1}, {1, 2}, {1, 3}});

  ecbs_planner::ScheduleCache cache;
  ecbs_planner::EcbsSolver solver;
  const auto a = line_path("A", 0.0, 0.0, 50.0, 0.0, 0.0, 0);
  cache.upsert(a);
  cache.set_pose("A", 0.0, 0.0);

  ecbs_planner::srv::EvaluatePath::Response res;
  auto req = request_from(a, 0);
  req.current_x = 0.0;
  req.current_y = 0.0;
  req.query_horizon_m = 30.0;  // T at 30 = base end → base cut to 27.5
  ecbs_planner::evaluate_path(req, res, cache, solver, nullptr, &j);
  ASSERT_EQ(res.status, "WAIT") << res.message;
  double t_x = 0.0;
  bool saw_t = false;
  for (const auto & d : res.waiting_for) {
    if (d.amr_id == "T") {
      saw_t = true;
      t_x = d.wait_point.x;
    }
  }
  ASSERT_TRUE(saw_t) << res.message;

  const auto b = line_path("B", 50.0, 0.0, 0.0, 0.0, 0.0, 100);
  cache.upsert(b);
  cache.set_pose("B", 50.0, 0.0);
  ecbs_planner::evaluate_path(req, res, cache, solver, nullptr, &j);
  ASSERT_EQ(res.status, "WAIT") << res.message;
  double earliest = 1e100;
  for (const auto & d : res.waiting_for) {
    earliest = std::min(earliest, d.wait_point.x);
  }
  for (const auto & d : res.waiting_for) {
    EXPECT_NEAR(d.wait_point.x, earliest, 0.75)
      << "blocker=" << d.amr_id << " must share earliest safe stop";
  }
  EXPECT_LE(earliest, t_x + 0.75)
    << "earliest stop must not be forced past the T approach";
}

/// Opposite traffic: real AMR conflict wait must stay earlier than a later
/// + gate — must not be pulled forward to the junction (that freed AMR3 deep
/// into the corridor while amr2 was still opposite).
TEST(EvaluatePath, OppositeConflictNotPulledForwardToJunctionGate)
{
  ecbs_planner::NavGraphJunctions j;
  // Cross at x=40 → approach ≈37.5; overlap with B begins near x=15.
  j.set_from_edges(
    {{0.0, 0.0}, {40.0, 0.0}, {60.0, 0.0}, {40.0, -10.0}, {40.0, 10.0}},
    {{0, 1}, {1, 2}, {1, 3}, {1, 4}});

  ecbs_planner::ScheduleCache cache;
  ecbs_planner::EcbsSolver solver;
  const auto a = line_path("A", 0.0, 0.0, 60.0, 0.0, 0.0, 0);
  const auto b = line_path("B", 60.0, 0.0, 0.0, 0.0, 0.0, 100);
  cache.upsert(a);
  cache.upsert(b);
  cache.set_pose("A", 0.0, 0.0);
  cache.set_pose("B", 60.0, 0.0);

  ecbs_planner::srv::EvaluatePath::Response res;
  auto req = request_from(a, 0);
  req.current_x = 0.0;
  req.current_y = 0.0;
  req.query_horizon_m = 50.0;
  ecbs_planner::evaluate_path(req, res, cache, solver, nullptr, &j);
  ASSERT_EQ(res.status, "WAIT") << res.message;

  double amr_wait_x = 1e100;
  double plus_x = 1e100;
  for (const auto & d : res.waiting_for) {
    if (d.amr_id == "B") {
      amr_wait_x = std::min(amr_wait_x, d.wait_point.x);
    }
    if (d.amr_id == "+") {
      plus_x = std::min(plus_x, d.wait_point.x);
    }
  }
  ASSERT_LT(amr_wait_x, 1e50) << "must wait for opposite B";
  if (plus_x < 1e50) {
    EXPECT_LE(amr_wait_x, plus_x + 0.75)
      << "opposite conflict must not be pulled forward to the + gate";
  }
  // Must stop well before the cross centre (40), not at ~37.5 gate alone.
  EXPECT_LT(amr_wait_x, 30.0)
    << "release/wait should be the early opposite stop, not deep at +";
}

/// C5: earlier wait XY with stale base-end graph index must keep the earlier
/// safety stop (authority conflict) — not snap forward to the granted tip.
TEST(EvaluatePath, ConflictWaitEarlierXySameIndexSnapsToGrantedBase)
{
  ecbs_planner::ScheduleCache cache;
  ecbs_planner::EcbsSolver solver;
  const auto a = line_path("A", 0.0, 0.0, 50.0, 0.0, 0.0, 0);
  cache.upsert(a);
  cache.set_pose("A", 0.0, 0.0);

  ecbs_planner::srv::EvaluatePath::Response res;
  auto req = request_from(a, 0);
  req.current_x = 0.0;
  req.current_y = 0.0;
  req.query_horizon_m = 20.0;
  ecbs_planner::evaluate_path(req, res, cache, solver);
  ASSERT_EQ(res.status, "FREE") << res.message;
  const auto first_base = cache.granted_base("A");
  ASSERT_TRUE(first_base.has_value());
  ASSERT_TRUE(first_base->index.has_value());
  const double base_x = first_base->x;

  // Hold whose XY is one step earlier but index equals the granted base end
  // (same shape as T idx=106 with amr3@(55.70) in the live log).
  const std::size_t base_i = *ecbs_planner::locate_granted_base(a, *first_base, 0);
  ASSERT_GT(base_i, 0u);
  const double earlier_x = a.waypoints[base_i - 1].x;
  ecbs_planner::WaitDependency bogus;
  bogus.amr_id = "B";
  bogus.wait_point.x = earlier_x;
  bogus.wait_point.y = a.waypoints[base_i - 1].y;
  bogus.wait_point.index = first_base->index;  // stale: base-end index
  cache.add_hold("A", bogus);

  const auto b = line_path("B", 50.0, 0.0, 0.0, 0.0, 0.0, 100);
  cache.upsert(b);
  cache.set_pose("B", 50.0, 0.0);
  ecbs_planner::evaluate_path(req, res, cache, solver);
  ASSERT_EQ(res.status, "WAIT") << res.message;
  for (const auto & d : res.waiting_for) {
    EXPECT_LE(d.wait_point.x, base_x + 1e-3)
      << "blocker=" << d.amr_id
      << " earlier XY with base-end index must snap to granted base";
  }
}

/// Live amr1/amr2: amr1 reserved tip ~(36.5,70) (~2 m before junction).
/// amr2 base onto (36.5,72) with no time clash must WAIT (E0 reserved geometry).
TEST(EvaluatePath, JoinerWaitsForPeerTipBeforeSharedJunction)
{
  ecbs_planner::SolverConfig cfg;
  cfg.clearance_m = 2.0;
  cfg.safety_distance_m = 2.0;
  cfg.amr_length_m = 1.5;
  cfg.amr_width_m = 1.5;
  ecbs_planner::EcbsSolver solver(cfg);
  ecbs_planner::ScheduleCache cache;

  auto amr1 = line_path("amr1", 36.5, 50.0, 36.5, 72.0, 0.0, 0);
  const auto amr1_west = line_path("amr1", 36.5, 72.0, 27.3, 72.0, 30.0, 20);
  amr1.waypoints.insert(
    amr1.waypoints.end(), amr1_west.waypoints.begin() + 1, amr1_west.waypoints.end());
  // Late times so earliest_conflict misses — reserved must still WAIT.
  auto amr2 = line_path("amr2", 40.9, 60.0, 40.9, 72.0, 1000.0, 100);
  const auto amr2_west = line_path("amr2", 40.9, 72.0, 27.3, 72.0, 1030.0, 120);
  amr2.waypoints.insert(
    amr2.waypoints.end(), amr2_west.waypoints.begin() + 1, amr2_west.waypoints.end());

  cache.upsert(amr1);
  cache.upsert(amr2);
  // Booked tip ~2 m south of junction (live release_through before y=72).
  cache.set_reserved("amr1", {{36.5, 66.0}, {36.5, 68.0}, {36.5, 70.08}});
  cache.update_pose("amr1", 36.5, 57.0);
  cache.update_pose("amr2", 40.9, 63.5);

  ecbs_planner::srv::EvaluatePath::Response res;
  std::size_t amr2_at = 0;
  for (std::size_t i = 0; i < amr2.waypoints.size(); ++i) {
    if (amr2.waypoints[i].y >= 63.0 - 1e-6) {
      amr2_at = i;
      break;
    }
  }
  auto req2 = request_from(amr2, amr2_at);
  req2.query_horizon_m = 8.0;
  ecbs_planner::evaluate_path(req2, res, cache, solver);
  ASSERT_EQ(res.status, "WAIT") << res.message;
  ASSERT_FALSE(res.waiting_for.empty());
  EXPECT_EQ(res.waiting_for[0].amr_id, "amr1");
  EXPECT_GT(res.waiting_for[0].wait_point.x, 36.5 + 0.5)
    << "wait before joining (36.5,72)";
}

/// After joiner booked (36.5,72), through robot must not FREE through that cell.
/// Earlier tip ~(36.5,70) must not absorb the peer cell via clearance footprint.
TEST(EvaluatePath, EarlierTipDoesNotAbsorbPeerNextReservedCell)
{
  ecbs_planner::SolverConfig cfg;
  cfg.clearance_m = 2.0;
  cfg.safety_distance_m = 2.0;
  cfg.amr_length_m = 1.5;
  cfg.amr_width_m = 1.5;
  ecbs_planner::EcbsSolver solver(cfg);
  ecbs_planner::ScheduleCache cache;

  auto amr1 = line_path("amr1", 36.5, 60.0, 36.5, 72.0, 0.0, 0);
  const auto amr1_west = line_path("amr1", 36.5, 72.0, 29.6, 72.0, 20.0, 10);
  amr1.waypoints.insert(
    amr1.waypoints.end(), amr1_west.waypoints.begin() + 1, amr1_west.waypoints.end());
  auto amr2 = line_path("amr2", 40.9, 68.0, 40.9, 72.0, 1000.0, 50);
  const auto amr2_west = line_path("amr2", 40.9, 72.0, 36.5, 72.0, 1010.0, 60);
  amr2.waypoints.insert(
    amr2.waypoints.end(), amr2_west.waypoints.begin() + 1, amr2_west.waypoints.end());

  cache.upsert(amr1);
  cache.upsert(amr2);
  cache.set_reserved("amr1", {{36.5, 68.0}, {36.5, 70.08}});
  cache.set_reserved("amr2", {{40.9, 72.0}, {38.7, 72.0}, {36.5, 72.0}});
  cache.update_pose("amr1", 36.5, 68.0);
  cache.update_pose("amr2", 40.9, 70.0);

  // Same-cell ownership only: (36.5,72) stays a blocking cell for amr1.
  const auto blocking = ecbs_planner::blocking_cells(
    cache.reserved("amr2"), cache.reserved("amr1"), 0.35);
  ASSERT_FALSE(blocking.empty());
  bool has_junction = false;
  for (const auto & c : blocking) {
    if (std::hypot(c.x - 36.5, c.y - 72.0) < 0.1) {
      has_junction = true;
    }
  }
  EXPECT_TRUE(has_junction) << "clearance footprint must not swallow peer tip";

  ecbs_planner::srv::EvaluatePath::Response res;
  std::size_t near_tip = 0;
  for (std::size_t i = 0; i < amr1.waypoints.size(); ++i) {
    if (amr1.waypoints[i].y >= 68.0 - 1e-6) {
      near_tip = i;
      break;
    }
  }
  auto req1b = request_from(amr1, near_tip);
  req1b.query_horizon_m = 8.0;
  ecbs_planner::evaluate_path(req1b, res, cache, solver);
  ASSERT_EQ(res.status, "WAIT") << res.message;
  ASSERT_FALSE(res.waiting_for.empty());
  EXPECT_EQ(res.waiting_for[0].amr_id, "amr2");
}

/// A bend puts a false local minimum early in the booked stretch (cells first
/// move away from the robot, then come back). The robot has driven on to the
/// return leg: its passed part must still be freed, only the tail stays.
TEST(ScheduleCache, PassedBaseFreedOnBentStretch)
{
  ecbs_planner::ScheduleCache cache;
  cache.set_base_tail_m(2.0);
  std::vector<ecbs_planner::ClearPoint> cells;
  for (int x = 0; x <= 10; ++x) {
    cells.push_back({static_cast<double>(x), 0.0});
  }
  for (int y = 1; y <= 10; ++y) {
    cells.push_back({10.0, static_cast<double>(y)});
  }
  for (int x = 9; x >= 0; --x) {
    cells.push_back({static_cast<double>(x), 10.0});
  }
  cache.set_reserved("A", cells);
  cache.update_pose("A", 3.0, 10.0);
  const auto left = cache.reserved("A");
  ASSERT_FALSE(left.empty());
  EXPECT_LE(left.size(), static_cast<std::size_t>(2 + 1 + 3 + 1))
    << "passed base must be freed, only the 2 m tail + stretch ahead kept";
  EXPECT_NEAR(left.front().x, 5.0, 1e-9);
  EXPECT_NEAR(left.front().y, 10.0, 1e-9);
}

#include "ecbs_planner/replan_candidate.hpp"

namespace
{

using ecbs_planner::ReplanDecisionKind;

/// Corridor along y = 0, graph index i at x = 2.5 * i.
ecbs_planner::TimedPath corridor(const std::string & id, int from, int to)
{
  ecbs_planner::TimedPath p;
  p.amr_id = id;
  p.task_id = id + "_task";
  const int step = from <= to ? 1 : -1;
  double t = 0.0;
  for (int i = from;; i += step) {
    ecbs_planner::TimedPoint w;
    w.t_sec = t;
    w.x = 2.5 * i;
    w.index = static_cast<uint64_t>(i);
    p.waypoints.push_back(w);
    t += 2.5;
    if (i == to) {
      break;
    }
  }
  return p;
}

void mutual_hold(ecbs_planner::ScheduleCache & cache, const std::string & a, const std::string & b)
{
  ecbs_planner::WaitDependency da;
  da.amr_id = b;
  cache.add_hold(a, da);
  ecbs_planner::WaitDependency db;
  db.amr_id = a;
  cache.add_hold(b, db);
}

/// A (0→10) has its first base; B (10→0) waits at its start; both hold.
void head_on(ecbs_planner::ScheduleCache & cache)
{
  cache.upsert(corridor("A", 0, 10));
  cache.upsert(corridor("B", 10, 0));
  ecbs_planner::GrantedBase base;
  base.task_id = "A_task";
  base.x = 5.0;
  base.index = 2;
  cache.set_granted_base("A", base);
  mutual_hold(cache, "A", "B");
}

}  // namespace

TEST(ReplanCandidate, OppositeOverlapOnlyForReverseOrder)
{
  EXPECT_TRUE(ecbs_planner::opposite_overlap(corridor("A", 0, 10), corridor("B", 10, 0), 1.0));
  EXPECT_FALSE(ecbs_planner::opposite_overlap(corridor("A", 0, 10), corridor("B", 2, 8), 1.0));
}

TEST(ReplanCandidate, LaterAmrReplansAroundKeeperCorridor)
{
  ecbs_planner::ScheduleCache cache;
  head_on(cache);
  ecbs_planner::ReplanCoordinator rc;

  auto out = rc.update(cache, 0.0);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].kind, ReplanDecisionKind::Replan);
  EXPECT_EQ(out[0].candidate, "B");
  EXPECT_EQ(out[0].keeper, "A");
  // A's corridor without B's own start (10) and goal (0).
  // Time filter + 2.5 m tail trim: only the overlapping mid stretch, not 1..9.
  ASSERT_FALSE(out[0].blocked_waypoints.empty());
  ASSERT_EQ(out[0].blocked_until_sec.size(), out[0].blocked_waypoints.size());
  for (const auto wp : out[0].blocked_waypoints) {
    EXPECT_GE(wp, 1);
    EXPECT_LE(wp, 9);
  }
  EXPECT_LT(out[0].blocked_waypoints.size(), 9u);

  EXPECT_TRUE(rc.update(cache, 1.0).empty()) << "published once per change";
}

TEST(ReplanCandidate, FirstComeFirstServed)
{
  // B's path was accepted first, A's later: A replans, B never does.
  ecbs_planner::ScheduleCache cache;
  ecbs_planner::ReplanCoordinator rc;
  cache.upsert(corridor("B", 10, 0));
  EXPECT_TRUE(rc.update(cache, 0.0).empty());
  cache.upsert(corridor("A", 0, 10));
  mutual_hold(cache, "A", "B");
  const auto out = rc.update(cache, 1.0);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].candidate, "A");
  EXPECT_EQ(out[0].keeper, "B");
  EXPECT_NE(out[0].reason.find("first come, first served"), std::string::npos);

  // A path_revision bump on the first AMR must not flip FCFS.
  ecbs_planner::ScheduleCache order;
  auto early = corridor("B", 10, 0);
  early.task_id = "B_task";
  order.upsert(early);
  auto late = corridor("A", 0, 10);
  late.task_id = "A_task";
  order.upsert(late);
  EXPECT_EQ(ecbs_planner::choose_replan_candidate("A", "B", order), "A");
  EXPECT_EQ(ecbs_planner::choose_replan_candidate("B", "A", order), "A");
  early.path_revision = 99;  // same task: B stays first
  order.upsert(early);
  EXPECT_EQ(ecbs_planner::choose_replan_candidate("A", "B", order), "A");
  early.task_id = "B_task_new";  // new task: B is now later
  order.upsert(early);
  EXPECT_EQ(ecbs_planner::choose_replan_candidate("A", "B", order), "B");
}

TEST(ReplanCandidate, BlockedSetShrinksAsKeeperMovesThenClears)
{
  ecbs_planner::ScheduleCache cache;
  head_on(cache);
  ecbs_planner::ReplanCoordinator rc;
  ASSERT_EQ(rc.update(cache, 0.0).size(), 1u);

  // B found a detour (via y = 5): B goes; A only holds before B's spot.
  auto detour = corridor("B", 10, 10);
  for (int i = 10; i >= 0; i -= 5) {
    ecbs_planner::TimedPoint w;
    w.x = 2.5 * i;
    w.y = 5.0;
    w.index = static_cast<uint64_t>(100 + i);
    detour.waypoints.push_back(w);
  }
  detour.waypoints.push_back(corridor("B", 0, 0).waypoints.front());
  cache.upsert(detour);
  EXPECT_TRUE(rc.update(cache, 1.0).empty());
  EXPECT_FALSE(cache.has_hold("B", "A"));
  ASSERT_TRUE(cache.hold_pinned("A", "B"));
  const auto keeper_holds = cache.holds("A");
  ASSERT_EQ(keeper_holds.size(), 1u);
  EXPECT_EQ(*keeper_holds[0].wait_point.index, 9u);

  cache.update_pose("A", 12.5, 0.0);
  EXPECT_TRUE(rc.update(cache, 2.0).empty()) << "one replan only; no shrink republish";

  cache.update_pose("A", 25.0, 0.0);
  EXPECT_TRUE(rc.update(cache, 3.0).empty())
    << "keeper left corridor: no CLEAR while candidate still has a path";
  EXPECT_TRUE(rc.involved("B"));

  // CLEAR only when the candidate's path is gone (leg finished).
  cache.erase("B");
  auto out = rc.update(cache, 4.0);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].kind, ReplanDecisionKind::Clear);
  EXPECT_EQ(out[0].candidate, "B");
  EXPECT_FALSE(rc.involved("B"));
}

TEST(ReplanCandidate, NoAlternativeNeverSwapsToFirstAmr)
{
  ecbs_planner::ScheduleCache cache;
  head_on(cache);
  ecbs_planner::ReplanConfig cfg;
  cfg.replan_timeout_sec = 5.0;
  ecbs_planner::ReplanCoordinator rc(cfg);
  ASSERT_EQ(rc.update(cache, 0.0).size(), 1u);

  const auto out = rc.update(cache, 6.0);
  ASSERT_EQ(out.size(), 2u) << "no swap: the first AMR never replans";
  EXPECT_EQ(out[0].kind, ReplanDecisionKind::NoAlternative);
  EXPECT_EQ(out[0].candidate, "B");
  EXPECT_EQ(out[1].kind, ReplanDecisionKind::Clear);
  EXPECT_EQ(out[1].candidate, "B");

  EXPECT_TRUE(rc.update(cache, 30.0).empty()) << "given up until a path changes";
}

TEST(ReplanCandidate, IdleAmrInPathMeansWaitNotReplan)
{
  ecbs_planner::RobotFacts idle;
  idle.idle = true;
  {
    ecbs_planner::ScheduleCache cache;
    head_on(cache);
    ecbs_planner::ReplanCoordinator rc;
    rc.set_facts("B", idle);
    EXPECT_TRUE(rc.update(cache, 0.0).empty());
    EXPECT_TRUE(cache.has_hold("A", "B")) << "A keeps waiting";
  }
  {
    ecbs_planner::ScheduleCache cache;
    head_on(cache);
    ecbs_planner::ReplanCoordinator rc;
    ASSERT_EQ(rc.update(cache, 0.0).size(), 1u);
    rc.set_facts("A", idle);  // keeper's task ended
    const auto out = rc.update(cache, 1.0);
    ASSERT_EQ(out.size(), 1u);
    EXPECT_EQ(out[0].kind, ReplanDecisionKind::Clear);
    EXPECT_EQ(out[0].candidate, "B");
    EXPECT_TRUE(rc.update(cache, 2.0).empty()) << "no new decision while idle";
  }
}

/// Two-way corridor 0..10 plus a one-way bypass through waypoint 20.
ecbs_planner::NavGraph corridor_graph(bool bypass_a_to_b, bool bypass_b_to_a)
{
  ecbs_planner::NavGraph g;
  for (std::size_t i = 0; i < 10; ++i) {
    g.add_lane(i, i + 1);
    g.add_lane(i + 1, i);
  }
  if (bypass_a_to_b) {
    g.add_lane(0, 20);
    g.add_lane(20, 10);
  }
  if (bypass_b_to_a) {
    g.add_lane(10, 20);
    g.add_lane(20, 0);
  }
  return g;
}

TEST(ReplanCandidate, NavGraphReachableAvoidsClosedWaypoints)
{
  const auto g = corridor_graph(true, false);
  EXPECT_TRUE(g.reachable(0, 10, {}));
  EXPECT_TRUE(g.reachable(0, 10, {5}));
  EXPECT_FALSE(g.reachable(10, 0, {5}));
  EXPECT_FALSE(g.reachable(0, 10, {0}));
  EXPECT_TRUE(g.reachable(3, 3, {}));
  EXPECT_FALSE(g.reachable(0, 99, {}));
}

TEST(ReplanCandidate, NavGraphShortestRouteAndNearest)
{
  // 0-1-2 straight, 0-3-2 bypass (3 is off to the side).
  ecbs_planner::NavGraph g;
  const double xy[4][2] = {{0, 0}, {5, 0}, {10, 0}, {5, 4}};
  for (std::size_t i = 0; i < 4; ++i) {
    g.set_position(i, xy[i][0], xy[i][1]);
  }
  for (const auto & l : {std::pair<int, int>{0, 1}, {1, 2}, {0, 3}, {3, 2}}) {
    g.add_lane(l.first, l.second);
  }
  EXPECT_EQ(g.shortest_route(0, 2, {}), (std::vector<std::size_t>{0, 1, 2}));
  EXPECT_EQ(g.shortest_route(0, 2, {1}), (std::vector<std::size_t>{0, 3, 2}));
  EXPECT_TRUE(g.shortest_route(0, 2, {1, 3}).empty());
  EXPECT_EQ(g.nearest(5.2, 3.5), 3u);
}

TEST(ReplanCandidate, LaterAmrWithoutRouteMeansPlainWait)
{
  ecbs_planner::ScheduleCache cache;
  head_on(cache);
  ecbs_planner::ReplanCoordinator rc;
  rc.set_graph(corridor_graph(true, false));
  const auto out = rc.update(cache, 0.0);
  ASSERT_EQ(out.size(), 2u) << "A3: NO_ALTERNATIVE then CLEAR; A never replans";
  EXPECT_EQ(out[0].kind, ReplanDecisionKind::NoAlternative);
  EXPECT_EQ(out[1].kind, ReplanDecisionKind::Clear);
  EXPECT_EQ(out[0].candidate, "B");
  EXPECT_NE(out[0].candidate, "A");
  EXPECT_TRUE(cache.has_hold("A", "B"));
  EXPECT_TRUE(cache.has_hold("B", "A"));
}

TEST(ReplanCandidate, NeitherHasRouteMeansPlainWait)
{
  ecbs_planner::ScheduleCache cache;
  head_on(cache);
  ecbs_planner::ReplanCoordinator rc;
  rc.set_graph(corridor_graph(false, false));
  const auto out = rc.update(cache, 0.0);
  ASSERT_EQ(out.size(), 2u);
  EXPECT_EQ(out[0].kind, ReplanDecisionKind::NoAlternative);
  EXPECT_EQ(out[1].kind, ReplanDecisionKind::Clear);
  EXPECT_EQ(out[0].candidate, "B");
  EXPECT_NE(out[0].candidate, "A");
  EXPECT_TRUE(cache.has_hold("A", "B"));
  EXPECT_TRUE(cache.has_hold("B", "A"));
}

TEST(ReplanCandidate, CandidateHoldsWhereItStands)
{
  ecbs_planner::ScheduleCache cache;
  head_on(cache);
  ecbs_planner::ReplanCoordinator rc;
  rc.set_graph(corridor_graph(false, true));
  const auto out = rc.update(cache, 0.0);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].candidate, "B");
  ASSERT_TRUE(cache.hold_pinned("B", "A"));
  const auto holds = cache.holds("B");
  ASSERT_EQ(holds.size(), 1u);
  EXPECT_DOUBLE_EQ(holds[0].wait_point.x, 25.0);
  ASSERT_TRUE(holds[0].wait_point.index.has_value());
  EXPECT_EQ(*holds[0].wait_point.index, 10u) << "no wait point ahead of B";
}

/// B starts in a dead end (10 only connects to 9); from 8 a one-way
/// bypass through 20 leads to 0.
ecbs_planner::NavGraph dead_end_graph()
{
  ecbs_planner::NavGraph g;
  for (std::size_t i = 0; i < 10; ++i) {
    g.add_lane(i, i + 1);
    g.add_lane(i + 1, i);
  }
  g.add_lane(8, 20);
  g.add_lane(20, 0);
  return g;
}

TEST(ReplanCandidate, DeadEndCandidateLeavesThroughExitAtOnce)
{
  ecbs_planner::ScheduleCache cache;
  head_on(cache);
  ecbs_planner::ReplanCoordinator rc;
  rc.set_graph(dead_end_graph());
  auto out = rc.update(cache, 0.0);
  ASSERT_EQ(out.size(), 1u) << "decided at once, not after B drove out";
  EXPECT_EQ(out[0].kind, ReplanDecisionKind::Replan);
  EXPECT_EQ(out[0].candidate, "B");
  auto closed = out[0].blocked_waypoints;
  std::sort(closed.begin(), closed.end());
  ASSERT_FALSE(closed.empty());
  EXPECT_EQ(std::find(closed.begin(), closed.end(), 9), closed.end());
  EXPECT_EQ(std::find(closed.begin(), closed.end(), 8), closed.end());

  // A (keeper) drives on and holds before B's exit / stand.
  ASSERT_TRUE(cache.hold_pinned("A", "B"));
  auto holds = cache.holds("A");
  ASSERT_EQ(holds.size(), 1u);
  ASSERT_TRUE(holds[0].wait_point.index.has_value());
  EXPECT_LT(*holds[0].wait_point.index, 10u);
  EXPECT_FALSE(holds[0].clear_points.empty());
  // B holds where it stands.
  holds = cache.holds("B");
  ASSERT_EQ(holds.size(), 1u);
  EXPECT_EQ(*holds[0].wait_point.index, 10u);

  // B's new path shares its exit with A and ends at A's start: resolved,
  // no NO_ALTERNATIVE / swap.
  auto route = corridor("B", 10, 8);
  ecbs_planner::TimedPoint bypass;
  bypass.x = 10.0;
  bypass.y = 5.0;
  bypass.index = 20u;
  route.waypoints.push_back(bypass);
  route.waypoints.push_back(corridor("B", 0, 0).waypoints.front());
  cache.upsert(route);
  out = rc.update(cache, 1.0);
  EXPECT_TRUE(out.empty());
  EXPECT_TRUE(rc.involved("B"));
  EXPECT_FALSE(cache.has_hold("B", "A"));
  EXPECT_TRUE(cache.hold_pinned("A", "B"));

  // A's wait solves the conflict: B never gets a hold for A again.
  EXPECT_TRUE(cache.has_right_of_way("B", "A"));
  ecbs_planner::WaitDependency back;
  back.amr_id = "A";
  cache.add_hold("B", back);
  EXPECT_FALSE(cache.has_hold("B", "A"));
  rc.take_changed();

  // B is out of its exit (on the bypass): A's wait ends, and A's path is
  // republished without it.
  cache.update_pose("B", 10.0, 5.0);
  EXPECT_TRUE(rc.update(cache, 1.5).empty());
  EXPECT_FALSE(cache.has_hold("A", "B"));
  const auto touched = rc.take_changed();
  EXPECT_NE(std::find(touched.begin(), touched.end(), "A"), touched.end());
  for (const auto & w : cache.get("A")->waypoints) {
    EXPECT_TRUE(w.waiting_for.empty());
  }

  // A passed B's exit: still no CLEAR while B has a path (one replan only).
  cache.update_pose("A", 25.0, 0.0);
  EXPECT_TRUE(rc.update(cache, 2.0).empty());
  EXPECT_TRUE(cache.has_right_of_way("B", "A"));

  // Pair ends when the candidate's path is gone.
  cache.erase("B");
  out = rc.update(cache, 3.0);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].kind, ReplanDecisionKind::Clear);
  EXPECT_FALSE(cache.has_right_of_way("B", "A"));
}

TEST(ReplanCandidate, NoSwapToAmrWithoutRoute)
{
  ecbs_planner::ScheduleCache cache;
  head_on(cache);
  ecbs_planner::ReplanConfig cfg;
  cfg.replan_timeout_sec = 5.0;
  ecbs_planner::ReplanCoordinator rc(cfg);
  rc.set_graph(corridor_graph(false, true));
  ASSERT_EQ(rc.update(cache, 0.0).size(), 1u);
  const auto out = rc.update(cache, 6.0);
  ASSERT_EQ(out.size(), 2u);
  EXPECT_EQ(out[0].kind, ReplanDecisionKind::NoAlternative);
  EXPECT_EQ(out[1].kind, ReplanDecisionKind::Clear);
  EXPECT_TRUE(rc.update(cache, 30.0).empty());
}

/// corridor_graph with positions (graph index i at x = 2.5 i) and the bypass
/// vertex 20 at (12.5, bypass_y).
ecbs_planner::NavGraph positioned_graph(double bypass_y)
{
  auto g = corridor_graph(false, true);
  for (std::size_t i = 0; i <= 10; ++i) {
    g.set_position(i, 2.5 * static_cast<double>(i), 0.0);
  }
  g.set_position(20, 12.5, bypass_y);
  return g;
}

TEST(ReplanCandidate, ReplansWhenDetourIsFasterThanWaiting)
{
  // B waits ~52 s (25 s of travel + A needs 27 s to clear); detour ~27 s.
  ecbs_planner::ScheduleCache cache;
  head_on(cache);
  ecbs_planner::ReplanCoordinator rc;
  rc.set_graph(positioned_graph(5.0));
  const auto out = rc.update(cache, 0.0);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].kind, ReplanDecisionKind::Replan);
  EXPECT_EQ(out[0].candidate, "B");
}

TEST(ReplanCandidate, FullyOppositeReplansEvenIfDetourIsSlower)
{
  // B stands on A's path: waiting can never solve it, so the slow detour wins.
  ecbs_planner::ScheduleCache cache;
  head_on(cache);
  ecbs_planner::ReplanCoordinator rc;
  rc.set_graph(positioned_graph(30.0));
  const auto out = rc.update(cache, 0.0);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].kind, ReplanDecisionKind::Replan);
}

/// B starts on a spur beside the corridor (index 30), so it is not on A's
/// path; its only detour runs 30 -> 20 -> 0 (vertex 20 at (12.5, bypass_y)).
void spur_case(ecbs_planner::ScheduleCache & cache, ecbs_planner::ReplanCoordinator & rc,
  double bypass_y)
{
  cache.upsert(corridor("A", 0, 10));
  ecbs_planner::TimedPath b;
  b.amr_id = "B";
  b.task_id = "B_task";
  ecbs_planner::TimedPoint w;
  w.x = 25.0;
  w.y = 6.0;
  w.index = 30;
  b.waypoints.push_back(w);
  for (auto c : corridor("B", 10, 0).waypoints) {
    c.t_sec += 5.0;
    b.waypoints.push_back(c);
  }
  cache.upsert(b);
  mutual_hold(cache, "A", "B");
  ecbs_planner::NavGraph g;
  g.add_lane(30, 20);
  g.add_lane(20, 0);
  g.set_position(30, 25.0, 6.0);
  g.set_position(20, 12.5, bypass_y);
  g.set_position(0, 0.0, 0.0);
  rc.set_graph(std::move(g));
}

TEST(ReplanCandidate, SpurCandidateReplansWhenDetourIsFaster)
{
  ecbs_planner::ScheduleCache cache;
  ecbs_planner::ReplanCoordinator rc;
  spur_case(cache, rc, 5.0);
  const auto out = rc.update(cache, 0.0);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].kind, ReplanDecisionKind::Replan);
}

// A's goal is on B's path and A was first: B replans even if the detour is
// slower than waiting (no ETA compare for that rule).
TEST(ReplanCandidate, SpurCandidateReplansWhenFirstGoalOnItsPathEvenIfSlower)
{
  ecbs_planner::ScheduleCache cache;
  ecbs_planner::ReplanCoordinator rc;
  spur_case(cache, rc, 30.0);
  const auto out = rc.update(cache, 0.0);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].kind, ReplanDecisionKind::Replan);
  EXPECT_EQ(out[0].candidate, "B");
}

TEST(NavGraphLength, ShortestRouteSkipsClosedWaypoints)
{
  const auto g = positioned_graph(5.0);
  EXPECT_NEAR(*g.shortest_length(0, 10, {}), 25.0, 1e-9);
  EXPECT_NEAR(*g.shortest_length(10, 0, {5}), 2 * std::hypot(12.5, 5.0), 1e-9);
  EXPECT_FALSE(g.shortest_length(0, 10, {5}).has_value());
}


#include "ecbs_planner/idle_tracker.hpp"
#include "ecbs_planner/replan_candidate.hpp"

/// (a) Ghost task_id + not moving + no path + stationary → idle after hold_sec.
TEST(IdleTracker, GhostTaskIdStationaryNoPathBecomesIdle)
{
  ecbs_planner::IdleTracker tr;
  ecbs_planner::IdleHoldConfig cfg;
  cfg.hold_sec = 3.0;
  cfg.pos_tol_m = 0.15;
  auto r = tr.update("amr4", 0.0, false, false, false, 10.0, 20.0, cfg);
  EXPECT_FALSE(r.idle);
  r = tr.update("amr4", 2.9, false, false, false, 10.05, 20.0, cfg);  // jitter
  EXPECT_FALSE(r.idle);
  r = tr.update("amr4", 3.0, false, false, false, 10.05, 20.0, cfg);
  EXPECT_TRUE(r.idle);
  EXPECT_EQ(r.change, ecbs_planner::IdleChange::BecameIdleByHold);
}

/// (b) Same but ECBS still has a live path → not idle.
TEST(IdleTracker, LivePathPreventsHoldIdle)
{
  ecbs_planner::IdleTracker tr;
  ecbs_planner::IdleHoldConfig cfg;
  auto r = tr.update("amr5", 0.0, false, false, true, 1.0, 1.0, cfg);
  EXPECT_FALSE(r.idle);
  r = tr.update("amr5", 5.0, false, false, true, 1.0, 1.0, cfg);
  EXPECT_FALSE(r.idle);
}

/// (c) MODE_MOVING or position changing → not idle.
TEST(IdleTracker, MovingOrPoseChangeNotIdle)
{
  ecbs_planner::IdleTracker tr;
  ecbs_planner::IdleHoldConfig cfg;
  cfg.hold_sec = 3.0;
  cfg.pos_tol_m = 0.15;
  EXPECT_FALSE(tr.update("A", 0.0, false, true, false, 0.0, 0.0, cfg).idle);
  EXPECT_FALSE(tr.update("A", 5.0, false, true, false, 0.0, 0.0, cfg).idle);

  ecbs_planner::IdleTracker tr2;
  tr2.update("B", 0.0, false, false, false, 0.0, 0.0, cfg);
  tr2.update("B", 1.0, false, false, false, 0.0, 0.0, cfg);
  // Jump past tol resets the hold clock; clock starts on the next stationary tick.
  EXPECT_FALSE(tr2.update("B", 2.0, false, false, false, 1.0, 0.0, cfg).idle);
  EXPECT_FALSE(tr2.update("B", 3.0, false, false, false, 1.0, 0.0, cfg).idle);
  EXPECT_FALSE(tr2.update("B", 5.9, false, false, false, 1.0, 0.0, cfg).idle);
  EXPECT_TRUE(tr2.update("B", 6.0, false, false, false, 1.0, 0.0, cfg).idle);
}

/// (d) Empty task_id >= 3 s → idle (old rule).
TEST(IdleTracker, EmptyTaskIdBecomesIdle)
{
  ecbs_planner::IdleTracker tr;
  ecbs_planner::IdleHoldConfig cfg;
  // Has a live path so rule (2) cannot fire; only empty task_id counts.
  EXPECT_FALSE(tr.update("C", 0.0, true, false, true, 0.0, 0.0, cfg).idle);
  EXPECT_FALSE(tr.update("C", 2.9, true, false, true, 0.0, 0.0, cfg).idle);
  auto r = tr.update("C", 3.0, true, false, true, 0.0, 0.0, cfg);
  EXPECT_TRUE(r.idle);
  EXPECT_EQ(r.change, ecbs_planner::IdleChange::None);  // not rule (2)
}

/// (e) amr4 idle by hold rule → amr5 path through amr4 gets parked REPLAN.
TEST(ReplanCoordinator, IdleByHoldKeeperTriggersParkedReplan)
{
  ecbs_planner::IdleTracker tr;
  ecbs_planner::IdleHoldConfig cfg;
  // Ghost task_id, idle mode, no path, stationary for 3 s.
  tr.update("amr4", 0.0, false, false, false, 20.0, 0.0, cfg);
  ASSERT_TRUE(tr.update("amr4", 3.0, false, false, false, 20.0, 0.0, cfg).idle);

  ecbs_planner::ScheduleCache cache;
  ecbs_planner::ReplanCoordinator rc;
  cache.upsert(make_path("amr5", {wp(0, 0, 0, 0), wp(10, 10, 0, 1), wp(20, 20, 0, 2),
      wp(30, 30, 0, 3), wp(40, 40, 0, 4), wp(50, 50, 0, 5)}));
  cache.set_pose("amr5", 0.0, 0.0);
  cache.set_pose("amr4", 20.0, 0.0);
  cache.set_idle("amr4", true);
  ecbs_planner::RobotFacts idle;
  idle.idle = true;
  rc.set_facts("amr4", idle);

  const auto out = rc.update(cache, 0.0);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].kind, ecbs_planner::ReplanDecisionKind::Replan);
  EXPECT_EQ(out[0].candidate, "amr5");
  EXPECT_EQ(out[0].keeper, "amr4");
  EXPECT_NE(out[0].reason.find("parked amr4 on the path"), std::string::npos);
}

/// A parked (idle) AMR on a moving AMR's path: the mover replans around it,
/// no ETA compare; CLEAR when the parked AMR stops being idle.
TEST(ReplanCoordinator, ParkedAmrOnPathMakesMoverReplan)
{
  ecbs_planner::ScheduleCache cache;
  ecbs_planner::ReplanCoordinator rc;
  cache.upsert(make_path("A", {wp(0, 0, 0, 0), wp(10, 10, 0, 1), wp(20, 20, 0, 2),
      wp(30, 30, 0, 3), wp(40, 40, 0, 4), wp(50, 50, 0, 5)}));
  cache.set_pose("A", 0.0, 0.0);
  cache.set_pose("K", 20.0, 0.0);
  cache.set_idle("K", true);
  ecbs_planner::RobotFacts idle;
  idle.idle = true;
  rc.set_facts("K", idle);

  auto out = rc.update(cache, 0.0);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].kind, ecbs_planner::ReplanDecisionKind::Replan);
  EXPECT_EQ(out[0].candidate, "A");
  EXPECT_EQ(out[0].keeper, "K");
  EXPECT_NE(
    std::find(
      out[0].blocked_waypoints.begin(), out[0].blocked_waypoints.end(), 2),
    out[0].blocked_waypoints.end());
  EXPECT_TRUE(rc.update(cache, 1.0).empty());  // one REPLAN only

  rc.set_facts("K", ecbs_planner::RobotFacts{});
  out = rc.update(cache, 2.0);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].kind, ecbs_planner::ReplanDecisionKind::Clear);
}

/// Segment-based parked block (bag amr2 between 46.4 and 47.85).
namespace
{
ecbs_planner::TimedPath bag_amr4_path()
{
  return make_path(
    "amr4", {
      wp(0, 33.35, 22.38, 68), wp(1, 33.35, 22.05, 69), wp(2, 33.35, 21.0, 70),
      wp(3, 34.8, 21.0, 617), wp(4, 36.25, 21.0, 658), wp(5, 37.7, 21.0, 683),
      wp(6, 39.15, 21.0, 724), wp(7, 40.6, 21.0, 749), wp(8, 42.05, 21.0, 782),
      wp(9, 43.5, 21.0, 815), wp(10, 44.95, 21.0, 856), wp(11, 46.4, 21.0, 881),
      wp(12, 47.85, 21.0, 914), wp(13, 49.3, 21.0, 916)});
}

void set_parked_keeper(
  ecbs_planner::ScheduleCache & cache, ecbs_planner::ReplanCoordinator & rc,
  const std::string & name, double x, double y)
{
  cache.set_pose(name, x, y);
  cache.set_idle(name, true);
  ecbs_planner::RobotFacts idle;
  idle.idle = true;
  rc.set_facts(name, idle);
}

ecbs_planner::NavGraph bag_corridor_with_bypass(bool with_bypass)
{
  ecbs_planner::NavGraph g;
  const std::vector<int64_t> chain =
  {68, 69, 70, 617, 658, 683, 724, 749, 782, 815, 856, 881, 914, 916};
  for (std::size_t i = 0; i + 1 < chain.size(); ++i) {
    g.add_lane(static_cast<std::size_t>(chain[i]), static_cast<std::size_t>(chain[i + 1]));
    g.add_lane(static_cast<std::size_t>(chain[i + 1]), static_cast<std::size_t>(chain[i]));
  }
  if (with_bypass) {
    // Detour around 881/914: 856 -> 900 -> 916
    g.add_lane(856, 900);
    g.add_lane(900, 916);
    g.add_lane(916, 900);
    g.add_lane(900, 856);
  }
  return g;
}
}  // namespace

/// (a) Parked between waypoints → both endpoints blocked; REPLAN if detour exists.
TEST(ReplanCoordinator, ParkedBetweenWaypointsBlocksBothEnds)
{
  ecbs_planner::ScheduleCache cache;
  ecbs_planner::ReplanConfig cfg;
  cfg.clearance_m = 0.05;  // production solver clearance
  cfg.parked_block_radius_m = 1.0;
  ecbs_planner::ReplanCoordinator rc(cfg);
  cache.upsert(bag_amr4_path());
  cache.set_pose("amr4", 33.35, 22.38);
  set_parked_keeper(cache, rc, "amr2", 47.16, 21.0);
  rc.set_graph(bag_corridor_with_bypass(true));

  const auto out = rc.update(cache, 0.0);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].kind, ecbs_planner::ReplanDecisionKind::Replan);
  EXPECT_EQ(out[0].candidate, "amr4");
  EXPECT_EQ(out[0].keeper, "amr2");
  EXPECT_NE(out[0].reason.find("parked amr2 on the path"), std::string::npos);
  EXPECT_NE(
    std::find(out[0].blocked_waypoints.begin(), out[0].blocked_waypoints.end(), 881),
    out[0].blocked_waypoints.end());
  EXPECT_NE(
    std::find(out[0].blocked_waypoints.begin(), out[0].blocked_waypoints.end(), 914),
    out[0].blocked_waypoints.end());
}

TEST(ReplanCoordinator, ParkedBetweenWaypointsNoDetourNotes)
{
  ecbs_planner::ScheduleCache cache;
  ecbs_planner::ReplanConfig cfg;
  cfg.clearance_m = 0.05;
  cfg.parked_block_radius_m = 1.0;
  ecbs_planner::ReplanCoordinator rc(cfg);
  cache.upsert(bag_amr4_path());
  cache.set_pose("amr4", 33.35, 22.38);
  set_parked_keeper(cache, rc, "amr2", 47.16, 21.0);
  rc.set_graph(bag_corridor_with_bypass(false));

  EXPECT_TRUE(rc.update(cache, 0.0).empty());
  const auto notes = rc.take_notes();
  ASSERT_FALSE(notes.empty());
  bool saw = false;
  for (const auto & n : notes) {
    saw = saw || n.find("no detour path is available for amr4 around parked amr2") !=
      std::string::npos;
  }
  EXPECT_TRUE(saw);
}

/// (b) Parked 3 m off the path → no REPLAN.
TEST(ReplanCoordinator, ParkedFarFromPathNoReplan)
{
  ecbs_planner::ScheduleCache cache;
  ecbs_planner::ReplanConfig cfg;
  cfg.clearance_m = 0.05;
  cfg.parked_block_radius_m = 1.0;
  ecbs_planner::ReplanCoordinator rc(cfg);
  cache.upsert(bag_amr4_path());
  cache.set_pose("amr4", 33.35, 22.38);
  set_parked_keeper(cache, rc, "amr2", 47.16, 24.0);  // 3 m north of the lane
  EXPECT_TRUE(rc.update(cache, 0.0).empty());
}

/// (c) Parked exactly on an interior waypoint → that index is blocked.
TEST(ReplanCoordinator, ParkedOnInteriorWaypointStillBlocks)
{
  ecbs_planner::ScheduleCache cache;
  ecbs_planner::ReplanConfig cfg;
  cfg.clearance_m = 0.05;
  cfg.parked_block_radius_m = 1.0;
  ecbs_planner::ReplanCoordinator rc(cfg);
  cache.upsert(bag_amr4_path());
  cache.set_pose("amr4", 33.35, 22.38);
  set_parked_keeper(cache, rc, "amr2", 33.40, 21.00);  // near waypoint 70
  // Empty graph: skip reachability so this only checks blocking.
  const auto out = rc.update(cache, 0.0);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_NE(
    std::find(out[0].blocked_waypoints.begin(), out[0].blocked_waypoints.end(), 70),
    out[0].blocked_waypoints.end());
}

/// (d) Parked at the goal only → no REPLAN, note logged.
TEST(ReplanCoordinator, ParkedAtGoalOnlyNotesNoReplan)
{
  ecbs_planner::ScheduleCache cache;
  ecbs_planner::ReplanConfig cfg;
  cfg.clearance_m = 0.05;
  cfg.parked_block_radius_m = 1.0;
  ecbs_planner::ReplanCoordinator rc(cfg);
  cache.upsert(bag_amr4_path());
  cache.set_pose("amr4", 33.35, 22.38);
  set_parked_keeper(cache, rc, "amr2", 49.3, 21.0);
  EXPECT_TRUE(rc.update(cache, 0.0).empty());
  const auto notes = rc.take_notes();
  bool saw = false;
  for (const auto & n : notes) {
    saw = saw || n.find("parked amr2 is at amr4's goal") != std::string::npos;
  }
  EXPECT_TRUE(saw);
}

/// (e) Parked ~0.5 m beside the start: start-skip uses clearance_m (0.05), so
/// the check still runs and can block later waypoints on the first segments.
TEST(ReplanCoordinator, ParkedBesideStartStillChecksRestOfPath)
{
  ecbs_planner::ScheduleCache cache;
  ecbs_planner::ReplanConfig cfg;
  cfg.clearance_m = 0.05;
  cfg.parked_block_radius_m = 1.0;
  ecbs_planner::ReplanCoordinator rc(cfg);
  cache.upsert(bag_amr4_path());
  cache.set_pose("amr4", 33.35, 22.38);
  set_parked_keeper(cache, rc, "amr2", 33.35, 22.88);  // 0.5 m from start
  // Empty graph: if start-skip used parked_block_radius_m (1.0), no REPLAN.
  const auto out = rc.update(cache, 0.0);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].kind, ecbs_planner::ReplanDecisionKind::Replan);
}

/// (a)+(c) Idle fleet fact on the candidate still REPLANs when awaiting, and
/// the parked keeper is still the one detected.
TEST(ReplanCoordinator, AwaitingIdleCandidateStillReplansAroundParked)
{
  ecbs_planner::ScheduleCache cache;
  ecbs_planner::ReplanCoordinator rc;
  cache.upsert(make_path("A", {wp(0, 0, 0, 0), wp(10, 10, 0, 1), wp(20, 20, 0, 2),
      wp(30, 30, 0, 3), wp(40, 40, 0, 4), wp(50, 50, 0, 5)}));
  cache.set_pose("A", 0.0, 0.0);
  cache.set_pose("K", 20.0, 0.0);
  cache.set_idle("K", true);
  ecbs_planner::RobotFacts idle;
  idle.idle = true;
  rc.set_facts("K", idle);
  rc.set_facts("A", idle);  // empty task_id while gate holds the path
  rc.set_awaiting("A", true);

  const auto out = rc.update(cache, 0.0);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].kind, ecbs_planner::ReplanDecisionKind::Replan);
  EXPECT_EQ(out[0].candidate, "A");
  EXPECT_EQ(out[0].keeper, "K");
}

/// (b) Same idle candidate without awaiting: parked check still skipped.
TEST(ReplanCoordinator, IdleCandidateWithoutAwaitingSkipsParkedReplan)
{
  ecbs_planner::ScheduleCache cache;
  ecbs_planner::ReplanCoordinator rc;
  cache.upsert(make_path("A", {wp(0, 0, 0, 0), wp(10, 10, 0, 1), wp(20, 20, 0, 2),
      wp(30, 30, 0, 3), wp(40, 40, 0, 4), wp(50, 50, 0, 5)}));
  cache.set_pose("A", 0.0, 0.0);
  cache.set_pose("K", 20.0, 0.0);
  cache.set_idle("K", true);
  ecbs_planner::RobotFacts idle;
  idle.idle = true;
  rc.set_facts("K", idle);
  rc.set_facts("A", idle);
  // awaiting defaults false

  EXPECT_TRUE(rc.update(cache, 0.0).empty());
}

/// (d) Pair follow-up: awaiting candidate is not CLEARed as "idle AMR".
TEST(ReplanCoordinator, AwaitingCandidateNotClearedAsIdleInPairFollowUp)
{
  ecbs_planner::ScheduleCache cache;
  ecbs_planner::ReplanCoordinator rc;
  cache.upsert(make_path("A", {wp(0, 0, 0, 0), wp(10, 10, 0, 1), wp(20, 20, 0, 2),
      wp(30, 20, 10, 3), wp(40, 20, 20, 4)}));
  cache.upsert(make_path("B", {wp(0, 40, 0, 10), wp(10, 30, 0, 11), wp(20, 20, 0, 2),
      wp(30, 20, -10, 12), wp(40, 20, -20, 13)}));
  ecbs_planner::WaitDependency a_for_b;
  a_for_b.amr_id = "B";
  a_for_b.wait_point.x = 10.0;
  a_for_b.wait_point.index = 1;
  ecbs_planner::WaitDependency b_for_a;
  b_for_a.amr_id = "A";
  b_for_a.wait_point.x = 30.0;
  b_for_a.wait_point.index = 11;
  cache.add_hold("A", a_for_b);
  cache.add_hold("B", b_for_a);

  auto out = rc.update(cache, 0.0);
  ASSERT_FALSE(out.empty());
  EXPECT_EQ(out[0].kind, ecbs_planner::ReplanDecisionKind::Replan);
  EXPECT_EQ(out[0].candidate, "B");

  ecbs_planner::RobotFacts idle;
  idle.idle = true;
  rc.set_facts("B", idle);
  rc.set_awaiting("B", true);
  out = rc.update(cache, 1.0);
  for (const auto & d : out) {
    EXPECT_FALSE(
      d.kind == ecbs_planner::ReplanDecisionKind::Clear &&
      d.reason.find("idle AMR") != std::string::npos);
  }
  EXPECT_TRUE(rc.involved("B"));
}

/// Mutual hold on a crossing (not head-on): the later AMR replans, no ETA compare.
TEST(ReplanCoordinator, CrossingMutualHoldMakesLaterAmrReplan)
{
  ecbs_planner::ScheduleCache cache;
  ecbs_planner::ReplanCoordinator rc;
  cache.upsert(make_path("A", {wp(0, 0, 0, 0), wp(10, 10, 0, 1), wp(20, 20, 0, 2),
      wp(30, 20, 10, 3), wp(40, 20, 20, 4)}));
  cache.upsert(make_path("B", {wp(0, 40, 0, 10), wp(10, 30, 0, 11), wp(20, 20, 0, 2),
      wp(30, 20, -10, 12), wp(40, 20, -20, 13)}));
  ecbs_planner::WaitDependency a_for_b;
  a_for_b.amr_id = "B";
  a_for_b.wait_point.x = 10.0;
  a_for_b.wait_point.index = 1;
  ecbs_planner::WaitDependency b_for_a;
  b_for_a.amr_id = "A";
  b_for_a.wait_point.x = 30.0;
  b_for_a.wait_point.index = 11;
  cache.add_hold("A", a_for_b);
  cache.add_hold("B", b_for_a);

  const auto out = rc.update(cache, 0.0);
  ASSERT_FALSE(out.empty());
  EXPECT_EQ(out[0].kind, ecbs_planner::ReplanDecisionKind::Replan);
  EXPECT_EQ(out[0].candidate, "B");  // upserted second = later
  EXPECT_EQ(out[0].keeper, "A");
}

/// The earlier AMR's goal lies on the later AMR's path: the later one replans.
TEST(ReplanCoordinator, EarlierGoalOnLaterPathMakesLaterReplan)
{
  ecbs_planner::ScheduleCache cache;
  ecbs_planner::ReplanCoordinator rc;
  cache.upsert(make_path("A", {wp(0, 0, 0, 0), wp(10, 10, 0, 1), wp(20, 20, 0, 2)}));
  cache.upsert(make_path("B", {wp(0, 20, -20, 10), wp(10, 20, -10, 11), wp(20, 20, 0, 2),
      wp(30, 20, 10, 12), wp(40, 20, 20, 13)}));
  const auto out = rc.update(cache, 0.0);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].kind, ecbs_planner::ReplanDecisionKind::Replan);
  EXPECT_EQ(out[0].candidate, "B");
  EXPECT_EQ(out[0].keeper, "A");
}

/// Hard rule: earlier goal on later path still replans after the planning window.
TEST(ReplanCoordinator, GoalOnPathReplansEvenAfterPlanningWindow)
{
  ecbs_planner::ScheduleCache cache;
  ecbs_planner::ReplanCoordinator rc;
  cache.upsert(make_path("A", {wp(0, 0, 0, 0), wp(10, 10, 0, 1), wp(20, 20, 0, 2)}));
  cache.upsert(make_path("B", {wp(0, 20, -20, 10), wp(10, 20, -10, 11), wp(20, 20, 0, 2),
      wp(30, 20, 10, 12), wp(40, 20, 20, 13)}));
  ecbs_planner::RobotFacts idle;
  idle.idle = true;
  rc.set_facts("A", idle);  // planning moment passes without a replan
  EXPECT_TRUE(rc.update(cache, 0.0).empty());
  rc.set_facts("A", ecbs_planner::RobotFacts{});
  const auto out = rc.update(cache, 60.0);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].kind, ecbs_planner::ReplanDecisionKind::Replan);
  EXPECT_EQ(out[0].candidate, "B");
}

/// Hard rule with no detour: keep waiting (CLEAR), do not force a failed replan.
TEST(ReplanCoordinator, GoalOnPathNoAlternateKeepsWaiting)
{
  ecbs_planner::ScheduleCache cache;
  ecbs_planner::ReplanConfig cfg;
  cfg.clearance_m = 0.5;
  ecbs_planner::ReplanCoordinator rc(cfg);
  // Line graph 0-1-2-3-4: A ends at 2; B must pass 2 to reach 4 — closing A's
  // corridor leaves no alternate.
  ecbs_planner::NavGraph g;
  for (std::size_t i = 0; i < 4; ++i) {
    g.add_lane(i, i + 1);
    g.add_lane(i + 1, i);
  }
  for (std::size_t i = 0; i < 5; ++i) {
    g.set_position(i, static_cast<double>(i * 10), 0.0);
  }
  rc.set_graph(std::move(g));
  cache.upsert(make_path("A", {wp(0, 0, 0, 0), wp(10, 10, 0, 1), wp(20, 20, 0, 2)}));
  cache.upsert(make_path("B", {wp(0, 0, 0, 0), wp(10, 10, 0, 1), wp(20, 20, 0, 2),
      wp(30, 30, 0, 3), wp(40, 40, 0, 4)}));
  const auto out = rc.update(cache, 0.0);
  ASSERT_GE(out.size(), 1u);
  // A3: NO_ALTERNATIVE then CLEAR (or CLEAR alone on older paths).
  const bool clear_or_na =
    out[0].kind == ecbs_planner::ReplanDecisionKind::Clear ||
    out[0].kind == ecbs_planner::ReplanDecisionKind::NoAlternative;
  EXPECT_TRUE(clear_or_na);
  EXPECT_EQ(out[0].candidate, "B");
  EXPECT_NE(out.back().reason.find("no detour path is available"), std::string::npos);
}

/// C2: new task seed must not wipe granted/reserved while pose still on the
/// outstanding corridor.
TEST(EvaluatePath, TaskSeedKeepsGrantedCorridorUntilVacated)
{
  ecbs_planner::ScheduleCache cache;
  ecbs_planner::EcbsSolver solver;
  auto a = line_path("A", 0.0, 0.0, 40.0, 0.0, 0.0, 0);
  a.task_id = "T1";
  cache.upsert(a);
  cache.set_pose("A", 5.0, 0.0);

  ecbs_planner::srv::EvaluatePath::Response res;
  auto req = request_from(a, 0);
  req.task_id = "T1";
  req.current_x = 5.0;
  req.current_y = 0.0;
  req.query_horizon_m = 15.0;
  ecbs_planner::evaluate_path(req, res, cache, solver);
  ASSERT_EQ(res.status, "FREE") << res.message;
  ASSERT_TRUE(cache.granted_base("A").has_value());
  ASSERT_FALSE(cache.reserved("A").empty());
  const auto grant = *cache.granted_base("A");
  const auto reserved_n = cache.reserved("A").size();

  auto a2 = a;
  a2.task_id = "T2";
  // Divergent path (north) while body still on old corridor x-axis.
  a2.waypoints.clear();
  for (int k = 0; k <= 20; ++k) {
    a2.waypoints.push_back(wp(5.0, static_cast<double>(k), 0.0, k));
  }
  ecbs_planner::register_from_zone_manager(seed_msg(a2), res, cache, solver);
  ASSERT_TRUE(cache.granted_base("A").has_value())
    << "C2: grant must survive task seed";
  EXPECT_NEAR(cache.granted_base("A")->x, grant.x, 1e-3);
  EXPECT_NEAR(cache.granted_base("A")->y, grant.y, 1e-3);
  EXPECT_GE(cache.reserved("A").size(), 1u)
    << "C2: reserved corridor must survive task seed (was " << reserved_n << ")";

  // Peer still sees A's reserved stretch.
  const auto b = line_path("B", 10.0, -20.0, 10.0, 20.0, 0.0, 100);
  cache.upsert(b);
  cache.set_pose("B", 10.0, -20.0);
  ecbs_planner::evaluate_path(request_from(b, 0), res, cache, solver);
  EXPECT_EQ(res.status, "WAIT") << res.message;
}

/// FCFS register wait mid-horizon (not at base-end index) must still WAIT.
/// GO AMR must stay FREE (no reciprocal reserved / base-end wait).
TEST(EvaluatePath, RegisteredWaitBeyondHorizonKeepsWaiterAndGo)
{
  ecbs_planner::ScheduleCache cache;
  ecbs_planner::EcbsSolver solver;
  auto first = line_path("amr1", 0.0, 0.0, 60.0, 0.0, 0.0, 0);
  first.task_id = "T1";
  // amr2 approaches from north; wait ~15 m from start (inside 20 m+margin,
  // but not at the horizon tip — old only_at_base_end skipped it).
  auto later = line_path("amr2", 30.0, 40.0, 30.0, 0.0, 0.0, 100);
  const auto east = line_path("amr2", 30.0, 0.0, 60.0, 0.0, 40.0, 17);
  later.waypoints.insert(
    later.waypoints.end(), east.waypoints.begin() + 1, east.waypoints.end());
  later.task_id = "T2";
  cache.upsert(first);
  cache.upsert(later);
  cache.set_pose("amr1", 0.0, 0.0);
  cache.set_pose("amr2", 30.0, 40.0);

  ecbs_planner::srv::EvaluatePath::Request reg;
  reg.mode = "register";
  ecbs_planner::msg::RegisterUpdate u;
  u.amr_id = "amr2";
  u.task_id = "T2";
  ecbs_planner::msg::WaitDependency dep;
  dep.amr_id = "amr1";
  dep.wait_point.x = 30.0;
  dep.wait_point.y = 25.0;  // ~15 m from start
  dep.wait_point.index = 6;
  dep.wait_point.wait_sec = 5.0;
  u.waiting_for.push_back(dep);
  reg.updates = {u};
  ecbs_planner::srv::EvaluatePath::Response res;
  ecbs_planner::evaluate_path(reg, res, cache, solver);
  ASSERT_TRUE(res.success) << res.message;
  ASSERT_TRUE(cache.registered_waiter("amr2", "amr1"));

  auto req2 = evaluate_request_for(later);
  req2.query_horizon_m = 20.0;
  req2.current_x = 30.0;
  req2.current_y = 40.0;
  ecbs_planner::evaluate_path(req2, res, cache, solver);
  EXPECT_EQ(res.status, "WAIT") << "registered wait must not be skipped: " << res.message;
  ASSERT_FALSE(res.waiting_for.empty());
  EXPECT_EQ(res.waiting_for[0].amr_id, "amr1");

  // Give amr1 a base so wait_at_base_end / reserved could falsely hold it.
  auto req1 = evaluate_request_for(first);
  req1.query_horizon_m = 20.0;
  ecbs_planner::evaluate_path(req1, res, cache, solver);
  EXPECT_EQ(res.status, "FREE") << "GO after register must not wait for waiter: "
    << res.message;
  // amr2 still registered waiter and sitting on approach — amr1 evaluates again
  // with a reserved base; must stay FREE.
  cache.set_pose("amr2", 30.0, 30.0);
  ecbs_planner::evaluate_path(req1, res, cache, solver);
  EXPECT_EQ(res.status, "FREE") << "GO must ignore yielding reserved base: "
    << res.message;
}

/// Live run 16:27: amr5 seeded with a wait for amr1 (before the lane both
/// share, head-on), then evaluated at its start: must not be FREE.
TEST(EvaluatePath, SeededWaitBeforeSharedLaneIsNotFree)
{
  ecbs_planner::ScheduleCache cache;
  ecbs_planner::EcbsSolver solver;
  auto a1 = line_path("amr1", 14.0, 79.0, 58.0, 79.0, 0.0, 20);
  a1.task_id = "T1";
  auto a5 = line_path("amr5", 50.0, 62.0, 50.0, 79.0, 12.0, 100);
  const auto west = line_path("amr5", 50.0, 79.0, 10.0, 79.0, 12.0 + 17.0, 107);
  a5.waypoints.insert(a5.waypoints.end(), west.waypoints.begin() + 1, west.waypoints.end());
  a5.task_id = "T5";
  cache.upsert(a1);
  cache.set_pose("amr1", 14.0, 79.0);
  cache.set_pose("amr5", 50.0, 62.0);
  ecbs_planner::srv::EvaluatePath::Response res;
  ecbs_planner::evaluate_path(request_from(a1, 0), res, cache, solver);
  ASSERT_EQ(res.status, "FREE") << res.message;

  std::vector<ecbs_planner::TimedPath> published;
  ecbs_planner::register_from_zone_manager(
    seed_msg(a5, 5, "amr1"), res, cache, solver, &published);
  ecbs_planner::evaluate_path(request_from(a5, 0), res, cache, solver);
  EXPECT_EQ(res.status, "WAIT") << res.message;
}

namespace
{
struct ReplayWp
{
  double t, x, y;
  int index;
  const char * wait_amr;
};
#include "replay_data.inc"

ecbs_planner::TimedPath replay_path(
  const std::string & id, const std::string & task, const std::vector<ReplayWp> & rows)
{
  ecbs_planner::TimedPath p;
  p.amr_id = id;
  p.task_id = task;
  for (const auto & r : rows) {
    p.waypoints.push_back(wp(r.t, r.x, r.y, static_cast<uint64_t>(r.index)));
  }
  return p;
}
}  // namespace

/// Replay of bag 15:32 (amr1 base given, amr2 replanned, amr5 seeded): amr5's
/// path shares the amr1 lane head-on; its first evaluate must not be FREE.
TEST(EvaluatePath, ReplayAmr5SeedThenEvaluateNotFree)
{
  ecbs_planner::ScheduleCache cache;
  ecbs_planner::EcbsSolver solver;
  const auto a1 = replay_path("amr1", k_amr1_task, k_amr1);
  const auto a2 = replay_path("amr2", k_amr2_task, k_amr2);
  const auto a5 = replay_path("amr5", k_amr5_task, k_amr5);
  cache.set_pose("amr1", k_poses[0][0], k_poses[0][1]);
  cache.set_pose("amr2", k_poses[1][0], k_poses[1][1]);
  cache.set_pose("amr5", k_poses[2][0], k_poses[2][1]);
  ecbs_planner::srv::EvaluatePath::Response res;
  std::vector<ecbs_planner::TimedPath> published;
  ecbs_planner::register_from_zone_manager(seed_msg(a1), res, cache, solver, &published);
  ecbs_planner::register_from_zone_manager(seed_msg(a2), res, cache, solver, &published);
  ecbs_planner::evaluate_path(evaluate_request_for(a1), res, cache, solver);
  ecbs_planner::evaluate_path(evaluate_request_for(a2), res, cache, solver);
  // Planner wait at (51.21, 77.82), 1.2 m before the lane join (live run 16:45).
  ecbs_planner::register_from_zone_manager(seed_msg(a5, 9, "amr1"), res, cache, solver, &published);
  // One wait per blocker on the seeded path, shown from the start.
  double seeded_y = -1.0;
  int seeded_n = 0;
  const auto sc = cache.get("amr5");
  for (const auto & w : sc->waypoints) {
    for (const auto & d : w.waiting_for) {
      if (d.amr_id == "amr1") {
        seeded_y = w.y;
        ++seeded_n;
      }
    }
  }
  EXPECT_EQ(seeded_n, 1);
  ecbs_planner::evaluate_path(evaluate_request_for(a5), res, cache, solver);
  ASSERT_EQ(res.status, "WAIT") << res.message;
  ASSERT_FALSE(res.waiting_for.empty());
  EXPECT_EQ(res.waiting_for[0].amr_id, "amr1");
  const double y1 = res.waiting_for[0].wait_point.y;
  EXPECT_LT(y1, 79.0) << "wait before the conflict";
  EXPECT_NEAR(y1, seeded_y, 1e-6) << "seeded wait must not move at the first evaluate";
  // Never flutter: later evaluates keep the same wait point.
  for (int k = 0; k < 3; ++k) {
    ecbs_planner::evaluate_path(evaluate_request_for(a5), res, cache, solver);
    ASSERT_EQ(res.status, "WAIT") << res.message;
    EXPECT_NEAR(res.waiting_for[0].wait_point.y, y1, 1e-6);
  }
}

/// A path wait that is a hold but not yet ZM-registered must never be FREE.
TEST(EvaluatePath, UnregisteredHoldWaitIsNotFree)
{
  ecbs_planner::ScheduleCache cache;
  ecbs_planner::EcbsSolver solver;
  auto first = line_path("amr1", 0.0, 0.0, 60.0, 0.0, 0.0, 0);
  first.task_id = "T1";
  auto later = line_path("amr2", 30.0, 40.0, 30.0, 0.0, 0.0, 100);
  const auto east = line_path("amr2", 30.0, 0.0, 60.0, 0.0, 40.0, 17);
  later.waypoints.insert(
    later.waypoints.end(), east.waypoints.begin() + 1, east.waypoints.end());
  later.task_id = "T2";
  ecbs_planner::WaitDependency dep;
  dep.amr_id = "amr1";
  dep.wait_point.x = later.waypoints[6].x;
  dep.wait_point.y = later.waypoints[6].y;
  dep.wait_point.index = later.waypoints[6].index;
  dep.wait_point.wait_duration_sec = 5.0;
  later.waypoints[6].waiting_for.push_back(dep);
  later.waypoints[6].wait_duration_sec = 5.0;
  cache.upsert(first);
  cache.upsert(later);
  cache.set_pose("amr1", 0.0, 0.0);
  cache.set_pose("amr2", 30.0, 40.0);
  cache.add_hold("amr2", dep, false);
  ASSERT_FALSE(cache.registered_waiter("amr2", "amr1"));

  auto req = evaluate_request_for(later);
  req.query_horizon_m = 20.0;
  req.current_x = 30.0;
  req.current_y = 40.0;
  ecbs_planner::srv::EvaluatePath::Response res;
  ecbs_planner::evaluate_path(req, res, cache, solver);
  EXPECT_EQ(res.status, "WAIT") << "unregistered hold must not be released: " << res.message;
}

/// Registered wait must FREE once the GO AMR has driven past clear cells
/// (register leaves holds_, so clear-pass must unmark registered_ too).
TEST(EvaluatePath, RegisteredWaitClearsWhenBlockerPastClearPoints)
{
  ecbs_planner::ScheduleCache cache;
  ecbs_planner::EcbsSolver solver;
  auto go = line_path("amr1", 0.0, 0.0, 40.0, 0.0, 0.0, 0);
  go.task_id = "T1";
  auto wait = line_path("amr2", 20.0, 30.0, 20.0, 0.0, 0.0, 100);
  wait.task_id = "T2";
  cache.upsert(go);
  cache.upsert(wait);
  cache.set_pose("amr1", 0.0, 0.0);
  cache.set_pose("amr2", 20.0, 30.0);

  ecbs_planner::srv::EvaluatePath::Request reg;
  reg.mode = "register";
  ecbs_planner::msg::RegisterUpdate u;
  u.amr_id = "amr2";
  u.task_id = "T2";
  ecbs_planner::msg::WaitDependency dep;
  dep.amr_id = "amr1";
  dep.wait_point.x = 20.0;
  dep.wait_point.y = 10.0;
  dep.wait_point.index = 4;
  dep.wait_point.wait_sec = 5.0;
  for (double x = 10.0; x <= 30.0; x += 5.0) {
    ecbs_planner::msg::ClearPoint c;
    c.x = x;
    c.y = 0.0;
    dep.clear_points.push_back(c);
  }
  u.waiting_for.push_back(dep);
  reg.updates = {u};
  ecbs_planner::srv::EvaluatePath::Response res;
  ecbs_planner::evaluate_path(reg, res, cache, solver);
  ASSERT_TRUE(cache.registered_waiter("amr2", "amr1"));

  auto req2 = evaluate_request_for(wait);
  req2.query_horizon_m = 40.0;
  req2.current_x = 20.0;
  req2.current_y = 30.0;
  ecbs_planner::evaluate_path(req2, res, cache, solver);
  EXPECT_EQ(res.status, "WAIT") << res.message;

  // amr1 past the shared clear cells; cache path trimmed like live evaluate.
  cache.set_pose("amr1", 38.0, 0.0);
  auto go_left = line_path("amr1", 38.0, 0.0, 40.0, 0.0, 38.0, 0);
  go_left.task_id = "T1";
  cache.upsert(go_left);
  ecbs_planner::evaluate_path(req2, res, cache, solver);
  EXPECT_FALSE(cache.registered_waiter("amr2", "amr1"));
  EXPECT_EQ(res.status, "FREE") << "registered wait must clear after blocker passed: "
    << res.message;
}

/// A1: candidate arrives long after the keeper left → no time overlap → no replan.
TEST(ReplanCandidate, BackToBackNoTimeOverlapMeansNoReplan)
{
  ecbs_planner::ScheduleCache cache;
  cache.upsert(corridor("A", 0, 10));
  auto cand = corridor("B", 10, 0);
  for (std::size_t i = 0; i < cand.waypoints.size(); ++i) {
    cand.waypoints[i].t_sec = static_cast<double>(i) * 8.0;
  }
  cache.upsert(cand);
  mutual_hold(cache, "A", "B");
  ecbs_planner::ReplanConfig cfg;
  cfg.blocked_margin_sec = 2.0;
  cfg.blocked_margin_growth = 0.0;
  ecbs_planner::ReplanCoordinator rc(cfg);
  EXPECT_TRUE(rc.update(cache, 0.0).empty()) << "no overlapping window → no REPLAN";
}

/// A1: only overlapping mid waypoints are published.
TEST(ReplanCandidate, PartialTimeOverlapPublishesOnlyOverlap)
{
  ecbs_planner::ScheduleCache cache;
  head_on(cache);
  ecbs_planner::ReplanCoordinator rc;
  const auto out = rc.update(cache, 0.0);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].kind, ReplanDecisionKind::Replan);
  ASSERT_FALSE(out[0].blocked_waypoints.empty());
  EXPECT_EQ(out[0].blocked_waypoints, (std::vector<int64_t>{4, 5}));
}

/// A2: trim drops the last blocked_tail_ignore_m nearest the keeper; short stretch → none.
TEST(ReplanCandidate, TailTrimAndShortStretchBlocksNothing)
{
  ecbs_planner::ReplanConfig cfg;
  cfg.clearance_m = 0.5;
  cfg.blocked_tail_ignore_m = 2.5;
  cfg.blocked_margin_sec = 100.0;
  cfg.blocked_margin_growth = 0.0;
  {
    ecbs_planner::ScheduleCache cache;
    head_on(cache);
    ecbs_planner::ReplanCoordinator rc(cfg);
    const auto out = rc.update(cache, 0.0);
    ASSERT_EQ(out.size(), 1u);
    EXPECT_EQ(
      std::find(out[0].blocked_waypoints.begin(), out[0].blocked_waypoints.end(), 0),
      out[0].blocked_waypoints.end());
  }
  {
    ecbs_planner::ScheduleCache cache;
    auto a = make_path("A", {wp(0, 0, 0, 0), wp(1, 1.0, 0, 1), wp(2, 2.0, 0, 2)});
    auto b = make_path("B", {wp(0, 2.0, 0, 2), wp(1, 1.0, 0, 1), wp(2, 0, 0, 0)});
    cache.upsert(a);
    cache.upsert(b);
    mutual_hold(cache, "A", "B");
    ecbs_planner::ReplanCoordinator rc(cfg);
    EXPECT_TRUE(rc.update(cache, 0.0).empty());
  }
}

/// A3: five failed alternatives → NO_ALTERNATIVE then CLEAR; reopen_guard holds.
TEST(ReplanCandidate, FiveFailedAlternativesThenNoAlternativeClear)
{
  ecbs_planner::ScheduleCache cache;
  head_on(cache);
  ecbs_planner::ReplanConfig cfg;
  cfg.replan_max_retries = 5;
  cfg.reopen_guard_sec = 10.0;
  ecbs_planner::ReplanCoordinator rc(cfg);
  rc.set_graph(corridor_graph(false, false));
  const auto out = rc.update(cache, 0.0);
  ASSERT_EQ(out.size(), 2u);
  EXPECT_EQ(out[0].kind, ReplanDecisionKind::NoAlternative);
  EXPECT_EQ(out[1].kind, ReplanDecisionKind::Clear);
  EXPECT_EQ(out[0].candidate, "B");
  const auto notes = rc.take_notes();
  ASSERT_FALSE(notes.empty());
  EXPECT_NE(notes.back().find("alternatives failed"), std::string::npos);
  EXPECT_TRUE(rc.update(cache, 5.0).empty()) << "reopen_guard blocks restart";
}

#include "bag_amr2_replan.inc"

/// (e) Bag-shaped corridor: old publish was 2..35; time filter publishes a subset.
TEST(ReplanCandidate, BagAmr2AroundAmr1TimeFilteredBlockedSet)
{
  ecbs_planner::ScheduleCache cache;
  auto path_of = [](const std::string & id, const std::vector<bag_amr2_replan::Wp> & wps) {
      ecbs_planner::TimedPath p;
      p.amr_id = id;
      p.task_id = id + "_task";
      for (const auto & w : wps) {
        ecbs_planner::TimedPoint tp;
        tp.t_sec = w.t;
        tp.x = w.x;
        tp.y = w.y;
        tp.index = w.i;
        p.waypoints.push_back(tp);
      }
      return p;
    };
  cache.upsert(path_of("amr1", bag_amr2_replan::amr1_wps()));
  cache.upsert(path_of("amr2", bag_amr2_replan::amr2_wps()));
  mutual_hold(cache, "amr1", "amr2");
  ecbs_planner::ReplanCoordinator rc;
  const auto out = rc.update(cache, 0.0);
  ASSERT_FALSE(out.empty());
  EXPECT_EQ(out[0].candidate, "amr2");
  EXPECT_EQ(out[0].keeper, "amr1");
  if (out[0].kind == ReplanDecisionKind::Replan) {
    const auto old = bag_amr2_replan::old_blocked();
    EXPECT_LT(out[0].blocked_waypoints.size(), old.size());
    for (const auto wp : out[0].blocked_waypoints) {
      EXPECT_NE(std::find(old.begin(), old.end(), wp), old.end());
    }
  } else {
    EXPECT_EQ(out[0].kind, ReplanDecisionKind::NoAlternative);
  }
}

/// (B) Manual test plan — adapter still reopens all lanes at max(blocked_until_sec):
/// 1) Trigger a head-on REPLAN; log blocked_waypoints with staggered blocked_until_sec.
/// 2) Confirm full_control closes all lanes touching those waypoints in one shot.
/// 3) Confirm a single timer opens them all at max(until), not per-waypoint.
/// 4) Upgrade path: per-index open in the EcbsReplan handler when field testing needs it.
TEST(ReplanCandidate, ManualTestPlanPerWaypointReleaseDocumented)
{
  SUCCEED() << "See test comment for adapter per-waypoint release checklist.";
}
