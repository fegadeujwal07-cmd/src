#include <gtest/gtest.h>

#include <limits>

#include "traffic_manager/path_horizon.hpp"

static rmf_fleet_msgs::msg::Location loc(const double x, const double y)
{
  rmf_fleet_msgs::msg::Location l;
  l.x = x;
  l.y = y;
  return l;
}

TEST(PathHorizon, CollectsAllWaypointsUntilTwentyMeters)
{
  std::vector<rmf_fleet_msgs::msg::Location> path;
  path.push_back(loc(0.0, 0.0));
  path.push_back(loc(10.0, 0.0));
  path.push_back(loc(10.0, 10.0));
  path.push_back(loc(0.0, 10.0));

  const auto slice = traffic_manager::slice_path_by_travel_distance(
    path, -1, 0.0, 0.0, 20.0);

  ASSERT_TRUE(slice.valid);
  EXPECT_EQ(slice.waypoints.size(), 3u);
  EXPECT_EQ(slice.first_path_index, 0u);
  EXPECT_EQ(slice.last_path_index, 2u);
  EXPECT_GE(slice.travel_distance_m, 20.0);
}

TEST(PathHorizon, BaseRangeStartsAtRobotPoseNotAtReleasedEnd)
{
  std::vector<rmf_fleet_msgs::msg::Location> path;
  for (int i = 0; i <= 20; ++i) {
    path.push_back(loc(i * 5.0, 0.0));  // 0..100 m
  }

  // Robot at 12 m → nearest waypoint 10 m (index 2); 20 m base ends at 30 m.
  const auto range = traffic_manager::base_range_from_pose(path, 12.0, 0.0, 20.0);

  ASSERT_TRUE(range.valid);
  EXPECT_EQ(range.start_index, 2u);
  EXPECT_EQ(range.end_index, 6u);
  EXPECT_GE(range.travel_distance_m, 20.0);
}

TEST(PathHorizon, FloorBehindPoseStillFindsNearestAhead)
{
  // Regression: early-exit nearest search stopped at a local rise and left
  // start behind the live pose, so 20 m horizon ended at already-released tip.
  std::vector<rmf_fleet_msgs::msg::Location> path;
  for (int i = 0; i <= 30; ++i) {
    path.push_back(loc(0.0, i * 2.0));  // 0..60 m along y
  }
  // Floor at index 4, robot near index 12 (y=24).
  const auto range = traffic_manager::base_range_from_pose(
    path, 0.0, 24.0, 20.0, 4);

  ASSERT_TRUE(range.valid);
  EXPECT_EQ(range.start_index, 12u);
  EXPECT_GE(range.travel_distance_m, 20.0);
  EXPECT_GT(range.end_index, 12u + 5u);
}

TEST(PathHorizon, NewBaseRequestFloorAtTipAdvancesPastAward)
{
  // amr4 stuck: pose still at wait (index 1) after FREE awarded tip 5; pose-only
  // horizon repeats [1..5]. Flooring at awarded tip must open the next 8 m.
  std::vector<rmf_fleet_msgs::msg::Location> path;
  for (int i = 0; i <= 18; ++i) {
    path.push_back(loc(0.0, i * 2.0));  // 2 m spacing
  }
  const int32_t awarded_tip = 5;
  const auto range = traffic_manager::base_range_from_pose(
    path, 0.0, 2.0, 8.0, awarded_tip, true);  // tip-extend: fix at tip

  ASSERT_TRUE(range.valid);
  EXPECT_EQ(range.start_index, static_cast<std::size_t>(awarded_tip));
  EXPECT_GT(range.end_index, static_cast<std::size_t>(awarded_tip));
  EXPECT_GE(range.travel_distance_m, 8.0);
}

TEST(PathHorizon, TipExtendDoesNotSnapToReturnLegNearPose)
{
  // Live amr1 bug: outbound x=36.5 north, then west, then south on x=27.3.
  // Pose mid-outbound near y=51; tip at outbound index 8. Nearest-from-tip
  // snapped to return-leg ~index 25 and released through destination.
  std::vector<rmf_fleet_msgs::msg::Location> path;
  for (int i = 0; i <= 10; ++i) {
    path.push_back(loc(36.5, 43.0 + i * 2.9));  // outbound to ~y=72
  }
  path.push_back(loc(34.0, 72.0));
  path.push_back(loc(31.0, 72.0));
  path.push_back(loc(27.3, 72.0));
  for (int i = 1; i <= 10; ++i) {
    path.push_back(loc(27.3, 72.0 - i * 2.9));  // return south
  }
  const int32_t tip = 8;  // still on outbound (~y=66)
  const auto snapped = traffic_manager::base_range_from_pose(
    path, 36.5, 51.4, 8.0, tip, false);
  // Without fix_start_at_min, nearest from tip is on the return leg.
  EXPECT_GT(snapped.start_index, static_cast<std::size_t>(tip) + 5u);

  const auto fixed = traffic_manager::base_range_from_pose(
    path, 36.5, 51.4, 8.0, tip, true);
  ASSERT_TRUE(fixed.valid);
  EXPECT_EQ(fixed.start_index, static_cast<std::size_t>(tip));
  EXPECT_LT(fixed.end_index, path.size() - 1u)
    << "must not release through destination in one tip-extend";
  EXPECT_GE(fixed.travel_distance_m, 8.0);
  EXPECT_LE(fixed.travel_distance_m, 12.0);
}

TEST(PathHorizon, MinStartPreventsSnapBackOnReturnLeg)
{
  // Outbound x=0..20, return parallel at y=1 so pose near end also matches early x.
  std::vector<rmf_fleet_msgs::msg::Location> path;
  for (int i = 0; i <= 20; ++i) {
    path.push_back(loc(static_cast<double>(i), 0.0));
  }
  for (int i = 19; i >= 0; --i) {
    path.push_back(loc(static_cast<double>(i), 1.0));
  }

  // Without floor, nearest to (5,1) could be early outbound (5,0).
  // With min_start past the turn, horizon stays on the return leg.
  const int32_t return_start = 21;  // first return waypoint (19,1)
  const auto range = traffic_manager::base_range_from_pose(
    path, 5.0, 1.0, 20.0, return_start);

  ASSERT_TRUE(range.valid);
  EXPECT_GE(range.start_index, static_cast<std::size_t>(return_start));
  EXPECT_GT(range.end_index, range.start_index);
}

TEST(PathHorizon, UTurnSegmentsCountTowardHorizon)
{
  std::vector<rmf_fleet_msgs::msg::Location> path;
  path.push_back(loc(0.0, 0.0));
  path.push_back(loc(5.0, 0.0));
  path.push_back(loc(5.0, 5.0));
  path.push_back(loc(5.0, 0.0));
  path.push_back(loc(10.0, 0.0));

  const auto slice = traffic_manager::slice_path_by_travel_distance(
    path, -1, 0.0, 0.0, 20.0);

  ASSERT_TRUE(slice.valid);
  EXPECT_GE(slice.travel_distance_m, 20.0);
  EXPECT_EQ(slice.waypoints.size(), 5u);
  EXPECT_EQ(slice.waypoints.front().x, 0.0);
  EXPECT_EQ(slice.waypoints.back().x, 10.0);
}

TEST(PathHorizon, NearestLegHonorsTolerance)
{
  std::vector<rmf_fleet_msgs::msg::Location> path;
  for (int i = 0; i <= 10; ++i) {
    path.push_back(loc(i * 2.0, 0.0));
  }
  // Far from path → -1 when tolerance is tight.
  EXPECT_EQ(
    traffic_manager::nearest_leg_index(path, 100.0, 100.0, 0.5, 0), -1);
  // Near index 3 (x=6).
  EXPECT_EQ(
    traffic_manager::nearest_leg_index(path, 6.1, 0.1, 0.5, 0), 3);
}

TEST(PathHorizon, NearestLegPrefersContinuousIndexOverLoopSnap)
{
  // Outbound then return near same XY — prefer_near keeps continuous tracking.
  std::vector<rmf_fleet_msgs::msg::Location> path;
  for (int i = 0; i <= 10; ++i) {
    path.push_back(loc(0.0, i * 2.0));
  }
  for (int i = 9; i >= 0; --i) {
    path.push_back(loc(0.2, i * 2.0));
  }
  const int32_t near = traffic_manager::nearest_leg_index(
    path, 0.0, 10.0, 1.0, 0, 5, std::numeric_limits<double>::quiet_NaN(), 8);
  EXPECT_GE(near, 4);
  EXPECT_LE(near, 8);
}
