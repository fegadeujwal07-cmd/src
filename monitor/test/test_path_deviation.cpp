#include <gtest/gtest.h>
#include <cmath>

#include "monitor/path_deviation.hpp"

// Test the static point_to_segment_distance geometric helper
TEST(PathDeviationTest, PointOnSegment)
{
  // Segment along X axis from (0, 0) to (10, 0)
  // Point directly on segment at (5, 0)
  const double dist = monitor::point_to_segment_distance(
    5.0, 0.0,
    0.0, 0.0,
    10.0, 0.0);
  EXPECT_NEAR(dist, 0.0, 1e-6);
}

TEST(PathDeviationTest, Within50mmTolerance)
{
  // 40 mm deviation (0.040 m) along Y axis
  const double dist = monitor::point_to_segment_distance(
    5.0, 0.040,
    0.0, 0.0,
    10.0, 0.0);
  EXPECT_NEAR(dist, 0.040, 1e-6);
  EXPECT_LE(dist, 0.050);  // Within 50 mm tolerance
}

TEST(PathDeviationTest, Exceeds50mmTolerance)
{
  // 55 mm deviation (0.055 m) along Y axis
  const double dist = monitor::point_to_segment_distance(
    5.0, 0.055,
    0.0, 0.0,
    10.0, 0.0);
  EXPECT_NEAR(dist, 0.055, 1e-6);
  EXPECT_GT(dist, 0.050);  // Exceeds 50 mm tolerance -> trigger stop
}

TEST(PathDeviationTest, ExactlyAtTolerance)
{
  // Exactly 50 mm deviation (0.050 m)
  const double dist = monitor::point_to_segment_distance(
    5.0, 0.050,
    0.0, 0.0,
    10.0, 0.0);
  EXPECT_NEAR(dist, 0.050, 1e-6);
}

TEST(PathDeviationTest, DiagonalSegment)
{
  // Segment from (0, 0) to (3, 4) — length = 5.0
  // Unit normal vector: (-4/5, 3/5) = (-0.8, 0.6)
  // Midpoint on segment: (1.5, 2.0)
  // Offset by 0.060 m (60 mm, outside tolerance):
  const double px = 1.5 - 0.06 * 0.8;
  const double py = 2.0 + 0.06 * 0.6;
  const double dist = monitor::point_to_segment_distance(
    px, py,
    0.0, 0.0,
    3.0, 4.0);
  EXPECT_NEAR(dist, 0.060, 1e-5);
  EXPECT_GT(dist, 0.050);
}

TEST(PathDeviationTest, ProjectionClampingEndpoints)
{
  // Segment from (2, 0) to (5, 0)
  // Point before segment start: (1.0, 0.03) -> closest point is (2, 0)
  const double dist_before = monitor::point_to_segment_distance(
    1.0, 0.03,
    2.0, 0.0,
    5.0, 0.0);
  EXPECT_NEAR(dist_before, std::hypot(1.0, 0.03), 1e-5);

  // Point after segment end: (6.0, 0.03) -> closest point is (5, 0)
  const double dist_after = monitor::point_to_segment_distance(
    6.0, 0.03,
    2.0, 0.0,
    5.0, 0.0);
  EXPECT_NEAR(dist_after, std::hypot(1.0, 0.03), 1e-5);
}

TEST(PathDeviationTest, DegenerateSegmentZeroLength)
{
  // A and B at same point (3.0, 4.0)
  const double dist = monitor::point_to_segment_distance(
    3.0, 4.055,
    3.0, 4.0,
    3.0, 4.0);
  EXPECT_NEAR(dist, 0.055, 1e-5);
  EXPECT_GT(dist, 0.050);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
