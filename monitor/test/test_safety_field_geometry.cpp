// =============================================================================
// Tests for the oriented-rectangle safety field overlap test.
//
// The circle this replaced needed no tests: one distance comparison, right or
// visibly wrong. A separating-axis test on two rotating rectangles is the other
// kind of code — a swapped sign or a transposed half-extent passes inspection,
// gives correct answers at 0 degrees, and fails somewhere in the middle. These
// cases pin the thresholds that the field is actually specified by.
//
// All distances below are derived from the shipped defaults:
//   body   1.353 m (nose to tail) x 1.403 m (side to side)
//   margin 0.40 m per side
//   field  2.153 x 2.203 m, so half-extents 1.0765 x 1.1015
// =============================================================================

#include <cmath>

#include <gtest/gtest.h>

#include "monitor/safety_field_geometry.hpp"

namespace {

constexpr double kBodyLength = 1.353;
constexpr double kBodyWidth = 1.403;
constexpr double kMargin = 0.40;

constexpr double kHalfLength = 0.5 * kBodyLength + kMargin;   // 1.0765
constexpr double kHalfWidth = 0.5 * kBodyWidth + kMargin;     // 1.1015

monitor::OrientedBox field(double x, double y, double yaw)
{
  monitor::OrientedBox b;
  b.cx = x;
  b.cy = y;
  b.yaw = yaw;
  b.half_length = kHalfLength;
  b.half_width = kHalfWidth;
  return b;
}

const double kDeg45 = M_PI / 4.0;

}  // namespace

// --- The two axis-aligned thresholds the field is specified by ---------------

TEST(SafetyFieldGeometry, NoseToTailTriggersAt2153mm)
{
  const double touch = 2.0 * kHalfLength;  // 2.153 m
  EXPECT_TRUE(monitor::boxes_overlap(field(0, 0, 0), field(touch - 0.05, 0, 0)));
  EXPECT_FALSE(monitor::boxes_overlap(field(0, 0, 0), field(touch + 0.05, 0, 0)));
}

TEST(SafetyFieldGeometry, SideBySideTriggersAt2203mm)
{
  const double touch = 2.0 * kHalfWidth;  // 2.203 m
  EXPECT_TRUE(monitor::boxes_overlap(field(0, 0, 0), field(0, touch - 0.05, 0)));
  EXPECT_FALSE(monitor::boxes_overlap(field(0, 0, 0), field(0, touch + 0.05, 0)));
}

TEST(SafetyFieldGeometry, TouchingExactlyCountsAsClear)
{
  const double touch = 2.0 * kHalfLength;
  EXPECT_FALSE(monitor::boxes_overlap(field(0, 0, 0), field(touch, 0, 0)));
  EXPECT_TRUE(monitor::boxes_overlap(field(0, 0, 0), field(touch - 1e-6, 0, 0)));
}

// --- The diagonal reach, which is the reason for the shape -------------------

TEST(SafetyFieldGeometry, CornerToCornerReachesFurtherThanAnyFace)
{
  // Two circumradii: the largest centre-to-centre distance at which two of
  // these rectangles can touch at all, in any relative heading. 3.080 m against
  // 2.203 m for the widest face — this gap is what the rectangle buys over the
  // 2.2 m circle it replaced.
  const double max_reach = 2.0 * std::hypot(kHalfLength, kHalfWidth);
  EXPECT_NEAR(max_reach, 3.0803, 1e-3);
  EXPECT_GT(max_reach, 2.0 * kHalfWidth);

  // Offset along the corner direction, where both axis thresholds are met at
  // the same instant.
  const double ux = kHalfLength / std::hypot(kHalfLength, kHalfWidth);
  const double uy = kHalfWidth / std::hypot(kHalfLength, kHalfWidth);

  const double inside = max_reach - 0.08;
  EXPECT_TRUE(
    monitor::boxes_overlap(
      field(0, 0, 0), field(inside * ux, inside * uy, 0)));

  const double outside = max_reach + 0.08;
  EXPECT_FALSE(
    monitor::boxes_overlap(
      field(0, 0, 0), field(outside * ux, outside * uy, 0)));
}

// --- Rotation, which a circle could not see at all ---------------------------

TEST(SafetyFieldGeometry, RotatingInPlaceCanCreateAConflict)
{
  // Two robots 2.18 m apart nose-to-tail — just beyond the 2.153 m face
  // threshold, so they are clear while both point along +x.
  const double gap = 2.18;
  EXPECT_FALSE(monitor::boxes_overlap(field(0, 0, 0), field(gap, 0, 0)));

  // Neither has moved a millimetre; both have turned 45 degrees on the spot.
  // The corners now sweep into each other and the fields overlap. This is the
  // case the whole change exists for, and the one no rotation-invariant circle
  // can ever report.
  EXPECT_TRUE(
    monitor::boxes_overlap(field(0, 0, kDeg45), field(gap, 0, kDeg45)));
}

TEST(SafetyFieldGeometry, OnlyOneRobotNeedsToTurn)
{
  const double gap = 2.18;
  EXPECT_TRUE(monitor::boxes_overlap(field(0, 0, 0), field(gap, 0, kDeg45)));
}

TEST(SafetyFieldGeometry, RotatingNinetyDegreesSwapsTheThresholds)
{
  // Turned a quarter turn, the robot presents its width along +x, so the
  // nose-to-tail threshold no longer applies in that direction.
  const double between = 2.18;  // past 2.153 (length) but inside 2.203 (width)
  EXPECT_FALSE(monitor::boxes_overlap(field(0, 0, 0), field(between, 0, 0)));
  EXPECT_TRUE(
    monitor::boxes_overlap(
      field(0, 0, M_PI / 2.0), field(between, 0, M_PI / 2.0)));
}

// --- Properties that must hold whatever the arithmetic does ------------------

TEST(SafetyFieldGeometry, OverlapIsSymmetric)
{
  const monitor::OrientedBox a = field(1.0, -2.0, 0.6);
  const monitor::OrientedBox b = field(2.4, -0.9, -1.2);
  EXPECT_EQ(monitor::boxes_overlap(a, b), monitor::boxes_overlap(b, a));
}

TEST(SafetyFieldGeometry, YawIsPeriodic)
{
  const monitor::OrientedBox a = field(0, 0, 0.3);
  EXPECT_EQ(
    monitor::boxes_overlap(a, field(2.0, 0.4, 1.1)),
    monitor::boxes_overlap(a, field(2.0, 0.4, 1.1 + 2.0 * M_PI)));
}

TEST(SafetyFieldGeometry, CoincidentRobotsAlwaysOverlap)
{
  EXPECT_TRUE(monitor::boxes_overlap(field(3, 4, 0.0), field(3, 4, 1.9)));
}

TEST(SafetyFieldGeometry, DistantRobotsNeverOverlap)
{
  for (double yaw = 0.0; yaw < 2.0 * M_PI; yaw += 0.2) {
    EXPECT_FALSE(monitor::boxes_overlap(field(0, 0, yaw), field(4.0, 4.0, -yaw)))
      << "yaw=" << yaw;
  }
}
