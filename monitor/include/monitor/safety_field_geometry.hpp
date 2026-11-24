// =============================================================================
// File: monitor/include/monitor/safety_field_geometry.hpp
// =============================================================================
// Overlap test for two rectangles that rotate with their robots.
//
// Lives in a header of its own, separate from safety_field_node.cpp, for one
// reason: it is the only part of the safety field that can be got WRONG
// silently. A circle overlap test is one line and is either right or obviously
// wrong. An oriented-rectangle test has four axes and a pair of sign
// conventions, and a mistake in it looks perfectly correct at 0 degrees and
// fails at 47. Keeping it here lets test_safety_field_geometry.cpp exercise it
// directly instead of having to stand up a ROS node.
//
// WHY THE SEPARATING AXIS THEOREM
//   Two convex shapes are disjoint if and only if some line exists onto which
//   their projections do not overlap. For two rectangles it is enough to test
//   four candidate directions — the two edge normals of each rectangle — since
//   any separating line can be rotated onto one of them without losing the
//   separation. So the test is exact and finite: four projections, no
//   iteration, no tolerance to tune.
//
// CONVENTION
//   yaw is the robot's heading in the map frame, radians, counter-clockwise,
//   0 pointing along +x — the same convention rmf_fleet_msgs/Location uses, so
//   Location::yaw can be copied straight in.
//
//   half_length is measured ALONG the heading (nose to tail) and half_width
//   ACROSS it (side to side). Getting these two the wrong way round does not
//   fail loudly, it just makes the field 25 mm wrong on a 1353 x 1403 mm
//   robot, so they are named rather than passed as a bare pair.
// =============================================================================

#ifndef MONITOR__SAFETY_FIELD_GEOMETRY_HPP_
#define MONITOR__SAFETY_FIELD_GEOMETRY_HPP_

#include <cmath>

namespace monitor
{

/// A rectangle centred on a robot and carried by it, heading included.
struct OrientedBox
{
  double cx{0.0};
  double cy{0.0};
  /// Heading in radians, map frame, CCW, 0 = +x.
  double yaw{0.0};
  /// Half-extent along the heading (nose to tail).
  double half_length{0.0};
  /// Half-extent across the heading (side to side).
  double half_width{0.0};
};

/// How far the box reaches from its own centre along the unit axis (nx, ny).
///
/// The box's own axes are u = (cos yaw, sin yaw) along its length and
/// v = (-sin yaw, cos yaw) across it. Its shadow on any direction is the sum of
/// each half-extent times how much that axis leans onto the direction, which is
/// why both terms are absolute values: a box reaches the same distance forwards
/// as backwards.
inline double projected_extent(const OrientedBox & b, double nx, double ny)
{
  const double c = std::cos(b.yaw);
  const double s = std::sin(b.yaw);
  return b.half_length * std::fabs(c * nx + s * ny) +
         b.half_width * std::fabs(-s * nx + c * ny);
}

/// True when the two rectangles share any area.
///
/// Touching exactly edge-to-edge counts as CLEAR, not as an overlap. The
/// comparison is `>=` on the gap for the same reason the circle version used a
/// strict `<`: a conflict should need real penetration, not a floating-point
/// coincidence at the boundary.
inline bool boxes_overlap(const OrientedBox & a, const OrientedBox & b)
{
  const double dx = b.cx - a.cx;
  const double dy = b.cy - a.cy;

  const double ca = std::cos(a.yaw);
  const double sa = std::sin(a.yaw);
  const double cb = std::cos(b.yaw);
  const double sb = std::sin(b.yaw);

  // a's length axis, a's width axis, b's length axis, b's width axis.
  const double axes[4][2] = {
    {ca, sa},
    {-sa, ca},
    {cb, sb},
    {-sb, cb}
  };

  for (const auto & n : axes) {
    const double centre_gap = std::fabs(dx * n[0] + dy * n[1]);
    const double reach =
      projected_extent(a, n[0], n[1]) + projected_extent(b, n[0], n[1]);
    if (centre_gap >= reach) {
      // A gap on one axis is proof of separation; the other three cannot
      // overturn it. This is what keeps the test cheap in the common case of
      // robots nowhere near each other.
      return false;
    }
  }
  return true;
}

}  // namespace monitor

#endif  // MONITOR__SAFETY_FIELD_GEOMETRY_HPP_
