// =============================================================================
// File: monitor/include/monitor/safety_field_tracker.hpp
// =============================================================================
// Which robots are in conflict, and which just entered or left it.
//
// Split out of safety_field_node.cpp for the same reason zone_tracker is split
// out of zone_monitor_node: the node is ROS wiring, and the part that can be
// WRONG is the bookkeeping. Here it is testable without standing up a node.
//
// WHAT IT GUARANTEES
//   Every robot reported as entered is reported as left exactly once, and is
//   never reported as entered twice without a leave in between. That is not
//   tidiness — zone_manager increments a pause refcount once per robot named in
//   a DETECTED message and decrements once per robot named in a RESOLVED,
//   without checking whether it already holds that robot. A repeated enter
//   leaves the count at 1 after the release and the robot paused for good.
//
// NO HYSTERESIS OF ANY KIND
//   There is one rectangle and no dwell. A conflict is raised on the first tick
//   the rectangles touch and dropped on the first tick they do not, so the
//   published state is always exactly what the geometry said this instant.
//
//   The consequence to know about: because the field rotates, two robots that
//   are standing still can enter and leave an overlap purely by turning, and
//   each of those transitions is now a real startPause / stopPause on the
//   floor. If that shows up as lurching during a turn, the fix is a dwell —
//   hold a conflict for a short period of continuous clearance before releasing
//   it — which damps the flicker in time without ever delaying a pause. It was
//   deliberately left out, not overlooked.
// =============================================================================

#ifndef MONITOR__SAFETY_FIELD_TRACKER_HPP_
#define MONITOR__SAFETY_FIELD_TRACKER_HPP_

#include <algorithm>
#include <iterator>
#include <map>
#include <set>
#include <string>
#include <vector>

#include "monitor/safety_field_geometry.hpp"

namespace monitor
{

/// Where a robot is and which way it is facing, map frame.
struct SafetyFieldPose
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

/// Two robots whose fields overlap, named in a stable order.
struct SafetyFieldPairRef
{
  std::string robot_a;
  std::string robot_b;
};

/// What changed on one evaluation tick.
struct SafetyFieldUpdate
{
  /// Robots that just entered conflict. Becomes a DETECTED message.
  std::vector<std::string> entered;
  /// Robots that just left it. Becomes a RESOLVED message.
  std::vector<std::string> left;
  /// Every pair currently in conflict, for the DETECTED payload and markers.
  std::vector<SafetyFieldPairRef> active;
  /// Pairs whose conflict ended this tick, for the RESOLVED payload.
  std::vector<SafetyFieldPairRef> ended;
  /// Every robot currently in conflict — what the markers colour red.
  std::set<std::string> conflicting;
};

class SafetyFieldTracker
{
public:
  /// half_length is measured along the robot's heading, half_width across it;
  /// both already include the safety margin.
  SafetyFieldTracker(double half_length, double half_width)
  : half_length_(half_length),
    half_width_(half_width)
  {
  }

  /// Fold one set of measured poses in and report the transitions.
  ///
  /// Takes no clock: with no dwell, the answer depends only on where the robots
  /// are right now.
  SafetyFieldUpdate update(const std::map<std::string, SafetyFieldPose> & poses)
  {
    SafetyFieldUpdate out;

    // Recomputed from scratch every tick. A robot that stopped reporting is
    // simply absent from poses, so its pairs fall out here and it is released
    // with no special case: with no position there is no evidence of an
    // overlap, and a robot left paused by a field nobody can see or clear is
    // worse than one that resumes.
    std::map<std::string, SafetyFieldPairRef> current;

    std::vector<std::string> names;
    names.reserve(poses.size());
    for (const auto & entry : poses) {
      names.push_back(entry.first);
    }

    for (std::size_t i = 0; i < names.size(); ++i) {
      for (std::size_t j = i + 1; j < names.size(); ++j) {
        if (boxes_overlap(
            box_for(poses.at(names[i])), box_for(poses.at(names[j]))))
        {
          current.emplace(
            pair_key(names[i], names[j]),
            SafetyFieldPairRef{names[i], names[j]});
        }
      }
    }

    for (const auto & [key, ref] : previous_) {
      if (current.count(key) == 0) {
        out.ended.push_back(ref);
      }
    }

    // Derived from the pairs rather than tracked beside them, so the two cannot
    // disagree about who is held.
    for (const auto & [key, ref] : current) {
      (void)key;
      out.conflicting.insert(ref.robot_a);
      out.conflicting.insert(ref.robot_b);
      out.active.push_back(ref);
    }

    std::set_difference(
      out.conflicting.begin(), out.conflicting.end(),
      held_.begin(), held_.end(), std::back_inserter(out.entered));
    std::set_difference(
      held_.begin(), held_.end(),
      out.conflicting.begin(), out.conflicting.end(),
      std::back_inserter(out.left));

    previous_ = std::move(current);
    held_ = out.conflicting;
    return out;
  }

  /// Robots currently believed to be held downstream. Exposed for logging and
  /// tests; the node never needs to consult it.
  const std::set<std::string> & held() const {return held_;}

private:
  OrientedBox box_for(const SafetyFieldPose & p) const
  {
    OrientedBox box;
    box.cx = p.x;
    box.cy = p.y;
    box.yaw = p.yaw;
    box.half_length = half_length_;
    box.half_width = half_width_;
    return box;
  }

  static std::string pair_key(const std::string & a, const std::string & b)
  {
    return (a < b) ? (a + "|" + b) : (b + "|" + a);
  }

  double half_length_;
  double half_width_;

  /// Last tick's overlapping pairs, kept only so a pair that ends can be named
  /// in the RESOLVED message it causes.
  std::map<std::string, SafetyFieldPairRef> previous_;

  /// The robots named in an `entered` and not yet in a `left` — i.e. what
  /// zone_manager currently believes it is holding on our behalf.
  std::set<std::string> held_;
};

}  // namespace monitor

#endif  // MONITOR__SAFETY_FIELD_TRACKER_HPP_
