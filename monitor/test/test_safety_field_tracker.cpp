// =============================================================================
// Tests for the safety field conflict bookkeeping.
//
// The property under test is the one zone_manager depends on and cannot check
// for itself: every robot reported as entered is reported as left EXACTLY once.
// Its note_conflict_detected() bumps a pause refcount without asking whether it
// already holds that robot, so a duplicated enter leaves the count at 1 after
// the release and the robot paused for good. A missing leave does the same.
//
// There is no hysteresis and no dwell: a conflict is raised on the first tick
// the rectangles touch and dropped on the first tick they do not. What these
// tests pin is that "immediately" still means exactly once in each direction.
//
// Robots are laid out along +x at 2.0 m spacing. With the shipped field
// (1.353 x 1.403 m body, 0.40 m margin, so 2.153 m nose-to-tail) neighbours at
// 2.0 m overlap and robots two apart at 4.0 m do not, which makes an
// amr1-amr2-amr3 chain out of three numbers.
// =============================================================================

#include <cmath>
#include <map>
#include <set>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "monitor/safety_field_tracker.hpp"

namespace {

constexpr double kHalfLength = 0.5 * 1.353 + 0.40;  // 1.0765
constexpr double kHalfWidth = 0.5 * 1.403 + 0.40;   // 1.1015

using Poses = std::map<std::string, monitor::SafetyFieldPose>;

monitor::SafetyFieldPose at(double x)
{
  monitor::SafetyFieldPose p;
  p.x = x;
  p.y = 0.0;
  p.yaw = 0.0;
  return p;
}

monitor::SafetyFieldTracker make_tracker()
{
  return monitor::SafetyFieldTracker(kHalfLength, kHalfWidth);
}

}  // namespace

TEST(SafetyFieldTracker, RobotsFarApartRaiseNothing)
{
  auto tracker = make_tracker();
  const auto update = tracker.update({{"amr1", at(0.0)}, {"amr2", at(9.0)}});

  EXPECT_TRUE(update.entered.empty());
  EXPECT_TRUE(update.left.empty());
  EXPECT_TRUE(update.conflicting.empty());
  EXPECT_TRUE(update.active.empty());
}

TEST(SafetyFieldTracker, EnteringIsReportedExactlyOnce)
{
  auto tracker = make_tracker();
  const Poses close{{"amr1", at(0.0)}, {"amr2", at(2.0)}};

  const auto first = tracker.update(close);
  EXPECT_EQ(first.entered, std::vector<std::string>({"amr1", "amr2"}));
  EXPECT_EQ(first.active.size(), 1u);
  EXPECT_TRUE(first.left.empty());

  // Still overlapping on every subsequent tick. Reporting either robot again
  // would double-bump zone_manager's refcount and strand it paused, so the
  // stream must go quiet until something actually changes.
  for (int tick = 1; tick < 40; ++tick) {
    const auto again = tracker.update(close);
    EXPECT_TRUE(again.entered.empty()) << "tick " << tick;
    EXPECT_TRUE(again.left.empty()) << "tick " << tick;
    EXPECT_EQ(again.conflicting.size(), 2u) << "tick " << tick;
  }
}

TEST(SafetyFieldTracker, ClearingIsReportedOnTheVeryNextTick)
{
  auto tracker = make_tracker();
  tracker.update({{"amr1", at(0.0)}, {"amr2", at(2.0)}});

  // No dwell: the tick after the rectangles part is the tick that releases.
  const auto released = tracker.update({{"amr1", at(0.0)}, {"amr2", at(9.0)}});

  EXPECT_EQ(released.left, std::vector<std::string>({"amr1", "amr2"}));
  EXPECT_EQ(released.ended.size(), 1u);
  EXPECT_TRUE(released.conflicting.empty());
  EXPECT_TRUE(tracker.held().empty());
}

TEST(SafetyFieldTracker, ClearingIsReportedExactlyOnce)
{
  auto tracker = make_tracker();
  const Poses apart{{"amr1", at(0.0)}, {"amr2", at(9.0)}};

  tracker.update({{"amr1", at(0.0)}, {"amr2", at(2.0)}});
  tracker.update(apart);

  // A second RESOLVED for a robot zone_manager is no longer holding would
  // decrement a refcount that is already at zero.
  for (int tick = 0; tick < 20; ++tick) {
    const auto again = tracker.update(apart);
    EXPECT_TRUE(again.left.empty()) << "tick " << tick;
    EXPECT_TRUE(again.entered.empty()) << "tick " << tick;
  }
}

TEST(SafetyFieldTracker, TouchingRepeatedlyAlternatesCleanly)
{
  // Two robots crossing the boundary back and forth — the shape of a pair
  // turning on the spot. Every enter must still be matched by exactly one
  // leave, however fast they alternate.
  auto tracker = make_tracker();
  const Poses close{{"amr1", at(0.0)}, {"amr2", at(2.0)}};
  const Poses apart{{"amr1", at(0.0)}, {"amr2", at(9.0)}};

  for (int cycle = 0; cycle < 10; ++cycle) {
    const auto on = tracker.update(close);
    EXPECT_EQ(on.entered, std::vector<std::string>({"amr1", "amr2"}))
      << "cycle " << cycle;
    EXPECT_TRUE(on.left.empty()) << "cycle " << cycle;

    const auto off = tracker.update(apart);
    EXPECT_EQ(off.left, std::vector<std::string>({"amr1", "amr2"}))
      << "cycle " << cycle;
    EXPECT_TRUE(off.entered.empty()) << "cycle " << cycle;
  }
}

TEST(SafetyFieldTracker, AThirdRobotJoiningReportsOnlyItself)
{
  auto tracker = make_tracker();
  tracker.update({{"amr1", at(0.0)}, {"amr2", at(2.0)}});

  // amr3 overlaps amr2 while amr1 and amr2 are still overlapping. Naming amr1
  // or amr2 again here is exactly the double-bump that strands a robot.
  const auto joined = tracker.update(
    {{"amr1", at(0.0)}, {"amr2", at(2.0)}, {"amr3", at(4.0)}});

  EXPECT_EQ(joined.entered, std::vector<std::string>({"amr3"}));
  EXPECT_TRUE(joined.left.empty());
  EXPECT_EQ(joined.conflicting.size(), 3u);
  EXPECT_EQ(joined.active.size(), 2u);  // amr1|amr2 and amr2|amr3
}

TEST(SafetyFieldTracker, OnePairClearingReleasesOnlyTheRobotThatLeft)
{
  auto tracker = make_tracker();
  tracker.update({{"amr1", at(0.0)}, {"amr2", at(2.0)}, {"amr3", at(4.0)}});

  // amr1 drives away; amr2 and amr3 are still overlapping each other. The
  // previous none->any / any->none edge trigger published nothing at all here,
  // which left amr1 held until the whole fleet cleared — and then named the
  // wrong robots when it finally did.
  const auto partial = tracker.update(
    {{"amr1", at(-20.0)}, {"amr2", at(2.0)}, {"amr3", at(4.0)}});

  EXPECT_EQ(partial.left, std::vector<std::string>({"amr1"}));
  EXPECT_TRUE(partial.entered.empty());
  EXPECT_EQ(partial.conflicting, std::set<std::string>({"amr2", "amr3"}));
  EXPECT_EQ(partial.ended.size(), 1u);
}

TEST(SafetyFieldTracker, ARobotThatStopsReportingIsReleased)
{
  auto tracker = make_tracker();
  tracker.update({{"amr1", at(0.0)}, {"amr2", at(2.0)}});

  // safety_field_node has dropped amr1's pose as stale. There is no longer any
  // evidence of an overlap, and a robot paused by a field nobody can see or
  // clear is worse than one that resumes.
  const auto dropped = tracker.update({{"amr2", at(2.0)}});

  EXPECT_EQ(dropped.left, std::vector<std::string>({"amr1", "amr2"}));
  EXPECT_TRUE(dropped.conflicting.empty());
  EXPECT_EQ(dropped.ended.size(), 1u);
}

TEST(SafetyFieldTracker, EveryEnterIsBalancedByExactlyOneLeave)
{
  // The invariant zone_manager's refcount depends on, swept over a long run of
  // robots converging, touching, parting and vanishing.
  auto tracker = make_tracker();
  std::map<std::string, int> balance;

  for (int step = 0; step < 400; ++step) {
    const double phase = 0.05 * step;
    Poses poses;
    poses["amr1"] = at(0.0);
    poses["amr2"] = at(1.6 + 1.2 * std::sin(phase));
    if (step % 37 != 0) {  // amr3 blinks in and out of /fleet_states
      monitor::SafetyFieldPose p = at(3.2 + 1.2 * std::cos(phase * 0.7));
      p.yaw = phase;       // and turns while it does
      poses["amr3"] = p;
    }

    const auto update = tracker.update(poses);

    for (const auto & id : update.entered) {
      balance[id] += 1;
      EXPECT_EQ(balance[id], 1) << id << " entered twice without leaving, step "
                               << step;
    }
    for (const auto & id : update.left) {
      balance[id] -= 1;
      EXPECT_EQ(balance[id], 0) << id << " left without being held, step "
                               << step;
    }
    for (const auto & id : update.conflicting) {
      EXPECT_EQ(balance[id], 1) << id << " conflicting but not held, step "
                               << step;
    }
  }

  for (const auto & [id, count] : balance) {
    EXPECT_GE(count, 0) << id;
    EXPECT_LE(count, 1) << id;
  }
}
