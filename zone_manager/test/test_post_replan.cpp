#include "zone_manager/post_replan.hpp"
#include "zone_manager/conflict_decision.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <string>
#include <vector>

using zone_manager::ConflictClass;
using zone_manager::ConflictRobotInput;
using zone_manager::PendingReplanWatch;
using zone_manager::PlanSnapshot;
using zone_manager::decide_conflict_action;
using zone_manager::is_new_plan_for_candidate;
using zone_manager::is_replan_watch_timed_out;
using zone_manager::robots_to_resume;

namespace
{

ConflictRobotInput make_robot(
  const std::string & id,
  const std::vector<std::string> & reserved,
  const std::vector<std::string> & sequence,
  const std::string & priority = "",
  bool loaded = false)
{
  ConflictRobotInput in;
  in.robot_id = id;
  in.reserved_zones = reserved;
  in.zone_sequence = sequence;
  in.fleet_mode = 2;
  in.mode_known = true;
  in.operating_mode = 1;
  in.priority.robot_id = id;
  in.priority.task_priority = priority;
  in.priority.loaded = loaded;
  in.priority.eta_minutes = 1.0;
  in.priority.first_request_time = std::chrono::steady_clock::now();
  return in;
}

PlanSnapshot snap(
  const std::string & current,
  const std::vector<std::string> & sequence,
  const std::vector<std::string> & path = {})
{
  PlanSnapshot s;
  s.current_zone = current;
  s.zone_sequence = sequence;
  s.planned_path = path;
  return s;
}

}  // namespace

TEST(PostReplanTest, DetectsNewPlanWhenSequenceChanges)
{
  const auto baseline = snap("5", {"5", "6", "7", "4", "3"});
  const auto incoming = snap("5", {"5", "6", "8", "9"});  // branch-like
  EXPECT_TRUE(is_new_plan_for_candidate(baseline, incoming));
}

TEST(PostReplanTest, IgnoresIdenticalPlan)
{
  const auto baseline = snap("5", {"5", "6", "7"}, {"a", "b"});
  EXPECT_FALSE(is_new_plan_for_candidate(baseline, baseline));
}

TEST(PostReplanTest, FirstPlanAfterEmptyBaselineCounts)
{
  PlanSnapshot empty;
  const auto incoming = snap("5", {"5", "6"});
  EXPECT_TRUE(is_new_plan_for_candidate(empty, incoming));
}

TEST(PostReplanTest, EmptyIncomingIsNotNewPlan)
{
  const auto baseline = snap("5", {"5", "6"});
  PlanSnapshot empty;
  EXPECT_FALSE(is_new_plan_for_candidate(baseline, empty));
}

// ---- zones_changed_since_arm: the interlock that gates the re-analysis -----
//
// The re-analysis must not run until the replanned route has been translated
// into zones. These pin the comparison that decides that.

TEST(PostReplanTest, ZoneInterlockFiresWhenSequenceChanges)
{
  zone_manager::PlanSnapshot arm;
  arm.zone_sequence = {"20", "19", "18", "17", "16"};
  arm.next_zones = {"19", "18", "17"};

  // The replanned route turns off at zone 12 instead of continuing to 17.
  EXPECT_TRUE(
    zone_manager::zones_changed_since_arm(
      arm, {"20", "19", "18", "12", "5"}, {"19", "18", "12"}));
}

TEST(PostReplanTest, ZoneInterlockHoldsWhenSequenceIsUnchanged)
{
  // The case that caused the deadlock: something arrives, but the zones are
  // still the pre-replan ones. Firing here re-decides the conflict against the
  // old route and concludes nothing changed — forever.
  zone_manager::PlanSnapshot arm;
  arm.zone_sequence = {"20", "19", "18", "17", "16"};
  arm.next_zones = {"19", "18", "17"};

  EXPECT_FALSE(
    zone_manager::zones_changed_since_arm(
      arm, {"20", "19", "18", "17", "16"}, {"19", "18", "17"}));
}

TEST(PostReplanTest, ZoneInterlockIgnoresAnEmptyReport)
{
  // A momentary empty zone report is not a new plan. Firing on it would
  // re-decide with no zones at all, which is worse than the stale zones the
  // interlock exists to prevent.
  zone_manager::PlanSnapshot arm;
  arm.zone_sequence = {"20", "19", "18"};
  arm.next_zones = {"19", "18"};

  EXPECT_FALSE(zone_manager::zones_changed_since_arm(arm, {}, {}));
}

TEST(PostReplanTest, ZoneInterlockFiresOnFirstSequenceAfterEmptyBaseline)
{
  // No zones known when the replan was requested: the first real sequence to
  // arrive IS the change.
  zone_manager::PlanSnapshot arm;
  EXPECT_TRUE(
    zone_manager::zones_changed_since_arm(arm, {"20", "19", "18"}, {"19"}));
  EXPECT_FALSE(zone_manager::zones_changed_since_arm(arm, {}, {}));
}

TEST(PostReplanTest, ZoneInterlockFiresWhenOnlyNextZonesChange)
{
  // zone_monitor can report a shorter lookahead before the full sequence
  // settles; a changed next_zone window still means the route moved.
  zone_manager::PlanSnapshot arm;
  arm.zone_sequence = {"20", "19", "18"};
  arm.next_zones = {"19", "18", "17"};

  EXPECT_TRUE(
    zone_manager::zones_changed_since_arm(arm, {"20", "19", "18"}, {"19", "18", "12"}));
}

TEST(PostReplanTest, ResumeSetExcludesPauseTargets)
{
  const auto resume = robots_to_resume(
    {"amr1", "amr2"},
    {"amr2"});
  ASSERT_EQ(resume.size(), 1u);
  EXPECT_EQ(resume[0], "amr1");
}

TEST(PostReplanTest, OppositeToBranchResumesThroughRobot)
{
  // After replan: shared prefix then diverge (branch vs through).
  auto through = make_robot(
    "amr1", {"5", "6", "7", "8"},
    {"5", "6", "7", "8", "9"},
    "high");
  auto branch = make_robot(
    "amr2", {"5", "6", "10"},
    {"5", "6", "10", "11"},
    "low");

  const auto d = decide_conflict_action("5", {through, branch});
  EXPECT_NE(d.conflict_class, ConflictClass::StraightOpposite);
  EXPECT_NE(d.conflict_class, ConflictClass::JunctionOpposite);

  const auto resume = robots_to_resume({"amr1", "amr2"}, d.pause_robot_ids);
  // Exactly one of the pair should resume under C1/C5-style single pause.
  EXPECT_FALSE(resume.empty());
  for (const auto & id : resume) {
    EXPECT_TRUE(
      std::find(d.pause_robot_ids.begin(), d.pause_robot_ids.end(), id) ==
      d.pause_robot_ids.end());
  }
}

TEST(PostReplanTest, StillOppositeResumeSetEmpty)
{
  auto amr1 = make_robot(
    "amr1", {"3", "4", "5", "6"},
    {"3", "4", "5", "6", "7", "8"},
    "high");
  auto amr2 = make_robot(
    "amr2", {"9", "8", "7", "6"},
    {"9", "8", "7", "6", "5", "4"},
    "low");

  const auto d = decide_conflict_action("6", {amr1, amr2});
  EXPECT_TRUE(
    d.conflict_class == ConflictClass::StraightOpposite ||
    d.conflict_class == ConflictClass::JunctionOpposite);

  // Opposite pauses both → nothing to resume via selective stopPause.
  const auto resume = robots_to_resume({"amr1", "amr2"}, d.pause_robot_ids);
  EXPECT_TRUE(resume.empty());
}

TEST(PostReplanTest, WatchTimeoutClearsAfterDeadline)
{
  PendingReplanWatch watch;
  watch.candidate_robot_id = "amr2";
  watch.zone_id = "5";
  watch.armed_at = std::chrono::steady_clock::now();

  EXPECT_FALSE(
    is_replan_watch_timed_out(watch, watch.armed_at, 3.0));

  const auto later = watch.armed_at + std::chrono::milliseconds(3100);
  EXPECT_TRUE(is_replan_watch_timed_out(watch, later, 3.0));

  // timeout_sec <= 0 disables
  EXPECT_FALSE(is_replan_watch_timed_out(watch, later, 0.0));
}
