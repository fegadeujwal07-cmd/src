#include "zone_manager/conflict_decision.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <string>
#include <vector>

using zone_manager::ConflictClass;
using zone_manager::ConflictDecision;
using zone_manager::ConflictRobotInput;
using zone_manager::TravelDirection;
using zone_manager::classify_travel_direction;
using zone_manager::decide_conflict_action;
using zone_manager::is_junction_conflict;
using zone_manager::remaining_blocks_other;

namespace
{

ConflictRobotInput make_robot(
  const std::string & id,
  const std::vector<std::string> & reserved,
  const std::vector<std::string> & sequence,
  const std::string & priority = "",
  bool loaded = false,
  uint32_t mode = 2,
  bool mode_known = true,
  int16_t operating_mode = 1)
{
  ConflictRobotInput in;
  in.robot_id = id;
  in.reserved_zones = reserved;
  in.zone_sequence = sequence;
  in.fleet_mode = mode;
  in.mode_known = mode_known;
  in.operating_mode = operating_mode;
  in.priority.robot_id = id;
  in.priority.task_priority = priority;
  in.priority.loaded = loaded;
  in.priority.eta_minutes = 1.0;
  in.priority.first_request_time = std::chrono::steady_clock::now();
  return in;
}

}  // namespace

TEST(ConflictDecisionTest, Condition1_StraightSameDirection_PauseRear)
{
  // AMR1 behind on zone 4; AMR2 already at 4.
  auto amr1 = make_robot(
    "amr1", {"1", "2", "3", "4"},
    {"1", "2", "3", "4", "5", "6", "7", "8"});
  auto amr2 = make_robot(
    "amr2", {"4", "5", "6", "7"},
    {"4", "5", "6", "7", "8", "9", "10"},
    "high", true);  // priority must not matter for C1 behind

  const auto d = decide_conflict_action("4", {amr1, amr2});
  EXPECT_EQ(d.direction, TravelDirection::Same);
  EXPECT_EQ(d.conflict_class, ConflictClass::StraightSameDirection);
  ASSERT_EQ(d.pause_robot_ids.size(), 1u);
  EXPECT_EQ(d.pause_robot_ids[0], "amr1");
  EXPECT_TRUE(d.replan_candidate_id.empty());
}

TEST(ConflictDecisionTest, Condition2_StraightOpposite_PauseBothAndReplanLowest)
{
  auto amr1 = make_robot(
    "amr1", {"3", "4", "5", "6"},
    {"3", "4", "5", "6", "7", "8", "9", "10"},
    "high");
  auto amr2 = make_robot(
    "amr2", {"9", "8", "7", "6"},
    {"9", "8", "7", "6", "5", "4", "3", "2", "1"},
    "low");

  EXPECT_EQ(
    classify_travel_direction(amr1.zone_sequence, amr2.zone_sequence, "6"),
    TravelDirection::Opposite);

  const auto d = decide_conflict_action("6", {amr1, amr2});
  EXPECT_EQ(d.conflict_class, ConflictClass::StraightOpposite);
  ASSERT_EQ(d.pause_robot_ids.size(), 2u);
  EXPECT_EQ(d.replan_candidate_id, "amr2");  // low priority
  EXPECT_EQ(d.replan_reason, "opposite_both_healthy_lowest_priority");
}

TEST(ConflictDecisionTest, Condition3a_EasinessBeatsPriority)
{
  // AMR2 has high priority but remaining [6,7,8] blocks AMR1 [6,7,8,9].
  auto amr1 = make_robot(
    "amr1", {"2", "3", "4", "5"},
    {"2", "3", "4", "5", "6", "7", "8", "9"},
    "low");
  auto amr2 = make_robot(
    "amr2", {"13", "12", "11", "5"},
    {"13", "12", "11", "5", "6", "7", "8"},
    "high", true);

  EXPECT_TRUE(is_junction_conflict(amr1.zone_sequence, amr2.zone_sequence, "5"));
  EXPECT_EQ(
    classify_travel_direction(amr1.zone_sequence, amr2.zone_sequence, "5"),
    TravelDirection::Same);

  EXPECT_TRUE(
    remaining_blocks_other({"6", "7", "8"}, {"6", "7", "8", "9"}));

  const auto d = decide_conflict_action("5", {amr1, amr2});
  EXPECT_EQ(d.conflict_class, ConflictClass::JunctionSameDirection);
  ASSERT_EQ(d.pause_robot_ids.size(), 1u);
  EXPECT_EQ(d.pause_robot_ids[0], "amr2");  // easiness, not priority
}

TEST(ConflictDecisionTest, Condition3b_SafetyBeforeEasiness)
{
  // Overlap at reserved idx 1 of amr1 → safety: let amr1 move, pause amr2.
  auto amr1 = make_robot(
    "amr1", {"4", "5", "6", "7"},
    {"2", "3", "4", "5", "6", "7", "8", "9"});
  auto amr2 = make_robot(
    "amr2", {"13", "12", "11", "5"},
    {"13", "12", "11", "5", "6", "7", "8"},
    "high", true);

  const auto d = decide_conflict_action("5", {amr1, amr2});
  ASSERT_EQ(d.pause_robot_ids.size(), 1u);
  EXPECT_EQ(d.pause_robot_ids[0], "amr2");
}

TEST(ConflictDecisionTest, Condition4_JunctionOpposite_PauseBoth)
{
  // Two distinct spur arms meet at 5 and head into each other's approach.
  // Even if topology classifies as straight-opposite vs junction-opposite,
  // both must be paused and a replan candidate published.
  auto amr1 = make_robot(
    "amr1", {"30", "31", "5", "51"},
    {"30", "31", "5", "51", "50"});
  auto amr2 = make_robot(
    "amr2", {"50", "51", "5", "31"},
    {"50", "51", "5", "31", "30"});

  EXPECT_EQ(
    classify_travel_direction(amr1.zone_sequence, amr2.zone_sequence, "5"),
    TravelDirection::Opposite);

  const auto d = decide_conflict_action("5", {amr1, amr2});
  EXPECT_TRUE(
    d.conflict_class == ConflictClass::JunctionOpposite ||
    d.conflict_class == ConflictClass::StraightOpposite);
  ASSERT_EQ(d.pause_robot_ids.size(), 2u);
  EXPECT_FALSE(d.replan_candidate_id.empty());
}

TEST(ConflictDecisionTest, Condition5_ThroughWinsOverBranch)
{
  // Horizontal through 4→5→6; branch 13→12→11→5 then toward 4 (opposing).
  auto amr1 = make_robot(
    "amr1", {"4", "5", "6", "7"},
    {"1", "2", "3", "4", "5", "6", "7", "8"});
  auto amr2 = make_robot(
    "amr2", {"13", "12", "11", "5"},
    {"13", "12", "11", "5", "4", "3", "2"});

  const auto d = decide_conflict_action("5", {amr1, amr2});
  EXPECT_EQ(d.conflict_class, ConflictClass::BranchVsThrough);
  ASSERT_EQ(d.pause_robot_ids.size(), 1u);
  EXPECT_EQ(d.pause_robot_ids[0], "amr2");
  EXPECT_TRUE(d.replan_candidate_id.empty());
}

TEST(ConflictDecisionTest, Condition5_LiveAmr1MainlineVsAmr3Branch)
{
  // Live: mutual head-on at 15, but amr3 arrived via exclusive spur 23/21.
  // Expect C5: pause amr3 only; amr1 (main line) continues.
  auto amr1 = make_robot(
    "amr1", {"13", "14", "15", "16"},
    {"13", "14", "15", "16", "17", "18", "19"});
  auto amr3 = make_robot(
    "amr3", {"23", "21", "16", "15"},
    {"23", "21", "16", "15", "14", "13", "12"});

  EXPECT_EQ(
    classify_travel_direction(amr1.zone_sequence, amr3.zone_sequence, "15"),
    TravelDirection::Opposite);
  EXPECT_TRUE(is_junction_conflict(amr1.zone_sequence, amr3.zone_sequence, "15"));

  const auto d = decide_conflict_action("15", {amr1, amr3});
  EXPECT_EQ(d.conflict_class, ConflictClass::BranchVsThrough);
  ASSERT_EQ(d.pause_robot_ids.size(), 1u);
  EXPECT_EQ(d.pause_robot_ids[0], "amr3");
  EXPECT_TRUE(d.replan_candidate_id.empty());
}

TEST(ConflictDecisionTest, OppositeUnhealthy_PreferHealthyAsReplanCandidate)
{
  // amr1 healthy (operating_mode=1), amr2 unhealthy (operating_mode=0)
  auto amr1 = make_robot(
    "amr1", {"3", "4", "5", "6"},
    {"3", "4", "5", "6", "7", "8", "9", "10"},
    "low", false, 2, true, 1);
  auto amr2 = make_robot(
    "amr2", {"9", "8", "7", "6"},
    {"9", "8", "7", "6", "5", "4", "3", "2", "1"},
    "high", false, 2, true, 0);

  const auto d = decide_conflict_action("6", {amr1, amr2});
  EXPECT_EQ(d.conflict_class, ConflictClass::StraightOpposite);
  EXPECT_EQ(d.replan_candidate_id, "amr1");
  EXPECT_EQ(d.replan_reason, "opposite_prefer_healthy");
}

TEST(ConflictDecisionTest, OppositeUnhealthy_UnknownModeTreatedAsUnhealthy)
{
  // amr1 healthy (1), amr2 unknown (-1) → treated as unhealthy → replan amr1
  auto amr1 = make_robot(
    "amr1", {"3", "4", "5", "6"},
    {"3", "4", "5", "6", "7", "8", "9", "10"},
    "low", false, 2, true, 1);
  auto amr2 = make_robot(
    "amr2", {"9", "8", "7", "6"},
    {"9", "8", "7", "6", "5", "4", "3", "2", "1"},
    "high", false, 2, true, -1);

  const auto d = decide_conflict_action("6", {amr1, amr2});
  EXPECT_EQ(d.replan_candidate_id, "amr1");
  EXPECT_EQ(d.replan_reason, "opposite_prefer_healthy");
}

TEST(ConflictDecisionTest, OppositeBothUnhealthy_NoReplanCandidate)
{
  // Both unhealthy → no replan candidate
  auto amr1 = make_robot(
    "amr1", {"3", "4", "5", "6"},
    {"3", "4", "5", "6", "7", "8", "9", "10"},
    "low", false, 2, true, 0);
  auto amr2 = make_robot(
    "amr2", {"9", "8", "7", "6"},
    {"9", "8", "7", "6", "5", "4", "3", "2", "1"},
    "high", false, 2, true, -1);

  const auto d = decide_conflict_action("6", {amr1, amr2});
  EXPECT_EQ(d.conflict_class, ConflictClass::StraightOpposite);
  ASSERT_EQ(d.pause_robot_ids.size(), 2u);
  EXPECT_TRUE(d.replan_candidate_id.empty());
  EXPECT_EQ(d.replan_reason, "opposite_both_unhealthy_no_replan");
}

// Short destination sequence ending inside the oncoming robot's path must still
// be C2 opposite: pause BOTH + replan. Must not be misclassified as C5 (pause
// only the longer robot because its approach zones are "exclusive").
TEST(ConflictDecisionTest, ShortSeqOpposite_PauseBothAtOverlap7)
{
  auto amr1 = make_robot(
    "amr1", {"4", "5", "6", "7"},
    {"4", "5", "6", "7"},
    "high");
  auto amr2 = make_robot(
    "amr2", {"10", "9", "8", "7"},
    {"10", "9", "8", "7", "6", "5", "4", "3", "2", "1", "21", "22"},
    "low");

  EXPECT_EQ(
    classify_travel_direction(amr1.zone_sequence, amr2.zone_sequence, "7"),
    TravelDirection::Opposite);

  const auto d = decide_conflict_action("7", {amr1, amr2});
  EXPECT_EQ(d.direction, TravelDirection::Opposite);
  EXPECT_TRUE(
    d.conflict_class == ConflictClass::StraightOpposite ||
    d.conflict_class == ConflictClass::JunctionOpposite);
  ASSERT_EQ(d.pause_robot_ids.size(), 2u);
  EXPECT_FALSE(d.replan_candidate_id.empty());
}

TEST(ConflictDecisionTest, ShortSeqOpposite_PauseBothAtMidCorridor6)
{
  // First mid-corridor overlap once AMR2 has reserved into 6 while AMR1 still
  // ends at 7 — previously C5 Case B paused only amr2 (exclusive 10/9/8).
  auto amr1 = make_robot(
    "amr1", {"4", "5", "6", "7"},
    {"4", "5", "6", "7"},
    "high");
  auto amr2 = make_robot(
    "amr2", {"9", "8", "7", "6"},
    {"10", "9", "8", "7", "6", "5", "4", "3", "2", "1", "21", "22"},
    "low");

  EXPECT_EQ(
    classify_travel_direction(amr1.zone_sequence, amr2.zone_sequence, "6"),
    TravelDirection::Opposite);

  const auto d = decide_conflict_action("6", {amr1, amr2});
  EXPECT_EQ(d.direction, TravelDirection::Opposite);
  EXPECT_NE(d.conflict_class, ConflictClass::BranchVsThrough);
  ASSERT_EQ(d.pause_robot_ids.size(), 2u);
  EXPECT_EQ(d.replan_candidate_id, "amr2");
  EXPECT_EQ(d.replan_reason, "opposite_both_healthy_lowest_priority");
}

TEST(ConflictDecisionTest, ShortSeqOpposite_PauseBothAtConflict5)
{
  auto amr1 = make_robot(
    "amr1", {"4", "5", "6", "7"},
    {"4", "5", "6", "7"},
    "high");
  auto amr2 = make_robot(
    "amr2", {"8", "7", "6", "5"},
    {"10", "9", "8", "7", "6", "5", "4", "3", "2", "1", "21", "22"},
    "low");

  const auto d = decide_conflict_action("5", {amr1, amr2});
  EXPECT_EQ(d.direction, TravelDirection::Opposite);
  EXPECT_NE(d.conflict_class, ConflictClass::BranchVsThrough);
  ASSERT_EQ(d.pause_robot_ids.size(), 2u);
  EXPECT_FALSE(d.replan_candidate_id.empty());
}

TEST(ConflictDecisionTest, LongCorridorOpposite_FromPartialStart)
{
  // Earlier live shape: AMR1 still early on a short remaining list; AMR2 far.
  auto amr1 = make_robot(
    "amr1", {"1", "2", "3", "4"},
    {"1", "2", "3", "4", "5", "6", "7"},
    "high");
  auto amr2 = make_robot(
    "amr2", {"11", "10", "9", "8"},
    {"11", "10", "9", "8", "7", "6", "5", "4", "3", "2", "1", "21", "22"},
    "low");

  // No reserved overlap yet; once they meet at 6 the decision must be dual-pause.
  amr1 = make_robot(
    "amr1", {"3", "4", "5", "6"},
    {"1", "2", "3", "4", "5", "6", "7"},
    "high");
  amr2 = make_robot(
    "amr2", {"9", "8", "7", "6"},
    {"11", "10", "9", "8", "7", "6", "5", "4", "3", "2", "1", "21", "22"},
    "low");

  const auto d = decide_conflict_action("6", {amr1, amr2});
  EXPECT_NE(d.conflict_class, ConflictClass::BranchVsThrough);
  ASSERT_EQ(d.pause_robot_ids.size(), 2u);
  EXPECT_FALSE(d.replan_candidate_id.empty());
}
