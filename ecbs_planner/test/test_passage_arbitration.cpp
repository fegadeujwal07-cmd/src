#include <algorithm>
#include <gtest/gtest.h>

#include "ecbs_planner/passage_arbitration.hpp"
#include "ecbs_planner/route_occurrence.hpp"

using ecbs_planner::ConflictCluster;
using ecbs_planner::ConflictRecord;
using ecbs_planner::ParticipantOverlap;
using ecbs_planner::RobotPriorityFacts;
using ecbs_planner::arbitrate_passage;
using ecbs_planner::b2b_front_rear_from_progress;
using ecbs_planner::kStatusUnresolvedOpposite;
using ecbs_planner::make_occurrence;

ConflictRecord make_geo(
  const std::string & id,
  const std::string & a,
  const std::string & b,
  const std::string & reason,
  bool a_blocks_b = false,
  bool b_blocks_a = false)
{
  ConflictRecord r;
  r.conflict_id = id;
  ParticipantOverlap pa;
  pa.amr_id = a;
  pa.blocks_other = a_blocks_b;
  pa.blocked_by_other = b_blocks_a;
  ParticipantOverlap pb;
  pb.amr_id = b;
  pb.blocks_other = b_blocks_a;
  pb.blocked_by_other = a_blocks_b;
  r.participants = {pa, pb};
  r.wait_reason = reason;
  r.topology = reason;
  r.mandatory = false;
  return r;
}

TEST(PassageArbitration, FullyOppositeHardHoldEvenWithCycle)
{
  ConflictCluster c;
  c.id = 1;
  c.members = {"amr1", "amr2"};
  c.has_cycle = true;
  c.proposed_precedence = {"amr1", "amr2"};
  std::vector<ConflictRecord> recs = {
    make_geo("o", "amr1", "amr2", "opposite"),
  };
  std::unordered_map<std::string, RobotPriorityFacts> facts;
  facts["amr1"] = {"amr1", "t1", "high", 1, true, true, 0, false, true};
  facts["amr2"] = {"amr2", "t2", "low", 1, true, true, 0, false, true};
  const auto d = arbitrate_passage(c, recs, facts, {});
  EXPECT_TRUE(d.unresolved_opposite);
  EXPECT_TRUE(d.go_robots.empty());
  EXPECT_EQ(d.wait_robots.size(), 2u);
  EXPECT_EQ(d.reason, kStatusUnresolvedOpposite);
}

TEST(PassageArbitration, PriorityCannotBypassOpposite)
{
  ConflictCluster c;
  c.id = 1;
  c.members = {"amr1", "amr2"};
  c.has_cycle = true;
  c.proposed_precedence = {"amr1", "amr2"};
  std::vector<ConflictRecord> recs = {
    make_geo("o", "amr1", "amr2", "opposite"),
  };
  std::unordered_map<std::string, RobotPriorityFacts> facts;
  facts["amr1"] = {"amr1", "t1", "high", 1, true, true, 0, false, true};
  facts["amr2"] = {"amr2", "t2", "", 0, true, true, 0, false, true};
  const auto d = arbitrate_passage(c, recs, facts, {});
  EXPECT_TRUE(d.go_robots.empty());
  EXPECT_TRUE(d.unresolved_opposite);
}

TEST(PassageArbitration, UnrelatedEasinessCannotCancelOppositePair)
{
  ConflictCluster c;
  c.id = 1;
  c.members = {"amr1", "amr2", "amr3"};
  c.proposed_precedence = {"amr1", "amr2", "amr3"};
  // Fully opposite AMR1/AMR2; AMR3 has easiness vs AMR1 on a separate overlap.
  auto opp = make_geo("o12", "amr1", "amr2", "opposite");
  auto easy = make_geo("e13", "amr1", "amr3", "plus", false, true);
  std::vector<ConflictRecord> recs = {opp, easy};
  std::unordered_map<std::string, RobotPriorityFacts> facts;
  facts["amr1"] = {"amr1", "t1", "high", 1, true, true, 0, false, true};
  facts["amr2"] = {"amr2", "t2", "low", 1, true, true, 0, false, true};
  facts["amr3"] = {"amr3", "t3", "low", 1, true, true, 0, false, true};
  const auto d = arbitrate_passage(c, recs, facts, {});
  EXPECT_TRUE(d.unresolved_opposite);
  EXPECT_TRUE(
    std::find(d.go_robots.begin(), d.go_robots.end(), "amr1") ==
      d.go_robots.end());
  EXPECT_TRUE(
    std::find(d.go_robots.begin(), d.go_robots.end(), "amr2") ==
      d.go_robots.end());
}

TEST(PassageArbitration, EasinessUsesRobotIdentityNotSortOrder)
{
  ConflictCluster c;
  c.id = 1;
  c.members = {"amr1", "amr2"};
  c.proposed_precedence = {"amr1", "amr2"};
  // amr2 blocks amr1 → amr1 has blocked_by_other → amr1 GO
  auto rec = make_geo("e", "amr2", "amr1", "plus", true, false);
  // Participants stored as amr2 then amr1 (not lex sorted) — must still work.
  std::vector<ConflictRecord> recs = {rec};
  std::unordered_map<std::string, RobotPriorityFacts> facts;
  facts["amr1"] = {"amr1", "t1", "low", 1, true, true, 0, false, true};
  facts["amr2"] = {"amr2", "t2", "high", 1, true, true, 0, false, true};
  const auto d = arbitrate_passage(c, recs, facts, {});
  ASSERT_EQ(d.go_robots.size(), 1u);
  EXPECT_EQ(d.go_robots.front(), "amr1");
  EXPECT_EQ(d.preference_kind, "easiness");
}

TEST(PassageArbitration, StaleTaskPriorityIgnored)
{
  ConflictCluster c;
  c.id = 1;
  c.members = {"amr1", "amr2"};
  c.proposed_precedence = {"amr1", "amr2"};
  std::vector<ConflictRecord> recs = {
    make_geo("t", "amr1", "amr2", "plus", false, false),
  };
  std::unordered_map<std::string, RobotPriorityFacts> facts;
  facts["amr1"] = {"amr1", "old-task", "high", 1, true, true, 0, false, true};
  facts["amr2"] = {"amr2", "task-b", "low", 1, true, true, 0, false, true};
  std::unordered_map<std::string, std::string> current = {
    {"amr1", "new-task"},
    {"amr2", "task-b"},
  };
  const auto d = arbitrate_passage(c, recs, facts, {}, current);
  EXPECT_FALSE(d.unresolved_opposite);
  if (!d.go_robots.empty()) {
    EXPECT_NE(d.go_robots.front(), "");
  }
}

TEST(PassageArbitration, B2BUsesProgressOnSharedParticipantsOnly)
{
  ConflictCluster c;
  c.id = 1;
  c.members = {"amr1", "amr2", "amr3"};
  c.proposed_precedence = {"amr1", "amr2", "amr3"};
  auto rec = make_geo("b", "amr1", "amr2", "back_to_back");
  // Same directed strip in metres: entry (0,0) → exit (10,0).
  for (auto & p : rec.participants) {
    p.entry = make_occurrence(1, 0, 0.0, 0.0);
    p.exit = make_occurrence(1, 10, 10.0, 0.0);
  }
  std::vector<ConflictRecord> recs = {rec};
  std::unordered_map<std::string, RobotPriorityFacts> facts;
  // amr2 physically further along the strip; amr3 not a participant.
  facts["amr1"] = {"amr1", "t1", "high", 1, true, true, 2, true, true, 2.0, 0.0, true, 0.0};
  facts["amr2"] = {"amr2", "t2", "low", 1, true, true, 8, true, true, 8.0, 0.0, true, 0.0};
  facts["amr3"] = {"amr3", "t3", "low", 1, true, true, 99, true, true, 99.0, 0.0, true, 0.0};
  const auto d = arbitrate_passage(c, recs, facts, {});
  ASSERT_EQ(d.go_robots.size(), 1u);
  EXPECT_EQ(d.go_robots.front(), "amr2");
  EXPECT_EQ(d.preference_kind, "b2b");
  EXPECT_EQ(b2b_front_rear_from_progress(8, 2), 1);
}

TEST(PassageArbitration, InfeasibleRobotExcluded)
{
  ConflictCluster c;
  c.id = 1;
  c.members = {"amr1", "amr2"};
  c.proposed_precedence = {"amr1", "amr2"};
  std::vector<ConflictRecord> recs = {
    make_geo("t", "amr1", "amr2", "plus", false, true),
  };
  std::unordered_map<std::string, RobotPriorityFacts> facts;
  facts["amr1"] = {"amr1", "t1", "high", 1, true, true, 0, false, false};
  facts["amr2"] = {"amr2", "t2", "low", 1, true, true, 0, false, true};
  const auto d = arbitrate_passage(c, recs, facts, {});
  EXPECT_EQ(
    std::find(d.wait_robots.begin(), d.wait_robots.end(), "amr1") !=
      d.wait_robots.end(),
    true);
}

TEST(PassageArbitration, PriorityCannotRevokeGrant)
{
  ConflictCluster c;
  c.id = 1;
  c.members = {"amr1", "amr2"};
  c.proposed_precedence = {"amr1", "amr2"};
  std::vector<ConflictRecord> recs = {
    make_geo("t", "amr1", "amr2", "plus", false, false),
  };
  std::unordered_map<std::string, RobotPriorityFacts> facts;
  facts["amr1"] = {"amr1", "t1", "low", 1, true, true, 0, false, true};
  facts["amr2"] = {"amr2", "t2", "high", 1, true, true, 0, false, true};
  const auto d = arbitrate_passage(c, recs, facts, {"amr1"});
  EXPECT_NE(
    std::find(d.go_robots.begin(), d.go_robots.end(), "amr1"),
    d.go_robots.end());
}
