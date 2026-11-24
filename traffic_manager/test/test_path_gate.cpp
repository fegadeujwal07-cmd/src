#include <gtest/gtest.h>

#include "traffic_manager/path_gate.hpp"

TEST(PathGate, StartsSessionOnNewIntent)
{
  traffic_manager::PathGate gate;
  traffic_manager::msg::PathIntent intent;
  intent.robot_name = "amr1";
  intent.fleet_name = "fleet";
  intent.request_id = 1;
  intent.path_revision = 0;
  rmf_fleet_msgs::msg::Location loc;
  loc.x = 1.0;
  loc.y = 2.0;
  intent.path.push_back(loc);

  EXPECT_EQ(
    gate.accept_intent(intent),
    traffic_manager::IntentAcceptResult::NewOrUpdatedSession);
  ASSERT_NE(gate.session("amr1"), nullptr);
  EXPECT_EQ(gate.session("amr1")->released_through_index, -1);

  EXPECT_EQ(
    gate.accept_intent(intent),
    traffic_manager::IntentAcceptResult::Duplicate);
}

TEST(PathGate, ResubmitWithNewRequestIdKeepsSession)
{
  traffic_manager::PathGate gate;
  traffic_manager::msg::PathIntent intent;
  intent.robot_name = "amr1";
  intent.fleet_name = "fleet";
  intent.request_id = 1;
  intent.path_revision = 0;
  rmf_fleet_msgs::msg::Location loc;
  loc.x = 1.0;
  loc.y = 2.0;
  intent.path.push_back(loc);

  EXPECT_EQ(
    gate.accept_intent(intent),
    traffic_manager::IntentAcceptResult::NewOrUpdatedSession);
  ASSERT_NE(gate.session("amr1"), nullptr);
  EXPECT_TRUE(gate.session("amr1")->pending_initial_ecbs);

  intent.request_id = 2;
  EXPECT_EQ(
    gate.accept_intent(intent),
    traffic_manager::IntentAcceptResult::Duplicate);
  EXPECT_EQ(gate.session("amr1")->intent.request_id, 2u);
  EXPECT_TRUE(gate.session("amr1")->pending_initial_ecbs);
  EXPECT_EQ(gate.session("amr1")->released_through_index, -1);
}

TEST(PathGate, CursorOnlyChangeIsDuplicate)
{
  traffic_manager::PathGate gate;
  traffic_manager::msg::PathIntent intent;
  intent.robot_name = "amr1";
  intent.request_id = 1;
  intent.path_revision = 0;
  intent.cursor_index = 0;
  rmf_fleet_msgs::msg::Location loc;
  loc.x = 1.0;
  loc.y = 2.0;
  loc.yaw = 0.0;
  loc.level_name = "L1";
  intent.path.push_back(loc);

  EXPECT_EQ(
    gate.accept_intent(intent),
    traffic_manager::IntentAcceptResult::NewOrUpdatedSession);
  const auto route_hash = gate.session("amr1")->route_identity_hash;

  intent.cursor_index = 3;
  EXPECT_EQ(
    gate.accept_intent(intent),
    traffic_manager::IntentAcceptResult::Duplicate);
  EXPECT_EQ(gate.session("amr1")->route_identity_hash, route_hash);
}

TEST(PathGate, ScheduleYawChangePreservesAuthority)
{
  traffic_manager::PathGate gate;
  traffic_manager::msg::PathIntent intent;
  intent.robot_name = "amr1";
  intent.request_id = 1;
  rmf_fleet_msgs::msg::Location loc;
  loc.x = 1.0;
  loc.y = 2.0;
  loc.yaw = 0.0;
  loc.level_name = "L1";
  intent.path.push_back(loc);
  EXPECT_EQ(
    gate.accept_intent(intent),
    traffic_manager::IntentAcceptResult::NewOrUpdatedSession);
  gate.set_released_through("amr1", 0);
  gate.session("amr1")->committed_grant_id = 42;

  intent.path[0].yaw = 1.57;
  EXPECT_EQ(
    gate.accept_intent(intent),
    traffic_manager::IntentAcceptResult::ScheduleOnlyUpdate);
  EXPECT_EQ(gate.session("amr1")->released_through_index, 0);
  EXPECT_EQ(gate.session("amr1")->committed_grant_id, 42u);
}

TEST(PathGate, CommittedGrantIsMonotonic)
{
  traffic_manager::PathGate gate;
  traffic_manager::msg::PathIntent intent;
  intent.robot_name = "amr1";
  rmf_fleet_msgs::msg::Location loc;
  loc.x = 0.0;
  loc.y = 0.0;
  intent.path.push_back(loc);
  gate.accept_intent(intent);
  gate.set_released_through("amr1", 5);
  gate.set_released_through("amr1", 3);
  EXPECT_EQ(gate.session("amr1")->released_through_index, 5);
}

TEST(PathGate, NewRouteStagesWhileGrantOutstanding)
{
  traffic_manager::PathGate gate;
  traffic_manager::msg::PathIntent intent;
  intent.robot_name = "amr1";
  intent.dispatch_task_id = "task-a";
  intent.path_revision = 0;
  rmf_fleet_msgs::msg::Location loc;
  loc.x = 0.0;
  loc.y = 0.0;
  intent.path.push_back(loc);
  gate.accept_intent(intent);
  gate.set_released_through("amr1", 0);

  intent.dispatch_task_id = "task-b";
  intent.path_revision = 1;
  intent.path[0].x = 5.0;
  EXPECT_EQ(
    gate.accept_intent(intent),
    traffic_manager::IntentAcceptResult::NewOrUpdatedSession);
  ASSERT_NE(gate.session("amr1"), nullptr);
  EXPECT_TRUE(gate.session("amr1")->has_staged_intent);
  EXPECT_EQ(gate.session("amr1")->released_through_index, 0);

  EXPECT_TRUE(gate.activate_staged_intent("amr1"));
  EXPECT_FALSE(gate.session("amr1")->has_staged_intent);
  EXPECT_EQ(gate.session("amr1")->released_through_index, -1);
  EXPECT_EQ(gate.session("amr1")->intent.dispatch_task_id, "task-b");
}

TEST(PathGate, AcknowledgeReleaseRejectsStaleUpdateId)
{
  traffic_manager::PathGate gate;
  traffic_manager::msg::PathIntent intent;
  intent.robot_name = "amr1";
  rmf_fleet_msgs::msg::Location loc;
  loc.x = 0.0;
  loc.y = 0.0;
  intent.path.push_back(loc);
  gate.accept_intent(intent);
  gate.set_released_through("amr1", 2);
  auto * s = gate.session("amr1");
  s->awaiting_robot_ack = true;
  s->last_published_release_update_id = 7;
  s->committed_grant_id = 99;

  EXPECT_FALSE(gate.acknowledge_release("amr1", 6));
  EXPECT_TRUE(s->awaiting_robot_ack);
  EXPECT_TRUE(gate.acknowledge_release("amr1", 7));
  EXPECT_FALSE(s->awaiting_robot_ack);
  EXPECT_EQ(s->acknowledged_grant, 2);
  EXPECT_EQ(s->last_acked_grant_id, 99u);
}

TEST(PathGate, PendingExtensionLatchesUntilCleared)
{
  traffic_manager::PathGate gate;
  traffic_manager::msg::PathIntent intent;
  intent.robot_name = "amr1";
  rmf_fleet_msgs::msg::Location loc;
  loc.x = 1.0;
  loc.y = 1.0;
  intent.path.push_back(loc);
  gate.accept_intent(intent);
  gate.latch_pending_extension("amr1");
  EXPECT_TRUE(gate.session("amr1")->pending_extension);
  gate.latch_pending_extension("amr1");  // idempotent
  EXPECT_TRUE(gate.session("amr1")->pending_extension);
  gate.clear_pending_extension("amr1");
  EXPECT_FALSE(gate.session("amr1")->pending_extension);
}

TEST(PathGate, EmptyPathPreservesOutstandingGrant)
{
  traffic_manager::PathGate gate;
  traffic_manager::msg::PathIntent intent;
  intent.robot_name = "amr1";
  rmf_fleet_msgs::msg::Location loc;
  loc.x = 0.0;
  loc.y = 0.0;
  intent.path.push_back(loc);
  gate.accept_intent(intent);
  gate.set_released_through("amr1", 0);
  intent.path.clear();
  EXPECT_EQ(
    gate.accept_intent(intent),
    traffic_manager::IntentAcceptResult::NewOrUpdatedSession);
  ASSERT_NE(gate.session("amr1"), nullptr);
  EXPECT_TRUE(gate.session("amr1")->has_staged_intent);
  EXPECT_EQ(gate.session("amr1")->released_through_index, 0);
}

/// Regression for first-base deadlock: after Commit ACCEPTED publishes
/// release_update_id=0, matching ACK must clear awaiting_robot_ack so a
/// subsequent newBaseRequest can latch pending_extension for update 1.
/// (Markers rebuild must run outside mutex_ — see CommitProposal callback.)
TEST(PathGate, FirstReleaseAckThenExtensionDemand)
{
  traffic_manager::PathGate gate;
  traffic_manager::msg::PathIntent intent;
  intent.robot_name = "amr2";
  intent.dispatch_task_id = "planner-uuid";
  intent.task_id = "1";
  intent.path_revision = 0;
  for (int i = 0; i < 27; ++i) {
    rmf_fleet_msgs::msg::Location loc;
    loc.x = static_cast<double>(i);
    loc.y = 0.0;
    intent.path.push_back(loc);
  }
  ASSERT_EQ(
    gate.accept_intent(intent),
    traffic_manager::IntentAcceptResult::NewOrUpdatedSession);

  // Simulate commit-then-publish of first base (through_index=6, update 0).
  auto * s = gate.session("amr2");
  ASSERT_NE(s, nullptr);
  gate.set_released_through("amr2", 6);
  s->committed_grant_id = 1;
  s->last_published_release_update_id = 0;
  s->next_release_update_id = 1;
  s->awaiting_robot_ack = true;
  gate.clear_pending_extension("amr2");

  // Wrong slot must not clear ACK (no bypass).
  EXPECT_FALSE(gate.acknowledge_release("amr2", 1));
  EXPECT_TRUE(s->awaiting_robot_ack);
  EXPECT_EQ(s->released_through_index, 6);

  // Matching update-0 ACK clears await; committed tip unchanged.
  EXPECT_TRUE(gate.acknowledge_release("amr2", 0));
  EXPECT_FALSE(s->awaiting_robot_ack);
  EXPECT_EQ(s->released_through_index, 6);
  EXPECT_EQ(s->acknowledged_grant, 6);

  // Subsequent newBaseRequest may latch extension for next query / update 1.
  gate.latch_pending_extension("amr2");
  EXPECT_TRUE(s->pending_extension);
  EXPECT_FALSE(s->awaiting_robot_ack);
  EXPECT_EQ(s->last_published_release_update_id, 0u);
  EXPECT_EQ(s->next_release_update_id, 1u);
}

/// Same-tip AUTHORITY WAIT that wrongly bumped release_update_id 0→1 while
/// execution still reports slot 0: recover ACK and roll id back.
TEST(PathGate, SameTipAckRecoversAfterMistakenBump)
{
  traffic_manager::PathGate gate;
  traffic_manager::msg::PathIntent intent;
  intent.robot_name = "amr2";
  intent.path_revision = 0;
  for (int i = 0; i < 10; ++i) {
    rmf_fleet_msgs::msg::Location loc;
    loc.x = static_cast<double>(i);
    intent.path.push_back(loc);
  }
  ASSERT_EQ(
    gate.accept_intent(intent),
    traffic_manager::IntentAcceptResult::NewOrUpdatedSession);
  auto * s = gate.session("amr2");
  ASSERT_NE(s, nullptr);
  gate.set_released_through("amr2", 6);
  s->acknowledged_grant = 6;  // already ACK'd FREE tip
  s->committed_grant_id = 1;
  s->last_published_release_update_id = 1;  // mistaken same-tip bump
  s->next_release_update_id = 2;
  s->awaiting_robot_ack = true;

  EXPECT_TRUE(gate.acknowledge_release("amr2", 0));
  EXPECT_FALSE(s->awaiting_robot_ack);
  EXPECT_EQ(s->last_published_release_update_id, 0u);
  // The next id is NOT rolled back: update 1 may still reach the robot later
  // (execution holds it until idle), so reusing 1 for the next extension
  // would be dropped as a duplicate and strand the robot (amr2, 2026-11-20).
  EXPECT_EQ(s->next_release_update_id, 2u);
  EXPECT_EQ(s->acknowledged_grant, 6);

  // Tip advanced: stale slot must not clear a newer expected id.
  gate.set_released_through("amr2", 12);
  s->last_published_release_update_id = 1;
  s->next_release_update_id = 2;
  s->awaiting_robot_ack = true;
  EXPECT_FALSE(gate.acknowledge_release("amr2", 0));
  EXPECT_TRUE(s->awaiting_robot_ack);
}

// Back-to-back task: the finished leg's cursor/progress must not carry over
// into the next task (it caused a false Grant COMPLETED on the new task).
TEST(PathGate, StagedActivationResetsLegCursor)
{
  traffic_manager::PathGate gate;
  traffic_manager::msg::PathIntent intent;
  intent.robot_name = "amr2";
  intent.request_id = 1;
  intent.dispatch_task_id = "task-a";
  for (int i = 0; i < 10; ++i) {
    rmf_fleet_msgs::msg::Location loc;
    loc.x = 2.0 * i;
    intent.path.push_back(loc);
  }
  gate.accept_intent(intent);
  gate.set_released_through("amr2", 9);
  auto * s = gate.session("amr2");
  ASSERT_NE(s, nullptr);
  s->leg_cursor_index = 9;
  s->observed_progress_index = 9;

  intent.request_id = 2;
  intent.dispatch_task_id = "task-b";
  for (auto & loc : intent.path) {
    loc.y = 10.0;
  }
  gate.accept_intent(intent);
  ASSERT_TRUE(gate.session("amr2")->has_staged_intent);
  ASSERT_TRUE(gate.activate_staged_intent("amr2"));
  s = gate.session("amr2");
  EXPECT_EQ(s->leg_cursor_index, -1);
  EXPECT_EQ(s->observed_progress_index, -1);
  EXPECT_EQ(s->released_through_index, -1);
  EXPECT_EQ(s->intent.request_id, 2u);
}
