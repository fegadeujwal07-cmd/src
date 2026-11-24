#include <gtest/gtest.h>

#include <cstdlib>
#include <fstream>
#include <string>

#include "monitor/zone_tracker.hpp"

namespace
{

/// First nav graph that actually exists on this machine.
///
/// The map-based tests used to hard-code one developer's absolute path, so on
/// every other machine they GTEST_SKIP()ped and silently never ran.
std::string nav_graph_path()
{
  const char * env = std::getenv("ZONE_TRACKER_NAV_GRAPH");
  const std::string candidates[] = {
    env ? std::string(env) : std::string(),
    "install/execution/share/execution/config/nav_graphs/0.yaml",
    "/home/vinay/FMS_Main/install/execution/share/execution/config/nav_graphs/0.yaml",
    "/home/robo-intern/apurva/install/execution/share/execution/config/nav_graphs/0.yaml",
  };
  for (const auto & path : candidates) {
    if (path.empty()) {
      continue;
    }
    std::ifstream probe(path);
    if (probe.good()) {
      return path;
    }
  }
  return {};
}

TEST(ZoneTrackerTest, NormalizeRobotId)
{
  EXPECT_EQ(monitor::ZoneTracker::normalize_robot_id("amr1"), "amr1");
  EXPECT_EQ(monitor::ZoneTracker::normalize_robot_id("AMR2"), "amr2");
  EXPECT_EQ(monitor::ZoneTracker::normalize_robot_id("robot_3"), "amr3");
}

TEST(ZoneTrackerTest, ZoneSequenceDeduplicatesConsecutiveZones)
{
  std::unordered_map<std::string, std::vector<std::pair<double, double>>> zone_rects{
    {"1", {{0.0, 0.0}, {10.0, 0.0}, {10.0, 10.0}, {0.0, 10.0}}},
    {"2", {{10.0, 0.0}, {20.0, 0.0}, {20.0, 10.0}, {10.0, 10.0}}},
  };

  const std::vector<std::pair<double, double>> path{
    {2.0, 5.0}, {8.0, 5.0}, {12.0, 5.0}, {18.0, 5.0},
  };

  const auto sequence = monitor::ZoneTracker::zone_sequence_from_xy_points(
    path, {}, {}, zone_rects, 4);

  ASSERT_GE(sequence.size(), 2U);
  EXPECT_EQ(sequence.front(), "1");
  EXPECT_EQ(sequence.back(), "2");
}

TEST(ZoneTrackerTest, CurrentAndNextFromPose)
{
  std::unordered_map<std::string, std::vector<std::pair<double, double>>> zone_rects{
    {"10", {{0.0, 0.0}, {10.0, 0.0}, {10.0, 10.0}, {0.0, 10.0}}},
    {"11", {{10.0, 0.0}, {20.0, 0.0}, {20.0, 10.0}, {10.0, 10.0}}},
    {"12", {{20.0, 0.0}, {30.0, 0.0}, {30.0, 10.0}, {20.0, 10.0}}},
    {"13", {{30.0, 0.0}, {40.0, 0.0}, {40.0, 10.0}, {30.0, 10.0}}},
    {"14", {{40.0, 0.0}, {50.0, 0.0}, {50.0, 10.0}, {40.0, 10.0}}},
  };
  const std::vector<std::string> sequence{"10", "11", "12", "13", "14"};

  const auto snapshot = monitor::ZoneTracker::current_and_next_from_pose(
    5.0, 5.0, {}, {}, zone_rects, sequence, -1);

  EXPECT_EQ(snapshot.current_zone, "10");
  ASSERT_EQ(snapshot.next_zone.size(), 3U);
  EXPECT_EQ(snapshot.next_zone[0], "11");
  EXPECT_EQ(snapshot.next_zone[1], "12");
  EXPECT_EQ(snapshot.next_zone[2], "13");
  // Passed zones vanished: remaining starts at current.
  ASSERT_EQ(snapshot.zone_sequence.size(), 5U);
  EXPECT_EQ(snapshot.zone_sequence.front(), "10");
}

TEST(ZoneTrackerTest, NextZoneLookaheadShrinksNearEnd)
{
  std::unordered_map<std::string, std::vector<std::pair<double, double>>> zone_rects{
    {"10", {{0.0, 0.0}, {10.0, 0.0}, {10.0, 10.0}, {0.0, 10.0}}},
    {"11", {{10.0, 0.0}, {20.0, 0.0}, {20.0, 10.0}, {10.0, 10.0}}},
    {"12", {{20.0, 0.0}, {30.0, 0.0}, {30.0, 10.0}, {20.0, 10.0}}},
  };
  const std::vector<std::string> sequence{"10", "11", "12"};

  // Pose in zone 11 → next should be only ["12"] (1 remaining).
  const auto mid = monitor::ZoneTracker::current_and_next_from_pose(
    15.0, 5.0, {}, {}, zone_rects, sequence, 0);
  EXPECT_EQ(mid.current_zone, "11");
  ASSERT_EQ(mid.next_zone.size(), 1U);
  EXPECT_EQ(mid.next_zone[0], "12");
  ASSERT_EQ(mid.zone_sequence.size(), 2U);
  EXPECT_EQ(mid.zone_sequence[0], "11");
  EXPECT_EQ(mid.zone_sequence[1], "12");

  // Pose in last zone → next empty; sequence is only current.
  const auto last = monitor::ZoneTracker::current_and_next_from_pose(
    25.0, 5.0, {}, {}, zone_rects, sequence, 1);
  EXPECT_EQ(last.current_zone, "12");
  EXPECT_TRUE(last.next_zone.empty());
  ASSERT_EQ(last.zone_sequence.size(), 1U);
  EXPECT_EQ(last.zone_sequence[0], "12");
}

TEST(ZoneTrackerTest, SnapshotEquality)
{
  monitor::ZoneSnapshot a{"10", {"11", "12"}, {"10", "11", "12"}, {"9"}};
  monitor::ZoneSnapshot b{"10", {"11", "12"}, {"10", "11", "12"}, {"9"}};
  monitor::ZoneSnapshot c{"10", {"12"}, {"10", "11", "12"}, {"9"}};
  EXPECT_EQ(a, b);
  EXPECT_NE(a, c);
}

TEST(ZoneTrackerTest, NumericPathUpdatePreferred)
{
  // Synthetic map via static helpers only — instance path needs YAML.
  // This verifies parse + sequence still work for CSV fallback.
  std::unordered_map<std::string, std::vector<std::pair<double, double>>> zone_rects{
    {"A", {{0.0, 0.0}, {5.0, 0.0}, {5.0, 5.0}, {0.0, 5.0}}},
    {"B", {{5.0, 0.0}, {10.0, 0.0}, {10.0, 5.0}, {5.0, 5.0}}},
  };
  const std::vector<std::pair<double, double>> xy{{1.0, 2.5}, {7.0, 2.5}};
  const auto seq = monitor::ZoneTracker::zone_sequence_from_xy_points(
    xy, {}, {}, zone_rects, 4);
  ASSERT_GE(seq.size(), 2U);
  EXPECT_EQ(seq.front(), "A");
  EXPECT_EQ(seq.back(), "B");
}

TEST(ZoneTrackerTest, LoadProductionNavGraphIfAvailable)
{
  monitor::ZoneTracker tracker;
  const std::string yaml_path =
    "/home/robo-intern/apurva/install/execution/share/execution/config/nav_graphs/0.yaml";
  if (!tracker.load_map(yaml_path, "L1")) {
    GTEST_SKIP() << "Nav graph YAML not available at " << yaml_path;
  }

  tracker.update_robot_planned_path(
    "amr1",
    {"25.832,33.992,-3.142", "10.650,18.347,-1.571"});
  tracker.update_robot_pose("amr1", 25.832, 33.992);

  const monitor::ZoneSnapshot zones = tracker.get_robot_zones("amr1");
  EXPECT_FALSE(zones.zone_sequence.empty());
  EXPECT_FALSE(zones.current_zone.empty());

  // Numeric path API should also populate sequence.
  tracker.update_robot_planned_path_xy(
    "amr2",
    {{25.832, 33.992}, {10.650, 18.347}});
  tracker.update_robot_pose("amr2", 25.832, 33.992);
  const monitor::ZoneSnapshot zones2 = tracker.get_robot_zones("amr2");
  EXPECT_FALSE(zones2.zone_sequence.empty());
}

TEST(ZoneTrackerTest, CompletionSnapsToRouteDestination)
{
  monitor::ZoneTracker tracker;
  const std::string yaml_path =
    "/home/robo-intern/apurva/install/execution/share/execution/config/nav_graphs/0.yaml";
  if (!tracker.load_map(yaml_path, "L1")) {
    GTEST_SKIP() << "Nav graph YAML not available at " << yaml_path;
  }

  // Conveyor5 corridor: route 10 -> 9 -> 8 -> 7, destination at (29.7, 7.05).
  const std::vector<std::pair<double, double>> path{
    {35.60625, 7.05},
    {33.6375, 7.05},
    {31.66875, 7.05},
    {29.7, 7.05},
  };
  tracker.update_robot_planned_path_xy("amr2", path);

  // Robot in zone 8 (one segment before destination) while task completes.
  tracker.update_robot_pose("amr2", 31.66875, 7.05);
  const monitor::ZoneSnapshot live = tracker.get_robot_zones("amr2");
  EXPECT_EQ(live.current_zone, "8");
  ASSERT_EQ(live.next_zone.size(), 1U);
  EXPECT_EQ(live.next_zone[0], "7");

  const monitor::ZoneSnapshot done = tracker.get_robot_zones_at_completion("amr2");
  EXPECT_EQ(done.current_zone, "7");
  EXPECT_TRUE(done.next_zone.empty());
  ASSERT_EQ(done.zone_sequence.size(), 1U);
  EXPECT_EQ(done.zone_sequence[0], "7");

  // Robot already at Conveyor5 / zone 7 — completion should keep live zones.
  tracker.update_robot_pose("amr2", 29.7, 7.05);
  const monitor::ZoneSnapshot at_dest = tracker.get_robot_zones_at_completion("amr2");
  EXPECT_EQ(at_dest.current_zone, "7");
  EXPECT_TRUE(at_dest.next_zone.empty());
}

TEST(ZoneTrackerTest, IdleZonesFromPoseOnly)
{
  monitor::ZoneTracker tracker;
  const std::string yaml_path =
    "/home/robo-intern/apurva/install/execution/share/execution/config/nav_graphs/0.yaml";
  if (!tracker.load_map(yaml_path, "L1")) {
    GTEST_SKIP() << "Nav graph YAML not available at " << yaml_path;
  }

  tracker.update_robot_pose("amr2", 29.7, 7.05);
  const monitor::ZoneSnapshot idle = tracker.get_robot_idle_zones("amr2");
  EXPECT_EQ(idle.current_zone, "7");
  EXPECT_TRUE(idle.next_zone.empty());
  ASSERT_EQ(idle.zone_sequence.size(), 1U);
  EXPECT_EQ(idle.zone_sequence[0], "7");
}

// Rear zones must depend on where the robot is FACING, not where it has been.
// Asserting the heading dependency rather than hard-coded zone ids keeps this
// valid when the nav graph is re-surveyed.
TEST(ZoneTrackerTest, RearZonesFollowHeading)
{
  monitor::ZoneTracker tracker;
  if (!tracker.load_map(nav_graph_path(), "L1")) {
    GTEST_SKIP() << "Nav graph YAML not available";
  }

  // Several ticks per heading: a zone is published only after consecutive
  // sightings (add debounce).
  auto settle = [&tracker](double x, double y, double yaw) {
      monitor::ZoneSnapshot snap;
      for (int i = 0; i < 4; ++i) {
        tracker.update_robot_pose("amr_rear", x, y, yaw);
        snap = tracker.get_robot_idle_zones("amr_rear");
      }
      return snap;
    };

  const monitor::ZoneSnapshot facing_east = settle(-8.9725, -11.38, 0.0);
  ASSERT_FALSE(facing_east.current_zone.empty());
  ASSERT_FALSE(facing_east.rear_zones.empty())
    << "a robot mid-corridor must have something behind it";

  for (const auto & zone : facing_east.rear_zones) {
    EXPECT_NE(zone, facing_east.current_zone)
      << "a rear zone is never the zone the robot occupies";
  }

  // Turn around at the SAME position: what is behind must change.
  const monitor::ZoneSnapshot facing_west = settle(-8.9725, -11.38, 3.14159);
  ASSERT_FALSE(facing_west.rear_zones.empty());
  EXPECT_EQ(facing_east.current_zone, facing_west.current_zone)
    << "same pose, so the occupied zone must not move";
  EXPECT_NE(facing_east.rear_zones, facing_west.rear_zones)
    << "reversing the heading must reverse which side is behind";
}

// A parked robot must KEEP its rear zones. The re-evaluation gate skips the
// graph walk when the pose has not changed; if it skipped the debounce too, a
// stationary robot would never confirm a zone and would sit unprotected —
// which is the case this feature exists for.
TEST(ZoneTrackerTest, RearZonesPersistWhileStationary)
{
  monitor::ZoneTracker tracker;
  if (!tracker.load_map(nav_graph_path(), "L1")) {
    GTEST_SKIP() << "Nav graph YAML not available";
  }

  tracker.update_robot_pose("amr_park", -8.9725, -11.38, 0.0);
  monitor::ZoneSnapshot snap;
  for (int i = 0; i < 10; ++i) {
    snap = tracker.get_robot_idle_zones("amr_park");
  }
  EXPECT_FALSE(snap.rear_zones.empty())
    << "a robot that never moves must still report rear zones";
}

// One path behind means ONE rear zone.
//
// The first version walked a fixed 4 m budget and reported every zone it met,
// so a straight lane with ~1.3 m zone spacing claimed three zones for a single
// path behind the robot (observed 2026-09-10: amr2 in zone 12 published
// ['1','11'], amr1 in zone 16 published ['14','15','5']). Each branch now stops
// at the first zone that is not the robot's own.
TEST(ZoneTrackerTest, StraightLaneYieldsExactlyOneRearZone)
{
  monitor::ZoneTracker tracker;
  if (!tracker.load_map(nav_graph_path(), "L1")) {
    GTEST_SKIP() << "Nav graph YAML not available";
  }

  // Mid-corridor, away from any junction: one lane ahead, one behind.
  monitor::ZoneSnapshot snap;
  for (int i = 0; i < 4; ++i) {
    tracker.update_robot_pose("amr_lane", -8.9725, -11.38, 0.0);
    snap = tracker.get_robot_idle_zones("amr_lane");
  }
  EXPECT_EQ(snap.rear_zones.size(), 1U)
    << "a robot on a straight lane has one path behind it, so one rear zone";
}

// A robot parked across a junction has nothing straight behind it, and both
// flanks must be claimed rather than one picked arbitrarily.
TEST(ZoneTrackerTest, PerpendicularAtJunctionClaimsBothFlanks)
{
  monitor::ZoneTracker tracker;
  if (!tracker.load_map(nav_graph_path(), "L1")) {
    GTEST_SKIP() << "Nav graph YAML not available";
  }

  auto settle = [&tracker](double yaw) {
      monitor::ZoneSnapshot snap;
      for (int i = 0; i < 4; ++i) {
        // v4: a 3-lane junction, arms west / east / south.
        tracker.update_robot_pose("amr_junction", -6.31, -11.38, yaw);
        snap = tracker.get_robot_idle_zones("amr_junction");
      }
      return snap;
    };

  // Facing east, the west arm is squarely behind — the abeam south arm is NOT
  // rear, and a robot coming down it would strike the side, not the back.
  EXPECT_EQ(settle(0.0).rear_zones.size(), 1U)
    << "an arm dead abeam is beside the robot, not behind it";

  // Facing south, nothing is behind: the west and east arms both sit at exactly
  // 90 degrees, so the relaxed pass claims both.
  EXPECT_EQ(settle(-1.5708).rear_zones.size(), 2U)
    << "no zone straight back, so both flanks are rear candidates";
}

}  // namespace

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
