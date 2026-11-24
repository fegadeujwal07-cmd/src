#include "zone_manager/conflict_detector.hpp"
#include "zone_manager/world_model.hpp"

#include <gtest/gtest.h>

using zone_manager::ConflictDetector;
using zone_manager::ConflictEventType;
using zone_manager::WorldModel;

TEST(WorldModelTest, ReservesCurrentAndNextThree)
{
  WorldModel world;
  world.register_zones({"12", "13", "14", "15", "18"});

  world.update_zones("R1", "12", {"13", "14", "15"});
  auto affected = world.sync_dirty_occupancy();
  EXPECT_EQ(affected.size(), 4u);
  EXPECT_EQ(world.occupant_count("12"), 1u);
  EXPECT_EQ(world.occupant_count("15"), 1u);

  const auto * r1 = world.robot("R1");
  ASSERT_NE(r1, nullptr);
  EXPECT_EQ(r1->reserved_zones().size(), 4u);
}

TEST(WorldModelTest, OnlyChangedZonesOnMove)
{
  WorldModel world;
  world.update_zones("R1", "10", {"11", "12", "13"});
  world.sync_dirty_occupancy();

  world.update_zones("R1", "11", {"12", "13", "14"});
  auto affected = world.sync_dirty_occupancy();
  EXPECT_EQ(affected.count("10"), 1u);
  EXPECT_EQ(affected.count("14"), 1u);
  EXPECT_EQ(affected.count("11"), 0u);
  EXPECT_EQ(affected.count("12"), 0u);
}

TEST(ConflictDetectorTest, DetectResolveNoDuplicate)
{
  WorldModel world;
  ConflictDetector detector;

  world.update_zones("R1", "12", {"13", "14", "15"});
  detector.evaluate(world, world.sync_dirty_occupancy());

  world.update_zones("R3", "14", {"25", "26", "27"});
  auto e1 = detector.evaluate(world, world.sync_dirty_occupancy());
  ASSERT_EQ(e1.size(), 1u);
  EXPECT_EQ(e1[0].type, ConflictEventType::Detected);
  EXPECT_EQ(e1[0].zone_id, "14");
  EXPECT_EQ(e1[0].robot_ids.size(), 2u);

  // Same state again → no event
  world.update_zones("R3", "14", {"25", "26", "27"});
  auto e2 = detector.evaluate(world, world.sync_dirty_occupancy());
  EXPECT_TRUE(e2.empty());

  world.update_zones("R3", "25", {"26", "27"});
  auto e3 = detector.evaluate(world, world.sync_dirty_occupancy());
  ASSERT_EQ(e3.size(), 1u);
  EXPECT_EQ(e3[0].type, ConflictEventType::Resolved);
}
