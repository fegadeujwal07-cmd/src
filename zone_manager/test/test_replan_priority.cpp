#include "zone_manager/replan_priority.hpp"

#include <gtest/gtest.h>

#include <chrono>

using zone_manager::RobotPriorityFacts;
using zone_manager::select_lowest_priority_robot;
using zone_manager::task_priority_rank;
using zone_manager::is_loaded_status;
using zone_manager::parse_eta_minutes;

TEST(ReplanPriorityTest, TaskPriorityBeatsLoaded)
{
  // amr1 has task priority; amr2 is only loaded → amr2 is replan candidate.
  const auto t0 = std::chrono::steady_clock::now();
  std::vector<RobotPriorityFacts> robots = {
    {"amr1", "high", false, 5.0, t0},
    {"amr2", "", true, 1.0, t0},
  };
  EXPECT_EQ(select_lowest_priority_robot(robots), "amr2");
}

TEST(ReplanPriorityTest, LoadedBeatsEmptyWhenNoPriority)
{
  const auto t0 = std::chrono::steady_clock::now();
  std::vector<RobotPriorityFacts> robots = {
    {"amr1", "", false, 1.0, t0},
    {"amr2", "", true, 5.0, t0},
  };
  EXPECT_EQ(select_lowest_priority_robot(robots), "amr1");
}

TEST(ReplanPriorityTest, LargerEtaLosesWhenPriorityAndLoadEqual)
{
  const auto t0 = std::chrono::steady_clock::now();
  std::vector<RobotPriorityFacts> robots = {
    {"amr1", "", true, 1.0, t0},
    {"amr2", "", true, 9.0, t0},
  };
  EXPECT_EQ(select_lowest_priority_robot(robots), "amr2");
}

TEST(ReplanPriorityTest, FifoFirstRequesterLoses)
{
  const auto t0 = std::chrono::steady_clock::now();
  const auto t1 = t0 + std::chrono::seconds(1);
  std::vector<RobotPriorityFacts> robots = {
    {"amr1", "", false, std::nullopt, t0},  // first request
    {"amr2", "", false, std::nullopt, t1},
  };
  EXPECT_EQ(select_lowest_priority_robot(robots), "amr1");
}

TEST(ReplanPriorityTest, Helpers)
{
  EXPECT_EQ(task_priority_rank("high"), 2);
  EXPECT_EQ(task_priority_rank("low"), 1);
  EXPECT_EQ(task_priority_rank(""), 0);
  EXPECT_TRUE(is_loaded_status(4));   // loaded
  EXPECT_FALSE(is_loaded_status(2));  // unloaded (empty)
  EXPECT_FALSE(is_loaded_status(1));
  EXPECT_FALSE(is_loaded_status(0));
  EXPECT_NEAR(*parse_eta_minutes("1.20 min"), 1.20, 1e-6);
}
