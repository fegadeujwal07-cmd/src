#include "zone_manager/behind_robot.hpp"

#include <gtest/gtest.h>

#include <string>
#include <unordered_map>
#include <vector>

using zone_manager::reserved_zone_index;
using zone_manager::select_robots_coming_from_behind;

TEST(BehindRobotTest, ExampleAmr2BehindOnLastZone)
{
  // AMR1 [1,2,3,4] current=1; AMR2 [7,8,9,1] → overlap 1 → pause AMR2 only.
  std::unordered_map<std::string, std::vector<std::string>> reserved = {
    {"amr1", {"1", "2", "3", "4"}},
    {"amr2", {"7", "8", "9", "1"}},
  };
  const auto behind = select_robots_coming_from_behind("1", reserved);
  ASSERT_EQ(behind.size(), 1u);
  EXPECT_EQ(behind[0], "amr2");
}

TEST(BehindRobotTest, SymmetricAmr1Behind)
{
  std::unordered_map<std::string, std::vector<std::string>> reserved = {
    {"amr1", {"7", "8", "9", "1"}},
    {"amr2", {"1", "2", "3", "4"}},
  };
  const auto behind = select_robots_coming_from_behind("1", reserved);
  ASSERT_EQ(behind.size(), 1u);
  EXPECT_EQ(behind[0], "amr1");
}

TEST(BehindRobotTest, MidPathOverlap)
{
  // Overlap zone 2: A has it as next[0] (idx1), B as current (idx0) → pause A.
  std::unordered_map<std::string, std::vector<std::string>> reserved = {
    {"amr1", {"1", "2", "3", "4"}},
    {"amr2", {"2", "3", "4", "5"}},
  };
  const auto behind = select_robots_coming_from_behind("2", reserved);
  ASSERT_EQ(behind.size(), 1u);
  EXPECT_EQ(behind[0], "amr1");
}

TEST(BehindRobotTest, TieSameIndexNoBehind)
{
  std::unordered_map<std::string, std::vector<std::string>> reserved = {
    {"amr1", {"1", "2", "3", "4"}},
    {"amr2", {"1", "5", "6", "7"}},
  };
  const auto behind = select_robots_coming_from_behind("1", reserved);
  EXPECT_TRUE(behind.empty());
}

TEST(BehindRobotTest, ThreeRobotsPauseAllButMostAdvanced)
{
  std::unordered_map<std::string, std::vector<std::string>> reserved = {
    {"amr1", {"1", "2", "3", "4"}},       // idx 0
    {"amr2", {"9", "1", "2", "3"}},       // idx 1
    {"amr3", {"8", "9", "7", "1"}},       // idx 3
  };
  const auto behind = select_robots_coming_from_behind("1", reserved);
  ASSERT_EQ(behind.size(), 2u);
  EXPECT_EQ(behind[0], "amr2");
  EXPECT_EQ(behind[1], "amr3");
}

TEST(BehindRobotTest, ReservedZoneIndex)
{
  EXPECT_EQ(reserved_zone_index({"1", "2", "3"}, "1").value(), 0u);
  EXPECT_EQ(reserved_zone_index({"1", "2", "3"}, "3").value(), 2u);
  EXPECT_FALSE(reserved_zone_index({"1", "2"}, "9").has_value());
}
