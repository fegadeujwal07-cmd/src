#include <gtest/gtest.h>

#include "monitor/eta_path_calculator.hpp"

TEST(EtaPathCalculator, StraightPathPositiveTime)
{
  monitor::EtaRobotParams params;
  params.operating_linear_speed_mps = 0.5;
  params.linear_accel_mps2 = 0.5;
  params.linear_decel_mps2 = 0.6;
  const std::vector<std::pair<double, double>> path = {
    {0.0, 0.0}, {10.0, 0.0}};
  const double t = monitor::estimate_path_travel_time_sec(path, params, 0.0);
  EXPECT_GT(t, 10.0);  // more than length/cruise because of accel/stop
  EXPECT_LT(t, 60.0);
}

TEST(EtaPathCalculator, ReachTimesMonotonic)
{
  monitor::EtaRobotParams params;
  const std::vector<std::pair<double, double>> path = {
    {0.0, 0.0}, {2.0, 0.0}, {5.0, 0.0}};
  const auto reach = monitor::path_point_reach_times_sec(path, params, 0.0);
  ASSERT_EQ(reach.size(), path.size());
  EXPECT_DOUBLE_EQ(reach.front(), 0.0);
  EXPECT_GE(reach[1], reach[0]);
  EXPECT_GE(reach[2], reach[1]);
  EXPECT_NEAR(
    reach.back(),
    monitor::estimate_path_travel_time_sec(path, params, 0.0),
    1e-9);
}
