#include <gtest/gtest.h>

#include <memory>
#include <vector>

#include "monitor/monitored_task_manager.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{

class MonitoredTaskManagerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("monitored_task_manager_test");
    pub_ = node_->create_publisher<monitor::msg::MonitoredTaskDetails>(
      "/test_monitored_task_details", 10);
    manager_ = std::make_unique<monitor::MonitoredTaskManager>(
      "amr_fleet", pub_, node_->get_logger());
  }

  void TearDown() override
  {
    manager_.reset();
    pub_.reset();
    node_.reset();
    rclcpp::shutdown();
  }

  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Publisher<monitor::msg::MonitoredTaskDetails>::SharedPtr pub_;
  std::unique_ptr<monitor::MonitoredTaskManager> manager_;
};

}  // namespace

TEST_F(MonitoredTaskManagerTest, StatusForTopicTrimsWhitespace)
{
  EXPECT_EQ(monitor::MonitoredTaskManager::status_for_topic("  underway  "), "underway");
}

TEST_F(MonitoredTaskManagerTest, PriorityBinaryHighLow)
{
  const nlohmann::json booking_high = {
    {"priority", {{"type", "binary"}, {"value", 1}}}
  };
  const nlohmann::json booking_low = {
    {"priority", {{"type", "binary"}, {"value", 0}}}
  };

  EXPECT_EQ(manager_->priority_for_topic("task-a", booking_high), "high");
  EXPECT_EQ(manager_->priority_for_topic("task-b", booking_low), "low");
}

TEST_F(MonitoredTaskManagerTest, HandlePublishRuleUnderwayThenCompleted)
{
  manager_->update_task_cache(
    "patrol.dispatch-0",
    nlohmann::json{
      {"task_id", "patrol.dispatch-0"},
      {"amr_id", "amr1"},
      {"category", "Patrol"},
      {"priority", "low"},
      {"phase", "Patrol"},
      {"planned_path", nlohmann::json::array({"1.0,2.0,0.0"})},
    });

  const auto underway = manager_->handle_publish_rule("patrol.dispatch-0", "underway");
  EXPECT_TRUE(underway.published);
  EXPECT_TRUE(underway.should_flush_pending);
  EXPECT_FALSE(underway.reached_final);
  EXPECT_TRUE(manager_->has_started("patrol.dispatch-0"));

  const auto completed = manager_->handle_publish_rule("patrol.dispatch-0", "completed");
  EXPECT_TRUE(completed.published);
  EXPECT_TRUE(completed.reached_final);
  EXPECT_FALSE(manager_->has_started("patrol.dispatch-0"));
}

TEST_F(MonitoredTaskManagerTest, ResolveTaskIdPrefersUnderwayTask)
{
  const std::unordered_map<std::string, std::string> underway{{"amr1", "patrol.dispatch-1"}};
  const std::unordered_map<std::string, std::string> coords{{"amr1", "patrol.dispatch-0"}};

  EXPECT_EQ(
    monitor::MonitoredTaskManager::resolve_task_id(
      "amr1", "other-task", underway, coords),
    "patrol.dispatch-1");
}

TEST_F(MonitoredTaskManagerTest, PlannedPathAndLoadStatusRoundTrip)
{
  manager_->set_planned_path("amr2", {"1.0,2.0,0.0", "3.0,4.0,0.0"});
  manager_->set_load_status("amr2", "loaded");

  EXPECT_EQ(manager_->get_planned_path("amr2").size(), 2U);
  EXPECT_EQ(manager_->get_load_status("amr2"), "loaded");
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
