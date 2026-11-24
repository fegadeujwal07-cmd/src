#ifndef MONITOR__MONITORED_TASK_MANAGER_HPP_
#define MONITOR__MONITORED_TASK_MANAGER_HPP_

#include <map>
#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include "monitor/msg/monitored_task_details.hpp"
#include "nlohmann/json.hpp"
#include "rclcpp/rclcpp.hpp"

namespace monitor
{

/// Result flags returned by handle_publish_rule (matches Python dict keys).
struct MonitoredPublishRuleResult
{
  bool published{false};
  bool should_flush_pending{false};
  bool reached_final{false};
};

/// C++ port of monitor/monitor_logic.py::MonitoredTaskManager.
class MonitoredTaskManager
{
public:
  MonitoredTaskManager(
    const std::string & fleet_name,
    rclcpp::Publisher<monitor::msg::MonitoredTaskDetails>::SharedPtr monitored_task_pub,
    rclcpp::Logger logger);

  bool has_started(const std::string & task_id) const;

  void set_planned_path(
    const std::string & robot_name,
    const std::vector<std::string> & planned_path);

  std::vector<std::string> get_planned_path(const std::string & robot_name) const;

  void set_load_status(const std::string & robot_name, const std::string & load_status);

  std::string get_load_status(const std::string & robot_name) const;

  static std::string status_for_topic(const std::string & status);

  std::string priority_for_topic(
    const std::string & task_id,
    const nlohmann::json & booking);

  void update_task_cache(
    const std::string & task_id,
    const nlohmann::json & updates);

  bool publish_task(const std::string & task_id);

  void clear_task_state(const std::string & task_id);

  MonitoredPublishRuleResult handle_publish_rule(
    const std::string & task_id,
    const std::string & new_status);

  static std::string resolve_task_id(
    const std::string & robot_name,
    const std::string & candidate_task_id,
    const std::unordered_map<std::string, std::string> & currently_underway_task,
    const std::unordered_map<std::string, std::string> & robot_to_coords_key);

private:
  using TaskCacheEntry = nlohmann::json;
  using Signature = std::vector<std::string>;

  static std::string json_string_or_empty(const nlohmann::json & value, const std::string & key);
  static std::vector<std::string> json_string_list_or_empty(const nlohmann::json & value, const std::string & key);
  static Signature build_signature(const TaskCacheEntry & entry, const std::string & default_fleet_name);
  static TaskCacheEntry make_default_entry(
    const std::string & task_id,
    const std::string & fleet_name);

  std::string fleet_name_;
  rclcpp::Publisher<monitor::msg::MonitoredTaskDetails>::SharedPtr pub_;
  rclcpp::Logger logger_;

  std::unordered_map<std::string, TaskCacheEntry> task_cache_;
  std::set<std::string> started_tasks_;
  std::unordered_map<std::string, std::string> observed_status_;
  std::unordered_map<std::string, nlohmann::json> priority_cache_;
  std::unordered_map<std::string, std::vector<std::string>> planned_path_by_robot_;
  std::unordered_map<std::string, std::string> load_status_by_robot_;
  std::unordered_map<std::string, Signature> last_published_signature_;
};

}  // namespace monitor

#endif  // MONITOR__MONITORED_TASK_MANAGER_HPP_
