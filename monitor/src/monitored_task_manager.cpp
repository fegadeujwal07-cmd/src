#include "monitor/monitored_task_manager.hpp"

#include <algorithm>
#include <cctype>

namespace monitor
{
namespace
{

/// Trim leading/trailing whitespace (matches Python str.strip()).
std::string trim_copy(const std::string & input)
{
  const auto start = input.find_first_not_of(" \t\r\n");
  if (start == std::string::npos) {
    return "";
  }
  const auto end = input.find_last_not_of(" \t\r\n");
  return input.substr(start, end - start + 1);
}

/// Convert JSON value to int using Python-like truthiness for "value or 0".
int json_to_int_or_zero(const nlohmann::json & value)
{
  if (value.is_null()) {
    return 0;
  }
  if (value.is_number_integer()) {
    return value.get<int>();
  }
  if (value.is_number_float()) {
    return static_cast<int>(value.get<double>());
  }
  if (value.is_string()) {
    const std::string text = value.get<std::string>();
    if (text.empty()) {
      return 0;
    }
    try {
      return std::stoi(text);
    } catch (...) {
      return 0;
    }
  }
  return 0;
}

/// Lowercase copy of a string (matches Python status_key = status_value.lower()).
std::string to_lower_copy(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
      return static_cast<char>(std::tolower(c));
    });
  return value;
}

}  // namespace

MonitoredTaskManager::MonitoredTaskManager(
  const std::string & fleet_name,
  rclcpp::Publisher<monitor::msg::MonitoredTaskDetails>::SharedPtr monitored_task_pub,
  rclcpp::Logger logger)
: fleet_name_(fleet_name),
  pub_(std::move(monitored_task_pub)),
  logger_(logger)
{
}

bool MonitoredTaskManager::has_started(const std::string & task_id) const
{
  return started_tasks_.count(task_id) > 0;
}

void MonitoredTaskManager::set_planned_path(
  const std::string & robot_name,
  const std::vector<std::string> & planned_path)
{
  if (!robot_name.empty()) {
    planned_path_by_robot_[robot_name] = planned_path.empty() ?
      std::vector<std::string>{} : planned_path;
  }
}

std::vector<std::string> MonitoredTaskManager::get_planned_path(
  const std::string & robot_name) const
{
  if (robot_name.empty()) {
    return {};
  }
  const auto it = planned_path_by_robot_.find(robot_name);
  return (it != planned_path_by_robot_.end()) ? it->second : std::vector<std::string>{};
}

void MonitoredTaskManager::set_load_status(
  const std::string & robot_name,
  const std::string & load_status)
{
  if (!robot_name.empty() && !load_status.empty()) {
    load_status_by_robot_[robot_name] = load_status;
  }
}

std::string MonitoredTaskManager::get_load_status(const std::string & robot_name) const
{
  if (robot_name.empty()) {
    return "";
  }
  const auto it = load_status_by_robot_.find(robot_name);
  return (it != load_status_by_robot_.end()) ? it->second : "";
}

std::string MonitoredTaskManager::status_for_topic(const std::string & status)
{
  return trim_copy(status);
}

std::string MonitoredTaskManager::priority_for_topic(
  const std::string & task_id,
  const nlohmann::json & booking)
{
  nlohmann::json booking_obj = booking.is_object() ? booking : nlohmann::json::object();
  nlohmann::json priority_value = nullptr;

  const auto priority_it = booking_obj.find("priority");
  if (priority_it != booking_obj.end() && !priority_it->is_null()) {
    priority_value = *priority_it;
    priority_cache_[task_id] = priority_value;
  } else {
    const auto cached_it = priority_cache_.find(task_id);
    if (cached_it != priority_cache_.end()) {
      priority_value = cached_it->second;
    }
  }

  if (priority_value.is_null()) {
    return "";
  }

  if (priority_value.is_object()) {
    const auto type_it = priority_value.find("type");
    if (type_it != priority_value.end() && type_it->is_string() && type_it->get<std::string>() == "binary") {
      const nlohmann::json value_node =
        priority_value.contains("value") ? priority_value["value"] : nlohmann::json(nullptr);
      return json_to_int_or_zero(value_node) > 0 ? "high" : "low";
    }
    return priority_value.dump();
  }

  if (priority_value.is_string()) {
    return priority_value.get<std::string>();
  }
  if (priority_value.is_number_integer()) {
    return std::to_string(priority_value.get<int>());
  }
  if (priority_value.is_number_float()) {
    return std::to_string(priority_value.get<double>());
  }
  if (priority_value.is_boolean()) {
    return priority_value.get<bool>() ? "true" : "false";
  }
  return priority_value.dump();
}

MonitoredTaskManager::TaskCacheEntry MonitoredTaskManager::make_default_entry(
  const std::string & task_id,
  const std::string & fleet_name)
{
  return nlohmann::json{
    {"task_id", task_id},
    {"amr_id", ""},
    {"fleet_name", fleet_name},
    {"category", ""},
    {"priority", ""},
    {"status", ""},
    {"load_status", ""},
    {"phase", ""},
    {"planned_path", nlohmann::json::array()},
  };
}

void MonitoredTaskManager::update_task_cache(
  const std::string & task_id,
  const nlohmann::json & updates)
{
  if (task_id.empty()) {
    return;
  }

  if (task_cache_.find(task_id) == task_cache_.end()) {
    task_cache_[task_id] = make_default_entry(task_id, fleet_name_);
  }

  if (!updates.is_object()) {
    return;
  }

  TaskCacheEntry & entry = task_cache_.at(task_id);
  for (const auto & [key, value] : updates.items()) {
    if (value.is_null()) {
      continue;
    }
    entry[key] = value;
  }
}

std::string MonitoredTaskManager::json_string_or_empty(
  const nlohmann::json & value,
  const std::string & key)
{
  const auto it = value.find(key);
  if (it == value.end() || it->is_null()) {
    return "";
  }
  if (it->is_string()) {
    return it->get<std::string>();
  }
  return it->dump();
}

std::vector<std::string> MonitoredTaskManager::json_string_list_or_empty(
  const nlohmann::json & value,
  const std::string & key)
{
  std::vector<std::string> result;
  const auto it = value.find(key);
  if (it == value.end() || !it->is_array()) {
    return result;
  }
  result.reserve(it->size());
  for (const auto & item : *it) {
    if (item.is_string()) {
      result.push_back(item.get<std::string>());
    } else {
      result.push_back(item.dump());
    }
  }
  return result;
}

MonitoredTaskManager::Signature MonitoredTaskManager::build_signature(
  const TaskCacheEntry & entry,
  const std::string & default_fleet_name)
{
  Signature signature;
  signature.reserve(8);
  signature.push_back(json_string_or_empty(entry, "task_id"));
  signature.push_back(json_string_or_empty(entry, "amr_id"));

  const std::string fleet = json_string_or_empty(entry, "fleet_name");
  signature.push_back(fleet.empty() ? default_fleet_name : fleet);

  signature.push_back(json_string_or_empty(entry, "category"));
  signature.push_back(json_string_or_empty(entry, "priority"));
  signature.push_back(json_string_or_empty(entry, "status"));
  signature.push_back(json_string_or_empty(entry, "load_status"));
  signature.push_back(json_string_or_empty(entry, "phase"));

  const std::vector<std::string> planned_path = json_string_list_or_empty(entry, "planned_path");
  signature.insert(signature.end(), planned_path.begin(), planned_path.end());
  return signature;
}

bool MonitoredTaskManager::publish_task(const std::string & task_id)
{
  const auto entry_it = task_cache_.find(task_id);
  if (entry_it == task_cache_.end()) {
    return false;
  }

  const TaskCacheEntry & entry = entry_it->second;
  const Signature signature = build_signature(entry, fleet_name_);

  const auto sig_it = last_published_signature_.find(task_id);
  if (sig_it != last_published_signature_.end() && sig_it->second == signature) {
    RCLCPP_DEBUG(logger_, "Skipped duplicate MonitoredTaskDetails publish for task '%s'", task_id.c_str());
    return false;
  }

  monitor::msg::MonitoredTaskDetails msg;
  msg.task_id = json_string_or_empty(entry, "task_id");
  msg.amr_id = json_string_or_empty(entry, "amr_id");

  const std::string fleet = json_string_or_empty(entry, "fleet_name");
  msg.fleet_name = fleet.empty() ? fleet_name_ : fleet;

  msg.category = json_string_or_empty(entry, "category");
  msg.priority = json_string_or_empty(entry, "priority");
  msg.status = json_string_or_empty(entry, "status");
  msg.load_status = json_string_or_empty(entry, "load_status");
  msg.phase = json_string_or_empty(entry, "phase");
  msg.planned_path = json_string_list_or_empty(entry, "planned_path");

  pub_->publish(msg);
  last_published_signature_[task_id] = signature;
  RCLCPP_DEBUG(
    logger_,
    "Published MonitoredTaskDetails for task '%s' robot '%s'",
    msg.task_id.c_str(),
    msg.amr_id.c_str());
  return true;
}

void MonitoredTaskManager::clear_task_state(const std::string & task_id)
{
  std::string robot_name;
  const auto entry_it = task_cache_.find(task_id);
  if (entry_it != task_cache_.end()) {
    robot_name = json_string_or_empty(entry_it->second, "amr_id");
    task_cache_.erase(entry_it);
  }

  started_tasks_.erase(task_id);
  observed_status_.erase(task_id);
  priority_cache_.erase(task_id);
  last_published_signature_.erase(task_id);

  if (!robot_name.empty()) {
    planned_path_by_robot_.erase(robot_name);
    load_status_by_robot_.erase(robot_name);
  }
}

MonitoredPublishRuleResult MonitoredTaskManager::handle_publish_rule(
  const std::string & task_id,
  const std::string & new_status)
{
  MonitoredPublishRuleResult result;
  if (task_id.empty()) {
    return result;
  }

  const auto prev_it = observed_status_.find(task_id);
  const bool has_prev_status = prev_it != observed_status_.end();
  const std::string prev_status_key = has_prev_status ? prev_it->second : "";

  const std::string status_value = status_for_topic(new_status);
  const std::string status_key = to_lower_copy(status_value);
  observed_status_[task_id] = status_key;
  update_task_cache(task_id, nlohmann::json{{"status", status_value}});

  result.reached_final =
    (status_key == "completed" || status_key == "canceled" || status_key == "failed");

  if (status_key == "underway") {
    if (prev_status_key != "underway") {
      started_tasks_.insert(task_id);
      result.published = publish_task(task_id);
      result.should_flush_pending = true;
    }
    return result;
  }

  if (
    started_tasks_.count(task_id) > 0 &&
    has_prev_status &&
    prev_status_key != status_key)
  {
    result.published = publish_task(task_id);
    result.should_flush_pending = true;
  }

  if (result.reached_final) {
    clear_task_state(task_id);
  }

  return result;
}

std::string MonitoredTaskManager::resolve_task_id(
  const std::string & robot_name,
  const std::string & candidate_task_id,
  const std::unordered_map<std::string, std::string> & currently_underway_task,
  const std::unordered_map<std::string, std::string> & robot_to_coords_key)
{
  if (!robot_name.empty()) {
    const auto underway_it = currently_underway_task.find(robot_name);
    if (underway_it != currently_underway_task.end() && !underway_it->second.empty()) {
      return underway_it->second;
    }
    const auto coords_it = robot_to_coords_key.find(robot_name);
    if (coords_it != robot_to_coords_key.end() && !coords_it->second.empty()) {
      return coords_it->second;
    }
  }
  return candidate_task_id;
}

}  // namespace monitor
