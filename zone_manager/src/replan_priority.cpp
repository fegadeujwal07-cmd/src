#include "zone_manager/replan_priority.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>
#include <sstream>

#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"

namespace zone_manager
{
namespace
{

std::string to_lower(std::string s)
{
  std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) {
      return static_cast<char>(std::tolower(c));
    });
  return s;
}

rclcpp::Logger priority_logger()
{
  return rclcpp::get_logger("zone_manager.replan_priority");
}

std::string eta_label(const std::optional<double> & eta)
{
  if (!eta.has_value()) {
    return "unknown";
  }
  std::ostringstream oss;
  oss << *eta << " min";
  return oss.str();
}

}  // namespace

std::optional<double> parse_eta_minutes(const std::string & eta)
{
  if (eta.empty()) {
    return std::nullopt;
  }
  std::string s = eta;
  // Strip trailing " min" / "min" if present.
  const auto lower = to_lower(s);
  const auto min_pos = lower.find("min");
  if (min_pos != std::string::npos) {
    s = s.substr(0, min_pos);
  }
  // Trim spaces.
  while (!s.empty() && std::isspace(static_cast<unsigned char>(s.back()))) {
    s.pop_back();
  }
  while (!s.empty() && std::isspace(static_cast<unsigned char>(s.front()))) {
    s.erase(s.begin());
  }
  if (s.empty()) {
    return std::nullopt;
  }
  try {
    return std::stod(s);
  } catch (...) {
    return std::nullopt;
  }
}

int task_priority_rank(const std::string & priority)
{
  const auto p = to_lower(priority);
  if (p.empty()) {
    return 0;
  }
  if (p == "high" || p == "1" || p == "true") {
    return 2;
  }
  if (p == "low" || p == "0" || p == "false") {
    return 1;
  }
  // Any other non-empty booking priority still beats "no priority".
  return 2;
}

bool is_loaded_status(int16_t fleet_load_status)
{
  // Single reference: /fleet_states load_status only.
  // 4 = loaded, 2 = unloaded (empty). All other values → unloaded.
  return fleet_load_status == 4;
}

std::string select_lowest_priority_robot(const std::vector<RobotPriorityFacts> & robots)
{
  if (robots.empty()) {
    return {};
  }

  RCLCPP_INFO(
    priority_logger(),
    "[Opposite] Conflict decision ranking uses operational-mode gate upstream, "
    "then priority, load status, ETA, and FIFO among candidates:");
  for (const auto & r : robots) {
    RCLCPP_INFO(
      priority_logger(),
      "[Opposite]   candidate=%s priority='%s'(rank=%d) loaded=%s eta=%s",
      r.robot_id.c_str(),
      r.task_priority.c_str(),
      task_priority_rank(r.task_priority),
      r.loaded ? "true" : "false",
      eta_label(r.eta_minutes).c_str());
  }

  const auto * worst = &robots.front();
  std::string deciding_factor = "initial_candidate";
  for (std::size_t i = 1; i < robots.size(); ++i) {
    const auto & a = *worst;
    const auto & b = robots[i];

    const int pa = task_priority_rank(a.task_priority);
    const int pb = task_priority_rank(b.task_priority);
    if (pb != pa) {
      if (pb < pa) {
        worst = &b;
        deciding_factor = "task_priority";
        RCLCPP_INFO(
          priority_logger(),
          "[Opposite] Ranking: %s loses to %s on task priority (rank %d < %d)",
          a.robot_id.c_str(), b.robot_id.c_str(), pb, pa);
      } else {
        RCLCPP_INFO(
          priority_logger(),
          "[Opposite] Ranking: %s keeps lead over %s on task priority (rank %d > %d)",
          a.robot_id.c_str(), b.robot_id.c_str(), pa, pb);
      }
      continue;
    }

    const int la = a.loaded ? 1 : 0;
    const int lb = b.loaded ? 1 : 0;
    if (lb != la) {
      if (lb < la) {
        worst = &b;
        deciding_factor = "load_status";
        RCLCPP_INFO(
          priority_logger(),
          "[Opposite] Ranking: %s loses to %s on load status (unloaded preferred for replan)",
          a.robot_id.c_str(), b.robot_id.c_str());
      } else {
        RCLCPP_INFO(
          priority_logger(),
          "[Opposite] Ranking: %s keeps lead over %s on load status (loaded kept)",
          a.robot_id.c_str(), b.robot_id.c_str());
      }
      continue;
    }

    // Smaller ETA = higher keep priority. Missing ETA = least urgent to keep.
    const double ea = a.eta_minutes.value_or(std::numeric_limits<double>::infinity());
    const double eb = b.eta_minutes.value_or(std::numeric_limits<double>::infinity());
    if (eb != ea) {
      // Larger ETA (or inf) → lower priority → candidate for replan.
      if (eb > ea) {
        worst = &b;
        deciding_factor = "ETA";
        RCLCPP_INFO(
          priority_logger(),
          "[Opposite] Ranking: %s loses to %s on ETA (%s > %s → replan larger/unknown ETA)",
          a.robot_id.c_str(), b.robot_id.c_str(),
          eta_label(b.eta_minutes).c_str(), eta_label(a.eta_minutes).c_str());
      } else {
        RCLCPP_INFO(
          priority_logger(),
          "[Opposite] Ranking: %s keeps lead over %s on ETA",
          a.robot_id.c_str(), b.robot_id.c_str());
      }
      continue;
    }

    // FIFO: earlier first_request → gets replan (lower keep priority).
    if (b.first_request_time < a.first_request_time) {
      worst = &b;
      deciding_factor = "FIFO";
      RCLCPP_INFO(
        priority_logger(),
        "[Opposite] Ranking: %s loses to %s on FIFO (earlier first-request is replan candidate)",
        a.robot_id.c_str(), b.robot_id.c_str());
    } else {
      RCLCPP_INFO(
        priority_logger(),
        "[Opposite] Ranking: %s keeps lead over %s on FIFO",
        a.robot_id.c_str(), b.robot_id.c_str());
    }
  }

  RCLCPP_INFO(
    priority_logger(),
    "[Opposite] Selected AMR %s for replan (deciding_factor=%s among priority/load/ETA/FIFO)",
    worst->robot_id.c_str(), deciding_factor.c_str());

  return worst->robot_id;
}

}  // namespace zone_manager
