// Standalone /nav_graphs1 publisher: subscribes to zone status, publishes filtered graph.
#include <algorithm>  // std::sort for readable zone lists in logs
#include <chrono>     // wall timer period and FunctionTimer
#include <memory>     // std::make_shared for the node
#include <mutex>      // std::mutex, std::lock_guard for shared state
#include <sstream>    // std::ostringstream for log formatting
#include <string>
#include <unordered_map>  // latest RobotZoneStatus per robot
#include <unordered_set>  // active zone union and publish dedup
#include <vector>

#include "monitor/msg/robot_zone_status.hpp"
#include "monitor/nav_graph_store.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rmf_building_map_msgs/msg/graph.hpp"

namespace monitor
{
namespace
{

// Log elapsed wall time when this object goes out of scope (covers all return paths).
class FunctionTimer
{
public:
  explicit FunctionTimer(const char * function_name)
  : function_name_(function_name),
    start_(std::chrono::steady_clock::now())
  {
  }

  ~FunctionTimer()
  {
    const auto end = std::chrono::steady_clock::now();
    const double elapsed_ms =
      std::chrono::duration<double, std::milli>(end - start_).count();
    RCLCPP_DEBUG(
      rclcpp::get_logger("nav_graphs1_zone_publisher"),
      "%s execution time: %.3f ms",
      function_name_,
      elapsed_ms);
  }

private:
  const char * function_name_;
  std::chrono::steady_clock::time_point start_;
};

}  // namespace

/// Subscribes to /robot_zone_status (current + next 3 zones) and publishes a
/// filtered nav graph on /nav_graphs1 containing only those zones' waypoints
/// and connecting lanes — same contract as the legacy nav_graphs1 visualizer,
/// extended for three lookahead zones via topic data sharing.
class NavGraphs1ZonePublisherNode : public rclcpp::Node
{
public:
  NavGraphs1ZonePublisherNode()
  : Node("nav_graphs1_zone_publisher")
  {
    // Declare ROS parameters (overridable from launch file).
    declare_parameter<std::string>(
      "source_yaml_path",
      "install/execution/share/execution/config/nav_graphs/0.yaml");
    declare_parameter<std::string>("level", "L1");
    declare_parameter<std::string>("graph_name", "amr_fleet");
    declare_parameter<std::string>("robot_zone_status_topic", "/robot_zone_status");
    declare_parameter<std::string>("nav_graphs1_topic", "/nav_graphs1");
    declare_parameter<double>("publish_rate_hz", 1.0);
    declare_parameter<bool>("require_active_zones", true);

    // Read parameter values used for map load and publishing.
    const std::string yaml_path = get_parameter("source_yaml_path").as_string();
    const std::string level = get_parameter("level").as_string();
    graph_name_ = get_parameter("graph_name").as_string();
    require_active_zones_ = get_parameter("require_active_zones").as_bool();

    // Load nav graph once; without a map the timer will no-op.
    if (!store_.load_map(yaml_path, level)) {
      RCLCPP_ERROR(
        get_logger(),
        "Failed to load nav graph from '%s' (level=%s). /nav_graphs1 disabled.",
        yaml_path.c_str(), level.c_str());
    } else {
      RCLCPP_INFO(
        get_logger(),
        "Nav graph loaded from '%s' (level=%s): vertices=%zu lanes=%zu zones=%zu",
        yaml_path.c_str(), level.c_str(),
        store_.vertices().size(), store_.lanes().size(), store_.zones().size());
    }

    const std::string zone_topic = get_parameter("robot_zone_status_topic").as_string();
    const std::string graph_topic = get_parameter("nav_graphs1_topic").as_string();

    // Late-joining RViz subscribers receive the last published graph.
    auto graph_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    graph_pub_ = create_publisher<rmf_building_map_msgs::msg::Graph>(graph_topic, graph_qos);

    // Cache the latest zone snapshot per robot from zone_monitor.
    zone_sub_ = create_subscription<monitor::msg::RobotZoneStatus>(
      zone_topic,
      rclcpp::QoS(rclcpp::KeepLast(50)),
      std::bind(&NavGraphs1ZonePublisherNode::on_robot_zone_status, this, std::placeholders::_1));

    // Periodic check whether active zones changed enough to republish.
    const double rate_hz = std::max(get_parameter("publish_rate_hz").as_double(), 0.1);
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate_hz),
      std::bind(&NavGraphs1ZonePublisherNode::on_publish_timer, this));

    RCLCPP_INFO(
      get_logger(),
      "nav_graphs1 zone publisher ready (sub: %s | pub: %s @ %.1f Hz)",
      zone_topic.c_str(), graph_topic.c_str(), rate_hz);
  }

private:
  // Store incoming zone status; mark dirty so the timer reconsider publishing.
  void on_robot_zone_status(const monitor::msg::RobotZoneStatus::SharedPtr msg)
  {
    const FunctionTimer timer("on_robot_zone_status");

    if (!msg || msg->amr_id.empty()) {
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    latest_by_robot_[msg->amr_id] = *msg;
    dirty_ = true;
  }

  // Build the union of current + next zones across all cached robots.
  std::unordered_set<std::string> collect_active_zones_locked() const
  {
    std::unordered_set<std::string> zones;
    for (const auto & [robot_id, status] : latest_by_robot_) {
      (void)robot_id;
      if (!status.current_zone.empty()) {
        zones.insert(status.current_zone);
      }
      // next_zone already carries up to the next 3 zones from zone_monitor.
      for (const auto & z : status.next_zone) {
        if (!z.empty()) {
          zones.insert(z);
        }
      }
    }
    return zones;
  }

  // Timer callback: publish filtered graph when zones change or dirty flag is set.
  void on_publish_timer()
  {
    const FunctionTimer timer("on_publish_timer");

    std::lock_guard<std::mutex> lock(mutex_);
    if (!store_.loaded()) {
      return;
    }

    const auto active_zones = collect_active_zones_locked();
    if (active_zones.empty()) {
      if (require_active_zones_) {
        // Log once when transitioning from active zones to none.
        if (dirty_ || !last_active_zones_.empty()) {
          last_active_zones_.clear();
          dirty_ = false;
          RCLCPP_INFO(get_logger(), "No active zones; skipping /nav_graphs1 publish");
        }
        return;
      }
    }

    // Skip rebuild when zone set unchanged and no new status arrived.
    if (!dirty_ && active_zones == last_active_zones_) {
      return;
    }

    const auto graph = store_.build_filtered_graph(active_zones, graph_name_);
    if (graph.vertices.empty()) {
      RCLCPP_WARN(
        get_logger(),
        "Filtered graph empty for %zu active zone(s); not publishing",
        active_zones.size());
      dirty_ = false;
      return;
    }

    graph_pub_->publish(graph);
    last_active_zones_ = active_zones;
    dirty_ = false;

    // Format sorted zone IDs for the publish log line.
    std::vector<std::string> sorted(active_zones.begin(), active_zones.end());
    std::sort(sorted.begin(), sorted.end());
    std::ostringstream ss;
    for (std::size_t i = 0; i < sorted.size(); ++i) {
      if (i > 0) {
        ss << ", ";
      }
      ss << sorted[i];
    }
    RCLCPP_INFO(
      get_logger(),
      "Published /nav_graphs1 for zones [%s] (vertices=%zu, edges=%zu)",
      ss.str().c_str(), graph.vertices.size(), graph.edges.size());
  }

  NavGraphStore store_;
  std::string graph_name_;
  bool require_active_zones_{true};
  bool dirty_{false};
  std::mutex mutex_;
  std::unordered_map<std::string, monitor::msg::RobotZoneStatus> latest_by_robot_;
  std::unordered_set<std::string> last_active_zones_;

  rclcpp::Publisher<rmf_building_map_msgs::msg::Graph>::SharedPtr graph_pub_;
  rclcpp::Subscription<monitor::msg::RobotZoneStatus>::SharedPtr zone_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace monitor

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<monitor::NavGraphs1ZonePublisherNode>());
  rclcpp::shutdown();
  return 0;
}
