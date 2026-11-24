#ifndef ZONE_MANAGER__ZONE_MANAGER_NODE_HPP_
#define ZONE_MANAGER__ZONE_MANAGER_NODE_HPP_

#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "monitor/msg/monitored_task_details.hpp"
#include "monitor/msg/robot_zone_status.hpp"
#include "monitor/msg/safety_field_conflict.hpp"
#include "monitor/msg/zone_catalog.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rmf_fleet_msgs/msg/fleet_state.hpp"
#include "zone_manager/conflict_decision.hpp"
#include "zone_manager/conflict_detector.hpp"
#include "zone_manager/behind_robot.hpp"
#include "zone_manager/msg/conflict.hpp"
#include "zone_manager/msg/replan_candidate.hpp"
#include "zone_manager/post_replan.hpp"
#include "zone_manager/replan_priority.hpp"
#include "zone_manager/world_model.hpp"

namespace zone_manager
{

/// Zone conflict detection with direction/topology-aware pause selection.
/// Opposite conflicts also publish /replan_candidate; after a new plan arrives
/// for the candidate, re-analyze without waiting for an occupancy edge.
class ZoneManagerNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit ZoneManagerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State & state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State & state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  struct RobotRuntimeInfo
  {
    std::string task_priority;
    std::string task_load_status;
    int16_t fleet_load_status{0};
    uint32_t fleet_mode{0};
    bool mode_known{false};
    /// VDA5050 operating mode from /fleet_states. -1 = unknown, 0 = unhealthy, 1 = healthy.
    int16_t operating_mode{-1};
    std::optional<double> eta_minutes;
    std::chrono::steady_clock::time_point first_request_time{};
    bool has_first_request{false};
  };

  /// Messages to publish after post-replan re-analysis (built under mutex).
  struct PostReplanPublish
  {
    std::string zone_id;
    std::vector<std::string> safety_start_pause;
    std::vector<std::string> selective_stop_pause;
    ConflictDecision decision;
    bool still_opposite{false};
  };

  void on_fleet_state(const rmf_fleet_msgs::msg::FleetState::SharedPtr msg);
  void on_robot_zone_status(const monitor::msg::RobotZoneStatus::SharedPtr msg);
  void on_zone_catalog(const monitor::msg::ZoneCatalog::SharedPtr msg);
  void on_monitored_task(const monitor::msg::MonitoredTaskDetails::SharedPtr msg);

  /// Proximity overlap reported by monitor's safety_field_node, which watches
  /// MEASURED positions and so covers geometry no zone describes — notably two
  /// robots closing side-to-side on adjacent lanes.
  ///
  /// Routed through here rather than straight to the adapter so that this node
  /// stays the single pause authority: the robot's pause is a latch, not a
  /// counter, so a second publisher would resume a robot this node still
  /// believes it is holding. Reusing pause_refcount_by_robot_ keeps the two
  /// sources composed correctly.
  ///
  /// Deliberately does NOT go through publish_conflicts()/decide_for_zone(),
  /// so no /replan_candidate can ever be raised from a safety overlap.
  void on_safety_field_conflict(
    const monitor::msg::SafetyFieldConflict::SharedPtr msg);
  void on_timer();
  void publish_conflicts(const std::vector<ConflictEvent> & events);

  std::vector<std::string> note_conflict_detected(
    const std::string & zone_id,
    const std::vector<std::string> & robot_ids);

  std::vector<std::string> note_conflict_resolved(const std::string & zone_id);

  /// Hold robots for a zone without double-bumping robots already in that zone set.
  void ensure_robots_held_for_zone(
    const std::string & zone_id,
    const std::vector<std::string> & robot_ids);

  /// Release specific robots from a zone's pause set; return those whose refcount hit 0.
  std::vector<std::string> release_robots_from_zone(
    const std::string & zone_id,
    const std::vector<std::string> & robot_ids);

  bool is_robot_held_for_conflict(const std::string & robot_id) const;

  ConflictDecision decide_for_zone(
    const std::string & zone_id,
    const std::vector<std::string> & robot_ids) const;

  void publish_replan_candidate(
    const std::string & zone_id,
    const ConflictDecision & decision,
    const std::vector<std::string> & conflicting_robots);

  void arm_pending_replan_watch(
    const std::string & zone_id,
    const std::string & candidate_robot_id,
    const std::vector<std::string> & conflicting_robots);

  PlanSnapshot snapshot_from_world(const std::string & robot_id) const;
  static PlanSnapshot snapshot_from_monitored(
    const monitor::msg::MonitoredTaskDetails & msg);

  /// Mutex must be held. Builds publish payload; clears the watch.
  PostReplanPublish handle_post_replan_reanalysis(PendingReplanWatch watch);

  /// Mutex must be held. Fires the post-replan re-analysis for `robot_id` when
  /// the zone list that just arrived differs from the one captured at arm time,
  /// and returns the resulting publish payloads. Empty when nothing changed, so
  /// the watch stays armed until it does or until it times out.
  ///
  /// Driven from on_robot_zone_status() rather than on_monitored_task():
  /// RobotZoneStatus is the only input whose zones are derived from the robot's
  /// planned path, so a changed sequence here proves the new route has actually
  /// been translated into zones. MonitoredTaskDetails carries a path and no
  /// zones at all, and re-deciding on its arrival meant re-deciding against the
  /// PRE-replan zone list.
  std::vector<PostReplanPublish> take_replan_watches_if_zones_changed(
    const std::string & robot_id,
    const std::vector<std::string> & incoming_sequence,
    const std::vector<std::string> & incoming_next);

  void publish_conflict_action(
    const std::string & zone_id,
    const std::string & action,
    const std::vector<std::string> & robot_ids);

  void flush_post_replan_publish(const PostReplanPublish & out);

  void check_pending_replan_timeouts(std::chrono::steady_clock::time_point now);

  RobotRuntimeInfo & ensure_robot_info(const std::string & robot_id);

  WorldModel world_;
  ConflictDetector detector_;
  std::mutex mutex_;
  std::unordered_map<std::string, std::chrono::steady_clock::time_point> last_zone_update_;
  std::unordered_map<std::string, RobotRuntimeInfo> robot_info_;

  std::unordered_map<std::string, std::unordered_set<std::string>> robots_by_conflict_zone_;
  std::unordered_map<std::string, int> pause_refcount_by_robot_;

  /// robot -> the zone its safety hold was recorded under.
  ///
  /// The hold is keyed by the zone the robot occupied when the overlap was
  /// detected. If it drifts to another zone before the overlap clears, the
  /// release must still target the key the hold was taken under, or
  /// release_robots_from_zone() finds no entry, never decrements the refcount,
  /// and the robot stays paused for good.
  std::unordered_map<std::string, std::string> safety_hold_zone_by_robot_;

  /// Keyed by candidate robot id; value holds one watch per conflict zone.
  /// Lookup is O(1); per-candidate zone list is tiny (typically 1–2).
  std::unordered_map<std::string, std::vector<PendingReplanWatch>>
    pending_replan_by_candidate_;

  double process_period_sec_{0.04};
  double stale_timeout_sec_{5.0};
  double replan_plan_timeout_sec_{3.0};
  std::string level_{"L1"};

  rclcpp_lifecycle::LifecyclePublisher<zone_manager::msg::Conflict>::SharedPtr conflict_pub_;
  rclcpp_lifecycle::LifecyclePublisher<zone_manager::msg::ReplanCandidate>::SharedPtr
    replan_candidate_pub_;
  rclcpp::Subscription<rmf_fleet_msgs::msg::FleetState>::SharedPtr fleet_sub_;
  rclcpp::Subscription<monitor::msg::RobotZoneStatus>::SharedPtr zone_status_sub_;
  rclcpp::Subscription<monitor::msg::ZoneCatalog>::SharedPtr catalog_sub_;
  rclcpp::Subscription<monitor::msg::MonitoredTaskDetails>::SharedPtr task_sub_;
  rclcpp::Subscription<monitor::msg::SafetyFieldConflict>::SharedPtr
    safety_field_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace zone_manager

#endif  // ZONE_MANAGER__ZONE_MANAGER_NODE_HPP_
