// =============================================================================
// execution_full_control for AMR_1500 Robots - Full Control Integration (C++)
// =============================================================================
// C++ port of execution/execution_full_control.py (Python file is READ-ONLY).
//
// SECTION 00 — Python lines 1–59: docstring + imports (see includes below)
// SECTION 01+ — types, adapter class, and logic added section-by-section
//
// Module purpose (Python lines 3–13):
//   Integrates with the RMF full control fleet adapter by publishing FleetState
//   messages. Enhanced with path planning, delivery task detection, and MQTT-based
//   robot communication.
// =============================================================================

#ifndef EXECUTION__EXECUTION_FULL_CONTROL_HPP_
#define EXECUTION__EXECUTION_FULL_CONTROL_HPP_

// --- Python lines 45–57: stdlib ---
#include <chrono>
#include <cmath>
#include <filesystem>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <regex>
#include <set>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

// --- Python lines 15–19: rclpy ---
#include "rclcpp/rclcpp.hpp"

// --- Python lines 21–28: ROS 2 messages ---
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "monitor/msg/monitored_task_details.hpp"
#include "monitor/msg/robot_zone_status.hpp"
#include "rmf_dispenser_msgs/msg/dispenser_result.hpp"
#include "traffic_manager/msg/base_extension_request.hpp"
#include "rmf_fleet_msgs/msg/fleet_state.hpp"
#include "rmf_fleet_msgs/msg/location.hpp"
#include "rmf_fleet_msgs/msg/mode_request.hpp"
#include "rmf_fleet_msgs/msg/path_request.hpp"
#include "rmf_fleet_msgs/msg/robot_lane_request.hpp"
#include "rmf_fleet_msgs/msg/robot_mode.hpp"
#include "rmf_fleet_msgs/msg/robot_state.hpp"
#include "rmf_ingestor_msgs/msg/ingestor_result.hpp"
#include "rmf_task_msgs/msg/api_request.hpp"
#include "rmf_task_msgs/msg/api_response.hpp"
#include "rmf_task_msgs/msg/dispatch_state.hpp"
#include "rmf_task_msgs/msg/dispatch_states.hpp"
#include "std_msgs/msg/string.hpp"
#include "zone_manager/msg/conflict.hpp"
#include "zone_manager/msg/replan_candidate.hpp"

// --- Python lines 30–32: TF2 ---
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"

// --- Python line 45: yaml (Section 01 NavGraphParser) ---
#include <yaml-cpp/yaml.h>

// --- Python lines 36–40: from execution.mqtt_client import MQTTFleetClient ---
#include "execution/mqtt_client.hpp"

// --- 4D / ASRS drop corridor (pure lane-selection logic) ---
#include "execution/drop_corridor.hpp"

namespace execution
{

// --- Python line 56: typing Dict[str, Any] (use nlohmann::json in Sections 11–12) ---
using JsonDict = std::map<std::string, std::string>;

// --- Python line 42: from monitor.monitor_logic import MonitoredTaskManager ---
struct MonitoredPublishRuleResult
{
  bool published{false};
  bool should_flush_pending{false};
  bool reached_final{false};
};

class MonitoredTaskManager
{
public:
  MonitoredTaskManager(
    const std::string & fleet_name,
    rclcpp::Publisher<monitor::msg::MonitoredTaskDetails>::SharedPtr monitored_task_pub,
    rclcpp::Logger logger);

  bool has_started(const std::string & task_id) const;
  void set_planned_path(const std::string & robot_name, const std::vector<std::string> & planned_path);
  std::vector<std::string> get_planned_path(const std::string & robot_name) const;
  void set_load_status(const std::string & robot_name, const std::string & load_status);
  std::string get_load_status(const std::string & robot_name) const;
  static std::string status_for_topic(const std::string & status);
  std::string priority_for_topic(const std::string & task_id, const AnyMap & booking);
  void update_task_cache(const std::string & task_id, const AnyMap & updates);
  bool publish_task(const std::string & task_id);
  void clear_task_state(const std::string & task_id);
  MonitoredPublishRuleResult handle_publish_rule(const std::string & task_id, const std::string & new_status);
  static std::string resolve_task_id(
    const std::string & robot_name,
    const std::string & candidate_task_id,
    const std::map<std::string, std::string> & currently_underway_task,
    const std::map<std::string, std::string> & robot_to_coords_key);

private:
  std::string fleet_name_;
  rclcpp::Publisher<monitor::msg::MonitoredTaskDetails>::SharedPtr pub_;
  rclcpp::Logger logger_;
  std::map<std::string, AnyMap> task_cache_;
  std::set<std::string> started_tasks_;
  std::map<std::string, std::string> observed_status_;
  std::map<std::string, YAML::Node> priority_cache_;
  std::map<std::string, std::vector<std::string>> planned_path_by_robot_;
  std::map<std::string, std::string> load_status_by_robot_;
  std::map<std::string, std::vector<std::string>> last_published_signature_;
};

struct VdaOrderFingerprint
{
  std::string order_id;
  int order_update_id{0};
  std::string phase;
  std::optional<std::pair<double, double>> last_wp_xy;
  // Hash of every node position in the order, so "same route" is detectable.
  std::size_t path_hash{0};

  // NOTE: order_update_id is deliberately NOT compared.
  //
  // orderUpdateId is derived from RMF's path_revision, and RMF bumps that on
  // every replan attempt even when the resulting route is identical (observed:
  // 7 orders for one patrol leg, byte-identical 7-node path, updateId 31..37).
  // Including it made every one of those look like a new order.
  //
  // What actually identifies an order is the task, the phase and the ROUTE.
  // Comparing path_hash gives the old "one order per phase" guarantee back for
  // an unchanged route, while still publishing when the robot is genuinely
  // rerouted.
  bool operator==(const VdaOrderFingerprint & other) const
  {
    return order_id == other.order_id &&
           phase == other.phase &&
           path_hash == other.path_hash &&
           last_wp_xy == other.last_wp_xy;
  }
};

// =============================================================================
// SECTION 01 — Python lines 60–252: Types + NavGraphParser
// =============================================================================

enum class RobotStatus
{
  Idle,
  Moving,
  Navigating,
  Charging,
  Error,
  Offline,
};

struct Waypoint
{
  std::string id;
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  std::optional<std::string> name;
  bool is_holding_point{false};
  bool is_charger{false};
};

struct Lane
{
  std::string id;
  std::string start_waypoint;
  std::string end_waypoint;
  std::map<std::string, YAML::Node> properties;
};

struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct Twist2D
{
  double linear{0.0};
  double angular{0.0};
};

struct AMRRobot
{
  std::string robot_id;
  Pose2D pose;
  Twist2D twist;
  double battery_percentage{0.0};
  // powerSupply.charging from the VDA5050 state: 1 = charging, 0 = not
  // charging, -1 = unknown (missing / not true-false / state too old).
  int charging_status{-1};
  RobotStatus status{RobotStatus::Idle};
  std::string current_task;
  std::string namespace_;
  std::string fleet_publish_task_id;
};

struct FleetConfig
{
  std::string fleet_name;
  std::map<std::string, AMRRobot> robots;
  std::string nav_graph_path;
  std::string map_name;
};

struct DeliveryTask
{
  std::string task_id;
  std::string pickup_location;
  std::string dropoff_location;
  std::string robot_name;
};

class NavGraphParser
{
public:
  explicit NavGraphParser(const std::string & nav_graph_path);

  std::string nav_graph_path;
  std::map<std::string, Waypoint> waypoints;
  std::map<std::string, Lane> lanes;
  std::string map_name{"L1"};

  // zone id -> lane indices belonging to that zone. Indices are positions in
  // the nav-graph YAML `lanes:` array, which match rmf_traffic::agv::Graph lane
  // indices 1:1 (parse_graph.cpp emits exactly one lane per YAML entry; the map
  // uses no dock/lift/bidirectional-flag features that would insert extras).
  //
  // A lane belongs to a zone if EITHER endpoint is one of the zone's waypoints,
  // i.e. lanes fully inside the zone AND lanes crossing its boundary. Crossing
  // lanes are included so that closing a zone also prevents a robot from
  // driving INTO it, not just within it.
  //
  // NOTE: the zone `edge:` field in the YAML holds boundary VERTEX indices, not
  // lane indices, and is deliberately not used here.
  std::map<std::string, std::vector<std::size_t>> zone_lanes;

  std::optional<Waypoint> get_waypoint_by_name(const std::string & name) const;
  std::optional<Waypoint> get_waypoint_by_id(const std::string & wp_id) const;
  std::vector<Waypoint> plan_path(const std::string & start_name, const std::string & end_name);
  /// Lane indices to close for a zone (empty if the zone is unknown).
  std::vector<std::size_t> lanes_for_zone(const std::string & zone_id) const;

  // Vertex and lane geometry for the 4D / ASRS drop corridor.
  const drop_corridor::Geometry & corridor_geometry() const
  {
    return corridor_geometry_;
  }

private:
  void parse_nav_graph();
  // Indexed exactly like the YAML arrays (= rmf_traffic graph indices).
  drop_corridor::Geometry corridor_geometry_;
};

// =============================================================================
// SECTION 02 — Python lines 273–543: Node init + ROS wiring
// =============================================================================

class RMFIntegratedFleetAdapter : public rclcpp::Node
{
public:
  RMFIntegratedFleetAdapter();

private:
  void load_configuration();
  void initialize_nav_graph_parser();
  std::string resolve_file_path(const std::string & file_path);
  void setup_mqtt_client();
  AMRRobot & register_robot_from_mqtt(const std::string & robot_id, const AnyMap & robot_data);
  bool mqtt_task_confirmed_by_api(const std::string & robot_id, const std::string & task_id) const;
  void update_robot_states(const std::map<std::string, AnyMap> & robots);

  void poll_amr_states_tick();
  // Section 15 — Mode + misc (Python lines 3527–3544, 4206–4297)
  void diagnostic_check();
  // Section 13 — Load + workcells (Python lines 3768–4091)
  void handle_load_status_event(const AnyMap & event);
  // Section 14 — Nav status (Python lines 4000–4036)
  void handle_nav_status_event(const AnyMap & event);
  void publish_fleet_state();
  void publish_base_extension_from_mqtt(const std::map<std::string, AnyMap> & robots);
  void publish_tf_frames(const std::map<std::string, AnyMap> & robots);
  void monitor_task_completion();
  void trigger_task_completion_callback(const std::string & robot_id, const std::string & task_id);
  // Section 08 — Monitored task (Python lines 1919–2167)
  static std::string monitored_load_status_text(int vda_load_status);
  std::string monitored_phase_for_task(const std::string & task_id, const std::string & category) const;
  std::string status_for_monitored_topic(const std::string & status) const;
  std::string priority_for_monitored_topic(const std::string & task_id, const AnyMap & booking) const;
  std::string resolve_monitored_task_id(
    const std::string & robot_name,
    const std::string & candidate_task_id = "") const;
  void update_monitored_task_cache(const std::string & task_id, const AnyMap & updates);
  void publish_monitored_task(const std::string & task_id);
  void handle_monitored_publish_rule(const std::string & task_id, const std::string & new_status);
  void clear_monitored_task_state(const std::string & task_id);
  static std::optional<std::string> extract_place_from_phase_detail(const std::string & detail_text);
  static std::optional<std::string> extract_place_from_labels(
    const YAML::Node & labels, const std::string & key_name);
  void record_task_phase_places(const std::string & task_id, const YAML::Node & task_state);
  void try_send_buffered_path_request(const std::string & robot_name);
  void queue_vda_order_after_monitored(
    const std::string & task_id,
    const std::string & robot_name,
    const std::string & phase,
    const AnyMap & order_payload,
    const std::string & source,
    int phase_index = 0,
    int path_revision = 0);
  void flush_pending_vda_orders(const std::string & task_id);
  std::pair<bool, std::string> can_publish_vda_order_for_phase(
    const std::string & robot_name, const std::string & phase) const;
  // Spin-action injection for pivoting (isolated logic).
  // arrival_theta_rad = heading when standing at the node (path[0].yaw, or
  // incoming leg atan2 for later nodes).
  // travel_theta_rad = outgoing map heading toward the next node (atan2).
  // Outputs yaw = arrival heading, spin_deg = travel heading (cardinal snap).
  std::optional<YAML::Node> compute_spin_action(
    const std::string & robot_name,
    double arrival_theta_rad,
    double travel_theta_rad,
    int node_sequence_id) const;
  // Section 09 — VDA5050 order build (Python lines 1892, 2169–2429)
  AnyMap build_path_request_payload(const rmf_fleet_msgs::msg::PathRequest & msg) const;
  int next_order_header_id(const std::string & robot_name);
  int next_instant_actions_header_id(const std::string & robot_name);
  bool publish_vda5050_instant_action(
    const std::string & robot_name,
    const std::string & action_type,
    const std::string & action_id,
    const std::vector<AnyMap> * action_parameters = nullptr,
    const std::string & action_descriptor = "");
  std::optional<AnyMap> build_vda5050_order_payload(
    const std::string & robot_name,
    const std::string & order_id,
    const std::string & phase,
    const AnyMap & path_request_data);
  std::pair<bool, std::string> official_dispatch_path_matches_booking(
    const std::string & path_dispatch_task_id,
    const std::string & coords_booking_id) const;
  // Section 07 — Coord + task ID helpers (Python lines 1579–1890)
  bool is_dispatch_booking_id(const std::string & s) const;
  static bool is_patrol_dispatch_id(const std::string & task_id);
  // Task-category resolution that does not depend on the task-id text.
  // Fleet dispatch yields "<category>.dispatch-N"; robot-targeted dispatch
  // (rmf_fleet_adapter TaskManager::_handle_direct_request) reuses the API
  // request_id, which is a uuid4 from the api-server. Both must work.
  static bool is_uuid_task_id(const std::string & task_id);
  void remember_task_category(const std::string & task_id, const std::string & category);
  std::string task_category_for(const std::string & task_id) const;
  bool is_patrol_task(const std::string & task_id) const;
  void flush_pending_after_startup();
  std::tuple<bool, std::string, int> patrol_path_matches_selected_places(
    const std::string & task_id,
    const YAML::Node & path_points,
    double current_robot_x,
    double current_robot_y,
    bool skip_first_pose_check = false) const;
  std::tuple<bool, std::string, int> patrol_path_matches_selected_places(
    const std::string & task_id,
    const std::vector<rmf_fleet_msgs::msg::Location> & path_points,
    double current_robot_x,
    double current_robot_y,
    bool skip_first_pose_check = false) const;
  bool should_send_path_request_by_coords(
    const std::string & coords_booking_id,
    double first_wp_x,
    double first_wp_y,
    double last_wp_x,
    double last_wp_y,
    double current_robot_x,
    double current_robot_y) const;
  bool delivery_awaiting_drop_vda_order(const std::string & booking_id) const;
  bool path_last_waypoint_matches_delivery_target(
    const std::string & coords_booking_id,
    double last_wp_x,
    double last_wp_y) const;
  bool coords_match(double x1, double y1, double x2, double y2) const;
  std::optional<AnyMap> build_phase_action_parameter(
    const std::string & phase, const std::string & place) const;
  std::optional<AnyMap> build_patrol_action_parameter(
    const std::string & robot_name, const std::string & place) const;
  void send_pickup_for_underway_task(const std::string & robot_name, const std::string & task_id);
  void send_patrol_for_underway_task(const std::string & robot_name, const std::string & task_id);
  // Section 13 — Load + workcells (Python lines 4038–4091)
  void publish_dispenser_result(const std::string & base_request_id, const std::string & event_id = "");
  void publish_ingestor_result(const std::string & base_request_id, const std::string & event_id = "");
  bool is_robot_at_target(const std::string & robot_id, const AMRRobot & robot);
  double robot_distance_to_nav_goal(const std::string & robot_id, const AMRRobot & robot) const;
  bool is_robot_within_rmf_path_finish_tolerance(
    const std::string & robot_id, const AMRRobot & robot) const;
  bool try_complete_dropoff(
    const std::string & robot_id,
    const std::string & base_request_id,
    const std::string & event_id,
    bool allow_defer = true);
  void clear_delivery_completion_state(
    const std::string & robot_name, const std::string & task_id);
  void finalize_robot_path_on_task_end(
    const std::string & robot_name, const std::string & task_id);
  bool is_task_executing_on_robot(
    const std::string & robot_name, const std::string & task_id) const;
  int advance_waypoint_sequence_if_progressed(const std::string & robot_id, const AMRRobot & robot);
  double calculate_distance(double x1, double y1, double x2, double y2) const;
  std::vector<rmf_fleet_msgs::msg::Location> store_and_filter_waypoints(
    const std::string & robot_id,
    const std::vector<rmf_fleet_msgs::msg::Location> & waypoints,
    double min_distance = 0.5,
    bool filter_visited = true);
  /// Drop passed waypoints so fleet_state.path is upcoming navigation only.
  static std::vector<rmf_fleet_msgs::msg::Location> trim_path_from_robot_pose(
    const std::vector<rmf_fleet_msgs::msg::Location> & path,
    double robot_x,
    double robot_y,
    double reached_m);
  void trim_waypoint_sequence_from_pose(
    std::vector<AnyMap> & sequence,
    const std::string & robot_id,
    double robot_x,
    double robot_y,
    double reached_m);
  /// Collapse passed / behind-pose waypoints (fleet_state.path = upcoming only).
  void trim_waypoint_sequence_to_robot_progress(
    std::vector<AnyMap> & sequence,
    const std::string & robot_id,
    double robot_x,
    double robot_y);
  static void sync_release_flags_on_sequence(
    std::vector<AnyMap> & sequence,
    const std::vector<rmf_fleet_msgs::msg::Location> & full_path,
    double xy_match_m = 0.08);
  static void append_missing_tail_waypoints(
    std::vector<AnyMap> & dst,
    const std::vector<AnyMap> & src,
    double xy_match_m = 0.08);
  static bool path_uses_release_flags(
    const std::vector<rmf_fleet_msgs::msg::Location> & path);
  static std::vector<rmf_fleet_msgs::msg::Location> waypoints_for_robot_order(
    const std::vector<rmf_fleet_msgs::msg::Location> & path);
  /// PathRequest with more traffic_manager release flags than the last VDA order.
  bool is_traffic_horizon_extension(
    const std::string & robot_name,
    const std::string & dispatch_booking_id,
    const rmf_fleet_msgs::msg::PathRequest & msg) const;
  void mark_waypoint_as_visited(const std::string & robot_id, double waypoint_x, double waypoint_y);
  int convert_status_to_mode(RobotStatus status) const;
  std::string robot_state_publish_task_id(const AMRRobot & robot) const;

  bool is_valid_rmf_task(const std::string & task_id);
  static bool is_terminal_task_status(const std::string & status);
  std::string path_request_dispatch_task_id(const rmf_fleet_msgs::msg::PathRequest & msg) const;
  bool clear_stale_local_cancel_status(const std::string & task_id, const std::string & reason);
  std::string resolve_booking_task_id_for_path_request(
    const std::string & robot_name,
    const std::string & path_task_id,
    const std::vector<rmf_fleet_msgs::msg::Location> * path,
    const std::string & dispatch_task_id = "");
  std::string resolve_coords_booking_id(
    const std::string & robot_name,
    const std::string & booking_from_path_resolve) const;
  // Section 10 — Path send/buffer (Python lines 2431–2779)
  void append_pending_path_request(
    const std::string & robot_name,
    const AnyMap & path_request_data,
    const std::string & task_id,
    const std::string & coords_booking_id,
    double first_wp_x,
    double first_wp_y,
    double last_wp_x,
    double last_wp_y);
  void path_request_send_or_buffer(const rmf_fleet_msgs::msg::PathRequest::SharedPtr msg);

  // Section 11 — Task API + dispatch (Python lines 2781–3059, 3546–3746)
  void process_patrol_api_payload(const std::string & request_id, const YAML::Node & payload);
  void process_drop_api_payload(const std::string & request_id, const YAML::Node & payload);
  void process_delivery_api_payload(
    const std::string & request_id,
    const YAML::Node & payload,
    const std::string & assigned_robot_override = "");

  void path_request_callback(const rmf_fleet_msgs::msg::PathRequest::SharedPtr msg);
  void dispatch_states_callback(const rmf_task_msgs::msg::DispatchStates::SharedPtr msg);
  void task_api_response_callback(const rmf_task_msgs::msg::ApiResponse::SharedPtr msg);
  // Section 14 — Cancel (Python lines 3282–3525)
  void handle_cancel_task_request(const std::string & task_id, const std::string & request_id);
  std::optional<std::string> find_robot_by_task_id(const std::string & task_id);
  void clear_queued_task_tracking(const std::string & task_id, const std::string & robot_name);
  void send_cancel_command_to_server(
    const std::string & robot_name,
    const std::string & task_id,
    const std::string & request_id);
  void clear_task_state(const std::string & robot_name, const std::string & task_id);
  void task_api_request_callback(const rmf_task_msgs::msg::ApiRequest::SharedPtr msg);
  // Section 15 — Mode + misc (Python lines 4206–4265)
  void mode_request_callback(const rmf_fleet_msgs::msg::ModeRequest::SharedPtr msg);
  void task_state_update_callback(const std_msgs::msg::String::SharedPtr msg);
  void zone_conflict_callback(const zone_manager::msg::Conflict::SharedPtr msg);
  // ---- Flowchart Stage 6: Path deviation alert callback -------------------
  void path_deviation_callback(const std_msgs::msg::String::SharedPtr msg);
  // ---- Phase 2: per-robot lane closure on zone replan candidates ----------
  void replan_candidate_callback(const zone_manager::msg::ReplanCandidate::SharedPtr msg);
  /// True when this PathRequest is a replan of the leg already published to the
  /// robot (same order, same leg, higher revision, same start/destination).
  bool is_replan_of_published_order(
    const std::string & robot_name,
    const std::string & dispatch_task_id,
    int phase_index,
    int path_revision,
    double last_wp_x,
    double last_wp_y) const;
  /// Cancel the robot's current Nav2 goal so it can adopt a replanned route.
  /// Cancels navigation only — the RMF/FMS task is left untouched.
  bool send_replan_nav_cancel(const std::string & robot_name, const std::string & order_id);
  /// Position-only hash of a PathRequest, used to tell a genuine reroute from a
  /// replan that returned the identical route.
  static std::size_t hash_path_request(const rmf_fleet_msgs::msg::PathRequest & msg);
  /// Close `lanes` for `robot_name` only and record the closure.
  /// When `request_replan` is false (drop corridor), RMF keeps the current
  /// phase and the next GoToPlace(drop) plans on the restricted graph.
  void apply_private_lane_closure(
    const std::string & robot_name,
    const std::string & zone_id,
    const std::string & blocked_zones,
    const std::vector<std::size_t> & lanes,
    const std::string & reason,
    bool request_replan = true);
  /// Reopen every private lane closure held by `robot_name` (no-op if none).
  /// When `replan` is false, lanes reopen with RobotLaneRequest.skip_replan so
  /// the current path (e.g. drop corridor plan) is kept.
  void clear_private_lane_closure(
    const std::string & robot_name, const std::string & reason,
    bool replan = true);
  /// Timer: clears closures older than private_closure_timeout_sec.
  void private_closure_timeout_tick();
  // Python lines 3748–3766, 4093–4110 — delivery queue/id helpers
  void try_start_next_delivery_task();
  static bool is_pickup_phase(const std::string & request_id);
  static std::optional<std::string> get_next_delivery_request_id(const std::string & request_id);
  static std::string get_base_delivery_request_id(const std::string & request_id);
  // Section 16 — Legacy path helpers (Python lines 4112–4204)
  bool is_delivery_task(const std::string & task_id) const;
  std::vector<rmf_fleet_msgs::msg::Location> enhance_path_for_delivery(
    const std::string & robot_name,
    const std::vector<rmf_fleet_msgs::msg::Location> & original_path,
    const std::string & task_id);
  std::vector<rmf_fleet_msgs::msg::Location> plan_delivery_path(
    const std::string & robot_name,
    const Pose2D & current_pos,
    const std::string & task_id);
  rmf_fleet_msgs::msg::Location create_location_msg(
    const Waypoint & waypoint, const std::string & level_name);

  // ---- 4D / ASRS drop corridor (logic in drop_corridor.hpp) --------------
  // Robot `robot_id` from fleet_config, or nullptr if unknown.
  const AMRRobot* FindRobot(const std::string& robot_id) const;
  // Drop point of `task_id` from task_pickup_drop_coords, if known.
  std::optional<drop_corridor::Point> LookupDropPoint(
      const std::string& task_id) const;
  // On pickup completion, closes every lane except the drop corridor for
  // `robot_id` only.
  void ApplyDropCorridorClosure(const std::string& robot_id,
                                const std::string& task_id);
  // Publishes the closure of every lane outside `corridor` for `robot_id`
  // and records it as that robot's corridor hold.
  void CloseLanesOutsideCorridor(const std::string& robot_id,
                                 const std::string& task_id,
                                 const drop_corridor::Point& drop,
                                 const drop_corridor::Corridor& corridor);
  // Releases the robot's corridor closure (no-op for a zone closure).
  void ReleaseDropCorridor(const std::string& robot_id,
                           const std::string& reason, bool replan);
  // PathRequest hook: marks the drop leg planned once RMF's path lies on
  // the corridor. Lanes stay blocked until the robot starts travelling.
  void OnDropCorridorPath(const rmf_fleet_msgs::msg::PathRequest& request);
  // Why `robot_id`'s corridor should be released now, if it should.
  // Sets `*replan` to false when the planned path must be kept.
  std::optional<std::string> DropCorridorReleaseReason(
      const std::string& robot_id, const drop_corridor::Hold& hold,
      double now_sec, bool* replan) const;
  // Timer: frees the lanes once the robot starts travelling to the drop;
  // fallback release at the drop point or when the drop phase is over.
  void DropCorridorTick();

  // --- Python lines 277–292: parameters ---
  std::string fleet_name;
  std::string mqtt_broker_host;
  int mqtt_broker_port{1883};
  std::string config_path;
  std::string nav_graph_path;
  double state_poll_interval{0.04};
  std::string map_conversion_mode;
  // Min |arrival→travel| heading change (deg) before injecting a spin action.
  // Default 45° so only real cardinal pivots (~90/180) are published; change
  // via ROS param spin_threshold_deg without a code edit.
  double spin_threshold_deg{45.0};

  // --- Python lines 295–361: state initialized in __init__ ---
  std::optional<FleetConfig> fleet_config;
  std::unique_ptr<NavGraphParser> nav_graph_parser;
  std::map<std::string, AMRRobot> robot_states;
  std::unique_ptr<MQTTFleetClient> mqtt_client;
  std::map<std::string, std::string> last_known_tasks;
  std::map<std::string, DeliveryTask> active_deliveries;
  int sequence_counter{0};
  std::map<std::string, AnyMap> target_positions;
  std::map<std::string, std::pair<std::string, int>> task_completion_delays;
  std::map<std::string, std::vector<AnyMap>> robot_waypoint_sequences;
  std::map<std::string, bool> robot_path_assigned;
  std::map<std::string, std::vector<AnyMap>> robot_visited_waypoints;
  std::map<std::string, bool> robot_task_cleared;
  std::map<std::string, int> robot_path_cleared;
  std::map<std::string, std::string> active_task_robots;
  std::map<std::string, AnyMap> delivery_task_details;
  std::map<std::string, std::string> delivery_stage_to_base;
  std::map<std::string, YAML::Node> pending_delivery_api_responses;
  std::map<std::string, std::string> dispatch_assignments;
  std::map<std::string, std::string> pending_robot_by_api_request_id;
  std::map<std::string, YAML::Node> pending_task_request_by_api_request_id;
  int task_api_response_count{0};
  std::set<std::string> seen_task_api_response_dedupe;
  std::vector<JsonDict> delivery_task_queue;
  std::optional<std::string> current_delivery_task_id;
  std::map<std::string, bool> nav_idle_by_robot;
  std::map<std::string, std::optional<int>> last_load_status_by_robot;
  std::map<std::string, std::optional<int>> last_operating_mode_by_robot;
  std::map<std::string, std::string> delivery_wait_log_phase;
  std::map<std::string, std::vector<AnyMap>> pending_vda_orders_by_task;
  std::map<std::string, VdaOrderFingerprint> last_published_vda_order;
  std::set<std::string> processed_load_events;
  std::set<std::string> processed_nav_events;
  std::map<std::string, bool> dropoff_in_progress;
  std::map<std::string, AnyMap> task_pickup_drop_coords;
  std::map<std::string, AnyMap> task_patrol_coords;
  std::map<std::string, std::string> robot_to_coords_key;
  // task_id -> "delivery" | "patrol" | "drop", recorded when the task API
  // response is processed. Works for both fleet-dispatch ids and uuid ids.
  std::map<std::string, std::string> task_category_by_id;
  std::map<std::string, std::vector<AnyMap>> pending_path_request_by_robot;
  std::map<std::string, std::set<std::string>> sent_path_request_phase;

  // --- Python lines 364–408: TF, publishers, monitored task manager ---
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
  rclcpp::Publisher<rmf_fleet_msgs::msg::FleetState>::SharedPtr fleet_states_pub;
  rclcpp::Publisher<traffic_manager::msg::BaseExtensionRequest>::SharedPtr base_extension_pub_;
  rclcpp::Publisher<rmf_dispenser_msgs::msg::DispenserResult>::SharedPtr dispenser_results_pub;
  rclcpp::Publisher<rmf_ingestor_msgs::msg::IngestorResult>::SharedPtr ingestor_results_pub;
  rclcpp::Publisher<monitor::msg::MonitoredTaskDetails>::SharedPtr monitored_task_pub;
  std::unique_ptr<MonitoredTaskManager> monitored_task_manager;

  // --- Python lines 412–417: path QoS stored on self.path_qos ---
  rclcpp::QoS path_qos;

  // --- Python lines 446–477: subscription handles ---
  rclcpp::Subscription<rmf_fleet_msgs::msg::PathRequest>::SharedPtr path_request_sub;
  rclcpp::Subscription<rmf_task_msgs::msg::DispatchStates>::SharedPtr dispatch_states_sub;
  rclcpp::Subscription<rmf_task_msgs::msg::ApiResponse>::SharedPtr task_api_sub;
  rclcpp::Subscription<rmf_task_msgs::msg::ApiResponse>::SharedPtr task_api_sub_volatile;
  rclcpp::Subscription<rmf_task_msgs::msg::ApiRequest>::SharedPtr task_api_request_sub;
  rclcpp::Subscription<rmf_fleet_msgs::msg::ModeRequest>::SharedPtr mode_request_sub;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr task_state_update_sub;
  rclcpp::Subscription<zone_manager::msg::Conflict>::SharedPtr zone_conflicts_sub;
  // ---- Phase 2 -----------------------------------------------------------
  rclcpp::Subscription<zone_manager::msg::ReplanCandidate>::SharedPtr replan_candidates_sub;
  rclcpp::Publisher<rmf_fleet_msgs::msg::RobotLaneRequest>::SharedPtr robot_lane_request_pub;
  rclcpp::TimerBase::SharedPtr private_closure_timer;

  /// Robots currently held paused by a zone conflict (startPause sent, no
  /// stopPause yet). A ReplanCandidate is only honoured for a robot in here,
  /// so the closure is applied AFTER the robot has actually been paused.
  std::set<std::string> paused_by_zone;

  /// Flowchart Stage 6 (Q): Robots held paused by path deviation (startPause sent, no automatic resume).
  /// While a robot is in this set, zone resume (stopPause) from zone_manager is blocked.
  std::set<std::string> paused_by_deviation;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr path_deviation_sub;

  /// Last route accepted for each robot, hashed by position.
  ///
  /// A replan that returns the SAME route must not trigger a cancelOrder: the
  /// duplicate order is dropped downstream by the route-unchanged fingerprint
  /// gate, and the robot would be left with its nav goal cancelled and nothing
  /// to replace it. That is the loop measured on 2026-09-22 — RMF's stuck
  /// detector bumping path_revision every 10 s, the planner returning the
  /// identical path, and a cancelOrder going out on each pass for 176 s.
  std::map<std::string, std::size_t> last_accepted_path_hash_by_robot;

  struct PrivateClosure
  {
    std::string zone_id;          // conflict zone
    // The zone(s) actually closed, joined with '+'. One per conflicting
    // robot: the first zone of that robot's remaining sequence which is not
    // this robot's target zone. Falls back to zone_id when none qualified.
    std::string blocked_zones;

    std::string blocked_zone_id;  // zone entered just after it (may be empty)
    std::vector<std::size_t> lanes;
    rclcpp::Time applied_at;
    std::string reason;
    // The task and leg this closure belongs to. A closure lives for the whole
    // leg; the releases are the TASK changing or the LEG changing.
    //
    // Releasing earlier — on the replan's own PathRequest, or on
    // ConflictResolved — reopened the lanes before the robot had committed to
    // the detour, and RMF immediately routed it back through the conflict zone.
    // Measured 2026-08-21: detect/resolve alternating on a 1.000 s cycle for
    // four rounds, 7 cancelOrders in 4 s, orderUpdateId churned to 8 in 8 s.
    //
    // order_id is what makes normal task completion work. There is no
    // completion hook to hang this on: RMFIntegratedFleetAdapter::
    // clear_task_state() is reached from exactly one place — an explicit cancel
    // of an EXECUTING task — so a task that simply finishes never releases
    // anything. Relying on that left amr1 holding zones 5+6 for 626 s across
    // three tasks on 2026-08-22, and its next task could not be planned at all.
    // phase_index alone cannot cover it either: a new patrol task also starts at
    // leg 0, so the number is unchanged. The dispatch id always differs.
    std::string order_id;
    int phase_index{-1};
    // Set only for a 4D / ASRS drop corridor (ApplyDropCorridorClosure).
    // Made at pickup completion for the DROP leg, so phase_index is -1 and
    // the pickup->drop leg change does not release it. Released when
    //   1. the drop leg is planned on the corridor (OnDropCorridorPath), AND
    //   2. the robot starts travelling to the drop (DropCorridorTick),
    // WITHOUT a replan, so the corridor path is kept. Fallbacks: arrival at
    // drop, drop completion, task change.
    std::optional<drop_corridor::Hold> corridor_hold;
  };
  /// robot name -> the private lane closure it currently holds.
  std::map<std::string, PrivateClosure> private_closure_by_robot;

  // ---- Zone sequences ----------------------------------------------------
  // /robot_zone_status carries each robot's remaining zone sequence (passed
  // zones dropped). Needed to find the zone a robot enters JUST AFTER the
  // conflict zone, so the closure can block both: the meeting point and the
  // corridor beyond it.
  //
  // Direction matters, so this must be the REPLANNING robot's sequence:
  //   amr1  3 -> 4 -> 5 -> [6] -> 7 -> 8 -> 9   successor of 6 is 7
  //   amr2  9 -> 8 -> 7 -> [6] -> 5 -> 4 -> 3   successor of 6 is 5
  //
  // Blocking the zone BEFORE the conflict was tried first and is worse on this
  // map. Measured over the nav graph, amr1 routed zone 3 -> zone 9:
  //   no closure   13.1 m   3 -> 4 -> 5 -> 6 -> 7 -> 8 -> 9
  //   block 5+6    30.4 m   3 -> 2 -> 1 -> 11 -> ... -> 20 -> 10 -> 9   (16 lanes)
  //   block 6+7    20.7 m   3 -> 4 -> 5 -> 16 -> ... -> 20 -> 10 -> 9   (10 lanes)
  // Zone 5 owns the lanes into zone 16 (the north corridor), so closing it also
  // closes the natural shortcut and forces the robot to reverse and take the
  // whole ring. Closing the conflict zone and the one past it leaves that
  // shortcut open, denies the meeting point just the same, and blocks 6 fewer
  // lanes.
  rclcpp::Subscription<monitor::msg::RobotZoneStatus>::SharedPtr
    robot_zone_status_sub;
  /// robot name -> remaining zone sequence, most recent report.
  std::map<std::string, std::vector<std::string>> zone_sequence_by_robot;

  void robot_zone_status_callback(
    const monitor::msg::RobotZoneStatus::SharedPtr msg);

  /// Zone the robot passes through immediately after `zone_id`, or "" if it is
  /// unknown or `zone_id` is last in the sequence.
  std::string zone_after(
    const std::string & robot_name, const std::string & zone_id) const;

  /// Zone containing `robot_name`'s destination — the last entry of its
  /// remaining zone sequence — or "" if the sequence is unknown. Never closed
  /// by a replan closure: fencing off the goal makes the replan unsatisfiable.
  std::string target_zone_of(const std::string & robot_name) const;

  /// First zone of `robot_name`'s remaining sequence that is not
  /// `target_zone`, or "" if the sequence is unknown or every zone in it is the
  /// target. Index 0 is where the robot is standing now, so this is normally
  /// its current zone; the walk forward only matters when that zone is the
  /// replanning robot's own destination.
  std::string first_zone_other_than(
    const std::string & robot_name, const std::string & target_zone) const;

  // ---- Deferred replan candidates ----------------------------------------
  // A ReplanCandidate is only actionable once the robot is in paused_by_zone,
  // but /zone_conflicts and /replan_candidate are separate topics with separate
  // DDS readers, so the candidate can still be delivered first even though
  // zone_manager now publishes the Conflict before it. Rather than dropping an
  // early candidate (which is what made zone 6 never replan: the candidate was
  // handled 551 us before the pause was recorded), hold it here and replay it
  // the moment the matching startPause arrives.
  //
  // Dropped when: the pause arrives (replayed), the conflict resolves, or the
  // deadline passes without a pause.
  struct DeferredReplanCandidate
  {
    zone_manager::msg::ReplanCandidate::SharedPtr msg;
    rclcpp::Time received_at;
  };
  /// robot name -> candidate waiting for that robot's startPause.
  std::map<std::string, DeferredReplanCandidate> deferred_replan_by_robot;

  /// How long a candidate may wait for its startPause before being discarded.
  /// The two publishes are sub-millisecond apart, so this only has to cover
  /// executor jitter; it is deliberately short so a stale candidate can never
  /// close lanes for a robot that was never actually paused.
  double deferred_replan_timeout_sec{2.0};

  /// Applies a candidate that is now actionable. Shared by the live path and
  /// the deferred replay so both behave identically.
  void apply_replan_candidate(
    const zone_manager::msg::ReplanCandidate::SharedPtr & msg,
    const std::string & robot_name,
    const std::string & zone_id,
    bool deferred);

  /// Replays a candidate held for `robot_name`, if one is waiting.
  void flush_deferred_replan_candidate(const std::string & robot_name);

  /// Drops candidates that never saw their startPause.
  void deferred_replan_timeout_tick();

  // ---- Replan order republishing ----------------------------------------
  // What we last published to a robot, so a replanned path for the SAME leg can
  // be re-sent as a VDA5050 order update (same orderId, higher orderUpdateId)
  // instead of being suppressed as a duplicate.
  struct PublishedOrder
  {
    std::string order_id;      // dispatch_task_id the order was built for
    std::string phase;         // pick | drop | patrol
    int phase_index{0};        // PathRequest.phase_index of that leg
    int path_revision{0};      // highest revision actually published
    std::size_t released_node_count{0};
    // Furthest released=true node of the published order. Released-node
    // *counts* cannot be compared across updates: build_vda5050_order_payload
    // re-anchors an update at the waypoint nearest the robot, trimming the
    // already-driven prefix, so a base extension that moves the base end
    // forward can carry FEWER released nodes than the order it extends.
    double released_end_x{0.0};
    double released_end_y{0.0};
    bool has_released_end{false};
    double dest_x{0.0};        // last waypoint of the published path
    double dest_y{0.0};
  };
  std::map<std::string, PublishedOrder> published_order_by_robot;

  // ---- Idle gate ---------------------------------------------------------
  // A VDA5050 order is only published while the robot reports MODE_IDLE (i.e.
  // VDA5050 driving=false, derived on the robot from the nav2
  // navigate_through_poses goal status). For a new task the robot is idle
  // anyway; for a REPLAN this is what removes the cancelOrder/order race:
  // cancelOrder -> robot decelerates (driving=true) -> stops (driving=false)
  // -> only then is the replanned order published, so the robot is able to
  // accept it. Orders that arrive while the robot is still moving stay queued
  // in pending_vda_orders_by_task and are retried.
  //
  // first time a task's order was held back, used for the timeout below.
  std::map<std::string, rclcpp::Time> order_hold_since_by_task;
  /// Publish anyway after this long if the robot never reports idle (MQTT gap,
  /// nav node dead) so a replan cannot hang forever.
  // Retained as a declared parameter for compatibility, but no longer used to
  // release an order: the idle gate is strict (see flush_pending_vda_orders).
  double order_idle_gate_timeout_sec{3.0};

  // Escalation for a hold that has stopped being routine. The gate never gives
  // up, so these only control how loudly a stuck hold is reported.
  double order_idle_gate_warn_after_sec{5.0};
  double order_idle_gate_warn_repeat_sec{10.0};

  // ---- Planned path staging (guard 2) ------------------------------------
  // /Monitored_task_details.planned_path must describe the route the robot is
  // ACTUALLY driving, not the one RMF has just decided on.
  //
  // Writing it straight from the PathRequest broke the replan sequence. On
  // 2026-08-20 zone 6: the replan path landed at t=.9784, zone_monitor
  // recomputed off it one tick later (vertices 30->24, edges 26->16), zone
  // manager saw the planned route no longer entering the conflict zone,
  // published ConflictResolved at t+1.0s and resumed BOTH robots — while amr1
  // was still physically driving the OLD route into amr2. The pause that the
  // whole replan depends on was torn down by the replan itself.
  //
  // So the path is staged here and applied only once the corresponding VDA5050
  // order has actually been published to the robot. Until then the zone layer
  // keeps seeing the route the robot is really on, the conflict stands, and the
  // robot stays paused — which is exactly the interval during which it should
  // be stopped. A path that is never published (suppressed by the revision gate,
  // e.g. an RMF negotiation reroute) never reaches the zone layer at all.
  struct StagedPlannedPath
  {
    std::vector<std::string> path;
    std::string monitored_task_id;   // empty = cache entry not updated
    std::string fleet_name;
    std::size_t filtered_count{0};   // for the log line only
  };
  // robot name -> "phase_index|path_revision" -> staged path.
  //
  // Keyed per leg, not just per robot, because more than one path for the same
  // robot can be queued at once (RMF can emit two routes for one plan). A plain
  // per-robot slot would let the second path overwrite the first and then be
  // applied when the FIRST one's order published — reporting a route the robot
  // was never sent.
  //
  // The first stage for a key wins and later ones at the same key are ignored,
  // which mirrors the revision gate in flush_pending_vda_orders(): only one
  // order per (task, phase, revision) is ever published, and it is the first.
  std::map<std::string, std::map<std::string, StagedPlannedPath>>
  staged_planned_path_by_robot;

  /// Key for staged_planned_path_by_robot's inner map.
  static std::string staged_path_key(int phase_index, int path_revision);

  /// VDA5050 orderUpdateId offset for a leg: orderUpdateId = base + path_revision.
  /// 0 for pick/patrol, 1 for drop. Pick and drop share one orderId, so the
  /// drop leg must start above the pick leg's ids (the sim bridge dedupes on
  /// orderId:orderUpdateId). traffic_manager decodes /BaseRequests acks with
  /// base 0, so publish_base_extension_from_mqtt strips this offset first.
  static int vda_order_update_base(const std::string & phase);

  /// Promotes the staged path for one published leg into
  /// /Monitored_task_details. Called only after a successful order publish.
  void apply_staged_planned_path(
    const std::string & robot_name,
    const std::string & order_id,
    int phase_index,
    int path_revision);

  /// task id -> held-seconds at which we last warned (0.0 = not yet warned).
  std::map<std::string, double> order_hold_warned_at_by_task;

  /// True when the robot is idle enough to be given a new order.
  bool robot_ready_for_order(const std::string & robot_name) const;

  /// Safety net: a closure older than this is cleared even if no explicit
  /// clear arrived (zone_manager crash, missed Resolved, adapter race).
  double private_closure_timeout_sec{60.0};

  // --- Python lines 509–519: task state tracking ---
  std::map<std::string, std::string> currently_underway_task;
  std::map<std::string, std::string> known_task_status;
  std::map<std::string, std::string> known_task_robot;
  std::map<std::string, std::map<std::string, std::string>> task_phase_places;
  std::map<std::string, int> vda_order_header_id_by_robot;
  std::map<std::string, int> vda_instant_actions_header_id_by_robot;
  struct PendingIngestorCompletion {
    std::string task_id;
    std::string event_id;
  };
  std::map<std::string, PendingIngestorCompletion> pending_ingestor_by_robot;

  rclcpp::TimerBase::SharedPtr state_poll_timer;
  rclcpp::TimerBase::SharedPtr diagnostic_timer;
  rclcpp::TimerBase::SharedPtr startup_flush_timer;
  bool startup_flush_done{false};

  static constexpr double ARRIVAL_THRESHOLD_M = 0.75;
  // Waypoint sweep look-ahead (advance_waypoint_sequence_if_progressed): how far
  // past the head waypoint the sweep may look for one the robot has reached.
  // Bounded both along the route and by count so a later stretch of the route
  // that happens to pass the same spot (out-and-back, U-turn) is never matched.
  static constexpr double LOOKAHEAD_MAX_PATH_M = 3.0;
  static constexpr std::size_t LOOKAHEAD_MAX_WAYPOINTS = 10;
  // Must match rmf_fleet_adapter estimation.cpp kPathFinishToleranceM (0.4)
  static constexpr double RMF_PATH_FINISH_TOLERANCE_M = 0.40;
  static constexpr double COORD_MATCH_EPSILON = 0.15;
  // RMF completes a leg up to kPathFinishToleranceM short of its goal and plans
  // the next one from that goal waypoint, so a path's first waypoint can sit
  // that far from the live pose. Must be >= RMF_PATH_FINISH_TOLERANCE_M.
  static constexpr double FIRST_WAYPOINT_POSE_TOLERANCE_M = ARRIVAL_THRESHOLD_M;
  // A powerSupply.charging value from a state older than this is unknown.
  static constexpr double CHARGING_STATE_MAX_AGE_SEC = 3.0;

  // 4D / ASRS drop corridor parameters.
  bool drop_corridor_enabled_ = true;
  // Robot within this distance (m) of the drop point = "reached drop".
  double drop_corridor_release_distance_m_ = 0.5;
  // Once the drop leg is planned, the robot has "started travelling" when it
  // reports moving or has moved this far (m).
  double drop_corridor_travel_start_m_ = 0.2;
  // Safety release: no drop plan on the corridor within this time (s) ->
  // free the corridor and let RMF plan the drop on the full graph. 0 = off.
  double drop_corridor_plan_timeout_s_ = 30.0;
  // Highway lines from the 4D layout sheet (empty = infer from the graph).
  drop_corridor::LayoutHighways drop_corridor_layout_;
  rclcpp::TimerBase::SharedPtr drop_corridor_timer_;
};

}  // namespace execution

#endif  // EXECUTION__EXECUTION_FULL_CONTROL_HPP_
