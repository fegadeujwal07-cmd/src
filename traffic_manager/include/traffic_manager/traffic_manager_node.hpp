#ifndef TRAFFIC_MANAGER__TRAFFIC_MANAGER_NODE_HPP_
#define TRAFFIC_MANAGER__TRAFFIC_MANAGER_NODE_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "ecbs_planner/msg/clearance_event.hpp"
#include "ecbs_planner/msg/grant_event.hpp"
#include "ecbs_planner/msg/to_zone_manager.hpp"
#include "ecbs_planner/srv/base_extension_query.hpp"
#include "ecbs_planner/srv/commit_proposal.hpp"
#include "ecbs_planner/srv/evaluate_path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rmf_fleet_msgs/msg/fleet_state.hpp"
#include "traffic_manager/msg/base_extension_request.hpp"
#include "traffic_manager/msg/path_intent.hpp"
#include "traffic_manager/msg/path_release.hpp"
#include "traffic_manager/path_gate.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace traffic_manager
{

class TrafficManagerNode : public rclcpp::Node
{
public:
  explicit TrafficManagerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void on_path_intent(const traffic_manager::msg::PathIntent::SharedPtr msg);
  void on_fleet_state(const rmf_fleet_msgs::msg::FleetState::SharedPtr msg);
  void on_base_extension_request(
    const traffic_manager::msg::BaseExtensionRequest::SharedPtr msg);
  void on_eval_timer();
  void on_to_zone_manager(const ecbs_planner::msg::ToZoneManager::SharedPtr msg);
  /// After a staged PathIntent became the active session (any path): register
  /// the NEW route with ECBS and start its first query (or clear if empty).
  /// Must be called WITHOUT mutex_ held.
  void complete_staged_activation(const std::string & robot_id, const char * where);
  /// ECBS → TM: decision changed / blocker cleared for this robot → re-query now.
  void on_ecbs_release_event(const ecbs_planner::msg::ClearanceEvent::SharedPtr msg);

  /// \p extend_from_awarded_tip: G6/T9 newBaseRequest — floor horizon at the
  /// already-awarded tip so the next FREE award can advance past it even when
  /// the live pose is still catching up (pose-only horizon stuck at same tip).
  void request_ecbs_evaluation(
    const std::string & robot_id,
    bool force,
    bool extend_from_awarded_tip = false);

  /// Release path: proposal → validate → commit.
  void request_base_extension_query(
    const std::string & robot_id,
    bool force,
    bool extend_from_awarded_tip = false);

  void handle_base_extension_query_response(
    const std::string & robot_id,
    rclcpp::Client<ecbs_planner::srv::BaseExtensionQuery>::SharedFuture future,
    int32_t horizon_end_index,
    std::size_t session_hash,
    uint64_t evaluation_id);

  void schedule_pending_initial_ecbs_eval(const std::string & robot_id);
  void run_pending_initial_ecbs_eval(const std::string & robot_id);
  void cancel_initial_ecbs_timer(const std::string & robot_id);

  /// Erase amr from ScheduleCache (new PathIntent / task end).
  void send_ecbs_clear(const std::string & robot_id);

  /// Upsert PathIntent geometry into ScheduleCache so BaseExtensionQuery indices
  /// match the route full_control executes (not /from_zone_manager sampling).
  /// Locking: acquires mutex_ internally — MUST NOT be called while mutex_ held.
  /// \p recovery_txn_id non-zero: validate identity on async response; ignore stale.
  void send_ecbs_seed_path_intent(
    const std::string & robot_id,
    bool then_schedule_initial_eval = false,
    uint64_t recovery_txn_id = 0);

  /// Under mutex_: start or refresh one route-recovery txn. Preserves tip/grant/
  /// pending_extension. Returns true if caller should seed outside the lock.
  bool begin_or_refresh_route_recovery_locked(
    PathReleaseSession & session,
    const std::string & reason);

  /// Under mutex_: validate recovery seed response identity. Returns true when
  /// tip-extend should run outside the lock after a successful seed.
  bool complete_route_recovery_seed_locked(
    PathReleaseSession & session,
    uint64_t recovery_txn_id,
    bool seed_ok,
    const std::string & seed_message);

  /// Notify ECBS of commit/reject. When publish_on_accept, PathRelease is
  /// published only after CommitProposal ACCEPTED/DUPLICATE.
  void commit_proposal_to_ecbs(
    const PathReleaseSession & session,
    uint64_t proposal_id,
    uint64_t grant_id,
    int32_t release_through_index,
    const std::string & status,
    const std::string & reason,
    bool publish_on_accept = false);

  /// Report verified physical clearance for one overlap to ECBS.
  void publish_clearance_event(
    const std::string & amr_id,
    const std::string & conflict_id,
    bool physically_cleared,
    const std::string & reason);

  /// Progress-driven: footprint clearance of overlaps; grant tip completion.
  void evaluate_grant_clearance_from_progress(const std::string & robot_id);

  /// Publish GrantEvent COMPLETED so ECBS retires commitment/active_go.
  void publish_grant_completed(
    const PathReleaseSession & session,
    uint64_t grant_id,
    const std::string & reason);

  /// Capture only overlaps this robot participates in (blocker side for clearance).
  void retain_participant_overlaps(
    PathReleaseSession & session,
    const ecbs_planner::srv::BaseExtensionQuery::Response & response,
    uint64_t grant_id);

  /// True when rear/load envelope is past exit with uncertainty margin.
  bool footprint_cleared_exit(
    const PathReleaseSession & session,
    const PathReleaseSession::TrackedOverlapClearance & t,
    double px, double py, int32_t progress) const;

  traffic_manager::msg::PathRelease make_release_msg(
    const PathReleaseSession & session,
    bool approved,
    int32_t release_through_index) const;

  void publish_release(const traffic_manager::msg::PathRelease & release);

  /// Rebuild PathRelease RViz markers.
  /// Locking contract: MUST NOT be called while mutex_ is already held.
  /// This function acquires mutex_ itself (std::mutex is not recursive).
  /// Callers that hold mutex_ must copy publish data, release the lock, then
  /// call this (see CommitProposal response callback).
  void rebuild_path_release_markers();
  void on_marker_republish_timer();

  bool robot_needs_reevaluation(
    const PathReleaseSession & session,
    double current_x,
    double current_y) const;

  PathGate gate_;
  std::mutex mutex_;

  std::unordered_map<std::string, rmf_fleet_msgs::msg::RobotState> robot_states_;

  rclcpp::Subscription<traffic_manager::msg::PathIntent>::SharedPtr intent_sub_;
  rclcpp::Subscription<rmf_fleet_msgs::msg::FleetState>::SharedPtr fleet_state_sub_;
  rclcpp::Subscription<traffic_manager::msg::BaseExtensionRequest>::SharedPtr base_request_sub_;
  rclcpp::Subscription<ecbs_planner::msg::ToZoneManager>::SharedPtr to_zone_sub_;
  rclcpp::Subscription<ecbs_planner::msg::ClearanceEvent>::SharedPtr release_event_sub_;
  rclcpp::Publisher<traffic_manager::msg::PathRelease>::SharedPtr release_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr path_release_marker_pub_;
  rclcpp::TimerBase::SharedPtr marker_republish_timer_;
  visualization_msgs::msg::MarkerArray last_path_release_markers_;
  rclcpp::Client<ecbs_planner::srv::EvaluatePath>::SharedPtr ecbs_client_;
  rclcpp::Client<ecbs_planner::srv::BaseExtensionQuery>::SharedPtr
    base_query_client_;
  rclcpp::Client<ecbs_planner::srv::CommitProposal>::SharedPtr commit_client_;
  rclcpp::Publisher<ecbs_planner::msg::GrantEvent>::SharedPtr grant_event_pub_;
  rclcpp::Publisher<ecbs_planner::msg::ClearanceEvent>::SharedPtr
    clearance_event_pub_;
  rclcpp::TimerBase::SharedPtr eval_timer_;
  std::unordered_map<std::string, rclcpp::TimerBase::SharedPtr> initial_eval_timers_;
  std::unordered_map<std::string, rclcpp::TimerBase::SharedPtr> to_zone_reeval_timers_;
  uint64_t next_grant_id_{1};
  uint64_t next_route_recovery_txn_id_{1};

  double query_horizon_m_{20.0};
  double initial_ecbs_defer_ms_{400.0};
  double reevaluate_travel_m_{12.0};
  bool send_task_id_to_ecbs_{false};
  double held_reevaluate_sec_{0.0};
  bool use_new_base_request_trigger_{true};
  /// Rear/load envelope + uncertainty for physical clearance evidence.
  double amr_length_m_{1.5};
  double clearance_uncertainty_m_{0.5};
  /// Standstill dwell before STOP_CONFIRMED (authority reconcile).
  double authority_stop_confirm_sec_{1.0};
  std::string fleet_state_topic_{"/fleet_states"};
  std::string base_extension_request_topic_{"/BaseRequests"};
  std::string to_zone_manager_topic_{"/to_zone_manager"};
  bool reevaluate_on_to_zone_{true};
  bool publish_path_release_markers_{true};
  /// Set when a robot ACKs an order update; the marker timer redraws.
  std::atomic<bool> release_markers_dirty_{false};
  std::string path_release_marker_topic_{"/traffic_manager/path_release_markers"};
  std::string marker_frame_id_{"map"};
  bool use_level_name_as_marker_frame_{false};
  double marker_republish_hz_{2.0};
  /// ECBS owns every traffic decision; TM never vetoes an ECBS proposal.
  bool ecbs_authoritative_{true};
  /// Minimum spacing between route-recovery transactions per robot.
  double recovery_min_interval_sec_{1.0};
  std::unordered_map<std::string, double> last_recovery_start_sec_;
};

}  // namespace traffic_manager

#endif  // TRAFFIC_MANAGER__TRAFFIC_MANAGER_NODE_HPP_
