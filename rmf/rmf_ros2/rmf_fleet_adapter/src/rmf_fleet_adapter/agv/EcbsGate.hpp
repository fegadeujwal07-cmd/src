#ifndef SRC__RMF_FLEET_ADAPTER__AGV__ECBSGATE_HPP
#define SRC__RMF_FLEET_ADAPTER__AGV__ECBSGATE_HPP

// added for ECBS — entire EcbsGate class (fleet-adapter side of evaluate_path).

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <rmf_traffic/agv/Planner.hpp>

#include <ecbs_planner/ecbs_solver.hpp>
#include <ecbs_planner/evaluate_path.hpp>
#include <ecbs_planner/msg/from_zone_manager.hpp>
#include <ecbs_planner/msg/to_zone_manager.hpp>
#include <ecbs_planner/msg/ecbs_replan.hpp>
#include <ecbs_planner/schedule_cache.hpp>
#include <ecbs_planner/srv/evaluate_path.hpp>

namespace rmf_fleet_adapter {
namespace agv {

/// Synchronous ECBS gate used inside GoToPlace::_execute_plan.
/// Runs BEFORE ExecutePlan::make so conflict waits are decided before the
/// robot is commanded (still above /robot_path_requests).
///
/// ScheduleCache is the single, dedicated store for ECBS path data: every
/// AMR's committed path -- whether it arrives via GoToPlace::process(),
/// the /evaluate_path_service, or /from_zone_manager -- is read from and
/// written to ScheduleCache only. Nothing here reads from or writes into
/// the RMF traffic schedule Database/Mirror/Itinerary.
///
/// /to_zone_manager publishes the full path once per (amr, task, phase,
/// path_revision). Mid-route remaining-path fragments are not published and
/// do not replace the committed full path in the schedule cache.
///
/// Client/server surface added for zone_manager <-> ECBS, both backed by
/// the same ScheduleCache as process():
///   - /evaluate_path_service (EvaluatePath.srv): mode="register" solves
///     overlaps and stores the path in ScheduleCache; mode="evaluate" reads
///     a previously cached path and returns a FREE/WAIT horizon slice.
///   - /from_zone_manager (FromZoneManager.msg): fire-and-forget register,
///     same ScheduleCache write path as EvaluatePath mode=register.
class EcbsGate
{
public:
  struct Config
  {
    double safety_margin_sec{2.0};
    double max_wait_sec{30.0};
    int max_iters{20};
    double position_epsilon_m{0.35};
    bool enabled{true};
    /// When true, do not offer /evaluate_path_service or /from_zone_manager here;
    /// use ecbs_planner evaluate_path_server (single ScheduleCache for TM + viz).
    bool external_evaluate_server{false};
  };

  static std::shared_ptr<EcbsGate> make(
    const rclcpp::Node::SharedPtr& node,
    Config config);

  /// Run ECBS on the greedy plan. Publishes /to_zone_manager on success
  /// (including any other AMRs that must wait). Publishes replan topic on fail.
  /// added for ECBS — process() no longer gates the leg; see below.
  /// \return always true: ECBS never blocks ExecutePlan. Unresolvable conflicts
  /// publish /ecbs_replan_candidate and are held by traffic_manager via evaluate.
  bool process(
    const std::string& fleet_name,
    const std::string& amr_id,
    const std::string& task_id,
    uint32_t phase_index,
    uint32_t path_revision,
    const rmf_traffic::agv::Plan& plan);

  bool enabled() const { return config_.enabled; }

  /// Task canceled (from the adapter's cancel/kill): drop this AMR's path,
  /// granted base and waits from the ScheduleCache (local and external server).
  void clear_robot(const std::string& amr_id);

private:
  struct PublishRecord
  {
    std::string task_id;
    uint32_t phase_index{0};
    uint32_t path_revision{0};
    std::size_t waypoint_count{0};
  };

  EcbsGate(
    const rclcpp::Node::SharedPtr& node,
    Config config);

  ecbs_planner::TimedPath to_timed_path(
    const std::string& fleet_name,
    const std::string& amr_id,
    const std::string& task_id,
    uint32_t phase_index,
    uint32_t path_revision,
    const rmf_traffic::agv::Plan& plan) const;

  /// True for mid-route / trivial plans that must not replace or republish
  /// the committed full path.
  bool is_progress_fragment(const ecbs_planner::TimedPath& candidate) const;

  bool already_published(const ecbs_planner::TimedPath& candidate) const;

  void remember_published(const ecbs_planner::TimedPath& path);

  void publish_to_zone(const ecbs_planner::TimedPath& path);
  void publish_replan(const ecbs_planner::SolveResult& result);

  /// Register solved paths on evaluate_path_server via /from_zone_manager.
  void publish_from_zone_for_external_server(const ecbs_planner::TimedPath& path);

  /// /evaluate_path_service handler: register or evaluate, entirely against
  /// ScheduleCache via ecbs_planner::evaluate_path().
  void handle_evaluate_path(
    const std::shared_ptr<ecbs_planner::srv::EvaluatePath::Request> request,
    std::shared_ptr<ecbs_planner::srv::EvaluatePath::Response> response);

  /// /from_zone_manager handler: fire-and-forget register into ScheduleCache
  /// via ecbs_planner::register_from_zone_manager().
  void handle_from_zone_manager(
    const ecbs_planner::msg::FromZoneManager::SharedPtr msg);

  Config config_;
  rclcpp::Logger logger_;
  mutable std::mutex mutex_;
  ecbs_planner::ScheduleCache cache_;
  ecbs_planner::EcbsSolver solver_;
  std::unordered_map<std::string, PublishRecord> last_published_;
  // External server mode: a seeded path is released (process() == true) only
  // after evaluate_path_server answers on /ecbs_path_approval for it.
  std::unordered_map<std::string, std::size_t> pending_sig_;
  std::unordered_map<std::string, std::size_t> approved_sig_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr approval_sub_;

  rclcpp::Publisher<ecbs_planner::msg::ToZoneManager>::SharedPtr to_zone_pub_;
  rclcpp::Publisher<ecbs_planner::msg::EcbsReplan>::SharedPtr replan_pub_;
  rclcpp::Service<ecbs_planner::srv::EvaluatePath>::SharedPtr evaluate_path_srv_;
  rclcpp::Client<ecbs_planner::srv::EvaluatePath>::SharedPtr clear_client_;
  rclcpp::Publisher<ecbs_planner::msg::FromZoneManager>::SharedPtr from_zone_pub_;
  rclcpp::Subscription<ecbs_planner::msg::FromZoneManager>::SharedPtr
    from_zone_sub_;
};

} // namespace agv
} // namespace rmf_fleet_adapter

#endif // SRC__RMF_FLEET_ADAPTER__AGV__ECBSGATE_HPP
