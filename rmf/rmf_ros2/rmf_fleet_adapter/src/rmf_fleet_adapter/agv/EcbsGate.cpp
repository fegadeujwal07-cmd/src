// added for ECBS — entire EcbsGate implementation (fleet-adapter side).
#include "EcbsGate.hpp"

#include <cmath>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <ecbs_planner/msg/clear_point.hpp>
#include <ecbs_planner/msg/path_point.hpp>
#include <ecbs_planner/msg/wait_dependency.hpp>
#include <ecbs_planner/evaluate_path.hpp>
#include <ecbs_planner/path_utils.hpp>
#include <rmf_traffic_ros2/Time.hpp>

namespace rmf_fleet_adapter {
namespace agv {
namespace {

builtin_interfaces::msg::Time sec_to_stamp(double t_sec)
{
  builtin_interfaces::msg::Time out;
  if (t_sec < 0.0)
    t_sec = 0.0;
  const auto sec = static_cast<int32_t>(std::floor(t_sec));
  out.sec = sec;
  out.nanosec = static_cast<uint32_t>(std::llround((t_sec - sec) * 1e9));
  if (out.nanosec >= 1000000000u)
  {
    out.sec += 1;
    out.nanosec -= 1000000000u;
  }
  return out;
}

double time_to_sec(const rmf_traffic::Time& t)
{
  using namespace std::chrono;
  return duration<double>(t.time_since_epoch()).count();
}

// Identity of a path's geometry (not times / revision): RMF retries re-plan
// the same route with new timestamps.
std::size_t path_signature(const ecbs_planner::TimedPath& path)
{
  // Skip the first waypoint (the robot's live pose drifts while it moves) and
  // use graph indices so an RMF retry of the same route keeps its signature.
  std::size_t h = 1469598103934665603ull;
  for (std::size_t i = 1; i < path.waypoints.size(); ++i)
  {
    const auto& w = path.waypoints[i];
    const auto k = w.index ? static_cast<std::size_t>(*w.index)
      : static_cast<std::size_t>(std::llround(w.x * 10) * 100003 + std::llround(w.y * 10));
    h = (h ^ k) * 1099511628211ull;
  }
  return h ^ std::hash<std::string>()(path.task_id);
}

} // namespace

//==============================================================================
std::shared_ptr<EcbsGate> EcbsGate::make(
  const rclcpp::Node::SharedPtr& node,
  Config config)
{
  return std::shared_ptr<EcbsGate>(new EcbsGate(node, std::move(config)));
}

//==============================================================================
EcbsGate::EcbsGate(
  const rclcpp::Node::SharedPtr& node,
  Config config)
: config_(std::move(config)),
  logger_(node->get_logger().get_child("ecbs_gate")),
  solver_([&]()
    {
      ecbs_planner::SolverConfig c;
      c.safety_margin_sec = config_.safety_margin_sec;
      c.max_wait_sec = config_.max_wait_sec;
      c.max_iters = config_.max_iters;
      c.position_epsilon_m = config_.position_epsilon_m;
      return ecbs_planner::EcbsSolver(c);
    }())
{
  const auto qos = rclcpp::SystemDefaultsQoS();
  to_zone_pub_ =
    node->create_publisher<ecbs_planner::msg::ToZoneManager>(
    "/to_zone_manager", qos);
  replan_pub_ =
    node->create_publisher<ecbs_planner::msg::EcbsReplan>(
    "/ecbs_replan_candidate", qos);

  std::string from_zone_topic = "(disabled)";
  std::string evaluate_service = "(disabled)";
  if (!config_.external_evaluate_server)
  {
    // Zone Manager <-> ECBS client/server path evaluation.
    if (!node->has_parameter("evaluate_path_service"))
    {
      node->declare_parameter<std::string>(
        "evaluate_path_service", "/evaluate_path_service");
    }
    evaluate_service =
      node->get_parameter("evaluate_path_service").as_string();
    evaluate_path_srv_ = node->create_service<ecbs_planner::srv::EvaluatePath>(
      evaluate_service,
      [this](
        const std::shared_ptr<ecbs_planner::srv::EvaluatePath::Request> request,
        std::shared_ptr<ecbs_planner::srv::EvaluatePath::Response> response)
      {
        handle_evaluate_path(request, response);
      });

    if (!node->has_parameter("from_zone_manager_topic"))
    {
      node->declare_parameter<std::string>(
        "from_zone_manager_topic", "/from_zone_manager");
    }
    from_zone_topic =
      node->get_parameter("from_zone_manager_topic").as_string();
    from_zone_sub_ = node->create_subscription<ecbs_planner::msg::FromZoneManager>(
      from_zone_topic,
      rclcpp::QoS(rclcpp::KeepLast(50)).reliable(),
      [this](const ecbs_planner::msg::FromZoneManager::SharedPtr msg)
      {
        handle_from_zone_manager(msg);
      });
  } else {
    if (!node->has_parameter("from_zone_manager_topic")) {
      node->declare_parameter<std::string>(
        "from_zone_manager_topic", "/from_zone_manager");
    }
    from_zone_topic =
      node->get_parameter("from_zone_manager_topic").as_string();
    from_zone_pub_ =
      node->create_publisher<ecbs_planner::msg::FromZoneManager>(
      from_zone_topic, rclcpp::QoS(rclcpp::KeepLast(50)).reliable());
    evaluate_service = "(client disabled; pub " + from_zone_topic + ")";
    clear_client_ = node->create_client<ecbs_planner::srv::EvaluatePath>(
      "/evaluate_path_service");
    approval_sub_ = node->create_subscription<std_msgs::msg::String>(
      "/ecbs_path_approval", rclcpp::QoS(rclcpp::KeepLast(50)).reliable(),
      [this](const std_msgs::msg::String::SharedPtr m)
      {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto it = pending_sig_.find(m->data);
        if (it == pending_sig_.end())
          return;
        approved_sig_[m->data] = it->second;
        pending_sig_.erase(it);
      });
  }

  RCLCPP_INFO(
    logger_,
    "ECBS gate ready (enabled=%s, external_evaluate_server=%s) -> "
    "/to_zone_manager (full path once per revision) + sub %s + service %s",
    config_.enabled ? "true" : "false",
    config_.external_evaluate_server ? "true" : "false",
    from_zone_topic.c_str(),
    evaluate_service.c_str());
}

//==============================================================================
ecbs_planner::TimedPath EcbsGate::to_timed_path(
  const std::string& fleet_name,
  const std::string& amr_id,
  const std::string& task_id,
  uint32_t phase_index,
  uint32_t path_revision,
  const rmf_traffic::agv::Plan& plan) const
{
  ecbs_planner::TimedPath path;
  path.fleet_name = fleet_name;
  path.amr_id = amr_id;
  path.task_id = task_id;
  path.dispatch_task_id = task_id;
  path.phase_index = phase_index;
  path.path_revision = path_revision;

  for (const auto& wp : plan.get_waypoints())
  {
    ecbs_planner::TimedPoint p;
    p.t_sec = time_to_sec(wp.time());
    const Eigen::Vector3d pos = wp.position();
    p.x = pos.x();
    p.y = pos.y();
    p.yaw = pos.z();
    if (wp.graph_index())
      p.index = *wp.graph_index();
    path.waypoints.push_back(std::move(p));
  }
  // Overwrite RMF DifferentialDrive times with monitor eta_path_calculator.
  if (!path.waypoints.empty())
  {
    path.waypoints.front().t_sec = 0.0;
    ecbs_planner::stamp_timed_path_with_eta(path, 0.0);
  }
  return path;
}

//==============================================================================
bool EcbsGate::is_progress_fragment(
  const ecbs_planner::TimedPath& candidate) const
{
  // Trivial / single-point plans are never useful as the "full path".
  if (candidate.waypoints.size() < 2)
    return true;

  const auto existing = cache_.get(candidate.amr_id);
  if (!existing.has_value())
    return false;

  // New task or phase -> treat as a fresh full path.
  if (existing->task_id != candidate.task_id ||
    existing->phase_index != candidate.phase_index)
  {
    return false;
  }

  // zone_manager (or similar) bumped revision -> accept as new full path.
  if (candidate.path_revision > existing->path_revision)
    return false;

  // Same task/phase without a higher revision, but fewer waypoints -> remaining
  // mid-route plan. Keep the committed full path; do not publish again.
  if (candidate.waypoints.size() < existing->waypoints.size())
    return true;

  return false;
}

//==============================================================================
bool EcbsGate::already_published(const ecbs_planner::TimedPath& candidate) const
{
  const auto it = last_published_.find(candidate.amr_id);
  if (it == last_published_.end())
    return false;

  const auto& rec = it->second;
  return rec.task_id == candidate.task_id &&
    rec.phase_index == candidate.phase_index &&
    rec.path_revision == candidate.path_revision;
}

//==============================================================================
void EcbsGate::remember_published(const ecbs_planner::TimedPath& path)
{
  PublishRecord rec;
  rec.task_id = path.task_id;
  rec.phase_index = path.phase_index;
  rec.path_revision = path.path_revision;
  rec.waypoint_count = path.waypoints.size();
  last_published_[path.amr_id] = std::move(rec);
}

//==============================================================================
void EcbsGate::publish_to_zone(const ecbs_planner::TimedPath& path)
{
  ecbs_planner::msg::ToZoneManager msg;
  msg.fleet_name = path.fleet_name;
  msg.amr_id = path.amr_id;
  msg.path.reserve(path.waypoints.size());

  for (const auto& wp : path.waypoints)
  {
    ecbs_planner::msg::PathPoint pt;
    pt.t = sec_to_stamp(wp.t_sec);
    pt.x = wp.x;
    pt.y = wp.y;
    pt.wait_sec = ecbs_planner::published_wait_sec(wp);
    for (const auto& dep : wp.waiting_for)
    {
      ecbs_planner::msg::WaitDependency d;
      d.amr_id = dep.amr_id;
      d.wait_point.x = dep.wait_point.x;
      d.wait_point.y = dep.wait_point.y;
      d.wait_point.index = dep.wait_point.index.has_value() ?
        static_cast<int64_t>(*dep.wait_point.index) : static_cast<int64_t>(-1);
      d.wait_point.wait_sec = dep.wait_point.wait_duration_sec;
      for (const auto& c : dep.clear_points)
      {
        ecbs_planner::msg::ClearPoint cp;
        cp.x = c.x;
        cp.y = c.y;
        d.clear_points.push_back(cp);
      }
      pt.waiting_for.push_back(std::move(d));
    }
    msg.path.push_back(std::move(pt));
  }

  to_zone_pub_->publish(msg);
  RCLCPP_INFO(
    logger_,
    "ECBS gate published FULL /to_zone_manager amr=%s task=%s phase=%u rev=%u wps=%zu",
    path.amr_id.c_str(),
    path.task_id.c_str(),
    path.phase_index,
    path.path_revision,
    msg.path.size());
}

//==============================================================================
void EcbsGate::publish_from_zone_for_external_server(
  const ecbs_planner::TimedPath& path)
{
  if (!config_.external_evaluate_server || !from_zone_pub_) {
    return;
  }
  auto msg = ecbs_planner::from_zone_manager_msg(path);
  from_zone_pub_->publish(msg);
  RCLCPP_INFO(
    logger_,
    "ECBS gate published /from_zone_manager amr=%s task=%s phase=%u rev=%u wps=%zu "
    "(evaluate_path_server ScheduleCache)",
    path.amr_id.c_str(),
    path.task_id.c_str(),
    path.phase_index,
    path.path_revision,
    msg.waypoints.size());
}

void EcbsGate::publish_replan(const ecbs_planner::SolveResult& result)
{
  // Head-on REPLAN/CLEAR/NO_ALTERNATIVE is owned by evaluate_path_server's
  // ReplanCoordinator. Publishing an empty decision here made full_control
  // treat it as "no route" spam and fought the one-shot replan.
  RCLCPP_WARN(
    logger_,
    "ECBS no wait-only resolution for %s: %s (evaluate_path_server decides replan)",
    result.path.amr_id.c_str(), result.reason.c_str());
}

// void EcbsGate::publish_replan(const ecbs_planner::SolveResult& result)
// {
//   ecbs_planner::msg::EcbsReplan msg;
//   msg.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
//   msg.amr_id = result.path.amr_id;
//   msg.task_id = result.path.task_id;
//   msg.reason = result.reason;
//   msg.conflicting_amrs = result.conflicting_amrs;
//   replan_pub_->publish(msg);
//   RCLCPP_WARN(
//     logger_,
//     "ECBS no wait-only resolution for %s: %s (published /ecbs_replan_candidate; "
//     "RMF workflow continues, traffic_manager holds via evaluate)",
//     msg.amr_id.c_str(), msg.reason.c_str());
// }

//==============================================================================
void EcbsGate::handle_evaluate_path(
  const std::shared_ptr<ecbs_planner::srv::EvaluatePath::Request> request,
  std::shared_ptr<ecbs_planner::srv::EvaluatePath::Response> response)
{
  if (!request || !response)
    return;

  std::vector<ecbs_planner::TimedPath> committed;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ecbs_planner::evaluate_path(
      *request, *response, cache_, solver_, &committed);
  }

  // Register mode: push every committed path (incl. mutual waits) to zone layer.
  for (const auto& path : committed)
    publish_to_zone(path);

  RCLCPP_INFO(
    logger_,
    "EvaluatePath service mode=%s amr=%s status=%s horizon_m=%.1f "
    "dep=[%ld,%ld] waiting_for=%zu (%s) to_zone=%zu",
    request->mode.empty() ? "(auto)" : request->mode.c_str(),
    response->amr_id.c_str(),
    response->status.c_str(),
    response->horizon_m,
    static_cast<long>(response->dep_from_index),
    static_cast<long>(response->dep_to_index),
    response->waiting_for.size(),
    response->message.c_str(),
    committed.size());
}

//==============================================================================
void EcbsGate::handle_from_zone_manager(
  const ecbs_planner::msg::FromZoneManager::SharedPtr msg)
{
  if (!msg)
    return;

  ecbs_planner::srv::EvaluatePath::Response res;
  std::vector<ecbs_planner::TimedPath> committed;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ecbs_planner::register_from_zone_manager(
      *msg, res, cache_, solver_, &committed);
  }

  for (const auto& path : committed)
    publish_to_zone(path);

  RCLCPP_INFO(
    logger_,
    "/from_zone_manager registered amr=%s status=%s horizon_m=%.0f wps_in=%zu "
    "(%s) to_zone=%zu",
    msg->amr_id.c_str(),
    res.status.c_str(),
    res.horizon_m,
    msg->waypoints.size(),
    res.message.c_str(),
    committed.size());
}

//==============================================================================
void EcbsGate::clear_robot(const std::string& amr_id)
{
  if (!config_.enabled || amr_id.empty())
    return;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    cache_.erase(amr_id);
    last_published_.erase(amr_id);
    pending_sig_.erase(amr_id);
    approved_sig_.erase(amr_id);
  }
  if (!clear_client_ || !clear_client_->service_is_ready())
  {
    RCLCPP_WARN(logger_, "ECBS clear skipped for %s: server not ready", amr_id.c_str());
    return;
  }
  auto req = std::make_shared<ecbs_planner::srv::EvaluatePath::Request>();
  req->mode = "clear";
  req->amr_id = amr_id;
  clear_client_->async_send_request(req);
  RCLCPP_INFO(logger_, "ECBS clear (task canceled) amr=%s", amr_id.c_str());
}

//==============================================================================
bool EcbsGate::process(
  const std::string& fleet_name,
  const std::string& amr_id,
  const std::string& task_id,
  uint32_t phase_index,
  uint32_t path_revision,
  const rmf_traffic::agv::Plan& plan)
{
  if (!config_.enabled)
    return true;

  auto candidate = to_timed_path(
    fleet_name, amr_id, task_id, phase_index, path_revision, plan);

  ecbs_planner::SolveResult result;
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (is_progress_fragment(candidate))
    {
      RCLCPP_DEBUG(
        logger_,
        "ECBS skip publish: progress fragment amr=%s wps=%zu rev=%u (keeping full cached path)",
        amr_id.c_str(), candidate.waypoints.size(), path_revision);
      return true;
    }

    // Hard gate: no PathRequest until evaluate_path_server approved this path.
    const auto sig = path_signature(candidate);
    if (config_.external_evaluate_server)
    {
      const auto ap = approved_sig_.find(amr_id);
      if (ap != approved_sig_.end() && ap->second == sig)
        return true;
      const auto pe = pending_sig_.find(amr_id);
      if (pe != pending_sig_.end() && pe->second == sig)
        return false;
    }

    if (!config_.external_evaluate_server && already_published(candidate))
    {
      RCLCPP_DEBUG(
        logger_,
        "ECBS skip publish: already published full path amr=%s task=%s phase=%u rev=%u",
        amr_id.c_str(), task_id.c_str(), phase_index, path_revision);
      return true;
    }

    if (config_.external_evaluate_server)
    {
      // RMF handover: forward the planned waypoints as-is, right now (before
      // ExecutePlan::make). evaluate_path_server's GlobalResolver evaluates
      // and decides every conflict on receipt; no local solve here — this
      // gate's cache has no live poses and its "updated_others" were stale
      // copies of other robots' routes.
      cache_.upsert(candidate);  // fragment / duplicate filtering only
      result.status = ecbs_planner::SolveStatus::Ok;
      result.path = candidate;
      result.updated_others.clear();

      // Arm the approval gate before the seed is published (no race with a
      // fast /ecbs_path_approval). GoToPlace retries until it is approved.
      approved_sig_.erase(amr_id);
      pending_sig_[amr_id] = sig;
    }
    else
    {
      result = solver_.solve(candidate, cache_);

      // Never interrupt the RMF workflow: keep the path as planned and let
      // traffic_manager hold the robot through evaluate (WAIT).
      if (result.status == ecbs_planner::SolveStatus::FailReplan)
      {
        publish_replan(result);
        result.path = candidate;
        result.updated_others.clear();
        cache_.upsert(candidate);
      }
    }
  }

  // Keep evaluate_path_server ScheduleCache aligned with this gate (traffic_manager
  // evaluates against that cache, not this local cache).
  // Register on evaluate_path_server BEFORE /to_zone_manager so traffic_manager
  // re-eval sees populated ScheduleCache (no sync service call — avoids executor deadlock).
  if (config_.external_evaluate_server) {
    publish_from_zone_for_external_server(result.path);
    for (const auto & other : result.updated_others) {
      publish_from_zone_for_external_server(other);
    }
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    remember_published(result.path);
    for (const auto& other : result.updated_others)
      remember_published(other);
  }

  // External server: it alone publishes /to_zone_manager (after applying base
  // ownership, holds and reserved bases). A second publisher here would race
  // it and could show waits the server removed. The path is withheld (no
  // ExecutePlan / PathRequest) until evaluate_path_server approves it.
  if (config_.external_evaluate_server)
  {
    RCLCPP_INFO(
      logger_, "ECBS gate: path of %s withheld until ECBS approves it",
      amr_id.c_str());
    return false;
  }

  // Publish candidate, then every other AMR that got waits/deps so
  // /to_zone_manager shows updated dependency-annotated paths (not only the
  // newly planned robot).
  publish_to_zone(result.path);
  for (const auto& other : result.updated_others)
  {
    publish_to_zone(other);
    RCLCPP_INFO(
      logger_,
      "ECBS republished /to_zone_manager for dependency-updated amr=%s "
      "(because of candidate=%s)",
      other.amr_id.c_str(), amr_id.c_str());
  }

  return true;
}

} // namespace agv
} // namespace rmf_fleet_adapter
