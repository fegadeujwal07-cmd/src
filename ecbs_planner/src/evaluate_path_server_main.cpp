#include <algorithm>
#include <cctype>
#include <map>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>
#include <rclcpp/rclcpp.hpp>
#include <rmf_building_map_msgs/msg/building_map.hpp>
#include <rmf_fleet_msgs/msg/fleet_state.hpp>

#include <ecbs_planner/ecbs_solver.hpp>
#include <ecbs_planner/evaluate_path.hpp>
#include <ecbs_planner/global_resolver.hpp>
#include <ecbs_planner/idle_tracker.hpp>
#include <ecbs_planner/msg/ecbs_replan.hpp>
#include <ecbs_planner/passage_arbitration.hpp>
#include <ecbs_planner/wait_reason.hpp>
#include <ecbs_planner/msg/clearance_event.hpp>
#include <ecbs_planner/msg/from_zone_manager.hpp>
#include <ecbs_planner/msg/grant_event.hpp>
#include <std_msgs/msg/string.hpp>
#include <ecbs_planner/msg/to_zone_manager.hpp>
#include <ecbs_planner/nav_graph_junctions.hpp>
#include <ecbs_planner/path_utils.hpp>
#include <ecbs_planner/proposal_engine.hpp>
#include <ecbs_planner/replan_candidate.hpp>
#include <ecbs_planner/schedule_cache.hpp>
#include <ecbs_planner/srv/base_extension_query.hpp>
#include <ecbs_planner/srv/commit_proposal.hpp>
#include <ecbs_planner/srv/evaluate_path.hpp>
#include <monitor/msg/monitored_task_details.hpp>

/// Standalone ECBS path server:
///   - sub /from_zone_manager → register full path into ScheduleCache
///   - srv /evaluate_path_service → register or evaluate horizon
///   - pub /to_zone_manager → every path committed after register (waits/deps)
class EvaluatePathServer : public rclcpp::Node
{
public:
  EvaluatePathServer()
  : Node("evaluate_path_server")
  {
    declare_parameter<std::string>("service_name", "/evaluate_path_service");
    declare_parameter<std::string>(
      "from_zone_manager_topic", "/from_zone_manager");
    declare_parameter<std::string>(
      "to_zone_manager_topic", "/to_zone_manager");
    // config/ecbs_params.yaml is the source of truth: its values are the
    // parameter defaults, so the node uses them even when started without
    // the params file. Parameters passed at launch still override them.
    ecbs_planner::SolverConfig defaults;
    {
      std::string yaml_path;
      try {
        yaml_path = ament_index_cpp::get_package_share_directory("ecbs_planner") +
          "/config/ecbs_params.yaml";
        defaults = ecbs_planner::load_solver_config_yaml(yaml_path, defaults);
        RCLCPP_INFO(
          get_logger(), "SolverConfig defaults loaded from %s", yaml_path.c_str());
      } catch (const std::exception & e) {
        RCLCPP_WARN(
          get_logger(),
          "SolverConfig defaults NOT loaded from ecbs_params.yaml (%s); using "
          "built-in fallbacks unless parameters are passed at launch", e.what());
      }
    }
    declare_parameter<double>("safety_margin_sec", defaults.safety_margin_sec);
    declare_parameter<double>("max_wait_sec", defaults.max_wait_sec);
    declare_parameter<int>("max_iters", defaults.max_iters);
    declare_parameter<double>("position_epsilon_m", defaults.position_epsilon_m);
    declare_parameter<double>("safety_distance_m", defaults.safety_distance_m);
    declare_parameter<double>(
      "evaluate_lookahead_margin_m", defaults.evaluate_lookahead_margin_m);
    declare_parameter<double>("clearance_m", defaults.clearance_m);
    declare_parameter<double>("amr_length_m", defaults.amr_length_m);
    declare_parameter<double>("amr_width_m", defaults.amr_width_m);
    declare_parameter<double>("junction_release_m", defaults.junction_release_m);

    ecbs_planner::SolverConfig cfg;
    cfg.safety_margin_sec = get_parameter("safety_margin_sec").as_double();
    cfg.max_wait_sec = get_parameter("max_wait_sec").as_double();
    cfg.max_iters = get_parameter("max_iters").as_int();
    cfg.position_epsilon_m = get_parameter("position_epsilon_m").as_double();
    cfg.safety_distance_m = get_parameter("safety_distance_m").as_double();
    cfg.evaluate_lookahead_margin_m =
      get_parameter("evaluate_lookahead_margin_m").as_double();
    cfg.clearance_m = get_parameter("clearance_m").as_double();
    cfg.amr_length_m = get_parameter("amr_length_m").as_double();
    cfg.amr_width_m = get_parameter("amr_width_m").as_double();
    cfg.junction_release_m = get_parameter("junction_release_m").as_double();
    solver_ = std::make_unique<ecbs_planner::EcbsSolver>(cfg);

    // ---- GlobalResolver: ECBS decides every conflict at path registration ----
    declare_parameter<bool>("global_resolver_enabled", true);
    declare_parameter<bool>("register_on_rmf_handover", true);
    declare_parameter<bool>("decide_only_on_rmf_handover", true);
    declare_parameter<double>("handover_wait_timeout_sec", 2.0);
    declare_parameter<double>("resolver_free_pass_ratio", 1.5);
    declare_parameter<double>("resolver_crossing_max_len_m", 3.0);
    declare_parameter<double>("resolver_body_back_m", 0.5);
    declare_parameter<std::string>("release_event_topic", "/ecbs/release_event");
    declare_parameter<std::string>("replan_topic", "/ecbs_replan_candidate");
    // Alias of replan_topic (GlobalResolver YAML name). replan_topic wins.
    declare_parameter<std::string>("replan_candidate_topic", "/ecbs_replan_candidate");
    // Resolver mode: also mirror RMF handovers into ScheduleCache and run the
    // ReplanCoordinator on them (detour-vs-wait, approval gate). false →
    // only the resolver's unresolvable-conflict report is published.
    declare_parameter<bool>("resolver_replan_enabled", true);
    resolver_enabled_ = get_parameter("global_resolver_enabled").as_bool();
    accept_gate_seed_ = get_parameter("register_on_rmf_handover").as_bool();
    {
      ecbs_planner::ResolverConfig rc;
      rc.conflict_distance_m = cfg.clearance_m;
      rc.safety_distance_m = cfg.safety_distance_m;
      rc.free_pass_ratio = get_parameter("resolver_free_pass_ratio").as_double();
      rc.crossing_max_len_m = get_parameter("resolver_crossing_max_len_m").as_double();
      rc.body_back_m = get_parameter("resolver_body_back_m").as_double();
      rc.decide_on_handover_only = accept_gate_seed_ &&
        get_parameter("decide_only_on_rmf_handover").as_bool();
      handover_wait_timeout_sec_ = get_parameter("handover_wait_timeout_sec").as_double();
      resolver_ = std::make_unique<ecbs_planner::GlobalResolver>(rc);
    }
    // Fallback for PathIntents parked waiting for their RMF handover.
    pending_timer_ = create_wall_timer(
      std::chrono::milliseconds(250), [this]() {on_pending_intent_timer();});
    release_event_pub_ = create_publisher<ecbs_planner::msg::ClearanceEvent>(
      get_parameter("release_event_topic").as_string(),
      rclcpp::QoS(rclcpp::KeepLast(50)).reliable());
    {
      const auto topic = get_parameter("replan_topic").as_string();
      const auto alias = get_parameter("replan_candidate_topic").as_string();
      if (alias != topic) {
        RCLCPP_WARN(
          get_logger(),
          "replan_candidate_topic='%s' differs from replan_topic='%s'; "
          "publishing replan decisions on '%s' only",
          alias.c_str(), topic.c_str(), topic.c_str());
      }
      replan_pub_ = create_publisher<ecbs_planner::msg::EcbsReplan>(
        topic, rclcpp::QoS(rclcpp::KeepLast(20)).reliable());
    }

    const auto to_zone_topic =
      get_parameter("to_zone_manager_topic").as_string();
    to_zone_pub_ = create_publisher<ecbs_planner::msg::ToZoneManager>(
      to_zone_topic, rclcpp::SystemDefaultsQoS());

    approval_pub_ = create_publisher<std_msgs::msg::String>(
      "/ecbs_path_approval", rclcpp::QoS(rclcpp::KeepLast(50)).reliable());
    const auto service_name = get_parameter("service_name").as_string();
    server_ = create_service<ecbs_planner::srv::EvaluatePath>(
      service_name,
      [this](
        const std::shared_ptr<ecbs_planner::srv::EvaluatePath::Request> req,
        std::shared_ptr<ecbs_planner::srv::EvaluatePath::Response> res)
      {
        if (resolver_enabled_) {
          handle_evaluate_resolver(req, res);
          return;
        }
        std::vector<ecbs_planner::TimedPath> committed;
        std::vector<ecbs_planner::ReplanDecision> decisions;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          last_request_[req->amr_id] = std::chrono::steady_clock::now();
          const bool is_register = !req->updates.empty() &&
            req->mode != "evaluate" && req->mode != "clear";
          if (is_register) {
            // Register is validated against ConflictRecords: make them current.
            ecbs_planner::refresh_conflicts_from_cache(
              proposal_state_, cache_, solver_->config().clearance_m,
              solver_->config().position_epsilon_m,
              solver_->config().safety_distance_m);
          }
          // CommitProposal owns granted-base authority (no dual grant write).
          ecbs_planner::evaluate_path(
            *req, *res, cache_, *solver_, &committed, &junctions_,
            /*record_granted_base=*/false, &proposal_state_.conflicts);
          if (replan_) {
            // Decide a head-on replan as soon as the deadlock exists, before
            // either AMR drives to a wait point. If it changed this AMR's
            // holds, answer from the updated cache (same validation inputs).
            decisions = replan_->update(cache_, now().seconds());
            const auto touched = collect_replan_changes(committed);
            if (std::find(touched.begin(), touched.end(), req->amr_id) != touched.end()) {
              *res = ecbs_planner::srv::EvaluatePath::Response();
              ecbs_planner::evaluate_path(
                *req, *res, cache_, *solver_, &committed, &junctions_,
                /*record_granted_base=*/false, &proposal_state_.conflicts);
            }
          }
          // Register accepted (validated against conflicts): persist it so the
          // arbiter sees waiter -> blocker as a mandatory dependency. Done after
          // the replan pass so it reflects the final answer.
          if (res->success && is_register) {
            const auto n = ecbs_planner::persist_register_dependencies(
              *req, cache_, proposal_state_.conflicts);
            ecbs_planner::refresh_conflicts_from_cache(
              proposal_state_, cache_, solver_->config().clearance_m,
              solver_->config().position_epsilon_m,
              solver_->config().safety_distance_m);
            RCLCPP_INFO(
              get_logger(), "register persisted %zu mandatory dependencies", n);
          }
        }
        publish_committed(committed);
        publish_decisions(decisions);
        RCLCPP_INFO(
          get_logger(),
          "EvaluatePath mode=%s amr=%s status=%s horizon_m=%.1f "
          "dep=[%ld,%ld] waiting_for=%zu success=%s to_zone=%zu",
          req->mode.empty() ? "(auto)" : req->mode.c_str(),
          res->amr_id.c_str(),
          res->status.c_str(),
          res->horizon_m,
          static_cast<long>(res->dep_from_index),
          static_cast<long>(res->dep_to_index),
          res->waiting_for.size(),
          res->success ? "true" : "false",
          committed.size());
      });

    const auto from_topic =
      get_parameter("from_zone_manager_topic").as_string();
    from_zone_sub_ = create_subscription<ecbs_planner::msg::FromZoneManager>(
      from_topic,
      rclcpp::QoS(rclcpp::KeepLast(50)).reliable(),
      [this](const ecbs_planner::msg::FromZoneManager::SharedPtr msg)
      {
        if (resolver_enabled_) {
          handle_gate_seed_resolver(msg);
          return;
        }
        ecbs_planner::srv::EvaluatePath::Response res;
        std::vector<ecbs_planner::TimedPath> committed;
        std::vector<ecbs_planner::ReplanDecision> decisions;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          last_request_[msg->amr_id] = std::chrono::steady_clock::now();
          // Before update(): empty task_id must not skip parked-AMR REPLAN.
          if (replan_) {
            replan_->set_awaiting(msg->amr_id, true);
          }
          ecbs_planner::register_from_zone_manager(
            *msg, res, cache_, *solver_, &committed);
          if (replan_) {
            decisions = replan_->update(cache_, now().seconds());
            collect_replan_changes(committed);
          }
        }
        // Mark awaiting BEFORE /to_zone_manager so a conflicted seed never
        // flashes on screen, then publish decisions / approval state.
        publish_approval(msg->amr_id, decisions);
        publish_committed(committed);
        publish_decisions(decisions);
        RCLCPP_INFO(
          get_logger(),
          "/from_zone_manager amr=%s status=%s horizon_m=%.0f wps=%zu (%s) "
          "published_to_zone=%zu",
          msg->amr_id.c_str(),
          res.status.c_str(),
          res.horizon_m,
          msg->waypoints.size(),
          res.message.c_str(),
          committed.size());
      });

    // Live positions: idle and waiting robots are obstacles for wait points;
    // what a moving robot has passed is freed (path behind it, base behind
    // it except the last base_tail_m).
    declare_parameter<std::string>("fleet_state_topic", "/fleet_states");
    // Stretch kept reserved behind a moving robot (m). Footprint
    // (clearance_m) is applied separately around cells — do not subtract it.
    declare_parameter<double>("base_tail_m", 2.0);
    // /fleet_states task_id is empty during tasks too: a robot is idle only
    // once the traffic manager (evaluates every few seconds while its task
    // runs, also while held at a wait) has not asked about it for this long.
    declare_parameter<double>("idle_after_sec", 10.0);
    // Cancelled robots often keep a ghost fleet task_id: treat as parked when
    // not moving, position stable, and ECBS has no live path for them.
    declare_parameter<double>("idle_hold_sec", 3.0);
    declare_parameter<double>("idle_pos_tol_m", 0.15);
    cache_.set_base_tail_m(get_parameter("base_tail_m").as_double());
    idle_after_ = std::chrono::duration<double>(get_parameter("idle_after_sec").as_double());
    idle_cfg_.empty_task_sec = 3.0;  // rule (1) unchanged
    idle_cfg_.hold_sec = get_parameter("idle_hold_sec").as_double();
    idle_cfg_.pos_tol_m = get_parameter("idle_pos_tol_m").as_double();
    const auto fleet_topic = get_parameter("fleet_state_topic").as_string();
    fleet_sub_ = create_subscription<rmf_fleet_msgs::msg::FleetState>(
      fleet_topic,
      rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
      [this](const rmf_fleet_msgs::msg::FleetState::SharedPtr msg)
      {
        if (resolver_enabled_) {
          handle_fleet_resolver(msg);
          return;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        const auto now = std::chrono::steady_clock::now();
        for (const auto & robot : msg->robots) {
          mirror_fleet_robot_locked(robot, now);
          auto & f = proposal_state_.priorities[robot.name];
          f.robot_id = robot.name;
          // MODE_IDLE / MODE_MOVING / charging treated available; adapter
          // MODE_OFF / emergency not available.
          f.available =
            robot.mode.mode != rmf_fleet_msgs::msg::RobotMode::MODE_ADAPTER_ERROR;
          f.healthy = robot.battery_percent < 0.0 || robot.battery_percent > 5.0;
          if (!robot.task_id.empty()) {
            proposal_state_.current_task_by_robot[robot.name] = robot.task_id;
          }
        }
      });

    // Full nav-graph → cache once → T (deg 3) / cross (deg ≥ 4) with spatial
    // hash. Prefer YAML; else first /building_map (not filtered /nav_graphs1).
    declare_parameter<std::string>("nav_graph_yaml", "");
    declare_parameter<std::string>("nav_graph_level", "L1");
    declare_parameter<std::string>("building_map_topic", "/building_map");

    std::string yaml_path = get_parameter("nav_graph_yaml").as_string();
    if (yaml_path.empty()) {
      try {
        yaml_path = ament_index_cpp::get_package_share_directory("execution") +
          "/config/nav_graphs/0.yaml";
      } catch (const std::exception &) {
        yaml_path.clear();
      }
    }
    const auto level = get_parameter("nav_graph_level").as_string();
    if (!yaml_path.empty() && junctions_.load_from_yaml(yaml_path, level)) {
      push_junctions_to_resolver();
      RCLCPP_INFO(
        get_logger(),
        "Cached full nav graph from YAML '%s' (level=%s): "
        "vertices=%zu edges=%zu → %zu T/cross junctions (hashed)",
        yaml_path.c_str(), level.c_str(),
        junctions_.vertex_count(), junctions_.edge_count(), junctions_.size());
    } else {
      RCLCPP_WARN(
        get_logger(),
        "Nav graph YAML not loaded (path='%s' level=%s); "
        "waiting for %s",
        yaml_path.c_str(), level.c_str(),
        get_parameter("building_map_topic").as_string().c_str());
    }

    const auto map_topic = get_parameter("building_map_topic").as_string();
    building_map_sub_ =
      create_subscription<rmf_building_map_msgs::msg::BuildingMap>(
      map_topic,
      rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
      [this](const rmf_building_map_msgs::msg::BuildingMap::SharedPtr msg)
      {
        // Load once — full map into cache; ignore later updates.
        if (junctions_.loaded()) {
          return;
        }
        std::vector<std::pair<double, double>> verts;
        std::vector<std::pair<std::size_t, std::size_t>> edges;
        for (const auto & level_msg : msg->levels) {
          for (const auto & g : level_msg.nav_graphs) {
            const std::size_t base = verts.size();
            for (const auto & v : g.vertices) {
              verts.emplace_back(v.x, v.y);
            }
            for (const auto & e : g.edges) {
              edges.emplace_back(
                base + static_cast<std::size_t>(e.v1_idx),
                base + static_cast<std::size_t>(e.v2_idx));
            }
          }
        }
        if (verts.empty()) {
          RCLCPP_WARN(
            get_logger(),
            "BuildingMap '%s' has no nav_graphs vertices", msg->name.c_str());
          return;
        }
        junctions_.set_from_edges(verts, edges);
        {
          std::lock_guard<std::mutex> lock(mutex_);
          push_junctions_to_resolver();
        }
        RCLCPP_INFO(
          get_logger(),
          "Cached full nav graph from BuildingMap '%s': "
          "vertices=%zu edges=%zu → %zu T/cross junctions (hashed)",
          msg->name.c_str(),
          junctions_.vertex_count(), junctions_.edge_count(),
          junctions_.size());
      });

    declare_parameter<bool>("replan_enabled", true);
    declare_parameter<double>("replan_timeout_sec", 15.0);
    // <0 → use safety_margin_sec so wait ETA matches the solver's clear margin.
    declare_parameter<double>("replan_blocked_margin_sec", -1.0);
    declare_parameter<double>("replan_blocked_margin_growth", 0.1);
    declare_parameter<double>("replan_blocked_tail_ignore_m", 2.5);
    declare_parameter<int>("replan_max_retries", 5);
    declare_parameter<double>("replan_period_sec", 1.0);
    // Wait before approving a seeded path so the replan check can see the
    // wait ETAs TM adds right after the seed (must exceed replan_period_sec).
    declare_parameter<double>("approval_settle_sec", 1.3);
    // Horizon of the pre-approval evaluate (probe_replans). Must match the
    // traffic_manager's query_horizon_m: <= 0 (default) reads it from the
    // installed traffic_manager/config/traffic_manager.yaml, so that YAML is
    // the only place the value is set.
    declare_parameter<double>("query_horizon_m", 0.0);
    probe_query_horizon_m_ = resolve_query_horizon_m(
      get_parameter("query_horizon_m").as_double());
    declare_parameter<double>("replan_reopen_guard_sec", 10.0);
    declare_parameter<double>("replan_default_speed_mps", 0.5);
    // Parked AMR on a path segment (not solver clearance_m, often 0.05).
    declare_parameter<double>("parked_block_radius_m", 1.0);
    declare_parameter<double>("goal_block_radius_m", 1.0);
    declare_parameter<int>("replan_max_iterations", 5);
    if (get_parameter("replan_enabled").as_bool()) {
      ecbs_planner::ReplanConfig rc;
      rc.clearance_m = cfg.clearance_m;
      rc.parked_block_radius_m =
        get_parameter("parked_block_radius_m").as_double();
      rc.goal_block_radius_m =
        get_parameter("goal_block_radius_m").as_double();
      rc.replan_max_iterations =
        get_parameter("replan_max_iterations").as_int();
      rc.safety_distance_m = cfg.safety_distance_m;
      rc.base_tail_m = get_parameter("base_tail_m").as_double();
      rc.replan_timeout_sec = get_parameter("replan_timeout_sec").as_double();
      {
        const double m = get_parameter("replan_blocked_margin_sec").as_double();
        rc.blocked_margin_sec =
          m >= 0.0 ? m : get_parameter("safety_margin_sec").as_double();
      }
      rc.blocked_margin_growth =
        get_parameter("replan_blocked_margin_growth").as_double();
      rc.blocked_tail_ignore_m =
        get_parameter("replan_blocked_tail_ignore_m").as_double();
      rc.replan_max_retries = get_parameter("replan_max_retries").as_int();
      rc.reopen_guard_sec = get_parameter("replan_reopen_guard_sec").as_double();
      rc.default_speed_mps = get_parameter("replan_default_speed_mps").as_double();
      replan_ = std::make_unique<ecbs_planner::ReplanCoordinator>(rc);
      load_replan_graph();
      resolver_replan_ = resolver_enabled_ &&
        get_parameter("resolver_replan_enabled").as_bool();
      const double period = get_parameter("replan_period_sec").as_double();
      replan_timer_ = create_wall_timer(
        std::chrono::duration<double>(period > 0.05 ? period : 1.0),
        [this]() {run_replan();});
    }

    // Priority: subscribe directly; missing priority = no preference (never block).
    declare_parameter<std::string>(
      "monitored_task_topic", "/Monitored_task_details");
    const auto monitored_topic =
      get_parameter("monitored_task_topic").as_string();
    monitored_sub_ =
      create_subscription<monitor::msg::MonitoredTaskDetails>(
      monitored_topic,
      rclcpp::QoS(rclcpp::KeepLast(50)).reliable(),
      [this](const monitor::msg::MonitoredTaskDetails::SharedPtr msg)
      {
        if (resolver_enabled_) {
          handle_monitored_resolver(msg);
          return;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        auto & f = proposal_state_.priorities[msg->amr_id];
        f.robot_id = msg->amr_id;
        f.task_id = msg->task_id;
        f.task_priority = msg->priority;
        f.priority_version = ++proposal_state_.priority_version;
        // Health/availability come from fleet_states; do not hardcode true.
        proposal_state_.current_task_by_robot[msg->amr_id] = msg->task_id;
      });

    declare_parameter<std::string>(
      "base_extension_query_service", "/base_extension_query");
    declare_parameter<std::string>(
      "commit_proposal_service", "/commit_proposal");
    declare_parameter<std::string>(
      "clearance_event_topic", "/traffic_manager/clearance_event");
    declare_parameter<std::string>(
      "grant_event_topic", "/traffic_manager/grant_event");

    base_query_server_ =
      create_service<ecbs_planner::srv::BaseExtensionQuery>(
      get_parameter("base_extension_query_service").as_string(),
      [this](
        const std::shared_ptr<ecbs_planner::srv::BaseExtensionQuery::Request> req,
        std::shared_ptr<ecbs_planner::srv::BaseExtensionQuery::Response> res)
      {
        handle_base_extension_query(req, res);
      });

    commit_server_ = create_service<ecbs_planner::srv::CommitProposal>(
      get_parameter("commit_proposal_service").as_string(),
      [this](
        const std::shared_ptr<ecbs_planner::srv::CommitProposal::Request> req,
        std::shared_ptr<ecbs_planner::srv::CommitProposal::Response> res)
      {
        handle_commit_proposal(req, res);
      });

    clearance_sub_ = create_subscription<ecbs_planner::msg::ClearanceEvent>(
      get_parameter("clearance_event_topic").as_string(),
      rclcpp::QoS(rclcpp::KeepLast(50)).reliable(),
      [this](const ecbs_planner::msg::ClearanceEvent::SharedPtr msg)
      {
        if (resolver_enabled_) {
          return;  // clearance is derived from /fleet_states progress
        }
        std::lock_guard<std::mutex> lock(mutex_);
        if (ecbs_planner::apply_clearance(
            proposal_state_, msg->amr_id, msg->conflict_id,
            msg->physically_cleared))
        {
          RCLCPP_INFO(
            get_logger(),
            "ClearanceEvent conflict=%s amr=%s cleared",
            msg->conflict_id.c_str(), msg->amr_id.c_str());
        } else if (msg->physically_cleared) {
          RCLCPP_WARN(
            get_logger(),
            "ClearanceEvent rejected conflict=%s amr=%s (not a participant)",
            msg->conflict_id.c_str(), msg->amr_id.c_str());
        }
      });

    grant_event_sub_ = create_subscription<ecbs_planner::msg::GrantEvent>(
      get_parameter("grant_event_topic").as_string(),
      rclcpp::QoS(rclcpp::KeepLast(50)).reliable(),
      [this](const ecbs_planner::msg::GrantEvent::SharedPtr msg)
      {
        if (resolver_enabled_) {
          return;  // commits arrive via CommitProposal
        }
        std::lock_guard<std::mutex> lock(mutex_);
        if (msg->status == ecbs_planner::kGrantCompleted) {
          const bool ok = ecbs_planner::retire_grant(
            proposal_state_, msg->amr_id, msg->grant_id, &cache_);
          RCLCPP_INFO(
            get_logger(),
            "GrantEvent COMPLETED amr=%s grant_id=%lu retire=%s",
            msg->amr_id.c_str(),
            static_cast<unsigned long>(msg->grant_id),
            ok ? "ok" : "miss");
          return;
        }
        ecbs_planner::RouteOccurrence from_occ;
        from_occ.path_revision = msg->from_occurrence.path_revision;
        from_occ.route_occurrence = msg->from_occurrence.route_occurrence;
        from_occ.segment_offset_m = msg->from_occurrence.segment_offset_m;
        from_occ.x = msg->from_occurrence.x;
        from_occ.y = msg->from_occurrence.y;
        ecbs_planner::RouteOccurrence to_occ;
        to_occ.path_revision = msg->to_occurrence.path_revision;
        to_occ.route_occurrence = msg->to_occurrence.route_occurrence;
        to_occ.segment_offset_m = msg->to_occurrence.segment_offset_m;
        to_occ.x = msg->to_occurrence.x;
        to_occ.y = msg->to_occurrence.y;
        const auto result = ecbs_planner::apply_commit(
          proposal_state_, msg->proposal_id, msg->grant_id, msg->amr_id,
          msg->status, msg->release_through_index, from_occ, to_occ,
          msg->path_revision, msg->task_id, &cache_);
        RCLCPP_INFO(
          get_logger(),
          "GrantEvent amr=%s status=%s apply=%s grant_id=%lu",
          msg->amr_id.c_str(), msg->status.c_str(), result.c_str(),
          static_cast<unsigned long>(msg->grant_id));
      });

    RCLCPP_INFO(
      get_logger(),
      "evaluate_path_server ready on %s + base_extension_query + commit_proposal, "
      "listening %s + %s + %s + %s, publishing %s (ECBS owns arbitration) — "
      "GlobalResolver %s (%s)",
      service_name.c_str(), from_topic.c_str(), fleet_topic.c_str(),
      map_topic.c_str(), monitored_topic.c_str(), to_zone_topic.c_str(),
      resolver_enabled_ ? "ENABLED: decisions at path registration, TM obeys" : "disabled",
      accept_gate_seed_ ?
      "decides on RMF handover /from_zone_manager; TM PathIntent binds" :
      "decides on TM PathIntent registration");
    const auto & c = solver_->config();
    RCLCPP_INFO(
      get_logger(),
      "ECBS SolverConfig: safety_distance_m=%.3f clearance_m=%.3f "
      "amr_length_m=%.3f amr_width_m=%.3f position_epsilon_m=%.3f "
      "evaluate_lookahead_margin_m=%.3f junction_release_m=%.3f "
      "safety_margin_sec=%.3f max_wait_sec=%.3f max_iters=%d",
      c.safety_distance_m, c.clearance_m, c.amr_length_m, c.amr_width_m,
      c.position_epsilon_m, c.evaluate_lookahead_margin_m, c.junction_release_m,
      c.safety_margin_sec, c.max_wait_sec, c.max_iters);
  }

private:
  /// Same indexing as rmf_fleet_adapter parse_graph: vertices numbered level
  /// by level (plus one extra waypoint per dock lane with an entry event),
  /// lanes directed, lift waypoints chained both ways.
  void load_replan_graph()
  {
    declare_parameter<std::string>("replan_nav_graph_file", "");
    std::string file = get_parameter("replan_nav_graph_file").as_string();
    if (file.empty()) {
      try {
        file = ament_index_cpp::get_package_share_directory("execution") +
          "/config/nav_graphs/0.yaml";
      } catch (const std::exception &) {
      }
    }
    ecbs_planner::NavGraph graph;
    try {
      const YAML::Node root = YAML::LoadFile(file);
      std::size_t vnum = 0;
      std::size_t extra = 0;
      std::unordered_map<std::string, std::vector<std::size_t>> lifts;
      for (const auto & level : root["levels"]) {
        std::size_t count = 0;
        for (const auto & v : level.second["vertices"]) {
          graph.set_position(vnum + count, v[0].as<double>(), v[1].as<double>());
          const YAML::Node opts = v[2];
          if (opts && opts["lift"] && !opts["lift"].as<std::string>().empty()) {
            lifts[opts["lift"].as<std::string>()].push_back(vnum + count);
          }
          ++count;
        }
        std::size_t docks = 0;
        for (const auto & lane : level.second["lanes"]) {
          const std::size_t begin = lane[0].as<std::size_t>() + vnum;
          const std::size_t end = lane[1].as<std::size_t>() + vnum;
          graph.add_lane(begin, end);
          const YAML::Node opts = lane[2];
          if (opts && opts["dock_name"] && opts["door_name"]) {
            ++docks;
          }
        }
        vnum += count + docks;
        extra += docks;
      }
      for (const auto & [name, wps] : lifts) {
        for (std::size_t i = 0; i + 1 < wps.size(); ++i) {
          graph.add_lane(wps[i], wps[i + 1]);
          graph.add_lane(wps[i + 1], wps[i]);
        }
      }
      RCLCPP_INFO(
        get_logger(), "Replan route check: nav graph %s (%zu waypoints%s)",
        file.c_str(), graph.num_waypoints(),
        extra ? ", dock waypoints shift indices" : "");
      replan_->set_graph(std::move(graph));
    } catch (const std::exception & e) {
      RCLCPP_WARN(
        get_logger(),
        "Replan route check disabled: cannot load nav graph [%s]: %s",
        file.c_str(), e.what());
    }
  }

  /// Pose + idle state of one robot into cache_ and the ReplanCoordinator
  /// (parked AMRs are obstacles; idle pairs are never replanned).
  /// Caller holds mutex_.
  void mirror_fleet_robot_locked(
    const rmf_fleet_msgs::msg::RobotState & robot,
    std::chrono::steady_clock::time_point now)
  {
    cache_.update_pose(robot.name, robot.location.x, robot.location.y);
    const auto asked = last_request_.find(robot.name);
    const bool quiet = asked == last_request_.end() || now - asked->second > idle_after_;
    cache_.set_idle(
      robot.name,
      quiet && robot.task_id.empty() &&
      robot.mode.mode != rmf_fleet_msgs::msg::RobotMode::MODE_MOVING);
    if (!replan_) {
      return;
    }
    const double t = this->now().seconds();
    const auto path = cache_.get(robot.name);
    const bool has_live_path = path && !path->waypoints.empty();
    const bool moving =
      robot.mode.mode == rmf_fleet_msgs::msg::RobotMode::MODE_MOVING;
    const auto upd = idle_tracker_.update(
      robot.name, t, robot.task_id.empty(), moving, has_live_path,
      robot.location.x, robot.location.y, idle_cfg_);
    if (upd.change == ecbs_planner::IdleChange::BecameIdleByHold) {
      RCLCPP_INFO(
        get_logger(),
        "amr=%s idle by mode/position/no path; fleet task_id='%s' ignored",
        robot.name.c_str(), robot.task_id.c_str());
    } else if (upd.change == ecbs_planner::IdleChange::LeftIdle) {
      RCLCPP_INFO(
        get_logger(), "amr=%s no longer idle (moved, path, or task)",
        robot.name.c_str());
    }
    ecbs_planner::RobotFacts f;
    f.idle = upd.idle;
    replan_->set_facts(robot.name, f);
  }

  /// Resolver mode: run the same seed-time conflict solve + replan check as
  /// the legacy /from_zone_manager path on the mirrored ScheduleCache. The
  /// GlobalResolver keeps sole authority over waits/releases; this only
  /// decides REPLAN (detour beats waiting) before the path is approved, so a
  /// conflicted route is never released and the alternate route comes back
  /// as a new handover that the resolver re-decides. Caller holds mutex_.
  std::vector<ecbs_planner::ReplanDecision> replan_on_handover_locked(
    const ecbs_planner::msg::FromZoneManager & msg)
  {
    std::vector<ecbs_planner::ReplanDecision> decisions;
    if (!resolver_replan_ || !replan_) {
      return decisions;
    }
    last_request_[msg.amr_id] = std::chrono::steady_clock::now();
    replan_->set_awaiting(msg.amr_id, true);
    ecbs_planner::srv::EvaluatePath::Response res;
    std::vector<ecbs_planner::TimedPath> committed;  // cache_ mirror only
    ecbs_planner::register_from_zone_manager(msg, res, cache_, *solver_, &committed);
    decisions = replan_->update(cache_, now().seconds());
    collect_replan_changes(committed);
    return decisions;
  }

  /// \p configured > 0 wins; otherwise traffic_manager.yaml's
  /// traffic_manager.ros__parameters.query_horizon_m; 10 m if unreadable.
  double resolve_query_horizon_m(double configured)
  {
    if (configured > 0.0) {
      RCLCPP_INFO(get_logger(), "query_horizon_m=%.2f (parameter)", configured);
      return configured;
    }
    std::string file;
    try {
      file = ament_index_cpp::get_package_share_directory("traffic_manager") +
        "/config/traffic_manager.yaml";
      const YAML::Node v =
        YAML::LoadFile(file)["traffic_manager"]["ros__parameters"]["query_horizon_m"];
      if (v && v.as<double>() > 0.0) {
        RCLCPP_INFO(
          get_logger(), "query_horizon_m=%.2f (from %s)", v.as<double>(), file.c_str());
        return v.as<double>();
      }
    } catch (const std::exception & e) {
      RCLCPP_WARN(
        get_logger(), "query_horizon_m not read from traffic_manager.yaml (%s)", e.what());
    }
    RCLCPP_WARN(
      get_logger(), "query_horizon_m not found in %s — using 10.0 m",
      file.empty() ? "traffic_manager share" : file.c_str());
    return 10.0;
  }

  void run_replan()
  {
    std::vector<ecbs_planner::ReplanDecision> decisions;
    std::vector<ecbs_planner::TimedPath> committed;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      decisions = replan_->update(cache_, now().seconds());
      collect_replan_changes(committed);
    }
    publish_committed(committed);
    publish_decisions(decisions);
  }

  /// The adapter's EcbsGate releases a seeded path only on this message, and
  /// /to_zone_manager shows it only then. The detour-vs-wait decision needs the
  /// wait ETA that TM's evaluates add AFTER the seed, so approval waits
  /// approval_settle_sec (>= one replan pass); a REPLAN for this AMR in that
  /// time keeps it withheld until the replanned path is seeded.
  void publish_approval(
    const std::string & amr, const std::vector<ecbs_planner::ReplanDecision> & decisions)
  {
    if (!replan_) {  // no replan timer to release it later
      approve(amr);
      return;
    }
    const bool replanning = std::any_of(
      decisions.begin(), decisions.end(), [&](const auto & d) {
        return d.kind == ecbs_planner::ReplanDecisionKind::Replan && d.candidate == amr;
      });
    const auto it = awaiting_approval_.find(amr);
    // Same-path retry: keep waiting. After a REPLAN, a new seed must re-arm.
    if (it != awaiting_approval_.end() && !it->second.replanned && !replanning) {
      return;
    }
    const double t = now().seconds();
    awaiting_approval_[amr] = {
      t, t + get_parameter("approval_settle_sec").as_double(), replanning};
    if (replanning) {
      RCLCPP_INFO(
        get_logger(),
        "path approval withheld amr=%s (REPLAN at seed — conflicted path never released)",
        amr.c_str());
    }
  }

  void approve(const std::string & amr)
  {
    awaiting_approval_.erase(amr);
    std_msgs::msg::String m;
    m.data = amr;
    approval_pub_->publish(m);
    std::vector<ecbs_planner::TimedPath> shown;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (replan_) {
        replan_->set_awaiting(amr, false);
      }
      if (auto p = cache_.get(amr)) {
        shown.push_back(*p);
      }
    }
    publish_committed(shown);  // now visible on /to_zone_manager
  }

  /// Approve once settled with no REPLAN, or once ECBS gave up on the detour
  /// (NO_ALTERNATIVE / CLEAR). Never approve a still-conflicted path on timeout:
  /// force fallback to the first RMF path, or keep withholding.
  void release_awaiting(const std::vector<ecbs_planner::ReplanDecision> & decisions)
  {
    const double t = now().seconds();
    const double timeout = get_parameter("replan_timeout_sec").as_double();
    std::vector<std::pair<std::string, const char *>> ready;
    std::vector<std::string> timeout_fallback;
    for (auto & [amr, w] : awaiting_approval_) {
      bool gave_up = false;
      const char * gave_why = "no detour";
      for (const auto & d : decisions) {
        if (d.candidate != amr) {
          continue;
        }
        if (d.kind == ecbs_planner::ReplanDecisionKind::Replan) {
          w.replanned = true;
        } else {
          gave_up = true;
          if (d.reason.find("using first RMF path") != std::string::npos) {
            gave_why = "fallback first RMF path";
          } else if (d.reason.find("path clear at iteration") != std::string::npos) {
            gave_why = "path clear";
          }
        }
      }
      if (gave_up) {
        ready.emplace_back(amr, gave_why);
      } else if (t - w.since > timeout) {
        if (w.replanned) {
          timeout_fallback.push_back(amr);
        } else {
        ready.emplace_back(amr, "replan timeout");
        }
      } else if (w.replanned) {
        // Conflicted path must never be released; wait for the alternate seed.
        continue;
      } else if (t >= w.ready_at) {
        ready.emplace_back(amr, "settled, no replan");
      }
    }

    std::vector<ecbs_planner::ReplanDecision> timeout_clears;
    if (!timeout_fallback.empty() && replan_) {
      std::lock_guard<std::mutex> lock(mutex_);
      for (const auto & amr : timeout_fallback) {
        if (replan_->force_obstacle_fallback(cache_, amr, timeout_clears)) {
          ready.emplace_back(amr, "fallback first RMF path");
        } else if (replan_->path_still_conflicted(cache_, amr, t)) {
          awaiting_approval_[amr].since = t;  // extend; never approve dirty
          RCLCPP_WARN(
            get_logger(),
            "replan timeout but path still conflicted; withholding amr=%s",
            amr.c_str());
        } else {
          ready.emplace_back(amr, "replan timeout");
        }
      }
    }
    if (!timeout_clears.empty()) {
      emit_decisions(timeout_clears);
    }

    for (const auto & [amr, why] : ready) {
      if (std::string(why) == "settled, no replan" && probe_replans(amr)) {
        awaiting_approval_[amr].replanned = true;  // REPLAN sent: path never released
        RCLCPP_INFO(get_logger(), "path withheld amr=%s (REPLAN after evaluate)", amr.c_str());
        continue;
      }
      RCLCPP_INFO(get_logger(), "path approved amr=%s (%s)", amr.c_str(), why);
      approve(amr);
    }
  }

  /// The same first evaluate TM would run after release, run BEFORE approval:
  /// the detour-vs-wait ETA only exists once the path was evaluated. True if
  /// it makes ECBS REPLAN this AMR (the decision is published here).
  bool probe_replans(const std::string & amr)
  {
    std::vector<ecbs_planner::ReplanDecision> decisions;
    std::vector<ecbs_planner::TimedPath> committed;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const auto path = cache_.get(amr);
      if (!path || path->waypoints.size() < 2) {
        return false;
      }
      ecbs_planner::srv::EvaluatePath::Request req;
      ecbs_planner::srv::EvaluatePath::Response res;
      req.mode = "evaluate";
      req.amr_id = amr;
      req.fleet_name = path->fleet_name;
      req.waypoints = ecbs_planner::from_zone_manager_msg(*path).waypoints;
      req.query_horizon_m = probe_query_horizon_m_;  // = TM query_horizon_m
      req.current_x = path->waypoints.front().x;
      req.current_y = path->waypoints.front().y;
      ecbs_planner::evaluate_path(
        req, res, cache_, *solver_, &committed, &junctions_,
        /*record_granted_base=*/false);
      decisions = replan_->update(cache_, now().seconds());
      collect_replan_changes(committed);
    }
    publish_committed(committed);
    emit_decisions(decisions);
    return std::any_of(
      decisions.begin(), decisions.end(), [&](const auto & d) {
        return d.kind == ecbs_planner::ReplanDecisionKind::Replan && d.candidate == amr;
      });
  }

  void publish_decisions(const std::vector<ecbs_planner::ReplanDecision> & decisions)
  {
    release_awaiting(decisions);
    emit_decisions(decisions);
  }

  void emit_decisions(const std::vector<ecbs_planner::ReplanDecision> & decisions)
  {
    if (replan_) {
      for (const auto & n : replan_->take_notes()) {
        RCLCPP_INFO(get_logger(), "replan check %s", n.c_str());
      }
    }
    for (const auto & d : decisions) {
      ecbs_planner::msg::EcbsReplan msg;
      msg.stamp = now();
      msg.amr_id = d.candidate;
      msg.task_id = d.task_id;
      msg.reason = d.reason;
      msg.conflicting_amrs = d.conflicting;
      switch (d.kind) {
        case ecbs_planner::ReplanDecisionKind::Replan:
          msg.decision = msg.DECISION_REPLAN;
          break;
        case ecbs_planner::ReplanDecisionKind::NoAlternative:
          msg.decision = msg.DECISION_NO_ALTERNATIVE;
          break;
        case ecbs_planner::ReplanDecisionKind::Clear:
          msg.decision = msg.DECISION_CLEAR;
          break;
      }
      msg.keep_amr = d.keeper;
      msg.blocked_waypoints = d.blocked_waypoints;
      msg.blocked_until_sec = d.blocked_until_sec;
      msg.revision = d.revision;
      replan_pub_->publish(msg);

      std::string wps;
      for (const auto w : d.blocked_waypoints) {
        wps += (wps.empty() ? "" : ",") + std::to_string(w);
      }
      RCLCPP_WARN(
        get_logger(),
        "/ecbs_replan_candidate rev=%u %s candidate=%s keep=%s blocked=[%s] (%s)",
        d.revision, msg.decision.c_str(), d.candidate.c_str(), d.keeper.c_str(),
        wps.c_str(), d.reason.c_str());
    }
  }

  /// Paths the replan coordinator changed (holds added or released), so
  /// /to_zone_manager never keeps showing a wait point that is gone.
  /// Call under mutex_.
  std::vector<std::string> collect_replan_changes(
    std::vector<ecbs_planner::TimedPath> & committed)
  {
    const auto touched = replan_->take_changed();
    for (const auto & amr : touched) {
      auto path = cache_.get(amr);
      if (!path) {
        continue;
      }
      auto it = std::find_if(
        committed.begin(), committed.end(),
        [&](const ecbs_planner::TimedPath & p) {return p.amr_id == amr;});
      if (it != committed.end()) {
        *it = std::move(*path);
      } else {
        committed.push_back(std::move(*path));
      }
    }
    return touched;
  }

  void handle_base_extension_query(
    const std::shared_ptr<ecbs_planner::srv::BaseExtensionQuery::Request> req,
    const std::shared_ptr<ecbs_planner::srv::BaseExtensionQuery::Response> res)
  {
    if (resolver_enabled_) {
      handle_query_resolver(req, res);
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    last_request_[req->amr_id] = std::chrono::steady_clock::now();
    ecbs_planner::ProposalQueryInput in;
    in.request_id = req->request_id;
    in.amr_id = req->amr_id;
    in.fleet_name = req->fleet_name;
    in.task_id = req->task_id;
    in.execution_order_id = req->execution_order_id;
    in.phase_index = req->phase_index;
    in.path_revision = req->path_revision;
    in.route_hash = req->route_hash;
    in.current_grant_through_index = req->current_grant_through_index;
    in.current_x = req->current_x;
    in.current_y = req->current_y;
    in.desired_extension_m = req->desired_extension_m;
    in.distance_origin = req->distance_origin.empty() ?
      "GRANT_TIP" : req->distance_origin;
    in.route_version = req->route_version;
    in.occupancy_version = req->occupancy_version;
    in.commitment_version = req->commitment_version;
    in.priority_version = req->priority_version;
    in.observed_progress.path_revision = req->observed_progress.path_revision;
    in.observed_progress.route_occurrence =
      req->observed_progress.route_occurrence;
    in.observed_progress.segment_offset_m =
      req->observed_progress.segment_offset_m;
    in.observed_progress.x = req->observed_progress.x;
    in.observed_progress.y = req->observed_progress.y;

    const double detect = solver_->config().clearance_m;
    const double eps = solver_->config().position_epsilon_m;
    const double safety = solver_->config().safety_distance_m;
    const auto result = ecbs_planner::query_base_extension(
      proposal_state_, cache_, in, detect, eps, safety);

    res->success = result.success;
    res->message = result.message;
    res->status = result.status;
    res->wait_reason = result.wait_reason;
    res->route_version = proposal_state_.route_version;
    res->occupancy_version = proposal_state_.occupancy_version;
    res->commitment_version = proposal_state_.commitment_version;
    res->priority_version = proposal_state_.priority_version;

    res->proposal.proposal_id = result.proposal_id;
    res->proposal.decision_id = result.decision_id;
    res->proposal.amr_id = req->amr_id;
    res->proposal.fleet_name = req->fleet_name;
    // Canonical identity from ScheduleCache / proposal — not reconstructed session order id.
    res->proposal.task_id = result.task_id.empty() ? req->task_id : result.task_id;
    res->proposal.execution_order_id = result.execution_order_id.empty() ?
      req->execution_order_id : result.execution_order_id;
    res->proposal.action_id = result.action_id;
    res->proposal.group_id = result.group_id;
    res->proposal.group_structure_version = result.group_structure_version;
    res->proposal.route_geometry_version = result.route_geometry_version;
    res->proposal.authority_version = result.authority_version;
    res->proposal.occupancy_clearance_version = result.occupancy_clearance_version;
    res->proposal.policy_priority_version = result.policy_priority_version;
    res->proposal.phase_index = req->phase_index;
    res->proposal.path_revision = result.from_occurrence.path_revision != 0 ?
      result.from_occurrence.path_revision : req->path_revision;
    res->proposal.status = result.status;
    res->proposal.decision_reason = result.decision_reason;
    res->proposal.wait_reason_code = result.wait_reason_code;
    res->proposal.proposed_horizon_m = result.proposed_horizon_m;
    res->proposal.from_occurrence.path_revision =
      result.from_occurrence.path_revision;
    res->proposal.from_occurrence.route_occurrence =
      result.from_occurrence.route_occurrence;
    res->proposal.from_occurrence.x = result.from_occurrence.x;
    res->proposal.from_occurrence.y = result.from_occurrence.y;
    res->proposal.to_occurrence.path_revision =
      result.to_occurrence.path_revision;
    res->proposal.to_occurrence.route_occurrence =
      result.to_occurrence.route_occurrence;
    res->proposal.to_occurrence.x = result.to_occurrence.x;
    res->proposal.to_occurrence.y = result.to_occurrence.y;
    res->proposal.priority_version = proposal_state_.priority_version;
    res->proposal.route_version = proposal_state_.route_version;
    res->proposal.occupancy_version = proposal_state_.occupancy_version;
    res->proposal.commitment_version = proposal_state_.commitment_version;
    res->proposal.required_conflict_ids = result.required_conflict_ids;

    auto fill_occ = [](ecbs_planner::msg::RouteOccurrence & m,
        const ecbs_planner::RouteOccurrence & o) {
        m.path_revision = o.path_revision;
        m.route_occurrence = o.route_occurrence;
        m.segment_offset_m = o.segment_offset_m;
        m.waypoint_index = o.waypoint_index;
        m.x = o.x;
        m.y = o.y;
        m.level_name = o.level_name;
      };
    for (const auto & rec : result.relevant_conflicts) {
      ecbs_planner::msg::ConflictRecord m;
      m.conflict_id = rec.conflict_id;
      m.fleet_name = rec.fleet_name;
      m.topology = rec.topology;
      m.wait_reason = rec.wait_reason;
      m.waiter_amr = rec.waiter_amr;
      m.blocker_amr = rec.blocker_amr;
      m.clearance_condition = rec.clearance_condition;
      m.mandatory = rec.mandatory;
      m.tentative = rec.tentative;
      m.physically_cleared = rec.physically_cleared;
      m.source_route_version = rec.source_route_version;
      m.source_occupancy_version = rec.source_occupancy_version;
      m.record_version = rec.record_version;
      for (const auto & p : rec.participants) {
        m.participant_amrs.push_back(p.amr_id);
        m.task_ids.push_back(p.task_id);
        m.phase_indices.push_back(p.phase_index);
        m.path_revisions.push_back(p.path_revision);
        m.blocks_other.push_back(p.blocks_other);
        m.blocked_by_other.push_back(p.blocked_by_other);
        ecbs_planner::msg::RouteOccurrence entry;
        fill_occ(entry, p.entry);
        m.entry_occurrence.push_back(entry);
        ecbs_planner::msg::RouteOccurrence exit;
        fill_occ(exit, p.exit);
        m.exit_occurrence.push_back(exit);
        ecbs_planner::msg::RouteOccurrence wait;
        fill_occ(wait, p.wait);
        m.wait_occurrence.push_back(wait);
      }
      res->relevant_conflicts.push_back(std::move(m));
    }

    for (const auto & dep : result.waiting_for) {
      ecbs_planner::msg::WaitDependency m;
      m.amr_id = dep.amr_id;
      m.topology = dep.topology;
      m.self_blocks_other = dep.self_blocks_other;
      m.other_blocks_self = dep.other_blocks_self;
      m.wait_point.x = dep.wait_point.x;
      m.wait_point.y = dep.wait_point.y;
      if (dep.wait_point.index) {
        m.wait_point.index = static_cast<int64_t>(*dep.wait_point.index);
      }
      for (const auto & c : dep.clear_points) {
        ecbs_planner::msg::ClearPoint cp;
        cp.x = c.x;
        cp.y = c.y;
        m.clear_points.push_back(cp);
      }
      res->waiting_for.push_back(m);
      res->proposal.waiting_constraints.push_back(m);
    }

    if (result.authority_conflict) {
      res->authority_conflict.amr_id = req->amr_id;
      res->authority_conflict.conflict_id = result.authority_conflict_id;
      res->authority_conflict.grant_id = result.grant_id_hit;
      res->authority_conflict.reason = result.message;
      res->authority_conflict.reservation_retained = true;
      res->authority_conflict.invoke_stop_protocol = true;
    }

    // Publish waits directly on /to_zone_manager for viz (not only after
    // EvaluatePath / PathRelease WAIT). Do not clear on FREE approach —
    // evaluate-stamped waits must stay until conflicts clear.
    if (!result.waiting_for.empty() ||
      result.status == ecbs_planner::kStatusWait)
    {
      publish_extension_wait_viz(req->amr_id, result.waiting_for);
    }

    RCLCPP_INFO(
      get_logger(),
      "BaseExtensionQuery amr=%s status=%s proposal=%lu waiters=%zu",
      req->amr_id.c_str(), res->status.c_str(),
      static_cast<unsigned long>(result.proposal_id),
      res->waiting_for.size());
  }

  /// Stamp BaseExtension waiting_for onto the cached path and publish
  /// /to_zone_manager so to_zone_manager_viz shows holds immediately.
  void publish_extension_wait_viz(
    const std::string & amr_id,
    const std::vector<ecbs_planner::WaitDependency> & waiting_for)
  {
    auto path_opt = cache_.get(amr_id);
    if (!path_opt || path_opt->waypoints.empty()) {
      return;
    }
    auto path = *path_opt;
    for (auto & wp : path.waypoints) {
      wp.waiting_for.clear();
      wp.wait_duration_sec = 0.0;
    }
    for (const auto & dep : waiting_for) {
      if (dep.amr_id.empty()) {
        continue;
      }
      std::size_t wait_at = 0;
      if (dep.wait_point.index.has_value()) {
        const auto local = ecbs_planner::local_index_for_grant_tip(
          path, static_cast<int32_t>(*dep.wait_point.index));
        wait_at = local ?
          static_cast<std::size_t>(*local) :
          ecbs_planner::nearest_waypoint_index(
            path, dep.wait_point.x, dep.wait_point.y);
      } else if (
        std::abs(dep.wait_point.x) > 1e-9 ||
        std::abs(dep.wait_point.y) > 1e-9)
      {
        wait_at = ecbs_planner::nearest_waypoint_index(
          path, dep.wait_point.x, dep.wait_point.y);
      }
      if (wait_at >= path.waypoints.size()) {
        wait_at = path.waypoints.size() - 1;
      }
      auto stamped = dep;
      if (std::abs(stamped.wait_point.x) < 1e-9 &&
        std::abs(stamped.wait_point.y) < 1e-9)
      {
        stamped.wait_point.x = path.waypoints[wait_at].x;
        stamped.wait_point.y = path.waypoints[wait_at].y;
      }
      if (stamped.wait_point.wait_duration_sec > 1e-6) {
        path.waypoints[wait_at].wait_duration_sec = std::max(
          path.waypoints[wait_at].wait_duration_sec,
          stamped.wait_point.wait_duration_sec);
      }
      path.waypoints[wait_at].waiting_for.push_back(std::move(stamped));
    }
    cache_.upsert(path);
    to_zone_pub_->publish(ecbs_planner::to_zone_manager_msg(path));
    RCLCPP_INFO(
      get_logger(),
      "Published /to_zone_manager amr=%s wps=%zu waits=%zu (extension viz)",
      amr_id.c_str(), path.waypoints.size(), waiting_for.size());
  }

  void handle_commit_proposal(
    const std::shared_ptr<ecbs_planner::srv::CommitProposal::Request> req,
    const std::shared_ptr<ecbs_planner::srv::CommitProposal::Response> res)
  {
    if (resolver_enabled_) {
      handle_commit_resolver(req, res);
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    ecbs_planner::RouteOccurrence from_occ;
    from_occ.path_revision = req->from_occurrence.path_revision;
    from_occ.route_occurrence = req->from_occurrence.route_occurrence;
    from_occ.segment_offset_m = req->from_occurrence.segment_offset_m;
    from_occ.x = req->from_occurrence.x;
    from_occ.y = req->from_occurrence.y;
    ecbs_planner::RouteOccurrence to_occ;
    to_occ.path_revision = req->to_occurrence.path_revision;
    to_occ.route_occurrence = req->to_occurrence.route_occurrence;
    to_occ.segment_offset_m = req->to_occurrence.segment_offset_m;
    to_occ.x = req->to_occurrence.x;
    to_occ.y = req->to_occurrence.y;
    if (req->status == ecbs_planner::kGrantCompleted) {
      res->result = ecbs_planner::retire_grant(
        proposal_state_, req->amr_id, req->grant_id, &cache_) ?
        "ACCEPTED" : "REJECTED_MISMATCH";
    } else {
      res->result = ecbs_planner::apply_commit(
        proposal_state_, req->proposal_id, req->grant_id, req->amr_id,
        req->status, req->release_through_index, from_occ, to_occ,
        req->path_revision, req->task_id, &cache_);
    }
    res->success = res->result == "ACCEPTED" || res->result == "DUPLICATE";
    res->message = res->result;
    res->commitment_version = proposal_state_.commitment_version;
  }

  void publish_committed(
    const std::vector<ecbs_planner::TimedPath> & committed)
  {
    if (resolver_enabled_) {
      return;  // GlobalResolver publishes /to_zone_manager (decided holds)
    }
    for (const auto & path : committed) {
      if (awaiting_approval_.count(path.amr_id)) {
        continue;  // not approved yet: keep it off the screen
      }
      auto msg = ecbs_planner::to_zone_manager_msg(path);
      to_zone_pub_->publish(msg);
      RCLCPP_INFO(
        get_logger(),
        "Published /to_zone_manager amr=%s wps=%zu",
        path.amr_id.c_str(), msg.path.size());
    }
    // Keep persistent overlaps in sync with seeded/committed paths.
    std::lock_guard<std::mutex> lock(mutex_);
    const double detect = solver_->config().clearance_m;
    const double eps = solver_->config().position_epsilon_m;
    const double safety = solver_->config().safety_distance_m;
    ecbs_planner::refresh_conflicts_from_cache(
      proposal_state_, cache_, detect, eps, safety);
  }

  // =====================================================================
  // GlobalResolver path — ECBS is the single traffic authority.
  //   TM PathIntent seed (EvaluatePath + waypoints) → register + decide now
  //   /fleet_states → progress → clearance → release events to TM
  //   BaseExtensionQuery → release slice capped by decided wait points
  //   CommitProposal → granted tip (re-validated, no foreign base)
  // =====================================================================
  static std::string lower(std::string v)
  {
    std::transform(v.begin(), v.end(), v.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
      });
    return v;
  }

  static std::string route_key_of(
    const std::string & fleet, const std::string & task, int phase, int rev)
  {
    return fleet + "|" + task + "|" + std::to_string(phase) + "|" + std::to_string(rev);
  }

  void push_junctions_to_resolver()
  {
    if (!resolver_) {
      return;
    }
    std::vector<std::pair<double, double>> centres;
    for (const auto & j : junctions_.junctions()) {
      centres.emplace_back(j.x, j.y);
    }
    resolver_->set_junctions(std::move(centres));
  }

  /// Logs, replan suggestions, release events and viz after any change.
  /// Caller holds mutex_.
  void flush_resolver_locked()
  {
    for (const auto & line : resolver_->take_log()) {
      if (line.rfind("[ECBS-UNRESOLVABLE]", 0) == 0) {
        RCLCPP_ERROR(get_logger(), "%s", line.c_str());
      } else {
        RCLCPP_INFO(get_logger(), "%s", line.c_str());
      }
    }
    for (const auto & rp : resolver_->take_replans()) {
      ecbs_planner::msg::EcbsReplan m;
      m.stamp = now();
      m.amr_id = rp.candidate;
      m.reason = rp.reason + " [" + rp.conflict_id + "]";
      m.conflicting_amrs = rp.robots;
      replan_pub_->publish(m);
    }
    for (const auto & amr : resolver_->take_changed()) {
      ecbs_planner::msg::ClearanceEvent ev;
      ev.stamp = now();
      ev.amr_id = amr;
      ev.physically_cleared = true;
      ev.reason = "ecbs_decision_update";
      release_event_pub_->publish(ev);
      publish_resolver_viz_locked(amr);
    }
  }

  /// /to_zone_manager view of the robot's route with its decided holds.
  void publish_resolver_viz_locked(const std::string & amr)
  {
    const auto pts = resolver_->route_points(amr);
    if (pts.empty()) {
      return;
    }
    ecbs_planner::msg::ToZoneManager msg;
    const auto fit = fleet_by_robot_.find(amr);
    msg.fleet_name = fit == fleet_by_robot_.end() ? std::string{} : fit->second;
    msg.amr_id = amr;
    msg.path.resize(pts.size());
    for (std::size_t i = 0; i < pts.size(); ++i) {
      msg.path[i].x = pts[i].x;
      msg.path[i].y = pts[i].y;
    }
    for (const auto & d : resolver_->decisions_for(amr)) {
      if (d.second != amr || d.cleared || d.wait_index < 0 ||
        static_cast<std::size_t>(d.wait_index) >= pts.size())
      {
        continue;
      }
      ecbs_planner::msg::WaitDependency dep;
      dep.amr_id = d.first;
      dep.topology = d.kind == ecbs_planner::OverlapKind::Opposite ?
        ecbs_planner::kTopoOpposite :
        (d.kind == ecbs_planner::OverlapKind::SameDirection ?
        ecbs_planner::kTopoSameWay : ecbs_planner::kTopoCross);
      dep.wait_point.x = d.wait_x;
      dep.wait_point.y = d.wait_y;
      dep.wait_point.index = d.wait_index;
      ecbs_planner::msg::ClearPoint cp;
      cp.x = d.release_x;
      cp.y = d.release_y;
      dep.clear_points.push_back(cp);
      dep.other_blocks_self = true;
      msg.path[static_cast<std::size_t>(d.wait_index)].waiting_for.push_back(dep);
    }
    to_zone_pub_->publish(msg);
  }

  void handle_evaluate_resolver(
    const std::shared_ptr<ecbs_planner::srv::EvaluatePath::Request> req,
    std::shared_ptr<ecbs_planner::srv::EvaluatePath::Response> res)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    res->amr_id = req->amr_id;
    res->task_id = req->task_id;
    res->path_revision = req->path_revision;
    res->status = "FREE";
    res->success = true;
    if (req->amr_id.empty()) {
      res->success = false;
      res->message = "amr_id is empty";
      return;
    }
    last_request_[req->amr_id] = std::chrono::steady_clock::now();
    if (!req->fleet_name.empty()) {
      fleet_by_robot_[req->amr_id] = req->fleet_name;
    }
    if (req->mode == "clear") {
      const auto p = poses_.find(req->amr_id);
      if (p != poses_.end()) {
        resolver_->set_standing(req->amr_id, p->second.first, p->second.second);
      }
      if (resolver_replan_) {
        cache_.erase(req->amr_id);
      }
      res->message = "route cleared — robot kept as standing obstacle";
      flush_resolver_locked();
      return;
    }
    if (req->waypoints.empty()) {
      res->message = "no waypoints — GlobalResolver registers PathIntent geometry only";
      return;
    }
    if (std::abs(req->current_x) > 1e-9 || std::abs(req->current_y) > 1e-9) {
      resolver_->update_pose(req->amr_id, req->current_x, req->current_y);
    }
    std::vector<ecbs_planner::ResolverRoutePoint> pts;
    pts.reserve(req->waypoints.size());
    for (const auto & w : req->waypoints) {
      pts.push_back({w.x, w.y});
    }
    // TM sends "<canonical task>#req<request_id>": route identity + session.
    std::string task = req->task_id;
    std::string session;
    const auto tag = task.find("#req");
    if (tag != std::string::npos) {
      session = task.substr(tag + 4);
      task = task.substr(0, tag);
    }
    const auto action = resolver_->register_intent(
      req->amr_id,
      route_key_of(req->fleet_name, task, req->phase_index, req->path_revision),
      session, pts);
    if (action == "awaiting_handover") {
      pending_since_.emplace(req->amr_id, std::chrono::steady_clock::now());
    } else {
      pending_since_.erase(req->amr_id);
    }
    res->message = "PathIntent " + action + " in GlobalResolver; decisions=" +
      std::to_string(resolver_->decisions_for(req->amr_id).size());
    flush_resolver_locked();
  }

  /// RMF handover: GoToPlace → EcbsGate → /from_zone_manager, before
  /// ExecutePlan::make. Register and decide every conflict right now.
  void handle_gate_seed_resolver(const ecbs_planner::msg::FromZoneManager::SharedPtr msg)
  {
    if (!msg || !accept_gate_seed_ || msg->waypoints.size() < 2) {
      return;
    }
    std::vector<ecbs_planner::ReplanDecision> decisions;
    const auto action = register_handover_and_check_replan(*msg, decisions);
    if (action == "ignored_stale") {
      return;  // duplicate copy: approval state of the live seed is unchanged
    }
    if (resolver_replan_ && replan_) {
      publish_approval(msg->amr_id, decisions);  // withheld while REPLAN pending
      publish_decisions(decisions);
    } else {
      approve(msg->amr_id);  // no replan check: release the seed at once
    }
  }

  /// Register + decide the handover in the GlobalResolver, then run the
  /// replan check on the same route. Returns the resolver action.
  std::string register_handover_and_check_replan(
    const ecbs_planner::msg::FromZoneManager & msg,
    std::vector<ecbs_planner::ReplanDecision> & decisions)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!msg.fleet_name.empty()) {
      fleet_by_robot_[msg.amr_id] = msg.fleet_name;
    }
    std::vector<ecbs_planner::ResolverRoutePoint> pts;
    pts.reserve(msg.waypoints.size());
    for (const auto & w : msg.waypoints) {
      pts.push_back({w.x, w.y});
    }
    const auto key = route_key_of(
      msg.fleet_name, msg.task_id, msg.phase_index, msg.path_revision);
    const auto action = resolver_->register_handover(msg.amr_id, key, pts);
    if (action == "ignored_stale") {
      RCLCPP_DEBUG(
        get_logger(), "RMF handover amr=%s key=[%s] ignored (stale copy)",
        msg.amr_id.c_str(), key.c_str());
    } else {
      if (action == "decided") {
        RCLCPP_INFO(
          get_logger(),
          "[ECBS-HANDOVER] %s key=[%s] wps=%zu — evaluated on RMF handover, decisions=%zu",
          msg.amr_id.c_str(), key.c_str(), pts.size(),
          resolver_->decisions_for(msg.amr_id).size());
      }
      // Same conflict picture → wait vs detour (ReplanCoordinator).
      decisions = replan_on_handover_locked(msg);
    }
    flush_resolver_locked();
    return action;
  }

  void on_pending_intent_timer()
  {
    if (!resolver_enabled_) {
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    const auto now_tp = std::chrono::steady_clock::now();
    bool any = false;
    // Recovery: re-check unresolvable conflicts every ~2 s with live poses.
    if (++retry_tick_ >= 8) {
      retry_tick_ = 0;
      any = !resolver_->retry_unresolvable().empty();
      // Robots ahead moved on: pull pending waits up to the nearest safe waypoint.
      any = resolver_->refine_waits() || any;
    }
    for (auto it = pending_since_.begin(); it != pending_since_.end(); ) {
      if (!resolver_->has_pending_intent(it->first)) {
        it = pending_since_.erase(it);  // handover arrived and bound it
        continue;
      }
      const double waited = std::chrono::duration<double>(now_tp - it->second).count();
      if (waited >= handover_wait_timeout_sec_) {
        RCLCPP_WARN(
          get_logger(),
          "[ECBS-HANDOVER] %s: no RMF handover %.1fs after its PathIntent — "
          "deciding on the PathIntent (is the fleet-adapter EcbsGate running with "
          "ecbs_external_evaluate_server=true?)", it->first.c_str(), waited);
        resolver_->decide_pending_intent(it->first);
        it = pending_since_.erase(it);
        any = true;
        continue;
      }
      ++it;
    }
    if (any) {
      flush_resolver_locked();
    }
  }

  void handle_fleet_resolver(const rmf_fleet_msgs::msg::FleetState::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto & robot : msg->robots) {
      poses_[robot.name] = {robot.location.x, robot.location.y};
      if (!fleet_by_robot_.count(robot.name)) {
        fleet_by_robot_[robot.name] = msg->name;
      }
      auto & f = facts_[robot.name];
      // zone_manager is_robot_healthy: not EMERGENCY (5) / ADAPTER_ERROR (8).
      f.healthy = robot.mode.mode != rmf_fleet_msgs::msg::RobotMode::MODE_EMERGENCY &&
        robot.mode.mode != rmf_fleet_msgs::msg::RobotMode::MODE_ADAPTER_ERROR;
      resolver_->set_facts(robot.name, f);
      resolver_->update_pose(robot.name, robot.location.x, robot.location.y);
      if (resolver_replan_) {
        mirror_fleet_robot_locked(robot, std::chrono::steady_clock::now());
      }
    }
    flush_resolver_locked();
  }

  void handle_monitored_resolver(const monitor::msg::MonitoredTaskDetails::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto & f = facts_[msg->amr_id];
    f.priority_rank = ecbs_planner::task_priority_rank(lower(msg->priority));
    const auto load = lower(msg->load_status);
    f.loaded = load == "loaded" || load == "4" || load == "true" || load == "full";
    resolver_->set_facts(msg->amr_id, f);
  }

  void handle_query_resolver(
    const std::shared_ptr<ecbs_planner::srv::BaseExtensionQuery::Request> req,
    const std::shared_ptr<ecbs_planner::srv::BaseExtensionQuery::Response> res)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    last_request_[req->amr_id] = std::chrono::steady_clock::now();
    if (req->current_x != 0.0 || req->current_y != 0.0) {
      resolver_->update_pose(req->amr_id, req->current_x, req->current_y);
    }
    const std::string session_key = req->task_id.empty() ? std::string() :
      route_key_of(
      req->fleet_name, req->task_id, req->phase_index,
      static_cast<int>(req->path_revision));
    const auto ans = resolver_->query(
      req->amr_id, req->current_grant_through_index, req->desired_extension_m,
      session_key);
    res->success = true;
    res->message = ans.reason;
    auto & p = res->proposal;
    p.amr_id = req->amr_id;
    p.fleet_name = req->fleet_name;
    p.task_id = req->task_id;
    p.execution_order_id = req->execution_order_id;
    p.phase_index = req->phase_index;
    p.path_revision = req->path_revision;
    p.decision_reason = ans.reason;
    p.required_conflict_ids = ans.conflict_ids;
    for (const auto & dep : ans.waiting_for) {
      ecbs_planner::msg::WaitDependency m;
      m.amr_id = dep.amr_id;
      m.topology = dep.topology;
      m.self_blocks_other = dep.self_blocks_other;
      m.other_blocks_self = dep.other_blocks_self;
      m.wait_point.x = dep.wait_point.x;
      m.wait_point.y = dep.wait_point.y;
      m.wait_point.index = dep.wait_point.index ?
        static_cast<int64_t>(*dep.wait_point.index) : -1;
      for (const auto & c : dep.clear_points) {
        ecbs_planner::msg::ClearPoint cp;
        cp.x = c.x;
        cp.y = c.y;
        m.clear_points.push_back(cp);
      }
      res->waiting_for.push_back(m);
      p.waiting_constraints.push_back(m);
    }
    switch (ans.status) {
      case ecbs_planner::ResolverStatus::Free:
        res->status = ecbs_planner::kStatusFreeProposal;
        p.proposal_id = ans.proposal_id;
        p.decision_id = ans.proposal_id;
        p.action_id = "resolver:" + req->amr_id + ":" + std::to_string(ans.from_index) +
          ":" + std::to_string(ans.through_index);
        p.proposed_horizon_m = ans.proposed_m;
        p.from_occurrence.path_revision = req->path_revision;
        p.from_occurrence.route_occurrence = static_cast<uint32_t>(std::max(0, ans.from_index));
        p.from_occurrence.waypoint_index = std::max(0, ans.from_index);
        p.from_occurrence.x = ans.from_x;
        p.from_occurrence.y = ans.from_y;
        p.to_occurrence.path_revision = req->path_revision;
        p.to_occurrence.route_occurrence = static_cast<uint32_t>(ans.through_index);
        p.to_occurrence.waypoint_index = ans.through_index;
        p.to_occurrence.x = ans.through_x;
        p.to_occurrence.y = ans.through_y;
        break;
      case ecbs_planner::ResolverStatus::Wait:
        res->status = ecbs_planner::kStatusWait;
        res->wait_reason = ans.waiting_for.empty() ? std::string{} :
          ecbs_planner::wait_reason_from_topology(ans.waiting_for.front().topology);
        p.wait_reason_code = ecbs_planner::kWaitClearance;
        break;
      case ecbs_planner::ResolverStatus::RouteComplete:
        res->status = "ROUTE_COMPLETE";
        break;
      case ecbs_planner::ResolverStatus::UnknownRoute:
        res->status = ecbs_planner::kStatusStaleRoute;
        break;
      case ecbs_planner::ResolverStatus::Recovery:
        res->status = ecbs_planner::kStatusRecoveryRequired;
        break;
    }
    p.status = res->status;

    // Log only when the answer changes (no per-poll spam).
    std::string sig = res->status + "@" + std::to_string(ans.through_index);
    for (const auto & d : ans.waiting_for) {
      sig += "|" + d.amr_id;
    }
    auto & last = last_answer_sig_[req->amr_id];
    if (last != sig) {
      last = sig;
      std::string blockers;
      for (const auto & d : ans.waiting_for) {
        blockers += (blockers.empty() ? "" : ",") + d.amr_id;
      }
      RCLCPP_INFO(
        get_logger(),
        "[ECBS-RELEASE] %s %s tip=%d → through=%d blockers=[%s] (%s)",
        req->amr_id.c_str(), res->status.c_str(), req->current_grant_through_index,
        ans.through_index, blockers.c_str(), ans.reason.c_str());
    }
    flush_resolver_locked();
  }

  void handle_commit_resolver(
    const std::shared_ptr<ecbs_planner::srv::CommitProposal::Request> req,
    const std::shared_ptr<ecbs_planner::srv::CommitProposal::Response> res)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    res->result = resolver_->commit(
      req->amr_id, req->proposal_id, req->release_through_index, req->status);
    res->success = res->result == "ACCEPTED" || res->result == "DUPLICATE";
    res->message = res->result;
    if (res->result == "ACCEPTED" && req->status == "COMMITTED") {
      ++resolver_commitment_version_;
    }
    res->commitment_version = resolver_commitment_version_;
    if (req->status == "COMMITTED") {
      RCLCPP_INFO(
        get_logger(), "[ECBS-COMMIT] %s proposal=%lu through=%d → %s",
        req->amr_id.c_str(), static_cast<unsigned long>(req->proposal_id),
        req->release_through_index, res->result.c_str());
    }
    flush_resolver_locked();
  }

  bool resolver_enabled_{true};
  bool accept_gate_seed_{false};
  std::unique_ptr<ecbs_planner::GlobalResolver> resolver_;
  rclcpp::Publisher<ecbs_planner::msg::ClearanceEvent>::SharedPtr release_event_pub_;
  rclcpp::Publisher<ecbs_planner::msg::EcbsReplan>::SharedPtr replan_pub_;
  std::unordered_map<std::string, std::pair<double, double>> poses_;
  std::unordered_map<std::string, std::string> fleet_by_robot_;
  std::unordered_map<std::string, ecbs_planner::ResolverFacts> facts_;
  std::unordered_map<std::string, std::string> last_answer_sig_;
  uint64_t resolver_commitment_version_{0};
  double handover_wait_timeout_sec_{2.0};
  int retry_tick_{0};
  rclcpp::TimerBase::SharedPtr pending_timer_;
  std::unordered_map<std::string, std::chrono::steady_clock::time_point> pending_since_;

  std::mutex mutex_;
  ecbs_planner::ScheduleCache cache_;
  ecbs_planner::ProposalEngineState proposal_state_;
  ecbs_planner::NavGraphJunctions junctions_;
  std::unique_ptr<ecbs_planner::EcbsSolver> solver_;
  rclcpp::Service<ecbs_planner::srv::EvaluatePath>::SharedPtr server_;
  rclcpp::Service<ecbs_planner::srv::BaseExtensionQuery>::SharedPtr
    base_query_server_;
  rclcpp::Service<ecbs_planner::srv::CommitProposal>::SharedPtr commit_server_;
  rclcpp::Subscription<ecbs_planner::msg::FromZoneManager>::SharedPtr
    from_zone_sub_;
  rclcpp::Publisher<ecbs_planner::msg::ToZoneManager>::SharedPtr to_zone_pub_;
  rclcpp::Subscription<rmf_fleet_msgs::msg::FleetState>::SharedPtr fleet_sub_;
  rclcpp::Subscription<rmf_building_map_msgs::msg::BuildingMap>::SharedPtr
    building_map_sub_;
  rclcpp::Subscription<monitor::msg::MonitoredTaskDetails>::SharedPtr
    monitored_sub_;
  rclcpp::Subscription<ecbs_planner::msg::ClearanceEvent>::SharedPtr
    clearance_sub_;
  rclcpp::Subscription<ecbs_planner::msg::GrantEvent>::SharedPtr
    grant_event_sub_;
  /// amr → last evaluate / path seed received for it.
  std::unordered_map<std::string, std::chrono::steady_clock::time_point> last_request_;
  std::chrono::duration<double> idle_after_{10.0};
  std::unique_ptr<ecbs_planner::ReplanCoordinator> replan_;
  /// Resolver mode + replan_: RMF handovers are mirrored into cache_ so the
  /// ReplanCoordinator sees the same routes the GlobalResolver decides.
  bool resolver_replan_{false};
  /// Pre-approval evaluate horizon (= traffic_manager query_horizon_m).
  double probe_query_horizon_m_{10.0};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr approval_pub_;
  struct Awaiting {double since; double ready_at; bool replanned;};
  std::map<std::string, Awaiting> awaiting_approval_;
  rclcpp::TimerBase::SharedPtr replan_timer_;
  ecbs_planner::IdleTracker idle_tracker_;
  ecbs_planner::IdleHoldConfig idle_cfg_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EvaluatePathServer>());
  rclcpp::shutdown();
  return 0;
}
