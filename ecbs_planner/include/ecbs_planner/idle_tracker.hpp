#ifndef ECBS_PLANNER__IDLE_TRACKER_HPP_
#define ECBS_PLANNER__IDLE_TRACKER_HPP_

#include <cmath>
#include <optional>
#include <string>
#include <unordered_map>

namespace ecbs_planner
{

/// How long empty-task_id / stationary-no-path must hold before idle.
struct IdleHoldConfig
{
  double empty_task_sec{3.0};  // rule (1): empty task_id continuously
  double hold_sec{3.0};        // rule (2): not moving + no ECBS path
  double pos_tol_m{0.15};      // motion below this does not reset hold_sec
};

enum class IdleChange
{
  None,
  BecameIdleByHold,  // rule (2) newly true (log once)
  LeftIdle,
};

struct IdleUpdateResult
{
  bool idle{false};
  IdleChange change{IdleChange::None};
};

/// Tracks per-AMR idle for ReplanCoordinator parked-keeper detection.
/// Rule (1): empty task_id for >= empty_task_sec.
/// Rule (2): not MODE_MOVING, position within pos_tol_m, no live ECBS path,
///           continuously for >= hold_sec.
class IdleTracker
{
public:
  IdleUpdateResult update(
    const std::string & amr,
    double now_sec,
    bool task_id_empty,
    bool mode_moving,
    bool has_live_path,
    double x,
    double y,
    const IdleHoldConfig & cfg)
  {
    auto & s = by_amr_[amr];
    if (!s.have_pose) {
      s.last_x = x;
      s.last_y = y;
      s.have_pose = true;
    }

    const double dist = std::hypot(x - s.last_x, y - s.last_y);
    const bool moved = dist > cfg.pos_tol_m;
    if (moved) {
      s.last_x = x;
      s.last_y = y;
    }

    // Rule (1): empty task_id continuously (emplace keeps first empty time).
    if (!task_id_empty) {
      s.empty_task_since.reset();
    } else if (!s.empty_task_since) {
      s.empty_task_since = now_sec;
    }
    const bool idle_by_empty = s.empty_task_since &&
      (now_sec - *s.empty_task_since >= cfg.empty_task_sec);

    // Rule (2): reset whenever driving, path still live, or moved past tol.
    const bool hold_ok = !mode_moving && !has_live_path && !moved;
    if (!hold_ok) {
      s.hold_since.reset();
      if (mode_moving || has_live_path) {
        s.last_x = x;
        s.last_y = y;
      }
    } else if (!s.hold_since) {
      s.hold_since = now_sec;
    }
    const bool idle_by_hold = s.hold_since &&
      (now_sec - *s.hold_since >= cfg.hold_sec);

    const bool idle = idle_by_empty || idle_by_hold;
    IdleUpdateResult out;
    out.idle = idle;
    if (idle && !s.was_idle && idle_by_hold) {
      out.change = IdleChange::BecameIdleByHold;
    } else if (!idle && s.was_idle) {
      out.change = IdleChange::LeftIdle;
    }
    s.was_idle = idle;
    return out;
  }

private:
  struct State
  {
    std::optional<double> empty_task_since;
    std::optional<double> hold_since;
    double last_x{0.0};
    double last_y{0.0};
    bool have_pose{false};
    bool was_idle{false};
  };
  std::unordered_map<std::string, State> by_amr_;
};

}  // namespace ecbs_planner

#endif  // ECBS_PLANNER__IDLE_TRACKER_HPP_
