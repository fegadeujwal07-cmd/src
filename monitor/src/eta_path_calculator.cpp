#include "monitor/eta_path_calculator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace monitor
{
namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kEps = 1e-9;

double wrap_pi(double a)
{
  while (a > kPi) {
    a -= 2.0 * kPi;
  }
  while (a < -kPi) {
    a += 2.0 * kPi;
  }
  return a;
}

double heading_rad(double x0, double y0, double x1, double y1)
{
  return std::atan2(y1 - y0, x1 - x0);
}

double seg_len(
  const std::pair<double, double> & a,
  const std::pair<double, double> & b)
{
  return std::hypot(b.first - a.first, b.second - a.second);
}

/// Time to travel distance d from v_in to end at v_out, never exceeding v_max.
double trapezoid_time(
  double d,
  double v_in,
  double v_max,
  double v_out,
  double a_acc,
  double a_dec)
{
  if (d <= kEps) {
    return 0.0;
  }
  v_in = std::max(0.0, v_in);
  v_out = std::max(0.0, v_out);
  v_max = std::max({v_max, v_in, v_out, 0.05});
  a_acc = std::max(a_acc, 0.01);
  a_dec = std::max(a_dec, 0.01);

  // Max peak reachable in distance d with given entry/exit speeds.
  const double numer =
    2.0 * a_acc * a_dec * d + a_dec * v_in * v_in + a_acc * v_out * v_out;
  const double denom = a_acc + a_dec;
  double v_peak = std::sqrt(std::max(numer / denom, 0.0));
  v_peak = std::min(v_peak, v_max);
  v_peak = std::max(v_peak, std::min(v_in, v_out));

  // If even connecting v_in -> v_out needs more than d, use average speed estimate.
  const double d_direct =
    std::abs(v_out * v_out - v_in * v_in) / (2.0 * (v_out >= v_in ? a_acc : a_dec));
  if (d_direct > d + 1e-6 && std::abs(v_out - v_in) > kEps) {
    const double v_avg = 0.5 * (v_in + v_out);
    return d / std::max(v_avg, 0.05);
  }

  const double d_acc = std::max(0.0, (v_peak * v_peak - v_in * v_in) / (2.0 * a_acc));
  const double d_dec = std::max(0.0, (v_peak * v_peak - v_out * v_out) / (2.0 * a_dec));
  double d_cruise = d - d_acc - d_dec;
  if (d_cruise < 0.0) {
    // Recompute peak for exact fit (no cruise).
    v_peak = std::sqrt(std::max(numer / denom, 0.0));
    v_peak = std::min(v_peak, v_max);
    const double d_acc2 = std::max(0.0, (v_peak * v_peak - v_in * v_in) / (2.0 * a_acc));
    const double d_dec2 = std::max(0.0, d - d_acc2);
    const double t_acc = std::abs(v_peak - v_in) / a_acc;
    // Exit speed implied by remaining decel distance
    const double v_exit_sq = std::max(0.0, v_peak * v_peak - 2.0 * a_dec * d_dec2);
    const double v_exit = std::sqrt(v_exit_sq);
    const double t_dec = std::abs(v_peak - v_exit) / a_dec;
    return t_acc + t_dec;
  }

  const double t_acc = std::abs(v_peak - v_in) / a_acc;
  const double t_dec = std::abs(v_peak - v_out) / a_dec;
  const double t_cruise = d_cruise / std::max(v_peak, 0.05);
  return t_acc + t_cruise + t_dec;
}

}  // namespace

std::vector<std::pair<double, double>> remaining_path_points(
  double robot_x,
  double robot_y,
  const std::vector<std::pair<double, double>> & planned_path)
{
  std::vector<std::pair<double, double>> out;
  if (planned_path.empty()) {
    return out;
  }
  if (planned_path.size() == 1) {
    out.push_back({robot_x, robot_y});
    out.push_back(planned_path.front());
    return out;
  }

  double best_dist = std::numeric_limits<double>::infinity();
  std::size_t best_seg = 0;
  double best_t = 0.0;
  for (std::size_t i = 0; i + 1 < planned_path.size(); ++i) {
    const double x0 = planned_path[i].first;
    const double y0 = planned_path[i].second;
    const double x1 = planned_path[i + 1].first;
    const double y1 = planned_path[i + 1].second;
    const double dx = x1 - x0;
    const double dy = y1 - y0;
    const double len2 = dx * dx + dy * dy;
    double t = 0.0;
    if (len2 > kEps) {
      t = ((robot_x - x0) * dx + (robot_y - y0) * dy) / len2;
      t = std::clamp(t, 0.0, 1.0);
    }
    const double px = x0 + t * dx;
    const double py = y0 + t * dy;
    const double dist = std::hypot(robot_x - px, robot_y - py);
    if (dist < best_dist) {
      best_dist = dist;
      best_seg = i;
      best_t = t;
    }
  }

  const double x0 = planned_path[best_seg].first;
  const double y0 = planned_path[best_seg].second;
  const double x1 = planned_path[best_seg + 1].first;
  const double y1 = planned_path[best_seg + 1].second;
  out.emplace_back(x0 + best_t * (x1 - x0), y0 + best_t * (y1 - y0));
  for (std::size_t i = best_seg + 1; i < planned_path.size(); ++i) {
    out.push_back(planned_path[i]);
  }
  return out;
}

std::vector<double> path_point_reach_times_sec(
  const std::vector<std::pair<double, double>> & path,
  const EtaRobotParams & params,
  double start_speed_mps)
{
  std::vector<double> reach(path.size(), 0.0);
  if (path.size() < 2) {
    return reach;
  }

  const double v_op = std::max(params.operating_linear_speed_mps, 0.05);
  const double v_curve = std::max(
    std::min(params.curve_linear_speed_mps, v_op), 0.05);
  const double a_acc = std::max(params.linear_accel_mps2, 0.01);
  const double a_dec = std::max(params.linear_decel_mps2, 0.01);
  const double heading_thresh_rad =
    params.heading_change_threshold_deg * kPi / 180.0;
  const double approach_m = std::max(params.approach_start_distance_m, 0.0);

  const std::size_t n_seg = path.size() - 1;
  std::vector<double> lengths(n_seg, 0.0);
  std::vector<double> headings(n_seg, 0.0);
  double total_len = 0.0;
  for (std::size_t i = 0; i < n_seg; ++i) {
    lengths[i] = seg_len(path[i], path[i + 1]);
    headings[i] = heading_rad(
      path[i].first, path[i].second, path[i + 1].first, path[i + 1].second);
    total_len += lengths[i];
  }
  if (total_len <= kEps) {
    return reach;
  }

  // Mark vertices where heading change exceeds threshold (curve corners).
  std::vector<bool> curve_vertex(path.size(), false);
  for (std::size_t i = 1; i + 1 < path.size(); ++i) {
    const double d_heading = std::abs(wrap_pi(headings[i] - headings[i - 1]));
    if (d_heading > heading_thresh_rad) {
      curve_vertex[i] = true;
    }
  }

  // Per-segment target cruise speed (curve vs straight).
  std::vector<double> v_target(n_seg, v_op);
  for (std::size_t i = 0; i < n_seg; ++i) {
    if (curve_vertex[i] || curve_vertex[i + 1]) {
      v_target[i] = v_curve;
    }
  }

  // Distance from start of each segment to path end.
  std::vector<double> dist_to_end(n_seg, 0.0);
  double acc = 0.0;
  for (std::size_t i = n_seg; i-- > 0;) {
    acc += lengths[i];
    dist_to_end[i] = acc;
  }

  // Soften speed before upcoming curve (approach window).
  for (std::size_t i = 0; i < n_seg; ++i) {
    if (v_target[i] <= v_curve + kEps) {
      continue;
    }
    // Look ahead for a curve segment within approach_m.
    double ahead = 0.0;
    for (std::size_t j = i + 1; j < n_seg; ++j) {
      ahead += lengths[j - 1];
      if (ahead > approach_m) {
        break;
      }
      if (v_target[j] <= v_curve + kEps) {
        v_target[i] = v_curve;
        break;
      }
    }
  }

  // Desired exit speed at end of each segment.
  // Stop only at the real last point. A long stop table must not force every
  // point in the last few meters down to 0, or those meters are counted as a
  // full stop-and-go and the live time sits near 10 s until arrival.
  std::vector<double> v_exit(n_seg, v_op);
  for (std::size_t i = 0; i + 1 < n_seg; ++i) {
    v_exit[i] = v_target[i + 1];
    const double dist_after = std::max(0.0, dist_to_end[i] - lengths[i]);
    const double v_brake = std::sqrt(std::max(0.0, 2.0 * a_dec * dist_after));
    v_exit[i] = std::min(v_exit[i], v_brake);
  }
  v_exit[n_seg - 1] = 0.0;

  double v = std::clamp(start_speed_mps, 0.0, v_op);
  for (std::size_t i = 0; i < n_seg; ++i) {
    const double v_max = v_target[i];
    const double v_out = std::min(v_exit[i], v_max);
    reach[i + 1] = reach[i] + trapezoid_time(lengths[i], v, v_max, v_out, a_acc, a_dec);
    v = v_out;
  }

  // Turn time belongs to a sharp corner only. A curve hop changes heading
  // while the robot is moving, so it must not add a stop-and-spin.
  // Flat pad for real stop-and-pivot: ~90° → 20 s, ~180° → 20 s.
  constexpr double kPivotTolDeg = 20.0;
  const double w_max = std::max(params.angular_speed_max_rps, 0.01);
  const double alpha = std::max(params.angular_accel_rps2, 0.01);
  const double sharp_rad = params.heading_max_deg * kPi / 180.0;
  for (std::size_t i = 1; i + 1 < path.size(); ++i) {
    const double d_heading = std::abs(wrap_pi(headings[i] - headings[i - 1]));
    if (d_heading <= sharp_rad) {
      continue;
    }
    const double t_spin_acc = std::min(std::sqrt(d_heading / alpha), w_max / alpha);
    const double theta_acc = 0.5 * alpha * t_spin_acc * t_spin_acc;
    double t_turn = 2.0 * t_spin_acc;
    if (2.0 * theta_acc < d_heading) {
      t_turn += (d_heading - 2.0 * theta_acc) / w_max;
    }
    const double deg = d_heading * 180.0 / kPi;
    if (deg >= (180.0 - kPivotTolDeg) ||
      std::abs(deg - 90.0) <= kPivotTolDeg)
    {
      t_turn += 20.0;
    }
    for (std::size_t j = i; j < reach.size(); ++j) {
      reach[j] += t_turn;
    }
  }

  return reach;
}

double estimate_path_travel_time_sec(
  const std::vector<std::pair<double, double>> & path,
  const EtaRobotParams & params,
  double start_speed_mps)
{
  const std::vector<double> reach =
    path_point_reach_times_sec(path, params, start_speed_mps);
  if (reach.empty()) {
    return 0.0;
  }
  return std::max(0.0, reach.back());
}

}  // namespace monitor
