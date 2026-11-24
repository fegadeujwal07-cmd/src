#include <algorithm>
#include <cmath>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <ecbs_planner/msg/to_zone_manager.hpp>

namespace
{

visualization_msgs::msg::Marker base_marker(
  const std::string & frame_id,
  const std::string & ns,
  int32_t id,
  int32_t type,
  float r, float g, float b, float a)
{
  visualization_msgs::msg::Marker m;
  m.header.frame_id = frame_id;
  m.ns = ns;
  m.id = id;
  m.type = type;
  m.action = visualization_msgs::msg::Marker::ADD;
  m.pose.orientation.w = 1.0;
  m.scale.x = 0.12;
  m.scale.y = 0.12;
  m.scale.z = 0.12;
  m.color.r = r;
  m.color.g = g;
  m.color.b = b;
  m.color.a = a;
  m.lifetime = rclcpp::Duration(0, 0);  // persistent until replaced
  return m;
}

geometry_msgs::msg::Point to_point(double x, double y)
{
  geometry_msgs::msg::Point p;
  p.x = x;
  p.y = y;
  p.z = 0.05;
  return p;
}

}  // namespace

/// Subscribes to /to_zone_manager, logs paths that have wait dependencies,
/// and publishes RViz markers:
///   - red  = shared cells (clear_points near the waiting robot's own path);
///     only neighbouring points are joined
///   - blue = one waiting point per robot: its first wait (where it stops),
///     labelled with every AMR it waits for.
class ToZoneVizNode : public rclcpp::Node
{
public:
  ToZoneVizNode()
  : Node("to_zone_manager_viz")
  {
    declare_parameter<std::string>("topic", "/to_zone_manager");
    declare_parameter<std::string>("marker_topic", "/ecbs_conflict_markers");
    declare_parameter<std::string>("frame_id", "map");
    declare_parameter<double>("wait_marker_scale", 0.35);
    declare_parameter<double>("overlap_line_width", 0.18);
    // Clear points farther than this from the waiting robot's path are not
    // shared cells and are not drawn.
    declare_parameter<double>("overlap_near_path_m", 2.0);
    // Red line only joins clear points closer than this to each other.
    declare_parameter<double>("overlap_max_gap_m", 2.5);

    topic_ = get_parameter("topic").as_string();
    marker_topic_ = get_parameter("marker_topic").as_string();
    frame_id_ = get_parameter("frame_id").as_string();
    wait_scale_ = get_parameter("wait_marker_scale").as_double();
    overlap_width_ = get_parameter("overlap_line_width").as_double();
    near_path_m_ = get_parameter("overlap_near_path_m").as_double();
    max_gap_m_ = get_parameter("overlap_max_gap_m").as_double();

    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      marker_topic_, rclcpp::QoS(rclcpp::KeepLast(10)).transient_local());

    sub_ = create_subscription<ecbs_planner::msg::ToZoneManager>(
      topic_,
      rclcpp::QoS(rclcpp::KeepLast(50)).reliable(),
      std::bind(&ToZoneVizNode::on_path, this, std::placeholders::_1));

    // Rebuild markers periodically so late RViz subscribers still see them.
    timer_ = create_wall_timer(
      std::chrono::seconds(1),
      [this]() {publish_markers();});

    RCLCPP_INFO(
      get_logger(),
      "to_zone_manager_viz listening on %s → markers on %s (frame=%s). "
      "Red=overlap corridor, Blue=wait points.",
      topic_.c_str(), marker_topic_.c_str(), frame_id_.c_str());
  }

private:
  void on_path(const ecbs_planner::msg::ToZoneManager::SharedPtr msg)
  {
    if (!msg || msg->amr_id.empty()) {
      return;
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      paths_[msg->amr_id] = *msg;
    }

    print_if_dependencies(*msg);
    publish_markers();
  }

  void print_if_dependencies(const ecbs_planner::msg::ToZoneManager & msg) const
  {
    bool any = false;
    for (std::size_t i = 0; i < msg.path.size(); ++i) {
      const auto & wp = msg.path[i];
      if (wp.wait_sec <= 1e-6 && wp.waiting_for.empty()) {
        continue;
      }
      any = true;
      RCLCPP_INFO(
        get_logger(),
        "────────────────────────────────────────");
      RCLCPP_INFO(
        get_logger(),
        "DEPENDENCY  amr=%s  fleet=%s  waypoint[%zu]=(%.2f, %.2f)  wait_sec=%.2f",
        msg.amr_id.c_str(),
        msg.fleet_name.c_str(),
        i, wp.x, wp.y, wp.wait_sec);

      for (const auto & dep : wp.waiting_for) {
        std::string clears;
        for (std::size_t c = 0; c < dep.clear_points.size(); ++c) {
          if (c > 0) {
            clears += ", ";
          }
          clears += "(" + std::to_string(dep.clear_points[c].x) + ", " +
            std::to_string(dep.clear_points[c].y) + ")";
        }
        RCLCPP_INFO(
          get_logger(),
          "  waiting_for amr=%s  topo=%s easiness(self_blocks=%d other_blocks=%d) "
          "wait_point=(%.2f, %.2f) idx=%ld  clear_points=[%s]",
          dep.amr_id.c_str(),
          dep.topology.empty() ? "-" : dep.topology.c_str(),
          dep.self_blocks_other ? 1 : 0,
          dep.other_blocks_self ? 1 : 0,
          dep.wait_point.x,
          dep.wait_point.y,
          static_cast<long>(dep.wait_point.index),
          clears.empty() ? "(none)" : clears.c_str());
      }
    }

    if (!any) {
      RCLCPP_DEBUG(
        get_logger(),
        "Path amr=%s has no wait dependencies (%zu waypoints)",
        msg.amr_id.c_str(), msg.path.size());
    } else {
      RCLCPP_INFO(
        get_logger(),
        "────────────────────────────────────────");
    }
  }

  void publish_markers()
  {
    visualization_msgs::msg::MarkerArray arr;
    const auto stamp = now();

    // No DELETEALL: RViz draws it before the re-adds and every rebuild
    // blinks. ADD replaces a marker with the same (ns, id) in place; ids
    // left over from a larger previous rebuild are deleted at the end.
    // Independent id sequences per namespace, starting at 1.
    int32_t path_id = 1;
    int32_t wait_id = 1;
    int32_t overlap_id = 1;

    std::map<std::string, ecbs_planner::msg::ToZoneManager> snapshot;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      snapshot = paths_;
    }

    for (const auto & kv : snapshot) {
      const auto & msg = kv.second;

      // Full path (light gray) for context.
      if (msg.path.size() >= 2) {
        auto line = base_marker(
          frame_id_, "ecbs_path", path_id++,
          visualization_msgs::msg::Marker::LINE_STRIP,
          0.7f, 0.7f, 0.7f, 0.5f);
        line.header.stamp = stamp;
        line.scale.x = 0.06;
        for (const auto & wp : msg.path) {
          line.points.push_back(to_point(wp.x, wp.y));
        }
        arr.markers.push_back(std::move(line));
      }

      // One waiting point per robot: earliest dep.wait_point along the path
      // (after ECBS enforce, all deps share that stop — never inside base).
      std::optional<std::size_t> best_i;
      double best_wait_x = 0.0;
      double best_wait_y = 0.0;
      double best_wait_sec = 0.0;
      double best_along = 1e100;
      auto path_along = [&](double x, double y) {
          double best_d = 1e100;
          std::size_t best_j = 0;
          for (std::size_t j = 0; j < msg.path.size(); ++j) {
            const double dx = msg.path[j].x - x;
            const double dy = msg.path[j].y - y;
            const double d = dx * dx + dy * dy;
            if (d < best_d) {
              best_d = d;
              best_j = j;
            }
          }
          return static_cast<double>(best_j);
        };
      std::vector<std::string> blockers;
      for (std::size_t i = 0; i < msg.path.size(); ++i) {
        const auto & wp = msg.path[i];
        if (wp.wait_sec <= 1e-6 && wp.waiting_for.empty()) {
          continue;
        }
        for (const auto & dep : wp.waiting_for) {
          if (std::find(blockers.begin(), blockers.end(), dep.amr_id) ==
            blockers.end())
          {
            blockers.push_back(dep.amr_id);
          }
          double wx = wp.x;
          double wy = wp.y;
          if (std::abs(dep.wait_point.x) > 1e-9 ||
            std::abs(dep.wait_point.y) > 1e-9 ||
            dep.wait_point.index >= 0)
          {
            wx = dep.wait_point.x;
            wy = dep.wait_point.y;
          }
          const double along = path_along(wx, wy);
          if (!best_i || along + 1e-9 < best_along) {
            best_i = i;
            best_along = along;
            best_wait_x = wx;
            best_wait_y = wy;
            best_wait_sec = std::max(wp.wait_sec, dep.wait_point.wait_sec);
          }
        }
        if (wp.waiting_for.empty() && wp.wait_sec > 1e-6 && !best_i) {
          best_i = i;
          best_along = static_cast<double>(i);
          best_wait_x = wp.x;
          best_wait_y = wp.y;
          best_wait_sec = wp.wait_sec;
        }
      }

      // Collect topology / easiness from deps at the chosen wait (for label).
      std::string topo_label;
      bool self_blocks = false;
      bool other_blocks = false;
      if (best_i) {
        for (const auto & dep : msg.path[*best_i].waiting_for) {
          if (!dep.topology.empty() && topo_label.empty()) {
            topo_label = dep.topology;
          }
          self_blocks = self_blocks || dep.self_blocks_other;
          other_blocks = other_blocks || dep.other_blocks_self;
        }
      }

      if (best_i) {
        auto wait_m = base_marker(
          frame_id_, "ecbs_wait", wait_id++,
          visualization_msgs::msg::Marker::SPHERE,
          0.1f, 0.35f, 1.0f, 0.95f);
        wait_m.header.stamp = stamp;
        wait_m.pose.position = to_point(best_wait_x, best_wait_y);
        wait_m.pose.position.z = 0.15;
        wait_m.scale.x = wait_scale_;
        wait_m.scale.y = wait_scale_;
        wait_m.scale.z = wait_scale_;
        arr.markers.push_back(std::move(wait_m));

        auto text = base_marker(
          frame_id_, "ecbs_wait", wait_id++,
          visualization_msgs::msg::Marker::TEXT_VIEW_FACING,
          0.1f, 0.35f, 1.0f, 1.0f);
        text.header.stamp = stamp;
        text.pose.position = to_point(best_wait_x, best_wait_y);
        text.pose.position.z = 0.55;
        text.scale.z = 0.25;
        text.text = msg.amr_id + " WAIT";
        if (!topo_label.empty()) {
          text.text += " [" + topo_label + "]";
        }
        text.text += " for";
        for (std::size_t b = 0; b < blockers.size(); ++b) {
          text.text += (b == 0 ? " " : ", ") + blockers[b];
        }
        if (self_blocks != other_blocks) {
          text.text += other_blocks ? " (easiness GO)" : " (easiness YIELD)";
        }
        if (best_wait_sec > 1e-6) {
          text.text += " " +
            std::to_string(static_cast<int>(std::lround(best_wait_sec))) + "s";
        }
        arr.markers.push_back(std::move(text));
      }

      for (const auto & wp : msg.path) {
        // Red = shared cells: clear points near this robot's own path. The
        // blocker's live position (changes every update) is left out when
        // its path cells are there.
        for (const auto & dep : wp.waiting_for) {
          const auto blocker = snapshot.find(dep.amr_id);
          const auto on_blocker_path = [&](double x, double y) {
              if (blocker == snapshot.end()) {
                return true;
              }
              for (const auto & bwp : blocker->second.path) {
                if (std::hypot(bwp.x - x, bwp.y - y) <= 0.3) {
                  return true;
                }
              }
              return false;
            };
          bool any_on_path = false;
          for (const auto & c : dep.clear_points) {
            any_on_path = any_on_path || on_blocker_path(c.x, c.y);
          }
          std::vector<geometry_msgs::msg::Point> cells;
          for (const auto & c : dep.clear_points) {
            if (near_path(msg, c.x, c.y) && (!any_on_path || on_blocker_path(c.x, c.y))) {
              cells.push_back(to_point(c.x, c.y));
            }
          }
          if (cells.empty()) {
            continue;
          }

          // Join only neighbouring points: no line across the map between
          // separate stretches.
          auto overlap = base_marker(
            frame_id_, "ecbs_overlap", overlap_id++,
            visualization_msgs::msg::Marker::LINE_LIST,
            1.0f, 0.05f, 0.05f, 0.95f);
          overlap.header.stamp = stamp;
          overlap.scale.x = overlap_width_;
          for (std::size_t k = 1; k < cells.size(); ++k) {
            if (std::hypot(cells[k].x - cells[k - 1].x, cells[k].y - cells[k - 1].y) <=
              max_gap_m_)
            {
              overlap.points.push_back(cells[k - 1]);
              overlap.points.push_back(cells[k]);
            }
          }
          if (!overlap.points.empty()) {
            arr.markers.push_back(std::move(overlap));
          }

          for (const auto & c : cells) {
            auto ball = base_marker(
              frame_id_, "ecbs_overlap", overlap_id++,
              visualization_msgs::msg::Marker::SPHERE,
              1.0f, 0.1f, 0.1f, 0.9f);
            ball.header.stamp = stamp;
            ball.pose.position = c;
            ball.pose.position.z = 0.12;
            ball.scale.x = wait_scale_ * 0.7;
            ball.scale.y = wait_scale_ * 0.7;
            ball.scale.z = wait_scale_ * 0.7;
            arr.markers.push_back(std::move(ball));
          }
        }
      }
    }

    // Delete ids the previous rebuild used but this one did not (robot gone,
    // fewer waits / overlaps), so nothing stale stays on screen.
    const std::map<std::string, int32_t> used{
      {"ecbs_path", path_id - 1}, {"ecbs_wait", wait_id - 1},
      {"ecbs_overlap", overlap_id - 1}};
    for (const auto & [ns, max_id] : used) {
      for (int32_t id = max_id + 1; id <= prev_max_id_[ns]; ++id) {
        visualization_msgs::msg::Marker del;
        del.header.frame_id = frame_id_;
        del.header.stamp = stamp;
        del.ns = ns;
        del.id = id;
        del.action = visualization_msgs::msg::Marker::DELETE;
        arr.markers.push_back(del);
      }
      prev_max_id_[ns] = max_id;
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      last_markers_ = arr;
    }
    marker_pub_->publish(arr);
  }

  /// Point (x, y) within near_path_m_ of a segment of \p msg's path.
  bool near_path(const ecbs_planner::msg::ToZoneManager & msg, double x, double y) const
  {
    const auto & p = msg.path;
    if (p.size() == 1) {
      return std::hypot(p[0].x - x, p[0].y - y) <= near_path_m_;
    }
    for (std::size_t i = 1; i < p.size(); ++i) {
      const double dx = p[i].x - p[i - 1].x;
      const double dy = p[i].y - p[i - 1].y;
      const double len2 = dx * dx + dy * dy;
      double t = len2 > 1e-12 ? ((x - p[i - 1].x) * dx + (y - p[i - 1].y) * dy) / len2 : 0.0;
      t = std::clamp(t, 0.0, 1.0);
      if (std::hypot(p[i - 1].x + t * dx - x, p[i - 1].y + t * dy - y) <= near_path_m_) {
        return true;
      }
    }
    return false;
  }

  std::string topic_;
  std::string marker_topic_;
  std::string frame_id_;
  double wait_scale_{0.35};
  double overlap_width_{0.18};
  double near_path_m_{2.0};
  double max_gap_m_{2.5};

  std::mutex mutex_;
  // Sorted by AMR id so each robot keeps the same marker ids every rebuild.
  std::map<std::string, ecbs_planner::msg::ToZoneManager> paths_;
  // Highest id used per namespace in the previous rebuild (stale-id cleanup).
  std::map<std::string, int32_t> prev_max_id_;
  visualization_msgs::msg::MarkerArray last_markers_;

  rclcpp::Subscription<ecbs_planner::msg::ToZoneManager>::SharedPtr sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ToZoneVizNode>());
  rclcpp::shutdown();
  return 0;
}
