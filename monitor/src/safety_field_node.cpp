// =============================================================================
// File: monitor/src/safety_field_node.cpp
// =============================================================================
// OBJECTIVE
//   A RECTANGULAR safety field per AMR, centred on the robot's MEASURED
//   position and turned to its MEASURED heading, so the field is carried by the
//   robot the way a mattress is carried rather than lying on the map underneath
//   it. When two fields overlap, both are drawn red instead of teal and the
//   affected AMR ids are published for zone_manager to pause.
//
// FIELD SIZE
//   The AMR body is 1.353 m nose-to-tail by 1.403 m side-to-side. The field is
//   that body inflated by a margin of 0.40 m on every side, giving a
//   2.153 x 2.203 m rectangle, so two fields touch with 0.80 m of clear floor
//   still between the two bodies.
//
//   Because the field is a rectangle and not a circle, the centre-to-centre
//   distance at which a conflict fires is no longer one number — it depends on
//   how the two robots are facing:
//
//     nose-to-tail       2.153 m
//     side-by-side       2.203 m
//     corner-to-corner   up to 3.080 m  (the sum of the two circumradii, and
//                                        the worst case over all headings)
//
//   That spread IS the point of the shape. The circle this replaced fired at a
//   flat 2.2 m in every direction and, being rotation-invariant, could not see a
//   corner swing out during a turn at all. The rectangle raises the conflict
//   while two robots crossing diagonally are still nearly a metre further apart,
//   and it tracks the corner that is actually doing the swinging.
//
// ONE RECTANGLE, NO HYSTERESIS
//   Trigger and release are the SAME rectangle, and there is no dwell. A
//   conflict is raised on the first tick the rectangles touch and dropped on
//   the first tick they do not, so what is published is always exactly what the
//   geometry says right now. The circle version carried a larger release radius
//   to stop a pair on the boundary chattering; that was dropped deliberately,
//   and so was the dwell that replaced it.
//
//   The consequence to know about: because the field rotates, two robots that
//   are standing still can enter AND leave an overlap purely by turning, and
//   each of those is now a real startPause / stopPause on the floor. If that
//   shows up as lurching during a turn, the fix is a dwell — hold a conflict
//   for a short period of continuous clearance before releasing it, which damps
//   the flicker in time and never delays a pause.
//
// INPUTS
//   /fleet_states            (rmf_fleet_msgs/FleetState) — real robot poses
// OUTPUTS
//   /safety_field_markers    (visualization_msgs/MarkerArray) — the rectangles
//   /safety_field_conflict   (monitor/SafetyFieldConflict)    — who overlaps
//
// WHY THIS LIVES IN monitor
//   The field must be centred on where the robot actually IS. Only two places
//   in the system carry that: execution (which publishes /fleet_states) and the
//   packages that consume it. It cannot live in RMF core — the schedule node
//   subscribes only to itinerary and negotiation topics and never receives a
//   robot position, which is why the circles RViz draws today sit at the
//   PLANNED position. Measured on 2026-08: "Robot [amr1] is very far
//   [15.0m] from where it is supposed to be" — a safety field 15 m from the
//   robot is worse than none.
//
//   zone_manager has the positions but is the decision layer (it chooses who
//   pauses). Detecting a geometric overlap is an observation, not a decision,
//   so it belongs beside the other observers. monitor already subscribes to
//   /fleet_states and already publishes MarkerArray for /zone_markers.
//
// WHY NOT rmf_traffic::DetectConflict
//   It was tried first and it CANNOT answer this question. DetectConflict is a
//   swept test: it asks "do these two bodies collide while moving along their
//   trajectories". A safety field asks "are these two shapes overlapping right
//   now", which is a different question, and RMF deliberately answers no to it.
//
//   DetectConflict.cpp:934 routes an already-close pair to detect_approach(),
//   and the comment at DetectConflict.cpp:606 states the rule outright: two
//   trajectories that START inside each other's vicinity are not a conflict, so
//   that robots get "an opportunity to back away from each other". Only getting
//   CLOSER counts, and closeness is measured with a continuous FCL collision
//   along the supplied motion. Feeding it a stationary here-and-now trajectory
//   therefore returns nullopt every single time.
//
//   Measured 2026-09-07 with the DetectConflict version of this node: 2614
//   frames over 261 s, amr1 and amr2 closed to 1.4875 m with a 3.00 m trigger
//   (an overlap depth of 1.51 m — each robot's centre inside the other's
//   field), 525 consecutive frames overlapping over 52 s. Zero turned red and
//   zero messages were published. log2 shows the schedule reporting
//   "conflicts":[] across the same window, for the same reason.
//
//   That route is now doubly closed. Even if the swept semantics were wanted,
//   RMF could not hold this shape: load_param.cpp builds every fleet profile as
//   make_final_convex<geometry::Circle>, so an oriented rectangle has nowhere to
//   live in a rmf_traffic::Profile as this workspace is configured. The test
//   lives here, in safety_field_geometry.hpp, where it is four lines of
//   separating-axis arithmetic and is covered by unit tests.
//
// THIS NODE DOES NOT PAUSE ANYTHING ITSELF
//   It publishes markers and ids. zone_manager subscribes to
//   /safety_field_conflict and owns the pause, holding safety overlaps under a
//   namespaced key in the same pause_refcount_by_robot_ counter its zone
//   conflicts use. That single counter is what makes two independent sources of
//   pause safe: a robot held by BOTH a zone conflict and a safety overlap stays
//   stopped until both clear, which a bare on/off latch could not express
//   (observed 2026-08-22: amr2 received two startPause and one stopPause, and
//   resumed). Publishing startPause from here as well would reintroduce exactly
//   that bug, so this node stays on the observing side of the line.
//
// MESSAGE CONTRACT — READ BEFORE CHANGING publish_conflict_deltas()
//   /safety_field_conflict carries a DELTA, not a roster. A DETECTED message
//   names only the robots that just ENTERED conflict; a RESOLVED message names
//   only those that just LEFT it. Exactly one DETECTED per robot per hold, and
//   exactly one matching RESOLVED.
//
//   This is not a style choice. zone_manager's note_conflict_detected()
//   increments pause_refcount_by_robot_ UNCONDITIONALLY — unlike its sibling
//   ensure_robots_held_for_zone(), which guards against a double-bump. Name a
//   robot in two DETECTED messages without a RESOLVED between them and its
//   refcount reaches 2, the single release takes it to 1, and the robot is
//   paused for good. Re-announcing the current state periodically, or resending
//   the full set on every change, would do precisely that.
//
//   The delta also fixes what the previous none->any / any->none edge trigger
//   got wrong with three or more robots: when one pair cleared while another
//   still overlapped, no message was sent at all, and the eventual RESOLVED
//   carried only the last tick's robots — leaving any robot whose pair had
//   cleared earlier held forever. With two AMRs the two schemes behave
//   identically; with three the old one strands a robot.
// =============================================================================

#include <rclcpp/rclcpp.hpp>

#include <rmf_fleet_msgs/msg/fleet_state.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <monitor/msg/safety_field_conflict.hpp>
#include <monitor/msg/safety_field_pair.hpp>
#include <monitor/safety_field_tracker.hpp>

#include <chrono>
#include <cmath>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace monitor {

namespace {

// Every default lives here and NOWHERE else. Each value is written once and
// referenced both as the ROS parameter default and as the member initialiser,
// so the two cannot drift. They had: the parameters were moved to 1.475 while
// the members still said 0.3 / 0.35, leaving two different answers in the file
// for the same question.
constexpr double kBodyLength = 0.5; //1.353;   // safety_field_length,  nose to tail
constexpr double kBodyWidth = 0.5; //1.403;    // safety_field_width,   side to side
constexpr double kMargin = 0.40;        // safety_field_margin,  per side
constexpr double kEvaluateRateHz = 10.0;       // evaluate_rate_hz
constexpr const char* kFrameId = "map";        // marker_frame_id
constexpr const char* kLevelName = "L1";       // level_name
constexpr const char* kFleetStatesTopic = "/fleet_states";

// Not a ROS parameter — a robot silent for this long has its field dropped
// rather than left sitting at a stale position. Listed here only so the
// constants are in one place.
constexpr double kStaleAfterSec = 3.0;

}  // namespace

class SafetyFieldNode : public rclcpp::Node
{
public:
  SafetyFieldNode()
  : Node("safety_field_node")
  {
    // The robot body, not the field. Nose-to-tail and side-to-side of the AMR
    // itself; the field is this plus the margin on every side.
    body_length_ = declare_parameter<double>("safety_field_length", kBodyLength);
    body_width_ = declare_parameter<double>("safety_field_width", kBodyWidth);

    // Clearance kept around the body, metres, added to all four sides. This is
    // the number to turn when the fields prove too eager or too timid — it is
    // the only one that changes WHERE the conflict fires. At 0.40 m two fields
    // touch with 0.80 m of floor between the bodies.
    margin_ = declare_parameter<double>("safety_field_margin", kMargin);
    if (margin_ < 0.0) {
      RCLCPP_WARN(
        get_logger(),
        "safety_field_margin (%.2f) is negative, which would shrink the field "
        "inside the robot body; clamping to 0.",
        margin_);
      margin_ = 0.0;
    }

    field_half_length_ = 0.5 * body_length_ + margin_;
    field_half_width_ = 0.5 * body_width_ + margin_;

    // The geometry and the enter/leave bookkeeping both live outside this node,
    // in headers that test_safety_field_geometry.cpp and
    // test_safety_field_tracker.cpp exercise directly. What is left here is ROS
    // wiring: subscribe, tick, colour, publish.
    tracker_.emplace(field_half_length_, field_half_width_);

    frame_id_ = declare_parameter<std::string>("marker_frame_id", kFrameId);
    level_ = declare_parameter<std::string>("level_name", kLevelName);
    const double rate =
      declare_parameter<double>("evaluate_rate_hz", kEvaluateRateHz);
    const std::string fleet_topic =
      declare_parameter<std::string>("fleet_states_topic", kFleetStatesTopic);

    fleet_sub_ = create_subscription<rmf_fleet_msgs::msg::FleetState>(
      fleet_topic, rclcpp::SensorDataQoS(),
      [this](rmf_fleet_msgs::msg::FleetState::SharedPtr msg)
      {
        on_fleet_state(std::move(msg));
      });

    // TRANSIENT_LOCAL. A VOLATILE publisher is incompatible with a subscriber
    // that asks for TRANSIENT_LOCAL, and RViz displays commonly do: observed
    // 2026-09-07, "New subscription discovered on topic
    // '/safety_field_markers', requesting incompatible QoS. No messages will be
    // sent to it. Last incompatible policy: DURABILITY_QOS_POLICY" — RViz got
    // nothing while ros2 topic echo (volatile by default) saw every frame. A
    // transient-local publisher satisfies BOTH kinds of subscriber, and hands a
    // late-joining RViz the current rectangles immediately instead of after the
    // next tick.
    markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/safety_field_markers", rclcpp::QoS(10).transient_local());

    // Depth 1 TRANSIENT_LOCAL, which hands a late-joining zone_manager the most
    // recent transition instead of nothing at all.
    //
    // Note the limit, since this topic now drives a pause: the message is a
    // delta, so one latched sample is not a snapshot of who is currently held.
    // A zone_manager that restarts mid-conflict recovers the last transition
    // only, and picks the rest up on the next one. Republishing state to close
    // that gap is NOT an option — see the message contract in the header.
    conflict_pub_ = create_publisher<monitor::msg::SafetyFieldConflict>(
      "/safety_field_conflict", rclcpp::QoS(1).transient_local());

    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(
          1.0 / (rate > 0.0 ? rate : kEvaluateRateHz))),
      [this]() { evaluate(); });

    RCLCPP_INFO(
      get_logger(),
      "Safety fields: %.3f x %.3f m body + %.2f m margin = %.3f x %.3f m "
      "rectangle, carried at the robot's measured heading. Conflict fires at "
      "%.3f m nose-to-tail, %.3f m side-by-side, up to %.3f m corner-to-corner, "
      "and clears the moment they part. Evaluated at %.1f Hz from %s. "
      "Markers on /safety_field_markers, conflicts on /safety_field_conflict "
      "for zone_manager to pause.",
      body_length_, body_width_, margin_,
      2.0 * field_half_length_, 2.0 * field_half_width_,
      2.0 * field_half_length_, 2.0 * field_half_width_,
      2.0 * std::hypot(field_half_length_, field_half_width_),
      rate, fleet_topic.c_str());
  }

private:
  struct RobotPose
  {
    SafetyFieldPose pose;
    rclcpp::Time stamp;
  };

  void on_fleet_state(rmf_fleet_msgs::msg::FleetState::SharedPtr msg)
  {
    if (!msg) {
      return;
    }
    for (const auto & robot : msg->robots) {
      if (robot.name.empty()) {
        continue;
      }
      // Ignore robots reported on another level; their fields cannot overlap
      // ours in any meaningful sense.
      if (!robot.location.level_name.empty() &&
        robot.location.level_name != level_)
      {
        continue;
      }
      RobotPose p;
      p.pose.x = robot.location.x;
      p.pose.y = robot.location.y;
      // Now load-bearing. The circle version stored yaw and never used it, so
      // nothing in this system has ever checked it; a rectangle pointed the
      // wrong way is a field pointed the wrong way, and that now pauses robots.
      p.pose.yaw = robot.location.yaw;
      p.stamp = now();
      poses_[robot.name] = p;
    }
  }

  // Centre-to-centre distance in metres. Reported on the message so a consumer
  // can see how close the robots actually came. Unlike the circle version this
  // is NO LONGER the quantity the overlap test compares — a rectangle has no
  // single trigger distance — so it is descriptive only.
  static double separation(const SafetyFieldPose & a, const SafetyFieldPose & b)
  {
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
  }

  monitor::msg::SafetyFieldPair pair_msg(const SafetyFieldPairRef & ref) const
  {
    monitor::msg::SafetyFieldPair out;
    out.robot_a = ref.robot_a;
    out.robot_b = ref.robot_b;
    const auto it_a = poses_.find(ref.robot_a);
    const auto it_b = poses_.find(ref.robot_b);
    out.separation = (it_a != poses_.end() && it_b != poses_.end()) ?
      static_cast<float>(separation(it_a->second.pose, it_b->second.pose)) :
      0.0f;
    return out;
  }

  std::vector<monitor::msg::SafetyFieldPair> pair_msgs(
    const std::vector<SafetyFieldPairRef> & refs) const
  {
    std::vector<monitor::msg::SafetyFieldPair> out;
    out.reserve(refs.size());
    for (const auto & ref : refs) {
      out.push_back(pair_msg(ref));
    }
    return out;
  }

  void evaluate()
  {
    // A robot that has stopped reporting must not keep a stale field sitting on
    // the map; drop it rather than testing a position that may be minutes old.
    // The tracker releases any conflict that robot was holding.
    drop_stale_poses();

    std::map<std::string, SafetyFieldPose> measured;
    for (const auto & [name, entry] : poses_) {
      measured.emplace(name, entry.pose);
    }

    const SafetyFieldUpdate update = tracker_->update(measured);

    publish_markers(update.conflicting);
    publish_conflict_deltas(update);
  }

  void drop_stale_poses()
  {
    const auto cutoff = now() - rclcpp::Duration::from_seconds(stale_after_sec_);
    for (auto it = poses_.begin(); it != poses_.end(); ) {
      if (it->second.stamp < cutoff) {
        RCLCPP_INFO(
          get_logger(),
          "Dropping safety field for %s: no /fleet_states update for %.0fs",
          it->first.c_str(), stale_after_sec_);
        it = poses_.erase(it);
      } else {
        ++it;
      }
    }
  }

  static std::string pair_key(const std::string & a, const std::string & b)
  {
    return (a < b) ? (a + "|" + b) : (b + "|" + a);
  }

  // A marker id must belong to the ROBOT, not to its position in the map. The
  // ids used to be handed out by a counter walking poses_, so erasing a stale
  // robot renumbered every robot after it: the survivor's rectangle jumped to a
  // different id while the vacated id kept drawing at the dead robot's last
  // position until its lifetime ran out. Assigned once here and never reused.
  //
  // Two markers are drawn per robot — the field and the body outline inside it
  // — so the robot's slot is doubled and the two ids derived from it.
  int marker_slot_for(const std::string & name)
  {
    const auto it = marker_slots_.find(name);
    if (it != marker_slots_.end()) {
      return it->second;
    }
    const int slot = next_marker_slot_++;
    marker_slots_.emplace(name, slot);
    return slot;
  }

  void publish_markers(const std::set<std::string> & conflicting)
  {
    visualization_msgs::msg::MarkerArray array;
    std::set<int> live_ids;

    for (const auto & [name, entry] : poses_) {
      const SafetyFieldPose & pose = entry.pose;
      const int slot = marker_slot_for(name);
      const bool in_conflict = conflicting.count(name) > 0;

      // The field itself: a flat slab the size of the inflated rectangle, laid
      // on the floor and turned to the robot's heading.
      visualization_msgs::msg::Marker field;
      field.header.frame_id = frame_id_;
      field.header.stamp = now();
      field.ns = "safety_field";
      field.id = 2 * slot;
      live_ids.insert(field.id);
      field.type = visualization_msgs::msg::Marker::CUBE;
      field.action = visualization_msgs::msg::Marker::ADD;
      field.pose.position.x = pose.x;
      field.pose.position.y = pose.y;
      field.pose.position.z = 0.0;
      // Yaw only: the field is flat and never rolls or pitches.
      field.pose.orientation.z = std::sin(0.5 * pose.yaw);
      field.pose.orientation.w = std::cos(0.5 * pose.yaw);
      // Full extents, so the drawn rectangle is the field the test uses.
      field.scale.x = 2.0 * field_half_length_;
      field.scale.y = 2.0 * field_half_width_;
      field.scale.z = 0.05;
      field.color.r = in_conflict ? 1.0f : 0.5f;
      field.color.g = in_conflict ? 0.0f : 1.0f;
      field.color.b = in_conflict ? 0.0f : 0.9f;
      field.color.a = 0.5f;
      // Republished every tick from the cached pose, so this only has to
      // outlive one evaluation period. It is the backstop for the node dying
      // outright: the rectangles then fade instead of freezing on the map.
      field.lifetime = rclcpp::Duration::from_seconds(1.0);
      array.markers.push_back(field);

      // The body outline inside it. Not decoration: the gap between this and
      // the slab around it IS safety_field_margin, so the number being tuned is
      // visible on screen instead of having to be taken on trust.
      visualization_msgs::msg::Marker body;
      body.header.frame_id = frame_id_;
      body.header.stamp = now();
      body.ns = "safety_field";
      body.id = 2 * slot + 1;
      live_ids.insert(body.id);
      body.type = visualization_msgs::msg::Marker::LINE_STRIP;
      body.action = visualization_msgs::msg::Marker::ADD;
      // LINE_STRIP points are given in the frame directly, so the marker pose
      // stays identity and the corners carry the rotation.
      body.pose.orientation.w = 1.0;
      body.scale.x = 0.03;  // line width
      body.color.r = 1.0f;
      body.color.g = 1.0f;
      body.color.b = 1.0f;
      body.color.a = 0.9f;
      body.lifetime = rclcpp::Duration::from_seconds(1.0);
      append_body_outline(pose, body.points);
      array.markers.push_back(body);
    }

    // An empty MarkerArray is a no-op in RViz — it does not remove anything —
    // so a robot that goes stale has to be retired by id. Without this its
    // rectangle would sit at the last known position for a further lifetime.
    for (const int gone : published_ids_) {
      if (live_ids.count(gone) > 0) {
        continue;
      }
      visualization_msgs::msg::Marker m;
      m.header.frame_id = frame_id_;
      m.header.stamp = now();
      m.ns = "safety_field";
      m.id = gone;
      m.action = visualization_msgs::msg::Marker::DELETE;
      array.markers.push_back(m);
    }

    if (!array.markers.empty()) {
      markers_pub_->publish(array);
    }
    published_ids_ = std::move(live_ids);
  }

  // Four corners of the robot body in map coordinates, closed back to the first
  // so the strip draws a complete rectangle.
  void append_body_outline(
    const SafetyFieldPose & pose,
    std::vector<geometry_msgs::msg::Point> & out) const
  {
    const double c = std::cos(pose.yaw);
    const double s = std::sin(pose.yaw);
    const double hl = 0.5 * body_length_;
    const double hw = 0.5 * body_width_;

    const double signs[5][2] = {
      {+1.0, +1.0}, {+1.0, -1.0}, {-1.0, -1.0}, {-1.0, +1.0}, {+1.0, +1.0}
    };
    out.reserve(5);
    for (const auto & sign : signs) {
      const double along = sign[0] * hl;
      const double across = sign[1] * hw;
      geometry_msgs::msg::Point p;
      p.x = pose.x + along * c - across * s;
      p.y = pose.y + along * s + across * c;
      p.z = 0.03;  // just above the slab, so it is not z-fought away
      out.push_back(p);
    }
  }

  void publish_conflict_deltas(const SafetyFieldUpdate & update)
  {
    // Deltas, not a roster — see the message contract in the file header. One
    // DETECTED per robot per hold and one matching RESOLVED, because
    // zone_manager increments its pause refcount once per named robot without
    // checking whether it is already holding that robot.

    // Released first. A robot never appears in both lists in the same tick, but
    // ordering the release ahead of the hold keeps the refcount on the far side
    // from momentarily counting a robot twice if that ever changes.
    if (!update.left.empty()) {
      const auto ended = pair_msgs(update.ended);
      monitor::msg::SafetyFieldConflict msg;
      msg.stamp = now();
      msg.event = monitor::msg::SafetyFieldConflict::RESOLVED;
      msg.robot_ids = update.left;
      // The pairs that just ended, which is what the released robots were held
      // by. Their separations are the distance the robots are at NOW — the
      // distance that cleared the conflict, not the one that opened it.
      msg.pairs = ended;
      conflict_pub_->publish(msg);
      RCLCPP_INFO(
        get_logger(), "Safety field conflict cleared: %s (%s)",
        join(update.left).c_str(), describe(ended).c_str());
    }

    if (!update.entered.empty()) {
      // Every pair still overlapping, not only the ones involving the newly
      // held robots: with three or more AMRs that is what distinguishes "amr3
      // joined the amr1/amr2 conflict" from "amr3 started its own".
      const auto active = pair_msgs(update.active);
      monitor::msg::SafetyFieldConflict msg;
      msg.stamp = now();
      msg.event = monitor::msg::SafetyFieldConflict::DETECTED;
      msg.robot_ids = update.entered;
      msg.pairs = active;
      conflict_pub_->publish(msg);
      RCLCPP_WARN(
        get_logger(), "Safety field conflict: %s (%s)",
        join(update.entered).c_str(), describe(active).c_str());
    }
  }

  static std::string join(const std::vector<std::string> & ids)
  {
    std::string out;
    for (std::size_t k = 0; k < ids.size(); ++k) {
      out += (k ? ", " : "") + ids[k];
    }
    return out;
  }

  static std::string describe(
    const std::vector<monitor::msg::SafetyFieldPair> & pairs)
  {
    std::string out;
    for (std::size_t k = 0; k < pairs.size(); ++k) {
      out += (k ? ", " : "") + pairs[k].robot_a + "<->" + pairs[k].robot_b +
        " at " + std::to_string(pairs[k].separation) + "m";
    }
    return out;
  }

  double body_length_{kBodyLength};
  double body_width_{kBodyWidth};
  double margin_{kMargin};
  double field_half_length_{0.5 * kBodyLength + kMargin};
  double field_half_width_{0.5 * kBodyWidth + kMargin};
  double stale_after_sec_{kStaleAfterSec};
  std::string frame_id_{kFrameId};
  std::string level_{kLevelName};

  std::map<std::string, RobotPose> poses_;

  // Holds last tick's overlapping pairs and the record of who zone_manager
  // currently believes it is holding on our behalf. Constructed once the
  // parameters are read, hence the optional.
  std::optional<SafetyFieldTracker> tracker_;

  std::map<std::string, int> marker_slots_;
  std::set<int> published_ids_;
  int next_marker_slot_{0};

  rclcpp::Subscription<rmf_fleet_msgs::msg::FleetState>::SharedPtr fleet_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  rclcpp::Publisher<monitor::msg::SafetyFieldConflict>::SharedPtr conflict_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace monitor

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<monitor::SafetyFieldNode>());
  rclcpp::shutdown();
  return 0;
}
