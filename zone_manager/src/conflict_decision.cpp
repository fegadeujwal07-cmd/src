#include "zone_manager/conflict_decision.hpp"

#include "zone_manager/behind_robot.hpp"

#include <algorithm>
#include <sstream>
#include <unordered_map>
#include <unordered_set>

#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"

namespace zone_manager
{
namespace
{

// rmf_fleet_msgs/RobotMode
constexpr uint32_t kModeEmergency = 5;
constexpr uint32_t kModeAdapterError = 8;

rclcpp::Logger decision_logger()
{
  return rclcpp::get_logger("zone_manager.conflict_decision");
}

const char * direction_name(TravelDirection d)
{
  switch (d) {
    case TravelDirection::Same: return "Same";
    case TravelDirection::Opposite: return "Opposite";
    default: return "Unknown";
  }
}

std::vector<std::string> suffix_after(
  const std::vector<std::string> & seq,
  std::size_t idx)
{
  if (idx + 1 >= seq.size()) {
    return {};
  }
  return {seq.begin() + static_cast<std::ptrdiff_t>(idx + 1), seq.end()};
}

std::vector<std::string> prefix_before(
  const std::vector<std::string> & seq,
  std::size_t idx)
{
  if (idx == 0 || idx > seq.size()) {
    return {};
  }
  return {seq.begin(), seq.begin() + static_cast<std::ptrdiff_t>(idx)};
}

bool contains_zone(const std::vector<std::string> & seq, const std::string & zone)
{
  return reserved_zone_index(seq, zone).has_value();
}

/// Prefer full zone_sequence; if empty, fall back to reserved zones.
const std::vector<std::string> & effective_sequence(const ConflictRobotInput & in)
{
  if (!in.zone_sequence.empty()) {
    return in.zone_sequence;
  }
  return in.reserved_zones;
}

std::string join_ids(const std::vector<std::string> & ids)
{
  std::ostringstream oss;
  for (std::size_t i = 0; i < ids.size(); ++i) {
    if (i) {
      oss << ",";
    }
    oss << ids[i];
  }
  return oss.str();
}

ConflictDecision decide_two_robot(
  const std::string & conflict_zone,
  const ConflictRobotInput & a,
  const ConflictRobotInput & b)
{
  ConflictDecision out;
  const auto & seq_a = effective_sequence(a);
  const auto & seq_b = effective_sequence(b);

  const auto idx_res_a = reserved_zone_index(a.reserved_zones, conflict_zone);
  const auto idx_res_b = reserved_zone_index(b.reserved_zones, conflict_zone);
  const auto idx_seq_a = reserved_zone_index(seq_a, conflict_zone);
  const auto idx_seq_b = reserved_zone_index(seq_b, conflict_zone);

  out.direction = classify_travel_direction(seq_a, seq_b, conflict_zone);
  const bool junction = is_junction_conflict(seq_a, seq_b, conflict_zone);

  RCLCPP_INFO(
    decision_logger(),
    "[Decide] zone=%s robots=%s and %s direction=%s junction=%s",
    conflict_zone.c_str(),
    a.robot_id.c_str(),
    b.robot_id.c_str(),
    direction_name(out.direction),
    junction ? "true" : "false");

  auto apply_opposite = [&](ConflictClass cls) {
      out.conflict_class = cls;
      out.pause_robot_ids = {a.robot_id, b.robot_id};
      std::sort(out.pause_robot_ids.begin(), out.pause_robot_ids.end());

      RCLCPP_INFO(
        decision_logger(),
        "[Opposite] Conflict detected for robots %s and %s on zone=%s",
        a.robot_id.c_str(), b.robot_id.c_str(), conflict_zone.c_str());
      RCLCPP_INFO(
        decision_logger(),
        "[Opposite] Send pause signals to both: [%s]",
        join_ids(out.pause_robot_ids).c_str());

      const bool healthy_a = is_operating_mode_healthy(a.operating_mode);
      const bool healthy_b = is_operating_mode_healthy(b.operating_mode);

      RCLCPP_INFO(
        decision_logger(),
        "[Opposite] Conflict decision — apex check: operating_mode "
        "(%s operating_mode=%d healthy=%s; %s operating_mode=%d healthy=%s)",
        a.robot_id.c_str(), static_cast<int>(a.operating_mode),
        healthy_a ? "true" : "false",
        b.robot_id.c_str(), static_cast<int>(b.operating_mode),
        healthy_b ? "true" : "false");

      if (healthy_a && healthy_b) {
        RCLCPP_INFO(
          decision_logger(),
          "[Opposite] Both robots operating_mode=1 (healthy); "
          "selecting replan candidate by priority, load, ETA, FIFO");
        out.replan_candidate_id = select_lowest_priority_robot({a.priority, b.priority});
        out.replan_reason = "opposite_both_healthy_lowest_priority";
        RCLCPP_INFO(
          decision_logger(),
          "[Opposite] Selected AMR %s and its reason: %s",
          out.replan_candidate_id.c_str(), out.replan_reason.c_str());
      } else if (healthy_a && !healthy_b) {
        out.replan_candidate_id = a.robot_id;
        out.replan_reason = "opposite_prefer_healthy";
        RCLCPP_INFO(
          decision_logger(),
          "[Opposite] %s is unhealthy (operating_mode=%d); "
          "selecting healthy robot %s for replan",
          b.robot_id.c_str(), static_cast<int>(b.operating_mode),
          out.replan_candidate_id.c_str());
      } else if (!healthy_a && healthy_b) {
        out.replan_candidate_id = b.robot_id;
        out.replan_reason = "opposite_prefer_healthy";
        RCLCPP_INFO(
          decision_logger(),
          "[Opposite] %s is unhealthy (operating_mode=%d); "
          "selecting healthy robot %s for replan",
          a.robot_id.c_str(), static_cast<int>(a.operating_mode),
          out.replan_candidate_id.c_str());
      } else {
        RCLCPP_ERROR(
          decision_logger(),
          "[Opposite] BOTH robots are unhealthy "
          "(%s operating_mode=%d, %s operating_mode=%d); "
          "NO replan candidate published — manual intervention required",
          a.robot_id.c_str(), static_cast<int>(a.operating_mode),
          b.robot_id.c_str(), static_cast<int>(b.operating_mode));
        out.replan_reason = "opposite_both_unhealthy_no_replan";
      }

      std::ostringstream dbg;
      dbg << "class=" << static_cast<int>(cls)
          << " pause=[" << join_ids(out.pause_robot_ids) << "]"
          << " replan=" << out.replan_candidate_id
          << " reason=" << out.replan_reason;
      out.debug_summary = dbg.str();
    };

  auto pause_one = [&](ConflictClass cls, const std::string & pause_id, const char * why) {
      out.conflict_class = cls;
      out.pause_robot_ids = {pause_id};
      RCLCPP_INFO(
        decision_logger(),
        "[SinglePause] Robot Identified for pause: %s (zone=%s class=%d)",
        pause_id.c_str(), conflict_zone.c_str(), static_cast<int>(cls));
      RCLCPP_INFO(
        decision_logger(),
        "[SinglePause] Why single robot pause: %s — selected robot %s; "
        "single robot stop is required.",
        why, pause_id.c_str());
      std::ostringstream dbg;
      dbg << "class=" << static_cast<int>(cls) << " pause=[" << pause_id << "] " << why;
      out.debug_summary = dbg.str();
    };

  // ----- C5: through on main vs branch joining against it -----
  if (junction && out.direction == TravelDirection::Opposite &&
    idx_seq_a.has_value() && idx_seq_b.has_value())
  {
    const auto after_a = suffix_after(seq_a, *idx_seq_a);
    const auto after_b = suffix_after(seq_b, *idx_seq_b);
    const auto before_a = prefix_before(seq_a, *idx_seq_a);
    const auto before_b = prefix_before(seq_b, *idx_seq_b);

    // True when any zone in `part` is absent from `other_seq` (side spur or
    // through-leg that leaves the other's world).
    auto has_exclusive =
      [](const std::vector<std::string> & part,
        const std::vector<std::string> & other_seq) {
        for (const auto & z : part) {
          if (!contains_zone(other_seq, z)) {
            return true;
          }
        }
        return false;
      };

    // Case A: one-sided oppose — branch turns into through's approach while
    // through continues forward (next not on branch approach).
    //
    // Require the through robot to have remaining zones that are NOT on the
    // other sequence. Otherwise a short corridor robot whose destination sits
    // inside the oncoming robot's path is mis-tagged as "through" and only the
    // longer robot is paused (no replan) — observed head-on on 4←→7 / 10←→1.
    if (!after_a.empty() && !after_b.empty()) {
      const bool b_against_a = contains_zone(before_a, after_b.front());
      const bool a_against_b = contains_zone(before_b, after_a.front());
      if (b_against_a && !a_against_b && has_exclusive(after_a, seq_b)) {
        pause_one(
          ConflictClass::BranchVsThrough, b.robot_id,
          "C5 through=A pause=branch B");
        return out;
      }
      if (a_against_b && !b_against_a && has_exclusive(after_b, seq_a)) {
        pause_one(
          ConflictClass::BranchVsThrough, a.robot_id,
          "C5 through=B pause=branch A");
        return out;
      }

      // Case B: mutual head-on after a spur merge. Pause the spur robot only
      // when the other robot truly continues past the junction (exclusive after).
      // Exclusive *approach* alone is not enough: a longer reverse sequence on
      // the same corridor (zones the short robot simply never listed) must be
      // treated as C2/C4 opposite → pause both + replan.
      const bool a_from_branch = has_exclusive(before_a, seq_b);
      const bool b_from_branch = has_exclusive(before_b, seq_a);
      if (a_from_branch && !b_from_branch && has_exclusive(after_b, seq_a)) {
        pause_one(
          ConflictClass::BranchVsThrough, a.robot_id,
          "C5 mainline=B pause=branch A (mutual opposite)");
        return out;
      }
      if (b_from_branch && !a_from_branch && has_exclusive(after_a, seq_b)) {
        pause_one(
          ConflictClass::BranchVsThrough, b.robot_id,
          "C5 mainline=A pause=branch B (mutual opposite)");
        return out;
      }
    }
  }

  // ----- C2 / C4 opposite → pause both + replan candidate -----
  if (out.direction == TravelDirection::Opposite) {
    apply_opposite(
      junction ? ConflictClass::JunctionOpposite : ConflictClass::StraightOpposite);
    return out;
  }

  // ----- Same direction (or unknown treated via behind) -----
  const ConflictClass same_cls = junction ?
    ConflictClass::JunctionSameDirection : ConflictClass::StraightSameDirection;

  // ----- Rear-zone protection ------------------------------------------
  //
  // Checked BEFORE the near/far rule, because that rule ranks robots by how far
  // ahead the contested zone lies on their route and a rear zone has no
  // position on that scale.
  //
  // The case this exists for: a stationary robot reserves only the zone it
  // stands in, so a robot whose target is a few hundred mm away shows no
  // conflict at all until the two overlap — by which time the approaching robot
  // is already braking, and overshoot puts it into the parked one. Claiming the
  // zone behind a robot turns that into a conflict several zones early.
  {
    const bool zone_is_a_rear =
      std::find(a.rear_zones.begin(), a.rear_zones.end(), conflict_zone) !=
      a.rear_zones.end();
    const bool zone_is_b_rear =
      std::find(b.rear_zones.begin(), b.rear_zones.end(), conflict_zone) !=
      b.rear_zones.end();

    // Only when exactly one robot claims it as rear. If it is behind BOTH —
    // two robots nose to tail — there is no "approaching" robot to single out,
    // so fall through and let the existing logic decide.
    if (zone_is_a_rear && !zone_is_b_rear) {
      pause_one(same_cls, b.robot_id, "rear: B entering A's rear zone");
      return out;
    }
    if (zone_is_b_rear && !zone_is_a_rear) {
      pause_one(same_cls, a.robot_id, "rear: A entering B's rear zone");
      return out;
    }
  }


  if (out.direction == TravelDirection::Same || out.direction == TravelDirection::Unknown) {
    // Safety first: reserved index 0 or 1 must proceed; pause the farther robot.
    const bool near_a = idx_res_a.has_value() && *idx_res_a <= 1;
    const bool near_b = idx_res_b.has_value() && *idx_res_b <= 1;
    const bool far_a = idx_res_a.has_value() && *idx_res_a > 1;
    const bool far_b = idx_res_b.has_value() && *idx_res_b > 1;

    if (near_a && far_b) {
      pause_one(same_cls, b.robot_id, "safety: A near (idx<=1), pause B");
      if (out.direction == TravelDirection::Unknown) {
        out.direction = TravelDirection::Same;
      }
      return out;
    }
    if (near_b && far_a) {
      pause_one(same_cls, a.robot_id, "safety: B near (idx<=1), pause A");
      if (out.direction == TravelDirection::Unknown) {
        out.direction = TravelDirection::Same;
      }
      return out;
    }

    // C3 easiness (junction, same direction): ignore priority; prefer robot that
    // does not finish on the other's remaining path.
    if (junction && out.direction == TravelDirection::Same &&
      idx_seq_a.has_value() && idx_seq_b.has_value())
    {
      const auto rem_a = suffix_after(seq_a, *idx_seq_a);
      const auto rem_b = suffix_after(seq_b, *idx_seq_b);
      const bool a_blocks_b = remaining_blocks_other(rem_a, rem_b);
      const bool b_blocks_a = remaining_blocks_other(rem_b, rem_a);

      if (b_blocks_a && !a_blocks_b) {
        // Letting B first leaves B on A's path → pause B, let A (easiness).
        pause_one(ConflictClass::JunctionSameDirection, b.robot_id,
          "easiness: B remaining blocks A; pause B (priority ignored)");
        return out;
      }
      if (a_blocks_b && !b_blocks_a) {
        pause_one(ConflictClass::JunctionSameDirection, a.robot_id,
          "easiness: A remaining blocks B; pause A (priority ignored)");
        return out;
      }
    }

    // C1 / fallback: pause robots coming from behind by reserved index.
    RCLCPP_INFO(
      decision_logger(),
      "[Behind] Evaluating behind-robot pause for zone=%s between %s and %s",
      conflict_zone.c_str(), a.robot_id.c_str(), b.robot_id.c_str());
    std::unordered_map<std::string, std::vector<std::string>> reserved_by_robot = {
      {a.robot_id, a.reserved_zones},
      {b.robot_id, b.reserved_zones},
    };
    out.pause_robot_ids = select_robots_coming_from_behind(conflict_zone, reserved_by_robot);
    out.conflict_class = same_cls;
    if (out.direction == TravelDirection::Unknown && !out.pause_robot_ids.empty()) {
      out.direction = TravelDirection::Same;
    }
    std::ostringstream dbg;
    dbg << "class=" << static_cast<int>(out.conflict_class)
        << " pause=[" << join_ids(out.pause_robot_ids) << "] behind_or_fallback";
    out.debug_summary = dbg.str();
    return out;
  }

  out.conflict_class = ConflictClass::Unknown;
  out.debug_summary = "unclassified";
  return out;
}

}  // namespace

bool is_robot_healthy(uint32_t fleet_mode, bool mode_known)
{
  if (!mode_known) {
    return true;
  }
  return fleet_mode != kModeEmergency && fleet_mode != kModeAdapterError;
}

bool is_operating_mode_healthy(int16_t operating_mode)
{
  return operating_mode == 1;
}

TravelDirection classify_travel_direction(
  const std::vector<std::string> & seq_a,
  const std::vector<std::string> & seq_b,
  const std::string & conflict_zone)
{
  const auto idx_a = reserved_zone_index(seq_a, conflict_zone);
  const auto idx_b = reserved_zone_index(seq_b, conflict_zone);
  if (!idx_a.has_value() || !idx_b.has_value()) {
    return TravelDirection::Unknown;
  }

  const auto after_a = suffix_after(seq_a, *idx_a);
  const auto after_b = suffix_after(seq_b, *idx_b);
  const auto before_a = prefix_before(seq_a, *idx_a);
  const auto before_b = prefix_before(seq_b, *idx_b);

  if (!after_a.empty() && !after_b.empty()) {
    const bool a_next_on_b_before = contains_zone(before_b, after_a.front());
    const bool b_next_on_a_before = contains_zone(before_a, after_b.front());
    // Classic head-on: each next zone lies on the other's approach.
    if (a_next_on_b_before && b_next_on_a_before) {
      return TravelDirection::Opposite;
    }
    // Junction / branch opposing through: one reverses onto the other's approach
    // while the other continues forward off that approach.
    if (b_next_on_a_before && !a_next_on_b_before &&
      !contains_zone(after_b, after_a.front()))
    {
      return TravelDirection::Opposite;
    }
    if (a_next_on_b_before && !b_next_on_a_before &&
      !contains_zone(after_a, after_b.front()))
    {
      return TravelDirection::Opposite;
    }
  }

  // Same: shared forward zones after conflict.
  if (!after_a.empty() && !after_b.empty()) {
    std::unordered_set<std::string> set_b(after_b.begin(), after_b.end());
    for (const auto & z : after_a) {
      if (set_b.count(z) > 0) {
        return TravelDirection::Same;
      }
    }
  }

  if (!after_a.empty() && contains_zone(after_b, after_a.front())) {
    return TravelDirection::Same;
  }
  if (!after_b.empty() && contains_zone(after_a, after_b.front())) {
    return TravelDirection::Same;
  }

  // ---------------------------------------------------------------------------
  // Fallback: relative traversal order of shared zones.
  //
  // Do NOT infer Same from "shared approach + empty after" before this runs:
  // a robot whose sequence ends at the conflict zone (destination on the
  // other's path) has empty after by construction, and shared before zones are
  // normal in a head-on corridor meet. That used to return Same and skip the
  // opposite fallback → single-robot safety pause, no replan.
  //
  // The prefix/suffix rules above cannot classify a conflict when the conflict
  // zone sits at either END of a robot's sequence:
  //
  //   * conflict zone == the robot's CURRENT zone  -> reserved_zone_index()
  //     returns 0 and prefix_before() returns {} by construction, so
  //     "the other robot's next zone is on my approach" can never be true.
  //   * conflict zone == the robot's LAST zone     -> suffix_after() is empty,
  //     so the whole head-on block (which requires both suffixes non-empty) is
  //     skipped.
  //
  // Both happen in an ordinary face-to-face meeting, e.g. (observed):
  //     amr1 zone_sequence = [6,7,8,9]                  conflict 9 -> after  empty
  //     amr2 zone_sequence = [9,10,9,8,7,6,5,4,3,2]     conflict 9 -> before empty
  // which fell through to Unknown and was then classified as a "safety" or
  // "branch" case, so no replan candidate was ever published.
  //
  // Direction is still recoverable: take any two zones both robots visit and
  // compare the order they visit them in. Reversed order => travelling toward
  // each other. Uses the LAST occurrence for each zone so a there-and-back
  // sequence (the 9,10,9 U-turn above) is judged on the leg that actually
  // continues, not the zone the robot is standing in.
  {
    auto last_index_of = [](const std::vector<std::string> & seq,
        const std::string & zone) -> std::optional<std::size_t>
      {
        for (std::size_t i = seq.size(); i > 0; --i) {
          if (seq[i - 1] == zone) {
            return i - 1;
          }
        }
        return std::nullopt;
      };

    std::vector<std::string> shared;
    for (const auto & z : seq_a) {
      if (contains_zone(seq_b, z) &&
        std::find(shared.begin(), shared.end(), z) == shared.end())
      {
        shared.push_back(z);
      }
    }

    for (std::size_t i = 0; i + 1 < shared.size(); ++i) {
      for (std::size_t j = i + 1; j < shared.size(); ++j) {
        const auto ai = last_index_of(seq_a, shared[i]);
        const auto aj = last_index_of(seq_a, shared[j]);
        const auto bi = last_index_of(seq_b, shared[i]);
        const auto bj = last_index_of(seq_b, shared[j]);
        if (!ai || !aj || !bi || !bj) {
          continue;
        }
        if (*ai == *aj || *bi == *bj) {
          continue;
        }
        const bool a_i_first = (*ai < *aj);
        const bool b_i_first = (*bi < *bj);
        if (a_i_first != b_i_first) {
          // Opposite traversal order over a shared pair of zones.
          return TravelDirection::Opposite;
        }
      }
    }
  }

  // Approaching same zone from a shared corridor (one has empty after), only
  // when the shared-order fallback above did not prove opposite travel.
  if (after_a.empty() || after_b.empty()) {
    for (const auto & z : before_a) {
      if (contains_zone(before_b, z)) {
        return TravelDirection::Same;
      }
    }
  }

  return TravelDirection::Unknown;
}

bool is_junction_conflict(
  const std::vector<std::string> & seq_a,
  const std::vector<std::string> & seq_b,
  const std::string & conflict_zone)
{
  const auto idx_a = reserved_zone_index(seq_a, conflict_zone);
  const auto idx_b = reserved_zone_index(seq_b, conflict_zone);
  if (!idx_a.has_value() || !idx_b.has_value()) {
    return false;
  }
  const auto before_a = prefix_before(seq_a, *idx_a);
  const auto before_b = prefix_before(seq_b, *idx_b);
  if (before_a.empty() || before_b.empty()) {
    return false;
  }
  // Approaches must be disjoint.
  for (const auto & z : before_a) {
    if (contains_zone(before_b, z)) {
      return false;
    }
  }
  // True branch/junction: at least one approach zone is absent from the other
  // robot's entire sequence (side spur), not merely opposite ends of one corridor.
  auto has_exclusive_approach =
    [](const std::vector<std::string> & before,
      const std::vector<std::string> & other_seq) {
      for (const auto & z : before) {
        if (!contains_zone(other_seq, z)) {
          return true;
        }
      }
      return false;
    };
  return has_exclusive_approach(before_a, seq_b) ||
         has_exclusive_approach(before_b, seq_a);
}

bool remaining_blocks_other(
  const std::vector<std::string> & rem_self,
  const std::vector<std::string> & rem_other)
{
  if (rem_self.empty() || rem_other.empty()) {
    return false;
  }
  if (rem_self.size() > rem_other.size()) {
    return false;
  }
  // rem_self is a prefix of rem_other → self destination sits on other's path.
  for (std::size_t i = 0; i < rem_self.size(); ++i) {
    if (rem_self[i] != rem_other[i]) {
      return false;
    }
  }
  return true;
}

ConflictDecision decide_conflict_action(
  const std::string & conflict_zone,
  const std::vector<ConflictRobotInput> & robots)
{
  ConflictDecision out;
  if (conflict_zone.empty() || robots.size() < 2) {
    out.debug_summary = "insufficient_robots";
    return out;
  }

  if (robots.size() == 2) {
    return decide_two_robot(conflict_zone, robots[0], robots[1]);
  }

  // N>2: legacy behind-index pause only.
  RCLCPP_INFO(
    rclcpp::get_logger("zone_manager.conflict_decision"),
    "[Behind] N>2 robots on zone=%s; using behind-index pause fallback",
    conflict_zone.c_str());
  std::unordered_map<std::string, std::vector<std::string>> reserved_by_robot;
  for (const auto & r : robots) {
    reserved_by_robot[r.robot_id] = r.reserved_zones;
  }
  out.pause_robot_ids = select_robots_coming_from_behind(conflict_zone, reserved_by_robot);
  out.conflict_class = ConflictClass::StraightSameDirection;
  out.direction = TravelDirection::Same;
  out.debug_summary = "n_gt_2_behind_fallback pause=[" + join_ids(out.pause_robot_ids) + "]";
  return out;
}

}  // namespace zone_manager
