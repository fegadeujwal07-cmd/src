#include "zone_manager/post_replan.hpp"

#include <algorithm>
#include <unordered_set>

namespace zone_manager
{
namespace
{

bool snapshot_has_plan(const PlanSnapshot & s)
{
  return !s.zone_sequence.empty() || !s.planned_path.empty() ||
         !s.current_zone.empty() || !s.next_zones.empty();
}

}  // namespace

bool zones_changed_since_arm(
  const PlanSnapshot & baseline,
  const std::vector<std::string> & incoming_sequence,
  const std::vector<std::string> & incoming_next)
{
  // Nothing to compare against yet: an empty arm-time sequence means the world
  // had no zones for this robot when the replan was requested, so the first
  // real sequence to arrive IS the change.
  if (baseline.zone_sequence.empty() && baseline.next_zones.empty()) {
    return !incoming_sequence.empty() || !incoming_next.empty();
  }

  // A momentary empty report is not a new plan. Treating it as one would fire
  // the re-analysis on no zones at all, which is worse than the stale zones
  // this function exists to prevent.
  if (incoming_sequence.empty() && incoming_next.empty()) {
    return false;
  }

  return incoming_sequence != baseline.zone_sequence ||
         incoming_next != baseline.next_zones;
}

bool is_new_plan_for_candidate(
  const PlanSnapshot & baseline,
  const PlanSnapshot & incoming)
{
  if (!snapshot_has_plan(incoming)) {
    return false;
  }
  // First usable plan after empty baseline, or any field change.
  if (!snapshot_has_plan(baseline)) {
    return true;
  }
  return incoming.current_zone != baseline.current_zone ||
         incoming.next_zones != baseline.next_zones ||
         incoming.zone_sequence != baseline.zone_sequence ||
         incoming.planned_path != baseline.planned_path;
}

std::vector<std::string> robots_to_resume(
  const std::vector<std::string> & conflicting_robots,
  const std::vector<std::string> & pause_robot_ids)
{
  std::unordered_set<std::string> pause_set(
    pause_robot_ids.begin(), pause_robot_ids.end());
  std::vector<std::string> resume;
  resume.reserve(conflicting_robots.size());
  for (const auto & id : conflicting_robots) {
    if (id.empty() || pause_set.count(id) > 0) {
      continue;
    }
    resume.push_back(id);
  }
  std::sort(resume.begin(), resume.end());
  resume.erase(std::unique(resume.begin(), resume.end()), resume.end());
  return resume;
}

bool is_replan_watch_timed_out(
  const PendingReplanWatch & watch,
  std::chrono::steady_clock::time_point now,
  double timeout_sec)
{
  if (timeout_sec <= 0.0 || watch.candidate_robot_id.empty()) {
    return false;
  }
  const double age = std::chrono::duration<double>(now - watch.armed_at).count();
  return age > timeout_sec;
}

}  // namespace zone_manager
