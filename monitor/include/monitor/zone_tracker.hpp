#ifndef MONITOR__ZONE_TRACKER_HPP_
#define MONITOR__ZONE_TRACKER_HPP_

#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "monitor/nav_graph_store.hpp"

namespace monitor
{

/// Zone fields for one robot (nav_graphs1 logic).
struct ZoneSnapshot
{
  std::string current_zone;
  /// Up to the next 3 upcoming zones (fewer if fewer remain).
  std::vector<std::string> next_zone;
  /// Remaining zones from current onward (passed zones dropped).
  std::vector<std::string> zone_sequence;

  /// Zones lane-reachable from BEHIND the robot: the space another AMR could
  /// drive into and rear-end this one.
  ///
  /// A LIST, not one zone: standing at the centre of a 90-degree junction with
  /// the heading perpendicular to the route, there is no single zone straight
  /// back — the adjacent zones to either side are both behind, and both need
  /// protecting.
  ///
  /// Derived from pose + heading + the nav graph ONLY. Nothing here reads
  /// zone_sequence, because the planner plans from where the robot IS: at zone
  /// 6 the sequence is [6,7,8,9] with the robot at index 0, so there is no
  /// earlier element to read and a sequence-based rear zone is empty exactly
  /// when a stationary robot needs one most.
  std::vector<std::string> rear_zones;

  bool operator==(const ZoneSnapshot & other) const
  {
    return current_zone == other.current_zone &&
           next_zone == other.next_zone &&
           zone_sequence == other.zone_sequence &&
           rear_zones == other.rear_zones;
  }

  bool operator!=(const ZoneSnapshot & other) const
  {
    return !(*this == other);
  }
};

/// Standalone zone tracker — does not modify MonitoredTaskManager state.
///
/// Optimizations:
///  - precomputed vertices_xy / vertex_zone_labels (once per map load)
///  - uniform-grid spatial index for zone + nearest-vertex lookups
///  - per-robot zone->sequence-index map for O(1) progress updates
class ZoneTracker
{
public:
  /// Load from YAML (parses once via NavGraphStore, then adopts that cache).
  bool load_map(const std::string & yaml_path, const std::string & level = "L1");

  /// Adopt an already-loaded nav graph (no second YAML parse).
  bool load_from_store(const NavGraphStore & store);

  /// yaw defaults so existing call sites keep compiling; without a real
  /// heading the rear zones cannot be computed and come back empty.
  void update_robot_pose(
    const std::string & robot_id, double x, double y, double yaw = 0.0);

  void update_robot_planned_path(
    const std::string & robot_id,
    const std::vector<std::string> & planned_path);

  /// Numeric path points (x, y) — preferred over CSV strings when available.
  void update_robot_planned_path_xy(
    const std::string & robot_id,
    const std::vector<std::pair<double, double>> & planned_points);

  void clear_robot_task(const std::string & robot_id);

  ZoneSnapshot get_robot_zones(const std::string & robot_id);

  /// Final zone snapshot when a task ends: use live pose if already at the route
  /// destination, otherwise snap to the last zone on the planned path (RMF may
  /// mark the task complete before the pose enters the destination zone bbox).
  ZoneSnapshot get_robot_zones_at_completion(const std::string & robot_id);

  /// Zone from live pose only — for robots with no active task.
  ZoneSnapshot get_robot_idle_zones(const std::string & robot_id);

  static std::string normalize_robot_id(const std::string & robot_name);

  static std::vector<std::string> zone_sequence_from_xy_points(
    const std::vector<std::pair<double, double>> & points,
    const std::vector<std::pair<double, double>> & vertices_xy,
    const std::vector<std::vector<std::string>> & vertex_zone_labels,
    const std::unordered_map<std::string, std::vector<std::pair<double, double>>> & zone_rects,
    int interpolate_steps = 8);

  /// Instance path that uses precomputed helpers + spatial index.
  std::vector<std::string> zone_sequence_from_xy_points_indexed(
    const std::vector<std::pair<double, double>> & points,
    int interpolate_steps = 8) const;

  static ZoneSnapshot current_and_next_from_pose(
    double px,
    double py,
    const std::vector<std::pair<double, double>> & vertices_xy,
    const std::vector<std::vector<std::string>> & vertex_zone_labels,
    const std::unordered_map<std::string, std::vector<std::pair<double, double>>> & zone_rects,
    const std::vector<std::string> & zone_sequence,
    int seq_progress);

  ZoneSnapshot current_and_next_from_pose_indexed(
    double px,
    double py,
    const std::vector<std::string> & zone_sequence,
    const std::unordered_map<std::string, int> & zone_to_seq_idx,
    int seq_progress) const;

  static std::string find_zone_at_point(
    double px,
    double py,
    const std::unordered_map<std::string, std::vector<std::pair<double, double>>> & zone_rects);

  static std::vector<std::pair<double, double>> parse_planned_path_strings(
    const std::vector<std::string> & planned_path);

private:
  struct Vertex
  {
    double x{0.0};
    double y{0.0};
    std::vector<std::string> zone_labels;
  };

  struct RobotZoneState
  {
    bool has_pose{false};
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
    bool has_task{false};

    /// Pose the rear zones were last computed at, so a parked robot is not
    /// re-walked every tick.
    bool has_rear_eval{false};
    double rear_eval_x{0.0};
    double rear_eval_y{0.0};
    double rear_eval_yaw{0.0};

    /// Debounce counters, keyed by zone id. Asymmetric on purpose: a zone is
    /// claimed quickly and released slowly, so a marginal rear zone lingers
    /// rather than flickering off. Every flicker would be a pause/resume pair
    /// on the wire.
    std::unordered_map<std::string, int> rear_seen;
    std::unordered_map<std::string, int> rear_missed;
    /// Last RAW walk result. Re-fed through the debounce every tick even when
    /// the graph walk is skipped, otherwise a parked robot — the case this
    /// feature exists for — would never reach the add threshold and would sit
    /// with no rear zones for ever.
    std::vector<std::string> rear_raw;
    /// Debounced result actually published.
    std::vector<std::string> rear_zones;
    std::vector<std::string> zone_sequence;
    std::unordered_map<std::string, int> zone_to_seq_idx;
    int seq_progress{-1};
  };

  struct SpatialGrid
  {
    bool enabled{false};
    double origin_x{0.0};
    double origin_y{0.0};
    double cell_size{5.0};
    int cols{0};
    int rows{0};
    /// cell_key -> zone ids that overlap the cell
    std::unordered_map<int64_t, std::vector<std::string>> zone_cells;
    /// cell_key -> vertex indices
    std::unordered_map<int64_t, std::vector<std::size_t>> vertex_cells;
  };

  static bool point_in_rectangle(
    double px,
    double py,
    const std::vector<std::pair<double, double>> & rect);

  static void build_zone_rects(
    const std::vector<Vertex> & vertices,
    const std::unordered_map<std::string, std::vector<int>> & zone_vertex_indices,
    std::unordered_map<std::string, std::vector<std::pair<double, double>>> & zone_rects);

  void rebuild_precomputed_helpers();
  void build_spatial_index();

  int64_t cell_key(int cx, int cy) const;
  bool world_to_cell(double x, double y, int & cx, int & cy) const;

  std::string find_zone_at_point_indexed(double px, double py) const;
  std::string nearest_vertex_zone(double px, double py) const;

  /// Nearest graph vertex to (px, py) via the spatial grid. O(1) in graph size:
  /// hashes to a cell and widens ring by ring, so a 10k-vertex graph costs the
  /// same as an 87-vertex one. nullopt if no vertex is within search_radius.
  std::optional<std::size_t> nearest_vertex_indexed(
    double px, double py, double search_radius) const;

  /// Zones lane-reachable from behind the robot at (px, py, yaw).
  ///
  /// Walks the lane graph outward from the nearest vertex, accumulating PATH
  /// length rather than straight-line distance, and keeps vertices lying in the
  /// rear half-plane. Path length matters: a zone 2 m away through a wall is
  /// unreachable and must not be claimed, and only a robot driving on a lane
  /// can rear-end this one.
  ///
  /// Deliberately geometric, not historical. The question is "what space is
  /// behind me that something could hit", not "where did I come from" — the two
  /// differ after a 90-degree turn, and the first is the safety-relevant one.
  std::vector<std::string> find_rear_zones_from_graph(
    double px,
    double py,
    double yaw,
    const std::string & current_zone) const;

  /// Feed a raw result through the add/drop counters and return the debounced
  /// set. Mutates state.
  std::vector<std::string> debounce_rear_zones(
    RobotZoneState & state,
    const std::vector<std::string> & raw);

  /// Compute (or reuse) this robot's rear zones and write them into snapshot.
  /// Skips the graph walk unless the pose actually moved.
  void fill_rear_zones(RobotZoneState & state, ZoneSnapshot & snapshot);

  static void rebuild_zone_index_map(
    const std::vector<std::string> & zone_sequence,
    std::unordered_map<std::string, int> & zone_to_seq_idx);

  /// Fill snapshot.next_zone (max 3) and trim snapshot.zone_sequence from current_idx.
  static void apply_remaining_and_lookahead(
    ZoneSnapshot & snapshot,
    const std::vector<std::string> & full_sequence,
    int current_idx);

  static constexpr int kNextZoneLookahead = 3;

  std::vector<Vertex> vertices_;
  std::vector<std::pair<double, double>> vertices_xy_;
  std::vector<std::vector<std::string>> vertex_zone_labels_;
  std::unordered_map<std::string, std::vector<std::pair<double, double>>> zone_rects_;
  /// Lane adjacency, built once from NavGraphStore::lanes(). Undirected: a
  /// robot can be approached along a lane from either end regardless of the
  /// direction the lane is normally driven.
  std::vector<std::vector<std::size_t>> vertex_adjacency_;
  SpatialGrid grid_;
  std::unordered_map<std::string, RobotZoneState> robot_states_;
  bool map_loaded_{false};
};

}  // namespace monitor

#endif  // MONITOR__ZONE_TRACKER_HPP_
