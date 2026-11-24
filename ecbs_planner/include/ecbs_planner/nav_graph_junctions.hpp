#ifndef ECBS_PLANNER__NAV_GRAPH_JUNCTIONS_HPP_
#define ECBS_PLANNER__NAV_GRAPH_JUNCTIONS_HPP_

#include <cstddef>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "ecbs_planner/types.hpp"

namespace ecbs_planner
{

/// Junction kind from nav-graph vertex degree (undirected neighbours).
enum class JunctionKind
{
  TSection,   // degree == 3
  Cross,      // degree >= 4
};

struct JunctionPoint
{
  double x{0.0};
  double y{0.0};
  JunctionKind kind{JunctionKind::TSection};
};

/// Sentinel WaitDependency::amr_id for a precautionary T-section gate.
inline constexpr const char * kJunctionWaitT = "T";
/// Sentinel WaitDependency::amr_id for a precautionary cross (+) gate.
inline constexpr const char * kJunctionWaitCross = "+";

inline bool is_junction_wait_id(const std::string & id)
{
  return id == kJunctionWaitT || id == kJunctionWaitCross;
}

inline const char * junction_wait_label(JunctionKind kind)
{
  return kind == JunctionKind::Cross ? kJunctionWaitCross : kJunctionWaitT;
}

/// T/cross near the base end, with the approach stop safety_m before it.
struct JunctionApproach
{
  std::size_t junction_i{0};   // path index of the centre
  JunctionKind kind{JunctionKind::TSection};
  std::size_t wait_i{0};       // path index of the approach stop
  ClearPoint wait_point;       // exact XY safety_m before the centre
};

/// Full nav-graph cache + T/cross junctions with a spatial hash for O(1)
/// near-lookups (low CPU on every evaluate). Load once from YAML or a
/// BuildingMap Graph; do not depend on filtered /nav_graphs1.
class NavGraphJunctions
{
public:
  /// Replace cache from undirected vertex degrees.
  /// \p vertices = (x, y) per graph vertex index; \p edges = (v1, v2) pairs.
  void set_from_edges(
    const std::vector<std::pair<double, double>> & vertices,
    const std::vector<std::pair<std::size_t, std::size_t>> & edges);

  /// Load full nav-graph YAML (levels.<level>.vertices / lanes). Returns false
  /// on missing file / parse error / empty vertices.
  bool load_from_yaml(const std::string & yaml_path, const std::string & level);

  /// True if the full graph has been loaded (vertices cached).
  bool loaded() const;

  /// True if no T/cross junctions were found (graph may still be loaded).
  bool empty() const;

  std::size_t size() const;

  /// Cached full-graph sizes (for logging).
  std::size_t vertex_count() const;
  std::size_t edge_count() const;

  /// Snapshot of current junctions (for tests / logging).
  std::vector<JunctionPoint> junctions() const;

  /// True if (x, y) is within \p match_m of a T or cross centre (hash lookup).
  bool near_junction(double x, double y, double match_m) const;

  /// Kind of the nearest T/cross within \p match_m, or nullopt.
  std::optional<JunctionKind> kind_near(
    double x, double y, double match_m) const;

  /// Hard rule: wait must not lie on a T/cross centre — always place it
  /// \p safety_m *before* the centre along the path.
  std::size_t snap_wait_off_junction(
    const TimedPath & path,
    std::size_t wait_i,
    std::size_t min_i,
    double safety_m) const;

  /// Same hard rule for an exact wait XY. Returns nullopt if the wait is
  /// already clear of every T/cross safety disc (or already a safe before-stop).
  std::optional<std::pair<std::size_t, ClearPoint>> snap_wait_point_off_junction(
    const TimedPath & path,
    std::size_t wait_i,
    double wait_x,
    double wait_y,
    std::size_t min_i,
    double safety_m) const;

  /// Junction whose centre lies within \p safety_m of the base end \p base_hi
  /// along \p path (either side of it): a robot stopping at the base end would
  /// sit inside that junction's safety zone, so the base must end \p safety_m
  /// *before* the centre (JunctionApproach::wait_*). A junction deeper inside
  /// the base (>= safety_m before its end) or further beyond it returns
  /// nothing — the full base is given. Nullopt if no such junction.
  std::optional<JunctionApproach> approach_near_base_end(
    const TimedPath & path,
    std::size_t start,
    std::size_t base_hi,
    double safety_m) const;

private:
  std::optional<JunctionPoint> find_near(
    double x, double y, double match_m) const;

  /// Path index of the junction centre nearest \p wait along the path, or npos.
  std::size_t junction_anchor_on_path(
    const TimedPath & path,
    std::size_t wait_i,
    double wait_x,
    double wait_y,
    std::size_t min_i,
    double safety_m) const;

  /// Hold index + XY exactly \p safety_m before path index \p junction_i.
  std::pair<std::size_t, ClearPoint> point_before_junction(
    const TimedPath & path,
    std::size_t junction_i,
    std::size_t min_i,
    double safety_m) const;

  void rebuild_hash_locked();

  static std::uint64_t cell_key(double x, double y);
  static constexpr double kCellM = 0.5;  // spatial hash cell size

  mutable std::mutex mutex_;
  /// Full graph (loaded once; used to rebuild junctions).
  std::vector<std::pair<double, double>> vertices_;
  std::vector<std::pair<std::size_t, std::size_t>> edges_;
  std::vector<JunctionPoint> junctions_;
  /// cell_key → indices into junctions_ (may have several per cell).
  std::unordered_map<std::uint64_t, std::vector<std::size_t>> hash_;
};

}  // namespace ecbs_planner

#endif  // ECBS_PLANNER__NAV_GRAPH_JUNCTIONS_HPP_
