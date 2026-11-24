// =============================================================================
// 4D / ASRS drop corridor: pure lane-selection logic (no ROS dependencies).
// =============================================================================
// Nav graph (from building YAML / RFID sheet):
//   * horizontal orange rows  = X highways (constant y)
//   * vertical orange columns = main Z spines (constant x)
//   * numbered Z columns      = aisle Z highways (pick / drop columns)
//
// After pickup the loaded robot may only use (private to that robot):
//   * every X-highway lane,
//   * every main Z-spine lane,
//   * its pickup Z column,
//   * its drop Z column (lanes into that drop),
// and every other pick/drop spur is closed for it. Other robots keep their own
// private free set so they can still reach their drop; if two meet, ECBS waits.
// =============================================================================

#ifndef EXECUTION_DROP_CORRIDOR_HPP_
#define EXECUTION_DROP_CORRIDOR_HPP_

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace execution::drop_corridor {

struct Point {
  double x = 0.0;
  double y = 0.0;
};

struct LaneEnds {
  int start = -1;
  int end = -1;
};

struct Geometry {
  std::vector<Point> vertices;
  std::vector<LaneEnds> lanes;
};

// Free lines for one loaded robot: all X highways, main spines, pick Z, drop Z.
struct Lines {
  double pickup_column_x = 0.0;
  double drop_column_x = 0.0;
  std::vector<double> highway_ys;
  std::vector<double> spine_xs;
  // Z highways (spines) are only free BETWEEN the outermost X highways, as in
  // the RFID sheet (the orange Z columns stop at the X-highway rows; above /
  // below them the same column is a pallet aisle). Set by FindCorridor.
  bool limit_spines_to_highway_span = false;
  double spine_ymin = 0.0;
  double spine_ymax = 0.0;
};

// Highway lines from the 4D layout sheet (A1462 RFID_Location_Sheet,
// "1500 loc G0"). Empty vector = infer that set from the graph geometry.
struct LayoutHighways {
  std::vector<double> x_highway_ys;  // horizontal orange rows (constant y)
  std::vector<double> z_highway_xs;  // vertical orange columns (constant x)
};

struct Corridor {
  Lines lines;
  std::vector<std::size_t> closed_lanes;
  std::size_t free_lane_count = 0;
  std::size_t pickup_vertex = 0;
  std::size_t drop_vertex = 0;
};

struct Hold {
  std::string task_id;
  Point drop;
  Lines lines;
  bool plan_ready = false;
  Point plan_pose;
  double plan_ready_sec = 0.0;
  // When the corridor was closed; used for the no-plan safety release.
  double closed_sec = 0.0;
};

// Free all X highways + main spines + pick/drop Z columns; block the rest.
// Returns nullopt when that set does not connect pickup→drop (BFS).
// `layout` gives the highway lines from the layout sheet; an empty list falls
// back to inferring that set from the graph.
std::optional<Corridor> FindCorridor(const Geometry& geometry,
                                     const Point& pickup, const Point& drop,
                                     const LayoutHighways& layout = {});

bool IsPathOnCorridor(const Lines& lines, const Point& drop,
                      double end_tolerance_m, const std::vector<Point>& path);

bool HasStartedTravel(bool reports_moving, const Point& plan_pose,
                      const Point& current_pose, double travel_start_m);

double Distance(const Point& a, const Point& b);

}  // namespace execution::drop_corridor

#endif  // EXECUTION_DROP_CORRIDOR_HPP_
