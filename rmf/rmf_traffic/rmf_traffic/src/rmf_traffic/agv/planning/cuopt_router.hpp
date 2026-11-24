/*
 * Copyright (C) 2024 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#ifndef SRC__RMF_TRAFFIC__AGV__PLANNING__CUOPT_ROUTER_HPP
#define SRC__RMF_TRAFFIC__AGV__PLANNING__CUOPT_ROUTER_HPP

#include <rmf_traffic/agv/Graph.hpp>
#include <rmf_traffic/agv/LaneClosure.hpp>
#include "../internal_Graph.hpp"
#include <optional>
#include <vector>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <mutex>

namespace rmf_traffic {
namespace agv {
namespace planning {

//==============================================================================
/// Result of cuOpt routing using distance_engine
struct CuOptResult
{
  std::vector<std::size_t> path;  // Waypoint indices in order
  double total_cost;              // Total path cost/distance from cost matrix
  double solve_time_ms;           // Time taken to solve (milliseconds)
  bool used_gpu;                  // Whether GPU acceleration was used (always true for cuOpt)
};

//==============================================================================
/// CuOpt Router using NVIDIA cuOpt distance_engine C++ API
///
/// Replaces A* / Dijkstra with GPU-accelerated routing while remaining
/// schedule-aware: callers can supply lane closures and a set of
/// per-call forbidden directed edges (from negotiation conflicts) so the
/// returned path avoids them.
class CuOptRouter
{
public:
  /// Encode a directed edge (from_wp, to_wp) into a single 64-bit key.
  /// Used by the forbidden-edges API below.
  static inline uint64_t encode_edge(std::size_t from, std::size_t to)
  {
    return (static_cast<uint64_t>(from) << 32) |
           static_cast<uint64_t>(static_cast<uint32_t>(to));
  }

  /// Backward-compatible API: shortest path with no lane closures and no
  /// forbidden edges. Internally forwards to the extended overload.
  static std::optional<CuOptResult> find_shortest_path(
    std::size_t start_vertex,
    std::size_t goal_vertex,
    const Graph::Implementation& graph_impl);

  /// Extended GPU shortest-path search.
  ///
  /// \param[in] start_vertex Starting waypoint index
  /// \param[in] goal_vertex  Goal waypoint index
  /// \param[in] graph_impl   RMF graph (waypoints + lanes)
  /// \param[in] lane_closures
  ///   When non-null, lanes marked closed in this object are excluded from the
  ///   CSR graph the GPU solver sees.
  /// \param[in] forbidden_edges
  ///   When non-null, every (from, to) edge encoded via `encode_edge()` in
  ///   this set is given an effectively-infinite weight, forcing the solver
  ///   to route around it. The forbidden list is applied on top of the
  ///   (cached) CSR without polluting the cache.
  /// \return CuOptResult on success, std::nullopt on failure / no path.
  static std::optional<CuOptResult> find_shortest_path(
    std::size_t start_vertex,
    std::size_t goal_vertex,
    const Graph::Implementation& graph_impl,
    const LaneClosure* lane_closures,
    const std::unordered_set<uint64_t>* forbidden_edges);

private:
  /// Cached CSR data
  struct CachedCSRData
  {
    /// Exact closure set this CSR was built for (nullopt = no closures).
    /// A cache hit is only used when it equals the caller's closures, so a
    /// hash collision can never hand one robot another robot's lane set.
    std::optional<LaneClosure> closures;
    std::vector<int> row_ptr;
    std::vector<int> col_ind;
    std::vector<float> weights;
  };

  /// Convert RMF graph to CSR format for cuOpt (with caching).
  /// `lane_closures`, if non-null, excludes closed lanes from the CSR.
  static void convert_to_csr(
    const Graph::Implementation& graph_impl,
    const LaneClosure* lane_closures,
    std::vector<int>& row_ptr,
    std::vector<int>& col_ind,
    std::vector<float>& weights);

  /// Solve using cuOpt distance_engine (GPU-accelerated)
  static std::optional<CuOptResult> solve_cuopt_distance_engine(
    std::size_t start_vertex,
    std::size_t goal_vertex,
    const Graph::Implementation& graph_impl,
    const std::vector<int>& row_ptr,
    const std::vector<int>& col_ind,
    const std::vector<float>& weights);

  /// Cache for CSR data (keyed by graph size + closure hash)
  /// Several entries may share a key (hash collision); each is checked for an
  /// exact closure match before use.
  static std::unordered_map<std::size_t, std::vector<CachedCSRData>> csr_cache_;
  /// Upper bound on cached CSRs (each private drop corridor adds one).
  static constexpr std::size_t kMaxCachedCSRs = 64;
  static std::size_t csr_cache_entries_;
  static std::mutex cache_mutex_;
  /// cuOpt distance_engine / CUDA context is not safe under concurrent calls.
  static std::mutex solve_mutex_;

  /// Generate cache key from graph + closures
  static std::size_t get_cache_key(
    const Graph::Implementation& graph_impl,
    const LaneClosure* lane_closures);
};

} // namespace planning
} // namespace agv
} // namespace rmf_traffic

#endif // SRC__RMF_TRAFFIC__AGV__PLANNING__CUOPT_ROUTER_HPP
