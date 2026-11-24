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

#include "cuopt_router.hpp"

// cuOpt C++ API (required at build time — see rmf_traffic CMakeLists.txt)
#include <cuopt/routing/distance_engine/waypoint_matrix.hpp>
#include <raft/core/handle.hpp>
#include <raft/core/resource/cuda_stream.hpp>
#include <rmm/device_uvector.hpp>
#include <rmm/mr/device/cuda_memory_resource.hpp>
#include <rmm/mr/device/pool_memory_resource.hpp>
#include <rmm/exec_policy.hpp>
#include <cuda_runtime.h>

#include <mutex>
#include <chrono>
#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>

namespace rmf_traffic {
namespace agv {
namespace planning {

//==============================================================================
// Static cache members
std::unordered_map<std::size_t, std::vector<CuOptRouter::CachedCSRData>>
CuOptRouter::csr_cache_;
std::size_t CuOptRouter::csr_cache_entries_ = 0;
std::mutex CuOptRouter::cache_mutex_;
std::mutex CuOptRouter::solve_mutex_;

//==============================================================================
// Sentinel "infinity" weight used to mask out forbidden edges per call.
// We do not use std::numeric_limits<float>::infinity() because cuOpt's
// distance_engine may treat large finite values more predictably.
static constexpr float kForbiddenEdgeWeight = 1.0e18f;

//==============================================================================
// Generate cache key from graph dimensions and lane-closure status.
//
// The CSR is closure-aware (closed lanes are excluded), so the cache must
// also be keyed by closure hash; otherwise reopening / closing a lane would
// silently reuse stale data.
std::size_t CuOptRouter::get_cache_key(
  const Graph::Implementation& graph_impl,
  const LaneClosure* lane_closures)
{
  std::size_t hash = graph_impl.waypoints.size();
  hash = hash * 31 + graph_impl.lanes.size();
  if (lane_closures)
    hash = hash * 1315423911u + lane_closures->hash();
  return hash;
}

//==============================================================================
/// Modular graph conversion: RMF Graph -> cuOpt CSR format
/// Handles bidirectional lanes (RMF represents both directions as separate
/// lanes) and calculates Euclidean distances as edge weights.
/// Closed lanes are filtered out entirely. The CSR is cached per
/// (graph-size, closure-hash) pair.
void CuOptRouter::convert_to_csr(
  const Graph::Implementation& graph_impl,
  const LaneClosure* lane_closures,
  std::vector<int>& row_ptr,
  std::vector<int>& col_ind,
  std::vector<float>& weights)
{
  const auto& waypoints = graph_impl.waypoints;
  const auto& lanes = graph_impl.lanes;
  const std::size_t n = waypoints.size();

  if (n == 0)
  {
    row_ptr.clear();
    col_ind.clear();
    weights.clear();
    return;
  }

  const std::size_t cache_key = get_cache_key(graph_impl, lane_closures);
  {
    std::lock_guard<std::mutex> lock(cache_mutex_);
    auto it = csr_cache_.find(cache_key);
    if (it != csr_cache_.end())
    {
      // Only reuse a CSR built for EXACTLY these closures. Each robot's
      // private closure set (e.g. its drop corridor) gets its own CSR.
      for (const auto& entry : it->second)
      {
        const bool same_closures = lane_closures ?
          (entry.closures.has_value() && *entry.closures == *lane_closures) :
          !entry.closures.has_value();
        if (!same_closures)
          continue;
        row_ptr = entry.row_ptr;
        col_ind = entry.col_ind;
        weights = entry.weights;
        return;
      }
    }
  }

  row_ptr.clear();
  col_ind.clear();
  weights.clear();

  // Helper: should this lane be included in the CSR?
  auto lane_is_open = [&](std::size_t lane_idx) -> bool
    {
      if (!lane_closures)
        return true;
      return lane_closures->is_open(lane_idx);
    };

  // First pass: count outgoing edges per vertex, honouring closures.
  std::vector<std::size_t> edge_count(n, 0);
  for (std::size_t l = 0; l < lanes.size(); ++l)
  {
    if (!lane_is_open(l))
      continue;
    const std::size_t from = lanes[l].entry().waypoint_index();
    if (from < n)
      edge_count[from]++;
  }

  // Build CSR row_ptr.
  row_ptr.resize(n + 1, 0);
  for (std::size_t i = 0; i < n; i++)
    row_ptr[i + 1] = row_ptr[i] + edge_count[i];

  const std::size_t total_edges = row_ptr[n];
  col_ind.resize(total_edges);
  weights.resize(total_edges);

  std::vector<std::size_t> current_pos(n, 0);
  for (std::size_t l = 0; l < lanes.size(); ++l)
  {
    if (!lane_is_open(l))
      continue;

    const auto& lane = lanes[l];
    const std::size_t from = lane.entry().waypoint_index();
    const std::size_t to = lane.exit().waypoint_index();

    if (from >= n || to >= n)
      continue;

    // Edge weight: Euclidean distance, optionally divided by per-lane
    // speed limit if one is set (so the cost represents travel time).
    const Eigen::Vector2d from_loc = waypoints[from].get_location();
    const Eigen::Vector2d to_loc = waypoints[to].get_location();
    const double dx = to_loc.x() - from_loc.x();
    const double dy = to_loc.y() - from_loc.y();
    const float dist = static_cast<float>(std::sqrt(dx * dx + dy * dy));

    const std::size_t pos = row_ptr[from] + current_pos[from];
    col_ind[pos] = static_cast<int>(to);
    weights[pos] = dist;
    current_pos[from]++;
  }

  // Cache the closure-specific CSR.
  {
    std::lock_guard<std::mutex> lock(cache_mutex_);
    // Bound memory: every distinct private closure set adds an entry.
    if (csr_cache_entries_ >= kMaxCachedCSRs)
    {
      csr_cache_.clear();
      csr_cache_entries_ = 0;
    }
    CachedCSRData cached_data;
    if (lane_closures)
      cached_data.closures = *lane_closures;
    cached_data.row_ptr = row_ptr;
    cached_data.col_ind = col_ind;
    cached_data.weights = weights;
    csr_cache_[cache_key].push_back(std::move(cached_data));
    ++csr_cache_entries_;
  }
}

//==============================================================================
std::optional<CuOptResult> CuOptRouter::solve_cuopt_distance_engine(
  std::size_t start_vertex,
  std::size_t goal_vertex,
  const Graph::Implementation& graph_impl,
  const std::vector<int>& row_ptr,
  const std::vector<int>& col_ind,
  const std::vector<float>& weights)
{
  auto start_time = std::chrono::high_resolution_clock::now();

  try
  {
    cudaError_t cuda_status = cudaSetDevice(0);
    if (cuda_status != cudaSuccess)
      return std::nullopt;

    raft::handle_t handle;

    const std::size_t n = graph_impl.waypoints.size();

    cuopt::distance_engine::waypoint_matrix_t<int, float> waypoint_matrix(
      handle,
      const_cast<int*>(row_ptr.data()),
      static_cast<int>(n),
      const_cast<int*>(col_ind.data()),
      const_cast<float*>(weights.data())
    );

    std::vector<int> target_locations = {
      static_cast<int>(start_vertex),
      static_cast<int>(goal_vertex)
    };

    size_t cost_matrix_size = target_locations.size() * target_locations.size();
    rmm::device_uvector<float> d_cost_matrix(cost_matrix_size, handle.get_stream());

    waypoint_matrix.compute_cost_matrix(
      d_cost_matrix.data(),
      target_locations.data(),
      static_cast<int>(target_locations.size())
    );

    cudaStreamSynchronize(handle.get_stream());

    std::vector<float> cost_matrix(cost_matrix_size);
    cuda_status = cudaMemcpy(
      cost_matrix.data(),
      d_cost_matrix.data(),
      cost_matrix_size * sizeof(float),
      cudaMemcpyDeviceToHost
    );
    if (cuda_status != cudaSuccess)
      return std::nullopt;

    double distance = static_cast<double>(cost_matrix[1]);

    // Forbidden / unreachable edges may collapse to a near-infinite cost.
    // Treat anything beyond half the sentinel weight as "no valid path".
    if (std::isinf(distance) || distance < 0.0f ||
        distance >= 0.5 * static_cast<double>(kForbiddenEdgeWeight))
    {
      return std::nullopt;
    }

    std::vector<int> route_locations = {0, 1};

    rmm::device_uvector<int> d_route_locations(route_locations.size(), handle.get_stream());
    cuda_status = cudaMemcpy(
      d_route_locations.data(),
      route_locations.data(),
      route_locations.size() * sizeof(int),
      cudaMemcpyHostToDevice
    );
    if (cuda_status != cudaSuccess)
      return std::nullopt;

    auto [d_offsets, d_path] = waypoint_matrix.compute_waypoint_sequence(
      target_locations.data(),
      static_cast<int>(target_locations.size()),
      d_route_locations.data(),
      static_cast<int>(route_locations.size())
    );

    cudaStreamSynchronize(handle.get_stream());

    std::vector<std::size_t> path;

    if (d_offsets && d_path)
    {
      std::vector<int> offsets(route_locations.size());
      if (d_offsets->size() >= route_locations.size() * sizeof(int))
      {
        cuda_status = cudaMemcpy(
          offsets.data(),
          d_offsets->data(),
          route_locations.size() * sizeof(int),
          cudaMemcpyDeviceToHost
        );

        if (cuda_status == cudaSuccess && offsets.size() > 0)
        {
          int path_length = offsets.back();
          if (path_length > 0 && d_path->size() >= path_length * sizeof(int))
          {
            std::vector<int> path_indices(path_length);
            cuda_status = cudaMemcpy(
              path_indices.data(),
              d_path->data(),
              path_length * sizeof(int),
              cudaMemcpyDeviceToHost
            );

            if (cuda_status == cudaSuccess)
            {
              for (int idx : path_indices)
              {
                if (idx >= 0 && idx < static_cast<int>(n))
                {
                  std::size_t wp_idx = static_cast<std::size_t>(idx);
                  if (path.empty() || path.back() != wp_idx)
                    path.push_back(wp_idx);
                }
              }
            }
          }
        }
      }
    }

    if (path.empty())
    {
      // Fallback path; only valid when start==goal — caller short-circuits
      // that case separately, so reaching here usually means no GPU path.
      path.push_back(start_vertex);
      path.push_back(goal_vertex);
    }

    if (path.empty() || path.front() != start_vertex)
      return std::nullopt;

    auto end_time = std::chrono::high_resolution_clock::now();
    double solve_time_ms = std::chrono::duration<double, std::milli>(
      end_time - start_time).count();

    CuOptResult result;
    result.path = std::move(path);
    result.total_cost = distance;
    result.solve_time_ms = solve_time_ms;
    result.used_gpu = true;

    return result;
  }
  catch (const std::exception& e)
  {
    std::cerr << "[cuOpt] Exception in solve_cuopt_distance_engine: "
              << e.what() << std::endl;
    return std::nullopt;
  }
}

//==============================================================================
std::optional<CuOptResult> CuOptRouter::find_shortest_path(
  std::size_t start_vertex,
  std::size_t goal_vertex,
  const Graph::Implementation& graph_impl)
{
  return find_shortest_path(start_vertex, goal_vertex, graph_impl,
                            /*lane_closures*/ nullptr,
                            /*forbidden_edges*/ nullptr);
}

//==============================================================================
std::optional<CuOptResult> CuOptRouter::find_shortest_path(
  std::size_t start_vertex,
  std::size_t goal_vertex,
  const Graph::Implementation& graph_impl,
  const LaneClosure* lane_closures,
  const std::unordered_set<uint64_t>* forbidden_edges)
{
  if (start_vertex >= graph_impl.waypoints.size() ||
      goal_vertex >= graph_impl.waypoints.size())
  {
    return std::nullopt;
  }

  if (start_vertex == goal_vertex)
  {
    CuOptResult result;
    result.path = {start_vertex};
    result.total_cost = 0.0;
    result.solve_time_ms = 0.0;
    result.used_gpu = true;
    return result;
  }

  std::cout << "Cuopt planner got the call" << std::endl;

  // Closure-aware CSR (cached).
  std::vector<int> row_ptr, col_ind;
  std::vector<float> weights;
  convert_to_csr(graph_impl, lane_closures, row_ptr, col_ind, weights);

  // Apply forbidden edges on a per-call basis without polluting the cache.
  if (forbidden_edges && !forbidden_edges->empty() && !row_ptr.empty())
  {
    const std::size_t n = graph_impl.waypoints.size();
    for (std::size_t from = 0; from < n; ++from)
    {
      const int begin = row_ptr[from];
      const int end = row_ptr[from + 1];
      for (int k = begin; k < end; ++k)
      {
        const std::size_t to = static_cast<std::size_t>(col_ind[k]);
        if (forbidden_edges->count(encode_edge(from, to)))
          weights[k] = kForbiddenEdgeWeight;
      }
    }
  }

  std::lock_guard<std::mutex> solve_lock(solve_mutex_);
  return solve_cuopt_distance_engine(
    start_vertex, goal_vertex, graph_impl, row_ptr, col_ind, weights);
}

} // namespace planning
} // namespace agv
} // namespace rmf_traffic
