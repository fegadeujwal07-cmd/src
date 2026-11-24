/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include <rclcpp/rclcpp.hpp>

#include <rmf_visualization_schedule/ScheduleDataNode.hpp>

#include <rmf_traffic_ros2/schedule/MirrorManager.hpp>
#include <rmf_traffic_ros2/StandardNames.hpp>
#include <rmf_traffic_ros2/Time.hpp>
#include <rmf_traffic_ros2/Trajectory.hpp>

namespace rmf_visualization_schedule {

//==============================================================================
class ScheduleDataNode::Implementation
{
public:

  Implementation() {}
  Implementation(const Implementation&) {}

  struct Data
  {
    rmf_traffic_ros2::schedule::MirrorManager mirror;

    Data(rmf_traffic_ros2::schedule::MirrorManager mirror_)
    : mirror(std::move(mirror_))
    {
      // Do nothing
    }
  };

  std::unique_ptr<Data> data;
  mutable std::mutex mutex;

  void start(Data data);
};

//==============================================================================
std::shared_ptr<ScheduleDataNode> ScheduleDataNode::make(
  const std::string& node_name,
  rmf_traffic::Duration wait_time,
  const rclcpp::NodeOptions& options)
{
  const auto start_time = std::chrono::steady_clock::now();
  std::shared_ptr<ScheduleDataNode> schedule_data(
    new ScheduleDataNode(std::move(node_name), options));

  // Creating a mirror manager that queries over all
  // Spacetime in the database schedule
  auto mirror_mgr_future = rmf_traffic_ros2::schedule::make_mirror(
    schedule_data, rmf_traffic::schedule::query_all(),
    &schedule_data->_pimpl->mutex);

  const auto stop_time = start_time + wait_time;
  while (rclcpp::ok() && std::chrono::steady_clock::now() < stop_time)
  {
    rclcpp::spin_some(schedule_data);
    using namespace std::chrono_literals;

    if (mirror_mgr_future.wait_for(0s) == std::future_status::ready)
    {
      schedule_data->_pimpl->start(
        Implementation::Data{mirror_mgr_future.get()});
      return schedule_data;
    }
  }

  RCLCPP_ERROR(
    schedule_data->get_logger(),
    "Mirror was not initialized in enough time [%ss]!",
    std::to_string(rmf_traffic::time::to_seconds(wait_time)).c_str());
  return nullptr;
}

//==============================================================================
ScheduleDataNode::ScheduleDataNode(
  std::string node_name,
  const rclcpp::NodeOptions& options)
: Node(std::move(node_name), options),
  _pimpl(rmf_utils::make_impl<Implementation>(Implementation{}))
{
  // Do nothing
}

//==============================================================================
void ScheduleDataNode::Implementation::start(
  ScheduleDataNode::Implementation::Data data_)
{
  data = std::make_unique<Data>(std::move(data_));
  data->mirror.update();
}

//==============================================================================
auto ScheduleDataNode::get_elements(
  const RequestParam& request_param) const -> std::vector<Element>
{
  std::lock_guard<std::mutex> lock(_pimpl->mutex);
  std::vector<Element> elements;
  const std::vector<std::string> maps {std::move(request_param.map_name)};
  const auto query = rmf_traffic::schedule::make_query(
    maps,
    &request_param.start_time,
    &request_param.finish_time);

  const auto view = _pimpl->data->mirror.view()->query(query);

  for (const auto& element : view)
    elements.push_back(element);

  return elements;
}

//==============================================================================
rmf_traffic::Time ScheduleDataNode::now()
{
  return rmf_traffic_ros2::convert(get_clock()->now());
}

//==============================================================================
std::mutex& ScheduleDataNode::get_mutex()
{
  return _pimpl->mutex;
}

//==============================================================================
const std::unordered_set<uint64_t> ScheduleDataNode::get_conflict_ids() const
{
  // Conflict ids used to come from the schedule node's negotiation notices.
  // Traffic negotiation has been removed (zone_manager handles conflicts), so
  // there are never any. Kept so ScheduleVisualizer::is_conflict() still works.
  return {};
}

//==============================================================================
const std::vector<std::vector<uint64_t>>
ScheduleDataNode::get_conflict_groups() const
{
  // Always empty, see get_conflict_ids(). The dashboard reads the "conflicts"
  // field of every trajectory response (rmf-web map.tsx: resp.conflicts
  // .flatMap), so TrajectoryServer must keep sending it, as an empty list.
  return {};
}

} // namespace rmf_visualization_schedule

