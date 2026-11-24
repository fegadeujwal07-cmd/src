cmake_minimum_required(VERSION 3.8)
project(monitor)

# ------------------------------------------------------------------------------
# Compiler options
# ------------------------------------------------------------------------------
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ------------------------------------------------------------------------------
# Dependencies
# ------------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(rmf_fleet_msgs REQUIRED)
find_package(rmf_building_map_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rmf_traffic REQUIRED)

# ------------------------------------------------------------------------------
# Messages
# ------------------------------------------------------------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MonitoredTaskDetails.msg"
  "msg/RobotZoneStatus.msg"
  "msg/SafetyFieldConflict.msg"
  "msg/SafetyFieldPair.msg"
  "msg/ZoneCatalog.msg"
  # SafetyFieldConflict.msg carries a builtin_interfaces/Time. The other three
  # messages are plain types, so this package never needed the dependency
  # before; without it the generated headers include
  # builtin_interfaces/msg/detail/time__struct.hpp and cannot find it.
  DEPENDENCIES builtin_interfaces
)

# ------------------------------------------------------------------------------
# C++ library — ETA path calculator (shared by zone_monitor + ecbs_planner)
# ------------------------------------------------------------------------------
add_library(${PROJECT_NAME}_eta SHARED
  src/eta_path_calculator.cpp
  src/eta_robot_params.cpp
)

target_include_directories(${PROJECT_NAME}_eta PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}_eta
  yaml-cpp
)

# Imported as monitor::monitor_eta by dependents (ecbs_planner).
add_library(monitor::monitor_eta ALIAS ${PROJECT_NAME}_eta)

# ------------------------------------------------------------------------------
# C++ library — MonitoredTaskManager (unchanged Python port)
# ------------------------------------------------------------------------------
add_library(${PROJECT_NAME}_logic SHARED
  src/monitored_task_manager.cpp
)

target_include_directories(${PROJECT_NAME}_logic PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}_logic
  rclcpp
)

target_link_libraries(${PROJECT_NAME}_logic
  nlohmann_json::nlohmann_json
)

rosidl_get_typesupport_target(
  monitor_logic_typesupport_target
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)
target_link_libraries(${PROJECT_NAME}_logic "${monitor_logic_typesupport_target}")

# ------------------------------------------------------------------------------
# C++ library — NavGraphStore (filter vertices/lanes by zone set)
# ------------------------------------------------------------------------------
add_library(${PROJECT_NAME}_nav_graph SHARED
  src/nav_graph_store.cpp
)

target_include_directories(${PROJECT_NAME}_nav_graph PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}_nav_graph
  rclcpp
  rmf_building_map_msgs
  visualization_msgs
  geometry_msgs
  std_msgs
  builtin_interfaces
)

target_link_libraries(${PROJECT_NAME}_nav_graph
  yaml-cpp
)

# ------------------------------------------------------------------------------
# C++ library — ZoneTracker (adopts shared NavGraphStore cache)
# ------------------------------------------------------------------------------
add_library(${PROJECT_NAME}_zone SHARED
  src/zone_tracker.cpp
)

target_include_directories(${PROJECT_NAME}_zone PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}_zone
  ${PROJECT_NAME}_nav_graph
  yaml-cpp
)

ament_target_dependencies(${PROJECT_NAME}_zone
  rclcpp
)

# ------------------------------------------------------------------------------
# Zone monitor node (zones + shared-cache /nav_graphs1)
# ------------------------------------------------------------------------------
add_executable(zone_monitor_node
  src/zone_monitor_node.cpp
  src/eta_path_calculator.cpp
  src/eta_robot_params.cpp
)

target_include_directories(zone_monitor_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

ament_target_dependencies(zone_monitor_node
  rclcpp
  rmf_fleet_msgs
  rmf_building_map_msgs
  visualization_msgs
  std_msgs
)

target_link_libraries(zone_monitor_node
  ${PROJECT_NAME}_eta
  ${PROJECT_NAME}_zone
  ${PROJECT_NAME}_nav_graph
  yaml-cpp
)

rosidl_get_typesupport_target(
  zone_node_typesupport_target
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)
target_link_libraries(zone_monitor_node "${zone_node_typesupport_target}")

# ------------------------------------------------------------------------------
# Optional standalone nav_graphs1 publisher (loads YAML again — prefer zone_monitor)
# ------------------------------------------------------------------------------
add_executable(nav_graphs1_zone_publisher_node
  src/nav_graphs1_zone_publisher_node.cpp
)

ament_target_dependencies(nav_graphs1_zone_publisher_node
  rclcpp
  rmf_building_map_msgs
)

target_link_libraries(nav_graphs1_zone_publisher_node
  ${PROJECT_NAME}_nav_graph
)

rosidl_get_typesupport_target(
  nav_graphs1_typesupport_target
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)
target_link_libraries(nav_graphs1_zone_publisher_node "${nav_graphs1_typesupport_target}")

# ------------------------------------------------------------------------------
# Safety field node
# ------------------------------------------------------------------------------
# A rectangular safety field per AMR, centred on the robot's MEASURED position
# from /fleet_states and turned to its measured heading, so the field is carried
# by the robot rather than lying on the map under it. Overlap is a
# separating-axis test on two oriented rectangles, in
# include/monitor/safety_field_geometry.hpp — NOT rmf_traffic, which answers a
# swept "will they collide along these trajectories" question and whose Profile
# holds circles only. The node publishes markers and ids; zone_manager owns the
# pause.
add_executable(safety_field_node
  src/safety_field_node.cpp
)

target_include_directories(safety_field_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

ament_target_dependencies(safety_field_node
  rclcpp
  rmf_fleet_msgs
  visualization_msgs
  geometry_msgs
  builtin_interfaces
)

rosidl_get_typesupport_target(
  safety_field_typesupport_target
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)
target_link_libraries(safety_field_node "${safety_field_typesupport_target}")

# ------------------------------------------------------------------------------
# Path deviation node (multi-segment cross-track error + debounce + recovery)
# ------------------------------------------------------------------------------
# Subscribes : /Monitored_task_details (monitor/msg/MonitoredTaskDetails)
#              /fleet_states           (rmf_fleet_msgs/FleetState)
#              /rmf_task_state_updates (std_msgs/String, YAML)
# Publishes  : /path_deviation         (std_msgs/String, JSON deviation|recovered)
# Consumed by execution (path_deviation_callback) to send startPause/stopPause.
# Uses ZoneTracker::normalize_robot_id / parse_planned_path_strings, hence the
# link to ${PROJECT_NAME}_zone; yaml-cpp parses the task-state YAML.
rosidl_get_typesupport_target(
  path_dev_typesupport_target
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)

add_executable(path_deviation
  src/path_deviation.cpp
)

target_include_directories(path_deviation PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

ament_target_dependencies(path_deviation
  rclcpp
  rmf_fleet_msgs
  std_msgs
)

target_link_libraries(path_deviation
  ${PROJECT_NAME}_zone
  yaml-cpp
  "${path_dev_typesupport_target}"
)

# Same source under a second name, for consistency with the other *_node targets.
add_executable(path_deviation_node
  src/path_deviation.cpp
)

target_include_directories(path_deviation_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

ament_target_dependencies(path_deviation_node
  rclcpp
  rmf_fleet_msgs
  std_msgs
)

target_link_libraries(path_deviation_node
  ${PROJECT_NAME}_zone
  yaml-cpp
  "${path_dev_typesupport_target}"
)

# ------------------------------------------------------------------------------
# Install
# ------------------------------------------------------------------------------
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}_eta ${PROJECT_NAME}_logic ${PROJECT_NAME}_zone ${PROJECT_NAME}_nav_graph
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS
    zone_monitor_node
    nav_graphs1_zone_publisher_node
    safety_field_node
    path_deviation
    path_deviation_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_eta ${PROJECT_NAME}_logic ${PROJECT_NAME}_zone ${PROJECT_NAME}_nav_graph)
ament_export_dependencies(rclcpp nlohmann_json yaml-cpp rmf_building_map_msgs visualization_msgs geometry_msgs builtin_interfaces rosidl_default_runtime)

# ------------------------------------------------------------------------------
# Launch (optional)
# ------------------------------------------------------------------------------
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
endif()

# ------------------------------------------------------------------------------
# Tests
# ------------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  ament_add_gtest(test_monitored_task_manager test/test_monitored_task_manager.cpp)
  if(TARGET test_monitored_task_manager)
    target_link_libraries(test_monitored_task_manager ${PROJECT_NAME}_logic)
    ament_target_dependencies(test_monitored_task_manager rclcpp)
  endif()

  ament_add_gtest(test_zone_tracker test/test_zone_tracker.cpp)
  if(TARGET test_zone_tracker)
    target_link_libraries(test_zone_tracker ${PROJECT_NAME}_zone)
  endif()

  ament_add_gtest(test_eta_path_calculator test/test_eta_path_calculator.cpp)
  if(TARGET test_eta_path_calculator)
    target_link_libraries(test_eta_path_calculator ${PROJECT_NAME}_eta)
  endif()

  # safety_field_geometry.hpp is header-only, so there is nothing to link —
  # only the include path is needed.
  ament_add_gtest(test_safety_field_geometry test/test_safety_field_geometry.cpp)
  if(TARGET test_safety_field_geometry)
    target_include_directories(test_safety_field_geometry PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    )
  endif()

  ament_add_gtest(test_safety_field_tracker test/test_safety_field_tracker.cpp)
  if(TARGET test_safety_field_tracker)
    target_include_directories(test_safety_field_tracker PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    )
  endif()

  # path_deviation.hpp includes ROS msg and rclcpp definitions
  ament_add_gtest(test_path_deviation test/test_path_deviation.cpp)
  if(TARGET test_path_deviation)
    target_include_directories(test_path_deviation PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    )
    ament_target_dependencies(test_path_deviation
      rclcpp
      rmf_fleet_msgs
      std_msgs
    )
    target_link_libraries(test_path_deviation
      "${path_dev_typesupport_target}"
    )
  endif()

  ament_lint_auto_find_test_dependencies()
endif()

# ------------------------------------------------------------------------------
# Package
# ------------------------------------------------------------------------------
ament_export_dependencies(rosidl_default_runtime)
ament_package()
