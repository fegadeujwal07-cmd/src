cmake_minimum_required(VERSION 3.5.0)

project(rmf_fleet_adapter)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Threads REQUIRED)
if(DEFINED ENV{RMF_ENABLE_FAILOVER})
  if($ENV{RMF_ENABLE_FAILOVER} EQUAL 1)
    find_package(stubborn_buddies REQUIRED)
    find_package(stubborn_buddies_msgs REQUIRED)
  endif()
endif()
include(GNUInstallDirs)

set(dep_pkgs
  rclcpp
  rclcpp_components
  rmf_utils
  rmf_dispenser_msgs
  rmf_ingestor_msgs
  rmf_fleet_msgs
  rmf_task_msgs
  rmf_traffic_ros2
  rmf_task_ros2
  rmf_battery
  rmf_task
  rmf_task_sequence
  std_msgs
  rmf_api_msgs
  rmf_websocket
  rmf_building_map_msgs
  nlohmann_json
  nlohmann_json_schema_validator_vendor
  # Zone-manager conflict messages. full_control subscribes to /zone_conflicts
  # so that a robot paused by the zone manager (over VDA5050, outside RMF)
  # stops reserving the corridor ahead of it in the traffic schedule.
  zone_manager
  traffic_manager
  # Wait-only ECBS core + msgs used by EcbsGate.
  ecbs_planner
)
foreach(pkg ${dep_pkgs})
  find_package(${pkg} REQUIRED)
endforeach()

if(BUILD_TESTING)
  find_package(ament_cmake_uncrustify REQUIRED)
  find_file(uncrustify_config_file
    NAMES "rmf_code_style.cfg"
    PATHS "${rmf_utils_DIR}/../../../share/rmf_utils/")

  ament_uncrustify(
    include src test rmf_rxcpp/include rmf_rxcpp/src rmf_rxcpp/test
    CONFIG_FILE ${uncrustify_config_file}
    LANGUAGE C++
    MAX_LINE_LENGTH 80
  )
endif()
# -----------------------------------------------------------------------------

add_subdirectory(rmf_rxcpp)

# -----------------------------------------------------------------------------

file(GLOB_RECURSE rmf_fleet_adapter_srcs "src/rmf_fleet_adapter/*.cpp")

add_library(rmf_fleet_adapter SHARED
  ${rmf_fleet_adapter_srcs}
)

ament_target_dependencies(rmf_fleet_adapter
  PUBLIC
    rmf_traffic_ros2
    rmf_task_ros2
    yaml-cpp
    rmf_fleet_msgs
    rmf_task_msgs
    rmf_dispenser_msgs
    rmf_ingestor_msgs
    rmf_building_map_msgs
    rclcpp
    ecbs_planner
)

target_link_libraries(rmf_fleet_adapter
  PUBLIC
    rmf_task::rmf_task
    rmf_task_sequence::rmf_task_sequence
    rmf_battery::rmf_battery
  PRIVATE
    rmf_rxcpp
    rmf_websocket::rmf_websocket
    nlohmann_json::nlohmann_json
    rmf_api_msgs::rmf_api_msgs
    nlohmann_json_schema_validator
)

# Link wait-only ECBS solver/cache library (EcbsGate).
if(TARGET ecbs_planner::ecbs_planner_core)
  target_link_libraries(rmf_fleet_adapter PRIVATE ecbs_planner::ecbs_planner_core)
elseif(TARGET ecbs_planner_core)
  target_link_libraries(rmf_fleet_adapter PRIVATE ecbs_planner_core)
endif()

target_include_directories(rmf_fleet_adapter
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/rmf_api_generate_schema_headers/include> # for auto-generated schema headers
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
  PRIVATE
    ${rmf_dispenser_msgs_INCLUDE_DIRS}
    ${rmf_ingestor_msgs_INCLUDE_DIRS}
    ${rmf_api_msgs_INCLUDE_DIRS}
    ${rmf_websocket_INCLUDE_DIR}
    ${rmf_traffic_ros2_INCLUDE_DIRS}
    ${rmf_building_map_msgs_INCLUDE_DIRS}
    ${nlohmann_json_schema_validator_INCLUDE_DIRS}
)

if (BUILD_TESTING)
  find_package(ament_cmake_catch2 REQUIRED)
  find_package(std_msgs REQUIRED)

  ament_add_catch2(
    test_rmf_fleet_adapter
      test/main.cpp
      test/services/test_FindEmergencyPullover.cpp
      test/services/test_FindPath.cpp
      test/test_Task.cpp
    TIMEOUT 300
  )
  ament_target_dependencies(test_rmf_fleet_adapter
    PUBLIC
      rmf_task_msgs
      rmf_dispenser_msgs
      rmf_ingestor_msgs
      std_msgs
      rmf_building_map_msgs
      rmf_websocket
  )
  target_include_directories(test_rmf_fleet_adapter
    PRIVATE
      # private includes of rmf_fleet_adapter
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/rmf_fleet_adapter>
      ${rmf_api_msgs_INCLUDE_DIRS}
      ${nlohmann_json_schema_validator_INCLUDE_DIRS}
  )
  target_link_libraries(test_rmf_fleet_adapter
    PRIVATE
      # private libraries of rmf_fleet_adapter
      rmf_rxcpp
      rmf_fleet_adapter
      rmf_utils::rmf_utils
      rmf_api_msgs::rmf_api_msgs
      nlohmann_json_schema_validator
  )

  target_compile_definitions(test_rmf_fleet_adapter
    PRIVATE
      "-DTEST_RESOURCES_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/test/resources/\"")

endif ()

# -----------------------------------------------------------------------------

add_executable(full_control src/full_control/main.cpp)

target_link_libraries(full_control
  PRIVATE
    rmf_fleet_adapter
    ${rmf_fleet_msgs_LIBRARIES}
    ${rmf_task_msgs_LIBRARIES}
    # /zone_conflicts messages. Linked the same keyword-signature way as the
    # message packages above — ament_target_dependencies() cannot be used here
    # because it emits the plain target_link_libraries() form, and CMake
    # refuses to mix plain and keyword signatures on one target.
    ${zone_manager_LIBRARIES}
    ${traffic_manager_LIBRARIES}
    ${ecbs_planner_LIBRARIES}
)

target_include_directories(full_control
  PRIVATE
    ${rmf_fleet_msgs_INCLUDE_DIRS}
    ${rmf_task_msgs_INCLUDE_DIRS}
    ${zone_manager_INCLUDE_DIRS}
    ${traffic_manager_INCLUDE_DIRS}
    ${ecbs_planner_INCLUDE_DIRS}
)

# -----------------------------------------------------------------------------

add_library(robot_state_aggregator_main SHARED
  src/robot_state_aggregator/RobotStateAggregator.cpp
)

target_include_directories(robot_state_aggregator_main
  PRIVATE
    include
    ${rclcpp_INCLUDE_DIRS}
    ${rmf_fleet_msgs_INCLUDE_DIRS}
    ${stubborn_buddies_msgs_INCLUDE_DIRS})

target_compile_definitions(robot_state_aggregator_main
  PRIVATE "COMPOSITION_BUILDING_DLL"
)


if(DEFINED ENV{RMF_ENABLE_FAILOVER})
  if($ENV{RMF_ENABLE_FAILOVER} EQUAL 1)
    ament_target_dependencies(robot_state_aggregator_main
      "rclcpp"
      "rclcpp_components"
      "std_msgs"
      "stubborn_buddies_msgs"
      "rmf_fleet_msgs"
    )
  else()
    ament_target_dependencies(robot_state_aggregator_main
      "rclcpp"
      "rclcpp_components"
      "std_msgs"
      "rmf_fleet_msgs"
    )
  endif()
else()
  ament_target_dependencies(robot_state_aggregator_main
    "rclcpp"
    "rclcpp_components"
    "std_msgs"
    "rmf_fleet_msgs"
  )
endif()

rclcpp_components_register_nodes(robot_state_aggregator_main
  "RobotStateAggregator"
)

set(node_plugins "${node_plugins}RobotStateAggregator;
    $<TARGET_FILE:robot_state_aggregator_main>\n")

# -----------------------------------------------------------------------------

rmf_api_generate_schema_headers(
  PACKAGE rmf_fleet_adapter
  SCHEMAS_DIR ${CMAKE_CURRENT_LIST_DIR}/schemas
)

# -----------------------------------------------------------------------------

ament_export_targets(rmf_fleet_adapter HAS_LIBRARY_TARGET)
ament_export_dependencies(${dep_pkgs})

install(
  TARGETS
    rmf_fleet_adapter
    full_control
    robot_state_aggregator_main
  EXPORT rmf_fleet_adapter
  RUNTIME DESTINATION lib/rmf_fleet_adapter
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

# -----------------------------------------------------------------------------

install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

ament_package()
