cmake_minimum_required(VERSION 3.8)
project(ecbs_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rmf_fleet_msgs REQUIRED)
find_package(rmf_traffic_msgs REQUIRED)
find_package(rmf_building_map_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(monitor REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ClearPoint.msg"
  "msg/WaitPoint.msg"
  "msg/WaitDependency.msg"
  "msg/PathPoint.msg"
  "msg/ToZoneManager.msg"
  "msg/EcbsReplan.msg"
  "msg/EvaluatePathWaypoint.msg"
  "msg/FromZoneManager.msg"
  "msg/RegisterUpdate.msg"
  "msg/RouteOccurrence.msg"
  "msg/ConflictRecord.msg"
  "msg/MovementProposal.msg"
  "msg/GrantEvent.msg"
  "msg/ClearanceEvent.msg"
  "msg/AuthorityConflict.msg"
  "srv/EvaluatePath.srv"
  "srv/BaseExtensionQuery.srv"
  "srv/CommitProposal.srv"
  DEPENDENCIES builtin_interfaces
)

add_library(${PROJECT_NAME}_core SHARED
  src/schedule_cache.cpp
  src/ecbs_solver.cpp
  src/evaluate_path.cpp
  src/nav_graph_junctions.cpp
  src/replan_candidate.cpp
  src/nav_graph.cpp
  src/conflict_store.cpp
  src/passage_arbitration.cpp
  src/proposal_engine.cpp
  src/passage_action.cpp
  src/action_compatibility.cpp
  src/dependency_graph.cpp
  src/conflict_group_builder.cpp
  src/group_passage_scheduler.cpp
  src/global_resolver.cpp
)

target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(${PROJECT_NAME}_core PUBLIC cxx_std_17)

# evaluate_path.cpp uses generated srv/msg types.
rosidl_get_typesupport_target(
  ecbs_typesupport_target
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)
# Keyword form only (do not mix with ament_target_dependencies plain link).
target_link_libraries(${PROJECT_NAME}_core PUBLIC
  "${ecbs_typesupport_target}"
  yaml-cpp
)
# ETA motion model (monitor::path_point_reach_times_sec) for TimedPath stamps.
if(TARGET monitor::monitor_eta)
  target_link_libraries(${PROJECT_NAME}_core PUBLIC monitor::monitor_eta)
elseif(TARGET monitor_eta)
  target_link_libraries(${PROJECT_NAME}_core PUBLIC monitor_eta)
endif()

add_executable(ecbs_planner_node
  src/main.cpp
  src/ecbs_node.cpp
)

ament_target_dependencies(ecbs_planner_node
  rclcpp
  builtin_interfaces
  rmf_fleet_msgs
  rmf_traffic_msgs
)

target_link_libraries(ecbs_planner_node
  ${PROJECT_NAME}_core
)

target_link_libraries(ecbs_planner_node "${ecbs_typesupport_target}")

add_executable(to_zone_manager_viz
  src/to_zone_viz_node.cpp
)

ament_target_dependencies(to_zone_manager_viz
  rclcpp
  builtin_interfaces
  geometry_msgs
  visualization_msgs
  rmf_fleet_msgs
)

target_link_libraries(to_zone_manager_viz "${ecbs_typesupport_target}")

add_executable(evaluate_path_server
  src/evaluate_path_server_main.cpp
)

ament_target_dependencies(evaluate_path_server
  rclcpp
  std_msgs
  rmf_fleet_msgs
  rmf_building_map_msgs
  ament_index_cpp
  monitor
)

target_link_libraries(evaluate_path_server
  ${PROJECT_NAME}_core
  "${ecbs_typesupport_target}"
)
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}_core
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Imported as ecbs_planner::ecbs_planner_core by dependents.
add_library(ecbs_planner::ecbs_planner_core ALIAS ${PROJECT_NAME}_core)

install(
  TARGETS ecbs_planner_node to_zone_manager_viz evaluate_path_server
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/mock_evaluate_path_client.py
  scripts/ecbs_test_common.py
  scripts/dummy_path_publisher.py
  scripts/dummy_evaluate_client.py
  scripts/dummy_conflict_publisher.py
  scripts/fcfs_register_tester.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  scripts/README_DUMMY_TEST.md
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_core)
ament_export_dependencies(
  rclcpp builtin_interfaces geometry_msgs visualization_msgs
  rmf_fleet_msgs rmf_traffic_msgs rmf_building_map_msgs monitor
  rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_ecbs_solver test/test_ecbs_solver.cpp)
  if(TARGET test_ecbs_solver)
    target_link_libraries(test_ecbs_solver ${PROJECT_NAME}_core)
  endif()

  ament_add_gtest(test_conflict_store test/test_conflict_store.cpp)
  if(TARGET test_conflict_store)
    target_link_libraries(test_conflict_store ${PROJECT_NAME}_core)
  endif()

  ament_add_gtest(test_passage_arbitration test/test_passage_arbitration.cpp)
  if(TARGET test_passage_arbitration)
    target_link_libraries(test_passage_arbitration ${PROJECT_NAME}_core)
  endif()

  ament_add_gtest(test_proposal_engine test/test_proposal_engine.cpp)
  if(TARGET test_proposal_engine)
    target_link_libraries(test_proposal_engine ${PROJECT_NAME}_core)
  endif()

  ament_add_gtest(test_global_resolver test/test_global_resolver.cpp)
  if(TARGET test_global_resolver)
    target_link_libraries(test_global_resolver ${PROJECT_NAME}_core)
  endif()

  ament_add_gtest(test_group_passage_scheduler test/test_group_passage_scheduler.cpp)
  if(TARGET test_group_passage_scheduler)
    target_link_libraries(test_group_passage_scheduler ${PROJECT_NAME}_core)
  endif()

  ament_add_gtest(test_solver_config_defaults test/test_solver_config_defaults.cpp)
  if(TARGET test_solver_config_defaults)
    target_link_libraries(test_solver_config_defaults ${PROJECT_NAME}_core)
  endif()
endif()

ament_package()
