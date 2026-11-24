#just version requirement because of use
cmake_minimum_required(VERSION 3.8)
project(zone_manager)

#GCC compiler
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#ROS2 dependencies for the compilation if any of code uses ros libraries then it must know the path of header.
#Required means if any of the dependey which is required is not available or not installed then stop the build and throw an error.
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rmf_fleet_msgs REQUIRED)
find_package(monitor REQUIRED)



rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Conflict.msg"
  "msg/ReplanCandidate.msg"
  DEPENDENCIES builtin_interfaces
)

add_library(${PROJECT_NAME}_core SHARED
  src/world_model.cpp
  src/conflict_detector.cpp
  src/replan_priority.cpp
  src/behind_robot.cpp
  src/conflict_decision.cpp
  src/post_replan.cpp
)

target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}_core
  rclcpp
)

add_executable(zone_manager_node
  src/main.cpp
  src/zone_manager_node.cpp
)

ament_target_dependencies(zone_manager_node
  rclcpp
  rclcpp_lifecycle
  lifecycle_msgs
  rmf_fleet_msgs
  monitor
)

target_link_libraries(zone_manager_node
  ${PROJECT_NAME}_core
)

rosidl_get_typesupport_target(
  zone_manager_typesupport_target
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)
target_link_libraries(zone_manager_node "${zone_manager_typesupport_target}")

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

install(
  TARGETS zone_manager_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_core)
ament_export_dependencies(
  rclcpp rclcpp_lifecycle lifecycle_msgs rmf_fleet_msgs monitor rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_world_model test/test_world_model.cpp)
  if(TARGET test_world_model)
    target_link_libraries(test_world_model ${PROJECT_NAME}_core)
  endif()
  ament_add_gtest(test_replan_priority test/test_replan_priority.cpp)
  if(TARGET test_replan_priority)
    target_link_libraries(test_replan_priority ${PROJECT_NAME}_core)
  endif()
  ament_add_gtest(test_behind_robot test/test_behind_robot.cpp)
  if(TARGET test_behind_robot)
    target_link_libraries(test_behind_robot ${PROJECT_NAME}_core)
  endif()
  ament_add_gtest(test_conflict_decision test/test_conflict_decision.cpp)
  if(TARGET test_conflict_decision)
    target_link_libraries(test_conflict_decision ${PROJECT_NAME}_core)
  endif()
  ament_add_gtest(test_post_replan test/test_post_replan.cpp)
  if(TARGET test_post_replan)
    target_link_libraries(test_post_replan ${PROJECT_NAME}_core)
  endif()
endif()

ament_package()
