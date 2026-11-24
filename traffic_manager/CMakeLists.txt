cmake_minimum_required(VERSION 3.8)
project(traffic_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rmf_fleet_msgs REQUIRED)
find_package(ecbs_planner REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PathIntent.msg"
  "msg/PathRelease.msg"
  "msg/BaseExtensionRequest.msg"
  DEPENDENCIES builtin_interfaces rmf_fleet_msgs
)

add_library(${PROJECT_NAME}_core SHARED
  src/path_gate.cpp
  src/path_horizon.cpp
  src/release_markers.cpp
)

target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}_core
  rclcpp
  rmf_fleet_msgs
  ecbs_planner
  visualization_msgs
  geometry_msgs
)

rosidl_get_typesupport_target(
  traffic_manager_typesupport_target
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)
target_link_libraries(${PROJECT_NAME}_core "${traffic_manager_typesupport_target}")

add_executable(traffic_manager_node
  src/main.cpp
  src/traffic_manager_node.cpp
)

ament_target_dependencies(traffic_manager_node
  rclcpp
  rmf_fleet_msgs
  ecbs_planner
  visualization_msgs
)

target_link_libraries(traffic_manager_node
  ${PROJECT_NAME}_core
  "${traffic_manager_typesupport_target}"
)

target_include_directories(traffic_manager_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}_core traffic_manager_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS launch/traffic_manager.launch.py
  DESTINATION share/${PROJECT_NAME}/launch
)

install(
  PROGRAMS
    launch/mock_ecbs_test.launch.py
    launch/traffic_manager_ecbs.launch.py
  DESTINATION share/${PROJECT_NAME}/launch
)

install(
  PROGRAMS scripts/mock_ecbs_eval_server.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES launch/traffic_manager.launch.xml
  DESTINATION share/${PROJECT_NAME}/launch
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_path_gate test/test_path_gate.cpp)
  target_link_libraries(test_path_gate ${PROJECT_NAME}_core "${traffic_manager_typesupport_target}")
  target_include_directories(test_path_gate PRIVATE include)

  ament_add_gtest(test_path_horizon test/test_path_horizon.cpp)
  target_link_libraries(test_path_horizon ${PROJECT_NAME}_core "${traffic_manager_typesupport_target}")
  target_include_directories(test_path_horizon PRIVATE include)

  ament_add_gtest(test_mutex_contract test/test_mutex_contract.cpp)
  target_link_libraries(test_mutex_contract ${PROJECT_NAME}_core)
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_core)
ament_export_dependencies(rosidl_default_runtime rmf_fleet_msgs)

ament_package()
