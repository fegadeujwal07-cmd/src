cmake_minimum_required(VERSION 3.8)
project(execution)

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
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(monitor REQUIRED)
find_package(zone_manager REQUIRED)
find_package(traffic_manager REQUIRED)
find_package(rmf_fleet_adapter REQUIRED)
find_package(rmf_api_msgs REQUIRED)
find_package(rmf_fleet_msgs REQUIRED)
find_package(rmf_dispenser_msgs REQUIRED)
find_package(rmf_ingestor_msgs REQUIRED)
find_package(rmf_task_msgs REQUIRED)
find_package(rmf_traffic_msgs REQUIRED)
find_package(rmf_traffic_ros2 REQUIRED)
find_package(rmf_task_ros2 REQUIRED)
find_package(rmf_utils REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_common REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(MOSQUITTO QUIET libmosquitto)
if(NOT MOSQUITTO_FOUND)
  # Fallback when only the runtime package (libmosquitto1) is installed and
  # libmosquitto-dev / pkg-config metadata is missing.
  find_path(MOSQUITTO_INCLUDE_DIRS mosquitto.h
    HINTS
      ${CMAKE_CURRENT_SOURCE_DIR}/third_party/mosquitto/include
      /usr/include
      /usr/local/include
  )
  find_library(MOSQUITTO_LIBRARIES
    NAMES mosquitto libmosquitto.so.1
    PATHS
      /usr/lib/x86_64-linux-gnu
      /usr/lib/aarch64-linux-gnu
      /usr/lib
      /usr/local/lib
  )
  if(NOT MOSQUITTO_INCLUDE_DIRS OR NOT MOSQUITTO_LIBRARIES)
    message(FATAL_ERROR
      "libmosquitto not found.\n"
      "Install development files with:\n"
      "  sudo apt install libmosquitto-dev\n"
      "Or ensure third_party/mosquitto/include/mosquitto.h exists and "
      "libmosquitto.so.1 is available on the system.")
  endif()
  message(STATUS "Found libmosquitto via fallback: ${MOSQUITTO_LIBRARIES}")
  message(STATUS "  include: ${MOSQUITTO_INCLUDE_DIRS}")
endif()

# ------------------------------------------------------------------------------
# Python Install
# ------------------------------------------------------------------------------
# Install the `execution` Python package onto PYTHONPATH (registers env hooks
# via ament_cmake_python so `import execution.mqtt_client` etc. work at runtime).
ament_python_install_package(${PROJECT_NAME})

# Install Python programs (entry-point style executables) that exist
set(python_programs)
foreach(prog
    execution_task_state_bridge.py
    continuous_inbound_delivery.py
)
  set(prog_path "${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}/${prog}")
  if(EXISTS "${prog_path}")
    list(APPEND python_programs ${PROJECT_NAME}/${prog})
  else()
    message(WARNING "Python program not found: ${prog_path}")
  endif()
endforeach()

if(python_programs)
install(PROGRAMS
    ${python_programs}
  DESTINATION lib/${PROJECT_NAME}
)
endif()

# ------------------------------------------------------------------------------
# C++ fleet adapter (replaces execution_full_control.py)
# ------------------------------------------------------------------------------
# Code review checklist 1.1: compile as C++20, no compiler extensions.
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(execution_full_control
  src/execution_full_control.cpp
  src/drop_corridor.cpp
  src/mqtt_client.cpp
)
target_include_directories(execution_full_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(execution_full_control
  rclcpp
  monitor
  zone_manager
  traffic_manager
  rmf_fleet_msgs
  rmf_dispenser_msgs
  rmf_ingestor_msgs
  rmf_task_msgs
  geometry_msgs
  std_msgs
  tf2_ros
  tf2_geometry_msgs
)
target_link_libraries(execution_full_control
  yaml-cpp ${MOSQUITTO_LIBRARIES}
  traffic_manager::traffic_manager__rosidl_typesupport_cpp)
target_include_directories(execution_full_control PRIVATE ${MOSQUITTO_INCLUDE_DIRS})
target_compile_options(execution_full_control PRIVATE ${MOSQUITTO_CFLAGS_OTHER})
install(TARGETS execution_full_control
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# ------------------------------------------------------------------------------
# Launch & Config
# ------------------------------------------------------------------------------
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  docs
  DESTINATION share/${PROJECT_NAME}
)

# ------------------------------------------------------------------------------
# Tests
# ------------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# ------------------------------------------------------------------------------
# Navigation Graph Generation (Optional)
# ------------------------------------------------------------------------------
# full_control reads share/execution/config/nav_graphs/0.yaml. The old package
# did not keep that file in source; it generated it from R3.building.yaml.
option(GENERATE_NAV_GRAPHS "Automatically generate nav graphs during build" ON)

if(GENERATE_NAV_GRAPHS)
  # Prefer the build directory for generated artifacts
  set(nav_src_yaml        ${CMAKE_CURRENT_SOURCE_DIR}/config/R3.building.yaml)
  set(nav_out_dir         ${CMAKE_CURRENT_BINARY_DIR}/nav_graphs)
  set(nav_phony_marker    ${nav_out_dir}/.generated)

  # Check if source file exists
  if(NOT EXISTS ${nav_src_yaml})
    message(WARNING "Nav graph source file not found: ${nav_src_yaml}")
    message(WARNING "Skipping nav graph generation. Create the file or disable GENERATE_NAV_GRAPHS.")
    set(GENERATE_NAV_GRAPHS OFF)
  endif()

  if(GENERATE_NAV_GRAPHS)
    # Get workspace root (package lives under src/execution)
    get_filename_component(workspace_root "${CMAKE_SOURCE_DIR}/../.." ABSOLUTE)
    set(building_map_generator_path "${workspace_root}/install/rmf_building_map_tools/lib/rmf_building_map_tools/building_map_generator")
    set(setup_bash_path "${workspace_root}/install/setup.bash")

    # The building_map_generator is a Python entry point that needs the ROS2 environment
    # Source setup.bash to set up PYTHONPATH and other environment variables
    add_custom_command(
      OUTPUT ${nav_phony_marker}
      COMMAND ${CMAKE_COMMAND} -E make_directory ${nav_out_dir}
      COMMAND bash -c "set -e; if [ -f \"${setup_bash_path}\" ]; then source \"${setup_bash_path}\"; fi; if [ -f \"${building_map_generator_path}\" ]; then \"${building_map_generator_path}\" nav ${nav_src_yaml} ${nav_out_dir}; elif command -v ros2 >/dev/null 2>&1; then ros2 run rmf_building_map_tools building_map_generator nav ${nav_src_yaml} ${nav_out_dir}; else echo 'Error: building_map_generator not found and ros2 not available.'; echo 'Expected executable at: ${building_map_generator_path}'; echo 'Build rmf_building_map_tools first with: colcon build --packages-select rmf_building_map_tools'; exit 1; fi"
      # Create the declared OUTPUT so the rule is considered satisfied
      COMMAND ${CMAKE_COMMAND} -E touch ${nav_phony_marker}
      DEPENDS ${nav_src_yaml}
      BYPRODUCTS
        ${nav_out_dir}/levels.nav.json
        ${nav_out_dir}/traffic_editor.nav.yaml
      COMMENT "Generating nav graphs from ${nav_src_yaml}"
      VERBATIM
    )

    add_custom_target(generate_nav_graphs ALL
      DEPENDS ${nav_phony_marker}
    )

    # Install the generated graphs with the rest of your config
    install(DIRECTORY ${nav_out_dir}/
            DESTINATION share/${PROJECT_NAME}/config/nav_graphs
            PATTERN ".generated" EXCLUDE)
  endif()
endif()

if(NOT GENERATE_NAV_GRAPHS)
  set(nav_src_fallback ${CMAKE_CURRENT_SOURCE_DIR}/config/nav_graphs/0.yaml)
  if(EXISTS ${nav_src_fallback})
    message(STATUS "Nav graph auto-generation off; using checked-in ${nav_src_fallback}")
    install(FILES ${nav_src_fallback}
            DESTINATION share/${PROJECT_NAME}/config/nav_graphs)
  else()
    message(WARNING
      "No nav graph will be installed. full_control needs "
      "share/execution/config/nav_graphs/0.yaml")
  endif()
endif()


# ------------------------------------------------------------------------------
# Package
# ------------------------------------------------------------------------------
ament_package()
