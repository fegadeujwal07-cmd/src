cmake_minimum_required(VERSION 3.10)

project(rmf_traffic VERSION 3.2.0)

set(CMAKE_EXPORT_COMPILE_COMMANDS on)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_BUILD_TYPE)
  # Use the Release build type by default if the user has not specified one
  set(CMAKE_BUILD_TYPE Release)
endif()

include(GNUInstallDirs)

# We need to use the internal version of FCL because the upstream version has
# bugs that are not yet fixed.
#find_package(fcl 0.6 QUIET)

if(fcl_FOUND)
  message(STATUS "Using installed FCL version: ${FCL_VERSION}")
else()
  message(STATUS "Using in-source FCL")
  add_subdirectory(thirdparty/fcl)
endif()
# TODO(MXG): Remove everything related to maintaining compatibility with 0.5,
# since we are now requiring FCL-0.6+
set(using_new_fcl true)
set(FCL_LIBRARIES fcl)

find_package(rmf_utils REQUIRED)
find_package(Threads REQUIRED)
find_package(yaml-cpp REQUIRED)

find_package(eigen3_cmake_module QUIET)
if (eigen3_cmake_module_FOUND)
  set(RMF_TRAFFIC_DEPENDS_EIGEN3_CMAKE_MODULE "find_dependency(eigen3_cmake_module)")
endif()
find_package(Eigen3 REQUIRED)

# ===== NVIDIA cuOpt (required) =====
# rmf_traffic links the cuOpt distance_engine C++ API from a Python venv layout
# (lib/python*/site-packages/libcuopt, nvidia/cuda_runtime, librmm, libraft, ...).
get_filename_component(WORKSPACE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../../.." ABSOLUTE)
message(STATUS "Workspace root: ${WORKSPACE_ROOT}")

set(RMF_TRAFFIC_CUOPT_VENV "" CACHE PATH
  "Python venv root containing NVIDIA cuOpt under lib/python*/site-packages/libcuopt. "
  "If empty, searches env RMF_TRAFFIC_CUOPT_VENV, ~/mutex/cuopt, ~/mutex/.../routing_feature, then this workspace.")

set(_cuopt_venv_candidates "")
if(RMF_TRAFFIC_CUOPT_VENV)
  list(APPEND _cuopt_venv_candidates "${RMF_TRAFFIC_CUOPT_VENV}")
endif()
if(DEFINED ENV{RMF_TRAFFIC_CUOPT_VENV} AND NOT "$ENV{RMF_TRAFFIC_CUOPT_VENV}" STREQUAL "")
  list(APPEND _cuopt_venv_candidates "$ENV{RMF_TRAFFIC_CUOPT_VENV}")
endif()
# This workspace's own cuOpt venv (<workspace>/routing_feature/cuopt-env)
list(APPEND _cuopt_venv_candidates "${WORKSPACE_ROOT}/routing_feature/cuopt-env")
# Typical layout when colcon ws is not the mutex/rmf_ws tree (cuOpt venv lives under $HOME/mutex)
if(DEFINED ENV{HOME})
  list(APPEND _cuopt_venv_candidates
    "$ENV{HOME}/mutex/cuopt"
    #"$ENV{HOME}/FMS_Saksham/routing_feature/cuopt-env"
    #"$ENV{HOME}/FMS_Saksham/routing_feature"
    "/home/apurva/routing_feature"
    "/home/apurva/routing_feature/cuopt-env"
  )
endif()
list(APPEND _cuopt_venv_candidates
  "${WORKSPACE_ROOT}/src/routing_feature/cuopt-env"
  "${WORKSPACE_ROOT}/src/cuopt_ws/cuopt-env"
)

set(_cuopt_found FALSE)
set(_cuopt_tried_ok "")
set(_seen_venv "")
foreach(_venv_root IN LISTS _cuopt_venv_candidates)
  if(NOT EXISTS "${_venv_root}")
    continue()
  endif()
  get_filename_component(_venv_abs "${_venv_root}" ABSOLUTE)
  list(FIND _seen_venv "${_venv_abs}" _dup)
  if(NOT _dup EQUAL -1)
    continue()
  endif()
  list(APPEND _seen_venv "${_venv_abs}")
  list(APPEND _cuopt_tried_ok "${_venv_abs}")
  message(STATUS "Checking cuOpt venv: ${_venv_abs}")

  file(GLOB _cuopt_roots "${_venv_abs}/lib/python*/site-packages/libcuopt")
  foreach(_cand IN LISTS _cuopt_roots)
    set(_hdr "${_cand}/include/cuopt/routing/distance_engine/waypoint_matrix.hpp")
    if(EXISTS "${_hdr}")
      get_filename_component(CUOPT_SITE_PACKAGES "${_cand}/.." ABSOLUTE)
      file(GLOB _cudart_libs "${CUOPT_SITE_PACKAGES}/nvidia/cuda_runtime/lib/libcudart.so*")
      if(_cudart_libs)
        list(SORT _cudart_libs)
        list(GET _cudart_libs 0 CUDART_LIB)
        set(CUOPT_ENV_DIR "${_venv_abs}")
        set(CUOPT_ROOT "${_cand}")
        set(_cuopt_found TRUE)
        break()
      endif()
    endif()
  endforeach()
  if(_cuopt_found)
    break()
  endif()
endforeach()

if(NOT _cuopt_found)
  string(REPLACE ";" "\n    " _tried_print "${_cuopt_tried_ok}")
  if(_tried_print STREQUAL "")
    set(_tried_print "(no existing venv paths in candidate list)")
  endif()
  message(FATAL_ERROR
    "rmf_traffic requires NVIDIA cuOpt in a Python venv (libcuopt headers + libcudart).\n"
    "  Required files:\n"
    "    .../lib/python*/site-packages/libcuopt/include/cuopt/routing/distance_engine/waypoint_matrix.hpp\n"
    "    .../site-packages/nvidia/cuda_runtime/lib/libcudart.so*\n"
    "  Venv roots that exist and were checked:\n    ${_tried_print}\n"
    "  Fix: install cuOpt wheels into a venv, then either:\n"
    "    export RMF_TRAFFIC_CUOPT_VENV=/path/to/venv\n"
    "    colcon build --cmake-args -DRMF_TRAFFIC_CUOPT_VENV=/path/to/venv\n"
    "  Or create ${WORKSPACE_ROOT}/src/routing_feature/cuopt-env (or cuopt_ws/cuopt-env) with those packages.")
endif()

message(STATUS "rmf_traffic: cuOpt enabled (${CUOPT_ROOT})")
get_filename_component(CUOPT_SITE_PACKAGES "${CUOPT_ROOT}/.." ABSOLUTE)
set(CUOPT_INCLUDE_DIR "${CUOPT_ROOT}/include")
set(CUOPT_LIB_DIR "${CUOPT_ROOT}/lib64")
set(CUOPT_LIB "${CUOPT_LIB_DIR}/libcuopt.so")
set(RMM_ROOT "${CUOPT_SITE_PACKAGES}/librmm")
set(RMM_INCLUDE_DIR "${RMM_ROOT}/include")
set(RMM_LIB_DIR "${RMM_ROOT}/lib64")
set(RMM_LIB "${RMM_LIB_DIR}/librmm.so")
set(RAFT_ROOT "${CUOPT_SITE_PACKAGES}/libraft")
set(RAFT_INCLUDE_DIR "${RAFT_ROOT}/include")
set(RAPIDS_LOGGER_ROOT "${CUOPT_SITE_PACKAGES}/rapids_logger")
set(RAPIDS_LOGGER_INCLUDE_DIR "${RAPIDS_LOGGER_ROOT}/include")
set(CUDA_RUNTIME_LIB_DIR "${CUOPT_SITE_PACKAGES}/nvidia/cuda_runtime/lib")
set(CUDA_RUNTIME_ROOT "${CUOPT_SITE_PACKAGES}/nvidia/cuda_runtime")
set(CUDA_INCLUDE_DIR "${CUDA_RUNTIME_ROOT}/include")
set(CUDA_NVCC_INCLUDE_DIR "${CUOPT_SITE_PACKAGES}/nvidia/cuda_nvcc/include")
file(GLOB NVIDIA_CUDA_PACKAGES "${CUOPT_SITE_PACKAGES}/nvidia/cuda_*/include")
file(GLOB NVIDIA_CUBLAS_INCLUDE "${CUOPT_SITE_PACKAGES}/nvidia/cublas/include")
file(GLOB NVIDIA_CUSPARSE_INCLUDE "${CUOPT_SITE_PACKAGES}/nvidia/cusparse/include")
file(GLOB NVIDIA_CUSOLVER_INCLUDE "${CUOPT_SITE_PACKAGES}/nvidia/cusolver/include")
file(GLOB NVIDIA_CURAND_INCLUDE "${CUOPT_SITE_PACKAGES}/nvidia/curand/include")
list(APPEND NVIDIA_CUDA_INCLUDE_DIRS
  ${NVIDIA_CUDA_PACKAGES}
  ${NVIDIA_CUBLAS_INCLUDE}
  ${NVIDIA_CUSPARSE_INCLUDE}
  ${NVIDIA_CUSOLVER_INCLUDE}
  ${NVIDIA_CURAND_INCLUDE}
)

# ===== Traffic control library
file(GLOB_RECURSE core_lib_srcs "src/rmf_traffic/*.cpp")
add_library(rmf_traffic SHARED
  ${core_lib_srcs}
)

# Unit tests removed with traffic negotiation (see 92a42c1); test/ sources are not built.

if(using_new_fcl)
  target_compile_definitions(rmf_traffic PRIVATE RMF_TRAFFIC__USING_FCL_0_6)
endif()

target_link_libraries(rmf_traffic
  PUBLIC
    rmf_utils::rmf_utils
    Threads::Threads
  PRIVATE
    ${FCL_LIBRARIES}
    yaml-cpp
)

target_include_directories(rmf_traffic
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    ${EIGEN3_INCLUDE_DIRS}
)

target_include_directories(rmf_traffic PRIVATE
  ${CUOPT_INCLUDE_DIR}
  ${RMM_INCLUDE_DIR}
  ${RAFT_INCLUDE_DIR}
  ${RAPIDS_LOGGER_INCLUDE_DIR}
  ${CUDA_INCLUDE_DIR}
  ${CUDA_NVCC_INCLUDE_DIR}
  ${NVIDIA_CUDA_INCLUDE_DIRS}
)

target_link_directories(rmf_traffic PRIVATE
  ${CUOPT_LIB_DIR}
  ${RMM_LIB_DIR}
  ${CUDA_RUNTIME_LIB_DIR}
)

target_link_libraries(rmf_traffic PRIVATE
  ${CUOPT_LIB}
  ${RMM_LIB}
  ${CUDART_LIB}
  pthread
  dl
)

set_target_properties(rmf_traffic PROPERTIES
  BUILD_RPATH "${CUOPT_LIB_DIR};${RMM_LIB_DIR};${CUDA_RUNTIME_LIB_DIR}"
  INSTALL_RPATH "${CUOPT_LIB_DIR};${RMM_LIB_DIR};${CUDA_RUNTIME_LIB_DIR}"
  BUILD_RPATH_USE_LAUNCHER TRUE
  INSTALL_RPATH_USE_LINK_PATH TRUE
)

set(CUOPT_RUNTIME_LIB_PATH "${CUOPT_LIB_DIR}:${RMM_LIB_DIR}:${CUDA_RUNTIME_LIB_DIR}")
target_compile_definitions(rmf_traffic PRIVATE
  LIBCUDACXX_ENABLE_EXPERIMENTAL_MEMORY_RESOURCE
  CUOPT_RUNTIME_LIB_PATH="${CUOPT_RUNTIME_LIB_PATH}"
)

message(STATUS "cuOpt libraries linked to rmf_traffic")
message(STATUS "cuOpt runtime library path: ${CUOPT_RUNTIME_LIB_PATH}")

# Create cmake config files
include(CMakePackageConfigHelpers)

set(INSTALL_CONFIG_DIR "${CMAKE_INSTALL_LIBDIR}/rmf_traffic/cmake")
set(PACKAGE_CONFIG_VERSION_FILE "${CMAKE_CURRENT_BINARY_DIR}/rmf_traffic-config-version.cmake")
set(PACKAGE_CONFIG_FILE "${CMAKE_CURRENT_BINARY_DIR}/rmf_traffic-config.cmake")

configure_package_config_file(
  "${CMAKE_CURRENT_LIST_DIR}/cmake/rmf_traffic-config.cmake.in"
  ${PACKAGE_CONFIG_FILE}
  INSTALL_DESTINATION ${INSTALL_CONFIG_DIR}
)

write_basic_package_version_file(
  ${PACKAGE_CONFIG_VERSION_FILE}
  COMPATIBILITY SameMajorVersion
)

install(
  TARGETS rmf_traffic
  EXPORT  rmf_traffic-targets
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

install(
  DIRECTORY include/rmf_traffic
  DESTINATION include
)


install(
  FILES
    ${PACKAGE_CONFIG_VERSION_FILE}
    ${PACKAGE_CONFIG_FILE}
  DESTINATION ${INSTALL_CONFIG_DIR}
)

install(
  EXPORT rmf_traffic-targets
  FILE rmf_traffic-targets.cmake
  NAMESPACE rmf_traffic::
  DESTINATION ${INSTALL_CONFIG_DIR}
)

export(
  EXPORT rmf_traffic-targets
  FILE ${CMAKE_CURRENT_BINARY_DIR}/rmf_traffic-targets.cmake
  NAMESPACE rmf_traffic::
)

# Export ament index path
file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/share/ament_index/resource_index/packages/${PROJECT_NAME} "")
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/share/ament_index/resource_index/packages/${PROJECT_NAME} DESTINATION share/ament_index/resource_index/packages)

file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/share/${PROJECT_NAME}/hook/ament_prefix_path.dsv "prepend-non-duplicate;AMENT_PREFIX_PATH;")
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/share/${PROJECT_NAME}/hook/ament_prefix_path.dsv DESTINATION share/${PROJECT_NAME}/hook)
