cmake_minimum_required(VERSION 3.5)
set(PLUGIN_NAME calib_ekf)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# set Release as default
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# find dependencies
set(PLUGIN_DEPENDENCIES
  ament_cmake
  rclcpp
  pluginlib
  as2_core
  nav_msgs
  geometry_msgs
  tf2
  mocap4r2_msgs
  tf2_ros
  vision_msgs
  eigen3_cmake_module
)

foreach(DEPENDENCY ${PLUGIN_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

find_package(Eigen3 REQUIRED NO_MODULE)

# The ekf_calib math sources are compiled directly into the plugin library rather
# than into their own target: a separate target named `ekf` would collide with the
# one simple_ekf already defines, and the generated CasADi entry points are
# extern "C" globals that must not be duplicated in the same process.
set(SOURCE_CPP_FILES
  src/${PLUGIN_NAME}.cpp
  lib/ekf_calib/src/calib_ekf_c_code.cpp
  lib/ekf_calib/src/ekf_datatype.cpp
  lib/ekf_calib/src/ekf_wrapper.cpp
)

# Library
add_library(${PLUGIN_NAME} SHARED ${SOURCE_CPP_FILES})
ament_target_dependencies(${PLUGIN_NAME} ${PLUGIN_DEPENDENCIES})
target_include_directories(${PLUGIN_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>        # Plugin headers
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/lib/ekf_calib/include>  # EKF library headers
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../include>  # Package headers
  $<INSTALL_INTERFACE:include>
  $<TARGET_PROPERTY:Eigen3::Eigen,INTERFACE_INCLUDE_DIRECTORIES>
)
target_link_libraries(${PLUGIN_NAME}
  ${PROJECT_NAME}
  ${PROJECT_NAME}_plugin_base
  Eigen3::Eigen
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PLUGIN_NAME}
)
ament_export_dependencies(
  Eigen3
)
ament_export_targets(
  export_${PLUGIN_NAME}
)

install(
  TARGETS ${PLUGIN_NAME}
  EXPORT export_${PLUGIN_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  add_subdirectory(tests)
endif()
