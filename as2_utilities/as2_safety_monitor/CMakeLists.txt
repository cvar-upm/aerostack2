cmake_minimum_required(VERSION 3.5)
set(PROJECT_NAME as2_safety_monitor)
project(${PROJECT_NAME} VERSION 1.1.3)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# set Release as default
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# find dependencies
set(PROJECT_DEPENDENCIES
  ament_cmake
  rclcpp
  as2_core
  as2_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  yaml-cpp
)

foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

# Include necessary directories
include_directories(
  include
)

# Set source files
set(SOURCE_CPP_FILES
  src/safety_check.cpp
  src/battery_check.cpp
  src/geocage_check.cpp
  src/as2_safety_monitor.cpp
)

# Create the library, shared by the node and the tests
add_library(${PROJECT_NAME} STATIC ${SOURCE_CPP_FILES})
ament_target_dependencies(${PROJECT_NAME} ${PROJECT_DEPENDENCIES})
target_link_libraries(${PROJECT_NAME} yaml-cpp::yaml-cpp)

# Create the node executable
add_executable(${PROJECT_NAME}_node src/as2_safety_monitor_node.cpp)
ament_target_dependencies(${PROJECT_NAME}_node ${PROJECT_DEPENDENCIES})
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})

# Install the config and launch directory
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME})

# Install the node executable
install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME})

# Build tests if testing is enabled
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  file(GLOB_RECURSE EXCLUDE_FILES
    include/${PROJECT_NAME}/pnpoly.hpp
    build/*
    install/*
  )
  set(AMENT_LINT_AUTO_FILE_EXCLUDE ${EXCLUDE_FILES})
  ament_lint_auto_find_test_dependencies()

  # The tests load the installed default config
  find_package(ament_index_cpp REQUIRED)
  list(APPEND PROJECT_DEPENDENCIES ament_index_cpp)
  add_subdirectory(tests)
endif()

ament_package()
