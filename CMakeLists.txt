cmake_minimum_required(VERSION 3.8)
project(two_towers)

# C++17 standard
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# BehaviorTree.CPP is REQUIRED: the behavior tree node is the only tracker.
# Previously this was QUIET, which let the build silently succeed without it.
find_package(behaviortree_cpp REQUIRED)

# Generate custom messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Detection.msg"
  "msg/DetectionArray.msg"
  "msg/TowerStatus.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

# Build without a Raspberry Pi attached (CI, laptops, unit tests).
# See src/cpp/sim/pigpiod_if2.h for what this substitutes.
option(TWO_TOWERS_SIM_GPIO "Stub out pigpio instead of linking the real library" OFF)

# Build Turret library (servo control)
add_library(turret_control
  src/cpp/ServoController.cpp
  src/cpp/Turret.cpp
)

target_include_directories(turret_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/cpp>
  $<INSTALL_INTERFACE:include>
)

if(TWO_TOWERS_SIM_GPIO)
  # Put the stub directory ahead of the system includes so that the unmodified
  # `#include <pigpiod_if2.h>` in Turret.cpp resolves to sim/ instead. No source
  # changes and no preprocessor branches in the control code itself.
  target_include_directories(turret_control BEFORE PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/cpp/sim>
  )
  message(STATUS "TWO_TOWERS_SIM_GPIO=ON - pigpio stubbed, servos will not move")
else()
  target_link_libraries(turret_control pigpiod_if2)
endif()

# Install the shared library to lib directory
install(TARGETS turret_control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# ============================================================================
# C++ executable: Simple tracker node (no BehaviorTree)
# ============================================================================
add_executable(orthanc_tracker_node src/cpp/orthanc_tracker_node.cpp)

target_include_directories(orthanc_tracker_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/cpp>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(orthanc_tracker_node 
  rclcpp 
  std_msgs 
)

# Link against turret control library
target_link_libraries(orthanc_tracker_node
  turret_control
)

# Link generated messages to C++ node
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(orthanc_tracker_node "${cpp_typesupport_target}")

# ============================================================================
# C++ executable: BehaviorTree tracker node
# ============================================================================
add_executable(orthanc_tracker_bt_node src/cpp/orthanc_tracker_node_bt.cpp)

target_include_directories(orthanc_tracker_bt_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/cpp>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(orthanc_tracker_bt_node
  rclcpp
  std_msgs
  behaviortree_cpp
)

target_link_libraries(orthanc_tracker_bt_node
  turret_control
  "${cpp_typesupport_target}"
)

# ============================================================================
# C++ executable: servo bringup tool (interactive, no ROS2)
# ============================================================================
# Manual pan/tilt jog utility. Used to verify wiring and find the mechanical
# limits of a new turret before running the tracker against it.
#
# SignalGenerator (sine/triangle sweeps) is only used by this tool, so it is
# compiled into the executable rather than into turret_control.
add_executable(servo_bringup
  tests/cpp/menu.cpp
  src/cpp/SignalGenerator.cpp
)

target_include_directories(servo_bringup PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/cpp>
)

target_link_libraries(servo_bringup turret_control)

# ============================================================================
# Install C++ executables
# ============================================================================

install(TARGETS
  orthanc_tracker_node
  orthanc_tracker_bt_node
  servo_bringup
  DESTINATION lib/${PROJECT_NAME}
)

# ============================================================================
# Python Setup - Install scripts directly (simpler approach)
# ============================================================================
# Install Python scripts (make them executable)
install(PROGRAMS
  src/python/two_towers_detector_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files (if needed by other packages)
install(DIRECTORY src/cpp/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.hpp"
)

# ============================================================================
# Launch files and per-tower parameters
# ============================================================================
# These must land in share/, not lib/: FindPackageShare in the launch files
# resolves against the share directory.
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# Export dependencies
ament_export_dependencies(rosidl_default_runtime)
ament_export_libraries(turret_control)
ament_export_include_directories(include)

ament_package()