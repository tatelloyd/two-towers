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

# Optional: BehaviorTree.CPP (comment out if not using BT version)
find_package(behaviortree_cpp QUIET)

# Generate custom messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Detection.msg"
  "msg/DetectionArray.msg"
  "msg/TurretState.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

# Build Turret library (servo control)
add_library(turret_control
  src/cpp/ServoController.cpp
  src/cpp/Turret.cpp
)

target_include_directories(turret_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/cpp>
  $<INSTALL_INTERFACE:include>
)

# Link pigpiod library
target_link_libraries(turret_control
  pigpiod_if2
)

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
# C++ executable: BehaviorTree tracker node (optional)
# ============================================================================
if(behaviortree_cpp_FOUND)
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
  )
  
  target_link_libraries(orthanc_tracker_bt_node "${cpp_typesupport_target}")
  
  message(STATUS "✅ BehaviorTree.CPP found - building BT tracker node")
else()
  message(STATUS "⚠️  BehaviorTree.CPP not found - skipping BT tracker node")
endif()

# ============================================================================
# Install C++ executables
# ============================================================================

install(TARGETS orthanc_tracker_node
  DESTINATION lib/${PROJECT_NAME}
)

if(behaviortree_cpp_FOUND)
  install(TARGETS orthanc_tracker_bt_node
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

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

# Export dependencies
ament_export_dependencies(rosidl_default_runtime)
ament_export_libraries(turret_control)
ament_export_include_directories(include)

ament_package()