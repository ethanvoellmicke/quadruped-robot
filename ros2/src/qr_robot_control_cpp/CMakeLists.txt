cmake_minimum_required(VERSION 3.8)

project(qr_robot_control_cpp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Enable useful compiler warnings.
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
  )
endif()

# -------------------------
# Find ROS 2 dependencies
# -------------------------

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(qr_robot_msgs REQUIRED)

# -------------------------
# Wi-Fi bridge node
# -------------------------

add_executable(
  wifi_bridge
  src/wifi_bridge.cpp
)

ament_target_dependencies(
  wifi_bridge
  rclcpp
)

target_link_libraries(
  wifi_bridge
  ${qr_robot_msgs_TARGETS}
)

# -------------------------
# Gait controller node
# -------------------------

add_executable(
  gait_controller
  src/gait_controller.cpp
  src/foot_trajectory.cpp
  src/leg_ik.cpp
)

target_include_directories(
  gait_controller
  PUBLIC
    include
)

ament_target_dependencies(
  gait_controller
  rclcpp
  sensor_msgs
  std_msgs
)

target_link_libraries(
  gait_controller
  ${qr_robot_msgs_TARGETS}
)

# -------------------------
# Install executables
# -------------------------

install(
  TARGETS
    wifi_bridge
    gait_controller
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(
  DIRECTORY include/
  DESTINATION include
)

# -------------------------
# Testing
# -------------------------

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()