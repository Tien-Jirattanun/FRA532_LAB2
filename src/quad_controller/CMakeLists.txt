cmake_minimum_required(VERSION 3.8)
project(quad_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Find Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(actuator_msgs REQUIRED)

# 2. Define Include Directories
include_directories(
  include
  ${Eigen3_INCLUDE_DIRS}
)

# 3. Create Executable for PID Controller
add_executable(pid_controller_node 
  src/pid_controller.cpp
  src/PID.cpp
)
ament_target_dependencies(pid_controller_node
  rclcpp nav_msgs geometry_msgs std_msgs tf2 actuator_msgs
)
target_link_libraries(pid_controller_node Eigen3::Eigen)

# 4. Create Executable for LQR Controller
add_executable(lqr_controller_node 
  src/lqr_controller.cpp
)
ament_target_dependencies(lqr_controller_node
  rclcpp nav_msgs geometry_msgs std_msgs tf2 actuator_msgs
)
target_link_libraries(lqr_controller_node Eigen3::Eigen)

# 5. Install
install(TARGETS
  pid_controller_node
  lqr_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()