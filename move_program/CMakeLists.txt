cmake_minimum_required(VERSION 3.8)
project(move_program)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_visual_tools REQUIRED)

add_executable(move_program src/move_program.cpp)
ament_target_dependencies(move_program rclcpp moveit_ros_planning_interface moveit_visual_tools)
# Install the executable
install(TARGETS
  move_program
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
