cmake_minimum_required(VERSION 3.8)
project(planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

set(ament_dependencies
  rclcpp
  geometry_msgs
  std_msgs
  visualization_msgs
)

add_executable(plan_trajectory_planner src/local_planner.cpp)

ament_target_dependencies(plan_trajectory_planner  ${ament_dependencies})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(TARGETS plan_trajectory_planner
  DESTINATION lib/${PROJECT_NAME})

ament_package()
