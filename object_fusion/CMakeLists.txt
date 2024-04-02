cmake_minimum_required(VERSION 3.8)
project(prcp_object_fusion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

set(ament_dependencies
  rclcpp
  geometry_msgs
  std_msgs
  visualization_msgs
  tf2_geometry_msgs
)

add_executable(prcp_object_fusion src/object_fusion_runnable.cpp)

ament_target_dependencies(prcp_object_fusion  ${ament_dependencies})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(TARGETS prcp_object_fusion
  DESTINATION lib/${PROJECT_NAME})

ament_package()
