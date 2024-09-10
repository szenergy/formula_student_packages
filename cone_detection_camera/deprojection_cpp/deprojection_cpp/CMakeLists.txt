cmake_minimum_required(VERSION 3.5)
project(deprojection_cpp)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)  # Add this line

add_executable(deprojection src/deprojection.cpp)
ament_target_dependencies(deprojection rclcpp std_msgs visualization_msgs)  # Add visualization_msgs here

install(TARGETS
  deprojection
  DESTINATION lib/${PROJECT_NAME})

ament_package()
