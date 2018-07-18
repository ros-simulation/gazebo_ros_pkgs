cmake_minimum_required(VERSION 3.5)
project(gazebo_ros)


# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # we dont use add_compile_options with pedantic in message packages
  # because the Python C extensions dont comply with it
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_dev REQUIRED)

include_directories(include)

add_library(gazebo_ros_node SHARED src/node.cpp src/executor.cpp)
ament_target_dependencies(gazebo_ros_node rclcpp gazebo_dev)

ament_export_include_directories(include)
ament_export_libraries(gazebo_ros_node)
ament_export_dependencies(rclcpp)
ament_export_dependencies(gazebo_dev)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

install(DIRECTORY include/
        DESTINATION include)

install(TARGETS gazebo_ros_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

