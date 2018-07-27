cmake_minimum_required(VERSION 3.5)
project(gazebo_plugins)

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
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(include
  ${gazebo_ros_INCLUDE_DIRS}
)

# gazebo_ros_template
add_library(gazebo_ros_template SHARED
  src/gazebo_ros_template.cpp
)
target_link_libraries(gazebo_ros_template
  ${gazebo_ros_LIBRARIES}
)
ament_target_dependencies(gazebo_ros_template
  "rclcpp"
  "gazebo_dev"
  "gazebo_ros"
)
ament_export_libraries(gazebo_ros_template)

ament_export_include_directories(include)
ament_export_dependencies(rclcpp)
ament_export_dependencies(gazebo_dev)
ament_export_dependencies(gazebo_ros)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

install(DIRECTORY include/
        DESTINATION include)

install(TARGETS gazebo_ros_template
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
