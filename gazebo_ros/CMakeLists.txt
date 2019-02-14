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
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tinyxml_vendor REQUIRED)

include_directories(
  include
  ${gazebo_dev_INCLUDE_DIRS}
)

# gazebo_ros_node
add_library(gazebo_ros_node SHARED
  src/executor.cpp
  src/node.cpp
)
ament_target_dependencies(gazebo_ros_node
  "gazebo_dev"
  "rclcpp"
)
ament_export_libraries(gazebo_ros_node)

# gazebo_ros_utils
add_library(gazebo_ros_utils SHARED
  src/utils.cpp
)
ament_target_dependencies(gazebo_ros_utils
  "gazebo_dev"
  "rclcpp"
)
ament_export_libraries(gazebo_ros_utils)

# gazebo_ros_init
add_library(gazebo_ros_init SHARED
  src/gazebo_ros_init.cpp
)
ament_target_dependencies(gazebo_ros_init
  "builtin_interfaces"
  "gazebo_dev"
  "rclcpp"
)
target_link_libraries(gazebo_ros_init
  gazebo_ros_node
  gazebo_ros_utils
)
ament_export_libraries(gazebo_ros_init)

# gazebo_ros_factory
add_library(gazebo_ros_factory SHARED
  src/gazebo_ros_factory.cpp
)
ament_target_dependencies(gazebo_ros_factory
  "rclcpp"
  "gazebo_dev"
  "gazebo_msgs"
)
target_link_libraries(gazebo_ros_factory
  gazebo_ros_node
  gazebo_ros_utils
)
ament_export_libraries(gazebo_ros_factory)

# gazebo_ros_state
add_library(gazebo_ros_state SHARED
  src/gazebo_ros_state.cpp
)
ament_target_dependencies(gazebo_ros_state
  "rclcpp"
  "gazebo_dev"
  "gazebo_msgs"
)
target_link_libraries(gazebo_ros_state
  gazebo_ros_node
)
ament_export_libraries(gazebo_ros_state)

ament_export_include_directories(include)
ament_export_dependencies(rclcpp)
ament_export_dependencies(gazebo_dev)

if(BUILD_TESTING)
  add_subdirectory(test)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

install(DIRECTORY include/
        DESTINATION include)

install(
  TARGETS
    gazebo_ros_factory
    gazebo_ros_init
    gazebo_ros_node
    gazebo_ros_state
    gazebo_ros_utils
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
