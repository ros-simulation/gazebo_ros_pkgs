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
find_package(geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(include
  ${gazebo_dev_INCLUDE_DIRS}
  ${gazebo_ros_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${image_transport_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
)
link_directories(${gazebo_dev_LIBRARY_DIRS})

# gazebo_ros_joint_state_publisher
add_library(gazebo_ros_joint_state_publisher SHARED
  src/gazebo_ros_joint_state_publisher.cpp
)
ament_target_dependencies(gazebo_ros_joint_state_publisher
  "gazebo_dev"
  "gazebo_ros"
  "rclcpp"
  "sensor_msgs"
)
ament_export_libraries(gazebo_ros_joint_state_publisher)

# gazebo_ros_diff_drive
add_library(gazebo_ros_diff_drive SHARED
  src/gazebo_ros_diff_drive.cpp
)
ament_target_dependencies(gazebo_ros_diff_drive
  "gazebo_dev"
  "gazebo_ros"
  "geometry_msgs"
  "nav_msgs"
  "rclcpp"
  "tf2"
  "tf2_geometry_msgs"
  "tf2_ros"
)
ament_export_libraries(gazebo_ros_diff_drive)

# gazebo_ros_force
add_library(gazebo_ros_force SHARED
  src/gazebo_ros_force.cpp
)
ament_target_dependencies(gazebo_ros_force
  "gazebo_dev"
  "gazebo_ros"
  "geometry_msgs"
  "rclcpp"
)
ament_export_libraries(gazebo_ros_force)

# gazebo_ros_template
add_library(gazebo_ros_template SHARED
  src/gazebo_ros_template.cpp
)
ament_target_dependencies(gazebo_ros_template
  "gazebo_dev"
  "gazebo_ros"
  "rclcpp"
)
ament_export_libraries(gazebo_ros_template)

# gazebo_ros_camera
add_library(gazebo_ros_camera SHARED
  src/gazebo_ros_camera.cpp
)
ament_target_dependencies(gazebo_ros_camera
  "gazebo_dev"
  "gazebo_ros"
  "image_transport"
  "rclcpp"
  "sensor_msgs"
  "std_msgs"
)
target_link_libraries(gazebo_ros_camera
  CameraPlugin
)
ament_export_libraries(gazebo_ros_camera)

# gazebo_ros_imu_sensor
add_library(gazebo_ros_imu_sensor SHARED
  src/gazebo_ros_imu_sensor.cpp
)
ament_target_dependencies(gazebo_ros_imu_sensor
  "gazebo_ros"
  "sensor_msgs"
  "gazebo_dev"
)
ament_export_libraries(gazebo_ros_imu_sensor)

# gazebo_ros_ray_sensor
add_library(gazebo_ros_ray_sensor SHARED
  src/gazebo_ros_ray_sensor.cpp
)
ament_target_dependencies(gazebo_ros_ray_sensor
  "gazebo_ros"
  "sensor_msgs"
)
ament_export_libraries(gazebo_ros_ray_sensor)

ament_export_include_directories(include)
ament_export_dependencies(rclcpp)
ament_export_dependencies(gazebo_dev)
ament_export_dependencies(gazebo_ros)

if(BUILD_TESTING)
  add_subdirectory(test)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

install(DIRECTORY include/
        DESTINATION include)

install(TARGETS
    gazebo_ros_camera
    gazebo_ros_diff_drive
    gazebo_ros_force
    gazebo_ros_imu_sensor
    gazebo_ros_joint_state_publisher
    gazebo_ros_ray_sensor
    gazebo_ros_template
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY
  worlds
  DESTINATION share/${PROJECT_NAME}/
)
