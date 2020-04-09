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

if(WIN32)
  add_compile_definitions(
    # For math constants
    _USE_MATH_DEFINES
    # Minimize Windows namespace collision
    NOMINMAX
    WIN32_LEAN_AND_MEAN
  )
endif()

find_package(ament_cmake REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(trajectory_msgs REQUIRED)

include_directories(include
  ${gazebo_dev_INCLUDE_DIRS}
  ${gazebo_ros_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${image_transport_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
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

# multi_camera_plugin
add_library(multi_camera_plugin SHARED
  src/multi_camera_plugin.cpp
)
ament_target_dependencies(multi_camera_plugin
  "gazebo_dev"
)
target_link_libraries(multi_camera_plugin
  gazebo_sensors
)
target_compile_definitions(multi_camera_plugin
  PRIVATE BUILDING_DLL
)

# gazebo_ros_camera
add_library(gazebo_ros_camera SHARED
  src/gazebo_ros_camera.cpp
)
ament_target_dependencies(gazebo_ros_camera
  "camera_info_manager"
  "gazebo_dev"
  "gazebo_ros"
  "image_transport"
  "rclcpp"
  "sensor_msgs"
  "std_msgs"
)
target_link_libraries(gazebo_ros_camera
  CameraPlugin
  DepthCameraPlugin
  multi_camera_plugin
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
target_link_libraries(gazebo_ros_imu_sensor
  ImuSensorPlugin
)
ament_export_libraries(gazebo_ros_imu_sensor)


# gazebo_ros_gps_sensor
add_library(gazebo_ros_gps_sensor SHARED
  src/gazebo_ros_gps_sensor.cpp
)
ament_target_dependencies(gazebo_ros_gps_sensor
  "gazebo_ros"
  "sensor_msgs"
  "gazebo_dev"
)
ament_export_libraries(gazebo_ros_gps_sensor)

# gazebo_ros_ray_sensor
add_library(gazebo_ros_ray_sensor SHARED
  src/gazebo_ros_ray_sensor.cpp
)
ament_target_dependencies(gazebo_ros_ray_sensor
  "gazebo_ros"
  "sensor_msgs"
)
ament_export_libraries(gazebo_ros_ray_sensor)

add_library(gazebo_ros_p3d SHARED
  src/gazebo_ros_p3d.cpp
)
ament_target_dependencies(gazebo_ros_p3d
  "gazebo_ros"
  "gazebo_dev"
  "geometry_msgs"
  "nav_msgs"
  "rclcpp"
  "tf2"
  "tf2_geometry_msgs"
  "tf2_ros"
)
ament_export_libraries(gazebo_ros_p3d)

# gazebo_ros_tricycle_drive
add_library(gazebo_ros_tricycle_drive SHARED
  src/gazebo_ros_tricycle_drive.cpp
)
ament_target_dependencies(gazebo_ros_tricycle_drive
  "gazebo_ros"
  "geometry_msgs"
  "nav_msgs"
  "rclcpp"
  "sensor_msgs"
  "tf2"
  "tf2_geometry_msgs"
  "tf2_ros"
)
ament_export_libraries(gazebo_ros_tricycle_drive)

# gazebo_ros_video
add_library(gazebo_ros_video SHARED
  src/gazebo_ros_video.cpp
)
ament_target_dependencies(gazebo_ros_video
  "gazebo_ros"
  "sensor_msgs"
  "cv_bridge"
)
ament_export_libraries(gazebo_ros_video)

# gazebo_ros_ft_sensor
add_library(gazebo_ros_ft_sensor SHARED
  src/gazebo_ros_ft_sensor.cpp
)
ament_target_dependencies(gazebo_ros_ft_sensor
  "gazebo_ros"
  "gazebo_dev"
  "rclcpp"
  "geometry_msgs"
)
ament_export_libraries(gazebo_ros_ft_sensor)

# gazebo_ros_bumper
add_library(gazebo_ros_bumper SHARED
  src/gazebo_ros_bumper.cpp
)
target_link_libraries(gazebo_ros_bumper
  ContactPlugin
)
ament_target_dependencies(gazebo_ros_bumper
  "gazebo_ros"
  "gazebo_msgs"
  "geometry_msgs"
)
ament_export_libraries(gazebo_ros_bumper)

# gazebo_ros_harness
add_library(gazebo_ros_harness SHARED
  src/gazebo_ros_harness.cpp
)
ament_target_dependencies(gazebo_ros_harness
  "gazebo_dev"
  "gazebo_ros"
  "rclcpp"
  "std_msgs"
)
target_link_libraries(gazebo_ros_harness
  HarnessPlugin
)
ament_export_libraries(gazebo_ros_harness)

# gazebo_ros_hand_of_god
add_library(gazebo_ros_hand_of_god SHARED
  src/gazebo_ros_hand_of_god.cpp
)
ament_target_dependencies(gazebo_ros_hand_of_god
  "gazebo_ros"
  "gazebo_dev"
  "rclcpp"
  "tf2"
  "tf2_geometry_msgs"
  "tf2_ros"
)
ament_export_libraries(gazebo_ros_hand_of_god)

# gazebo_ros_ackermann_drive
add_library(gazebo_ros_ackermann_drive SHARED
  src/gazebo_ros_ackermann_drive.cpp
)
ament_target_dependencies(gazebo_ros_ackermann_drive
  "gazebo_dev"
  "gazebo_ros"
  "geometry_msgs"
  "nav_msgs"
  "rclcpp"
  "tf2"
  "tf2_geometry_msgs"
  "tf2_ros"
)
ament_export_libraries(gazebo_ros_ackermann_drive)

# gazebo_ros_elevator
add_library(gazebo_ros_elevator SHARED
  src/gazebo_ros_elevator.cpp
)
ament_target_dependencies(gazebo_ros_elevator
  "gazebo_ros"
  "gazebo_dev"
  "rclcpp"
  "std_msgs"
)
target_link_libraries(gazebo_ros_elevator
  ElevatorPlugin
)
ament_export_libraries(gazebo_ros_elevator)

# gazebo_ros_vacuum_gripper
add_library(gazebo_ros_vacuum_gripper SHARED
  src/gazebo_ros_vacuum_gripper.cpp
)
ament_target_dependencies(gazebo_ros_vacuum_gripper
  "gazebo_ros"
  "gazebo_dev"
  "rclcpp"
  "std_msgs"
  "std_srvs"
)
ament_export_libraries(gazebo_ros_vacuum_gripper)

# gazebo_ros_joint_pose_trajectory
add_library(gazebo_ros_joint_pose_trajectory SHARED
  src/gazebo_ros_joint_pose_trajectory.cpp
)
ament_target_dependencies(gazebo_ros_joint_pose_trajectory
  "gazebo_dev"
  "gazebo_ros"
  "rclcpp"
  "trajectory_msgs"
)
ament_export_libraries(gazebo_ros_joint_pose_trajectory)

# gazebo_ros_planar_move
add_library(gazebo_ros_planar_move SHARED
  src/gazebo_ros_planar_move.cpp
)
ament_target_dependencies(gazebo_ros_planar_move
  "gazebo_dev"
  "gazebo_ros"
  "nav_msgs"
  "rclcpp"
  "tf2"
  "tf2_geometry_msgs"
  "tf2_ros"
)
ament_export_libraries(gazebo_ros_planar_move)

# gazebo_ros_projector
add_library(gazebo_ros_projector SHARED
  src/gazebo_ros_projector.cpp
)
ament_target_dependencies(gazebo_ros_projector
  "gazebo_ros"
  "gazebo_dev"
  "rclcpp"
  "std_msgs"
)
ament_export_libraries(gazebo_ros_projector)

ament_export_include_directories(include)
ament_export_dependencies(rclcpp)
ament_export_dependencies(gazebo_dev)
ament_export_dependencies(gazebo_msgs)
ament_export_dependencies(gazebo_ros)

if(BUILD_TESTING)
  add_subdirectory(test)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

if(NOT WIN32)
  if(NOT APPLE)
    set(
      AMENT_CMAKE_ENVIRONMENT_HOOKS_DESC_gazebo_plugins
      "prepend-non-duplicate;LD_LIBRARY_PATH;${GAZEBO_PLUGIN_PATH}")
  else()
    set(
      AMENT_CMAKE_ENVIRONMENT_HOOKS_DESC_gazebo_plugins
      "prepend-non-duplicate;DYLD_LIBRARY_PATH;${GAZEBO_PLUGIN_PATH}")
  endif()
endif()
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/gazebo_plugins.sh.in")

ament_package()

install(DIRECTORY include/
        DESTINATION include)

install(TARGETS
    gazebo_ros_ackermann_drive
    gazebo_ros_bumper
    gazebo_ros_camera
    gazebo_ros_diff_drive
    gazebo_ros_elevator
    gazebo_ros_force
    gazebo_ros_ft_sensor
    gazebo_ros_hand_of_god
    gazebo_ros_harness
    gazebo_ros_imu_sensor
    gazebo_ros_gps_sensor
    gazebo_ros_joint_pose_trajectory
    gazebo_ros_joint_state_publisher
    gazebo_ros_planar_move
    gazebo_ros_projector
    gazebo_ros_ray_sensor
    gazebo_ros_p3d
    gazebo_ros_template
    gazebo_ros_tricycle_drive
    gazebo_ros_vacuum_gripper
    gazebo_ros_video
    multi_camera_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY
  worlds
  DESTINATION share/${PROJECT_NAME}/
)
