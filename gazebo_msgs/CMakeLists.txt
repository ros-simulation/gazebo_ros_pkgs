cmake_minimum_required(VERSION 3.5)

project(gazebo_msgs)

if(NOT DEFINED IGNORE_GAZEBO_MSGS_WARNING)
  if(DEFINED ENV{ROS_DISTRO})
    set(ROS_DISTRO $ENV{ROS_DISTRO})
    if(NOT (ROS_DISTRO STREQUAL "iron" OR
            ROS_DISTRO STREQUAL "humble"))
      message(WARNING "This gazebo_msgs package hosts messages designed initially for Gazebo Classic "
        "which is not available on ROS 2 Jazzy which is unavailable on ROS 2 Jazzy. "
        "The new Gazebo uses the ros_gz bridge message vailable at:"
        "https://github.com/gazebosim/ros_gz/tree/ros2/ros_gz_bridge#bridge-communication-between-ros-and-gazebo\n"
        "To avoid this warning and use this Gazebo Classic messages anyway you can use flag -DIGNORE_GAZEBO_MSGS_WARNING")
    endif()
  endif()
endif()

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
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/ContactState.msg"
  "msg/ContactsState.msg"
  "msg/EntityState.msg"
  "msg/LinkState.msg"
  "msg/LinkStates.msg"
  "msg/ModelState.msg"
  "msg/ModelStates.msg"
  "msg/ODEJointProperties.msg"
  "msg/ODEPhysics.msg"
  "msg/PerformanceMetrics.msg"
  "msg/SensorPerformanceMetric.msg"
  "msg/WheelSlip.msg"
  "msg/WorldState.msg"
)

set(srv_files
  "srv/ApplyBodyWrench.srv"
  "srv/ApplyLinkWrench.srv"
  "srv/ApplyJointEffort.srv"
  "srv/BodyRequest.srv"
  "srv/DeleteEntity.srv"
  "srv/DeleteLight.srv"
  "srv/DeleteModel.srv"
  "srv/GetJointProperties.srv"
  "srv/GetEntityState.srv"
  "srv/GetLightProperties.srv"
  "srv/GetLinkProperties.srv"
  "srv/GetLinkState.srv"
  "srv/GetModelList.srv"
  "srv/GetModelProperties.srv"
  "srv/GetModelState.srv"
  "srv/GetPhysicsProperties.srv"
  "srv/GetWorldProperties.srv"
  "srv/JointRequest.srv"
  "srv/LinkRequest.srv"
  "srv/SetEntityState.srv"
  "srv/SetJointProperties.srv"
  "srv/SetJointTrajectory.srv"
  "srv/SetLightProperties.srv"
  "srv/SetLinkProperties.srv"
  "srv/SetLinkState.srv"
  "srv/SetModelConfiguration.srv"
  "srv/SetModelState.srv"
  "srv/SetPhysicsProperties.srv"
  "srv/SpawnEntity.srv"
  "srv/SpawnModel.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}

  DEPENDENCIES
  builtin_interfaces
  std_msgs
  geometry_msgs
  trajectory_msgs

  ADD_LINTER_TESTS
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()

install(
  FILES gazebo_msgs_mapping_rules.yaml
  DESTINATION share/${PROJECT_NAME})
