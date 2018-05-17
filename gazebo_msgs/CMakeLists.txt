cmake_minimum_required(VERSION 3.5)

project(gazebo_msgs)

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
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/ContactsState.msg"
  "msg/ContactState.msg"
  "msg/LinkState.msg"
  "msg/LinkStates.msg"
  "msg/ModelState.msg"
  "msg/ModelStates.msg"
  "msg/ODEJointProperties.msg"
  "msg/ODEPhysics.msg"
  "msg/WorldState.msg"
)

set(srv_files
  "srv/ApplyBodyWrench.srv"
  "srv/DeleteModel.srv"
  "srv/DeleteLight.srv"
  "srv/GetLinkState.srv"
  "srv/GetPhysicsProperties.srv"
  "srv/SetJointProperties.srv"
  "srv/SetModelConfiguration.srv"
  "srv/SpawnModel.srv"
  "srv/ApplyJointEffort.srv"
  "srv/GetJointProperties.srv"
  "srv/GetModelProperties.srv"
  "srv/GetWorldProperties.srv"
  "srv/SetLinkProperties.srv"
  "srv/SetModelState.srv"
  "srv/BodyRequest.srv"
  "srv/GetLinkProperties.srv"
  "srv/GetModelState.srv"
  "srv/JointRequest.srv"
  "srv/SetLinkState.srv"
  "srv/SetPhysicsProperties.srv"
  "srv/SetJointTrajectory.srv"
  "srv/GetLightProperties.srv"
  "srv/SetLightProperties.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  
  DEPENDENCIES
  builtin_interfaces
  std_msgs
  geometry_msgs
  sensor_msgs
  trajectory_msgs
  
  ADD_LINTER_TESTS
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()






