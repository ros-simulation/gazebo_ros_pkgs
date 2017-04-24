cmake_minimum_required(VERSION 2.8.3)
project(gazebo_ros_control)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  gazebo_dev
  roscpp
  std_msgs
  control_toolbox
  controller_manager
  hardware_interface
  transmission_interface
  pluginlib
  joint_limits_interface
  urdf
  angles
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    std_msgs
    controller_manager
    control_toolbox
    pluginlib
    hardware_interface
    transmission_interface
    joint_limits_interface
    urdf
    angles
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME} default_robot_hw_sim
)

link_directories(
  ${catkin_LIBRARY_DIRS}
)

include_directories(include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

## Libraries
add_library(${PROJECT_NAME} src/gazebo_ros_control_plugin.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_library(default_robot_hw_sim src/default_robot_hw_sim.cpp)
target_link_libraries(default_robot_hw_sim ${catkin_LIBRARIES})

## Install
install(TARGETS ${PROJECT_NAME} default_robot_hw_sim
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES robot_hw_sim_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
