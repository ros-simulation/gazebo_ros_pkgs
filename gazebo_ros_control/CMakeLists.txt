cmake_minimum_required(VERSION 3.5.0)
project(gazebo_ros_control)

find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
#find_package(control_toolbox REQUIRED)
find_package(controller_manager REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(transmission_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(joint_limits_interface REQUIRED)
find_package(urdf REQUIRED)
find_package(angles REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

set(LIBS
  ${gazebo_dev_LIBRARIES}
  ${rclcpp_LIBRARIES} ${std_msgs_LIBRARIES}
  ${controller_manager_LIBRARIES} #${control_toolbox_LIBRARIES}
  ${controller_interface_LIBRARIES}
  ${pluginlib_LIBRARIES} ${hardware_interface_LIBRARIES}
  ${transmission_interface_LIBRARIES}
  ${joint_limits_interface_LIBRARIES}
  ${urdf_LIBRARIES}
  ${angles_LIBRARIES}
  ${yaml_cpp_vendor_LIBRARIES}
)

set(INCLUDE_DIRS
  ${Boost_INCLUDE_DIRS}
  ${gazebo_dev_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS} ${std_msgs_INCLUDE_DIRS}
  ${controller_manager_INCLUDE_DIRS} #${control_toolbox_INCLUDE_DIRS}
  ${controller_interface_INCLUDE_DIRS}
  ${pluginlib_INCLUDE_DIRS} ${hardware_interface_INCLUDE_DIRS}
  ${transmission_interface_INCLUDE_DIRS}
  ${joint_limits_interface_INCLUDE_DIRS}
  ${urdf_INCLUDE_DIRS}
  ${angles_INCLUDE_DIRS}
  ${yaml_cpp_vendor_INCLUDE_DIRS}
)
include_directories(include)

## Libraries
add_library(${PROJECT_NAME} SHARED src/gazebo_ros_control_plugin.cpp src/default_robot_hw_sim.cpp)
target_link_libraries(${PROJECT_NAME} PRIVATE ${LIBS})
target_include_directories(${PROJECT_NAME} PRIVATE ${INCLUDE_DIRS})

add_library(default_robot_hw_sim SHARED src/default_robot_hw_sim.cpp)
target_link_libraries(default_robot_hw_sim PRIVATE ${LIBS})
target_include_directories(default_robot_hw_sim PRIVATE ${INCLUDE_DIRS})

## Install
install(TARGETS ${PROJECT_NAME} default_robot_hw_sim
  ARCHIVE
  DESTINATION lib
  LIBRARY 
  DESTINATION share/${PROJECT_NAME}
  RUNTIME
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(ament_cmake)
ament_export_dependencies(rclcpp)
ament_export_libraries(${PROJECT_NAME} ${LIBS})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(FILES robot_hw_sim_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
