# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(gazebo_ros_control)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin
  REQUIRED COMPONENTS
  roscpp
  gazebo_ros
  controller_manager
  hardware_interface
)


# Find gazebo
include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

add_library(gazebo_ros_control src/gazebo_ros_control.cpp)
target_link_libraries(gazebo_ros_control ${catkin_LIBRARIES})

# TODO: fill in what other packages will need to use this package
## DEPENDS: system dependencies of this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## INCLUDE_DIRS: 
## LIBRARIES: libraries you create in this project that dependent projects also need
catkin_package(
  DEPENDS
    roscpp
    gazebo_ros
    controller_manager
  CATKIN_DEPENDS # TODO
  INCLUDE_DIRS # TODO include
  LIBRARIES # TODO
)
