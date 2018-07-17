cmake_minimum_required(VERSION 3.5)
project(gazebo_dev)

find_package(ament_cmake REQUIRED)
ament_package(CONFIG_EXTRAS "cmake/gazebo_dev-extras.cmake")
