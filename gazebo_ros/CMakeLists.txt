cmake_minimum_required(VERSION 3.5)
project(gazebo_ros)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # we dont use add_compile_options with pedantic in message packages
  # because the Python C extensions dont comply with it
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -Wno-inconsistent-missing-override")
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
find_package(builtin_interfaces REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rcl REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tinyxml_vendor REQUIRED)

# Needed for EOL GUI notice
find_package(Qt5Widgets REQUIRED)
find_package(Qt5Core REQUIRED)

link_directories(
  ${gazebo_dev_LIBRARY_DIRS}
)

ament_python_install_package(scripts)

# gazebo_ros_node
add_library(gazebo_ros_node SHARED
  src/executor.cpp
  src/node.cpp
  src/qos.cpp
)
target_include_directories(gazebo_ros_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${gazebo_dev_INCLUDE_DIRS}
)
if(WIN32)
  target_compile_definitions(gazebo_ros_node PRIVATE "GAZEBO_ROS_NODE_BUILDING_DLL")
endif()

ament_target_dependencies(gazebo_ros_node
  "gazebo_dev"
  "rclcpp"
  "rcl"
  "rmw"
)
ament_export_libraries(gazebo_ros_node)

# gazebo_ros_utils
add_library(gazebo_ros_utils SHARED
  src/utils.cpp
)
target_include_directories(gazebo_ros_utils
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
if(WIN32)
  target_compile_definitions(gazebo_ros_utils PRIVATE "GAZEBO_ROS_UTILS_BUILDING_DLL")
endif()
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
  "gazebo_msgs"
  "rclcpp"
  "std_srvs"
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

# gazebo_ros_properties
add_library(gazebo_ros_properties SHARED
  src/gazebo_ros_properties.cpp
)
ament_target_dependencies(gazebo_ros_properties
  "rclcpp"
  "gazebo_dev"
  "gazebo_msgs"
)
target_link_libraries(gazebo_ros_properties
  gazebo_ros_node
)
ament_export_libraries(gazebo_ros_properties)

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

# gazebo_ros_force_system
add_library(gazebo_ros_force_system SHARED
  src/gazebo_ros_force_system.cpp
)
ament_target_dependencies(gazebo_ros_force_system
  "rclcpp"
  "gazebo_dev"
  "gazebo_msgs"
)
target_link_libraries(gazebo_ros_force_system
  gazebo_ros_node
)

set(CMAKE_AUTOMOC ON)
add_library(gazebo_ros_eol_gui  SHARED src/gazebo_ros_eol_gui.cpp)

target_link_libraries(gazebo_ros_eol_gui
  ${Qt5Core_LIBRARIES}
  ${Qt5Widgets_LIBRARIES}
  gazebo_ros_utils
)
ament_target_dependencies(gazebo_ros_eol_gui
  "gazebo_dev"
)
set(CMAKE_AUTOMOC OFF)

ament_export_libraries(gazebo_ros_force_system)

ament_export_include_directories(include)
ament_export_dependencies(rclcpp)
ament_export_dependencies(gazebo_dev)

if(BUILD_TESTING)
  add_subdirectory(test)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_cpplint
  )
  ament_lint_auto_find_test_dependencies()
  ament_cpplint(
    FILTERS -build/include_order
  )
endif()

ament_package()

install(DIRECTORY include/
        DESTINATION include)

install(
  PROGRAMS
    scripts/gazebo_ros_paths.py
    DESTINATION lib/${PROJECT_NAME}/
)

install(
  TARGETS
    gazebo_ros_factory
    gazebo_ros_force_system
    gazebo_ros_init
    gazebo_ros_node
    gazebo_ros_properties
    gazebo_ros_state
    gazebo_ros_utils
    gazebo_ros_eol_gui
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(
  PROGRAMS
    scripts/spawn_entity.py
    DESTINATION lib/${PROJECT_NAME}/
)

install(DIRECTORY
  launch worlds
  DESTINATION share/${PROJECT_NAME}/
)
