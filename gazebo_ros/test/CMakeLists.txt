find_package(ament_cmake_gtest REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(launch_testing_ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# Plugins
set(plugins
  args_init
  create_node_without_init
  multiple_nodes
  ros_world_plugin
  sdf_node
)

foreach(plugin ${plugins})
  add_library(${plugin} SHARED plugins/${plugin}.cpp)
  target_link_libraries(${plugin} gazebo_ros_node)
  ament_target_dependencies(${plugin}
    "rclcpp"
    "gazebo_dev"
    "std_msgs"
  )
endforeach()

# Worlds
file(COPY worlds DESTINATION .)

# Tests
set(tests
  test_conversions
  test_gazebo_ros_factory
  test_gazebo_ros_init
  test_gazebo_ros_joint_effort
  test_gazebo_ros_link_wrench
  test_gazebo_ros_properties
  test_gazebo_ros_state
  test_node
  test_plugins
  test_qos
  test_sim_time
  test_utils
)

foreach(test ${tests})
  ament_add_gtest(${test}
    ${test}.cpp
    WORKING_DIRECTORY
      ${CMAKE_CURRENT_BINARY_DIR}
    # Long timeout because has to run gazebo several times
    TIMEOUT
      120
  )
  target_link_libraries(${test}
    gazebo_ros_node
    gazebo_ros_utils
    gazebo_test_fixture
  )
  ament_target_dependencies(${test}
    "builtin_interfaces"
    "gazebo_dev"
    "gazebo_msgs"
    "geometry_msgs"
    "rclcpp"
    "sensor_msgs"
    "std_msgs"
    "std_srvs"
  )
endforeach()

add_executable(mock_robot_state_publisher
  mock_robot_state_publisher/robot_state_publisher.cpp
)
ament_target_dependencies(mock_robot_state_publisher
  "rclcpp"
  "std_msgs"
)
add_launch_test(entity_spawner.test.py
  ARGS "mock_robot_state_publisher:=${CMAKE_CURRENT_BINARY_DIR}/mock_robot_state_publisher"
)

add_executable(mock_gazebo_ros_factory
  mock_gazebo_ros_factory/gazebo_ros_factory.cpp
)
ament_target_dependencies(mock_gazebo_ros_factory
  "rclcpp"
  "gazebo_msgs"
)
add_launch_test(mock_entity_spawner.test.py
  ARGS
    "mock_robot_state_publisher:=${CMAKE_CURRENT_BINARY_DIR}/mock_robot_state_publisher"
    "mock_gazebo_ros_factory:=${CMAKE_CURRENT_BINARY_DIR}/mock_gazebo_ros_factory"
)
