find_package(ament_cmake_gtest REQUIRED)

# Worlds
file(COPY worlds DESTINATION .)

# Tests
set(tests
  test_gazebo_ros_camera
  test_gazebo_ros_diff_drive
  test_gazebo_ros_force
  test_gazebo_ros_imu_sensor
  test_gazebo_ros_joint_state_publisher
  test_gazebo_ros_ray_sensor
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
    gazebo_test_fixture
  )
  ament_target_dependencies(${test}
    "gazebo_dev"
    "gazebo_ros"
    "geometry_msgs"
    "image_transport"
    "nav_msgs"
    "rclcpp"
    "sensor_msgs"
  )
endforeach()

