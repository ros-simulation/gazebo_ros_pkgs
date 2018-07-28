find_package(ament_cmake_gtest REQUIRED)

# Worlds
file(COPY worlds DESTINATION .)

# Tests
set(tests
  test_gazebo_ros_force
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
    "rclcpp"
    "gazebo_dev"
    "gazebo_ros"
    "geometry_msgs"
  )
endforeach()

