find_package(ament_cmake_gtest REQUIRED)
find_package(ament_lint_auto REQUIRED)
find_package(std_msgs REQUIRED)

# Linters
ament_lint_auto_find_test_dependencies()

# Plugins
set(plugins
  args_init
  create_node_without_init
  multiple_nodes
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

# Tests
set(tests
  test_gazebo_ros_init
  test_plugins
)

foreach(test ${tests})
  ament_add_gtest(${test}
    ${test}.cpp
    WORKING_DIRECTORY
      ${CMAKE_CURRENT_BINARY_DIR}
  )
  ament_target_dependencies(${test}
    "rclcpp"
    "std_msgs"
  )
endforeach()

