find_package(ament_cmake_gtest REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
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

# Helper macro for generating tests
macro(add_gazebo_ros_pkgs_test test)
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
endmacro(add_gazebo_ros_pkgs_test)

# Tests that don't require gpu to run
set(no_gpu_tests
  test_conversions
  test_node
  test_utils
)

foreach(test ${no_gpu_tests})
  add_gazebo_ros_pkgs_test("${test}")
endforeach()

if(ENABLE_DISPLAY_TESTS)
	set(gpu_tests
		test_gazebo_ros_factory
		test_gazebo_ros_init
		test_gazebo_ros_state
		test_plugins
		test_sim_time
	)

	foreach(gpu_test ${tests})
		add_gazebo_ros_pkgs_test("${ŧest}")

		set_test_properties(${test}
			PROPERTIES
			LABELS "gpu_test"
		)
	endforeach()
endif()
