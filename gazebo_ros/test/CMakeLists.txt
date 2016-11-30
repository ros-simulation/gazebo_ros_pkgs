# Tests
if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  add_rostest_gtest(set_model_state-test
                    set_model_state/set_model_state_test.test
                    set_model_state/set_model_state_test.cpp)
  target_link_libraries(set_model_state-test ${catkin_LIBRARIES})

  add_rostest_gtest(spawn_delete-test
                    spawn_delete/spawn_delete.test
                    spawn_delete/spawn_delete.cpp)
  target_link_libraries(spawn_delete-test gazebo_test_fixture ${catkin_LIBRARIES})
endif()
