# Tests
if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(set_model_state-test
                    set_model_state_test/set_model_state_test.test 
                    set_model_state_test/set_model_state_test.cpp)
  target_link_libraries(set_model_state-test ${catkin_LIBRARIES})
endif()
