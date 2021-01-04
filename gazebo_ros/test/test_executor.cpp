// Copyright 2021 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <thread>
#include <sstream>

#include <gtest/gtest.h>
#include <gazebo_ros/executor.hpp>
#include <rclcpp/node.hpp>

// Regression test for https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1169
TEST(TestExecutor, AddRemoveNodes)
{
  rclcpp::init(0, nullptr);

  // Create an Executor
  auto executor = std::make_shared<gazebo_ros::Executor>();

  // Add and remove nodes repeatedly
  // Test that this does not cause a segfault
  size_t num_nodes = 100;
  for (size_t i = 0; i < num_nodes; ++i) {
    std::ostringstream name;
    name << "node_" << i;
    auto node = std::make_shared<rclcpp::Node>(name.str());
    executor->add_node(node);
    // Sleeping here helps exaggerate the issue
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    executor->remove_node(node);
  }
  rclcpp::shutdown();
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
