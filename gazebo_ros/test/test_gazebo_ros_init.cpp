// Copyright 2018 Open Source Robotics Foundation, Inc.
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

#include <unistd.h>
#include <stdlib.h>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <chrono>
#include <memory>
#include <thread>

using namespace std::chrono_literals;

class GazeboRosInitTest : public ::testing::Test
{
public:
  void TearDown();
  int pid_{-1};
};

void GazeboRosInitTest::TearDown()
{
  // If fork failed, don't need to do anything
  if (pid_ < 0) {
    return;
  }

  // Kill gazebo (simulating ^C command)
  if (kill(pid_, SIGINT)) {
    throw std::runtime_error("could not kill");
  }

  // Wait for gazebo to terminate
  wait(nullptr);
}

// Check that the option to remap /test to /new_test passed from the command line
// affects a world plugin loaded afterwards.
TEST_F(GazeboRosInitTest, load)
{
  // Fork process so gazebo can be run as child
  pid_ = fork();
  if (pid_ < 0) {
    throw std::runtime_error("fork failed");
  }

  // Child process
  if (0 == pid_) {

    ASSERT_TRUE(execlp("gzserver", "/usr/bin/gzserver", "--verbose", "-s", "libgazebo_ros_init.so",
       "ros_world_plugin.world", "ros_world_plugin:/test:=/new_test",  NULL));

    exit(1);
  }

  // Create node and executor
  rclcpp::executors::SingleThreadedExecutor executor;
  auto node = std::make_shared<rclcpp::Node>("test_node");
  executor.add_node(node);

  bool received{false};

  // Subscribe to /new_test topic
  auto sub = node->create_subscription<std_msgs::msg::String>("new_test",
      [&received](const std_msgs::msg::String::SharedPtr) {
        received = true;
      });

  // Wait until message is received or timeout after 5 seconds
  using namespace std::literals::chrono_literals;
  auto timeout = node->now() + rclcpp::Duration(5s);
  while (!received && node->now() < timeout) {
    executor.spin_once(50ms);
  }

  // Wait a little while so gazebo isn't torn down before created
  rclcpp::sleep_for(1s);

  // Assert a message was received
  EXPECT_TRUE(received);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

