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

#include <memory>

using namespace std::chrono_literals;

class TestGazeboRosPluginExample : public ::testing::Test
{
public:
  static void SetUpTestCase();
  static void TearDownTestCase();
  static int pid;
};

int TestGazeboRosPluginExample::pid = -1;

void TestGazeboRosPluginExample::SetUpTestCase()
{
  // Fork process so gazebo can be run as child
  pid = fork();
  if (pid < 0) {
    throw std::runtime_error("fork failed");
  }

  // Child process
  if (0 == pid) {
    // Run gazebo with the example plugin with publishes a string to /test
    if (execlp("gzserver", "/usr/bin/gzserver", "-s", "./libgazebo_ros_plugin_example.so", NULL)) {
      exit(1);
    }
  }
}

void TestGazeboRosPluginExample::TearDownTestCase()
{
  // If fork failed, don't need to do anything
  if (pid < 0) {
    return;
  }

  // Kill gazebo (simulating ^C command)
  if (kill(pid, SIGINT)) {
    throw std::runtime_error("could not kill");
  }

  // Wait for gazebo to terminate
  wait(nullptr);
}

TEST_F(TestGazeboRosPluginExample, todo)
{
  // Create node and executor
  rclcpp::executors::SingleThreadedExecutor executor;
  auto node = std::make_shared<rclcpp::Node>("my_node");
  executor.add_node(node);

  // Subscribe to topic published by plugin
  using StringPtr = std_msgs::msg::String::SharedPtr;
  StringPtr msg;
  auto subscriber = node->create_subscription<std_msgs::msg::String>("test",
      [&msg](const StringPtr _msg) {
        printf("message received!!\n");
        msg = _msg;
      });

  // Wait until message is received or timeout after 5 seconds
  using namespace std::literals::chrono_literals;
  auto timeout = node->now() + rclcpp::Duration(5s);
  while (nullptr == msg && node->now() < timeout) {
    executor.spin_once(50ms);
  }

  // Wait a little while so gazebo isn't torn down before created
  rclcpp::sleep_for(1s);

  // Assert a message was received
  ASSERT_NE(msg, nullptr);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
