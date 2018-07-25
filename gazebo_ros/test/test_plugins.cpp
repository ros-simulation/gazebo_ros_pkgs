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
#include <string>
#include <utility>
#include <vector>

struct TestParams
{
  std::string plugin;
  std::vector<std::string> topics;
  std::vector<std::string> extra_args;
};

class TestPlugins : public ::testing::TestWithParam<TestParams>
{
public:
  // Documentation inherited
  void TearDown() override;

  // Documentation inherited
  void SetUp() override;

private:
  int pid_ = -1;
};

void TestPlugins::SetUp()
{
  auto params = GetParam();

  unsigned int i = 0;
  std::vector<const char *> args(params.extra_args.size() + 5);
  args[i++] = "/usr/bin/gzserver";
  args[i++] = "--verbose";
  args[i++] = "-s";
  args[i++] = params.plugin.c_str();
  for (unsigned int j = i; j < params.extra_args.size() + i; ++j) {
    args[j] = params.extra_args[j - i].c_str();
  }
  args[i + params.extra_args.size()] = NULL;

  // Fork process so gazebo can be run as child
  pid_ = fork();

  // Child process
  if (0 == pid_) {
    // Run gazebo with a plugin with publishes a string to /test
    ASSERT_TRUE(execvp("gzserver", const_cast<char **>(args.data())));
    exit(1);
  } else {
    // Parent process
    ASSERT_GT(pid_, 0);
  }
}

void TestPlugins::TearDown()
{
  // Kill gazebo (simulating ^C command)
  EXPECT_FALSE(kill(pid_, SIGINT));

  // Wait for gazebo to terminate
  wait(nullptr);
}

TEST_P(TestPlugins, TestTopicsReceived)
{
  auto topics = GetParam().topics;
  // Create node and executor
  rclcpp::executors::SingleThreadedExecutor executor;
  auto node = std::make_shared<rclcpp::Node>("my_node");
  executor.add_node(node);


  // Subscribe to topics published by plugin
  using StringPtr = std_msgs::msg::String::SharedPtr;

  // Track which topics we have received a message from
  size_t topics_received_from = 0;
  std::vector<bool> received(topics.size(), false);

  std::vector<std::shared_ptr<rclcpp::Subscription<std_msgs::msg::String>>> subs;
  for (size_t i = 0; i < topics.size(); ++i) {
    subs.push_back(node->create_subscription<std_msgs::msg::String>(topics[i],
      [&topics_received_from, &received, i](const StringPtr msg) {
        (void) msg;
        // If this is the first message from this topic, increment the counter
        if (!received[i]) {
          received[i] = true;
          ++topics_received_from;
        }
      }));
  }

  // Wait until message is received or timeout occurs
  using namespace std::literals::chrono_literals;
  auto timeout = node->now() + rclcpp::Duration(15s);

  while (topics_received_from != topics.size() && node->now() < timeout) {
    executor.spin_once(200ms);
  }

  // Wait a little while so gazebo isn't torn down before created
  rclcpp::sleep_for(1s);

  // Assert a message was received
  EXPECT_EQ(topics_received_from, topics.size());
}

INSTANTIATE_TEST_CASE_P(Plugins, TestPlugins, ::testing::Values(
    TestParams({"./libargs_init.so", {"test"}, {}}),
    TestParams({"./libcreate_node_without_init.so", {"test"}, {}}),
    TestParams({"./libmultiple_nodes.so", {"testA", "testB"}, {}}),
    TestParams({"libgazebo_ros_init.so", {"new_test"}, {"worlds/ros_world_plugin.world",
        "ros_world_plugin:/test:=/new_test"}}),
    TestParams({"libgazebo_ros_init.so", {"/foo/my_topic"}, {"worlds/sdf_node_plugin.world"}})
  ), );

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
