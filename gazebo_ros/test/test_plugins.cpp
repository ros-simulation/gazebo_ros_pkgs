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
};

class TestPlugins : public ::testing::TestWithParam<TestParams>
{
public:
  // Documentation inherited
  void TearDown() override;

  /// Run gzserver with the given system plugin
  /// \param[in] params Path of plugin library to load and a vector of topics to subscribe to.
  void Run(TestParams params);

private:
  int pid_ = -1;
};

void TestPlugins::TearDown()
{
  // If fork failed, don't need to do anything
  ASSERT_GE(pid_, 0);

  // Kill gazebo (simulating ^C command)
  EXPECT_FALSE(kill(pid_, SIGINT));

  // Wait for gazebo to terminate
  wait(nullptr);
}

void TestPlugins::Run(TestParams params)
{
  const char * plugin = params.plugin.c_str();
  auto topics = params.topics;

  // Fork process so gazebo can be run as child
  pid_ = fork();
  if (pid_ < 0) {
    throw std::runtime_error("fork failed");
  }

  // Child process
  if (0 == pid_) {
    // Run gazebo with a plugin with publishes a string to /test
    ASSERT_TRUE(execlp("gzserver", "/usr/bin/gzserver", "-s", plugin, NULL));

    exit(1);
  }

  // Create node and executor
  rclcpp::executors::SingleThreadedExecutor executor;
  auto node = std::make_shared<rclcpp::Node>("my_node");
  executor.add_node(node);

  // Subscribe to topics published by plugin
  using StringPtr = std_msgs::msg::String::SharedPtr;
  std::vector<StringPtr> msgs;

  std::vector<std::shared_ptr<rclcpp::Subscription<std_msgs::msg::String>>> subs;
  for (auto topic : topics)
  {
    subs.push_back(node->create_subscription<std_msgs::msg::String>(topic,
        [&msgs](const StringPtr msg) {
          printf("message received!!\n");
          msgs.push_back(msg);
        }));
  }

  // Wait until message is received or timeout after 5 seconds
  using namespace std::literals::chrono_literals;
  auto timeout = node->now() + rclcpp::Duration(5s);
  while (msgs.size() != topics.size() && node->now() < timeout) {
    executor.spin_once(50ms);
  }

  // Wait a little while so gazebo isn't torn down before created
  rclcpp::sleep_for(1s);

  // Assert a message was received
  EXPECT_EQ(msgs.size(), topics.size());
}

TEST_P(TestPlugins, Run)
{
  Run(GetParam());
}

INSTANTIATE_TEST_CASE_P(Plugins, TestPlugins, ::testing::Values(
   TestParams({"./libargs_init.so", {"test"}}),
   TestParams({"./libcreate_node_without_init.so", {"test"}}),
   TestParams({"./libmultiple_nodes.so", {"testA", "testB"}})
));

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
