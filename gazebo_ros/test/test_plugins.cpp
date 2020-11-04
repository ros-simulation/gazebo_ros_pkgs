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

#include <gtest/gtest.h>

#include <gazebo_ros/testing_utils.hpp>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <memory>
#include <string>
#include <utility>
#include <vector>

struct TestParams
{
  std::vector<const char *> args;
  std::vector<std::string> topics;
};

class TestPlugins : public ::testing::TestWithParam<TestParams>
{
public:
  TestPlugins() {}
  void SetUp() override;
  void TearDown() override;

protected:
  std::unique_ptr<gazebo_ros::GazeboProcess> gazebo_process_;
};

void TestPlugins::SetUp()
{
  std::cout << "Starting gzserver process with [" << GetParam().args[1] << "]" << std::endl;
  gazebo_process_ = std::make_unique<gazebo_ros::GazeboProcess>(GetParam().args);
  ASSERT_GT(gazebo_process_->Run(), 0);
}

void TestPlugins::TearDown()
{
  ASSERT_GE(gazebo_process_->Terminate(), 0);
  gazebo_process_.reset();
}

TEST_P(TestPlugins, TestTopicsReceived)
{
  auto topics = GetParam().topics;
  auto node = std::make_shared<rclcpp::Node>("test_topics_received");
  for (auto topic : topics) {
    auto msg = gazebo_ros::get_message_or_timeout<std_msgs::msg::String>(node, topic);
    EXPECT_NE(msg, nullptr) << topic;
  }

  using namespace std::literals::chrono_literals;
  rclcpp::sleep_for(1s);
}

INSTANTIATE_TEST_SUITE_P(
  Plugins, TestPlugins, ::testing::Values(
    TestParams({{"-s", "./libargs_init.so"}, {"test"}}),
    TestParams({{"-s", "./libcreate_node_without_init.so"}, {"test"}}),
    TestParams({{"-s", "./libmultiple_nodes.so"}, {"testA", "testB"}}),
    TestParams(
      {{"-s", "libgazebo_ros_init.so", "worlds/ros_world_plugin.world",
        "hello_ros_world:/test:=/new_test"}, {"new_test"}}),
    TestParams(
      {{"-s", "libgazebo_ros_init.so", "-s", "libgazebo_ros_factory.so",
        "worlds/ros_world_plugin.world"}, {"test"}}),
    TestParams({{"-s", "libgazebo_ros_init.so", "worlds/sdf_node_plugin.world"}, {"/foo/my_topic"}})
    // cppcheck-suppress syntaxError
));

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
