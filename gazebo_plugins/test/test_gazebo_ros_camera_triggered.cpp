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

#include <memory>
#include <string>

#include "gazebo/test/ServerFixture.hh"

#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>

using namespace std::literals::chrono_literals; // NOLINT

class GazeboRosTriggeredCameraTest : public gazebo::ServerFixture
{
};

// Test if the camera image is published at all, and that the timestamp
// is not too long in the past.
TEST_F(GazeboRosTriggeredCameraTest, CameraSubscribeTest)
{
  // Load test world and start paused
  this->Load("worlds/gazebo_ros_camera_triggered.world", true);

  // World
  auto world = gazebo::physics::get_world();
  ASSERT_NE(nullptr, world);

  // Create node and executor
  auto node = std::make_shared<rclcpp::Node>("gazebo_ros_camera_triggered_test");
  ASSERT_NE(nullptr, node);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  // Subscribe
  unsigned int msg_count{0};
  builtin_interfaces::msg::Time image_stamp;

  auto sub = image_transport::create_subscription(
    node.get(),
    "test_triggered_cam/image_raw_test",
    [&](const sensor_msgs::msg::Image::ConstSharedPtr & msg) {
      image_stamp = msg->header.stamp;
      ++msg_count;
    },
    "raw");

  // Step a bit and check that we do not get any messages
  world->Step(100);
  executor.spin_once(100ms);

  EXPECT_EQ(0u, msg_count);

  // Trigger camera once
  std::string trigger_topic{"test_triggered_cam/image_trigger_test"};
  auto pub = node->create_publisher<std_msgs::msg::Empty>(
    trigger_topic, rclcpp::QoS(rclcpp::KeepLast(1)));
  std_msgs::msg::Empty msg;

  // Wait for trigger subscriber
  unsigned int sleep = 0;
  unsigned int max_sleep = 30;
  while (sleep < max_sleep && node->count_subscribers(trigger_topic) == 0) {
    gazebo::common::Time::MSleep(100);
  }
  EXPECT_EQ(1u, node->count_subscribers(trigger_topic));

  pub->publish(msg);

  // Step a bit and check that we get exactly one message
  sleep = 0;
  while (sleep < max_sleep && msg_count == 0) {
    world->Step(100);
    executor.spin_once(100ms);
    sleep++;
  }

  EXPECT_EQ(1u, msg_count);

  // Trigger camera twice
  pub->publish(msg);
  executor.spin_once(100ms);
  executor.spin_once(100ms);
  pub->publish(msg);
  executor.spin_once(100ms);

  // Step a bit and check that we get exactly two messages
  sleep = 0;
  while (sleep < max_sleep && msg_count < 3) {
    world->Step(100);
    executor.spin_once(100ms);
    sleep++;
  }

  EXPECT_EQ(3u, msg_count);

  sub.shutdown();
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
