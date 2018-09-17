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

#include <gazebo/test/ServerFixture.hh>
#include <image_transport/image_transport.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>

#include <memory>

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

  auto sub = image_transport::create_subscription(node,
      "test_triggered_cam/camera1/image_raw",
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
  auto pub = node->create_publisher<std_msgs::msg::Empty>(
    "test_triggered_cam/camera1/image_trigger");
  std_msgs::msg::Empty msg;
  pub->publish(msg);

  // Step a bit and check that we get exactly one message
  world->Step(100);
  executor.spin_once(100ms);

  EXPECT_EQ(1u, msg_count);

  // Trigger camera twice
  pub->publish(msg);
  executor.spin_once(100ms);
  executor.spin_once(100ms);
  pub->publish(msg);
  executor.spin_once(100ms);

  // Step a bit and check that we get exactly two messages
  world->Step(100);
  executor.spin_once(100ms);

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
