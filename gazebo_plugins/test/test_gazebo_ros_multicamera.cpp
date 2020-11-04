// Copyright 2019 Open Source Robotics Foundation, Inc.
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

#include <memory>
#include <string>

using namespace std::literals::chrono_literals; // NOLINT

/// Test parameters
struct TestParams
{
  /// Path to world file
  std::string world;

  /// Image topics to subscribe to
  std::string left_topic, right_topic;
};

class GazeboRosMultiCameraTest
  : public gazebo::ServerFixture, public ::testing::WithParamInterface<TestParams>
{
};

// Test that the multi camera images are published and have correct timestamp
TEST_P(GazeboRosMultiCameraTest, CameraSubscribeTest)
{
  // Load test world and start paused
  this->Load(GetParam().world, true);

  // World
  auto world = gazebo::physics::get_world();
  ASSERT_NE(nullptr, world);

  // Create node and executor
  auto node = std::make_shared<rclcpp::Node>("gazebo_ros_multicamera_test");
  ASSERT_NE(nullptr, node);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  // Subscribe
  unsigned int msg_count_left{0};
  builtin_interfaces::msg::Time image_stamp_left;

  auto sub_left = image_transport::create_subscription(
    node.get(), GetParam().left_topic,
    [&](const sensor_msgs::msg::Image::ConstSharedPtr & msg) {
      image_stamp_left = msg->header.stamp;
      ++msg_count_left;
    },
    "raw");

  // Update rate is 0.5 Hz, so we step 3s sim time to be sure we get exactly 1 image at 2s
  world->Step(3000);
  unsigned int sleep = 0;
  unsigned int max_sleep = 30;
  while (sleep < max_sleep && msg_count_left == 0) {
    executor.spin_once(100ms);
    sleep++;
  }

  EXPECT_EQ(1u, msg_count_left);
  EXPECT_EQ(2.0, image_stamp_left.sec);

  // Clean up
  sub_left.shutdown();

  unsigned int msg_count_right{0};
  builtin_interfaces::msg::Time image_stamp_right;
  auto sub_right = image_transport::create_subscription(
    node.get(), GetParam().right_topic,
    [&](const sensor_msgs::msg::Image::ConstSharedPtr & msg) {
      image_stamp_right = msg->header.stamp;
      ++msg_count_right;
    },
    "raw");

  // Update rate is 0.5 Hz, so we step 3s sim time to be sure we get exactly 1 image at 4s
  world->Step(3000);
  sleep = 0;
  max_sleep = 30;
  while (sleep < max_sleep && msg_count_right == 0) {
    executor.spin_once(100ms);
    sleep++;
  }

  EXPECT_EQ(1u, msg_count_right);
  EXPECT_EQ(4.0, image_stamp_right.sec);

  // Clean up
  sub_right.shutdown();
}

INSTANTIATE_TEST_SUITE_P(
  GazeboRosMultiCamera, GazeboRosMultiCameraTest, ::testing::Values(
    TestParams(
      {"worlds/gazebo_ros_multicamera.world",
        "test_cam/camera/left/image_test",
        "test_cam/camera/right/image_test"})
));

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
