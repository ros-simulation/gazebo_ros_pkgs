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

#include <memory>
#include <string>

using namespace std::literals::chrono_literals; // NOLINT

/// Test parameters
struct TestParams
{
  /// Path to world file
  std::string world;

  /// Raw image topic to subscribe to
  std::string topic;
};

class GazeboRosCameraTest
  : public gazebo::ServerFixture, public ::testing::WithParamInterface<TestParams>
{
};

// Test that the camera image is published and has correct timestamp
TEST_P(GazeboRosCameraTest, CameraSubscribeTest)
{
  // Load test world and start paused
  this->Load(GetParam().world, true);

  // World
  auto world = gazebo::physics::get_world();
  ASSERT_NE(nullptr, world);

  // Create node and executor
  auto node = std::make_shared<rclcpp::Node>("gazebo_ros_camera_test");
  ASSERT_NE(nullptr, node);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  // Subscribe
  unsigned int msg_count{0};
  builtin_interfaces::msg::Time image_stamp;

  auto sub = image_transport::create_subscription(node, GetParam().topic,
      [&](const sensor_msgs::msg::Image::ConstSharedPtr & msg) {
        image_stamp = msg->header.stamp;
        ++msg_count;
      },
      "raw");

  // Update rate is 0.5 Hz, so we step 3s sim time to be sure we get exactly 1 image at 2s
  world->Step(3000);
  executor.spin_once(100ms);

  EXPECT_EQ(1u, msg_count);
  EXPECT_EQ(2.0, image_stamp.sec);

  // Clean up
  sub.shutdown();
}

INSTANTIATE_TEST_CASE_P(GazeboRosCamera, GazeboRosCameraTest, ::testing::Values(
    TestParams({"worlds/gazebo_ros_camera.world",
      "test_cam/camera/image_test"}),
    TestParams({"worlds/gazebo_ros_camera_16bit.world",
      "test_cam_16bit/image_test_16bit"})
  ), );

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
