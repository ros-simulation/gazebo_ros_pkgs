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

using namespace std::literals::chrono_literals; // NOLINT

class GazeboRosCameraTest : public gazebo::ServerFixture
{
};

// Test if the camera image is published at all, and that the timestamp
// is not too long in the past.
TEST_F(GazeboRosCameraTest, CameraSubscribeTest)
{
  // Load test world and start paused
  this->Load("worlds/gazebo_ros_camera.world", true);

  // World
  auto world = gazebo::physics::get_world();
  ASSERT_NE(nullptr, world);

  // Create node and executor
  auto node = std::make_shared<rclcpp::Node>("gazebo_ros_joint_state_publisher_test");
  ASSERT_NE(nullptr, node);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  // Subscribe
  bool has_new_image{false};
  builtin_interfaces::msg::Time image_stamp;

  auto sub = image_transport::create_subscription(node, "test_cam/image_raw",
    [&](const sensor_msgs::msg::Image::ConstSharedPtr & msg) {
      image_stamp = msg->header.stamp;
      has_new_image = true;
    },
    "raw");

  // Update rate is 0.5 Hz, so we step 3s sim time to be sure we get 1 image at 2s
  world->Step(3000);
  executor.spin_once(100ms);
  gazebo::common::Time::MSleep(100);
  EXPECT_TRUE(has_new_image);

  EXPECT_EQ(2.0, image_stamp.sec);
  EXPECT_EQ(0.0, image_stamp.nanosec);

  // Clean up
  sub.shutdown();

  // Artificialy trigger sigInt
  // TODO(louise) Why doesn't the test end?
  gazebo::event::Events::sigInt();
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();

std::cout << ret << std::endl;

  return ret;
}
