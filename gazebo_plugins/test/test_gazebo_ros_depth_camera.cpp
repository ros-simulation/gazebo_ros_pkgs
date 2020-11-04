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
#include <sensor_msgs/msg/point_cloud2.hpp>
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
  std::string raw_image_topic;

  /// Depth image topic to subscribe to
  std::string depth_image_topic;

  /// Point cloud topic to subscribe to
  std::string pcl_topic;
};

class GazeboRosDepthCameraTest
  : public gazebo::ServerFixture, public ::testing::WithParamInterface<TestParams>
{
};

// Test that the camera image is published and has correct timestamp
TEST_P(GazeboRosDepthCameraTest, DepthCameraSubscribeTest)
{
  // Load test world and start paused
  this->Load(GetParam().world, true);

  // World
  auto world = gazebo::physics::get_world();
  ASSERT_NE(nullptr, world);

  // Create node and executor
  auto node = std::make_shared<rclcpp::Node>("gazebo_ros_depth_camera_test");
  ASSERT_NE(nullptr, node);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  // Subscribe to raw images
  unsigned int msg_count{0};
  builtin_interfaces::msg::Time image_stamp;

  auto sub = image_transport::create_subscription(
    node.get(), GetParam().raw_image_topic,
    [&](const sensor_msgs::msg::Image::ConstSharedPtr & msg) {
      image_stamp = msg->header.stamp;
      ++msg_count;
    }, "raw");

  // Update rate is 0.5 Hz, so we step 3s sim time to be sure we get exactly 1 image at 2s
  world->Step(3000);
  unsigned int sleep = 0;
  unsigned int max_sleep = 30;
  while (sleep < max_sleep && msg_count == 0) {
    executor.spin_once(100ms);
    sleep++;
  }

  EXPECT_EQ(1u, msg_count);
  EXPECT_EQ(2.0, image_stamp.sec);

  // Clean up
  sub.shutdown();

  // Subscribe to depth images
  unsigned int msg_count_depth{0};
  builtin_interfaces::msg::Time image_stamp_depth;

  auto sub_depth = image_transport::create_subscription(
    node.get(), GetParam().depth_image_topic,
    [&](const sensor_msgs::msg::Image::ConstSharedPtr & msg) {
      image_stamp_depth = msg->header.stamp;
      ++msg_count_depth;
    }, "raw");

  // Update rate is 0.5 Hz, so we step 3s sim time to be sure we get exactly 1 image at 2s
  world->Step(3000);
  sleep = 0;
  max_sleep = 30;
  while (sleep < max_sleep && msg_count_depth == 0) {
    executor.spin_once(100ms);
    sleep++;
  }

  EXPECT_EQ(1u, msg_count_depth);
  EXPECT_EQ(4.0, image_stamp_depth.sec);

  // Clean up
  sub_depth.shutdown();

  // Subscribe to point cloud
  unsigned int msg_count_pcl{0};
  builtin_interfaces::msg::Time image_stamp_pcl;

  auto sub_pcl = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    GetParam().pcl_topic, rclcpp::QoS(rclcpp::KeepLast(1)),
    [&](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
      image_stamp_pcl = msg->header.stamp;
      ++msg_count_pcl;
    });

  // Update rate is 0.5 Hz, so we step 3s sim time to be sure we get exactly 1 image at 2s
  world->Step(3000);
  sleep = 0;
  max_sleep = 30;
  while (sleep < max_sleep && msg_count_pcl == 0) {
    executor.spin_once(100ms);
    sleep++;
  }

  EXPECT_EQ(1u, msg_count_pcl);
  EXPECT_EQ(8.0, image_stamp_pcl.sec);
}

INSTANTIATE_TEST_SUITE_P(
  GazeboRosDepthCamera, GazeboRosDepthCameraTest, ::testing::Values(
    TestParams(
      {"worlds/gazebo_ros_depth_camera.world",
        "test_cam/camera/raw_image_test",
        "test_cam/camera/depth_image_test",
        "test_cam/camera/points_test"})
));

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
