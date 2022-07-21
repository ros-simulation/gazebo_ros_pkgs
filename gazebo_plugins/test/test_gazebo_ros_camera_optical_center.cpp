// Copyright 2022 Open Source Robotics Foundation, Inc.
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

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
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

  /// Camera info topic to subscribe to
  std::string info_topic;
};

class GazeboRosCameraTest
    : public gazebo::ServerFixture, public ::testing::WithParamInterface<TestParams>
{
 public:
  void TearDown() override
  {
    // Make sure they're destroyed even if test fails by ASSERT
    cam_info_sub_.reset();
    ServerFixture::TearDown();
  }

  /// Subscribe to camera info.
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
};

TEST_P(GazeboRosCameraTest, CameraDefaultOpticalCenter)
{
  // Load test world and start paused
  this->Load(GetParam().world, true);

  // World
  auto world = gazebo::physics::get_world();
  ASSERT_NE(nullptr, world);

  // Create node and executor
  auto node = std::make_shared<rclcpp::Node>("gazebo_ros_camera_optical_center_test");
  ASSERT_NE(nullptr, node);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  // Subscribe to camera info
  sensor_msgs::msg::CameraInfo::SharedPtr cam_info;
  cam_info_sub_ = node->create_subscription<sensor_msgs::msg::CameraInfo>(
      GetParam().info_topic, rclcpp::SensorDataQoS(),
      [&cam_info](const sensor_msgs::msg::CameraInfo::SharedPtr _msg) {
        cam_info = _msg;
      });

  world->Step(5000);
  executor.spin_once(100ms);
  executor.spin_once(100ms);
  executor.spin_once(100ms);
  executor.spin_once(100ms);

  ASSERT_NE(nullptr, cam_info);

  // Load camera coefficients from published ROS information
  auto intrinsic_matrix = cv::Mat(3, 3, CV_64F);
  if (cam_info->k.size() == 9) {
    memcpy(
        intrinsic_matrix.data, cam_info->k.data(),
        cam_info->k.size() * sizeof(double));
  }

  // check optical center cx, cy equals to default values if no sdf
  // tag is provided.
  ASSERT_DOUBLE_EQ(intrinsic_matrix.at<double>(0, 2), 320.5);
  ASSERT_DOUBLE_EQ(intrinsic_matrix.at<double>(1, 2), 240.5);

  // Clean up
  cam_info_sub_.reset();
}

INSTANTIATE_TEST_SUITE_P(
    GazeboRosCamera, GazeboRosCameraTest, ::testing::Values(
    TestParams({"worlds/gazebo_ros_camera_optical_center.world",
                      "/camera1/camera_info"})
  ));

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
