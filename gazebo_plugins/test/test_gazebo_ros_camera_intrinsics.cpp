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

#include <opencv2/core/core.hpp>
#include <gazebo/test/ServerFixture.hh>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>

using namespace std::literals::chrono_literals; // NOLINT

/// Test parameters
struct TestParams
{
  /// Path to world file
  std::string world;

  /// Camera1 info topic to subscribe to
  std::string info_topic_1;

  /// Camera2 info topic to subscribe to
  std::string info_topic_2;
};

class GazeboRosCameraIntrinsicsTest
    : public gazebo::ServerFixture, public ::testing::WithParamInterface<TestParams>
{
 public:
  void TearDown() override
  {
    // Make sure they're destroyed even if test fails by ASSERT
    cam_info_1_sub_.reset();
    ServerFixture::TearDown();
  }

  /// Subscribe to camera info_topic_1.
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_1_sub_;

  /// Subscribe to camera info_topic_2.
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_2_sub_;
};

TEST_P(GazeboRosCameraIntrinsicsTest, CameraIntrinsics)
{
  // Load test world and start paused
  this->Load(GetParam().world, true);

  // World
  auto world = gazebo::physics::get_world();
  ASSERT_NE(nullptr, world);

  // Create node and executor
  auto node = std::make_shared<rclcpp::Node>("gazebo_ros_camera_intrinsics_test");
  ASSERT_NE(nullptr, node);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  // Subscribe to info_topic_1
  sensor_msgs::msg::CameraInfo::SharedPtr cam_info_1;
  cam_info_1_sub_ = node->create_subscription<sensor_msgs::msg::CameraInfo>(
      GetParam().info_topic_1, rclcpp::SensorDataQoS(),
      [&cam_info_1](const sensor_msgs::msg::CameraInfo::SharedPtr _msg) {
        cam_info_1 = _msg;
      });

  // Subscribe to info_topic_2
  sensor_msgs::msg::CameraInfo::SharedPtr cam_info_2;
  cam_info_2_sub_ = node->create_subscription<sensor_msgs::msg::CameraInfo>(
      GetParam().info_topic_2, rclcpp::SensorDataQoS(),
      [&cam_info_2](const sensor_msgs::msg::CameraInfo::SharedPtr _msg) {
        cam_info_2 = _msg;
      });

  world->Step(5000);
  executor.spin_once(100ms);
  executor.spin_once(100ms);
  executor.spin_once(100ms);
  executor.spin_once(100ms);

  ASSERT_NE(nullptr, cam_info_1);
  ASSERT_NE(nullptr, cam_info_2);

  // Cleanup transport so we don't get new messages while proceeding the test
  // Clean up
  cam_info_1_sub_.reset();
  cam_info_2_sub_.reset();

  // Load camera coefficients from published ROS information
  auto intrinsic_matrix_1 = cv::Mat(3, 3, CV_64F);
  if (cam_info_1->k.size() == 9) {
  memcpy(
      intrinsic_matrix_1.data, cam_info_1->k.data(),
      cam_info_1->k.size() * sizeof(double));
  }

  auto intrinsic_matrix_2 = cv::Mat(3, 3, CV_64F);
  if (cam_info_2->k.size() == 9) {
    memcpy(
        intrinsic_matrix_2.data, cam_info_2->k.data(),
        cam_info_2->k.size() * sizeof(double));
  }

  // ##############################################################
  // camera1 with no camera lens intrinsics sdf tag provided
  {
    // same as sdf file
    double width = 640.0;
    double height = 480.0;
    double hfov = 1.3962634;

    // calculations
    double ratio = width / height;
    double vfov = 2.0 * atan(tan(hfov / 2.0) / ratio);
    double computed_fx =
        (static_cast<double>(width)) / (2.0 * tan(hfov / 2.0));
    double computed_fy =
        (static_cast<double>(height)) / (2.0 * tan(vfov / 2.0));

    // verify that the camera intrinsics from gazebo when no camera lens
    // intrinsics tag is provided in sdf file.
    // optical centre
    ASSERT_DOUBLE_EQ(intrinsic_matrix_1.at<double>(0, 2), width / 2);
    ASSERT_DOUBLE_EQ(intrinsic_matrix_1.at<double>(1, 2), height / 2);
    // focal length
    ASSERT_DOUBLE_EQ(intrinsic_matrix_1.at<double>(0, 0), 381.36241912841797);
    ASSERT_DOUBLE_EQ(intrinsic_matrix_1.at<double>(1, 1), 381.36242866516113);

    // check if focal length from gazebo matches computed focal length
    double error = 1e-2;
    ASSERT_NEAR(intrinsic_matrix_1.at<double>(0, 0), computed_fx, error);
    ASSERT_NEAR(intrinsic_matrix_1.at<double>(1, 1), computed_fy, error);
  }

  // camera2 with camera lens intrinsics sdf tag provided
  {
    // optical centre
    ASSERT_DOUBLE_EQ(intrinsic_matrix_2.at<double>(0, 2), 500.0);
    ASSERT_DOUBLE_EQ(intrinsic_matrix_2.at<double>(1, 2), 500.0);
    // focal length
    ASSERT_DOUBLE_EQ(intrinsic_matrix_2.at<double>(0, 0), 1000.0);
    ASSERT_DOUBLE_EQ(intrinsic_matrix_2.at<double>(1, 1), 800.0);
  }
}

INSTANTIATE_TEST_SUITE_P(
    GazeboRosCamera, GazeboRosCameraIntrinsicsTest, ::testing::Values(
    TestParams({"worlds/gazebo_ros_camera_intrinsics.world",
                "/camera1/camera_info", "/camera2/camera_info"})
));

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
