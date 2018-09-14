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

//#include <cv_bridge/cv_bridge.h>
#include <gazebo/test/ServerFixture.hh>
#include <image_transport/image_transport.h>
//#include <opencv2/core/core.hpp>
//#include <opencv2/imgproc/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>

//using namespace cv;
using namespace std::literals::chrono_literals; // NOLINT

/// Test parameters
struct TestParams
{
  /// Path to world file
  std::string world;

  /// Undistorted image topic to subscribe to
  std::string undistorted_topic;

  /// Distorted image topic to subscribe to
  std::string distorted_topic;

  /// Distorted camera info topic to subscribe to
  std::string distorted_cam_topic;
};

//void diffBetween(Mat& orig, Mat& diff, long& total_diff)
//{
//  MatIterator_<Vec3b> it, end;
//  Vec3b orig_pixel, diff_pixel;
//  total_diff = 0;
//
//  for(int i=0; i<orig.rows; i++)
//  {
//    for(int j=0; j<orig.cols; j++)
//    {
//      orig_pixel = orig.at<cv::Vec3b>(i,j);
//      diff_pixel = diff.at<cv::Vec3b>(i,j);
//      total_diff += abs(orig_pixel[0] - diff_pixel[0]) +
//                    abs(orig_pixel[1] - diff_pixel[1]) +
//                    abs(orig_pixel[2] - diff_pixel[2]);
//    }
//  }
//}

class GazeboRosCameraDistortionTest : public gazebo::ServerFixture,
                                      public ::testing::WithParamInterface<TestParams>
{
public:
  void TearDown() override
  {
    // Make sure they're destroyed even if test fails by ASSERT
    cam_sub_undistorted_.shutdown();
    cam_sub_distorted_.shutdown();
    cam_info_distorted_sub_.reset();
  }

  /// Subscribe to distorted images.
  image_transport::Subscriber cam_sub_distorted_;

  /// Subscribe to undistorted images.
  image_transport::Subscriber cam_sub_undistorted_;

  /// Subscribe to distorted camera info.
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_distorted_sub_;
};

TEST_P(GazeboRosCameraDistortionTest, CameraSubscribeTest)
{
  // Load test world and start paused
  this->Load(GetParam().world, true);

  // World
  auto world = gazebo::physics::get_world();
  ASSERT_NE(nullptr, world);

  // Create node and executor
  auto node = std::make_shared<rclcpp::Node>("gazebo_ros_camera_distortion_test");
  ASSERT_NE(nullptr, node);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  // Subscribe to undistorted image
  sensor_msgs::msg::Image::ConstSharedPtr cam_image_undistorted;
  cam_sub_undistorted_ = image_transport::create_subscription(node,
    GetParam().undistorted_topic,
    [&cam_image_undistorted](const sensor_msgs::msg::Image::ConstSharedPtr & _msg) {
      cam_image_undistorted = _msg;
    },
    "raw");

  // Subscribe to distorted image
  sensor_msgs::msg::Image::ConstSharedPtr cam_image_distorted;
  cam_sub_distorted_ = image_transport::create_subscription(node,
    GetParam().distorted_topic,
    [&cam_image_distorted](const sensor_msgs::msg::Image::ConstSharedPtr & _msg) {
      cam_image_distorted = _msg;
    },
    "raw");

  // Subscribe to distorted camera info
  sensor_msgs::msg::CameraInfo::SharedPtr cam_info_distorted;
  cam_info_distorted_sub_ = node->create_subscription<sensor_msgs::msg::CameraInfo>(
    GetParam().distorted_cam_topic,
    [&cam_info_distorted](const sensor_msgs::msg::CameraInfo::SharedPtr _msg) {
      cam_info_distorted = _msg;
    });

  // Update rate is 0.5 Hz, so we step 3s sim time to be sure we get exactly 1 image at 2s
  world->Step(3000);
  executor.spin_once(100ms);
  executor.spin_once(100ms);

  ASSERT_NE(nullptr, cam_image_undistorted);
  ASSERT_NE(nullptr, cam_image_distorted);
  // ASSERT_NE(nullptr, cam_info_distorted);

  // Cleanup transport so we don't get new messages while proceeding the test
  cam_sub_undistorted_.shutdown();
  cam_sub_distorted_.shutdown();
  cam_info_distorted_sub_.reset();

  // load camera coefficients from published ROS information
//  Mat intrinsic_distorted_matrix = Mat(3, 3, CV_64F);
//  if (cam_info_distorted->K.size() == 9)
//  {
//    memcpy(intrinsic_distorted_matrix.data, cam_info_distorted->K.data(),
//      cam_info_distorted->K.size()*sizeof(double));
//  }
//  Mat distortion_coeffs = Mat(5, 1, CV_64F);
//  if (cam_info_distorted->D.size() == 5)
//  {
//    memcpy(distortion_coeffs.data, cam_info_distorted->D.data(),
//      cam_info_distorted->D.size()*sizeof(double));
//  }

  // Information acquired, now test the quality of the undistortion
//  Mat distorted = Mat(cv_bridge::toCvCopy(cam_image_distorted)->image);
//  Mat fixed = distorted.clone();
//  Mat undistorted = Mat(cv_bridge::toCvCopy(cam_image_undistorted)->image);
//
//  // crop the image to remove black borders leftover from (un)distortion
//  int crop_border = 50;
//  cv::Rect myROI(crop_border, crop_border,
//    fixed.rows - 2 * crop_border, fixed.cols - 2 * crop_border);
//  cv::Mat fixed_crop = fixed(myROI);
//  cv::Mat undistorted_crop = undistorted(myROI);
//
//  undistort(distorted, fixed, intrinsic_distorted_matrix, distortion_coeffs);
//
//  //Ensure that we didn't crop away everything
//  ASSERT_GT(distorted.rows, 0);
//  ASSERT_GT(distorted.cols, 0);
//  ASSERT_GT(undistorted.rows, 0);
//  ASSERT_GT(undistorted.cols, 0);
//  ASSERT_GT(fixed.rows, 0);
//  ASSERT_GT(fixed.cols, 0);
//
//  // The difference between the undistorted image and the no-distortion camera
//  // image should be the lowest when we use the correct distortion parameters.
//  long diff1 = 0, diff2 = 0;
//  diffBetween(fixed_crop, undistorted_crop, diff1);
//
//  const double OFFSET = 0.01;
//
//  // test each parameter, varying one at a time
//  for(size_t i = 0; i < 5; ++i)
//  {
//    distortion_coeffs.at<double>(i,0) += OFFSET;
//    undistort(distorted, fixed, intrinsic_distorted_matrix, distortion_coeffs);
//    diffBetween(fixed_crop, undistorted_crop, diff2);
//    EXPECT_GE(diff2, diff1);
//    distortion_coeffs.at<double>(i,0) -= OFFSET;
//
//    distortion_coeffs.at<double>(i,0) -= OFFSET;
//    undistort(distorted, fixed, intrinsic_distorted_matrix, distortion_coeffs);
//    diffBetween(fixed_crop, undistorted_crop, diff2);
//    EXPECT_GE(diff2, diff1);
//    distortion_coeffs.at<double>(i,0) += OFFSET;
//  }
}

INSTANTIATE_TEST_CASE_P(GazeboRosCameraDistortion, GazeboRosCameraDistortionTest, ::testing::Values(
  // TODO(louise) Use mapped topics once this issue is solved:
  // https://github.com/ros-perception/image_common/issues/93
  TestParams({"worlds/gazebo_ros_camera_distortion_barrel.world",
              "undistorted_camera/image_raw",
              "distorted_camera/image_raw",
              "test_cam_distorted/camera_info"}),
  TestParams({"worlds/gazebo_ros_camera_distortion_pincushion.world",
              "test_cam_undistorted/image_raw",
              "test_cam_distorted/image_raw",
              "test_camdistorted_/camera_info"})
), );

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
