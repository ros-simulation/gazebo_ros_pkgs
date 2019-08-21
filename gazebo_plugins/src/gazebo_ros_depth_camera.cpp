// Copyright 2019 Open Source Robotics Foundation
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

#include <sdf/Element.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/rendering/Distortion.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <ignition/math/Helpers.hh>

#include <camera_info_manager/camera_info_manager.h>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/node.hpp>
#include <gazebo_ros/utils.hpp>
#include <image_transport/image_transport.h>
#include <sensor_msgs/fill_image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <algorithm>
#include <limits>
#include <memory>
#include <mutex>
#include <string>

// Not installed on Dashing because it will be removed on Eloquent
#include "gazebo_ros_depth_camera.hpp"

#define FLOAT_SIZE sizeof(float)

namespace gazebo_plugins
{
class GazeboRosDepthCameraPrivate
{
public:
  /// A pointer to the GazeboROS node.
  gazebo_ros::Node::SharedPtr ros_node_{nullptr};

  /// Image publisher.
  image_transport::Publisher image_pub_;

  /// Camera info publisher.
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_{nullptr};

  /// Depth image publisher.
  image_transport::Publisher depth_image_pub_;

  /// Depth camera info publisher.
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr depth_camera_info_pub_{nullptr};

  /// Point cloud publisher.
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;

  /// Trigger subscriber, in case it's a triggered camera
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr trigger_sub_{nullptr};

  /// Camera info manager
  std::shared_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;

  /// Image encoding
  std::string type_;

  /// Camera name, to be used on topics.
  std::string camera_name_;

  /// Frame name, to be used by TF.
  std::string frame_name_;

  /// Step size
  int skip_;

  /// Connects to pre-render events.
  gazebo::event::ConnectionPtr pre_render_connection_;

  /// Keeps track of how many times the camera has been triggered since it last published an image.
  int triggered{0};

  /// Protects trigger.
  std::mutex trigger_mutex_;

  /// Lock for image message
  std::mutex image_mutex_;

  /// Store current camera image.
  sensor_msgs::msg::Image image_msg_;

  /// Store current point cloud.
  sensor_msgs::msg::PointCloud2 cloud_msg_;

  /// Horizontal FOV of camera
  double hfov_;

  /// Min valid depth
  double min_depth_;

  /// Max valid depth
  double max_depth_;
};

GazeboRosDepthCamera::GazeboRosDepthCamera()
: impl_(std::make_unique<GazeboRosDepthCameraPrivate>())
{
}

GazeboRosDepthCamera::~GazeboRosDepthCamera()
{
  impl_->image_pub_.shutdown();
  impl_->ros_node_.reset();
}

void GazeboRosDepthCamera::Load(gazebo::sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
  gazebo::DepthCameraPlugin::Load(_sensor, _sdf);

  // Camera name
  impl_->camera_name_ = _sdf->Get<std::string>("camera_name", _sensor->Name()).first;

  // Get tf frame for output
  impl_->frame_name_ = gazebo_ros::SensorFrameID(*_sensor, *_sdf);

  // Initialize ROS node
  impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);

  // Image publisher
  impl_->image_pub_ = image_transport::create_publisher(impl_->ros_node_.get(),
      impl_->camera_name_ + "/image_raw");

//  RCLCPP_INFO(impl_->ros_node_->get_logger(), "Publishing images to [%s]",
//    impl_->image_pub_.getTopic().c_str());

  // Camera info publisher
  impl_->camera_info_pub_ = impl_->ros_node_->create_publisher<sensor_msgs::msg::CameraInfo>(
    impl_->camera_name_ + "/camera_info_raw", rclcpp::SensorDataQoS());

  RCLCPP_INFO(impl_->ros_node_->get_logger(), "Publishing camera info to [%s]",
    impl_->camera_info_pub_->get_topic_name());

  // Depth image publisher
  impl_->depth_image_pub_ = image_transport::create_publisher(impl_->ros_node_.get(),
      impl_->camera_name_ + "/depth/image_raw");

//  RCLCPP_INFO(impl_->ros_node_->get_logger(), "Publishing depth images to [%s]",
//              impl_->depth_image_pub_.getTopic().c_str());

  // Depth info publisher
  impl_->depth_camera_info_pub_ = impl_->ros_node_->create_publisher<sensor_msgs::msg::CameraInfo>(
    impl_->camera_name_ + "/depth/camera_info", rclcpp::QoS(rclcpp::KeepLast(1)));

  RCLCPP_INFO(impl_->ros_node_->get_logger(), "Publishing depth camera info to [%s]",
    impl_->depth_camera_info_pub_->get_topic_name());

  // Point cloud publisher
  impl_->point_cloud_pub_ = impl_->ros_node_->create_publisher<sensor_msgs::msg::PointCloud2>(
    impl_->camera_name_ + "/points", rclcpp::QoS(rclcpp::KeepLast(1)));

  RCLCPP_INFO(impl_->ros_node_->get_logger(), "Publishing pointcloud to [%s]",
    impl_->point_cloud_pub_->get_topic_name());

  // Trigger
  if (_sdf->Get<bool>("triggered", false).first) {
    impl_->trigger_sub_ = impl_->ros_node_->create_subscription<std_msgs::msg::Empty>(
      impl_->camera_name_ + "/image_trigger", rclcpp::QoS(rclcpp::KeepLast(1)),
      std::bind(&GazeboRosDepthCamera::OnTrigger, this, std::placeholders::_1));

    RCLCPP_INFO(impl_->ros_node_->get_logger(), "Subscribed to [%s]",
      impl_->trigger_sub_->get_topic_name());

    SetCameraEnabled(false);
    impl_->pre_render_connection_ = gazebo::event::Events::ConnectPreRender(
      std::bind(&GazeboRosDepthCamera::PreRender, this));
  }

  // Buffer size
  if (format == "L8" || format == "L_INT8") {
    impl_->type_ = sensor_msgs::image_encodings::MONO8;
    impl_->skip_ = 1;
  } else if (format == "L16" || format == "L_INT16") {
    impl_->type_ = sensor_msgs::image_encodings::MONO16;
    impl_->skip_ = 2;
  } else if (format == "R8G8B8" || format == "RGB_INT8") {
    impl_->type_ = sensor_msgs::image_encodings::RGB8;
    impl_->skip_ = 3;
  } else if (format == "B8G8R8" || format == "BGR_INT8") {
    impl_->type_ = sensor_msgs::image_encodings::BGR8;
    impl_->skip_ = 3;
  } else if (format == "R16G16B16" || format == "RGB_INT16") {
    impl_->type_ = sensor_msgs::image_encodings::RGB16;
    impl_->skip_ = 6;
  } else if (format == "BAYER_RGGB8") {
    RCLCPP_INFO(impl_->ros_node_->get_logger(),
      "bayer simulation may be computationally expensive.");
    impl_->type_ = sensor_msgs::image_encodings::BAYER_RGGB8;
    impl_->skip_ = 1;
  } else if (format == "BAYER_BGGR8") {
    RCLCPP_INFO(impl_->ros_node_->get_logger(),
      "bayer simulation may be computationally expensive.");
    impl_->type_ = sensor_msgs::image_encodings::BAYER_BGGR8;
    impl_->skip_ = 1;
  } else if (format == "BAYER_GBRG8") {
    RCLCPP_INFO(impl_->ros_node_->get_logger(),
      "bayer simulation may be computationally expensive.");
    impl_->type_ = sensor_msgs::image_encodings::BAYER_GBRG8;
    impl_->skip_ = 1;
  } else if (format == "BAYER_GRBG8") {
    RCLCPP_INFO(impl_->ros_node_->get_logger(),
      "bayer simulation may be computationally expensive.");
    impl_->type_ = sensor_msgs::image_encodings::BAYER_GRBG8;
    impl_->skip_ = 1;
  } else {
    RCLCPP_ERROR(impl_->ros_node_->get_logger(), "Unsupported Gazebo ImageFormat, using BGR8\n");
    impl_->type_ = sensor_msgs::image_encodings::BGR8;
    impl_->skip_ = 3;
  }

  // C parameters
  auto default_cx = (static_cast<double>(width) + 1.0) / 2.0;
  auto cx = _sdf->Get<double>("cx", default_cx).first;

  auto default_cy = (static_cast<double>(height) + 1.0) / 2.0;
  auto cy = _sdf->Get<double>("cy", default_cy).first;

  impl_->hfov_ = depthCamera->HFOV().Radian();
  double computed_focal_length = (static_cast<double>(width)) / (2.0 * tan(impl_->hfov_ / 2.0));

  // Focal length
  auto focal_length = _sdf->Get<double>("focal_length", 0.0).first;
  if (focal_length == 0) {
    focal_length = computed_focal_length;
  } else if (!ignition::math::equal(focal_length, computed_focal_length)) {
    RCLCPP_WARN(impl_->ros_node_->get_logger(),
      "The <focal_length> [%f] you have provided for camera [%s]"
      " is inconsistent with specified <image_width> [%d] and"
      " HFOV [%f]. Please double check to see that"
      " focal_length = width / (2.0 * tan(HFOV/2.0))."
      " The expected focal_length value is [%f],"
      " please update your camera model description accordingly.",
      focal_length, _sensor->Name().c_str(), width, impl_->hfov_, computed_focal_length);
  }

  // CameraInfo
  sensor_msgs::msg::CameraInfo camera_info_msg;
  camera_info_msg.header.frame_id = impl_->frame_name_;
  camera_info_msg.height = height;
  camera_info_msg.width = width;
  camera_info_msg.distortion_model = "plumb_bob";
  camera_info_msg.d.resize(5);

  // Allow the user to disable automatic cropping (used to remove barrel
  // distortion black border. The crop can be useful, but also skewes
  // the lens distortion, making the supplied k and t values incorrect.
  auto border_crop = _sdf->Get<bool>("border_crop", true).first;
  auto hack_baseline = _sdf->Get<double>("hack_baseline", 0.0).first;

  // Get distortion from camera
  double distortion_k1{0.0};
  double distortion_k2{0.0};
  double distortion_k3{0.0};
  double distortion_t1{0.0};
  double distortion_t2{0.0};
  if (depthCamera->LensDistortion()) {
    depthCamera->LensDistortion()->SetCrop(border_crop);

    distortion_k1 = depthCamera->LensDistortion()->K1();
    distortion_k2 = depthCamera->LensDistortion()->K2();
    distortion_k3 = depthCamera->LensDistortion()->K3();
    distortion_t1 = depthCamera->LensDistortion()->P1();
    distortion_t2 = depthCamera->LensDistortion()->P2();
  }

  // D = {k1, k2, t1, t2, k3}, as specified in:
  // - sensor_msgs/CameraInfo: http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html
  // - OpenCV: http://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
  camera_info_msg.d[0] = distortion_k1;
  camera_info_msg.d[1] = distortion_k2;
  camera_info_msg.d[2] = distortion_t1;
  camera_info_msg.d[3] = distortion_t2;
  camera_info_msg.d[4] = distortion_k3;

  // Original camera matrix
  camera_info_msg.k[0] = focal_length;
  camera_info_msg.k[1] = 0.0;
  camera_info_msg.k[2] = cx;
  camera_info_msg.k[3] = 0.0;
  camera_info_msg.k[4] = focal_length;
  camera_info_msg.k[5] = cy;
  camera_info_msg.k[6] = 0.0;
  camera_info_msg.k[7] = 0.0;
  camera_info_msg.k[8] = 1.0;

  // rectification
  camera_info_msg.r[0] = 1.0;
  camera_info_msg.r[1] = 0.0;
  camera_info_msg.r[2] = 0.0;
  camera_info_msg.r[3] = 0.0;
  camera_info_msg.r[4] = 1.0;
  camera_info_msg.r[5] = 0.0;
  camera_info_msg.r[6] = 0.0;
  camera_info_msg.r[7] = 0.0;
  camera_info_msg.r[8] = 1.0;

  // camera_ projection matrix (same as camera_ matrix due
  // to lack of distortion/rectification) (is this generated?)
  camera_info_msg.p[0] = focal_length;
  camera_info_msg.p[1] = 0.0;
  camera_info_msg.p[2] = cx;
  camera_info_msg.p[3] = -focal_length * hack_baseline;
  camera_info_msg.p[4] = 0.0;
  camera_info_msg.p[5] = focal_length;
  camera_info_msg.p[6] = cy;
  camera_info_msg.p[7] = 0.0;
  camera_info_msg.p[8] = 0.0;
  camera_info_msg.p[9] = 0.0;
  camera_info_msg.p[10] = 1.0;
  camera_info_msg.p[11] = 0.0;

  // Initialize camera_info_manager
  impl_->camera_info_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(
    impl_->ros_node_.get(), impl_->camera_name_);
  impl_->camera_info_manager_->setCameraInfo(camera_info_msg);

  impl_->min_depth_ = _sdf->Get<double>("min_depth", 0.4).first;
  impl_->max_depth_ = _sdf->Get<double>("max_depth", std::numeric_limits<float>::infinity()).first;
}

////////////////////////////////////////////////////////////////////////////////
void GazeboRosDepthCamera::OnNewImageFrame(
  const unsigned char * _image,
  unsigned int _width,
  unsigned int _height,
  unsigned int /*_depth*/,
  const std::string & /*_format*/)
{
  // TODO(shivesh) Enable / disable sensor once SubscriberStatusCallback has been ported to ROS2

  auto sensor_update_time = parentSensor->LastMeasurementTime();

  // Publish camera info
  auto camera_info_msg = impl_->camera_info_manager_->getCameraInfo();
  camera_info_msg.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(
    sensor_update_time);

  impl_->camera_info_pub_->publish(camera_info_msg);

  std::lock_guard<std::mutex> image_lock(impl_->image_mutex_);

  // Publish image
  impl_->image_msg_.header.frame_id = impl_->frame_name_;
  impl_->image_msg_.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(
    sensor_update_time);

  // Copy from src to image_msg
  sensor_msgs::fillImage(impl_->image_msg_, impl_->type_, _height, _width,
    impl_->skip_ * _width, reinterpret_cast<const void *>(_image));

  impl_->image_pub_.publish(impl_->image_msg_);

  // Disable camera if it's a triggered camera
  if (nullptr != impl_->trigger_sub_) {
    SetCameraEnabled(false);
    std::lock_guard<std::mutex> lock(impl_->trigger_mutex_);
    impl_->triggered = std::max(impl_->triggered - 1, 0);
  }
}

////////////////////////////////////////////////////////////////////////////////
void GazeboRosDepthCamera::OnNewDepthFrame(
  const float * _image,
  unsigned int _width,
  unsigned int _height,
  unsigned int /*_depth*/,
  const std::string & /*_format*/)
{
  // TODO(shivesh) Enable / disable sensor once SubscriberStatusCallback has been ported to ROS2

  auto sensor_update_time =
    gazebo_ros::Convert<builtin_interfaces::msg::Time>(parentSensor->LastMeasurementTime());

  // Publish depth image
  sensor_msgs::msg::Image image_msg;
  image_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  image_msg.header.frame_id = impl_->frame_name_;
  image_msg.header.stamp = sensor_update_time;
  image_msg.width = _width;
  image_msg.height = _height;
  image_msg.step = FLOAT_SIZE * _width;
  image_msg.data.resize(_width * _height * FLOAT_SIZE);
  image_msg.is_bigendian = 0;

  float pos_inf = std::numeric_limits<float>::infinity();
  float neg_inf = -pos_inf;

  // Copy from src to image_msg
  for (uint32_t j = 0; j < _height; j++) {
    for (uint32_t i = 0; i < _width; i++) {
      int index = i + j * _width;
      float depth = _image[index];
      if (impl_->min_depth_ < depth && depth < impl_->max_depth_) {
        std::memcpy(&image_msg.data[index * FLOAT_SIZE], &depth, FLOAT_SIZE);
      } else if (depth <= impl_->min_depth_) {
        std::memcpy(&image_msg.data[index * FLOAT_SIZE], &neg_inf, FLOAT_SIZE);
      } else {
        std::memcpy(&image_msg.data[index * FLOAT_SIZE], &pos_inf, FLOAT_SIZE);
      }
    }
  }

  impl_->depth_image_pub_.publish(image_msg);

  // Publish camera info
  auto camera_info_msg = impl_->camera_info_manager_->getCameraInfo();
  camera_info_msg.header.stamp = sensor_update_time;

  impl_->depth_camera_info_pub_->publish(camera_info_msg);

  // Publish point cloud
  impl_->cloud_msg_.header.frame_id = impl_->frame_name_;
  impl_->cloud_msg_.header.stamp = sensor_update_time;
  impl_->cloud_msg_.width = _width;
  impl_->cloud_msg_.height = _height;
  impl_->cloud_msg_.row_step = impl_->cloud_msg_.point_step * _width * _height;

  sensor_msgs::PointCloud2Modifier cloud_modifier(impl_->cloud_msg_);
  cloud_modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
  cloud_modifier.resize(_width * _height);

  impl_->cloud_msg_.is_dense = true;

  int cloud_index = 0;

  double fl = (static_cast<double>(_width)) / (2.0 * tan(impl_->hfov_ / 2.0));

  std::lock_guard<std::mutex> image_lock(impl_->image_mutex_);

  for (uint32_t j = 0; j < _height; j++) {
    double pAngle;
    if (_height > 1) {
      pAngle = atan2(static_cast<double>(j) - 0.5 * static_cast<double>(_height - 1), fl);
    } else {
      pAngle = 0.0;
    }

    for (uint32_t i = 0; i < _width; ++i) {
      double yAngle;
      if (_width > 1) {
        yAngle = atan2(static_cast<double>(i) - 0.5 * static_cast<double>(_width - 1), fl);
      } else {
        yAngle = 0.0;
      }

      // in optical frame
      // hardcoded rotation rpy(-M_PI/2, 0, -M_PI/2) is built-in
      // to urdf, where the *_optical_frame should have above relative
      // rotation from the physical camera *_frame

      int index = i + j * _width;
      float depth = _image[index];

      if (depth > impl_->min_depth_ && depth < impl_->max_depth_) {
        auto x = static_cast<float>(depth * tan(yAngle));
        auto y = static_cast<float>(depth * tan(pAngle));

        std::memcpy(&impl_->cloud_msg_.data[cloud_index], &x, FLOAT_SIZE);
        std::memcpy(&impl_->cloud_msg_.data[cloud_index + 4], &y, FLOAT_SIZE);
        std::memcpy(&impl_->cloud_msg_.data[cloud_index + 8], &depth, FLOAT_SIZE);
      } else if (depth <= impl_->min_depth_) {
        // point before valid range
        std::memcpy(&impl_->cloud_msg_.data[cloud_index], &neg_inf, FLOAT_SIZE);
        std::memcpy(&impl_->cloud_msg_.data[cloud_index + 4], &neg_inf, FLOAT_SIZE);
        std::memcpy(&impl_->cloud_msg_.data[cloud_index + 8], &neg_inf, FLOAT_SIZE);
        impl_->cloud_msg_.is_dense = false;
      } else {
        // point after valid range
        std::memcpy(&impl_->cloud_msg_.data[cloud_index], &pos_inf, FLOAT_SIZE);
        std::memcpy(&impl_->cloud_msg_.data[cloud_index + 4], &pos_inf, FLOAT_SIZE);
        std::memcpy(&impl_->cloud_msg_.data[cloud_index + 8], &pos_inf, FLOAT_SIZE);
        impl_->cloud_msg_.is_dense = false;
      }

      if (impl_->image_msg_.data.size() == _width * _height * 3) {
        // color
        std::memcpy(&impl_->cloud_msg_.data[cloud_index + 16],
          &impl_->image_msg_.data[index * 3], 3 * sizeof(uint8_t));
      } else if (impl_->image_msg_.data.size() == _height * _width) {
        std::memcpy(&impl_->cloud_msg_.data[cloud_index + 16],
          &impl_->image_msg_.data[index], 3 * sizeof(uint8_t));
      } else {
        // no image
        std::memset(&impl_->cloud_msg_.data[cloud_index + 16], 0, 3 * sizeof(uint8_t));
      }
      cloud_index += impl_->cloud_msg_.point_step;
    }
  }

  impl_->point_cloud_pub_->publish(impl_->cloud_msg_);

  // Disable camera if it's a triggered camera
  if (nullptr != impl_->trigger_sub_) {
    SetCameraEnabled(false);
    std::lock_guard<std::mutex> lock(impl_->trigger_mutex_);
    impl_->triggered = std::max(impl_->triggered - 1, 0);
  }
}

void GazeboRosDepthCamera::SetCameraEnabled(const bool _enabled)
{
  parentSensor->SetActive(_enabled);
  parentSensor->SetUpdateRate(_enabled ? 0.0 : ignition::math::MIN_D);
}

void GazeboRosDepthCamera::PreRender()
{
  std::lock_guard<std::mutex> lock(impl_->trigger_mutex_);
  if (impl_->triggered > 0) {
    SetCameraEnabled(true);
  }
}

void GazeboRosDepthCamera::OnTrigger(const std_msgs::msg::Empty::SharedPtr)
{
  std::lock_guard<std::mutex> lock(impl_->trigger_mutex_);
  impl_->triggered++;
}

GZ_REGISTER_SENSOR_PLUGIN(GazeboRosDepthCamera)
}  // namespace gazebo_plugins
