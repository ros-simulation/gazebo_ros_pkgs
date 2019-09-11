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

#include <algorithm>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

// Not installed on Dashing because it will be removed on Eloquent
#include "gazebo_ros_multi_camera.hpp"

namespace gazebo_plugins
{
class GazeboRosMultiCameraPrivate
{
public:
  /// A pointer to the GazeboROS node.
  gazebo_ros::Node::SharedPtr ros_node_{nullptr};

  /// Image publishers.
  std::vector<image_transport::Publisher> image_pub_;

  /// Camera info publishers.
  std::vector<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr> camera_info_pub_;

  /// Trigger subscriber, in case it's a triggered camera
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr trigger_sub_{nullptr};

  /// Camera info managers
  std::vector<std::shared_ptr<camera_info_manager::CameraInfoManager>> camera_info_manager_;

  /// Image encodings
  std::vector<std::string> img_encoding_;

  /// Frame name, to be used by TF.
  std::string frame_name_;

  /// Step sizes for fillImage
  std::vector<uint32_t> img_step_;

  /// Connects to pre-render events.
  gazebo::event::ConnectionPtr pre_render_connection_;

  /// Keeps track of how many times the camera has been triggered since it last published an image.
  int triggered{0};

  /// Protects trigger.
  std::mutex trigger_mutex_;
};

GazeboRosMultiCamera::GazeboRosMultiCamera()
: impl_(std::make_unique<GazeboRosMultiCameraPrivate>())
{
}

GazeboRosMultiCamera::~GazeboRosMultiCamera()
{
  for (auto pub : impl_->image_pub_) {
    pub.shutdown();
  }
}

void GazeboRosMultiCamera::Load(gazebo::sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
  MultiCameraPlugin::Load(_sensor, _sdf);

  // Camera name
  auto camera_name = _sdf->Get<std::string>("camera_name", _sensor->Name()).first;

  // Get tf frame for output
  impl_->frame_name_ = gazebo_ros::SensorFrameID(*_sensor, *_sdf);

  // Initialize ROS node
  impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);

  auto num_cameras = MultiCameraPlugin::parent_sensor_->CameraCount();

  for (uint64_t i = 0; i < num_cameras; ++i) {
    // Image publisher
    impl_->image_pub_.push_back(image_transport::create_publisher(impl_->ros_node_.get(),
      camera_name + "/" + MultiCameraPlugin::camera_[i]->Name() + "/image_raw"));

    // RCLCPP_INFO(impl_->ros_node_->get_logger(), "Publishing %s camera images to [%s]",
    //   MultiCameraPlugin::camera_[i]->Name().c_str(),
    //   impl_->image_pub_.back().getTopic());

    // Camera info publisher
    impl_->camera_info_pub_.push_back(
      impl_->ros_node_->create_publisher<sensor_msgs::msg::CameraInfo>(
        camera_name + "/" + MultiCameraPlugin::camera_[i]->Name() + "/camera_info",
        rclcpp::SensorDataQoS()));

    RCLCPP_INFO(impl_->ros_node_->get_logger(), "Publishing %s camera info to [%s]",
      MultiCameraPlugin::camera_[i]->Name().c_str(),
      impl_->camera_info_pub_[i]->get_topic_name());
  }

  // Trigger
  if (_sdf->Get<bool>("triggered", false).first) {
    impl_->trigger_sub_ = impl_->ros_node_->create_subscription<std_msgs::msg::Empty>(
      camera_name + "/image_trigger",
      rclcpp::QoS(rclcpp::KeepLast(1)),
      std::bind(&GazeboRosMultiCamera::OnTrigger, this, std::placeholders::_1));

    RCLCPP_INFO(impl_->ros_node_->get_logger(), "Subscribed to [%s]",
      impl_->trigger_sub_->get_topic_name());

    SetCameraEnabled(false);
    impl_->pre_render_connection_ = gazebo::event::Events::ConnectPreRender(
      std::bind(&GazeboRosMultiCamera::PreRender, this));
  }

  auto image_format = MultiCameraPlugin::format_;

  for (const auto & format : image_format) {
    if (format == "L8" || format == "L_INT8") {
      impl_->img_encoding_.emplace_back(sensor_msgs::image_encodings::MONO8);
      impl_->img_step_.push_back(1);
    } else if (format == "L16" || format == "L_INT16") {
      impl_->img_encoding_.emplace_back(sensor_msgs::image_encodings::MONO16);
      impl_->img_step_.push_back(2);
    } else if (format == "R8G8B8" || format == "RGB_INT8") {
      impl_->img_encoding_.emplace_back(sensor_msgs::image_encodings::RGB8);
      impl_->img_step_.push_back(3);
    } else if (format == "B8G8R8" || format == "BGR_INT8") {
      impl_->img_encoding_.emplace_back(sensor_msgs::image_encodings::BGR8);
      impl_->img_step_.push_back(3);
    } else if (format == "R16G16B16" || format == "RGB_INT16") {
      impl_->img_encoding_.emplace_back(sensor_msgs::image_encodings::RGB16);
      impl_->img_step_.push_back(6);
    } else if (format == "BAYER_RGGB8") {
      RCLCPP_INFO(impl_->ros_node_->get_logger(),
        "bayer simulation may be computationally expensive.");
      impl_->img_encoding_.emplace_back(sensor_msgs::image_encodings::BAYER_RGGB8);
      impl_->img_step_.push_back(1);
    } else if (format == "BAYER_BGGR8") {
      RCLCPP_INFO(impl_->ros_node_->get_logger(),
        "bayer simulation may be computationally expensive.");
      impl_->img_encoding_.emplace_back(sensor_msgs::image_encodings::BAYER_BGGR8);
      impl_->img_step_.push_back(1);
    } else if (format == "BAYER_GBRG8") {
      RCLCPP_INFO(impl_->ros_node_->get_logger(),
        "bayer simulation may be computationally expensive.");
      impl_->img_encoding_.emplace_back(sensor_msgs::image_encodings::BAYER_GBRG8);
      impl_->img_step_.push_back(1);
    } else if (format == "BAYER_GRBG8") {
      RCLCPP_INFO(impl_->ros_node_->get_logger(),
        "bayer simulation may be computationally expensive.");
      impl_->img_encoding_.emplace_back(sensor_msgs::image_encodings::BAYER_GRBG8);
      impl_->img_step_.push_back(1);
    } else {
      RCLCPP_ERROR(impl_->ros_node_->get_logger(), "Unsupported Gazebo ImageFormat, using BGR8\n");
      impl_->img_encoding_.emplace_back(sensor_msgs::image_encodings::BGR8);
      impl_->img_step_.push_back(3);
    }
  }

  auto width = MultiCameraPlugin::width_;
  auto height = MultiCameraPlugin::height_;
  auto camera = MultiCameraPlugin::camera_;

  for (uint64_t i = 0; i < num_cameras; ++i) {
    // C parameters
    auto default_cx = (static_cast<double>(width[i]) + 1.0) / 2.0;
    auto cx = _sdf->Get<double>("cx", default_cx).first;

    auto default_cy = (static_cast<double>(height[i]) + 1.0) / 2.0;
    auto cy = _sdf->Get<double>("cy", default_cy).first;

    auto hfov = camera[i]->HFOV().Radian();

    double computed_focal_length =
      (static_cast<double>(width[i])) / (2.0 * tan(hfov / 2.0));

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
        focal_length, _sensor->Name().c_str(), width[i], hfov, computed_focal_length);
    }

    // CameraInfo
    sensor_msgs::msg::CameraInfo camera_info_msg;
    camera_info_msg.header.frame_id = impl_->frame_name_;
    camera_info_msg.height = height[i];
    camera_info_msg.width = width[i];
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
    if (camera[i]->LensDistortion()) {
      camera[i]->LensDistortion()->SetCrop(border_crop);

      distortion_k1 = camera[i]->LensDistortion()->K1();
      distortion_k2 = camera[i]->LensDistortion()->K2();
      distortion_k3 = camera[i]->LensDistortion()->K3();
      distortion_t1 = camera[i]->LensDistortion()->P1();
      distortion_t2 = camera[i]->LensDistortion()->P2();
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
    impl_->camera_info_manager_.push_back(std::make_shared<camera_info_manager::CameraInfoManager>(
        impl_->ros_node_.get(), camera_name));
    impl_->camera_info_manager_.back()->setCameraInfo(camera_info_msg);
  }

  impl_->ros_node_->declare_parameter(
    "update_rate", MultiCameraPlugin::parent_sensor_->UpdateRate());

  auto existing_callback = impl_->ros_node_->set_on_parameters_set_callback(nullptr);
  auto param_change_callback =
    [this, existing_callback](std::vector<rclcpp::Parameter> parameters) {
      auto result = rcl_interfaces::msg::SetParametersResult();
      if (nullptr != existing_callback) {
        result = existing_callback(parameters);
        if (!result.successful) {
          return result;
        }
      }

      result.successful = true;
      for (const auto & parameter : parameters) {
        std::string param_name = parameter.get_name();
        if (param_name == "update_rate") {
          if (nullptr != impl_->trigger_sub_) {
            RCLCPP_WARN(impl_->ros_node_->get_logger(),
              "Cannot set update rate for triggered camera");
            result.successful = false;
          } else {
            rclcpp::ParameterType parameter_type = parameter.get_type();
            if (rclcpp::ParameterType::PARAMETER_DOUBLE == parameter_type) {
              double rate = parameter.as_double();
              MultiCameraPlugin::parent_sensor_->SetUpdateRate(rate);

              if (rate >= 0.0) {
                RCLCPP_INFO(impl_->ros_node_->get_logger(),
                  "Camera update rate changed to [%.2f Hz]", rate);
              } else {
                RCLCPP_WARN(impl_->ros_node_->get_logger(),
                  "Camera update rate should be positive. Setting to maximum");
              }
            } else {
              RCLCPP_WARN(impl_->ros_node_->get_logger(),
                "Value for param [update_rate] has to be of double type.");
              result.successful = false;
            }
          }
        }
      }
      return result;
    };

  impl_->ros_node_->set_on_parameters_set_callback(param_change_callback);
}

////////////////////////////////////////////////////////////////////////////////
void GazeboRosMultiCamera::OnNewMultiFrame(
  const unsigned char * _image,
  unsigned int _width,
  unsigned int _height,
  unsigned int /*_depth*/,
  const std::string & /*_format*/,
  const int _camera_num)
{
  // TODO(shivesh) Enable / disable sensor once SubscriberStatusCallback has been ported to ROS2

  gazebo::common::Time sensor_update_time =
    MultiCameraPlugin::parent_sensor_->LastMeasurementTime();

  // Publish camera info
  auto camera_info_msg = impl_->camera_info_manager_[_camera_num]->getCameraInfo();
  camera_info_msg.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(
    sensor_update_time);

  impl_->camera_info_pub_[_camera_num]->publish(camera_info_msg);

  // Publish image
  auto image_msg = sensor_msgs::msg::Image();
  image_msg.header.frame_id = impl_->frame_name_;
  image_msg.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(sensor_update_time);

  // Copy from src to image_msg
  sensor_msgs::fillImage(image_msg, impl_->img_encoding_[_camera_num], _height, _width,
    impl_->img_step_[_camera_num] * _width, reinterpret_cast<const void *>(_image));

  impl_->image_pub_[_camera_num].publish(image_msg);

  // Disable camera if it's a triggered camera
  if (nullptr != impl_->trigger_sub_) {
    SetCameraEnabled(false);

    std::lock_guard<std::mutex> lock(impl_->trigger_mutex_);
    impl_->triggered = std::max(impl_->triggered - 1, 0);
  }
}

void GazeboRosMultiCamera::SetCameraEnabled(const bool _enabled)
{
  parent_sensor_->SetActive(_enabled);
  parent_sensor_->SetUpdateRate(_enabled ? 0.0 : ignition::math::MIN_D);
}

void GazeboRosMultiCamera::PreRender()
{
  std::lock_guard<std::mutex> lock(impl_->trigger_mutex_);
  if (impl_->triggered > 0) {
    SetCameraEnabled(true);
  }
}

void GazeboRosMultiCamera::OnTrigger(const std_msgs::msg::Empty::SharedPtr)
{
  std::lock_guard<std::mutex> lock(impl_->trigger_mutex_);
  impl_->triggered++;
}

GZ_REGISTER_SENSOR_PLUGIN(GazeboRosMultiCamera)
}  // namespace gazebo_plugins
