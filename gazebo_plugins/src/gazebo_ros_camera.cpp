// Copyright 2013 Open Source Robotics Foundation
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
#include <ignition/math/Helpers.hh>

#include <camera_info_manager/camera_info_manager.h>
#include <gazebo_plugins/gazebo_ros_camera.hpp>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/node.hpp>
#include <gazebo_ros/utils.hpp>
#include <image_transport/image_transport.h>
#include <sensor_msgs/fill_image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <mutex>

namespace gazebo_plugins
{
class GazeboRosCameraPrivate
{
public:
  /// A pointer to the GazeboROS node.
  gazebo_ros::Node::SharedPtr ros_node_{nullptr};

  /// Image publisher.
  image_transport::Publisher image_pub_;

  /// Camera info publisher.
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_{nullptr};

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

  /// Keeps track of how many times the camera has been triggered since it last published an
  /// image.
  int triggered{0};

  /// Protects trigger.
  std::mutex trigger_mutex_;
};

GazeboRosCamera::GazeboRosCamera()
: impl_(std::make_unique<GazeboRosCameraPrivate>())
{
}

GazeboRosCamera::~GazeboRosCamera()
{
  impl_->image_pub_.shutdown();
  impl_->camera_info_pub_.reset();
  impl_->ros_node_.reset();
}

void GazeboRosCamera::Load(gazebo::sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
  gazebo::CameraPlugin::Load(_sensor, _sdf);

  // Camera name
  impl_->camera_name_ = _sdf->Get<std::string>("camera_name", _sensor->Name()).first;

  // Get tf frame for output
  impl_->frame_name_ = gazebo_ros::SensorFrameID(*_sensor, *_sdf);

  // Initialize ROS node
  impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);

  // Image publisher
  // TODO(louise) Migrate image_connect logic once SubscriberStatusCallback is ported to ROS2
  impl_->image_pub_ = image_transport::create_publisher(impl_->ros_node_,
    impl_->camera_name_ + "/image_raw");

  // TODO(louise) Uncomment this once image_transport::Publisher has a function to return the
  // full topic.
  // RCLCPP_INFO(impl_->ros_node_->get_logger(), "Publishing images to [%s]",
  //  impl_->image_pub_.getTopic());

  // Camera info publisher
  // TODO(louise) Migrate ImageConnect logic once SubscriberStatusCallback is ported to ROS2
  impl_->camera_info_pub_ = impl_->ros_node_->create_publisher<sensor_msgs::msg::CameraInfo>(
    impl_->camera_name_ + "/camera_info");

  RCLCPP_INFO(impl_->ros_node_->get_logger(), "Publishing camera info to [%s]",
    impl_->camera_info_pub_->get_topic_name());

  // Trigger
  if (_sdf->Get<bool>("triggered", false).first) {
   rmw_qos_profile_t qos_profile = rmw_qos_profile_default;
   qos_profile.depth = 1;

   impl_->trigger_sub_ = impl_->ros_node_->create_subscription<std_msgs::msg::Empty>(
     impl_->camera_name_ + "/image_trigger",
     std::bind(&GazeboRosCamera::OnTrigger, this, std::placeholders::_1),
     qos_profile);

   RCLCPP_INFO(impl_->ros_node_->get_logger(), "Subscribed to [%s]",
     impl_->trigger_sub_->get_topic_name());

   SetCameraEnabled(false);
   impl_->pre_render_connection_ = gazebo::event::Events::ConnectPreRender(
     std::bind(&GazeboRosCamera::PreRender, this));
 }

  // Dynamic reconfigure
//    dyn_srv_ =
//      new dynamic_reconfigure::Server<gazebo_plugins::GazeboRosCameraConfig>
//      (*this->rosnode_);
//    dynamic_reconfigure::Server<gazebo_plugins::GazeboRosCameraConfig>
//      ::CallbackType f =
//      boost::bind(&GazeboRosCameraUtils::configCallback, this, _1, _2);
//    dyn_srv_->setCallback(f);
//  }

  // Buffer size
  if (this->format == "L8" || this->format == "L_INT8")
  {
    impl_->type_ = sensor_msgs::image_encodings::MONO8;
    impl_->skip_ = 1;
  }
  else if (this->format == "L16" || this->format == "L_INT16")
  {
    impl_->type_ = sensor_msgs::image_encodings::MONO16;
    impl_->skip_ = 2;
  }
  else if (this->format == "R8G8B8" || this->format == "RGB_INT8")
  {
    impl_->type_ = sensor_msgs::image_encodings::RGB8;
    impl_->skip_ = 3;
  }
  else if (this->format == "B8G8R8" || this->format == "BGR_INT8")
  {
    impl_->type_ = sensor_msgs::image_encodings::BGR8;
    impl_->skip_ = 3;
  }
  else if (this->format == "R16G16B16" ||  this->format == "RGB_INT16")
  {
    impl_->type_ = sensor_msgs::image_encodings::RGB16;
    impl_->skip_ = 6;
  }
  else if (this->format == "BAYER_RGGB8")
  {
    RCLCPP_INFO(impl_->ros_node_->get_logger(),
      "bayer simulation may be computationally expensive.");
    impl_->type_ = sensor_msgs::image_encodings::BAYER_RGGB8;
    impl_->skip_ = 1;
  }
  else if (this->format == "BAYER_BGGR8")
  {
    RCLCPP_INFO(impl_->ros_node_->get_logger(),
      "bayer simulation may be computationally expensive.");
    impl_->type_ = sensor_msgs::image_encodings::BAYER_BGGR8;
    impl_->skip_ = 1;
  }
  else if (this->format == "BAYER_GBRG8")
  {
    RCLCPP_INFO(impl_->ros_node_->get_logger(),
      "bayer simulation may be computationally expensive.");
    impl_->type_ = sensor_msgs::image_encodings::BAYER_GBRG8;
    impl_->skip_ = 1;
  }
  else if (this->format == "BAYER_GRBG8")
  {
    RCLCPP_INFO(impl_->ros_node_->get_logger(),
      "bayer simulation may be computationally expensive.");
    impl_->type_ = sensor_msgs::image_encodings::BAYER_GRBG8;
    impl_->skip_ = 1;
  }
  else
  {
    RCLCPP_ERROR(impl_->ros_node_->get_logger(), "Unsupported Gazebo ImageFormat, using BGR8\n");
    impl_->type_ = sensor_msgs::image_encodings::BGR8;
    impl_->skip_ = 3;
  }

  // C parameters
  auto default_cx = (static_cast<double>(this->width) + 1.0) /2.0;
  auto cx = _sdf->Get<double>("cx", default_cx).first;

  auto default_cy = (static_cast<double>(this->height) + 1.0) /2.0;
  auto cy = _sdf->Get<double>("cy", default_cy).first;

  double hfov = this->camera->HFOV().Radian();
  double computed_focal_length = (static_cast<double>(this->width)) / (2.0 * tan(hfov / 2.0));

  // Focal length
  auto focal_length = _sdf->Get<double>("focal_length", 0.0).first;
  if (focal_length == 0)
  {
    focal_length = computed_focal_length;
  }
  else
  {
    if (!ignition::math::equal(focal_length, computed_focal_length))
    {
      RCLCPP_WARN(impl_->ros_node_->get_logger(),
        "The <focal_length> [%f] you have provided for camera [%s]"
        " is inconsistent with specified <image_width> [%d] and"
        " HFOV [%f]. Please double check to see that"
        " focal_length = width / (2.0 * tan(HFOV/2.0))."
        " The expected focal_length value is [%f],"
        " please update your camera model description accordingly.",
         focal_length, _sensor->Name().c_str(), this->width, hfov, computed_focal_length);
    }
  }

  // CameraInfo
  sensor_msgs::msg::CameraInfo camera_info_msg;
  camera_info_msg.header.frame_id = impl_->frame_name_;
  camera_info_msg.height = this->height;
  camera_info_msg.width  = this->width;
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
  if (this->camera->LensDistortion())
  {
    this->camera->LensDistortion()->SetCrop(border_crop);

    distortion_k1 = this->camera->LensDistortion()->K1();
    distortion_k2 = this->camera->LensDistortion()->K2();
    distortion_k3 = this->camera->LensDistortion()->K3();
    distortion_t1 = this->camera->LensDistortion()->P1();
    distortion_t2 = this->camera->LensDistortion()->P2();
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
          impl_->ros_node_, impl_->camera_name_);
  impl_->camera_info_manager_->setCameraInfo(camera_info_msg);
}

////////////////////////////////////////////////////////////////////////////////
void GazeboRosCamera::OnNewFrame(
    const unsigned char *_image,
    unsigned int _width,
    unsigned int _height,
    unsigned int /*_depth*/,
    const std::string & /*_format*/)
{
  // TODO(louise) Enable / disable sensor once SubscriberStatusCallback has been ported to ROS2

  auto sensor_update_time = this->parentSensor->LastMeasurementTime();

  // Publish image
  sensor_msgs::msg::Image image_msg;
  image_msg.header.frame_id = impl_->frame_name_;
  image_msg.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(sensor_update_time);

  // Copy from src to image_msg
  sensor_msgs::fillImage(image_msg, impl_->type_, _height, _width,
      impl_->skip_ * _width, reinterpret_cast<const void*>(_image));

  impl_->image_pub_.publish(image_msg);

  // Publish camera info
  auto camera_info_msg = impl_->camera_info_manager_->getCameraInfo();
  camera_info_msg.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(
      sensor_update_time);

  impl_->camera_info_pub_->publish(camera_info_msg);

  // Disable camera if it's a triggered camera
  if (nullptr != impl_->trigger_sub_) {
    SetCameraEnabled(false);

    std::lock_guard<std::mutex> lock(impl_->trigger_mutex_);
    impl_->triggered = std::max(impl_->triggered-1, 0);
  }
}

void GazeboRosCamera::SetCameraEnabled(const bool _enabled)
{
  this->parentSensor->SetActive(_enabled);
  this->parentSensor->SetUpdateRate(_enabled ? 0.0 : ignition::math::MIN_D);
}

void GazeboRosCamera::PreRender()
{
  std::lock_guard<std::mutex> lock(impl_->trigger_mutex_);
  if (impl_->triggered > 0) {
    SetCameraEnabled(true);
  }
}

void GazeboRosCamera::OnTrigger(const std_msgs::msg::Empty::SharedPtr)
{
  std::lock_guard<std::mutex> lock(impl_->trigger_mutex_);
  impl_->triggered++;
}

GZ_REGISTER_SENSOR_PLUGIN(GazeboRosCamera)
}  // namespace gazebo_plugins
