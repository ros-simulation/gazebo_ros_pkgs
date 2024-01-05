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
#include <gazebo/sensors/SensorTypes.hh>
#ifdef IGN_PROFILER_ENABLE
#include <ignition/common/Profiler.hh>
#endif
#include <ignition/math/Helpers.hh>

#include <camera_info_manager/camera_info_manager.hpp>
#include <gazebo_plugins/gazebo_ros_camera.hpp>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/node.hpp>
#include <gazebo_ros/utils.hpp>
#include <image_transport/image_transport.hpp>
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
#include <vector>

#define FLOAT_SIZE sizeof(float)

namespace gazebo_plugins
{
class GazeboRosCameraPrivate
{
public:
  /// Indicates type of camera
  enum SensorType
  {
    /// Depth Camera
    DEPTH,

    /// Normal RGB Camera
    CAMERA,

    /// Multi Camera
    MULTICAMERA
  };

  /// Depth, Normal or Multi Camera
  SensorType sensor_type_;

  /// A pointer to the GazeboROS node.
  gazebo_ros::Node::SharedPtr ros_node_{nullptr};

  /// Image publishers.
  std::vector<image_transport::Publisher> image_pub_;

  /// Camera info publishers.
  std::vector<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr> camera_info_pub_;

  /// Depth image publisher.
  image_transport::Publisher depth_image_pub_;

  /// Depth camera info publisher.
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr depth_camera_info_pub_{nullptr};

  /// Point cloud publisher.
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;

  /// Trigger subscriber, in case it's a triggered camera
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr trigger_sub_{nullptr};

  /// Camera info managers
  std::vector<std::shared_ptr<camera_info_manager::CameraInfoManager>> camera_info_manager_;

  /// Image encodings
  std::vector<std::string> img_encoding_;

  /// Camera name, to be used on topics.
  std::string camera_name_;

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

  /// Lock for image message
  std::mutex image_mutex_;

  /// Store current camera image.
  sensor_msgs::msg::Image image_msg_;

  /// Store current point cloud.
  sensor_msgs::msg::PointCloud2 cloud_msg_;

  /// Pointers to cameras
  std::vector<gazebo::rendering::CameraPtr> camera_;

  /// Horizontal FOV of cameras
  std::vector<double> hfov_;

  /// Min valid depth
  double min_depth_;

  /// Max valid depth
  double max_depth_;

  /// Number of cameras
  uint64_t num_cameras_{1};
};

GazeboRosCamera::GazeboRosCamera()
: impl_(std::make_unique<GazeboRosCameraPrivate>())
{
}

GazeboRosCamera::~GazeboRosCamera()
{
  for (auto pub : impl_->image_pub_) {
    pub.shutdown();
  }
  if (param_change_callback_handler_) {
    impl_->ros_node_->remove_on_set_parameters_callback(param_change_callback_handler_.get());
  }
  param_change_callback_handler_.reset();
}

void GazeboRosCamera::Load(gazebo::sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
  // Initialize ROS node
  impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);

  // Get QoS profiles
  const gazebo_ros::QoS & qos = impl_->ros_node_->get_qos();

  if (std::dynamic_pointer_cast<gazebo::sensors::MultiCameraSensor>(_sensor)) {
    impl_->sensor_type_ = GazeboRosCameraPrivate::MULTICAMERA;
    MultiCameraPlugin::Load(_sensor, _sdf);
    impl_->num_cameras_ = MultiCameraPlugin::parent_sensor_->CameraCount();
  } else if (std::dynamic_pointer_cast<gazebo::sensors::DepthCameraSensor>(_sensor)) {
    impl_->sensor_type_ = GazeboRosCameraPrivate::DEPTH;
    gazebo::DepthCameraPlugin::Load(_sensor, _sdf);
  } else if (std::dynamic_pointer_cast<gazebo::sensors::CameraSensor>(_sensor)) {
    impl_->sensor_type_ = GazeboRosCameraPrivate::CAMERA;
    gazebo::CameraPlugin::Load(_sensor, _sdf);
  } else {
    RCLCPP_ERROR(
      impl_->ros_node_->get_logger(),
      "Plugin must be attached to sensor of type `camera`, `depth` or `multicamera`");
    impl_->ros_node_.reset();
    return;
  }

  // Camera name
  impl_->camera_name_ = _sdf->Get<std::string>("camera_name", _sensor->Name()).first;

  // Get tf frame for output
  impl_->frame_name_ = gazebo_ros::SensorFrameID(*_sensor, *_sdf);

  if (impl_->sensor_type_ != GazeboRosCameraPrivate::MULTICAMERA) {
    // Image publisher
    // TODO(louise) Migrate image_connect logic once SubscriberStatusCallback is ported to ROS2
    const std::string camera_topic = impl_->camera_name_ + "/image_raw";
    impl_->image_pub_.push_back(
      image_transport::create_publisher(
        impl_->ros_node_.get(), camera_topic, qos.get_publisher_qos(
          camera_topic, rclcpp::SensorDataQoS().reliable()).get_rmw_qos_profile()));

    // TODO(louise) Uncomment this once image_transport::Publisher has a function to return the
    // full topic.
    // RCLCPP_INFO(
    //   impl_->ros_node_->get_logger(), "Publishing images to [%s]", impl_->image_pub_.getTopic());

    // Camera info publisher
    // TODO(louise) Migrate ImageConnect logic once SubscriberStatusCallback is ported to ROS2
    const std::string camera_info_topic = impl_->camera_name_ + "/camera_info";
    impl_->camera_info_pub_.push_back(
      impl_->ros_node_->create_publisher<sensor_msgs::msg::CameraInfo>(
        camera_info_topic, qos.get_publisher_qos(
          camera_info_topic, rclcpp::SensorDataQoS().reliable())));

    RCLCPP_INFO(
      impl_->ros_node_->get_logger(), "Publishing camera info to [%s]",
      impl_->camera_info_pub_.back()->get_topic_name());

  } else {
    for (uint64_t i = 0; i < impl_->num_cameras_; ++i) {
      const std::string camera_topic =
        impl_->camera_name_ + "/" + MultiCameraPlugin::camera_[i]->Name() + "/image_raw";
      // Image publisher
      impl_->image_pub_.push_back(
        image_transport::create_publisher(
          impl_->ros_node_.get(), camera_topic, qos.get_publisher_qos(
            camera_topic, rclcpp::SensorDataQoS().reliable()).get_rmw_qos_profile()));

      // RCLCPP_INFO(
      //   impl_->ros_node_->get_logger(), "Publishing %s camera images to [%s]",
      //   MultiCameraPlugin::camera_[i]->Name().c_str(),
      //   impl_->image_pub_.back().getTopic());

      const std::string camera_info_topic =
        impl_->camera_name_ + "/" + MultiCameraPlugin::camera_[i]->Name() + "/camera_info";
      // Camera info publisher
      impl_->camera_info_pub_.push_back(
        impl_->ros_node_->create_publisher<sensor_msgs::msg::CameraInfo>(
          camera_info_topic, qos.get_publisher_qos(
            camera_info_topic, rclcpp::SensorDataQoS().reliable())));

      RCLCPP_INFO(
        impl_->ros_node_->get_logger(), "Publishing %s camera info to [%s]",
        MultiCameraPlugin::camera_[i]->Name().c_str(),
        impl_->camera_info_pub_[i]->get_topic_name());
    }
  }

  if (impl_->sensor_type_ == GazeboRosCameraPrivate::DEPTH) {
    const std::string depth_topic = impl_->camera_name_ + "/depth/image_raw";
    // Depth image publisher
    impl_->depth_image_pub_ = image_transport::create_publisher(
      impl_->ros_node_.get(), depth_topic, qos.get_publisher_qos(
        depth_topic, rclcpp::SensorDataQoS().reliable()).get_rmw_qos_profile());

    // RCLCPP_INFO(impl_->ros_node_->get_logger(), "Publishing depth images to [%s]",
    //   impl_->depth_image_pub_.getTopic().c_str());

    const std::string depth_info_topic = impl_->camera_name_ + "/depth/camera_info";
    // Depth info publisher
    impl_->depth_camera_info_pub_ =
      impl_->ros_node_->create_publisher<sensor_msgs::msg::CameraInfo>(
      depth_info_topic, qos.get_publisher_qos(depth_info_topic, rclcpp::QoS(1)));

    RCLCPP_INFO(
      impl_->ros_node_->get_logger(), "Publishing depth camera info to [%s]",
      impl_->depth_camera_info_pub_->get_topic_name());

    const std::string point_cloud_topic = impl_->camera_name_ + "/points";
    // Point cloud publisher
    impl_->point_cloud_pub_ = impl_->ros_node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      point_cloud_topic, qos.get_publisher_qos(point_cloud_topic, rclcpp::QoS(1)));

    RCLCPP_INFO(
      impl_->ros_node_->get_logger(), "Publishing pointcloud to [%s]",
      impl_->point_cloud_pub_->get_topic_name());
  }

  // Trigger
  if (_sdf->Get<bool>("triggered", false).first) {
    const std::string trigger_topic = impl_->camera_name_ + "/image_trigger";
    impl_->trigger_sub_ = impl_->ros_node_->create_subscription<std_msgs::msg::Empty>(
      trigger_topic, qos.get_subscription_qos(trigger_topic, rclcpp::QoS(1)),
      std::bind(&GazeboRosCamera::OnTrigger, this, std::placeholders::_1));

    RCLCPP_INFO(
      impl_->ros_node_->get_logger(), "Subscribed to [%s]",
      impl_->trigger_sub_->get_topic_name());

    SetCameraEnabled(false);
    impl_->pre_render_connection_ = gazebo::event::Events::ConnectPreRender(
      std::bind(&GazeboRosCamera::PreRender, this));
  }

  // Buffer size
  std::vector<std::string> image_format;
  if (impl_->sensor_type_ == GazeboRosCameraPrivate::CAMERA) {
    image_format.push_back(CameraPlugin::format);
  } else if (impl_->sensor_type_ == GazeboRosCameraPrivate::DEPTH) {
    image_format.push_back(DepthCameraPlugin::format);
  } else {
    image_format = MultiCameraPlugin::format_;
  }

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
      RCLCPP_INFO(
        impl_->ros_node_->get_logger(),
        "bayer simulation may be computationally expensive.");
      impl_->img_encoding_.emplace_back(sensor_msgs::image_encodings::BAYER_RGGB8);
      impl_->img_step_.push_back(1);
    } else if (format == "BAYER_BGGR8") {
      RCLCPP_INFO(
        impl_->ros_node_->get_logger(),
        "bayer simulation may be computationally expensive.");
      impl_->img_encoding_.emplace_back(sensor_msgs::image_encodings::BAYER_BGGR8);
      impl_->img_step_.push_back(1);
    } else if (format == "BAYER_GBRG8") {
      RCLCPP_INFO(
        impl_->ros_node_->get_logger(),
        "bayer simulation may be computationally expensive.");
      impl_->img_encoding_.emplace_back(sensor_msgs::image_encodings::BAYER_GBRG8);
      impl_->img_step_.push_back(1);
    } else if (format == "BAYER_GRBG8") {
      RCLCPP_INFO(
        impl_->ros_node_->get_logger(),
        "bayer simulation may be computationally expensive.");
      impl_->img_encoding_.emplace_back(sensor_msgs::image_encodings::BAYER_GRBG8);
      impl_->img_step_.push_back(1);
    } else {
      RCLCPP_ERROR(impl_->ros_node_->get_logger(), "Unsupported Gazebo ImageFormat, using BGR8\n");
      impl_->img_encoding_.emplace_back(sensor_msgs::image_encodings::BGR8);
      impl_->img_step_.push_back(3);
    }
  }

  std::vector<unsigned int> width, height;
  if (impl_->sensor_type_ == GazeboRosCameraPrivate::CAMERA) {
    width.push_back(CameraPlugin::width);
    height.push_back(CameraPlugin::height);
    impl_->camera_.push_back(CameraPlugin::camera);
  } else if (impl_->sensor_type_ == GazeboRosCameraPrivate::DEPTH) {
    width.push_back(DepthCameraPlugin::width);
    height.push_back(DepthCameraPlugin::height);
    impl_->camera_.emplace_back(DepthCameraPlugin::depthCamera);
  } else {
    width = MultiCameraPlugin::width_;
    height = MultiCameraPlugin::height_;
    impl_->camera_ = MultiCameraPlugin::camera_;
  }

  for (uint64_t i = 0; i < impl_->num_cameras_; ++i) {
    // C parameters
    auto default_cx = (static_cast<double>(width[i]) + 1.0) / 2.0;
    auto cx = _sdf->Get<double>("cx", default_cx).first;

    auto default_cy = (static_cast<double>(height[i]) + 1.0) / 2.0;
    auto cy = _sdf->Get<double>("cy", default_cy).first;

    impl_->hfov_.push_back(impl_->camera_[i]->HFOV().Radian());

    double computed_focal_length =
      (static_cast<double>(width[i])) / (2.0 * tan(impl_->hfov_[i] / 2.0));

    // Focal length
    auto focal_length = _sdf->Get<double>("focal_length", 0.0).first;
    if (focal_length == 0) {
      focal_length = computed_focal_length;
    } else if (!ignition::math::equal(focal_length, computed_focal_length)) {
      RCLCPP_WARN(
        impl_->ros_node_->get_logger(),
        "The <focal_length> [%f] you have provided for camera [%s]"
        " is inconsistent with specified <image_width> [%d] and"
        " HFOV [%f]. Please double check to see that"
        " focal_length = width / (2.0 * tan(HFOV/2.0))."
        " The expected focal_length value is [%f],"
        " please update your camera model description accordingly.",
        focal_length, _sensor->Name().c_str(), width[i], impl_->hfov_[i], computed_focal_length);
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
    if (impl_->camera_[i]->LensDistortion()) {
      impl_->camera_[i]->LensDistortion()->SetCrop(border_crop);

      distortion_k1 = impl_->camera_[i]->LensDistortion()->K1();
      distortion_k2 = impl_->camera_[i]->LensDistortion()->K2();
      distortion_k3 = impl_->camera_[i]->LensDistortion()->K3();
      distortion_t1 = impl_->camera_[i]->LensDistortion()->P1();
      distortion_t2 = impl_->camera_[i]->LensDistortion()->P2();
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
    // Get string from SDF and parse into Matrix3d for populating message
    std::string default_rmat("1.0 0.0 0.0 0.0 1.0 0.0 0.0 0.0 1.0");
    std::istringstream stream(_sdf->Get<std::string>("rectification_matrix", default_rmat).first);

    ignition::math::Matrix3d rmat;
    stream >> rmat;
    camera_info_msg.r[0] = rmat(0, 0);
    camera_info_msg.r[1] = rmat(0, 1);
    camera_info_msg.r[2] = rmat(0, 2);
    camera_info_msg.r[3] = rmat(1, 0);
    camera_info_msg.r[4] = rmat(1, 1);
    camera_info_msg.r[5] = rmat(1, 2);
    camera_info_msg.r[6] = rmat(2, 0);
    camera_info_msg.r[7] = rmat(2, 1);
    camera_info_msg.r[8] = rmat(2, 2);

    // projection matrix
    camera_info_msg.p[0] = _sdf->Get<double>("P_fx", focal_length).first;
    camera_info_msg.p[1] = 0.0;
    camera_info_msg.p[2] = _sdf->Get<double>("P_cx", cx).first;
    camera_info_msg.p[3] = _sdf->Get<double>("Tx", -focal_length * hack_baseline).first;
    camera_info_msg.p[4] = 0.0;
    camera_info_msg.p[5] = _sdf->Get<double>("P_fy", focal_length).first;
    camera_info_msg.p[6] = _sdf->Get<double>("P_cy", cy).first;
    camera_info_msg.p[7] = _sdf->Get<double>("Ty", 0).first;
    camera_info_msg.p[8] = 0.0;
    camera_info_msg.p[9] = 0.0;
    camera_info_msg.p[10] = 1.0;
    camera_info_msg.p[11] = 0.0;

    // Initialize camera_info_manager
    impl_->camera_info_manager_.push_back(
      std::make_shared<camera_info_manager::CameraInfoManager>(
        impl_->ros_node_.get(), impl_->camera_name_));
    impl_->camera_info_manager_.back()->setCameraInfo(camera_info_msg);
  }

  if (impl_->sensor_type_ == GazeboRosCameraPrivate::DEPTH) {
    impl_->min_depth_ = _sdf->Get<double>("min_depth", 0.4).first;
    impl_->max_depth_ =
      _sdf->Get<double>("max_depth", std::numeric_limits<float>::infinity()).first;

    // Initialize point cloud message
    impl_->cloud_msg_.fields.resize(4);
    impl_->cloud_msg_.fields[0].name = "x";
    impl_->cloud_msg_.fields[0].offset = 0;
    impl_->cloud_msg_.fields[0].datatype = 7;
    impl_->cloud_msg_.fields[0].count = 1;
    impl_->cloud_msg_.fields[1].name = "y";
    impl_->cloud_msg_.fields[1].offset = 4;
    impl_->cloud_msg_.fields[1].datatype = 7;
    impl_->cloud_msg_.fields[1].count = 1;
    impl_->cloud_msg_.fields[2].name = "z";
    impl_->cloud_msg_.fields[2].offset = 8;
    impl_->cloud_msg_.fields[2].datatype = 7;
    impl_->cloud_msg_.fields[2].count = 1;
    impl_->cloud_msg_.fields[3].name = "rgb";
    impl_->cloud_msg_.fields[3].offset = 16;
    impl_->cloud_msg_.fields[3].datatype = 7;
    impl_->cloud_msg_.fields[3].count = 1;

    impl_->cloud_msg_.header.frame_id = impl_->frame_name_;
  }

  // Dynamic reconfigure
  if (impl_->sensor_type_ == GazeboRosCameraPrivate::CAMERA) {
    impl_->ros_node_->declare_parameter(
      "update_rate", gazebo::CameraPlugin::parentSensor->UpdateRate());
  } else if (impl_->sensor_type_ == GazeboRosCameraPrivate::DEPTH) {
    impl_->ros_node_->declare_parameter(
      "update_rate", gazebo::DepthCameraPlugin::parentSensor->UpdateRate());
  } else {
    impl_->ros_node_->declare_parameter(
      "update_rate", MultiCameraPlugin::parent_sensor_->UpdateRate());
  }

  auto param_change_callback =
    [this](std::vector<rclcpp::Parameter> parameters) {
      auto result = rcl_interfaces::msg::SetParametersResult();
      result.successful = true;
      for (const auto & parameter : parameters) {
        std::string param_name = parameter.get_name();
        if (param_name == "update_rate") {
          if (nullptr != impl_->trigger_sub_) {
            RCLCPP_WARN(
              impl_->ros_node_->get_logger(),
              "Cannot set update rate for triggered camera");
            result.successful = false;
          } else {
            rclcpp::ParameterType parameter_type = parameter.get_type();
            if (rclcpp::ParameterType::PARAMETER_DOUBLE == parameter_type) {
              double rate = parameter.as_double();

              if (impl_->sensor_type_ == GazeboRosCameraPrivate::CAMERA) {
                gazebo::CameraPlugin::parentSensor->SetUpdateRate(rate);
              } else if (impl_->sensor_type_ == GazeboRosCameraPrivate::DEPTH) {
                gazebo::DepthCameraPlugin::parentSensor->SetUpdateRate(rate);
              } else {
                MultiCameraPlugin::parent_sensor_->SetUpdateRate(rate);
              }

              if (rate >= 0.0) {
                RCLCPP_INFO(
                  impl_->ros_node_->get_logger(),
                  "Camera update rate changed to [%.2f Hz]", rate);
              } else {
                RCLCPP_WARN(
                  impl_->ros_node_->get_logger(),
                  "Camera update rate should be positive. Setting to maximum");
              }
            } else {
              RCLCPP_WARN(
                impl_->ros_node_->get_logger(),
                "Value for param [update_rate] has to be of double type.");
              result.successful = false;
            }
          }
        }
      }
      return result;
    };

  param_change_callback_handler_ =
    impl_->ros_node_->add_on_set_parameters_callback(param_change_callback);
}

void GazeboRosCamera::NewFrame(
  const unsigned char * _image,
  unsigned int _width,
  unsigned int _height,
  const int _camera_num)
{
  // TODO(shivesh) Enable / disable sensor once SubscriberStatusCallback has been ported to ROS2

  gazebo::common::Time sensor_update_time;

  if (impl_->sensor_type_ == GazeboRosCameraPrivate::CAMERA) {
    sensor_update_time = CameraPlugin::parentSensor->LastMeasurementTime();
  } else if (impl_->sensor_type_ == GazeboRosCameraPrivate::DEPTH) {
    sensor_update_time = DepthCameraPlugin::parentSensor->LastMeasurementTime();
  } else {
    sensor_update_time = MultiCameraPlugin::parent_sensor_->LastMeasurementTime();
  }

  // Publish camera info
  auto camera_info_msg = impl_->camera_info_manager_[_camera_num]->getCameraInfo();
  camera_info_msg.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(
    sensor_update_time);

  impl_->camera_info_pub_[_camera_num]->publish(camera_info_msg);

  std::lock_guard<std::mutex> image_lock(impl_->image_mutex_);

  // Publish image
  impl_->image_msg_.header.frame_id = impl_->frame_name_;
  impl_->image_msg_.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(
    sensor_update_time);

  // Copy from src to image_msg
  sensor_msgs::fillImage(
    impl_->image_msg_, impl_->img_encoding_[_camera_num], _height, _width,
    impl_->img_step_[_camera_num] * _width, reinterpret_cast<const void *>(_image));

  impl_->image_pub_[_camera_num].publish(impl_->image_msg_);

  // Disable camera if it's a triggered camera
  if (nullptr != impl_->trigger_sub_) {
    SetCameraEnabled(false);

    std::lock_guard<std::mutex> lock(impl_->trigger_mutex_);
    impl_->triggered = std::max(impl_->triggered - 1, 0);
  }
}

////////////////////////////////////////////////////////////////////////////////
void GazeboRosCamera::OnNewFrame(
  const unsigned char * _image,
  unsigned int _width,
  unsigned int _height,
  unsigned int /*_depth*/,
  const std::string & /*_format*/)
{
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE("GazeboRosCamera::OnNewFrame");
  IGN_PROFILE_BEGIN("fill ROS message");
#endif
  NewFrame(_image, _width, _height, 0);
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_END();
#endif
}

////////////////////////////////////////////////////////////////////////////////
void GazeboRosCamera::OnNewImageFrame(
  const unsigned char * _image,
  unsigned int _width,
  unsigned int _height,
  unsigned int /*_depth*/,
  const std::string & /*_format*/)
{
  NewFrame(_image, _width, _height, 0);
}

////////////////////////////////////////////////////////////////////////////////
void GazeboRosCamera::OnNewDepthFrame(
  const float * _image,
  unsigned int _width,
  unsigned int _height,
  unsigned int /*_depth*/,
  const std::string & /*_format*/)
{
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE("GazeboRosCamera::OnNewDepthFrame");
  IGN_PROFILE_BEGIN("fill ROS depth message");
#endif
  // TODO(shivesh) Enable / disable sensor once SubscriberStatusCallback has been ported to ROS2

  auto sensor_update_time = gazebo_ros::Convert<builtin_interfaces::msg::Time>(
    DepthCameraPlugin::parentSensor->LastMeasurementTime());

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

  int index = 0;

  float pos_inf = std::numeric_limits<float>::infinity();
  float neg_inf = -pos_inf;

  // Copy from src to image_msg
  for (uint32_t j = 0; j < _height; j++) {
    for (uint32_t i = 0; i < _width; i++) {
      index = i + j * _width;
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
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_END();
  IGN_PROFILE_BEGIN("publish depth image");
#endif
  impl_->depth_image_pub_.publish(image_msg);
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_END();
#endif
  // Publish camera info
  auto camera_info_msg = impl_->camera_info_manager_[0]->getCameraInfo();
  camera_info_msg.header.stamp = sensor_update_time;

#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_BEGIN("publish camera depth info");
#endif
  impl_->depth_camera_info_pub_->publish(camera_info_msg);
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_END();

  IGN_PROFILE_BEGIN("fill ROS cloud point message");
#endif
  // Publish point cloud
  impl_->cloud_msg_.header.stamp = sensor_update_time;
  impl_->cloud_msg_.width = _width;
  impl_->cloud_msg_.height = _height;
  impl_->cloud_msg_.row_step = impl_->cloud_msg_.point_step * _width * _height;

  sensor_msgs::PointCloud2Modifier cloud_modifier(impl_->cloud_msg_);
  cloud_modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
  impl_->cloud_msg_.data.resize(_width * _height * impl_->cloud_msg_.point_step);
  impl_->cloud_msg_.is_dense = true;

  int image_index = 0;
  int cloud_index = 0;

  double fl = (static_cast<double>(_width)) / (2.0 * tan(impl_->hfov_[0] / 2.0));

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

      float depth = _image[image_index++];

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
        std::memcpy(
          &impl_->cloud_msg_.data[cloud_index + 16],
          &impl_->image_msg_.data[(i + j * _width) * 3], 3 * sizeof(uint8_t));
      } else if (impl_->image_msg_.data.size() == _height * _width) {
        std::memcpy(
          &impl_->cloud_msg_.data[cloud_index + 16],
          &impl_->image_msg_.data[i + j * _width], 3 * sizeof(uint8_t));
      } else {
        // no image
        std::memset(&impl_->cloud_msg_.data[cloud_index + 16], 0, 3 * sizeof(uint8_t));
      }
      cloud_index += impl_->cloud_msg_.point_step;
    }
  }
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_END();

  IGN_PROFILE_BEGIN("publish cloud point");
#endif
  impl_->point_cloud_pub_->publish(impl_->cloud_msg_);
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_END();
#endif
  // Disable camera if it's a triggered camera
  if (nullptr != impl_->trigger_sub_) {
    SetCameraEnabled(false);
    std::lock_guard<std::mutex> lock(impl_->trigger_mutex_);
    impl_->triggered = std::max(impl_->triggered - 1, 0);
  }
}

////////////////////////////////////////////////////////////////////////////////
void GazeboRosCamera::OnNewMultiFrame(
  const unsigned char * _image,
  unsigned int _width,
  unsigned int _height,
  unsigned int /*_depth*/,
  const std::string & /*_format*/,
  const int _camera_num)
{
  NewFrame(_image, _width, _height, _camera_num);
}

void GazeboRosCamera::SetCameraEnabled(const bool _enabled)
{
  if (impl_->sensor_type_ == GazeboRosCameraPrivate::CAMERA) {
    CameraPlugin::parentSensor->SetActive(_enabled);
    CameraPlugin::parentSensor->SetUpdateRate(_enabled ? 0.0 : ignition::math::MIN_D);
  } else if (impl_->sensor_type_ == GazeboRosCameraPrivate::DEPTH) {
    DepthCameraPlugin::parentSensor->SetActive(_enabled);
    DepthCameraPlugin::parentSensor->SetUpdateRate(_enabled ? 0.0 : ignition::math::MIN_D);
  } else {
    MultiCameraPlugin::parent_sensor_->SetActive(_enabled);
    MultiCameraPlugin::parent_sensor_->SetUpdateRate(_enabled ? 0.0 : ignition::math::MIN_D);
  }
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

std::vector<gazebo::rendering::CameraPtr> GazeboRosCamera::GetCameras() const
{
  return impl_->camera_;
}

std::string GazeboRosCamera::GetCameraName() const
{
  return impl_->camera_name_;
}

uint64_t GazeboRosCamera::GetNumCameras() const
{
  return impl_->num_cameras_;
}

gazebo_ros::Node::SharedPtr GazeboRosCamera::GetRosNode() const
{
  return impl_->ros_node_;
}

extern "C" GZ_PLUGIN_VISIBLE gazebo::SensorPlugin * RegisterPlugin();
gazebo::SensorPlugin * RegisterPlugin()
{
  return (gazebo::CameraPlugin *)(new GazeboRosCamera());
}
}  // namespace gazebo_plugins
