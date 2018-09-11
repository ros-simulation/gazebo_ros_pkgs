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

#include <sdf/sdf.hh>
#include <gazebo/rendering/Distortion.hh>

#include <gazebo_plugins/gazebo_ros_camera.hpp>
#include <gazebo_ros/node.hpp>
#include <image_transport/image_transport.h>
#include <sensor_msgs/msg/camera_info.hpp>

namespace gazebo_plugins
{
class GazeboRosCameraPrivate
{
public:
  /// A pointer to the GazeboROS node.
  gazebo_ros::Node::SharedPtr ros_node_;

  ///
//  image_transport::ImageTransport it_;

  ///
  image_transport::Publisher image_pub_;

  ///
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
};

GazeboRosCamera::GazeboRosCamera()
: impl_(std::make_unique<GazeboRosCameraPrivate>())
{
}

GazeboRosCamera::~GazeboRosCamera()
{
}

void GazeboRosCamera::Load(gazebo::sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
  gazebo::CameraPlugin::Load(_sensor, _sdf);

//  if (!this->sdf->HasElement("cameraName"))
//    ROS_DEBUG_NAMED("camera_utils", "Camera plugin missing <cameraName>, default to empty");
//  else
//    this->camera_name_ = this->sdf->Get<std::string>("cameraName");
//
//  // overwrite camera suffix
//  // example usage in gazebo_ros_multicamera
//  this->camera_name_ += _camera_name_suffix;
//
//  if (!this->sdf->HasElement("frameName"))
//    ROS_DEBUG_NAMED("camera_utils", "Camera plugin missing <frameName>, defaults to /world");
//  else
//    this->frame_name_ = this->sdf->Get<std::string>("frameName");
//

  // Initialize ROS node
  impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);

  // TODO(louise) Doesn't have a default constructor, figure out how to keep as member variable
  auto it_ = new image_transport::ImageTransport(impl_->ros_node_);

//  if (!this->camera_name_.empty())
//  {
//    dyn_srv_ =
//      new dynamic_reconfigure::Server<gazebo_plugins::GazeboRosCameraConfig>
//      (*this->rosnode_);
//    dynamic_reconfigure::Server<gazebo_plugins::GazeboRosCameraConfig>
//      ::CallbackType f =
//      boost::bind(&GazeboRosCameraUtils::configCallback, this, _1, _2);
//    dyn_srv_->setCallback(f);
//  }
//  else
//  {
//    ROS_WARN_NAMED("camera_utils", "dynamic reconfigure is not enabled for this image topic [%s]"
//             " becuase <cameraName> is not specified",
//             this->image_topic_name_.c_str());
//  }

  // Image publisher
  auto image_topic_name = _sdf->Get<std::string>("image_topic_name", "image_raw").first;
  // TODO(louise) SubscriberStatusCallback has not been ported yet

  // Sensor generation off by default.  Must do this before advertising the
  // associated ROS topics.
//  this->parentSensor->SetActive(false);
//  if (!this->image_connect_count_)
//    this->image_connect_count_ = boost::shared_ptr<int>(new int(0));
//  if (!this->image_connect_count_lock_)
//    this->image_connect_count_lock_ = boost::shared_ptr<boost::mutex>(new boost::mutex);
//  if (!this->was_active_)
//    this->was_active_ = boost::shared_ptr<bool>(new bool(false));
//  impl_->image_pub_ = it_->advertise(
//    image_topic_name, 2,
//    boost::bind(&GazeboRosCameraUtils::ImageConnect, this),
//    boost::bind(&GazeboRosCameraUtils::ImageDisconnect, this),
//    ros::VoidPtr(), true);

  impl_->image_pub_ = it_->advertise(image_topic_name, 2);

  // Camera info publisher
  auto camera_info_topic_name =
    _sdf->Get<std::string>("camera_info_topic_name", "camera_info").first;
//  // camera info publish rate will be synchronized to image sensor
//  // publish rates.
//  // If someone connects to camera_info, sensor will be activated
//  // and camera_info will be published alongside image_raw with the
//  // same timestamps.  This incurrs additional computational cost when
//  // there are subscribers to camera_info, but better mimics behavior
//  // of image_pipeline.
//  ros::AdvertiseOptions cio =
//    ros::AdvertiseOptions::create<sensor_msgs::CameraInfo>(
//    this->camera_info_topic_name_, 2,
//    boost::bind(&GazeboRosCameraUtils::ImageConnect, this),
//    boost::bind(&GazeboRosCameraUtils::ImageDisconnect, this),
//    ros::VoidPtr(), &this->camera_queue_);
 impl_->camera_info_pub_ = impl_->ros_node_->create_publisher<sensor_msgs::msg::CameraInfo>(
    camera_info_topic_name);

  // Trigger
  auto trigger_topic_name = _sdf->Get<std::string>("trigger_topic_name", "image_trigger").first;
//  if (this->CanTriggerCamera())
//  {
//    ros::SubscribeOptions trigger_so =
//      ros::SubscribeOptions::create<std_msgs::Empty>(
//          this->trigger_topic_name_, 1,
//          boost::bind(&GazeboRosCameraUtils::TriggerCameraInternal, this, _1),
//          ros::VoidPtr(), &this->camera_queue_);
//    this->trigger_subscriber_ = this->rosnode_->subscribe(trigger_so);
//  }
//

  // set buffer size
//  if (this->format_ == "L8" || this->format_ == "L_INT8")
//  {
//    this->type_ = sensor_msgs::image_encodings::MONO8;
//    this->skip_ = 1;
//  }
//  else if (this->format_ == "L16" || this->format_ == "L_INT16")
//  {
//    this->type_ = sensor_msgs::image_encodings::MONO16;
//    this->skip_ = 2;
//  }
//  else if (this->format_ == "R8G8B8" || this->format_ == "RGB_INT8")
//  {
//    this->type_ = sensor_msgs::image_encodings::RGB8;
//    this->skip_ = 3;
//  }
//  else if (this->format_ == "B8G8R8" || this->format_ == "BGR_INT8")
//  {
//    this->type_ = sensor_msgs::image_encodings::BGR8;
//    this->skip_ = 3;
//  }
//  else if (this->format_ == "R16G16B16" ||  this->format_ == "RGB_INT16")
//  {
//    this->type_ = sensor_msgs::image_encodings::RGB16;
//    this->skip_ = 6;
//  }
//  else if (this->format_ == "BAYER_RGGB8")
//  {
//    ROS_INFO_NAMED("camera_utils", "bayer simulation maybe computationally expensive.");
//    this->type_ = sensor_msgs::image_encodings::BAYER_RGGB8;
//    this->skip_ = 1;
//  }
//  else if (this->format_ == "BAYER_BGGR8")
//  {
//    ROS_INFO_NAMED("camera_utils", "bayer simulation maybe computationally expensive.");
//    this->type_ = sensor_msgs::image_encodings::BAYER_BGGR8;
//    this->skip_ = 1;
//  }
//  else if (this->format_ == "BAYER_GBRG8")
//  {
//    ROS_INFO_NAMED("camera_utils", "bayer simulation maybe computationally expensive.");
//    this->type_ = sensor_msgs::image_encodings::BAYER_GBRG8;
//    this->skip_ = 1;
//  }
//  else if (this->format_ == "BAYER_GRBG8")
//  {
//    ROS_INFO_NAMED("camera_utils", "bayer simulation maybe computationally expensive.");
//    this->type_ = sensor_msgs::image_encodings::BAYER_GRBG8;
//    this->skip_ = 1;
//  }
//  else
//  {
//    ROS_ERROR_NAMED("camera_utils", "Unsupported Gazebo ImageFormat\n");
//    this->type_ = sensor_msgs::image_encodings::BGR8;
//    this->skip_ = 3;
//  }

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
//  camera_info_msg.header.frame_id = this->frame_name_;
  camera_info_msg.height = this->height;
  camera_info_msg.width  = this->width;
  camera_info_msg.distortion_model = "plumb_bob";
  camera_info_msg.d.resize(5);

  // Allow the user to disable automatic cropping (used to remove barrel
  // distortion black border. The crop can be useful, but also skewes
  // the lens distortion, making the supplied k and t values incorrect.
  auto border_crop = _sdf->Get<bool>("border_crop", true).first;
  if (this->camera->LensDistortion())
  {
    this->camera->LensDistortion()->SetCrop(border_crop);
  }

  // Get distortion parameters from gazebo sensor if auto_distortion is true
  auto auto_distortion = _sdf->Get<bool>("auto_distortion", true).first;
  auto distortion_k1 = _sdf->Get<double>("distortion_k1", 0.0).first;
  auto distortion_k2 = _sdf->Get<double>("distortion_k2", 0.0).first;
  auto distortion_k3 = _sdf->Get<double>("distortion_k3", 0.0).first;
  auto distortion_t1 = _sdf->Get<double>("distortion_t1", 0.0).first;
  auto distortion_t2 = _sdf->Get<double>("distortion_t2", 0.0).first;
  auto hack_baseline = _sdf->Get<double>("hack_baseline", 0.0).first;
  if (auto_distortion)
  {
    RCLCPP_INFO(impl_->ros_node_->get_logger(),
      "Auto-distortion is true, <distortion> parameters will be ignored.");
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

//  // Initialize camera_info_manager
//  // TODO(louise) Not ported to ROS2 yet
//  this->camera_info_manager_.reset(new camera_info_manager::CameraInfoManager(
//          *this->rosnode_, this->camera_name_));
//  this->camera_info_manager_->setCameraInfo(camera_info_msg);

//  load_event_();
}

////////////////////////////////////////////////////////////////////////////////
void GazeboRosCamera::OnNewFrame(
    const unsigned char *_image,
    unsigned int _width,
    unsigned int _height,
    unsigned int _depth,
    const std::string &_format)
{
  auto sensor_update_time = this->parentSensor->LastMeasurementTime();

  if (!this->parentSensor->IsActive())
  {
//    if ((*this->image_connect_count_) > 0)
//      // do this first so there's chance for sensor to run once after activated
//      this->parentSensor->SetActive(true);
//    return;
  }

//    if ((*this->image_connect_count_) > 0)
//    {
//      if (sensor_update_time < this->last_update_time_)
//      {
//          ROS_WARN_NAMED("camera", "Negative sensor update time difference detected.");
//          this->last_update_time_ = sensor_update_time;
//      }
//
//      // OnNewFrame is triggered at the gazebo sensor <update_rate>
//      // while there is also a plugin <updateRate> that can throttle the
//      // rate down further (but then why not reduce the sensor rate?
//      // what is the use case?).
//      // Setting the <updateRate> to zero will make this plugin
//      // update at the gazebo sensor <update_rate>, update_period_ will be
//      // zero and the conditional always will be true.
//      if (sensor_update_time - this->last_update_time_ >= this->update_period_)
//      {
//        this->PutCameraData(_image, sensor_update_time);
//        this->PublishCameraInfo(sensor_update_time);
//        this->last_update_time_ = sensor_update_time;
//      }
//    }
}
GZ_REGISTER_SENSOR_PLUGIN(GazeboRosCamera)
}  // namespace gazebo_plugins
