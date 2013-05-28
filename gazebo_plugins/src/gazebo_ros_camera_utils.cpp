/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003
 *     Nate Koenig & Andrew Howard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/*
 @mainpage
   Desc: GazeboRosCameraUtils plugin for simulating camera_s in Gazebo
   Author: John Hsu
   Date: 24 Sept 2008
   SVN info: $Id$
 @htmlinclude manifest.html
 @b GazeboRosCameraUtils plugin broadcasts ROS Image messages
 */

#include <algorithm>
#include <assert.h>
#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>

#include <gazebo_plugins/gazebo_ros_camera_utils.h>

#include "physics/World.hh"
#include "physics/HingeJoint.hh"
#include "sensors/Sensor.hh"
#include "sdf/interface/SDF.hh"
#include "sdf/interface/Param.hh"
#include "common/Exception.hh"
#include "sensors/CameraSensor.hh"
#include "sensors/SensorTypes.hh"
#include "rendering/Camera.hh"

#include "sensor_msgs/Image.h"
#include "sensor_msgs/fill_image.h"
#include "image_transport/image_transport.h"

#include <geometry_msgs/Point32.h>
#include <sensor_msgs/ChannelFloat32.h>

#include "tf/tf.h"

namespace gazebo
{

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosCameraUtils::GazeboRosCameraUtils()
{
  this->image_connect_count_ = 0;
  this->info_connect_count_ = 0;

  // maintain for one more release for backwards compatibility with pr2_gazebo_plugins
  this->imageConnectCount = this->image_connect_count_;
  this->infoConnectCount = this->info_connect_count_;

  this->last_update_time_ = common::Time(0);
  this->last_info_update_time_ = common::Time(0);
}

void GazeboRosCameraUtils::configCallback(gazebo_plugins::GazeboRosCameraConfig &config, uint32_t level)
{
  ROS_INFO("Reconfigure request for the gazebo ros camera_: %s. New rate: %.2f", this->camera_name_.c_str(), config.imager_rate);
  this->parentSensor_->SetUpdateRate(config.imager_rate);
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosCameraUtils::~GazeboRosCameraUtils()
{
  this->parentSensor_->SetActive(false);
  this->rosnode_->shutdown();
  this->camera_queue_.clear();
  this->camera_queue_.disable();
  this->callback_queue_thread_.join();
  delete this->rosnode_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosCameraUtils::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{
  // Get the world name.
  std::string world_name = _parent->GetWorldName();

  // Get the world_
  this->world_ = physics::get_world(world_name);

  // maintain for one more release for backwards compatibility with pr2_gazebo_plugins
  this->world = this->world_;

  this->robot_namespace_ = "";
  if (_sdf->HasElement("robotNamespace"))
    this->robot_namespace_ = _sdf->GetElement("robotNamespace")->GetValueString() + "/";

  this->image_topic_name_ = "image_raw";
  if (_sdf->GetElement("imageTopicName"))
    this->image_topic_name_ = _sdf->GetElement("imageTopicName")->GetValueString();

  this->camera_info_topic_name_ = "camera_info";
  if (_sdf->HasElement("cameraInfoTopicName"))
    this->camera_info_topic_name_ = _sdf->GetElement("cameraInfoTopicName")->GetValueString();

  if (!_sdf->HasElement("cameraName"))
    ROS_INFO("Camera plugin missing <cameraName>, default to empty");
  else
    this->camera_name_ = _sdf->GetElement("cameraName")->GetValueString();

  if (!_sdf->HasElement("frameName"))
    ROS_INFO("Camera plugin missing <frameName>, defaults to /world");
  else
    this->frame_name_ = _sdf->GetElement("frameName")->GetValueString();

  if (!_sdf->GetElement("updateRate"))
  {
    ROS_INFO("Camera plugin missing <updateRate>, defaults to 0");
    this->update_rate_ = 0;
  }
  else
    this->update_rate_ = _sdf->GetElement("updateRate")->GetValueDouble();

  if (!_sdf->GetElement("CxPrime"))
  {
    ROS_INFO("Camera plugin missing <CxPrime>, defaults to 0");
    this->cx_prime_ = 0;
  }
  else
    this->cx_prime_ = _sdf->GetElement("CxPrime")->GetValueDouble();

  if (!_sdf->HasElement("Cx"))
  {
    ROS_INFO("Camera plugin missing <Cx>, defaults to 0");
    this->cx_= 0;
  }
  else
    this->cx_ = _sdf->GetElement("Cx")->GetValueDouble();

  if (!_sdf->HasElement("Cy"))
  {
    ROS_INFO("Camera plugin missing <Cy>, defaults to 0");
    this->cy_= 0;
  }
  else
    this->cy_ = _sdf->GetElement("Cy")->GetValueDouble();

  if (!_sdf->HasElement("focalLength"))
  {
    ROS_INFO("Camera plugin missing <focalLength>, defaults to 0");
    this->focal_length_= 0;
  }
  else
    this->focal_length_ = _sdf->GetElement("focalLength")->GetValueDouble();

  if (!_sdf->HasElement("hackBaseline"))
  {
    ROS_INFO("Camera plugin missing <hackBaseline>, defaults to 0");
    this->hack_baseline_= 0;
  }
  else
    this->hack_baseline_ = _sdf->GetElement("hackBaseline")->GetValueDouble();

  if (!_sdf->HasElement("distortionK1"))
  {
    ROS_INFO("Camera plugin missing <distortionK1>, defaults to 0");
    this->distortion_k1_= 0;
  }
  else
    this->distortion_k1_ = _sdf->GetElement("distortionK1")->GetValueDouble();

  if (!_sdf->HasElement("distortionK2"))
  {
    ROS_INFO("Camera plugin missing <distortionK2>, defaults to 0");
    this->distortion_k2_= 0;
  }
  else
    this->distortion_k2_ = _sdf->GetElement("distortionK2")->GetValueDouble();

  if (!_sdf->HasElement("distortionK3"))
  {
    ROS_INFO("Camera plugin missing <distortionK3>, defaults to 0");
    this->distortion_k3_= 0;
  }
  else
    this->distortion_k3_ = _sdf->GetElement("distortionK3")->GetValueDouble();

  if (!_sdf->HasElement("distortionT1"))
  {
    ROS_INFO("Camera plugin missing <distortionT1>, defaults to 0");
    this->distortion_t1_= 0;
  }
  else
    this->distortion_t1_ = _sdf->GetElement("distortionT1")->GetValueDouble();

  if (!_sdf->HasElement("distortionT2"))
  {
    ROS_INFO("Camera plugin missing <distortionT2>, defaults to 0");
    this->distortion_t2_= 0;
  }
  else
    this->distortion_t2_ = _sdf->GetElement("distortionT2")->GetValueDouble();

  if ((this->distortion_k1_ != 0.0) || (this->distortion_k2_ != 0.0) ||
      (this->distortion_k3_ != 0.0) || (this->distortion_t1_ != 0.0) ||
      (this->distortion_t2_ != 0.0))
  {
    ROS_WARN("gazebo_ros_camera_ simulation does not support non-zero distortion parameters right now, your simulation maybe wrong.");
  }


  // Setup ROS
  if (!ros::isInitialized())
  {
    int argc = 0;
    char** argv = NULL;
    ros::init( argc, argv, "gazebo",
               ros::init_options::NoSigintHandler |
               ros::init_options::AnonymousName );
  }

  this->rosnode_ = new ros::NodeHandle(this->robot_namespace_+"/"+this->camera_name_);

  this->itnode_ = new image_transport::ImageTransport(*this->rosnode_);

  // resolve tf prefix
  std::string prefix;
  this->rosnode_->getParam(std::string("tf_prefix"), prefix);
  this->frame_name_ = tf::resolve(prefix, this->frame_name_);

  if (!this->camera_name_.empty())
  {
    dyn_srv_ = new dynamic_reconfigure::Server<gazebo_plugins::GazeboRosCameraConfig>(*this->rosnode_);
    dynamic_reconfigure::Server<gazebo_plugins::GazeboRosCameraConfig>::CallbackType f = boost::bind(&GazeboRosCameraUtils::configCallback, this, _1, _2);
    dyn_srv_->setCallback(f);
  }
  else
  {
    ROS_WARN("dynamic reconfigure is not enabled for this image topic [%s] becuase <cameraName> is not specified",this->image_topic_name_.c_str());
  }

  this->image_pub_ = this->itnode_->advertise(
    this->image_topic_name_,1,
    boost::bind( &GazeboRosCameraUtils::ImageConnect,this),
    boost::bind( &GazeboRosCameraUtils::ImageDisconnect,this),
    ros::VoidPtr(), &this->camera_queue_);

  ros::AdvertiseOptions camera_info_ao =
    ros::AdvertiseOptions::create<sensor_msgs::CameraInfo>(
        this->camera_info_topic_name_,1,
        boost::bind( &GazeboRosCameraUtils::InfoConnect,this),
        boost::bind( &GazeboRosCameraUtils::InfoDisconnect,this),
        ros::VoidPtr(), &this->camera_queue_);
  this->camera_info_pub_ = this->rosnode_->advertise(camera_info_ao);

  ros::SubscribeOptions zoom_so =
    ros::SubscribeOptions::create<std_msgs::Float64>(
        "set_hfov",1,
        boost::bind( &GazeboRosCameraUtils::SetHFOV,this,_1),
        ros::VoidPtr(), &this->camera_queue_);
  this->cameraHFOVSubscriber_ = this->rosnode_->subscribe(zoom_so);

  ros::SubscribeOptions rate_so =
    ros::SubscribeOptions::create<std_msgs::Float64>(
        "set_update_rate",1,
        boost::bind( &GazeboRosCameraUtils::SetUpdateRate,this,_1),
        ros::VoidPtr(), &this->camera_queue_);
  this->cameraUpdateRateSubscriber_ = this->rosnode_->subscribe(rate_so);

  this->Init();
}

////////////////////////////////////////////////////////////////////////////////
// Increment count
void GazeboRosCameraUtils::InfoConnect()
{
  this->info_connect_count_++;
  // maintain for one more release for backwards compatibility with pr2_gazebo_plugins
  this->infoConnectCount = this->info_connect_count_;
}
////////////////////////////////////////////////////////////////////////////////
// Decrement count
void GazeboRosCameraUtils::InfoDisconnect()
{
  this->info_connect_count_--;
  // maintain for one more release for backwards compatibility with pr2_gazebo_plugins
  this->infoConnectCount = this->info_connect_count_;
}

////////////////////////////////////////////////////////////////////////////////
// Set Horizontal Field of View
void GazeboRosCameraUtils::SetHFOV(const std_msgs::Float64::ConstPtr& hfov)
{
  this->camera_->SetHFOV(hfov->data);
}

////////////////////////////////////////////////////////////////////////////////
// Set Update Rate
void GazeboRosCameraUtils::SetUpdateRate(const std_msgs::Float64::ConstPtr& update_rate)
{
  this->parentSensor_->SetUpdateRate(update_rate->data);
}

////////////////////////////////////////////////////////////////////////////////
// Increment count
void GazeboRosCameraUtils::ImageConnect()
{
  this->image_connect_count_++;
  // maintain for one more release for backwards compatibility with pr2_gazebo_plugins
  this->imageConnectCount = this->image_connect_count_;
  this->parentSensor_->SetActive(true);
}
////////////////////////////////////////////////////////////////////////////////
// Decrement count
void GazeboRosCameraUtils::ImageDisconnect()
{
  this->image_connect_count_--;
  // maintain for one more release for backwards compatibility with pr2_gazebo_plugins
  this->imageConnectCount = this->image_connect_count_;
  if (this->image_connect_count_ <= 0)
    this->parentSensor_->SetActive(false);
}

////////////////////////////////////////////////////////////////////////////////
// Initialize the controller
void GazeboRosCameraUtils::Init()
{
  // set parent sensor update rate
  this->parentSensor_->SetUpdateRate(this->update_rate_);

  // prepare to throttle this plugin at the same rate
  // ideally, we should invoke a plugin update when the sensor updates,
  // have to think about how to do that properly later
  if (this->update_rate_ > 0.0)
    this->update_period_ = 1.0/this->update_rate_;
  else
    this->update_period_ = 0.0;


  // sensor generation off by default
  this->parentSensor_->SetActive(false);

  // set buffer size
  if (this->format_ == "L8")
  {
    this->type_ = sensor_msgs::image_encodings::MONO8;
    this->skip_ = 1;
  }
  else if (this->format_ == "R8G8B8")
  {
    this->type_ = sensor_msgs::image_encodings::RGB8;
    this->skip_ = 3;
  }
  else if (this->format_ == "B8G8R8")
  {
    this->type_ = sensor_msgs::image_encodings::BGR8;
    this->skip_ = 3;
  }
  else if (this->format_ == "BAYER_RGGB8")
  {
    ROS_INFO("bayer simulation maybe computationally expensive.");
    this->type_ = sensor_msgs::image_encodings::BAYER_RGGB8;
    this->skip_ = 1;
  }
  else if (this->format_ == "BAYER_BGGR8")
  {
    ROS_INFO("bayer simulation maybe computationally expensive.");
    this->type_ = sensor_msgs::image_encodings::BAYER_BGGR8;
    this->skip_ = 1;
  }
  else if (this->format_ == "BAYER_GBRG8")
  {
    ROS_INFO("bayer simulation maybe computationally expensive.");
    this->type_ = sensor_msgs::image_encodings::BAYER_GBRG8;
    this->skip_ = 1;
  }
  else if (this->format_ == "BAYER_GRBG8")
  {
    ROS_INFO("bayer simulation maybe computationally expensive.");
    this->type_ = sensor_msgs::image_encodings::BAYER_GRBG8;
    this->skip_ = 1;
  }
  else
  {
    ROS_ERROR("Unsupported Gazebo ImageFormat\n");
    this->type_ = sensor_msgs::image_encodings::BGR8;
    this->skip_ = 3;
  }

  /// Compute camera_ parameters if set to 0
  if (this->cx_prime_ == 0)
    this->cx_prime_ = ((double)this->width_+1.0) /2.0;
  if (this->cx_ == 0)
    this->cx_ = ((double)this->width_+1.0) /2.0;
  if (this->cy_ == 0)
    this->cy_ = ((double)this->height_+1.0) /2.0;


  double computed_focal_length = ((double)this->width_) / (2.0 *tan(this->camera_->GetHFOV().Radian()/2.0));
  if (this->focal_length_ == 0)
  {
    this->focal_length_ = computed_focal_length;
  }
  else
  {
    if (!gazebo::math::equal(this->focal_length_, computed_focal_length)) // check against float precision
    {
      ROS_WARN("The <focal_length>[%f] you have provided for camera_ [%s] is inconsistent with specified image_width [%d] and HFOV [%f].   Please double check to see that focal_length = width_ / (2.0 * tan( HFOV/2.0 )), the explected focal_lengtth value is [%f], please update your camera_ model description accordingly.",
                this->focal_length_,this->parentSensor_->GetName().c_str(),this->width_,this->camera_->GetHFOV().Radian(),
                computed_focal_length);
    }
  }


  // start custom queue for camera_
  this->callback_queue_thread_ = boost::thread( boost::bind( &GazeboRosCameraUtils::CameraQueueThread,this ) );
}

////////////////////////////////////////////////////////////////////////////////
// Put camera_ data to the interface
void GazeboRosCameraUtils::PutCameraData(const unsigned char *_src, common::Time &last_update_time)
{
  this->sensor_update_time_ = last_update_time;
  this->PutCameraData(_src);
}

void GazeboRosCameraUtils::PutCameraData(const unsigned char *_src)
{
  this->lock_.lock();

  // copy data into image
  this->image_msg_.header.frame_id = this->frame_name_;
  this->image_msg_.header.stamp.sec = this->sensor_update_time_.sec;
  this->image_msg_.header.stamp.nsec = this->sensor_update_time_.nsec;

  /// don't bother if there are no subscribers
  if (this->image_connect_count_ > 0)
  {
    // copy from src to image_msg_
    fillImage(this->image_msg_,
        this->type_,
        this->height_,
        this->width_,
        this->skip_*this->width_,
        (void*)_src );

    // publish to ros
    this->image_pub_.publish(this->image_msg_);
  }

  this->lock_.unlock();
}



////////////////////////////////////////////////////////////////////////////////
// Put camera_ data to the interface
void GazeboRosCameraUtils::PublishCameraInfo(common::Time &last_update_time)
{
  this->sensor_update_time_ = last_update_time;
  this->PublishCameraInfo();
}

void GazeboRosCameraUtils::PublishCameraInfo()
{
  if (this->info_connect_count_ > 0)
  {
    this->sensor_update_time_ = this->parentSensor_->GetLastUpdateTime();
    common::Time cur_time = this->world_->GetSimTime();
    if (cur_time - this->last_info_update_time_ >= this->update_period_)
    {
      this->PublishCameraInfo(this->camera_info_pub_);
      this->last_info_update_time_ = cur_time;
    }
  }
}

void GazeboRosCameraUtils::PublishCameraInfo(ros::Publisher &camera_info_publisher)
{
  sensor_msgs::CameraInfo camera_info_msg;
  // fill CameraInfo
  camera_info_msg.header.frame_id = this->frame_name_;

  camera_info_msg.header.stamp.sec = this->sensor_update_time_.sec;
  camera_info_msg.header.stamp.nsec = this->sensor_update_time_.nsec;
  camera_info_msg.height = this->height_;
  camera_info_msg.width  = this->width_;
  // distortion
  camera_info_msg.distortion_model = "plumb_bob";
  camera_info_msg.D.resize(5);
  camera_info_msg.D[0] = this->distortion_k1_;
  camera_info_msg.D[1] = this->distortion_k2_;
  camera_info_msg.D[2] = this->distortion_k3_;
  camera_info_msg.D[3] = this->distortion_t1_;
  camera_info_msg.D[4] = this->distortion_t2_;
  // original camera_ matrix
  camera_info_msg.K[0] = this->focal_length_;
  camera_info_msg.K[1] = 0.0;
  camera_info_msg.K[2] = this->cx_;
  camera_info_msg.K[3] = 0.0;
  camera_info_msg.K[4] = this->focal_length_;
  camera_info_msg.K[5] = this->cy_;
  camera_info_msg.K[6] = 0.0;
  camera_info_msg.K[7] = 0.0;
  camera_info_msg.K[8] = 1.0;
  // rectification
  camera_info_msg.R[0] = 1.0;
  camera_info_msg.R[1] = 0.0;
  camera_info_msg.R[2] = 0.0;
  camera_info_msg.R[3] = 0.0;
  camera_info_msg.R[4] = 1.0;
  camera_info_msg.R[5] = 0.0;
  camera_info_msg.R[6] = 0.0;
  camera_info_msg.R[7] = 0.0;
  camera_info_msg.R[8] = 1.0;
  // camera_ projection matrix (same as camera_ matrix due to lack of distortion/rectification) (is this generated?)
  camera_info_msg.P[0] = this->focal_length_;
  camera_info_msg.P[1] = 0.0;
  camera_info_msg.P[2] = this->cx_;
  camera_info_msg.P[3] = -this->focal_length_ * this->hack_baseline_;
  camera_info_msg.P[4] = 0.0;
  camera_info_msg.P[5] = this->focal_length_;
  camera_info_msg.P[6] = this->cy_;
  camera_info_msg.P[7] = 0.0;
  camera_info_msg.P[8] = 0.0;
  camera_info_msg.P[9] = 0.0;
  camera_info_msg.P[10] = 1.0;
  camera_info_msg.P[11] = 0.0;

  camera_info_publisher.publish(camera_info_msg);
}


////////////////////////////////////////////////////////////////////////////////
// Put camera_ data to the interface
void GazeboRosCameraUtils::CameraQueueThread()
{
  static const double timeout = 0.001;

  while (this->rosnode_->ok())
  {
    /// publish CameraInfo
    this->PublishCameraInfo();

    /// take care of callback queue
    this->camera_queue_.callAvailable(ros::WallDuration(timeout));
  }
}

}
