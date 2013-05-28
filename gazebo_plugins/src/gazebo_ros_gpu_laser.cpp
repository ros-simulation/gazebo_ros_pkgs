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
   Desc: GazeboRosGpuLaser plugin for simulating ray sensors in Gazebo
   Author: Mihai Emanuel Dolha
   Date: 29 March 2012
   SVN info: $Id$
 @htmlinclude manifest.html
 @b GazeboRosGpuLaser plugin broadcasts ROS Image messages
 */

#include <algorithm>
#include <assert.h>
#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>

#include "sensor_msgs/fill_image.h"
#include "image_transport/image_transport.h"

#include <gazebo_plugins/gazebo_ros_gpu_laser.h>

#include "sensors/Sensor.hh"
#include "sensors/GpuRaySensor.hh"
#include "sdf/interface/SDF.hh"
#include "sensors/SensorTypes.hh"

#include "tf/tf.h"

#include <iostream>
#include <fstream>

namespace gazebo
{

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosGpuLaser::GazeboRosGpuLaser()
{
  this->laser_connect_count_ = 0;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosGpuLaser::~GazeboRosGpuLaser()
{
  this->parentSensor->SetActive(false);
  this->rosnode_->shutdown();
  this->queue_.clear();
  this->queue_.disable();
  this->callback_queue_thread_.join();
//  this->timelog_.close();
  delete this->rosnode_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosGpuLaser::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{
  GpuRayPlugin::Load(_parent, _sdf);

  //GazeboRosCameraUtils::Load(_parent, _sdf);
  this->robot_namespace_ = "";
  if (_sdf->HasElement("robotNamespace"))
    this->robot_namespace_ = _sdf->GetElement("robotNamespace")->GetValueString() + "/";

  // point cloud stuff
  if (!_sdf->GetElement("topicName"))
    this->laser_topic_name_ = "scan";
  else
    this->laser_topic_name_ = _sdf->GetElement("topicName")->GetValueString();

  if (!_sdf->GetElement("pointCloudCutoff"))
    this->point_cloud_cutoff_ = 0.4;
  else
    this->point_cloud_cutoff_ = _sdf->GetElement("pointCloudCutoff")->GetValueDouble();

  this->frame_name_ = "/world";
  if (_sdf->HasElement("frameName"))
    this->frame_name_ = _sdf->GetElement("frameName")->GetValueString();

  if (!_sdf->HasElement("gaussianNoise"))
  {
    ROS_INFO("Block laser plugin missing <gaussianNoise>, defaults to 0.0");
    this->gaussian_noise_ = 0;
  }
  else
    this->gaussian_noise_ = _sdf->GetElement("gaussianNoise")->GetValueDouble();

  if (!_sdf->HasElement("hokuyoMinIntensity"))
  {
    ROS_INFO("Block laser plugin missing <hokuyoMinIntensity>, defaults to 101");
    this->hokuyo_min_intensity_ = 101;
  }
  else
    this->hokuyo_min_intensity_ = _sdf->GetElement("hokuyoMinIntensity")->GetValueDouble();

  ROS_INFO("INFO: gazebo_ros_laser plugin should set minimum intensity to %f due to cutoff in hokuyo filters." , this->hokuyo_min_intensity_);

  if (!_sdf->GetElement("updateRate"))
  {
    ROS_INFO("Camera plugin missing <updateRate>, defaults to 0");
    this->update_rate_ = 0;
  }
  else
    this->update_rate_ = _sdf->GetElement("updateRate")->GetValueDouble();


  // Setup ROS
  if (!ros::isInitialized())
  {
    int argc = 0;
    char** argv = NULL;
    ros::init( argc, argv, "gazebo",
               ros::init_options::NoSigintHandler |
               ros::init_options::AnonymousName );
  }

  this->rosnode_ = new ros::NodeHandle(this->robot_namespace_);

  // resolve tf prefix
  std::string prefix;
  this->rosnode_->getParam(std::string("tf_prefix"), prefix);
  this->frame_name_ = tf::resolve(prefix, this->frame_name_);

  ros::AdvertiseOptions laser_scan_ao;

  if (this->parentSensor->GetVerticalRangeCount() != 1)
    laser_scan_ao = ros::AdvertiseOptions::create<pcl::PointCloud<pcl::PointXYZI> >(
      this->laser_topic_name_,1,
      boost::bind( &GazeboRosGpuLaser::LaserConnect,this),
      boost::bind( &GazeboRosGpuLaser::LaserDisconnect,this),
      ros::VoidPtr(), &this->queue_);
  else
    laser_scan_ao = ros::AdvertiseOptions::create<sensor_msgs::LaserScan>(
      this->laser_topic_name_,1,
      boost::bind( &GazeboRosGpuLaser::LaserConnect,this),
      boost::bind( &GazeboRosGpuLaser::LaserDisconnect,this),
      ros::VoidPtr(), &this->queue_);
  this->laser_scan_pub_ = this->rosnode_->advertise(laser_scan_ao);

//  std::stringstream logName;
//  logName << "/tmp/" << this->parentSensor->GetName() << "_log.txt";

//  this->timelog_.open(logName.str().c_str());

  //this->itnode_ = new image_transport::ImageTransport(*this->rosnode_);
  //this->image_pub_ = this->itnode_->advertise(
  //  "target1",1,
  //  boost::bind( &GazeboRosGpuLaser::ImageConnect,this),
  //  boost::bind( &GazeboRosGpuLaser::ImageDisconnect,this),
  //  ros::VoidPtr(), &this->queue_);

  //this->image2_pub_ = this->itnode_->advertise(
  //  "target2",1,
  //  boost::bind( &GazeboRosGpuLaser::ImageConnect,this),
  //  boost::bind( &GazeboRosGpuLaser::ImageDisconnect,this),
  //  ros::VoidPtr(), &this->queue_);

  //this->image3_pub_ = this->itnode_->advertise(
  //  "target3",1,
  //  boost::bind( &GazeboRosGpuLaser::ImageConnect,this),
  //  boost::bind( &GazeboRosGpuLaser::ImageDisconnect,this),
  //  ros::VoidPtr(), &this->queue_);

  //this->image4_pub_ = this->itnode_->advertise(
  //  "final_image",1,
  //  boost::bind( &GazeboRosGpuLaser::ImageConnect,this),
  //  boost::bind( &GazeboRosGpuLaser::ImageDisconnect,this),
  //  ros::VoidPtr(), &this->queue_);


  this->Init();
}

//void GazeboRosGpuLaser::PutCameraData(const unsigned char *_src, unsigned int w, unsigned int h, unsigned int d, image_transport::Publisher *pub_)
//{
//  // copy data into image
//  sensor_msgs::Image msg_;
//  msg_.header.frame_id = this->frame_name_;
//  msg_.header.stamp.sec = this->sensor_update_time_.sec;
//  msg_.header.stamp.nsec = this->sensor_update_time_.nsec;
//
//  /// @todo: don't bother if there are no subscribers
//  if (pub_->getNumSubscribers() > 0)
//  {
//    // copy from src to image_msg_
//    fillImage(msg_,
//        sensor_msgs::image_encodings::RGB8,
//        h,
//        w,
//        d*w,
//        (void*)_src );
//
//    // publish to ros
//    pub_->publish(msg_);
//  }
//}
//
//////////////////////////////////////////////////////////////////////////////////
//// Increment count
//void GazeboRosGpuLaser::ImageConnect()
//{
//  this->imageConnectCount++;
//  this->parentSensor->SetActive(true);
//}
//////////////////////////////////////////////////////////////////////////////////
//// Decrement count
//void GazeboRosGpuLaser::ImageDisconnect()
//{
//  this->imageConnectCount--;
//  if (this->imageConnectCount <= 0)
//    this->parentSensor->SetActive(false);
//}
//
//////////////////////////////////////////////////////////////////////////////////
//// Update the controller
//void GazeboRosGpuLaser::OnNewImageFrame(const unsigned char *_image,
//    unsigned int _width, unsigned int _height, unsigned int _depth,
//    unsigned int cam)
//{
//  //ROS_ERROR("camera_ new frame %s %s",this->parentSensor_->GetName().c_str(),this->frame_name_.c_str());
//  this->sensor_update_time_ = this->parentSensor->GetLastUpdateTime();
//
//  if (!this->parentSensor->IsActive())
//  {
//    if (this->imageConnectCount > 0)
//      // do this first so there's chance for sensor to run 1 frame after activate
//      this->parentSensor->SetActive(true);
//  }
//  else
//  {
//    if (this->imageConnectCount > 0)
//    {
//      switch (cam)
//      {
//        case 1: this->PutCameraData(_image, _width, _height, _depth, &this->image_pub_);break;
//        case 2: this->PutCameraData(_image, _width, _height, _depth, &this->image2_pub_);break;
//        case 3: this->PutCameraData(_image, _width, _height, _depth, &this->image3_pub_);break;
//        case 4: this->PutCameraData(_image, _width, _height, _depth, &this->image4_pub_);break;
//      }
//    }
//  }
//}

////////////////////////////////////////////////////////////////////////////////
void GazeboRosGpuLaser::Init()
{
  // set parent sensor update rate
  this->parentSensor->SetUpdateRate(this->update_rate_);

  // prepare to throttle this plugin at the same rate
  // ideally, we should invoke a plugin update when the sensor updates,
  // have to think about how to do that properly later
  if (this->update_rate_ > 0.0)
    this->update_period_ = 1.0/this->update_rate_;
  else
    this->update_period_ = 0.0;

  // start custom queue for camera_
  this->callback_queue_thread_ = boost::thread( boost::bind( &GazeboRosGpuLaser::QueueThread,this ) );

  this->last_publish_ = ros::WallTime::now();
  // this->logCount_ = 0;
}

////////////////////////////////////////////////////////////////////////////////
// Increment count
void GazeboRosGpuLaser::LaserConnect()
{
  this->laser_connect_count_++;
  this->parentSensor->SetActive(true);
}
////////////////////////////////////////////////////////////////////////////////
// Decrement count
void GazeboRosGpuLaser::LaserDisconnect()
{
  this->laser_connect_count_--;

  if (this->laser_connect_count_ <= 0)
    this->parentSensor->SetActive(false);
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosGpuLaser::OnNewLaserFrame(const float *_image,
    unsigned int _width, unsigned int _height, unsigned int _depth,
    const std::string &_format)
{
  this->sensor_update_time_ = this->parentSensor->GetLastUpdateTime();
  if (!this->parentSensor->IsActive())
  {
    if (this->laser_connect_count_ > 0)
      // do this first so there's chance for sensor to run 1 frame after activate
      this->parentSensor->SetActive(true);
  }
  else
  {
    if (this->laser_connect_count_ > 0)
    {
      if (this->parentSensor->GetVerticalRangeCount() == 1)
        PublishLaserScan(_image, width);
      else
        PublishPointCloud(_image, width, height);
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
void GazeboRosGpuLaser::PublishLaserScan(const float *_scan,
    unsigned int _width)
{
  math::Angle maxAngle = this->parentSensor->GetAngleMax();
  math::Angle minAngle = this->parentSensor->GetAngleMin();

  this->laser_scan_msg_.header.frame_id = this->frame_name_;
  this->laser_scan_msg_.header.stamp.sec = this->sensor_update_time_.sec;
  this->laser_scan_msg_.header.stamp.nsec = this->sensor_update_time_.nsec;

  this->laser_scan_msg_.angle_min = minAngle.Radian();
  this->laser_scan_msg_.angle_max = maxAngle.Radian();
  this->laser_scan_msg_.angle_increment = (maxAngle.Radian() - minAngle.Radian())/((double)(_width -1)); // for computing yaw
  this->laser_scan_msg_.time_increment  = 0; // instantaneous simulator scan
  this->laser_scan_msg_.scan_time       = 0; // FIXME: what's this?
  this->laser_scan_msg_.range_min = this->parentSensor->GetRangeMin();
  this->laser_scan_msg_.range_max = this->parentSensor->GetRangeMax();
  this->laser_scan_msg_.ranges.clear();
  this->laser_scan_msg_.intensities.clear();

  for(unsigned int i=0; i < width; i++)
  {
    float range = _scan[3 * i];
    if (range < this->parentSensor->GetRangeMin())
      range = this->parentSensor->GetRangeMax();
    this->laser_scan_msg_.ranges.push_back(range + this->GaussianKernel(0,this->gaussian_noise_));
    this->laser_scan_msg_.intensities.push_back(_scan[3*i+1]);
  }

//  unsigned int logLimit = 101;

//  if (this->logCount_ < logLimit)
//  {
//    ros::WallTime curr_time = ros::WallTime::now();
//    ros::WallDuration duration = curr_time - this->last_publish_;
//    if (this->logCount_ != 0)
//      this->timelog_ << (duration.toSec() * 1000) << "\n";
//    ROS_WARN("%f", duration.toSec() * 1000);
    this->laser_scan_pub_.publish(this->laser_scan_msg_);
//    this->last_publish_ = curr_time;
//    this->logCount_++;
//  }
//  else
//    if (this->logCount_ == logLimit)
//      this->timelog_.close();

}

////////////////////////////////////////////////////////////////////////////////
void GazeboRosGpuLaser::PublishPointCloud(const float *_scan,
    unsigned int _width, unsigned int _height)
{
  this->point_cloud_msg_.header.frame_id = this->frame_name_;
  this->point_cloud_msg_.header.stamp.sec = this->sensor_update_time_.sec;
  this->point_cloud_msg_.header.stamp.nsec = this->sensor_update_time_.nsec;

  this->point_cloud_msg_.points.clear();
  this->point_cloud_msg_.is_dense = true;

  math::Angle maxAngle = this->parentSensor->GetAngleMax();
  math::Angle minAngle = this->parentSensor->GetAngleMin();

  math::Angle verticalMaxAngle = this->parentSensor->GetVerticalAngleMax();
  math::Angle verticalMinAngle = this->parentSensor->GetVerticalAngleMin();

  double dH = (maxAngle.Radian() - minAngle.Radian()) / (_width - 1) ;
  double dV = (verticalMaxAngle.Radian() - verticalMinAngle.Radian()) / (_height - 1);

  double alpha = ((minAngle + maxAngle) / 2.0).Radian();

  for (unsigned int j = 0; j < _height; j++)
    for (unsigned int i = 0; i < _width; i++)
    {
      double hAngle = (i * dH) + minAngle.Radian() - alpha;
      double vAngle = (j * dV) + verticalMinAngle.Radian();
      float r = _scan[3 * (i + j * _width)];

      pcl::PointXYZI pr;

      if ((r < this->parentSensor->GetRangeMin()) ||
          (r >= this->parentSensor->GetRangeMax()))
        r = this->parentSensor->GetRangeMax();

      pcl::PointXYZI p;

      if (this->parentSensor->IsHorizontal())
      {
        p.x = r * cos(vAngle) * cos(hAngle) + this->GaussianKernel(0,this->gaussian_noise_);
        p.y = r *               sin(hAngle) + this->GaussianKernel(0,this->gaussian_noise_);
        p.z = r * sin(vAngle) * cos(hAngle) + this->GaussianKernel(0,this->gaussian_noise_);
      }
      else
      {
        p.x = r * cos(vAngle) * cos(hAngle) + this->GaussianKernel(0,this->gaussian_noise_);
        p.y = r * cos(vAngle) * sin(hAngle) + this->GaussianKernel(0,this->gaussian_noise_);
        p.z = r * sin(vAngle) + this->GaussianKernel(0,this->gaussian_noise_);
      }

      pr.x = cos(alpha)*p.x - sin(alpha)*p.y;
      pr.y = sin(alpha)*p.x + cos(alpha)*p.y;
      pr.z = p.z;
      pr.intensity = _scan[3 * (i + j * _width) + 1];

      this->point_cloud_msg_.points.push_back(pr);
    }

    //std::cerr<<"--------------------------------\n";

//  if (this->logCount_ < 10001)
//  {
//    ros::WallTime curr_time = ros::WallTime::now();
//    ros::WallDuration duration = curr_time - this->last_publish_;
//    if (this->logCount_ != 0)
//      this->timelog_ << duration << "\n";
    this->laser_scan_pub_.publish(this->point_cloud_msg_);
//    this->last_publish_ = curr_time;
//  }
}


//////////////////////////////////////////////////////////////////////////////
// Utility for adding noise
double GazeboRosGpuLaser::GaussianKernel(double mu,double sigma)
{
  // using Box-Muller transform to generate two independent standard normally disbributed normal variables
  // see wikipedia
  double U = (double)rand()/(double)RAND_MAX; // normalized uniform random variable
  double V = (double)rand()/(double)RAND_MAX; // normalized uniform random variable
  double X = sqrt(-2.0 * ::log(U)) * cos( 2.0*M_PI * V);
  //double Y = sqrt(-2.0 * ::log(U)) * sin( 2.0*M_PI * V); // the other indep. normal variable
  // we'll just use X
  // scale to our mu and sigma
  X = sigma * X + mu;
  return X;
}

////////////////////////////////////////////////////////////////////////////////
// Put camera_ data to the interface
void GazeboRosGpuLaser::QueueThread()
{
  static const double timeout = 0.001;

  while (this->rosnode_->ok())
  {
    /// take care of callback queue
    this->queue_.callAvailable(ros::WallDuration(timeout));
  }
}

// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboRosGpuLaser)

}
