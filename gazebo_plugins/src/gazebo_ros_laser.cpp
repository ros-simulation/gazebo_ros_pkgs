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
 * Desc: Ros Laser controller.
 * Author: Nathan Koenig
 * Date: 01 Feb 2007
 * SVN info: $Id: gazebo_ros_laser.cpp 6683 2008-06-25 19:12:30Z natepak $
 */

#include <algorithm>
#include <assert.h>

#include <gazebo_plugins/gazebo_ros_laser.h>

#include "physics/World.hh"
#include "physics/HingeJoint.hh"
#include "sensors/Sensor.hh"
#include "sdf/interface/SDF.hh"
#include "sdf/interface/Param.hh"
#include "common/Exception.hh"
#include "sensors/RaySensor.hh"
#include "sensors/SensorTypes.hh"
#include "tf/tf.h"

namespace gazebo
{

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosLaser::GazeboRosLaser()
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosLaser::~GazeboRosLaser()
{
  this->laser_queue_.clear();
  this->laser_queue_.disable();
  this->rosnode_->shutdown();
  this->callback_queue_thread_.join();

  delete this->rosnode_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosLaser::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{
  // load plugin
  RayPlugin::Load(_parent, _sdf);

  // Get then name of the parent sensor
  this->parent_sensor_ = _parent;

  // Get the world name.
  std::string worldName = _parent->GetWorldName();
  this->world_ = physics::get_world(worldName);

  this->last_update_time_ = common::Time(0);

  this->parent_ray_sensor_ = boost::shared_dynamic_cast<sensors::RaySensor>(this->parent_sensor_);

  if (!this->parent_ray_sensor_)
    gzthrow("GazeboRosLaser controller requires a Ray Sensor as its parent");

  this->robot_namespace_ = "";
  if (_sdf->HasElement("robotNamespace"))
    this->robot_namespace_ = _sdf->GetElement("robotNamespace")->GetValueString() + "/";

  if (!_sdf->HasElement("frameName"))
  {
    ROS_INFO("Laser plugin missing <frameName>, defaults to /world");
    this->frame_name_ = "/world";
  }
  else
    this->frame_name_ = _sdf->GetElement("frameName")->GetValueString();

  if (!_sdf->HasElement("topicName"))
  {
    ROS_INFO("Laser plugin missing <topicName>, defaults to /world");
    this->topic_name_ = "/world";
  }
  else
    this->topic_name_ = _sdf->GetElement("topicName")->GetValueString();

  if (!_sdf->HasElement("gaussianNoise"))
  {
    ROS_INFO("Laser plugin missing <gaussianNoise>, defaults to 0.0");
    this->gaussian_noise_ = 0;
  }
  else
    this->gaussian_noise_ = _sdf->GetElement("gaussianNoise")->GetValueDouble();

  if (!_sdf->HasElement("hokuyoMinIntensity"))
  {
    ROS_INFO("Laser plugin missing <hokuyoMinIntensity>, defaults to 101");
    this->hokuyo_min_intensity_ = 101;
  }
  else
    this->hokuyo_min_intensity_ = _sdf->GetElement("hokuyoMinIntensity")->GetValueDouble();

  ROS_INFO("INFO: gazebo_ros_laser plugin should set minimum intensity to %f due to cutoff in hokuyo filters." , this->hokuyo_min_intensity_);

  if (!_sdf->GetElement("updateRate"))
  {
    ROS_INFO("Laser plugin missing <updateRate>, defaults to 0");
    this->update_rate_ = 0;
  }
  else
    this->update_rate_ = _sdf->GetElement("updateRate")->GetValueDouble();

  // set parent sensor update rate
  this->parent_sensor_->SetUpdateRate(this->update_rate_);

  // prepare to throttle this plugin at the same rate
  // ideally, we should invoke a plugin update when the sensor updates,
  // have to think about how to do that properly later
  if (this->update_rate_ > 0.0)
    this->update_period_ = 1.0/this->update_rate_;
  else
    this->update_period_ = 0.0;

  this->laser_connect_count_ = 0;


  if (!ros::isInitialized())
  {
    ROS_FATAL("while loading gazebo_ros_force plugin, ros is not initialized, please load a gazebo system plugin that initializes ros (e.g. libgazebo_ros_api_plugin.so from gazebo ros package)\n");
    return;
  }

  this->rosnode_ = new ros::NodeHandle(this->robot_namespace_);

  // resolve tf prefix
  std::string prefix;
  this->rosnode_->getParam(std::string("tf_prefix"), prefix);
  this->frame_name_ = tf::resolve(prefix, this->frame_name_);

  if (this->topic_name_ != "")
  {
    ros::AdvertiseOptions ao = ros::AdvertiseOptions::create<sensor_msgs::LaserScan>(
      this->topic_name_,1,
      boost::bind( &GazeboRosLaser::LaserConnect,this),
      boost::bind( &GazeboRosLaser::LaserDisconnect,this), ros::VoidPtr(), &this->laser_queue_);
    this->pub_ = this->rosnode_->advertise(ao);
  }


  // Initialize the controller

  // sensor generation off by default
  this->parent_ray_sensor_->SetActive(false);
  // start custom queue for laser
  this->callback_queue_thread_ = boost::thread( boost::bind( &GazeboRosLaser::LaserQueueThread,this ) );

}

////////////////////////////////////////////////////////////////////////////////
// Increment count
void GazeboRosLaser::LaserConnect()
{
  this->laser_connect_count_++;
  this->parent_ray_sensor_->SetActive(true);
}
////////////////////////////////////////////////////////////////////////////////
// Decrement count
void GazeboRosLaser::LaserDisconnect()
{
  this->laser_connect_count_--;

  if (this->laser_connect_count_ == 0)
    this->parent_ray_sensor_->SetActive(false);
}


////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosLaser::OnNewLaserScans()
{
  if (this->topic_name_ != "")
  {
    common::Time cur_time = this->world_->GetSimTime();
    if (cur_time - this->last_update_time_ >= this->update_period_)
    {
      common::Time sensor_update_time = this->parent_sensor_->GetLastUpdateTime();
      this->PutLaserData(sensor_update_time);
      this->last_update_time_ = cur_time;
    }
  }
  else
  {
    ROS_INFO("gazebo_ros_laser topic name not set");
  }
}

////////////////////////////////////////////////////////////////////////////////
// Put laser data to the interface
void GazeboRosLaser::PutLaserData(common::Time &_updateTime)
{
  int i, ja, jb;
  double ra, rb, r, b;
  double intensity;

  this->parent_ray_sensor_->SetActive(false);

  math::Angle maxAngle = this->parent_ray_sensor_->GetAngleMax();
  math::Angle minAngle = this->parent_ray_sensor_->GetAngleMin();

  double maxRange = this->parent_ray_sensor_->GetRangeMax();
  double minRange = this->parent_ray_sensor_->GetRangeMin();
  int rayCount = this->parent_ray_sensor_->GetRayCount();
  int rangeCount = this->parent_ray_sensor_->GetRangeCount();

  /***************************************************************/
  /*                                                             */
  /*  point scan from laser                                      */
  /*                                                             */
  /***************************************************************/
  this->lock_.lock();
  // Add Frame Name
  this->laser_msg_.header.frame_id = this->frame_name_;
  this->laser_msg_.header.stamp.sec = _updateTime.sec;
  this->laser_msg_.header.stamp.nsec = _updateTime.nsec;


  double tmp_res_angle = (maxAngle.Radian() - minAngle.Radian())/((double)(rangeCount -1)); // for computing yaw
  this->laser_msg_.angle_min = minAngle.Radian();
  this->laser_msg_.angle_max = maxAngle.Radian();
  this->laser_msg_.angle_increment = tmp_res_angle;
  this->laser_msg_.time_increment  = 0; // instantaneous simulator scan
  this->laser_msg_.scan_time       = 0; // FIXME: what's this?
  this->laser_msg_.range_min = minRange;
  this->laser_msg_.range_max = maxRange;
  this->laser_msg_.ranges.clear();
  this->laser_msg_.intensities.clear();

  // Interpolate the range readings from the rays
  for (i = 0; i<rangeCount; i++)
  {
    b = (double) i * (rayCount - 1) / (rangeCount - 1);
    ja = (int) floor(b);
    jb = std::min(ja + 1, rayCount - 1);
    b = b - floor(b);

    assert(ja >= 0 && ja < rayCount);
    assert(jb >= 0 && jb < rayCount);

    ra = std::min(this->parent_ray_sensor_->GetLaserShape()->GetRange(ja) , maxRange-minRange); // length of ray
    rb = std::min(this->parent_ray_sensor_->GetLaserShape()->GetRange(jb) , maxRange-minRange); // length of ray

    // Range is linear interpolation if values are close,
    // and min if they are very different
    //if (fabs(ra - rb) < 0.10)
      r = (1 - b) * ra + b * rb;
    //else r = std::min(ra, rb);

    // Intensity is averaged
    intensity = 0.5*( this->parent_ray_sensor_->GetLaserShape()->GetRetro(ja)
                    + this->parent_ray_sensor_->GetLaserShape()->GetRetro(jb));

    /***************************************************************/
    /*                                                             */
    /*  point scan from laser                                      */
    /*                                                             */
    /***************************************************************/
    this->laser_msg_.ranges.push_back(std::min(r + minRange + this->GaussianKernel(0,this->gaussian_noise_), maxRange));
    this->laser_msg_.intensities.push_back(std::max(this->hokuyo_min_intensity_,intensity + this->GaussianKernel(0,this->gaussian_noise_)));
  }

  this->parent_ray_sensor_->SetActive(true);

  // send data out via ros message
  if (this->laser_connect_count_ > 0 && this->topic_name_ != "")
      this->pub_.publish(this->laser_msg_);

  this->lock_.unlock();

}

//////////////////////////////////////////////////////////////////////////////
// Utility for adding noise
double GazeboRosLaser::GaussianKernel(double mu,double sigma)
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
// Put laser data to the interface
void GazeboRosLaser::LaserQueueThread()
{
  static const double timeout = 0.01;

  while (this->rosnode_->ok())
  {
    this->laser_queue_.callAvailable(ros::WallDuration(timeout));
  }
}

// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboRosLaser)

}
