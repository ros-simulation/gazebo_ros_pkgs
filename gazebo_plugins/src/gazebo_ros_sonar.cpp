/*
 * Software License Agreement (Modified BSD License)
 *
 *  Copyright (c) 2013-2015, PAL Robotics, S.L.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PAL Robotics, S.L. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/** \author Jose Capriles, Bence Magyar, Moussab Bennehar. */

#include <algorithm>
#include <string>
#include <assert.h>

#include <gazebo/physics/World.hh>
#include <gazebo/physics/HingeJoint.hh>
#include <gazebo/sensors/Sensor.hh>
#include <sdf/sdf.hh>
#include <sdf/Param.hh>
#include <gazebo/common/Exception.hh>
#include <gazebo/sensors/SonarSensor.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/transport/transport.hh>

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include "gazebo_plugins/gazebo_ros_sonar.h"

namespace gazebo
{
// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboRosSonar)

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosSonar::GazeboRosSonar()
{
  this->seed = 0;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosSonar::~GazeboRosSonar()
{
  this->rosnode_->shutdown();
  delete this->rosnode_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosSonar::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{
  // load plugin
  SonarPlugin::Load(_parent, this->sdf);
  // Get the world name.
  std::string worldName = _parent->WorldName();
  this->world_ = physics::get_world(worldName);
  // save pointers
  this->sdf = _sdf;

  this->last_update_time_ = common::Time(0);

  GAZEBO_SENSORS_USING_DYNAMIC_POINTER_CAST;
  this->parent_sonar_sensor_ =      
    dynamic_pointer_cast<sensors::SonarSensor>(_parent);

  if (!this->parent_sonar_sensor_)
    gzthrow("GazeboRosSonar controller requires a Sonar Sensor as its parent");

  this->robot_namespace_ =  GetRobotNamespace(_parent, _sdf, "Sonar");
    
  if (!this->sdf->HasElement("frameName"))
  {
    ROS_INFO_NAMED("range", "Range plugin missing <frameName>, defaults to /world");
    this->frame_name_ = "/world";
  }
  else
    this->frame_name_ = this->sdf->Get<std::string>("frameName");

  ROS_INFO("#########    frame name : %s  #############", this->frame_name_.c_str());

  if (!this->sdf->HasElement("topicName"))
  {
    ROS_INFO_NAMED("sonar", "Sonar plugin missing <topicName>, defaults to /sonar");
    this->topic_name_ = "/sonar";
  }
  else
    this->topic_name_ = this->sdf->Get<std::string>("topicName");

  if (!this->sdf->HasElement("fov"))
  {
      ROS_WARN_NAMED("sonar", "Sonar plugin missing <fov>, defaults to 0.05");
      this->fov_ = 0.05;
  }
  else
      this->fov_ = _sdf->GetElement("fov")->Get<double>();

  if (!this->sdf->HasElement("gaussianNoise"))
  {
    ROS_INFO_NAMED("sonar", "Sonar plugin missing <gaussianNoise>, defaults to 0.0");
    this->gaussian_noise_ = 0;
  }
  else
    this->gaussian_noise_ = this->sdf->Get<double>("gaussianNoise");

  this->sonar_connect_count_ = 0;

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM_NAMED("sonar", "A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  ROS_INFO_NAMED("sonar", "Starting Sonar Plugin (ns = %s)", this->robot_namespace_.c_str() );
  // ros callback queue for processing subscription
  this->deferred_load_thread_ = boost::thread(
    boost::bind(&GazeboRosSonar::LoadThread, this));
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosSonar::LoadThread()
{
  this->gazebo_node_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
  this->gazebo_node_->Init(this->world_name_);

  this->pmq.startServiceThread();

  this->rosnode_ = new ros::NodeHandle(this->robot_namespace_);


  this->tf_prefix_ = tf::getPrefixParam(*this->rosnode_);
  if(this->tf_prefix_.empty()) {
      this->tf_prefix_ = this->robot_namespace_;
      boost::trim_right_if(this->tf_prefix_,boost::is_any_of("/"));
  }
  ROS_INFO_NAMED("sonar", "Sonar Plugin (ns = %s)  <tf_prefix_>, set to \"%s\"",
             this->robot_namespace_.c_str(), this->tf_prefix_.c_str());

  // resolve tf prefix
  this->frame_name_ = tf::resolve(this->tf_prefix_, this->frame_name_);

  if (this->topic_name_ != "")
  {
    ros::AdvertiseOptions ao =
      ros::AdvertiseOptions::create<sensor_msgs::Range>(
      this->topic_name_, 1,
      boost::bind(&GazeboRosSonar::SonarConnect, this),
      boost::bind(&GazeboRosSonar::SonarDisconnect, this),
      ros::VoidPtr(), NULL);
    this->pub_ = this->rosnode_->advertise(ao);
    this->pub_queue_ = this->pmq.addPub<sensor_msgs::Range>();
  }

  // Initialize the controller

  // sensor generation off by default
  this->parent_sonar_sensor_->SetActive(false);
}

////////////////////////////////////////////////////////////////////////////////
// Increment count
void GazeboRosSonar::SonarConnect()
{
  this->sonar_connect_count_++;
  // this->parent_sonar_sensor_->SetActive(true);
    if (this->sonar_connect_count_ == 1)
  this->sonar_sub_ =
    this->gazebo_node_->Subscribe(this->parent_sonar_sensor_->Topic(),
                                  &GazeboRosSonar::OnScan, this);
}
////////////////////////////////////////////////////////////////////////////////
// Decrement count
void GazeboRosSonar::SonarDisconnect()
{
  this->sonar_connect_count_--;
  if (this->sonar_connect_count_ == 0)
    this->sonar_sub_.reset();
}


////////////////////////////////////////////////////////////////////////////////
// Update the plugin
void GazeboRosSonar::OnScan(ConstSonarStampedPtr &_msg)
{
  sensor_msgs::Range range_msg_;
  range_msg_.header.stamp = ros::Time(_msg->time().sec(), _msg->time().nsec());
  range_msg_.header.frame_id = this->frame_name_;

  range_msg_.radiation_type = sensor_msgs::Range::ULTRASOUND;

  range_msg_.field_of_view = fov_;
  range_msg_.max_range = this->parent_sonar_sensor_->RangeMax();
  range_msg_.min_range = this->parent_sonar_sensor_->RangeMin();

  range_msg_.range = _msg->sonar().range();
  if (range_msg_.range < range_msg_.max_range)
    range_msg_.range = std::min(range_msg_.range + this->GaussianKernel(0,gaussian_noise_), parent_sonar_sensor_->RangeMax());

  this->pub_queue_->push(range_msg_, this->pub_);

}

//////////////////////////////////////////////////////////////////////////////
// Utility for adding noise
double GazeboRosSonar::GaussianKernel(double mu, double sigma)
{
  // using Box-Muller transform to generate two independent standard
  // normally disbributed normal variables see wikipedia

  // normalized uniform random variable
  double U = static_cast<double>(rand_r(&this->seed)) /
             static_cast<double>(RAND_MAX);

  // normalized uniform random variable
  double V = static_cast<double>(rand_r(&this->seed)) /
             static_cast<double>(RAND_MAX);

  double X = sqrt(-2.0 * ::log(U)) * cos(2.0*M_PI * V);
  // double Y = sqrt(-2.0 * ::log(U)) * sin(2.0*M_PI * V);

  // there are 2 indep. vars, we'll just use X
  // scale to our mu and sigma
  X = sigma * X + mu;
  return X;
}
}
