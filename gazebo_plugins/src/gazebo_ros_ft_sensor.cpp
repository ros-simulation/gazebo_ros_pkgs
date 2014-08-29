/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
/*
 * Desc: Force Torque Sensor Plugin
 * Author: Francisco Suarez-Ruiz
 * Date: 5 June 2014
 */

#include <gazebo_plugins/gazebo_ros_ft_sensor.h>
#include <tf/tf.h>

namespace gazebo
{
GZ_REGISTER_MODEL_PLUGIN(GazeboRosFT);

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosFT::GazeboRosFT()
{
  this->ft_connect_count_ = 0;
  this->seed = 0;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosFT::~GazeboRosFT()
{
  event::Events::DisconnectWorldUpdateBegin(this->update_connection_);
  // Custom Callback Queue
  this->queue_.clear();
  this->queue_.disable();
  this->rosnode_->shutdown();
  this->callback_queue_thread_.join();
  delete this->rosnode_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosFT::Load( physics::ModelPtr _model, sdf::ElementPtr _sdf )
{
  // Save pointers
  this->model_ = _model;
  this->world_ = this->model_->GetWorld();
  
  // load parameters
  this->robot_namespace_ = "";
  if (_sdf->HasElement("robotNamespace"))
    this->robot_namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";

  if (!_sdf->HasElement("jointName"))
  {
    ROS_FATAL("ft_sensor plugin missing <jointName>, cannot proceed");
    return;
  }
  else
    this->joint_name_ = _sdf->GetElement("jointName")->Get<std::string>();

  this->joint_ = this->model_->GetJoint(this->joint_name_);
  if (!this->joint_)
  {
    ROS_FATAL("gazebo_ros_ft_sensor plugin error: jointName: %s does not exist\n",this->joint_name_.c_str());
    return;
  }
  
  this->parent_link_ = this->joint_->GetParent();
  this->child_link_ = this->joint_->GetChild();
  this->frame_name_ = this->child_link_->GetName();
  
  ROS_INFO("ft_sensor plugin reporting wrench values to the frame [%s]", this->frame_name_.c_str());

  if (!_sdf->HasElement("topicName"))
  {
    ROS_FATAL("ft_sensor plugin missing <topicName>, cannot proceed");
    return;
  }
  else
    this->topic_name_ = _sdf->GetElement("topicName")->Get<std::string>();
  
  if (!_sdf->HasElement("gaussianNoise"))
  {
    ROS_INFO("imu plugin missing <gaussianNoise>, defaults to 0.0");
    this->gaussian_noise_ = 0.0;
  }
  else
    this->gaussian_noise_ = _sdf->Get<double>("gaussianNoise");
  
  if (!_sdf->HasElement("updateRate"))
  {
    ROS_DEBUG("ft_sensor plugin missing <updateRate>, defaults to 0.0"
             " (as fast as possible)");
    this->update_rate_ = 0;
  }
  else
    this->update_rate_ = _sdf->GetElement("updateRate")->Get<double>();

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  this->rosnode_ = new ros::NodeHandle(this->robot_namespace_);
  
  // resolve tf prefix
  std::string prefix;
  this->rosnode_->getParam(std::string("tf_prefix"), prefix);
  this->frame_name_ = tf::resolve(prefix, this->frame_name_);

  // Custom Callback Queue
  ros::AdvertiseOptions ao = ros::AdvertiseOptions::create<geometry_msgs::WrenchStamped>(
    this->topic_name_,1,
    boost::bind( &GazeboRosFT::FTConnect,this),
    boost::bind( &GazeboRosFT::FTDisconnect,this), ros::VoidPtr(), &this->queue_);
  this->pub_ = this->rosnode_->advertise(ao);
  
  // Custom Callback Queue
  this->callback_queue_thread_ = boost::thread( boost::bind( &GazeboRosFT::QueueThread,this ) );
  
  // New Mechanism for Updating every World Cycle
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GazeboRosFT::UpdateChild, this));
}

////////////////////////////////////////////////////////////////////////////////
// Someone subscribes to me
void GazeboRosFT::FTConnect()
{
  this->ft_connect_count_++;
}

////////////////////////////////////////////////////////////////////////////////
// Someone subscribes to me
void GazeboRosFT::FTDisconnect()
{
  this->ft_connect_count_--;
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosFT::UpdateChild()
{
  common::Time cur_time = this->world_->GetSimTime();

  // rate control
  if (this->update_rate_ > 0 &&
      (cur_time-this->last_time_).Double() < (1.0/this->update_rate_))
    return;
    
  if (this->ft_connect_count_ == 0)
    return;

  physics::JointWrench wrench;
  math::Vector3 torque;
  math::Vector3 force;

  // FIXME: Should include options for diferent frames and measure directions
  // E.g: https://bitbucket.org/osrf/gazebo/raw/default/gazebo/sensors/ForceTorqueSensor.hh
  // Get force torque at the joint
  // The wrench is reported in the CHILD <frame>
  // The <measure_direction> is child_to_parent
  wrench = this->joint_->GetForceTorque(0);
  force = wrench.body2Force;
  torque = wrench.body2Torque;


  this->lock_.lock();
  // copy data into wrench message
  this->wrench_msg_.header.frame_id = this->frame_name_;
  this->wrench_msg_.header.stamp.sec = (this->world_->GetSimTime()).sec;
  this->wrench_msg_.header.stamp.nsec = (this->world_->GetSimTime()).nsec;

  this->wrench_msg_.wrench.force.x = force.x + this->GaussianKernel(0, this->gaussian_noise_);
  this->wrench_msg_.wrench.force.y = force.y + this->GaussianKernel(0, this->gaussian_noise_);
  this->wrench_msg_.wrench.force.z = force.z + this->GaussianKernel(0, this->gaussian_noise_);
  this->wrench_msg_.wrench.torque.x = torque.x + this->GaussianKernel(0, this->gaussian_noise_);
  this->wrench_msg_.wrench.torque.y = torque.y + this->GaussianKernel(0, this->gaussian_noise_);
  this->wrench_msg_.wrench.torque.z = torque.z + this->GaussianKernel(0, this->gaussian_noise_);

  this->pub_.publish(this->wrench_msg_);
  this->lock_.unlock();
  
  // save last time stamp
  this->last_time_ = cur_time;
}

//////////////////////////////////////////////////////////////////////////////
// Utility for adding noise
double GazeboRosFT::GaussianKernel(double mu, double sigma)
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

// Custom Callback Queue
////////////////////////////////////////////////////////////////////////////////
// custom callback queue thread
void GazeboRosFT::QueueThread()
{
  static const double timeout = 0.01;

  while (this->rosnode_->ok())
  {
    this->queue_.callAvailable(ros::WallDuration(timeout));
  }
}

}
