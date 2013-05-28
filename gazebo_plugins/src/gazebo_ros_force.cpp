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
   Desc: GazeboRosForce plugin for manipulating objects in Gazebo
   Author: John Hsu
   Date: 24 Sept 2008
   SVN info: $Id$
 @htmlinclude manifest.html
 @b GazeboRosForce plugin reads ROS geometry_msgs/Wrench messages
 */

#include <algorithm>
#include <assert.h>

#include <gazebo_plugins/gazebo_ros_force.h>

namespace gazebo
{

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosForce::GazeboRosForce()
{
  this->wrench_msg_.force.x = 0;
  this->wrench_msg_.force.y = 0;
  this->wrench_msg_.force.z = 0;
  this->wrench_msg_.torque.x = 0;
  this->wrench_msg_.torque.y = 0;
  this->wrench_msg_.torque.z = 0;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosForce::~GazeboRosForce()
{
  event::Events::DisconnectWorldUpdateStart(this->update_connection_);

  // Custom Callback Queue
  this->queue_.clear();
  this->queue_.disable();
  this->rosnode_->shutdown();
  this->callback_queue_thread_.join();

  delete this->rosnode_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosForce::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  // Get the world name.
  this->world_ = _model->GetWorld();
  
  // load parameters
  this->robot_namespace_ = "";
  if (_sdf->HasElement("robotNamespace"))
    this->robot_namespace_ = _sdf->GetElement("robotNamespace")->GetValueString() + "/";

  if (!_sdf->HasElement("bodyName"))
  {
    ROS_FATAL("f3d plugin missing <bodyName>, cannot proceed");
    return;
  }
  else
    this->link_name_ = _sdf->GetElement("bodyName")->GetValueString();

  this->link_ = _model->GetLink(this->link_name_);
  if (!this->link_)
  {
    ROS_FATAL("gazebo_ros_f3d plugin error: link named: %s does not exist\n",this->link_name_.c_str());
    return;
  }

  if (!_sdf->HasElement("topicName"))
  {
    ROS_FATAL("f3d plugin missing <topicName>, cannot proceed");
    return;
  }
  else
    this->topic_name_ = _sdf->GetElement("topicName")->GetValueString();


  // initialize ros if not done so already
  if (!ros::isInitialized())
  {
    ROS_FATAL("while loading gazebo_ros_force plugin, ros is not initialized, please load a gazebo system plugin that initializes ros (e.g. libgazebo_ros_api_plugin.so from gazebo ros package)\n");
    return;
  }

  this->rosnode_ = new ros::NodeHandle(this->robot_namespace_);

  // Custom Callback Queue
  ros::SubscribeOptions so = ros::SubscribeOptions::create<geometry_msgs::Wrench>(
    this->topic_name_,1,
    boost::bind( &GazeboRosForce::UpdateObjectForce,this,_1),
    ros::VoidPtr(), &this->queue_);
  this->sub_ = this->rosnode_->subscribe(so);

  // Custom Callback Queue
  this->callback_queue_thread_ = boost::thread( boost::bind( &GazeboRosForce::QueueThread,this ) );

  // New Mechanism for Updating every World Cycle
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->update_connection_ = event::Events::ConnectWorldUpdateStart(
      boost::bind(&GazeboRosForce::UpdateChild, this));
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosForce::UpdateObjectForce(const geometry_msgs::Wrench::ConstPtr& _msg)
{
  this->wrench_msg_.force.x = _msg->force.x;
  this->wrench_msg_.force.y = _msg->force.y;
  this->wrench_msg_.force.z = _msg->force.z;
  this->wrench_msg_.torque.x = _msg->torque.x;
  this->wrench_msg_.torque.y = _msg->torque.y;
  this->wrench_msg_.torque.z = _msg->torque.z;
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosForce::UpdateChild()
{
  this->lock_.lock();
  math::Vector3 force(this->wrench_msg_.force.x,this->wrench_msg_.force.y,this->wrench_msg_.force.z);
  math::Vector3 torque(this->wrench_msg_.torque.x,this->wrench_msg_.torque.y,this->wrench_msg_.torque.z);
  this->link_->AddForce(force);
  this->link_->AddTorque(torque);
  this->lock_.unlock();
}

// Custom Callback Queue
////////////////////////////////////////////////////////////////////////////////
// custom callback queue thread
void GazeboRosForce::QueueThread()
{
  static const double timeout = 0.01;

  while (this->rosnode_->ok())
  {
    this->queue_.callAvailable(ros::WallDuration(timeout));
  }
}

GZ_REGISTER_MODEL_PLUGIN(GazeboRosForce);

}
