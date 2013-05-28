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
   Desc: Force Feed Back Ground Truth
   Author: Sachin Chitta and John Hsu
   Date: 1 June 2008
   SVN info: $Id$
 @htmlinclude manifest.html
 @b GazeboRosF3D plugin broadcasts forces acting on the body specified by name.
 */

#include <gazebo_plugins/gazebo_ros_f3d.h>

#include "tf/tf.h"

namespace gazebo
{

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosF3D::GazeboRosF3D()
{
  this->f3d_connect_count_ = 0;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosF3D::~GazeboRosF3D()
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
void GazeboRosF3D::Load( physics::ModelPtr _parent, sdf::ElementPtr _sdf )
{
  // Get the world name.
  this->world_ = _parent->GetWorld();
  
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

  this->link_ = _parent->GetLink(this->link_name_);
  if (!this->link_)
  {
    ROS_FATAL("gazebo_ros_f3d plugin error: bodyName: %s does not exist\n",this->link_name_.c_str());
    return;
  }

  if (!_sdf->HasElement("topicName"))
  {
    ROS_FATAL("f3d plugin missing <topicName>, cannot proceed");
    return;
  }
  else
    this->topic_name_ = _sdf->GetElement("topicName")->GetValueString();

  if (!_sdf->HasElement("frameName"))
  {
    ROS_INFO("f3d plugin missing <frameName>, defaults to world");
    this->frame_name_ = "world";
  }
  else
  {
    this->frame_name_ = _sdf->GetElement("frameName")->GetValueString();
    // todo: frameName not used
    ROS_INFO("f3d plugin specifies <frameName> [%s], not used, default to world",this->frame_name_.c_str());
  }
  
  if (!ros::isInitialized())
  {
    int argc = 0;
    char** argv = NULL;
    ros::init(argc,argv,"gazebo",ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
  }

  this->rosnode_ = new ros::NodeHandle(this->robot_namespace_);
  
  // resolve tf prefix
  std::string prefix;
  this->rosnode_->getParam(std::string("tf_prefix"), prefix);
  this->frame_name_ = tf::resolve(prefix, this->frame_name_);

  // Custom Callback Queue
  ros::AdvertiseOptions ao = ros::AdvertiseOptions::create<geometry_msgs::WrenchStamped>(
    this->topic_name_,1,
    boost::bind( &GazeboRosF3D::F3DConnect,this),
    boost::bind( &GazeboRosF3D::F3DDisconnect,this), ros::VoidPtr(), &this->queue_);
  this->pub_ = this->rosnode_->advertise(ao);
  
  // Custom Callback Queue
  this->callback_queue_thread_ = boost::thread( boost::bind( &GazeboRosF3D::QueueThread,this ) );
  
  // New Mechanism for Updating every World Cycle
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->update_connection_ = event::Events::ConnectWorldUpdateStart(
      boost::bind(&GazeboRosF3D::UpdateChild, this));
}

////////////////////////////////////////////////////////////////////////////////
// Someone subscribes to me
void GazeboRosF3D::F3DConnect()
{
  this->f3d_connect_count_++;
}

////////////////////////////////////////////////////////////////////////////////
// Someone subscribes to me
void GazeboRosF3D::F3DDisconnect()
{
  this->f3d_connect_count_--;
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosF3D::UpdateChild()
{
  if (this->f3d_connect_count_ == 0)
    return;

  math::Vector3 torque;
  math::Vector3 force;

  // get force on body
  force = this->link_->GetWorldForce();

  // get torque on body
  torque = this->link_->GetWorldTorque();

  this->lock_.lock();
  // copy data into wrench message
  this->wrench_msg_.header.frame_id = this->frame_name_;
  this->wrench_msg_.header.stamp.sec = (this->world_->GetSimTime()).sec;
  this->wrench_msg_.header.stamp.nsec = (this->world_->GetSimTime()).nsec;

  this->wrench_msg_.wrench.force.x    = force.x;
  this->wrench_msg_.wrench.force.y    = force.y;
  this->wrench_msg_.wrench.force.z    = force.z;
  this->wrench_msg_.wrench.torque.x   = torque.x;
  this->wrench_msg_.wrench.torque.y   = torque.y;
  this->wrench_msg_.wrench.torque.z   = torque.z;

  this->pub_.publish(this->wrench_msg_);
  this->lock_.unlock();

}

// Custom Callback Queue
////////////////////////////////////////////////////////////////////////////////
// custom callback queue thread
void GazeboRosF3D::QueueThread()
{
  static const double timeout = 0.01;

  while (this->rosnode_->ok())
  {
    this->queue_.callAvailable(ros::WallDuration(timeout));
  }
}

GZ_REGISTER_MODEL_PLUGIN(GazeboRosF3D);

}
