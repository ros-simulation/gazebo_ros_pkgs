/*
 * Copyright 2013 Open Source Robotics Foundation
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
   Desc: GazeboRosPose plugin for manipulating objects in Gazebo
   Author: John Hsu
   Date: 24 Sept 2008
 */

#include <algorithm>
#include <assert.h>

#include <gazebo_plugins/gazebo_ros_pose.h>

namespace gazebo
{
GZ_REGISTER_MODEL_PLUGIN(GazeboRosPose);

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosPose::GazeboRosPose()
{
  this->pose_msg_.position.x = 0;
  this->pose_msg_.position.y = 0;
  this->pose_msg_.position.z = 0;
  this->pose_msg_.orientation.w = 1;
  this->pose_msg_.orientation.x = 0;
  this->pose_msg_.orientation.y = 0;
  this->pose_msg_.orientation.z = 0;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosPose::~GazeboRosPose()
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
void GazeboRosPose::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  // Get the world name.
  this->model_ = _model;
  this->world_ = _model->GetWorld();

  // load parameters
  this->robot_namespace_ = "";
  if (_sdf->HasElement("robotNamespace"))
    this->robot_namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";

  if (!_sdf->HasElement("bodyName"))
  {
    ROS_FATAL("pose plugin missing <bodyName>, cannot proceed");
    return;
  }
  else
    this->link_name_ = _sdf->GetElement("bodyName")->Get<std::string>();

  this->link_ = _model->GetLink(this->link_name_);
  if (!this->link_)
  {
    ROS_FATAL("gazebo_ros_pose plugin error: link named: %s does not exist\n",this->link_name_.c_str());
    return;
  }

  if (_sdf->HasElement("topicName"))
  {
    this->topic_name_ = _sdf->GetElement("topicName")->Get<std::string>();
  }
  else
    this->topic_name_ = "";

  if (_sdf->HasElement("serviceName"))
  {
    this->service_name_ = _sdf->GetElement("serviceName")->Get<std::string>();
  }
  else
    this->service_name_ = "";

  if (_sdf->HasElement("velocityFeedback"))
  {
    this->use_velocity_feedback_ = _sdf->GetElement("velocityFeedback")->Get<bool>();
  }
  else
    this->use_velocity_feedback_ = false;

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  this->rosnode_ = new ros::NodeHandle(this->robot_namespace_);

  // Custom Callback Queue
  ros::SubscribeOptions so = ros::SubscribeOptions::create<geometry_msgs::Pose>(
    this->topic_name_,1,
    boost::bind( &GazeboRosPose::UpdateObjectPose,this,_1),
    ros::VoidPtr(), &this->queue_);
  this->sub_ = this->rosnode_->subscribe(so);

  if (this->service_name_ != "")
  {
    ros::AdvertiseServiceOptions srv_aso =
      ros::AdvertiseServiceOptions::create<gazebo_msgs::SetPose>(
      this->service_name_,
      boost::bind(&GazeboRosPose::UpdateObjectPose, this, _1, _2),
      ros::VoidPtr(), &this->queue_);
    this->srv_ = this->rosnode_->advertiseService(srv_aso);
  }

  // Custom Callback Queue
  this->callback_queue_thread_ = boost::thread( boost::bind( &GazeboRosPose::QueueThread,this ) );

  // New Mechanism for Updating every World Cycle
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  if (this->topic_name_ != "")
  {
    this->update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboRosPose::UpdateChild, this));
  }
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosPose::UpdateObjectPose(const geometry_msgs::Pose::ConstPtr& _msg)
{
  this->pose_msg_.position.x = _msg->position.x;
  this->pose_msg_.position.y = _msg->position.y;
  this->pose_msg_.position.z = _msg->position.z;
  this->pose_msg_.orientation.w = _msg->orientation.w;
  this->pose_msg_.orientation.x = _msg->orientation.x;
  this->pose_msg_.orientation.y = _msg->orientation.y;
  this->pose_msg_.orientation.z = _msg->orientation.z;
}

bool GazeboRosPose::UpdateObjectPose(const gazebo_msgs::SetPose::Request& req,
                                     const gazebo_msgs::SetPose::Response& res)
{
  this->pose_msg_ = req.pose;
  UpdateChild();
  return true;
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosPose::UpdateChild()
{
  this->lock_.lock();
  math::Vector3 position(this->pose_msg_.position.x,
                         this->pose_msg_.position.y,
                         this->pose_msg_.position.z);
  math::Quaternion orientation(this->pose_msg_.orientation.w,
                               this->pose_msg_.orientation.x,
                               this->pose_msg_.orientation.y,
                               this->pose_msg_.orientation.z);
  math::Pose target_pose(position, orientation);

  if (!use_velocity_feedback_) {
    this->model_->SetLinkWorldPose(target_pose, this->link_);
    this->model_->SetLinearVel(math::Vector3(0, 0, 0));
    this->model_->SetLinearAccel(math::Vector3(0, 0, 0));
    this->model_->SetAngularVel(math::Vector3(0, 0, 0));
    this->model_->SetAngularAccel(math::Vector3(0, 0, 0));
  } else {
    math:: Pose current_pose = this->link_->GetWorldPose();
    math:: Pose diff_pose = target_pose - current_pose;
    math::Vector3 diff_pos = diff_pose.pos;
    math::Quaternion diff_rot = diff_pose.rot;
    ROS_INFO_STREAM("target_pose - current_pose: " << target_pose - current_pose);
    this->link_->SetLinearVel(math::Vector3(diff_pos.x, diff_pos.y, diff_pos.z));
    this->link_->SetLinearAccel(math::Vector3(0, 0, 0));
    this->link_->SetAngularVel(math::Vector3(diff_rot.GetRoll(), diff_rot.GetPitch(), diff_rot.GetYaw()));
    this->link_->SetAngularAccel(math::Vector3(0, 0, 0));
  }

  this->lock_.unlock();
}

// Custom Callback Queue
////////////////////////////////////////////////////////////////////////////////
// custom callback queue thread
void GazeboRosPose::QueueThread()
{
  static const double timeout = 0.01;

  while (this->rosnode_->ok())
  {
    this->queue_.callAvailable(ros::WallDuration(timeout));
  }
}

}
