/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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
#include "gazebo_plugins/gazebo_ros_motor_thermal.h"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(GazeboRosMotorThermal);

/////////////////////////////////////////////////
GazeboRosMotorThermal::GazeboRosMotorThermal()
: MotorThermalPlugin()
{
}

/////////////////////////////////////////////////
GazeboRosThermal::~GazeboRosThermal()
{
  this->queue_.clear();
  this->queue_.disable();
  this->rosnode_->shutdown();
  this->callbackQueueThread_.join();

  delete this->rosnode_;
}

/////////////////////////////////////////////////
void GazeboRosMotorThermal::Load(physics::ModelPtr _model,
                                 sdf::ElementPtr _sdf)
{
  MotorThermalPlugin::Load(_model, _sdf);

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized,"
        << "unable to load plugin. Load the Gazebo system plugin "
        << "'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  this->rosnode = new ros::NodeHandle("");

  std::string topicBase = "/motor_thermal_plugin/" + this->GetHandle() + "/";
  this->torquePub = this->rosnode_->advertise<std_msgs::Float32>(
      topicBase + "torque", 1);
  this->coilPub = this->rosnode_->advertise<std_msgs::Float32>(
      topicBase + "coil", 1);
  this->casePub = this->rosnode_->advertise<std_msgs::Float32>(
      topicBase + "case", 1);

  // start custom queue
  this->callbackQueueThread =
    boost::thread(boost::bind(&GazeboRosMotorThermal::QueueThread, this));

  this->node = transport::NodePtr(new transport::Node());
  this->torqueSub = this->node->Subscribe(topicbase + "torque",
      &GazeboRosMotorThermal::OnTorque, this);
  this->coilSub = this->node->Subscribe(topicbase + "coil",
      &GazeboRosMotorThermal::OnCoil, this);
  this->casSub = this->node->Subscribe(topicbase + "case",
      &GazeboRosMotorThermal::OnCase, this);
}

/////////////////////////////////////////////////
void GazeboRosMotorThermal::OnTorque(ConstTorqueMsgPtr _msg)
{
  msgs::Float32 msg;
  msg.data = _msg.torque();
  this->torquePub->Publish(msg);
}

/////////////////////////////////////////////////
void GazeboRosMotorThermal::OnCoil(ConstTemperatureMsgPtr _msg)
{
  msgs::Float32 msg;
  msg.data = _msg.temperature();
  this->coilPub->Publish(msg);
}

/////////////////////////////////////////////////
void GazeboRosMotorThermal::OnCase(ConstTemperatureMsgPtr _msg)
{
  msgs::Float32 msg;
  msg.data = _msg.temperature();
  this->casePub->Publish(msg);
}

/////////////////////////////////////////////////
void GazeboRosMotorThermal::QueueThread()
{
  static const double timeout = 0.01;

  while (this->rosnode_->ok())
    this->queue_.callAvailable(ros::WallDuration(timeout));
}
#endif
