/*
 * Copyright (C) 2017 Open Source Robotics Foundation
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
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Model.hh>
#include <sdf/sdf.hh>

#include "gazebo_plugins/gazebo_ros_wheel_slip.h"

namespace gazebo
{

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboRosWheelSlip)

/////////////////////////////////////////////////
GazeboRosWheelSlip::GazeboRosWheelSlip()
{
}

/////////////////////////////////////////////////
GazeboRosWheelSlip::~GazeboRosWheelSlip()
{
  // Custom Callback Queue
  this->queue_.clear();
  this->queue_.disable();

  this->rosnode_->shutdown();
  delete this->rosnode_;
}

/////////////////////////////////////////////////
void GazeboRosWheelSlip::configCallback(
  gazebo_plugins::WheelSlipConfig &config, uint32_t /*level*/)
{
  ROS_INFO_NAMED("wheel_slip", "Reconfigure request for the gazebo ros wheel_slip: %s. New slip compliances, lateral: %.3e, longitudinal: %.3e",
           this->GetParentModel()->GetScopedName().c_str(),
           config.slip_compliance_unitless_lateral,
           config.slip_compliance_unitless_longitudinal);
  this->SetSlipComplianceLateral(config.slip_compliance_unitless_lateral);
  this->SetSlipComplianceLongitudinal(config.slip_compliance_unitless_longitudinal);
}

/////////////////////////////////////////////////
// Load the controller
void GazeboRosWheelSlip::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  // Load the plugin
  WheelSlipPlugin::Load(_parent, _sdf);

  if (_sdf->HasElement("robotNamespace"))
  {
    this->robotNamespace_ = _sdf->Get<std::string>("robotNamespace") + "/";
  }
  if (this->robotNamespace_.empty() ||
      this->robotNamespace_ == "/" ||
      this->robotNamespace_ == "//")
  {
    this->robotNamespace_ = "wheel_slip/";
  }
  this->robotNamespace_ = _parent->GetName() + "/" + this->robotNamespace_;

  // Init ROS
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM_NAMED("wheel_slip", "Not loading plugin since ROS hasn't been "
          << "properly initialized.  Try starting gazebo with ros plugin:\n"
          << "  gazebo -s libgazebo_ros_api_plugin.so\n");
    return;
  }

  this->rosnode_ = new ros::NodeHandle(this->robotNamespace_);

  // set up dynamic reconfigure
  dyn_srv_ =
    new dynamic_reconfigure::Server<gazebo_plugins::WheelSlipConfig>
    (*this->rosnode_);
  dynamic_reconfigure::Server<gazebo_plugins::WheelSlipConfig>
    ::CallbackType f =
    boost::bind(&GazeboRosWheelSlip::configCallback, this, _1, _2);
  dyn_srv_->setCallback(f);

  this->wheelSlipPub_ = this->rosnode_->advertise<sensor_msgs::JointState>("wheel_slips", 1000);

  // Custom Callback Queue
  this->callbackQueueThread_ =
    boost::thread(boost::bind(&GazeboRosWheelSlip::QueueThread, this));

  // Callback for each simulation step
  this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&GazeboRosWheelSlip::Update, this));
}

/////////////////////////////////////////////////
void GazeboRosWheelSlip::PublishWheelSlips(
              const std::map<std::string, ignition::math::Vector3d> &_slips)
{
  ros::Time current_time = ros::Time::now();

  this->wheelSlips_.header.stamp = current_time;
  this->wheelSlips_.name.resize(_slips.size());
  this->wheelSlips_.position.resize(_slips.size());
  this->wheelSlips_.velocity.resize(_slips.size());
  this->wheelSlips_.effort.resize(_slips.size());

  int i = 0;
  for (const auto &slip : _slips)
  {
    this->wheelSlips_.name[i] = slip.first;
    this->wheelSlips_.position[i] = slip.second.X();
    this->wheelSlips_.velocity[i] = slip.second.Y();
    this->wheelSlips_.effort[i] = slip.second.Z();
    ++i;
  }
  this->wheelSlipPub_.publish(this->wheelSlips_);
}

/////////////////////////////////////////////////
void GazeboRosWheelSlip::Update()
{
  this->GetSlips(this->slipsMap_);
  this->PublishWheelSlips(this->slipsMap_);
}

/////////////////////////////////////////////////
void GazeboRosWheelSlip::QueueThread()
{
  static const double timeout = 0.01;

  while (this->rosnode_->ok())
  {
    this->queue_.callAvailable(ros::WallDuration(timeout));
  }
}
}
