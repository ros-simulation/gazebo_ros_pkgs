// Copyright 2020 Open Source Robotics Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>

#include <gazebo/transport/transport.hh>
#include <gazebo_plugins/gazebo_ros_wheel_slip.hpp>
#include <gazebo_ros/node.hpp>
#include <std_msgs/msg/bool.hpp>

#include <memory>
#include <string>

namespace gazebo_plugins
{
class GazeboRosWheelSlipPrivate
{
public:
  /// A pointer to the GazeboROS node.
  gazebo_ros::Node::SharedPtr ros_node_;
};

GazeboRosWheelSlip::GazeboRosWheelSlip()
: impl_(std::make_unique<GazeboRosWheelSlipPrivate>())
{
}

GazeboRosWheelSlip::~GazeboRosWheelSlip()
{
}

void GazeboRosWheelSlip::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  // Initialize ROS node
  impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);
}

GZ_REGISTER_MODEL_PLUGIN(GazeboRosWheelSlip)
}  // namespace gazebo_plugins
