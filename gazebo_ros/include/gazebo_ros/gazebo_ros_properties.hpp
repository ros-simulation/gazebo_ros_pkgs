// Copyright 2018 Open Source Robotics Foundation, Inc.
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

#ifndef GAZEBO_ROS__GAZEBO_ROS_PROPERTIES_HPP_
#define GAZEBO_ROS__GAZEBO_ROS_PROPERTIES_HPP_

#include <gazebo/common/Plugin.hh>

#include <memory>

namespace gazebo_ros
{

class GazeboRosPropertiesPrivate;

/// Provides services and topics to query and set the properties of entities in simulation.
///
///  Services:
///
///      get_model_properties (gazebo_msgs::srv::GetModelProperties)
///          Get a model's properties including
///          parent's name, links, collisions, joints and child links.
///
///      get_joint_properties (gazebo_msgs::srv::GetJointProperties)
///          Get a joint's properties including
///          joint type, dynamics, position and velocity.
///
///      set_joint_properties (gazebo_msgs::srv::SetJointProperties)
///          Set OJE joint properties.
///
///      get_link_properties (gazebo_msgs::srv::GetLinkProperties)
///          Get a link's properties including
///          center of mass, gravity, mass and inertia.
///
///      set_link_properties (gazebo_msgs::srv::SetLinkProperties)
///          Set a link's properties including
///          center of mass, gravity, mass and inertia.
///
///      get_light_properties (gazebo_msgs::srv::GetLightProperties)
///          Get light's properties including color and attenuation.
///
///      set_light_properties (gazebo_msgs::srv::SetLightProperties)
///          Set light's properties including color and attenuation.
///
class GazeboRosProperties : public gazebo::WorldPlugin
{
public:
  /// Constructor
  GazeboRosProperties();

  /// Destructor
  virtual ~GazeboRosProperties();

  // Documentation inherited
  void Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr _sdf) override;

private:
  std::unique_ptr<GazeboRosPropertiesPrivate> impl_;
};

}  // namespace gazebo_ros
#endif  // GAZEBO_ROS__GAZEBO_ROS_PROPERTIES_HPP_
