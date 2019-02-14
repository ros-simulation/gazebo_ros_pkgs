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

#ifndef GAZEBO_ROS__GAZEBO_ROS_STATE_HPP_
#define GAZEBO_ROS__GAZEBO_ROS_STATE_HPP_

#include <gazebo/common/Plugin.hh>

#include <memory>

namespace gazebo_ros
{

class GazeboRosStatePrivate;

/// Provides services and topics to query and set the state of entities in
/// simualtion, such as position and velocity.
///
///  Services:
///
///      get_entity_state (gazebo_msgs::srv::GetEntityState)
///          Get an entity's position and velocity.
///
///      set_entity_state (gazebo_msgs::srv::SetEntityState)
///          Set an entity's position and velocity.
class GazeboRosState : public gazebo::WorldPlugin
{
public:
  /// Constructor
  GazeboRosState();

  /// Destructor
  virtual ~GazeboRosState();

  // Documentation inherited
  void Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr _sdf) override;

private:
  std::unique_ptr<GazeboRosStatePrivate> impl_;
};

}  // namespace gazebo_ros
#endif  // GAZEBO_ROS__GAZEBO_ROS_STATE_HPP_
