// Copyright 2019 Open Source Robotics Foundation, Inc.
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

#ifndef GAZEBO_ROS__GAZEBO_ROS_FORCE_SYSTEM_HPP_
#define GAZEBO_ROS__GAZEBO_ROS_FORCE_SYSTEM_HPP_

#include <gazebo/common/Plugin.hh>

#include <memory>

namespace gazebo_ros
{

class GazeboRosForceSystemPrivate;

/// Provides services and topics to
/// Apply and clear wrenches on gazebo links.
/// Apply and clear forces on gazebo joints
///
/// Services:
///
///      apply_link_wrench (gazebo_msgs::srv::ApplyLinkWrench)
///          Apply wrench on gazebo links
///
///      clear_link_wrenches (gazebo_msgs::srv::LinkRequest)
///          Clear wrenches on gazebo links.
///
///      apply_joint_effort (gazebo_msgs::srv::ApplyJointEffort)
///          Apply effort on gazebo joints
///
///      clear_joint_efforts (gazebo_msgs::srv::JointRequest)
///          Clear efforts on gazebo joints.
class GazeboRosForceSystem : public gazebo::SystemPlugin
{
public:
  /// Constructor
  GazeboRosForceSystem();

  /// Destructor
  virtual ~GazeboRosForceSystem();

  // Documentation inherited
  void Load(int argc, char ** argv) override;

private:
  std::unique_ptr<GazeboRosForceSystemPrivate> impl_;
};

}  // namespace gazebo_ros
#endif  // GAZEBO_ROS__GAZEBO_ROS_FORCE_SYSTEM_HPP_
