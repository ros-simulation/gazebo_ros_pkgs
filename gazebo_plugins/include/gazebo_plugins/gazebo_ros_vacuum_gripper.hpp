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

#ifndef GAZEBO_PLUGINS__GAZEBO_ROS_VACUUM_GRIPPER_HPP_
#define GAZEBO_PLUGINS__GAZEBO_ROS_VACUUM_GRIPPER_HPP_

#include <gazebo/common/Plugin.hh>

#include <memory>

namespace gazebo_plugins
{
class GazeboRosVacuumGripperPrivate;

/// Vacuum Gripper plugin for attracting entities around the model like vacuum
/*
 * \author  Kentaro Wada
 *
 * \date 7 Dec 2015
 */
/**
  Example Usage:
  \code{.xml}
    <plugin name='vacuum_gripper' filename='libgazebo_ros_vacuum_gripper.so'>

      <ros>

        <!-- Add a namespace -->
        <namespace>/demo</namespace>

        <!-- Remapping service and topic names -->
        <remapping>switch:=custom_switch</remapping>
        <remapping>grasping:=custom_grasping</remapping>
      </ros>

      <!-- Link associated with gripper -->
      <link_name>link</link_name>

      <!-- Max distance to attract entities -->
      <max_distance>10.0</max_distance>

      <!-- List of entities to be not attracted by the gripper -->
      <fixed>ground_plane</fixed>
      <fixed>wall</fixed>

    </plugin>
  \endcode
*/
class GazeboRosVacuumGripper : public gazebo::ModelPlugin
{
public:
  /// Constructor
  GazeboRosVacuumGripper();

  /// Destructor
  ~GazeboRosVacuumGripper();

protected:
  // Documentation inherited
  void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

private:
  /// Private data pointer
  std::unique_ptr<GazeboRosVacuumGripperPrivate> impl_;
};
}  // namespace gazebo_plugins

#endif  // GAZEBO_PLUGINS__GAZEBO_ROS_VACUUM_GRIPPER_HPP_
