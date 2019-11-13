// Copyright 2012 Open Source Robotics Foundation, Inc.
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

#ifndef GAZEBO_PLUGINS__GAZEBO_ROS_FORCE_HPP_
#define GAZEBO_PLUGINS__GAZEBO_ROS_FORCE_HPP_

#include <gazebo/common/Plugin.hh>
#include <geometry_msgs/msg/wrench.hpp>

#include <memory>

namespace gazebo_plugins
{
class GazeboRosForcePrivate;

/// This plugin collects data from a ROS topic and applies wrench to a link accordingly.
/**
  \details The last received force will be continuously added to the link at every simulation iteration.
  Send an empty / zero message to stop applying a force.

  Example Usage:
  \code{.xml}
    <plugin name="gazebo_ros_force" filename="libgazebo_ros_force.so">

      <ros>

        <!-- Add a namespace -->
        <namespace>/test</namespace>

        <!-- Remap the default topic -->
        <remapping>gazebo_ros_force:=force_test</remapping>

      </ros>

      <!-- Name of link within model which will receive the force -->
      <link_name>link</link_name>

      <!-- Frame where the force/torque will be applied (options: world; link)-->
      <force_frame>link</force_frame>

    </plugin>
  \endcode
*/

class GazeboRosForce : public gazebo::ModelPlugin
{
public:
  /// Constructor
  GazeboRosForce();

  /// Destructor
  virtual ~GazeboRosForce();

protected:
  // Documentation inherited
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override;

  /// Optional callback to be called at every simulation iteration.
  virtual void OnUpdate();

private:
  /// Callback when a ROS Wrench message is received
  /// \param[in] msg The Incoming ROS message representing the new force to
  /// exert.
  void OnRosWrenchMsg(const geometry_msgs::msg::Wrench::SharedPtr msg);

  /// Private data pointer
  std::unique_ptr<GazeboRosForcePrivate> impl_;
};
}  // namespace gazebo_plugins

#endif  // GAZEBO_PLUGINS__GAZEBO_ROS_FORCE_HPP_
