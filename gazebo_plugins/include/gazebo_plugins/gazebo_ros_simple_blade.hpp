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

#ifndef GAZEBO_PLUGINS__GAZEBO_ROS_SIMPLE_BLADE_HPP_
#define GAZEBO_PLUGINS__GAZEBO_ROS_SIMPLE_BLADE_HPP_

#include <gazebo/common/Plugin.hh>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/wrench.hpp>

#include <memory>

namespace gazebo_plugins
{
class GazeboRosSimpleBladePrivate;

/// This plugin collects data from a ROS topic and applies angular velocity to a link (drone motor - blade) accordingly.
/// This angular velocity generates torque and force on the body
/**
  \details The last received angular velocity will be continuously added to the link at every simulation iteration.
  Send an empty / zero message to stop applying an incremental force/torque.

  Example Usage:
  \code{.xml}
    </plugin>

    <plugin name="gazebo_ros_simple_blade" filename="libgazebo_ros_simple_blade.so">

        <ros>

        <!-- Add a namespace -->
        <namespace>/test</namespace>

        <!-- Remap the default topic -->
        <argument>gazebo_ros_simple_blade:=simple_blade_test</argument>

      </ros>

      <!-- Name of link within model which will receive the force/torque -->
      <link_name>link1</link_name>

      <!-- Offset between motor blade and motor center of gravity -->
      <blade_offset>x1 y1 z1</blade_offset>

      <!-- Name of link within model which will receive the force/torque -->
      <link_name>link2</link_name>

      <!-- Offset between motor blade and motor center of gravity -->
      <blade_offset>x2 y2 z2</blade_offset>

      <!-- Thrust constant which will multiply the motor squared angular velocity to obtain Force along the motor z axis -->
      <!-- Same value for all motors -->
      <kf_gain>kf</kf_gain>

      <!-- Drag constant which will multiply the motor squared angular velocity to obtain the drag torque along the motor z axis -->
      <!-- Same value for all motors -->
      <kf_gain>kf</kf_gain>

      <!-- Vector with values of 1 or 0 to specify if the motor rotation is clockwise or anti-clockwise-->
      <clockwise>motor_1_rotation_direction motor_2_rotation_direction motor_3_rotation_direction ... motor_n_rotation_direction</clockwise>

      </plugin>

  \endcode
*/

class GazeboRosSimpleBlade : public gazebo::ModelPlugin
{
public:
  /// Constructor
  GazeboRosSimpleBlade();

  /// Destructor
  virtual ~GazeboRosSimpleBlade();

protected:
  // Documentation inherited
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override;

  /// Optional callback to be called at every simulation iteration.
  virtual void OnUpdate();

private:
  /// Callback when a ROS Twist message is received
  /// \param[in] msg The Incoming ROS message representing the new velocity to
  /// exert a torque.
  void OnRosJointStateMsg(const sensor_msgs::msg::JointState::SharedPtr msg);

  /// Private data pointer
  std::unique_ptr<GazeboRosSimpleBladePrivate> impl_;
};
}  // namespace gazebo_plugins

#endif  // GAZEBO_PLUGINS__GAZEBO_ROS_SIMPLE_BLADE_HPP_
