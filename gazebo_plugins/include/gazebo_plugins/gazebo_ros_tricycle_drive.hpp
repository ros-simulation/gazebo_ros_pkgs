// Copyright (c) 2010, Daniel Hewlett, Antons Rebguns
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of the company nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef GAZEBO_PLUGINS__GAZEBO_ROS_TRICYCLE_DRIVE_HPP_
#define GAZEBO_PLUGINS__GAZEBO_ROS_TRICYCLE_DRIVE_HPP_

#include <gazebo/common/Plugin.hh>

#include <memory>

namespace gazebo_plugins
{
class GazeboRosTricycleDrivePrivate;

/// A tricycle drive plugin for gazebo.
/**
  Example Usage:
  \code{.xml}

  <plugin name='tricycle_drive' filename='libgazebo_ros_tricycle_drive.so'>

    <ros>
      <namespace></namespace>
      <remapping>cmd_vel:=cmd_vel</remapping>
      <remapping>odom:=odom</remapping>
    </ros>

    <!-- wheels -->
    <steering_joint>wheel_front_steer</steering_joint>
    <actuated_wheel_joint>wheel_front_spin</actuated_wheel_joint>
    <encoder_wheel_left_joint>wheel_rear_left_spin</encoder_wheel_left_joint>
    <encoder_wheel_right_joint>wheel_rear_right_spin</encoder_wheel_right_joint>

    <!-- kinematics -->
    <wheel_separation>1.0</wheel_separation>
    <encoder_wheel_diameter>0.3</encoder_wheel_diameter>
    <actuated_wheel_diameter>0.3</actuated_wheel_diameter>

    <!-- limits -->
    <max_wheel_torque>20</max_wheel_torque>
    <max_wheel_acceleration>5.0</max_wheel_acceleration>
    <max_steering_speed>1.0</max_steering_speed>

    <!-- output -->
    <publish_odom>true</publish_odom>
    <publish_wheel_tf>true</publish_wheel_tf>
    <publish_wheel_joint_state>true</publish_wheel_joint_state>

    <odometry_frame>odom</odometry_frame>
    <robot_base_frame>fork</robot_base_frame>

  </plugin>

  \endcode
*/

class GazeboRosTricycleDrive : public gazebo::ModelPlugin
{
public:
  /// Constructor
  GazeboRosTricycleDrive();

  /// Destructor
  ~GazeboRosTricycleDrive();

protected:
  // Documentation inherited
  void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

  // Documentation inherited
  void Reset() override;

private:
  /// Private data pointer
  std::unique_ptr<GazeboRosTricycleDrivePrivate> impl_;
};
}  // namespace gazebo_plugins


#endif  // GAZEBO_PLUGINS__GAZEBO_ROS_TRICYCLE_DRIVE_HPP_
