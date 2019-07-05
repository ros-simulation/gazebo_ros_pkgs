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

#ifndef GAZEBO_PLUGINS__GAZEBO_ROS_ACKERMANN_DRIVE_HPP_
#define GAZEBO_PLUGINS__GAZEBO_ROS_ACKERMANN_DRIVE_HPP_

#include <gazebo/common/Plugin.hh>

#include <memory>

namespace gazebo_plugins
{
class GazeboRosAckermannDrivePrivate;

/// A ackermann drive plugin for car like robots. Subscribes to geometry_msgs/twist

/**
  Example Usage:
  \code{.xml}
    <plugin name="gazebo_ros_ackermann_drive" filename="libgazebo_ros_ackermann_drive.so">

      <ros>
        <namespace>demo</namespace>
        <argument>cmd_vel:=cmd_demo</argument>
        <argument>odom:=odom_demo</argument>
        <argument>distance:=distance_demo</argument>
      </ros>

      <update_rate>100.0</update_rate>

      <!-- wheels -->
      <front_left_joint>front_left_wheel_joint</front_left_joint>
      <front_right_joint>front_right_wheel_joint</front_right_joint>
      <rear_left_joint>rear_left_wheel_joint</rear_left_joint>
      <rear_right_joint>rear_right_wheel_joint</rear_right_joint>
      <left_steering_joint>front_left_steer_joint</left_steering_joint>
      <right_steering_joint>front_right_steer_joint</right_steering_joint>
      <steering_wheel_joint>steering_joint</steering_wheel_joint>

      <!-- Max absolute steer angle for tyre in radians-->
      <!-- Any cmd_vel angular z greater than this would be capped -->
      <max_steer>0.6458</max_steer>

      <!-- Max absolute steering angle of steering wheel -->
      <max_steering_angle>7.85</max_steering_angle>

      <!-- Max absolute linear speed in m/s -->
      <max_speed>20</max_speed>

      <!-- PID tuning -->
      <left_steering_p_gain>1000</left_steering_p_gain>
      <left_steering_i_gain>100</left_steering_i_gain>
      <left_steering_d_gain>1e15</left_steering_d_gain>
      <right_steering_p_gain>1000</right_steering_p_gain>
      <right_steering_i_gain>100</right_steering_i_gain>
      <right_steering_d_gain>1e15</right_steering_d_gain>
      <linear_velocity_p_gain>1300</linear_velocity_p_gain>
      <linear_velocity_i_gain>0.1</linear_velocity_i_gain>
      <linear_velocity_d_gain>10000</linear_velocity_d_gain>

      <!-- output -->
      <publish_odom>true</publish_odom>
      <publish_odom_tf>true</publish_odom_tf>
      <publish_wheel_tf>true</publish_wheel_tf>
      <publish_distance>true</publish_distance>

      <odometry_frame>odom_demo</odometry_frame>
      <robot_base_frame>chassis</robot_base_frame>

    </plugin>
  \endcode
*/
class GazeboRosAckermannDrive : public gazebo::ModelPlugin
{
public:
  /// Constructor
  GazeboRosAckermannDrive();

  /// Destructor
  ~GazeboRosAckermannDrive();

protected:
  // Documentation inherited
  void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

  // Documentation inherited
  void Reset() override;

private:
  /// Private data pointer
  std::unique_ptr<GazeboRosAckermannDrivePrivate> impl_;
};
}  // namespace gazebo_plugins

#endif  // GAZEBO_PLUGINS__GAZEBO_ROS_ACKERMANN_DRIVE_HPP_
