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

#ifndef GAZEBO_PLUGINS__GAZEBO_ROS_PLANAR_MOVE_HPP_
#define GAZEBO_PLUGINS__GAZEBO_ROS_PLANAR_MOVE_HPP_

#include <gazebo/common/Plugin.hh>

#include <memory>

namespace gazebo_plugins
{
class GazeboRosPlanarMovePrivate;

/// Simple model controller that uses a twist message to move an entity on the xy plane.
/*
 * \author  Piyush Khandelwal (piyushk@gmail.com)
 *
 * \date  29 July 2013
 */

/**
  Example Usage:
  \code{.xml}
    <plugin name="gazebo_ros_planar_move" filename="libgazebo_ros_planar_move.so">

      <ros>

        <!-- Add a namespace -->
        <namespace>/demo</namespace>

        <!-- Remap the default topic -->
        <remapping>cmd_vel:=custom_cmd_vel</remapping>
        <remapping>odom:=custom_odom</remapping>

      </ros>

      <update_rate>100</update_rate>
      <publish_rate>10</publish_rate>

      <!-- output -->
      <publish_odom>true</publish_odom>
      <publish_odom_tf>true</publish_odom_tf>

      <odometry_frame>odom_demo</odometry_frame>
      <robot_base_frame>link</robot_base_frame>

      <covariance_x>0.0001</covariance_x>
      <covariance_y>0.0001</covariance_y>
      <covariance_yaw>0.01</covariance_yaw>

    </plugin>
  \endcode
*/

class GazeboRosPlanarMove : public gazebo::ModelPlugin
{
public:
  /// Constructor
  GazeboRosPlanarMove();

  /// Destructor
  ~GazeboRosPlanarMove();

protected:
  // Documentation inherited
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override;

  // Documentation inherited
  void Reset() override;

private:
  /// Private data pointer
  std::unique_ptr<GazeboRosPlanarMovePrivate> impl_;
};
}  // namespace gazebo_plugins

#endif  // GAZEBO_PLUGINS__GAZEBO_ROS_PLANAR_MOVE_HPP_
