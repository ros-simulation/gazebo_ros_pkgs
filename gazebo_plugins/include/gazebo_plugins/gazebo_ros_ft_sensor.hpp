// Copyright 2012-2014 Open Source Robotics Foundation
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

#ifndef GAZEBO_PLUGINS__GAZEBO_ROS_FT_SENSOR_HPP_
#define GAZEBO_PLUGINS__GAZEBO_ROS_FT_SENSOR_HPP_

#include <gazebo/common/Plugin.hh>

#include <memory>

namespace gazebo_plugins
{
class GazeboRosFTSensorPrivate;

/// This is a controller that simulates a 6 dof force and torque sensor on link or joint.
/// For joints, the wrench is reported in the joint child link frame and the
/// measure direction is child-to-parent link. (Force and Torque Feed Back Ground Truth)
/// If <body_name> is specified, the plugin acts as sensor on a link, otherwise if
/// <joint_name> is specified, it acts as a sensor on a joint
/*
 * \author  John Hsu
 * \author Francisco Suarez-Ruiz
 */
/**
  Example Usage:
  \code{.xml}
    <plugin name="gazebo_ros_ft_sensor" filename="libgazebo_ros_ft_sensor.so">

      <ros>

        <!-- Add a namespace -->
        <namespace>/demo</namespace>

        <!-- Remap the default topic -->
        <remapping>wrench:=wrench_demo</remapping>

      </ros>

      <!-- Link name -->
      <body_name>link</body_name>

      <!-- Set frame id of published message. Used only when sensor used on links -->
      <frame_name>demo_world</frame_name>

      <!-- Update rate in Hz, defaults to 0.0, which means as fast as possible -->
      <update_rate>1</update_rate>

      <!-- Standard deviation of the noise to be added to the reported wrench messages. -->
      <gaussian_noise>0.01</gaussian_noise>

    </plugin>
  \endcode
*/
class GazeboRosFTSensor : public gazebo::ModelPlugin
{
public:
  /// Constructor
  GazeboRosFTSensor();

  /// Destructor
  ~GazeboRosFTSensor();

protected:
  // Documentation inherited
  void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

private:
  /// Private data pointer
  std::unique_ptr<GazeboRosFTSensorPrivate> impl_;
};
}  // namespace gazebo_plugins

#endif  // GAZEBO_PLUGINS__GAZEBO_ROS_FT_SENSOR_HPP_
