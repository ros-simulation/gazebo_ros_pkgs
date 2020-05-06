// Copyright 2019 Open Source Robotics Foundation
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

#ifndef GAZEBO_PLUGINS__GAZEBO_ROS_WHEEL_SLIP_HPP_
#define GAZEBO_PLUGINS__GAZEBO_ROS_WHEEL_SLIP_HPP_

#include <gazebo/common/Plugin.hh>
#include <gazebo/plugins/WheelSlipPlugin.hh>

#include <memory>

namespace gazebo_plugins
{
class GazeboRosWheelSlipPrivate;

/// A plugin for adjusting wheel slip parameters in gazebo.
/// The plugin can set separate longitudinal and lateral wheel slip compliance
/// parameters for separate wheel links.
///  1. slip_compliance_unitless_lateral
///      - Type: double
///      - Description: Unitless slip compliance (slip / friction) in the
///           lateral direction. This value is applied to all wheels declared
///           in the WheelSlipPlugin.
///
///  2. slip_compliance_unitless_longitudinal
///      - Type: double
///      - Description: Unitless slip compliance (slip / friction) in the
///           longitudinal direction. This value is applied to all wheels declared
///           in the WheelSlipPlugin.
/// See the WheelSlipPlugin documentation at the following location for more details:
/// http://osrf-distributions.s3.amazonaws.com/gazebo/api/11.0.0/classgazebo_1_1WheelSlipPlugin.html#details
/**
  Example Usage:
  \code{.xml}
    <!-- Plugin to set wheel slip parameters according to wheel speed -->
    <plugin name="projector" filename="libgazebo_ros_wheel_speed.so">
      <ros>
        <namespace>wheel_slip_front</namespace>
      </ros>
      <wheel link_name="wheel_front_left">
        <slip_compliance_lateral>0</slip_compliance_lateral>
        <slip_compliance_longitudinal>0.1</slip_compliance_longitudinal>
        <wheel_normal_force>100</wheel_normal_force>
      </wheel>
      <wheel link_name="wheel_front_right">
        <slip_compliance_lateral>0</slip_compliance_lateral>
        <slip_compliance_longitudinal>0.1</slip_compliance_longitudinal>
        <wheel_normal_force>100</wheel_normal_force>
      </wheel>
    </plugin>
  \endcode
*/
class GazeboRosWheelSlip : public gazebo::WheelSlipPlugin
{
public:
  /// Constructor
  GazeboRosWheelSlip();

  /// Destructor
  ~GazeboRosWheelSlip();

protected:
  // Documentation inherited
  void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

private:
  /// Private data pointer
  std::unique_ptr<GazeboRosWheelSlipPrivate> impl_;
};
}  // namespace gazebo_plugins

#endif  // GAZEBO_PLUGINS__GAZEBO_ROS_WHEEL_SLIP_HPP_
