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

/// Description:
/// ------------
/// A plugin for adjusting wheel slip parameters in gazebo.
/// The plugin can set separate longitudinal and lateral wheel slip compliance
/// parameters for separate wheel links. THe following ROS parameters can be specified in
/// the world file (These are optional, you may not may not declare them):
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
///
///  3. slip_compliance_unitless_lateral/wheel_name
///      - Type: double
///      - Description: Unitless slip compliance (slip / friction) in the
///           lateral direction. This value is applied to the wheel named 'wheel_name'
///           declaredin in the WheelSlipPlugin.
///
///  4. slip_compliance_unitless_longitudinal/wheel_name
///      - Type: double
///      - Description: Unitless slip compliance (slip / friction) in the
///           longitudinal direction. This value is applied to the wheel named 'wheel_name'
///           declared in the WheelSlipPlugin.
///
/// Precedence order and deafult values:
/// -----------------------------------
/// Slip compliance values can be declared in 3 ways:
///  1. SDF parameters, for e.g. :
///       <wheel link_name="wheel_front">
///         <slip_compliance_lateral>1</slip_compliance_lateral>
///         <slip_compliance_longitudinal>2</slip_compliance_longitudinal>
///         <wheel_normal_force>77</wheel_normal_force>
///       </wheel>
///
///      These if not specified, will default to 0.0. Any negative values will ignored and
//       set to 0.0
///
///  2. ROS parameters for each wheel, for e.g :
///       <ros>
///         <parameter name="slip_compliance_unitless_lateral/wheel_front" type="double">10.0
///           </parameter>
///         <parameter name="slip_compliance_unitless_longitudinal/wheel_front" type="double">11.0
///           </parameter>
///       </ros>
///
///      These if not specified, default to SDF parameters set for each wheel. If specified, they
///      will override the ones set as SDF parameters.
///
///  3. ROS parameters for all wheels, for e.g :
///      <ros>
///         <parameter name="slip_compliance_unitless_lateral" type="double">10.0</parameter>
///         <parameter name="slip_compliance_unitless_longitudinal" type="double">11.0</parameter>
///      </ros>
///
///     These if not specified will default to the last ones set in the SDF tags. In this case,
///     they won't override anything.
///     If these are specified, they override both SDF and the ROS parameters for each wheel.
///
///   Precedence order :
///   ROS Params for all wheels > ROS params for individual wheels > SDF parameters
///   Check out the test cases for more information and expected behaviour.
///
/// See the WheelSlipPlugin documentation at the following location for more details:
/// http://osrf-distributions.s3.amazonaws.com/gazebo/api/11.0.0/classgazebo_1_1WheelSlipPlugin.html#details
/**
  Example Usage:
  \code{.xml}
    <!-- Plugin to set wheel slip parameters according to wheel speed -->
    <plugin name="projector" filename="libgazebo_ros_wheel_speed.so">
      <ros>
        <namespace>wheel_slip_front</namespace>
        <parameter name="slip_compliance_unitless_lateral/wheel_front_left" type="double">10.0</parameter>
        <parameter name="slip_compliance_unitless_longitudinal/wheel_front_left" type="double">11.0</parameter>
        <parameter name="slip_compliance_unitless_lateral" type="double">10.0</parameter>
        <parameter name="slip_compliance_unitless_longitudinal" type="double">11.0</parameter>
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
