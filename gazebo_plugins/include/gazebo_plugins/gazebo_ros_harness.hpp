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

#ifndef GAZEBO_PLUGINS__GAZEBO_ROS_HARNESS_HPP_
#define GAZEBO_PLUGINS__GAZEBO_ROS_HARNESS_HPP_

#include <gazebo/plugins/HarnessPlugin.hh>
#include <std_msgs/msg/empty.hpp>

#include <memory>

namespace gazebo_plugins
{
class GazeboRosHarnessPrivate;

/// Harness plugin for gazebo.
/// Used to lower a model in a smooth manner
/**
  Example Usage:
  \code{.xml}
    <plugin name="gazebo_ros_harness" filename="libgazebo_ros_harness.so">

      <ros>
        <!-- Add a namespace -->
        <namespace>demo</namespace>
        <remapping>box/harness/velocity:=velocity_demo</remapping>
        <remapping>box/harness/detach:=detach_demo</remapping>
      </ros>
      <!-- set initial harness velocity -->
      <!--<init_vel>-0.1</init_vel>-->

      <joint name="joint1" type="prismatic">
          <!-- The joint that raises or lowers the harness.
           This must be specified within the body of this plugin. -->
          <pose>0 0 0 0 0 0</pose>
          <parent>world</parent>
          <child>link</child>
          <axis>
            <xyz>0 0 1</xyz>
            <dynamics>
              <damping>10</damping>
            </dynamics>
            <limit>
              <lower>-1.5</lower>
              <upper>1.5</upper>
              <effort>10000</effort>
              <velocity>-1</velocity>
              <stiffness>0</stiffness>
              <dissipation>0</dissipation>
            </limit>
          </axis>
          <physics>
            <ode>
              <implicit_spring_damper>1</implicit_spring_damper>
              <limit>
                <cfm>0.0</cfm>
                <erp>0.0</erp>
              </limit>
            </ode>
          </physics>
        </joint>

        <winch>
          <joint>joint1</joint>
          <!-- PID value for velocity control of the winch. -->
          <pos_pid>
            <p>1000000</p>
            <i>0</i>
            <d>0</d>
            <i_min>0</i_min>
            <i_max>0</i_max>
            <cmd_min>-10000</cmd_min>
            <cmd_max>10000</cmd_max>
          </pos_pid>

          <vel_pid>
            <p>10000</p>
            <i>0</i>
            <d>0</d>
            <i_min>0</i_min>
            <i_max>0</i_max>
            <cmd_min>0</cmd_min>
            <cmd_max>10000</cmd_max>
          </vel_pid>
        </winch>

        <!-- Joint to detach. Once the joint is detached, it cannot be
          reattached. This must be a joint specified within the
          body of this plugin. -->
        <detach>joint1</detach>

    </plugin>
  \endcode
*/
class GazeboRosHarness : public gazebo::HarnessPlugin
{
public:
  /// Constructor
  GazeboRosHarness();

  /// Destructor
  ~GazeboRosHarness();

protected:
  /// Callback for receiving detach messages.
  /// \param[in] msg Empty message.
  void OnDetach(const std_msgs::msg::Empty::ConstSharedPtr msg);

  // Documentation inherited
  void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

private:
  /// Private data pointer
  std::unique_ptr<GazeboRosHarnessPrivate> impl_;
};
}  // namespace gazebo_plugins

#endif  // GAZEBO_PLUGINS__GAZEBO_ROS_HARNESS_HPP_
