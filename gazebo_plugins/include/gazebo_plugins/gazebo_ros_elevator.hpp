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

#ifndef GAZEBO_PLUGINS__GAZEBO_ROS_ELEVATOR_HPP_
#define GAZEBO_PLUGINS__GAZEBO_ROS_ELEVATOR_HPP_

#include <gazebo/plugins/ElevatorPlugin.hh>
#include <std_msgs/msg/string.hpp>

#include <memory>

namespace gazebo_plugins
{
class GazeboRosElevatorPrivate;

/// A elevator plugin for gazebo.
/**
  Example Usage:
  \code{.xml}
    <!-- Plugin to control the elevator -->
    <plugin name="elevator" filename="libgazebo_ros_elevator.so">
      <ros>
        <namespace>demo</namespace>

        <!-- topic remapping -->
        <remapping>elevator:=elevator_demo</remapping>
      </ros>

      <!-- min and max floor constraints -->
      <bottom_floor>0</bottom_floor>
      <top_floor>1</top_floor>
  \endcode
*/
class GazeboRosElevator : public gazebo::ElevatorPlugin
{
public:
  /// Constructor
  GazeboRosElevator();

  /// Destructor
  ~GazeboRosElevator();

protected:
  /// Callback for receiving message on elevator's topic.
  /// \param[in] msg String message that contains a elevator command.
  void OnElevator(const std_msgs::msg::String::ConstSharedPtr msg);

  // Documentation inherited
  void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

private:
  /// Private data pointer
  std::unique_ptr<GazeboRosElevatorPrivate> impl_;
};
}  // namespace gazebo_plugins

#endif  // GAZEBO_PLUGINS__GAZEBO_ROS_ELEVATOR_HPP_
