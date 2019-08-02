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

#ifndef GAZEBO_PLUGINS__GAZEBO_ROS_HAND_OF_GOD_HPP_
#define GAZEBO_PLUGINS__GAZEBO_ROS_HAND_OF_GOD_HPP_

#include <gazebo/common/Plugin.hh>

#include <memory>

namespace gazebo_plugins
{
class GazeboRosHandOfGodPrivate;

/// Drives a floating object around based on the location of a TF frame.
/// It is useful for connecting human input
/**
  Example Usage:
  \code{.xml}
    <plugin name="gazebo_ros_hand_of_god" filename="libgazebo_ros_hand_of_god.so">

      <ros>

        <!-- Add a namespace -->
        <namespace>/demo</namespace>

      </ros>

      <!-- This is required -->
      <link_name>link</link_name>

      <!-- Defaults to world -->
      <!-- The plugin expects TF `frame_id` + "_desired" and broadcasts `frame_id` + "_actual" -->
      <frame_id>link</frame_id>

      <!-- Set force and torque gains -->
      <ka>200</ka>
      <kl>200</kl>

    </plugin>
  \endcode
*/
class GazeboRosHandOfGod : public gazebo::ModelPlugin
{
public:
  /// Constructor
  GazeboRosHandOfGod();

  /// Destructor
  ~GazeboRosHandOfGod();

protected:
  // Documentation inherited
  void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

private:
  /// Private data pointer
  std::unique_ptr<GazeboRosHandOfGodPrivate> impl_;
};
}  // namespace gazebo_plugins

#endif  // GAZEBO_PLUGINS__GAZEBO_ROS_HAND_OF_GOD_HPP_
