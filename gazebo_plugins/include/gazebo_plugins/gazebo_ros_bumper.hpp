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

#ifndef GAZEBO_PLUGINS__GAZEBO_ROS_BUMPER_HPP_
#define GAZEBO_PLUGINS__GAZEBO_ROS_BUMPER_HPP_

#include <gazebo/common/Plugin.hh>

#include <memory>

namespace gazebo_plugins
{
class GazeboRosBumperPrivate;

/// A plugin that publishes contact states of a body using contact sensor.
/**
  Example Usage:
  \code{.xml}
    <plugin name="plugin_name" filename="libgazebo_ros_bumper.so">
      <!-- Change namespace and topics so:
           Bumper state is published to: /custom_ns/custom_topic
      -->
      <ros>
        <namespace>custom_ns</namespace>
        <remapping>bumper_states:=custom_topic</remapping>
      </ros>

      <!-- Set TF frame name. Defaults to world -->
      <frame_name>custom_frame</frame_name>
    </plugin>
  \endcode
*/
class GazeboRosBumper : public gazebo::SensorPlugin
{
public:
  /// Constructor
  GazeboRosBumper();

  /// Destructor
  virtual ~GazeboRosBumper();

  // Documentation Inherited
  void Load(gazebo::sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

private:
  /// Private data pointer
  std::unique_ptr<GazeboRosBumperPrivate> impl_;
};

}  // namespace gazebo_plugins

#endif  // GAZEBO_PLUGINS__GAZEBO_ROS_BUMPER_HPP_
