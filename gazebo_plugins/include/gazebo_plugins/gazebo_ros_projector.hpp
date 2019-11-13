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

#ifndef GAZEBO_PLUGINS__GAZEBO_ROS_PROJECTOR_HPP_
#define GAZEBO_PLUGINS__GAZEBO_ROS_PROJECTOR_HPP_

#include <gazebo/common/Plugin.hh>

#include <memory>

namespace gazebo_plugins
{
class GazeboRosProjectorPrivate;

/// A projector plugin for gazebo.
/**
  Example Usage:
  \code{.xml}
    <!-- Plugin to control the projector -->
    <plugin name="projector" filename="libgazebo_ros_projector.so">
      <ros>
        <namespace>demo</namespace>

        <!-- topic remapping -->
        <remapping>switch:=switch_demo</remapping>

        <projector_link>projector_link</projector_link>
        <projector_name>my_projector</projector_name>
      </ros>

  \endcode
*/
class GazeboRosProjector : public gazebo::ModelPlugin
{
public:
  /// Constructor
  GazeboRosProjector();

  /// Destructor
  ~GazeboRosProjector();

protected:
  // Documentation inherited
  void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

private:
  /// Private data pointer
  std::unique_ptr<GazeboRosProjectorPrivate> impl_;
};
}  // namespace gazebo_plugins

#endif  // GAZEBO_PLUGINS__GAZEBO_ROS_PROJECTOR_HPP_
