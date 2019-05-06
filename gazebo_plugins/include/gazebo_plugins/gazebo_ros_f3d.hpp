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

#ifndef GAZEBO_PLUGINS__GAZEBO_ROS_F3D_HPP_
#define GAZEBO_PLUGINS__GAZEBO_ROS_F3D_HPP_

#include <gazebo/common/Plugin.hh>

#include <memory>

namespace gazebo_plugins
{
class GazeboRosF3DPrivate;

/// This is a controller that simulates a 6 dof force sensor.
/// (Force Feed Back Ground Truth)
/*
 * \author  John Hsu
 *
 * \date 24 Sept 2008
 */
/**
  Example Usage:
  \code{.xml}
    <plugin name="gazebo_ros_f3d" filename="libgazebo_ros_f3d.so">

      <ros>

        <!-- Add a namespace -->
        <namespace>/demo</namespace>

        <!-- Remap the default topic -->
        <argument>wrench:=wrench_demo</argument>

      </ros>

      <!-- Link name -->
      <body_name>link</body_name>

      <!-- Frame id of published message -->
      <frame_name>demo_world</frame_name>

    </plugin>
  \endcode
*/
class GazeboRosF3D : public gazebo::ModelPlugin
{
public:
  /// Constructor
  GazeboRosF3D();

  /// Destructor
  ~GazeboRosF3D();

protected:
  // Documentation inherited
  void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

private:
  /// Private data pointer
  std::unique_ptr<GazeboRosF3DPrivate> impl_;
};
}  // namespace gazebo_plugins

#endif  // GAZEBO_PLUGINS__GAZEBO_ROS_F3D_HPP_
