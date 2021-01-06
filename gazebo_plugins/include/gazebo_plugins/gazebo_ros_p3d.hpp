// Copyright 2012 Open Source Robotics Foundation, Inc.
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

#ifndef GAZEBO_PLUGINS__GAZEBO_ROS_P3D_HPP_
#define GAZEBO_PLUGINS__GAZEBO_ROS_P3D_HPP_

#include <gazebo/physics/physics.hh>
#include <gazebo/common/Plugin.hh>

#include <memory>

namespace gazebo_plugins
{

class GazeboRosP3DPrivate;

/// Broadcasts the inertial pose of an model's link via a nav_msgs/Odometry message on a ROS topic.
/**
  Example Usage:
  \code{.xml}
    <plugin name="gazebo_ros_force" filename="libgazebo_ros_force.so">

      <ros>

        <!-- Add a namespace -->
        <namespace>/demo</namespace>

        <!-- Remap the default topic -->
        <remapping>odom:=p3d_demo</remapping>

      </ros>

      <!-- Name of the link within this model whose pose will be published -->
      <body_name>box_link</body_name>

      <!-- Name of another link within this model to use as a reference frame.
           Remove the tag to use the world as a reference. -->
      <frame_name>sphere_link</frame_name>

      <!-- Update rate in Hz, defaults to 0.0, which means as fast as possible -->
      <update_rate>1</update_rate>

      <!-- Translation offset to be added to the pose. -->
      <xyz_offset>10 10 10</xyz_offset>

      <!-- Rotation offset to be added to the pose, in Euler angles. -->
      <rpy_offset>0.1 0.1 0.1</rpy_offset>

      <!-- Standard deviation of the noise to be added to the reported velocities. -->
      <gaussian_noise>0.01</gaussian_noise>

    </plugin>
  \endcode
*/
class GazeboRosP3D : public gazebo::ModelPlugin
{
public:
  /// Constructor
  GazeboRosP3D();

  /// Destructor
  virtual ~GazeboRosP3D();

  // Documentation inherited
  void Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf) override;

private:
  /// Private data pointer
  std::unique_ptr<GazeboRosP3DPrivate> impl_;
};

}  // namespace gazebo_plugins

#endif  // GAZEBO_PLUGINS__GAZEBO_ROS_P3D_HPP_
