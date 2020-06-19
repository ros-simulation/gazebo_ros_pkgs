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

#ifndef GAZEBO_PLUGINS__GAZEBO_ROS_JOINT_POSE_TRAJECTORY_HPP_
#define GAZEBO_PLUGINS__GAZEBO_ROS_JOINT_POSE_TRAJECTORY_HPP_

#include <gazebo/common/Plugin.hh>

#include <memory>

namespace gazebo_plugins
{
class GazeboRosJointPoseTrajectoryPrivate;

/// Set the trajectory of points to be followed by joints in simulation.
/// Currently only positions specified in the trajectory_msgs are handled.
/**
  Example Usage:
  \code{.xml}
    <plugin name="gazebo_ros_joint_pose_trajectory"
        filename="libgazebo_ros_joint_pose_trajectory.so">

      <ros>

        <!-- Add a namespace -->
        <namespace>/my_namespace</namespace>

        <!-- Remap the default topic -->
        <remapping>set_joint_trajectory:=my_trajectory</remapping>

      </ros>

      <!-- Update rate in Hz -->
      <update_rate>2</update_rate>

    </plugin>
  \endcode
*/
class GazeboRosJointPoseTrajectory : public gazebo::ModelPlugin
{
public:
  /// Constructor
  GazeboRosJointPoseTrajectory();

  /// Destructor
  ~GazeboRosJointPoseTrajectory();

protected:
  // Documentation inherited
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override;

private:
  /// Private data pointer
  std::unique_ptr<GazeboRosJointPoseTrajectoryPrivate> impl_;
};
}  // namespace gazebo_plugins
#endif  // GAZEBO_PLUGINS__GAZEBO_ROS_JOINT_POSE_TRAJECTORY_HPP_
