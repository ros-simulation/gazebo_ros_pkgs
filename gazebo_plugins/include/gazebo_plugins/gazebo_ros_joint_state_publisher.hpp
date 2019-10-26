// Copyright (c) 2013, Markus Bader <markus.bader@tuwien.ac.at>
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of the company nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef GAZEBO_PLUGINS__GAZEBO_ROS_JOINT_STATE_PUBLISHER_HPP_
#define GAZEBO_PLUGINS__GAZEBO_ROS_JOINT_STATE_PUBLISHER_HPP_

#include <gazebo/common/Plugin.hh>

#include <memory>

namespace gazebo_plugins
{
class GazeboRosJointStatePublisherPrivate;

/// Publish the state of joints in simulation to a given ROS topic.
/**
  \details If the joint contains more than one axis, only the state of the first axis is reported.

  Example Usage:
  \code{.xml}
    <plugin name="gazebo_ros_joint_state_publisher"
        filename="libgazebo_ros_joint_state_publisher.so">

      <ros>

        <!-- Add a namespace -->
        <namespace>/ny_namespace</namespace>

        <!-- Remap the default topic -->
        <remapping>joint_states:=my_joint_states</remapping>

      </ros>

      <!-- Update rate in Hertz -->
      <update_rate>2</update_rate>

      <!-- Name of joints in the model whose states will be published. -->
      <joint_name>left_wheel</joint_name>
      <joint_name>right_wheel</joint_name>
      <joint_name>elbow</joint_name>
      <joint_name>shoulder</joint_name>

    </plugin>
  \endcode
*/
class GazeboRosJointStatePublisher : public gazebo::ModelPlugin
{
public:
  /// Constructor
  GazeboRosJointStatePublisher();

  /// Destructor
  ~GazeboRosJointStatePublisher();

protected:
  // Documentation inherited
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override;

private:
  /// Callback to be called at every simulation iteration.
  /// Private data pointer
  std::unique_ptr<GazeboRosJointStatePublisherPrivate> impl_;
};
}  // namespace gazebo_plugins
#endif  // GAZEBO_PLUGINS__GAZEBO_ROS_JOINT_STATE_PUBLISHER_HPP_
