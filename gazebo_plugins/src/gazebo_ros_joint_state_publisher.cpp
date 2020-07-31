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

#include <gazebo/common/Events.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo_plugins/gazebo_ros_joint_state_publisher.hpp>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/node.hpp>
#ifdef IGN_PROFILER_ENABLE
#include <ignition/common/Profiler.hh>
#endif
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <memory>
#include <string>
#include <vector>

namespace gazebo_plugins
{
class GazeboRosJointStatePublisherPrivate
{
public:
  /// Callback to be called at every simulation iteration.
  /// \param[in] info Updated simulation info.
  void OnUpdate(const gazebo::common::UpdateInfo & info);

  /// A pointer to the GazeboROS node.
  gazebo_ros::Node::SharedPtr ros_node_;

  /// Joint state publisher.
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;

  /// Joints being tracked.
  std::vector<gazebo::physics::JointPtr> joints_;

  /// Period in seconds
  double update_period_;

  /// Keep last time an update was published
  gazebo::common::Time last_update_time_;

  /// Pointer to the update event connection.
  gazebo::event::ConnectionPtr update_connection_;
};

GazeboRosJointStatePublisher::GazeboRosJointStatePublisher()
: impl_(std::make_unique<GazeboRosJointStatePublisherPrivate>())
{
}

GazeboRosJointStatePublisher::~GazeboRosJointStatePublisher()
{
}

void GazeboRosJointStatePublisher::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
  // ROS node
  impl_->ros_node_ = gazebo_ros::Node::Get(sdf);

  // Get QoS profiles
  const gazebo_ros::QoS & qos = impl_->ros_node_->get_qos();

  // Joints
  if (!sdf->HasElement("joint_name")) {
    RCLCPP_ERROR(impl_->ros_node_->get_logger(), "Plugin missing <joint_name>s");
    impl_->ros_node_.reset();
    return;
  }

  sdf::ElementPtr joint_elem = sdf->GetElement("joint_name");
  while (joint_elem) {
    auto joint_name = joint_elem->Get<std::string>();

    auto joint = model->GetJoint(joint_name);
    if (!joint) {
      RCLCPP_ERROR(impl_->ros_node_->get_logger(), "Joint %s does not exist!", joint_name.c_str());
    } else {
      impl_->joints_.push_back(joint);
      RCLCPP_INFO(
        impl_->ros_node_->get_logger(), "Going to publish joint [%s]",
        joint_name.c_str() );
    }

    joint_elem = joint_elem->GetNextElement("joint_name");
  }

  if (impl_->joints_.empty()) {
    RCLCPP_ERROR(impl_->ros_node_->get_logger(), "No joints found.");
    impl_->ros_node_.reset();
    return;
  }

  // Update rate
  double update_rate = 100.0;
  if (!sdf->HasElement("update_rate")) {
    RCLCPP_INFO(
      impl_->ros_node_->get_logger(), "Missing <update_rate>, defaults to %f", update_rate);
  } else {
    update_rate = sdf->GetElement("update_rate")->Get<double>();
  }

  if (update_rate > 0.0) {
    impl_->update_period_ = 1.0 / update_rate;
  } else {
    impl_->update_period_ = 0.0;
  }

  impl_->last_update_time_ = model->GetWorld()->SimTime();

  // Joint state publisher
  impl_->joint_state_pub_ = impl_->ros_node_->create_publisher<sensor_msgs::msg::JointState>(
    "joint_states", qos.get_publisher_qos("joint_states", rclcpp::QoS(1000)));

  // Callback on every iteration
  impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&GazeboRosJointStatePublisherPrivate::OnUpdate, impl_.get(), std::placeholders::_1));
}

void GazeboRosJointStatePublisherPrivate::OnUpdate(const gazebo::common::UpdateInfo & info)
{
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE("GazeboRosJointStatePublisherPrivate::OnUpdate");
#endif
  gazebo::common::Time current_time = info.simTime;

  // If the world is reset, for example
  if (current_time < last_update_time_) {
    RCLCPP_INFO(ros_node_->get_logger(), "Negative sim time difference detected.");
    last_update_time_ = current_time;
  }

  // Check period
  double seconds_since_last_update = (current_time - last_update_time_).Double();

  if (seconds_since_last_update < update_period_) {
    return;
  }

#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_BEGIN("fill ROS message");
#endif
  // Populate message
  sensor_msgs::msg::JointState joint_state;

  joint_state.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(current_time);
  joint_state.name.resize(joints_.size());
  joint_state.position.resize(joints_.size());
  joint_state.velocity.resize(joints_.size());

  for (unsigned int i = 0; i < joints_.size(); ++i) {
    auto joint = joints_[i];
    double velocity = joint->GetVelocity(0);
    double position = joint->Position(0);
    joint_state.name[i] = joint->GetName();
    joint_state.position[i] = position;
    joint_state.velocity[i] = velocity;
  }
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_END();
  IGN_PROFILE_BEGIN("publish");
#endif
  // Publish
  joint_state_pub_->publish(joint_state);
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_END();
#endif
  // Update time
  last_update_time_ = current_time;
}

GZ_REGISTER_MODEL_PLUGIN(GazeboRosJointStatePublisher)
}  // namespace gazebo_plugins
