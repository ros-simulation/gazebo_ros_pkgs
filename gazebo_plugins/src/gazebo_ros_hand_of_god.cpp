// Copyright (c) 2013, Open Source Robotics Foundation
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

/*
 * \file  gazebo_ros_hand_of_god.cpp
 *
 * \brief A "hand-of-god" plugin which drives a floating object around based
 *  on the location of a TF frame. This plugin is useful for connecting human input
 *
 * \author  Jonathan Bohren
 *
 */

#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo_plugins/gazebo_ros_hand_of_god.hpp>
#include <gazebo_ros/conversions/geometry_msgs.hpp>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/node.hpp>
#ifdef IGN_PROFILER_ENABLE
#include <ignition/common/Profiler.hh>
#endif
#include <sdf/sdf.hh>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <memory>
#include <string>

namespace gazebo_plugins
{
class GazeboRosHandOfGodPrivate
{
public:
  /// Callback to be called at every simulation iteration.
  void OnUpdate(const gazebo::common::UpdateInfo & info);

  /// A pointer to the GazeboROS node.
  gazebo_ros::Node::SharedPtr ros_node_;

  /// Pointer to link.
  gazebo::physics::LinkPtr link_;

  /// TF buffer
  std::shared_ptr<tf2_ros::Buffer> buffer_;

  /// TF listener
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_;

  /// To broadcast TFs
  std::shared_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;

  /// frame ID
  std::string frame_;

  /// Applied force and torque gains
  double kl_, ka_, cl_, ca_;

  /// Connection to event called at every world iteration.
  gazebo::event::ConnectionPtr update_connection_;
};

GazeboRosHandOfGod::GazeboRosHandOfGod()
: impl_(std::make_unique<GazeboRosHandOfGodPrivate>())
{
}

GazeboRosHandOfGod::~GazeboRosHandOfGod()
{
}

void GazeboRosHandOfGod::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  // Initialize ROS node
  impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);

  impl_->frame_ = _sdf->Get<std::string>("frame_id", "world").first;

  impl_->kl_ = _sdf->Get<double>("kl", 200).first;
  impl_->ka_ = _sdf->Get<double>("ka", 200).first;

  if (_sdf->HasElement("link_name")) {
    auto link_name = _sdf->Get<std::string>("link_name");
    impl_->link_ = _model->GetLink(link_name);
    if (!impl_->link_) {
      RCLCPP_ERROR(
        impl_->ros_node_->get_logger(), "Link [%s] not found. Aborting", link_name.c_str());
      impl_->ros_node_.reset();
      return;
    }
  } else {
    RCLCPP_ERROR(impl_->ros_node_->get_logger(), "Please specify <link_name>. Aborting");
    impl_->ros_node_.reset();
    return;
  }

  impl_->link_->SetGravityMode(false);

  impl_->cl_ = 2.0 * sqrt(impl_->kl_ * impl_->link_->GetInertial()->Mass());
  impl_->ca_ = 2.0 * sqrt(impl_->ka_ * impl_->link_->GetInertial()->IXX());

  impl_->buffer_ = std::make_shared<tf2_ros::Buffer>(impl_->ros_node_->get_clock());
  impl_->transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*impl_->buffer_);
  impl_->transform_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(impl_->ros_node_);

  // Listen to the update event (broadcast every simulation iteration)
  impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&GazeboRosHandOfGodPrivate::OnUpdate, impl_.get(), std::placeholders::_1));
}

void GazeboRosHandOfGodPrivate::OnUpdate(const gazebo::common::UpdateInfo & info)
{
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE("GazeboRosHandOfGodPrivate::OnUpdate");
  IGN_PROFILE_BEGIN("lookupTransform");
#endif
  // Get TF transform relative to the /world frame
  geometry_msgs::msg::TransformStamped hog_desired_tform;
  try {
    hog_desired_tform = buffer_->lookupTransform("world", frame_ + "_desired", tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_ONCE(ros_node_->get_logger(), "%s", ex.what());
#ifdef IGN_PROFILER_ENABLE
    IGN_PROFILE_END();
#endif
    return;
  }
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_END();
  IGN_PROFILE_BEGIN("fill ROS message");
#endif
  // Convert TF transform to Gazebo Pose
  auto hog_desired = gazebo_ros::Convert<ignition::math::Pose3d>(hog_desired_tform.transform);

  // Relative transform from actual to desired pose
  ignition::math::Pose3d world_pose = link_->DirtyPose();
  ignition::math::Vector3d world_linear_vel = link_->WorldLinearVel();
  ignition::math::Vector3d relative_angular_vel = link_->RelativeAngularVel();

  ignition::math::Vector3d err_pos = hog_desired.Pos() - world_pose.Pos();
  // Get exponential coordinates for rotation
  ignition::math::Quaterniond err_rot = (ignition::math::Matrix4d(world_pose.Rot()).Inverse() *
    ignition::math::Matrix4d(hog_desired.Rot())).Rotation();

  ignition::math::Vector3d err_vec(err_rot.Log().X(), err_rot.Log().Y(), err_rot.Log().Z());

  link_->AddForce(kl_ * err_pos - cl_ * world_linear_vel);

  link_->AddRelativeTorque(ka_ * err_vec - ca_ * relative_angular_vel);

  // Convert actual pose to TransformStamped message
  geometry_msgs::msg::TransformStamped hog_actual_tform;

  hog_actual_tform.header.frame_id = "world";
  gazebo::common::Time current_time = info.simTime;
  hog_actual_tform.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(current_time);

  hog_actual_tform.child_frame_id = frame_ + "_actual";

  hog_actual_tform.transform = gazebo_ros::Convert<geometry_msgs::msg::Transform>(world_pose);
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_END();
  IGN_PROFILE_BEGIN("sendTransform");
#endif
  transform_broadcaster_->sendTransform(hog_actual_tform);
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_END();
#endif
}

GZ_REGISTER_MODEL_PLUGIN(GazeboRosHandOfGod)
}  // namespace gazebo_plugins
