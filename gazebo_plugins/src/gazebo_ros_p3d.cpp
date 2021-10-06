// Copyright 2013 Open Source Robotics Foundation, Inc.
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

#include <ignition/math/Rand.hh>
#include <gazebo_ros/node.hpp>
#include <gazebo_ros/utils.hpp>
#include <gazebo_ros/conversions/geometry_msgs.hpp>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_plugins/gazebo_ros_p3d.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#ifdef NO_ERROR
// NO_ERROR is a macro defined in Windows that's used as an enum in tf2
#undef NO_ERROR
#endif

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#ifdef IGN_PROFILER_ENABLE
#include <ignition/common/Profiler.hh>
#endif

#include <string>
#include <memory>

namespace gazebo_plugins
{

class GazeboRosP3DPrivate
{
public:
  /// Callback to be called at every simulation iteration
  /// \param[in] info Updated simulation info
  void OnUpdate(const gazebo::common::UpdateInfo & info);

  /// The link being traked.
  gazebo::physics::LinkPtr link_{nullptr};

  /// The body of the frame to display pose, twist
  gazebo::physics::LinkPtr reference_link_{nullptr};

  /// Pointer to ros node
  gazebo_ros::Node::SharedPtr ros_node_{nullptr};

  /// Odometry publisher
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_{nullptr};

  /// Odom topic name
  std::string topic_name_{"odom"};

  /// Frame transform name, should match name of reference link, or be world.
  std::string frame_name_{"world"};

  /// Constant xyz and rpy offsets
  ignition::math::Pose3d offset_;

  /// Keep track of the last update time.
  gazebo::common::Time last_time_;

  /// Publish rate in Hz.
  double update_rate_{0.0};

  /// Gaussian noise
  double gaussian_noise_;

  /// Pointer to the update event connection
  gazebo::event::ConnectionPtr update_connection_{nullptr};
};

GazeboRosP3D::GazeboRosP3D()
: impl_(std::make_unique<GazeboRosP3DPrivate>())
{
}

GazeboRosP3D::~GazeboRosP3D()
{
}

// Load the controller
void GazeboRosP3D::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
  // Configure the plugin from the SDF file
  impl_->ros_node_ = gazebo_ros::Node::Get(sdf, model);

  // Get QoS profiles
  const gazebo_ros::QoS & qos = impl_->ros_node_->get_qos();

  if (!sdf->HasElement("update_rate")) {
    RCLCPP_DEBUG(
      impl_->ros_node_->get_logger(), "p3d plugin missing <update_rate>, defaults to 0.0"
      " (as fast as possible)");
  } else {
    impl_->update_rate_ = sdf->GetElement("update_rate")->Get<double>();
  }

  std::string link_name;
  if (!sdf->HasElement("body_name")) {
    RCLCPP_ERROR(impl_->ros_node_->get_logger(), "Missing <body_name>, cannot proceed");
    return;
  } else {
    link_name = sdf->GetElement("body_name")->Get<std::string>();
  }

  impl_->link_ = model->GetLink(link_name);
  if (!impl_->link_) {
    RCLCPP_ERROR(
      impl_->ros_node_->get_logger(), "body_name: %s does not exist\n",
      link_name.c_str());
    return;
  }

  impl_->pub_ = impl_->ros_node_->create_publisher<nav_msgs::msg::Odometry>(
    impl_->topic_name_, qos.get_publisher_qos(
      impl_->topic_name_, rclcpp::SensorDataQoS().reliable()));
  impl_->topic_name_ = impl_->pub_->get_topic_name();
  RCLCPP_DEBUG(
    impl_->ros_node_->get_logger(), "Publishing on topic [%s]", impl_->topic_name_.c_str());

  if (sdf->HasElement("xyz_offsets")) {
    RCLCPP_WARN(
      impl_->ros_node_->get_logger(), "<xyz_offsets> is deprecated, use <xyz_offset> instead.");
    impl_->offset_.Pos() = sdf->GetElement("xyz_offsets")->Get<ignition::math::Vector3d>();
  }
  if (!sdf->HasElement("xyz_offset")) {
    if (!sdf->HasElement("xyz_offsets")) {
      RCLCPP_DEBUG(impl_->ros_node_->get_logger(), "Missing <xyz_offset>, defaults to 0s");
    }
  } else {
    impl_->offset_.Pos() = sdf->GetElement("xyz_offset")->Get<ignition::math::Vector3d>();
  }

  if (sdf->HasElement("rpy_offsets")) {
    RCLCPP_WARN(
      impl_->ros_node_->get_logger(), "<rpy_offsets> is deprecated, use <rpy_offset> instead.");
    impl_->offset_.Rot() = ignition::math::Quaterniond(
      sdf->GetElement("rpy_offsets")->Get<ignition::math::Vector3d>());
  }
  if (!sdf->HasElement("rpy_offset")) {
    if (!sdf->HasElement("rpy_offsets")) {
      RCLCPP_DEBUG(impl_->ros_node_->get_logger(), "Missing <rpy_offset>, defaults to 0s");
    }
  } else {
    impl_->offset_.Rot() = ignition::math::Quaterniond(
      sdf->GetElement("rpy_offset")->Get<ignition::math::Vector3d>());
  }

  if (!sdf->HasElement("gaussian_noise")) {
    RCLCPP_DEBUG(impl_->ros_node_->get_logger(), "Missing <gassian_noise>, defaults to 0.0");
    impl_->gaussian_noise_ = 0;
  } else {
    impl_->gaussian_noise_ = sdf->GetElement("gaussian_noise")->Get<double>();
  }

  impl_->last_time_ = model->GetWorld()->SimTime();

  if (!sdf->HasElement("frame_name")) {
    RCLCPP_DEBUG(
      impl_->ros_node_->get_logger(), "Missing <frame_name>, defaults to world");
  } else {
    impl_->frame_name_ = sdf->GetElement("frame_name")->Get<std::string>();
  }

  // If frame_name specified is "/world", "world", "/map" or "map" report
  // back inertial values in the gazebo world
  if (impl_->frame_name_ != "/world" && impl_->frame_name_ != "world" &&
    impl_->frame_name_ != "/map" && impl_->frame_name_ != "map")
  {
    impl_->reference_link_ = model->GetLink(impl_->frame_name_);
    if (!impl_->reference_link_) {
      RCLCPP_WARN(
        impl_->ros_node_->get_logger(), "<frame_name> [%s] does not exist.",
        impl_->frame_name_.c_str());
    }
  }

  // Listen to the update event. This event is broadcast every simulation iteration
  impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&GazeboRosP3DPrivate::OnUpdate, impl_.get(), std::placeholders::_1));
}

// Update the controller
void GazeboRosP3DPrivate::OnUpdate(const gazebo::common::UpdateInfo & info)
{
  if (!link_) {
    return;
  }
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE("GazeboRosP3DPrivate::OnUpdate");
#endif
  gazebo::common::Time current_time = info.simTime;

  if (current_time < last_time_) {
    RCLCPP_WARN(ros_node_->get_logger(), "Negative update time difference detected.");
    last_time_ = current_time;
  }

  // Rate control
  if (update_rate_ > 0 &&
    (current_time - last_time_).Double() < (1.0 / update_rate_))
  {
    return;
  }

  // If we don't have any subscribers, don't bother composing and sending the message
  if (ros_node_->count_subscribers(topic_name_) == 0) {
    return;
  }

  // Differentiate to get accelerations
  double tmp_dt = current_time.Double() - last_time_.Double();
  if (tmp_dt == 0) {
    return;
  }
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_BEGIN("fill ROS message");
#endif
  nav_msgs::msg::Odometry pose_msg;

  // Copy data into pose message
  pose_msg.header.frame_id = frame_name_;
  pose_msg.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(current_time);
  pose_msg.child_frame_id = link_->GetName();

  // Get inertial rates
  ignition::math::Vector3d vpos = link_->WorldLinearVel();
  ignition::math::Vector3d veul = link_->WorldAngularVel();

  // Get pose/orientation
  auto pose = link_->WorldPose();

  // Apply reference frame
  if (reference_link_) {
    // Convert to relative pose, rates
    auto frame_pose = reference_link_->WorldPose();
    auto frame_vpos = reference_link_->WorldLinearVel();
    auto frame_veul = reference_link_->WorldAngularVel();

    pose.Pos() = pose.Pos() - frame_pose.Pos();
    pose.Pos() = frame_pose.Rot().RotateVectorReverse(pose.Pos());
    pose.Rot() *= frame_pose.Rot().Inverse();

    vpos = frame_pose.Rot().RotateVector(vpos - frame_vpos);
    veul = frame_pose.Rot().RotateVector(veul - frame_veul);
  }

  // Apply constant offsets

  // Apply XYZ offsets and get position and rotation components
  pose.Pos() = pose.Pos() + offset_.Pos();
  // Apply RPY offsets
  pose.Rot() = offset_.Rot() * pose.Rot();
  pose.Rot().Normalize();

  // Fill out messages
  pose_msg.pose.pose.position = gazebo_ros::Convert<geometry_msgs::msg::Point>(pose.Pos());
  pose_msg.pose.pose.orientation = gazebo_ros::Convert<geometry_msgs::msg::Quaternion>(pose.Rot());

  pose_msg.twist.twist.linear.x = vpos.X() + ignition::math::Rand::DblNormal(0, gaussian_noise_);
  pose_msg.twist.twist.linear.y = vpos.Y() + ignition::math::Rand::DblNormal(0, gaussian_noise_);
  pose_msg.twist.twist.linear.z = vpos.Z() + ignition::math::Rand::DblNormal(0, gaussian_noise_);
  pose_msg.twist.twist.angular.x = veul.X() + ignition::math::Rand::DblNormal(0, gaussian_noise_);
  pose_msg.twist.twist.angular.y = veul.Y() + ignition::math::Rand::DblNormal(0, gaussian_noise_);
  pose_msg.twist.twist.angular.z = veul.Z() + ignition::math::Rand::DblNormal(0, gaussian_noise_);

  // Fill in covariance matrix
  /// @TODO: let user set separate linear and angular covariance values
  double gn2 = gaussian_noise_ * gaussian_noise_;
  pose_msg.pose.covariance[0] = gn2;
  pose_msg.pose.covariance[7] = gn2;
  pose_msg.pose.covariance[14] = gn2;
  pose_msg.pose.covariance[21] = gn2;
  pose_msg.pose.covariance[28] = gn2;
  pose_msg.pose.covariance[35] = gn2;
  pose_msg.twist.covariance[0] = gn2;
  pose_msg.twist.covariance[7] = gn2;
  pose_msg.twist.covariance[14] = gn2;
  pose_msg.twist.covariance[21] = gn2;
  pose_msg.twist.covariance[28] = gn2;
  pose_msg.twist.covariance[35] = gn2;
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_END();
  IGN_PROFILE_BEGIN("publish");
#endif
  // Publish to ROS
  pub_->publish(pose_msg);
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_END();
#endif
  // Save last time stamp
  last_time_ = current_time;
}

GZ_REGISTER_MODEL_PLUGIN(GazeboRosP3D)

}  // namespace gazebo_plugins
