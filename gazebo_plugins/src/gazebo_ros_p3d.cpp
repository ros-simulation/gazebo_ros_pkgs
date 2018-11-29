/*
 * Copyright 2013 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <gazebo_ros/node.hpp>
#include <gazebo_ros/utils.hpp>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_plugins/gazebo_ros_p3d.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <nav_msgs/msg/odometry.hpp>

#include <string>
#include <memory>

namespace gazebo_plugins
{

class GazeboRosP3DPrivate
{
public:
  gazebo::physics::WorldPtr world_;
  gazebo::physics::ModelPtr model_;

  /// \brief The parent Model
  gazebo::physics::LinkPtr link_;

  /// \brief The body of the frame to display pose, twist
  gazebo::physics::LinkPtr reference_link_;

  /// \brief pointer to ros node
  gazebo_ros::Node::SharedPtr ros_node_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;

  /// \brief store bodyname
  std::string link_name_;

  /// \brief topic name
  std::string topic_name_;

  /// \brief frame transform name, should match link name
  std::string frame_name_;

  /// \brief allow specifying constant xyz and rpy offsets
  ignition::math::Pose3d offset_;

  /// \brief save last_time
  gazebo::common::Time last_time_;

  // rate control
  double update_rate_;

  /// \brief Gaussian noise
  double gaussian_noise_;

  /// \brief for setting ROS name space
  std::string robot_namespace_;

  // Pointer to the update event connection
  gazebo::event::ConnectionPtr update_connection_;
};

GazeboRosP3D::GazeboRosP3D()
: impl_(std::make_unique<GazeboRosP3DPrivate>())
{
}

GazeboRosP3D::~GazeboRosP3D()
{
}

// Load the controller
void GazeboRosP3D::Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  // Create ROS configured from sdf
  impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);

  // Get the world name
  impl_->world_ = _parent->GetWorld();
  impl_->model_ = _parent;

  if (!_sdf->HasElement("update_rate")) {
    RCLCPP_DEBUG(impl_->ros_node_->get_logger(), "p3d plugin missing <update_rate>, defaults to 0.0"
      " (as fast as possible)");
    impl_->update_rate_ = 0;
  } else {
    impl_->update_rate_ = _sdf->GetElement("update_rate")->Get<double>();
  }

  if (!_sdf->HasElement("body_name")) {
    RCLCPP_ERROR(impl_->ros_node_->get_logger(), "p3d plugin missing <body_name>, cannot proceed");
    return;
  } else {
    impl_->link_name_ = _sdf->GetElement("body_name")->Get<std::string>();
  }

  impl_->link_ = _parent->GetLink(impl_->link_name_);
  if (!impl_->link_) {
    RCLCPP_ERROR(
      impl_->ros_node_->get_logger(), "gazebo_ros_p3d plugin error: body_name: %s does not exist\n",
      impl_->link_name_.c_str());
    return;
  }

  if (!_sdf->HasElement("topic_name")) {
    RCLCPP_ERROR(impl_->ros_node_->get_logger(), "p3d plugin missing <topic_name>, cannot proceed");
    return;
  } else {
    impl_->topic_name_ = _sdf->GetElement("topic_name")->Get<std::string>();
  }

  if (impl_->topic_name_ != "") {
    impl_->pub_ = impl_->ros_node_->create_publisher<nav_msgs::msg::Odometry>(impl_->topic_name_);
  }

  if (!_sdf->HasElement("frame_name")) {
    RCLCPP_DEBUG(
      impl_->ros_node_->get_logger(), "p3d plugin missing <frame_name>, defaults to world");
    impl_->frame_name_ = "world";
  } else {
    impl_->frame_name_ = _sdf->GetElement("frame_name")->Get<std::string>();
  }

  if (!_sdf->HasElement("xyz_offsets")) {
    RCLCPP_DEBUG(impl_->ros_node_->get_logger(),
      "p3d plugin missing <xyz_offsets>, defaults to 0s");
    impl_->offset_.Pos() = ignition::math::Vector3d(0, 0, 0);
  } else {
    impl_->offset_.Pos() = _sdf->GetElement("xyz_offsets")->Get<ignition::math::Vector3d>();
  }

  if (!_sdf->HasElement("rpy_offsets")) {
    RCLCPP_DEBUG(impl_->ros_node_->get_logger(),
      "p3d plugin missing <rpy_offsets>, defaults to 0s");
    impl_->offset_.Rot() = ignition::math::Quaterniond(ignition::math::Vector3d(0, 0, 0));
  } else {
    impl_->offset_.Rot() = ignition::math::Quaterniond(_sdf->GetElement(
          "rpy_offsets")->Get<ignition::math::Vector3d>());
  }

  if (!_sdf->HasElement("gaussian_noise")) {
    RCLCPP_DEBUG(
      impl_->ros_node_->get_logger(), "p3d plugin missing <gassian_noise>, defaults to 0.0");
    impl_->gaussian_noise_ = 0;
  } else {
    impl_->gaussian_noise_ = _sdf->GetElement("gaussian_noise")->Get<double>();
  }

  impl_->last_time_ = impl_->world_->SimTime();

  // If frame_name specified is "/world", "world", "/map" or "map" report
  // back inertial values in the gazebo world
  if (impl_->frame_name_ != "/world" &&
    impl_->frame_name_ != "world" &&
    impl_->frame_name_ != "/map" &&
    impl_->frame_name_ != "map")
  {
    impl_->reference_link_ = impl_->model_->GetLink(impl_->frame_name_);
    if (!impl_->reference_link_) {
      RCLCPP_ERROR(
        impl_->ros_node_->get_logger(), "gazebo_ros_p3d plugin: frame_name: %s does not exist, will"
        " not publish pose\n", impl_->frame_name_.c_str());
      return;
    }
  }

  // New Mechanism for Updating every World Cycle
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    boost::bind(&GazeboRosP3D::UpdateChild, this));
}

// Update the controller
void GazeboRosP3D::UpdateChild()
{
  if (!impl_->link_) {
    return;
  }

  gazebo::common::Time current_time = impl_->world_->SimTime();

  if (current_time < impl_->last_time_) {
    RCLCPP_WARN(impl_->ros_node_->get_logger(), "Negative update time difference detected.");
    impl_->last_time_ = current_time;
  }

  // Rate control
  if (impl_->update_rate_ > 0 &&
    (current_time - impl_->last_time_).Double() < (1.0 / impl_->update_rate_))
  {
    return;
  }

  // If we don't have any subscribers, don't bother composing and sending the message
  if (impl_->ros_node_->count_subscribers(impl_->topic_name_) == 0) {
    return;
  }

  // Differentiate to get accelerations
  double tmp_dt = current_time.Double() - impl_->last_time_.Double();
  if (tmp_dt == 0) {
    return;
  }

  if (impl_->topic_name_ != "") {
    nav_msgs::msg::Odometry pose_msg;

    // copy data into pose message
    pose_msg.header.frame_id = impl_->frame_name_;
    pose_msg.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(current_time);
    pose_msg.child_frame_id = impl_->link_name_;

    ignition::math::Pose3d pose, frame_pose;
    ignition::math::Vector3d frame_vpos;
    ignition::math::Vector3d frame_veul;

    // Get inertial rates
    // Get Pose/Orientation
    ignition::math::Vector3d vpos = impl_->link_->WorldLinearVel();
    ignition::math::Vector3d veul = impl_->link_->WorldAngularVel();

    pose = impl_->link_->WorldPose();

    // Apply reference frame
    if (impl_->reference_link_) {
      // Convert to relative pose, rates
      frame_pose = impl_->reference_link_->WorldPose();
      frame_vpos = impl_->reference_link_->WorldLinearVel();
      frame_veul = impl_->reference_link_->WorldAngularVel();

      pose.Pos() = pose.Pos() - frame_pose.Pos();
      pose.Pos() = frame_pose.Rot().RotateVectorReverse(pose.Pos());
      pose.Rot() *= frame_pose.Rot().Inverse();

      vpos = frame_pose.Rot().RotateVector(vpos - frame_vpos);
      veul = frame_pose.Rot().RotateVector(veul - frame_veul);
    }

    // Apply constant offsets

    // Apply xyz offsets and get position and rotation components
    pose.Pos() = pose.Pos() + impl_->offset_.Pos();
    // Apply rpy offsets
    pose.Rot() = impl_->offset_.Rot() * pose.Rot();
    pose.Rot().Normalize();

    // Fill out messages
    pose_msg.pose.pose.position.x = pose.Pos().X();
    pose_msg.pose.pose.position.y = pose.Pos().Y();
    pose_msg.pose.pose.position.z = pose.Pos().Z();
    pose_msg.pose.pose.orientation.x = pose.Rot().X();
    pose_msg.pose.pose.orientation.y = pose.Rot().Y();
    pose_msg.pose.pose.orientation.z = pose.Rot().Z();
    pose_msg.pose.pose.orientation.w = pose.Rot().W();

    pose_msg.twist.twist.linear.x = vpos.X() + GaussianKernel(0, impl_->gaussian_noise_);
    pose_msg.twist.twist.linear.y = vpos.Y() + GaussianKernel(0, impl_->gaussian_noise_);
    pose_msg.twist.twist.linear.z = vpos.Z() +
      GaussianKernel(0, impl_->gaussian_noise_);
    // pass euler angular rates
    pose_msg.twist.twist.angular.x = veul.X() +
      GaussianKernel(0, impl_->gaussian_noise_);
    pose_msg.twist.twist.angular.y = veul.Y() +
      GaussianKernel(0, impl_->gaussian_noise_);
    pose_msg.twist.twist.angular.z = veul.Z() +
      GaussianKernel(0, impl_->gaussian_noise_);

    // Fill in covariance matrix
    /// @TODO: let user set separate linear and angular covariance values
    double gn2 = impl_->gaussian_noise_ * impl_->gaussian_noise_;
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

    // Publish to ROS
    impl_->pub_->publish(pose_msg);
  }

  // Save last time stamp
  impl_->last_time_ = current_time;
}

// Utility for adding noise
double GazeboRosP3D::GaussianKernel(double mu, double sigma)
{
  // Using Box-Muller transform to generate two independent standard
  // normally disbributed normal variables see wikipedia

  unsigned int seed = 0;

  // Normalized uniform random variable
  double U = static_cast<double>(rand_r(&seed)) /
    static_cast<double>(RAND_MAX);

  // Normalized uniform random variable
  double V = static_cast<double>(rand_r(&seed)) /
    static_cast<double>(RAND_MAX);

  double X = sqrt(-2.0 * ::log(U)) * cos(2.0 * M_PI * V);

  // There are 2 indep. vars, we'll just use X
  // scale to our mu and sigma
  X = sigma * X + mu;
  return X;
}

GZ_REGISTER_MODEL_PLUGIN(GazeboRosP3D)

}  // namespace gazebo_plugins
