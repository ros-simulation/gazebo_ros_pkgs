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

/*
 * \file  gazebo_ros_tricycle_drive.cpp
 *
 * \brief A tricycle drive plugin for gazebo.
 *
 * \author  Markus Bader <markus.bader@tuwien.ac.at>\
 *
 * \date 22th of June 2014
 */

#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo_plugins/gazebo_ros_tricycle_drive.hpp>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/conversions/geometry_msgs.hpp>
#include <gazebo_ros/node.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#ifdef IGN_PROFILER_ENABLE
#include <ignition/common/Profiler.hh>
#endif
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <sdf/sdf.hh>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

namespace gazebo_plugins
{
class GazeboRosTricycleDrivePrivate
{
public:
  /// Indicates where the odometry info is coming from
  enum OdomSource
  {
    /// Use an ancoder
    ENCODER,

    /// Use ground truth from simulation world
    WORLD,
  };

  /// Indicates which joint
  enum
  {
    /// Steering joint
    STEERING,

    /// Actuated wheel joint
    WHEEL_ACTUATED,

    /// Left wheel encoder joint
    WHEEL_ENCODER_LEFT,

    /// Right wheel encoder joint
    WHEEL_ENCODER_RIGHT
  };

  /// Callback to be called at every simulation iteration.
  /// \param[in] _info Updated simulation info.
  void OnUpdate(const gazebo::common::UpdateInfo & _info);

  /// Callback when a velocity command is received.
  /// \param[in] _msg Twist command message.
  void OnCmdVel(const geometry_msgs::msg::Twist::ConstSharedPtr cmd_msg);

  /// Update odometry according to encoder.
  /// \param[in] _current_time Current simulation time
  void UpdateOdometryEncoder(const gazebo::common::Time & _current_time);

  /// Publish odometry messages
  /// \param[in] _current_time Current simulation time
  void PublishOdometryMsg(const gazebo::common::Time & _current_time);

  /// Publish trasforms for the wheels
  /// \param[in] _current_time Current simulation time
  void PublishWheelsTf(const gazebo::common::Time & _current_time);

  /// Publish joints for the wheels
  /// \param[in] _current_time Current simulation time
  void PublishWheelJointState(const gazebo::common::Time & _current_time);

  /// Determines required speed of the wheels based on target vel
  /// \param[in] target_speed Required speed of robot
  /// \param[in] target_angle Required yaw orientation of robot
  /// \param[in] dt time difference between consecutive motor commands
  void MotorController(double target_speed, double target_angle, double dt);

  /// A pointer to the GazeboROS node.
  gazebo_ros::Node::SharedPtr ros_node_;

  /// Subscriber to command velocities
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

  /// Odometry publisher
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;

  /// Joint State publisher
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;

  /// Connection to event called at every world iteration.
  gazebo::event::ConnectionPtr update_connection_;

  /// Diameter of encoder wheels, in meters.
  double encoder_wheel_diameter_;

  /// Diameter of actuated wheels, in meters.
  double actuated_wheel_diameter_;

  /// Maximum wheel acceleration
  double max_wheel_accel_;

  /// Maximum wheel deceleration
  double max_wheel_decel_;

  /// Maximum wheel speed tolerance
  double max_wheel_speed_tol_;

  /// Maximum steering angle tolerance
  double max_steering_angle_tol_;

  /// Maximum steering speed
  double max_steering_speed_;

  /// Distance between the wheels, in meters.
  double wheel_separation_;

  /// Maximum wheel torque, in Nm.
  double max_wheel_torque_;

  /// Keep latest twist message
  geometry_msgs::msg::Twist cmd_;

  /// Keep latest joint state message
  sensor_msgs::msg::JointState joint_state_;

  /// Pointers to wheel joints.
  std::vector<gazebo::physics::JointPtr> joints_;

  /// Pointer to model.
  gazebo::physics::ModelPtr model_;

  /// To broadcast TFs
  std::shared_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;

  /// Protect variables accessed on callbacks.
  std::mutex lock_;

  /// Update period in seconds.
  double update_period_;

  /// Keep encoder data.
  geometry_msgs::msg::Pose2D pose_encoder_;

  /// Odometry frame ID
  std::string odometry_frame_;

  /// Last time the encoder was updated
  gazebo::common::Time last_odom_update_;

  /// Last time the actuator was updated
  gazebo::common::Time last_actuator_update_;

  /// Either ENCODER or WORLD
  OdomSource odom_source_;

  /// Keep latest odometry message
  nav_msgs::msg::Odometry odom_;

  /// Robot base frame ID
  std::string robot_base_frame_;

  /// True to publish wheel-to-base transforms.
  bool publish_wheel_tf_;

  /// True to publish wheel joints.
  bool publish_wheel_joint_state_;

  /// True to publish odometry messages.
  bool publish_odom_;
};

GazeboRosTricycleDrive::GazeboRosTricycleDrive()
: impl_(std::make_unique<GazeboRosTricycleDrivePrivate>())
{
}

GazeboRosTricycleDrive::~GazeboRosTricycleDrive()
{
}

void GazeboRosTricycleDrive::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  impl_->model_ = _model;

  // Initialize ROS node
  impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);

  // Get QoS profiles
  const gazebo_ros::QoS & qos = impl_->ros_node_->get_qos();

  // Odometry
  impl_->odometry_frame_ = _sdf->Get<std::string>("odometry_frame", "odom").first;
  impl_->robot_base_frame_ = _sdf->Get<std::string>("robot_base_frame", "base_link").first;
  impl_->odom_source_ = static_cast<GazeboRosTricycleDrivePrivate::OdomSource>(
    _sdf->Get<int>("odometry_source", 1).first);

  // Kinematic properties
  impl_->actuated_wheel_diameter_ = _sdf->Get<double>("actuated_wheel_diameter", 0.15).first;
  impl_->encoder_wheel_diameter_ = _sdf->Get<double>("encoder_wheel_diameter", 0.15).first;
  impl_->wheel_separation_ = _sdf->Get<double>("wheel_separation", 0.5).first;

  // Dynamic properties
  impl_->max_wheel_accel_ = _sdf->Get<double>("max_wheel_acceleration", 0).first;
  impl_->max_wheel_decel_ = _sdf->Get<double>(
    "max_wheel_deceleration", impl_->max_wheel_accel_).first;
  impl_->max_wheel_speed_tol_ = _sdf->Get<double>("max_wheel_speed_tolerance", 0.01).first;
  impl_->max_steering_speed_ = _sdf->Get<double>("max_steering_speed", 0).first;
  impl_->max_steering_angle_tol_ = _sdf->Get<double>("max_steering_angle_tolerance", 0.01).first;


  // Get joints
  impl_->joints_.resize(4);

  auto steering_joint = _sdf->Get<std::string>("steering_joint", "front_steering_joint").first;
  impl_->joints_[GazeboRosTricycleDrivePrivate::STEERING] = _model->GetJoint(steering_joint);
  if (!impl_->joints_[GazeboRosTricycleDrivePrivate::STEERING]) {
    RCLCPP_ERROR(
      impl_->ros_node_->get_logger(),
      "Steering joint [%s] not found, plugin will not work.", steering_joint.c_str());
    impl_->ros_node_.reset();
    return;
  }

  auto wheel_actuated_joint = _sdf->Get<std::string>(
    "actuated_wheel_joint", "front_wheel_joint").first;
  impl_->joints_[GazeboRosTricycleDrivePrivate::WHEEL_ACTUATED] = _model->GetJoint(
    wheel_actuated_joint);
  if (!impl_->joints_[GazeboRosTricycleDrivePrivate::WHEEL_ACTUATED]) {
    RCLCPP_ERROR(
      impl_->ros_node_->get_logger(),
      "Wheel actuated joint [%s] not found, plugin will not work.",
      wheel_actuated_joint.c_str());
    impl_->ros_node_.reset();
    return;
  }

  auto wheel_encoder_left_joint = _sdf->Get<std::string>(
    "encoder_wheel_left_joint", "left_wheel_joint").first;
  impl_->joints_[GazeboRosTricycleDrivePrivate::WHEEL_ENCODER_LEFT] = _model->GetJoint(
    wheel_encoder_left_joint);
  if (!impl_->joints_[GazeboRosTricycleDrivePrivate::WHEEL_ENCODER_LEFT]) {
    RCLCPP_ERROR(
      impl_->ros_node_->get_logger(),
      "Left wheel encoder joint [%s] not found, plugin will not work.",
      wheel_encoder_left_joint.c_str());
    impl_->ros_node_.reset();
    return;
  }

  auto wheel_encoder_right_joint = _sdf->Get<std::string>(
    "encoder_wheel_right_joint", "right_wheel_joint").first;
  impl_->joints_[GazeboRosTricycleDrivePrivate::WHEEL_ENCODER_RIGHT] = _model->GetJoint(
    wheel_encoder_right_joint);
  if (!impl_->joints_[GazeboRosTricycleDrivePrivate::WHEEL_ENCODER_RIGHT]) {
    RCLCPP_ERROR(
      impl_->ros_node_->get_logger(),
      "Right wheel encoder joint [%s] not found, plugin will not work.",
      wheel_encoder_right_joint.c_str());
    impl_->ros_node_.reset();
    return;
  }

  impl_->max_wheel_torque_ = _sdf->Get<double>("max_wheel_torque", 0.15).first;
  impl_->joints_[GazeboRosTricycleDrivePrivate::WHEEL_ACTUATED]->SetParam(
    "fmax", 0, impl_->max_wheel_torque_);
  impl_->joints_[GazeboRosTricycleDrivePrivate::STEERING]->SetParam(
    "fmax", 0, impl_->max_wheel_torque_);

  // Update rate
  auto update_rate = _sdf->Get<double>("update_rate", 100).first;
  if (update_rate > 0.0) {
    impl_->update_period_ = 1.0 / update_rate;
  } else {
    impl_->update_period_ = 0.0;
  }
  impl_->last_actuator_update_ = _model->GetWorld()->SimTime();

  impl_->cmd_vel_sub_ = impl_->ros_node_->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", qos.get_subscription_qos("cmd_vel", rclcpp::QoS(1)),
    std::bind(&GazeboRosTricycleDrivePrivate::OnCmdVel, impl_.get(), std::placeholders::_1));

  RCLCPP_INFO(
    impl_->ros_node_->get_logger(), "Subscribe to [%s]", impl_->cmd_vel_sub_->get_topic_name());

  impl_->publish_wheel_tf_ = _sdf->Get<bool>("publish_wheel_tf", false).first;

  impl_->publish_wheel_joint_state_ = _sdf->Get<bool>("publish_wheel_joint_state", false).first;
  if (impl_->publish_wheel_joint_state_) {
    impl_->joint_state_publisher_ =
      impl_->ros_node_->create_publisher<sensor_msgs::msg::JointState>(
      "joint_states", qos.get_publisher_qos("joint_states", rclcpp::QoS(1000)));
    RCLCPP_INFO(
      impl_->ros_node_->get_logger(), "Advertise joint_states on [%s]",
      impl_->joint_state_publisher_->get_topic_name());
  }

  // Advertise odometry topic
  impl_->publish_odom_ = _sdf->Get<bool>("publish_odom", false).first;
  if (impl_->publish_odom_) {
    impl_->odometry_pub_ = impl_->ros_node_->create_publisher<nav_msgs::msg::Odometry>(
      "odom", qos.get_publisher_qos("odom", rclcpp::QoS(1)));

    RCLCPP_INFO(
      impl_->ros_node_->get_logger(), "Advertise odometry on [%s]",
      impl_->odometry_pub_->get_topic_name());
  }

  // Create TF broadcaster if needed
  if (impl_->publish_wheel_tf_ || impl_->publish_odom_) {
    impl_->transform_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(
      impl_->ros_node_);

    if (impl_->publish_odom_) {
      RCLCPP_INFO(
        impl_->ros_node_->get_logger(),
        "Publishing odom transforms between [%s] and [%s]", impl_->odometry_frame_.c_str(),
        impl_->robot_base_frame_.c_str());
    }

    if (impl_->publish_wheel_tf_) {
      RCLCPP_INFO(impl_->ros_node_->get_logger(), "Publishing wheel transforms");
    }
  }

  // Initialize joint state message
  impl_->joint_state_.name.resize(impl_->joints_.size());
  impl_->joint_state_.position.resize(impl_->joints_.size());
  impl_->joint_state_.velocity.resize(impl_->joints_.size());
  impl_->joint_state_.effort.resize(impl_->joints_.size());
  for (std::size_t i = 0; i < impl_->joints_.size(); i++) {
    impl_->joint_state_.name[i] = impl_->joints_[i]->GetName();
  }

  // Initialize odom message
  impl_->odom_.header.frame_id = impl_->odometry_frame_;
  impl_->odom_.child_frame_id = impl_->robot_base_frame_;
  impl_->odom_.pose.covariance[0] = 0.00001;
  impl_->odom_.pose.covariance[7] = 0.00001;
  impl_->odom_.pose.covariance[14] = 1000000000000.0;
  impl_->odom_.pose.covariance[21] = 1000000000000.0;
  impl_->odom_.pose.covariance[28] = 1000000000000.0;
  impl_->odom_.pose.covariance[35] = 0.001;

  // listen to the update event (broadcast every simulation iteration)
  impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&GazeboRosTricycleDrivePrivate::OnUpdate, impl_.get(), std::placeholders::_1));
}

void GazeboRosTricycleDrive::Reset()
{
  std::lock_guard<std::mutex> scoped_lock(impl_->lock_);

  if (impl_->joints_[GazeboRosTricycleDrivePrivate::STEERING] &&
    impl_->joints_[GazeboRosTricycleDrivePrivate::WHEEL_ACTUATED])
  {
    gazebo::common::Time current_time =
      impl_->joints_[GazeboRosTricycleDrivePrivate::STEERING]->GetWorld()->SimTime();
    impl_->joints_[GazeboRosTricycleDrivePrivate::WHEEL_ACTUATED]->SetParam(
      "fmax", 0, impl_->max_wheel_torque_);
    impl_->joints_[GazeboRosTricycleDrivePrivate::STEERING]->SetParam(
      "fmax", 0, impl_->max_wheel_torque_);
    impl_->joints_[GazeboRosTricycleDrivePrivate::WHEEL_ACTUATED]->SetParam("vel", 0, 0.0);
    impl_->joints_[GazeboRosTricycleDrivePrivate::STEERING]->SetParam("vel", 0, 0.0);
    impl_->last_actuator_update_ = current_time;
    impl_->last_odom_update_ = current_time;
  }

  impl_->pose_encoder_.x = 0;
  impl_->pose_encoder_.y = 0;
  impl_->pose_encoder_.theta = 0;
  impl_->cmd_.linear.x = 0;
  impl_->cmd_.angular.z = 0;
}

void GazeboRosTricycleDrivePrivate::PublishWheelJointState(
  const gazebo::common::Time & _current_time)
{
  joint_state_.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(_current_time);

  for (std::size_t i = 0; i < joints_.size(); i++) {
    joint_state_.position[i] = joints_[i]->Position(0);
    joint_state_.velocity[i] = joints_[i]->GetVelocity(0);
    joint_state_.effort[i] = joints_[i]->GetForce(0);
  }
  joint_state_publisher_->publish(joint_state_);
}

void GazeboRosTricycleDrivePrivate::PublishWheelsTf(const gazebo::common::Time & _current_time)
{
  rclcpp::Time current_time = gazebo_ros::Convert<builtin_interfaces::msg::Time>(_current_time);

  for (auto & joint : joints_) {
    std::string frame = joint->GetName();
    std::string parent_frame = joint->GetParent()->GetName();

    ignition::math::Pose3d pose = joint->GetChild()->RelativePose();

    geometry_msgs::msg::TransformStamped msg;
    msg.header.stamp = current_time;
    msg.header.frame_id = parent_frame;
    msg.child_frame_id = frame;
    msg.transform.translation = gazebo_ros::Convert<geometry_msgs::msg::Vector3>(pose.Pos());
    msg.transform.rotation = gazebo_ros::Convert<geometry_msgs::msg::Quaternion>(pose.Rot());
    transform_broadcaster_->sendTransform(msg);
  }
}

void GazeboRosTricycleDrivePrivate::OnUpdate(const gazebo::common::UpdateInfo & _info)
{
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE("GazeboRosTricycleDrivePrivate::OnUpdate");
  IGN_PROFILE_BEGIN("UpdateOdometryEncoder");
#endif
  gazebo::common::Time current_time = _info.simTime;
  if (odom_source_ == ENCODER) {
    UpdateOdometryEncoder(current_time);
  }
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_END();
#endif
  double seconds_since_last_update = ( current_time - last_actuator_update_ ).Double();

  if (seconds_since_last_update < update_period_) {
    return;
  }

  if (publish_odom_) {
#ifdef IGN_PROFILER_ENABLE
    IGN_PROFILE_BEGIN("PublishOdometryMsg");
#endif
    PublishOdometryMsg(current_time);
#ifdef IGN_PROFILER_ENABLE
    IGN_PROFILE_END();
#endif
  }

  if (publish_wheel_tf_) {
#ifdef IGN_PROFILER_ENABLE
    IGN_PROFILE_BEGIN("PublishWheelsTf");
#endif
    PublishWheelsTf(current_time);
#ifdef IGN_PROFILER_ENABLE
    IGN_PROFILE_END();
#endif
  }
  if (publish_wheel_joint_state_) {
#ifdef IGN_PROFILER_ENABLE
    IGN_PROFILE_BEGIN("PublishWheelJointState");
#endif
    PublishWheelJointState(current_time);
#ifdef IGN_PROFILER_ENABLE
    IGN_PROFILE_END();
#endif
  }

  std::unique_lock<std::mutex> lock(lock_);
  double target_wheel_rotation_speed = cmd_.linear.x / ( actuated_wheel_diameter_ / 2.0 );
  double target_steering_angle = cmd_.angular.z;
  lock.unlock();

#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_BEGIN("MotorController");
#endif
  MotorController(
    target_wheel_rotation_speed, target_steering_angle, seconds_since_last_update);
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_END();
#endif
  //  RCLCPP_INFO(ros_node_->get_logger(),
  //  "v = %f, w = %f ", target_wheel_rotation_speed, target_steering_angle);

  last_actuator_update_ = _info.simTime;
}

void GazeboRosTricycleDrivePrivate::MotorController(
  double target_speed, double target_angle, double dt)
{
  double applied_speed = target_speed;
  double applied_angle = target_angle;

  double current_speed = joints_[WHEEL_ACTUATED]->GetVelocity(0);
  if (max_wheel_accel_ > 0 || max_wheel_decel_ > 0) {
    double diff_speed = current_speed - target_speed;
    if (fabs(diff_speed) < max_wheel_speed_tol_) {
      applied_speed = current_speed;
    } else if (-diff_speed > max_wheel_accel_ * dt) {
      applied_speed = current_speed + max_wheel_accel_ * dt;
    } else if (diff_speed > max_wheel_decel_ * dt) {
      applied_speed = current_speed - max_wheel_decel_ * dt;
    }
  }

  joints_[WHEEL_ACTUATED]->SetParam("vel", 0, applied_speed);

  double current_angle = joints_[STEERING]->Position(0);

  // truncate target angle
  if (target_angle > +M_PI / 2.0) {
    target_angle = +M_PI / 2.0;
  } else if (target_angle < -M_PI / 2.0) {
    target_angle = -M_PI / 2.0;
  }

  // if max_steering_speed_ is > 0, use speed control, otherwise use position control
  // With position control, one cannot expect dynamics to work correctly.
  double diff_angle = current_angle - target_angle;
  double applied_steering_speed = 0;
  if (max_steering_speed_ > 0) {
    // this means we will steer using steering speed
    if (fabs(diff_angle) < max_steering_angle_tol_) {
      // we're withing angle tolerance
      applied_steering_speed = 0;
    } else if (diff_angle < target_speed) {
      // steer toward target angle
      applied_steering_speed = max_steering_speed_;
    } else {
      // steer toward target angle
      applied_steering_speed = -max_steering_speed_;
    }

    // use speed control, not recommended, for better dynamics use force control
    joints_[STEERING]->SetParam("vel", 0, applied_steering_speed);
  } else {
    // max_steering_speed_ is zero, use position control.
    // This is not a good idea if we want dynamics to work.
    if (fabs(diff_angle) < max_steering_speed_ * dt) {
      // we can take a step and still not overshoot target
      if (diff_angle > 0) {
        applied_angle = current_angle - max_steering_speed_ * dt;
      } else {
        applied_angle = current_angle + max_steering_speed_ * dt;
      }
    } else {
      applied_angle = target_angle;
    }

    joints_[STEERING]->SetPosition(0, applied_angle, true);
  }
  //  RCLCPP_INFO(ros_node_->get_logger(),
  //  "target: [%3.2f, %3.2f], current: [%3.2f, %3.2f], applied: [%3.2f, %3.2f/%3.2f] ",
  //   target_speed, target_angle, current_speed, current_angle, applied_speed,
  //   applied_angle, applied_steering_speed );
}

void GazeboRosTricycleDrivePrivate::OnCmdVel(
  const geometry_msgs::msg::Twist::ConstSharedPtr cmd_msg)
{
  std::lock_guard<std::mutex> scoped_lock(lock_);
  cmd_.linear.x = cmd_msg->linear.x;
  cmd_.angular.z = cmd_msg->angular.z;
}

void GazeboRosTricycleDrivePrivate::UpdateOdometryEncoder(
  const gazebo::common::Time & _current_time)
{
  double vl = joints_[WHEEL_ENCODER_LEFT]->GetVelocity(0);
  double vr = joints_[WHEEL_ENCODER_RIGHT]->GetVelocity(0);

  double seconds_since_last_update = (_current_time - last_odom_update_).Double();
  last_odom_update_ = _current_time;

  double b = wheel_separation_;

  // Book: Sigwart 2011 Autonomous Mobile Robots page:337
  double sl = vl * (encoder_wheel_diameter_ / 2.0) * seconds_since_last_update;
  double sr = vr * (encoder_wheel_diameter_ / 2.0) * seconds_since_last_update;

  double dx = (sl + sr) / 2.0 * cos(pose_encoder_.theta + (sl - sr) / (2.0 * b));
  double dy = (sl + sr) / 2.0 * sin(pose_encoder_.theta + (sl - sr) / (2.0 * b));
  double dtheta = (sl - sr) / b;

  pose_encoder_.x += dx;
  pose_encoder_.y += dy;
  pose_encoder_.theta += dtheta;

  double w = dtheta / seconds_since_last_update;

  tf2::Vector3 vt;
  vt = tf2::Vector3(pose_encoder_.x, pose_encoder_.y, 0);
  odom_.pose.pose.position.x = vt.x();
  odom_.pose.pose.position.y = vt.y();
  odom_.pose.pose.position.z = vt.z();

  tf2::Quaternion qt;
  qt.setRPY(0, 0, pose_encoder_.theta);
  odom_.pose.pose.orientation = tf2::toMsg(qt);

  odom_.twist.twist.angular.z = w;
  odom_.twist.twist.linear.x = dx / seconds_since_last_update;
  odom_.twist.twist.linear.y = dy / seconds_since_last_update;
}

void GazeboRosTricycleDrivePrivate::PublishOdometryMsg(const gazebo::common::Time & _current_time)
{
  rclcpp::Time current_time = gazebo_ros::Convert<builtin_interfaces::msg::Time>(_current_time);

  if (odom_source_ == WORLD) {
    // getting data form gazebo world
    ignition::math::Pose3d pose = model_->WorldPose();

    odom_.pose.pose.position = gazebo_ros::Convert<geometry_msgs::msg::Point>(pose.Pos());
    odom_.pose.pose.orientation = gazebo_ros::Convert<geometry_msgs::msg::Quaternion>(pose.Rot());

    // get velocity in /odom frame
    ignition::math::Vector3d linear;
    linear = model_->WorldLinearVel();
    odom_.twist.twist.angular.z = model_->WorldAngularVel().Z();


    // convert velocity to child_frame_id (aka base_footprint)
    auto yaw = static_cast<float>(pose.Rot().Yaw());
    odom_.twist.twist.linear.x = cosf(yaw) * linear.X() + sinf(yaw) * linear.Y();
    odom_.twist.twist.linear.y = cosf(yaw) * linear.Y() - sinf(yaw) * linear.X();
  }

  geometry_msgs::msg::TransformStamped msg;
  msg.header.stamp = current_time;
  msg.header.frame_id = odometry_frame_;
  msg.child_frame_id = robot_base_frame_;
  msg.transform.translation =
    gazebo_ros::Convert<geometry_msgs::msg::Vector3>(odom_.pose.pose.position);
  msg.transform.rotation = odom_.pose.pose.orientation;

  transform_broadcaster_->sendTransform(msg);

  // set header stamp
  odom_.header.stamp = current_time;

  odometry_pub_->publish(odom_);
}

GZ_REGISTER_MODEL_PLUGIN(GazeboRosTricycleDrive)
}  // namespace gazebo_plugins
