// Copyright (c) 2010, Daniel Hewlett, Antons Rebguns
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
 * \file  gazebo_ros_diff_drive.cpp
 *
 * \brief A differential drive plugin for gazebo. Based on the diffdrive plugin
 * developed for the erratic robot (see copyright notice above). The original
 * plugin can be found in the ROS package gazebo_erratic_plugins.
 *
 * \author  Piyush Khandelwal (piyushk@gmail.com)
 *
 * $ Id: 06/21/2013 11:23:40 AM piyushk $
 */

/*
 *
 * The support of acceleration limit was added by
 * \author   George Todoran <todorangrg@gmail.com>
 * \author   Markus Bader <markus.bader@tuwien.ac.at>
 * \date 22th of May 2014
 */

/* 
 * \file gazebo_ros_omni_3wd_drive.cpp
 * 
 * \brief A omnidirectional drive plugin for gazebo. It is based on the diff_drive
 * developed by P. Khandelwal and improved by G. Todoran and M. Bader
 * (see above notes). The diff_drive plugin can be found in this package, as 
 * gazebo_ros_diff_drive.cpp.
 * \author   Enrico Sutera (enricosutera@outlook.com)
 * \date 10th of November 2020
 */

#include <gazebo/common/Time.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo_plugins/gazebo_ros_omni_3wd_drive.hpp>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/conversions/geometry_msgs.hpp>
#include <gazebo_ros/node.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sdf/sdf.hh>

#ifdef NO_ERROR
// NO_ERROR is a macro defined in Windows that's used as an enum in tf2
#undef NO_ERROR
#endif

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <string>
#include <vector>

namespace gazebo_plugins
{
class GazeboRosOmni3wdDrivePrivate
{
public:
  /// Indicates where the odometry info is coming from
  enum OdomSource
  {
    /// Use an ancoder
    ENCODER = 0,

    /// Use ground truth from simulation world
    WORLD = 1,
  };

  /// Indicates which wheel
  enum
  {
    /// Right wheel
    RIGHT = 0,

    /// Left wheel
    LEFT = 1,

    /// Back wheel
    BACK = 2,
  };

  /// Callback to be called at every simulation iteration.
  /// \param[in] _info Updated simulation info.
  void OnUpdate(const gazebo::common::UpdateInfo & _info);

  /// Callback when a velocity command is received.
  /// \param[in] _msg Twist command message.
  void OnCmdVel(const geometry_msgs::msg::Twist::SharedPtr _msg);

  /// Update wheel velocities according to latest target velocities.
  void UpdateWheelVelocities();

  /// Update odometry according to encoder.
  /// \param[in] _current_time Current simulation time
  void UpdateOdometryEncoder(const gazebo::common::Time & _current_time);

  /// Update odometry according to world
  void UpdateOdometryWorld();

  /// Publish odometry transforms
  /// \param[in] _current_time Current simulation time
  void PublishOdometryTf(const gazebo::common::Time & _current_time);

  /// Publish trasforms for the wheels
  /// \param[in] _current_time Current simulation time
  void PublishWheelsTf(const gazebo::common::Time & _current_time);

  /// Publish odometry messages
  /// \param[in] _current_time Current simulation time
  void PublishOdometryMsg(const gazebo::common::Time & _current_time);

  /// A pointer to the GazeboROS node.
  gazebo_ros::Node::SharedPtr ros_node_;

  /// Subscriber to command velocities
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

  /// Odometry publisher
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;

  /// Connection to event called at every world iteration.
  gazebo::event::ConnectionPtr update_connection_;

  /// Distance between the wheels, in meters.
  ///####################################################################################################
  //std::vector<double> wheel_separation_;
  double wheel_separation_;

  /// Diameter of wheels, in meters.
  ///####################################################################################################
  //std::vector<double> wheel_diameter_;
  double wheel_diameter_;
  /// Maximum wheel torque, in Nm.
  ///####################################################################################################
  double max_wheel_torque_;

  /// Maximum wheel acceleration
  ///####################################################################################################
  double max_wheel_accel_;

  /// Desired wheel speed.
  std::vector<double> desired_wheel_speed_;

  /// Speed sent to wheel.
  std::vector<double> wheel_speed_instr_;

  /// Pointers to wheel joints.
  std::vector<gazebo::physics::JointPtr> joints_;

  /// Pointer to model.
  gazebo::physics::ModelPtr model_;

  /// To broadcast TFs
  std::shared_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;

  /// Protect variables accessed on callbacks.
  std::mutex lock_;

  /// Linear velocity in X received on command (m/s).
  double target_x_{0.0};

  /// Linear velocity in Y received on command (m/s).
  double target_y_{0.0};

  /// Angular velocity in Z received on command (rad/s).
  double target_rot_{0.0};

  /// Update period in seconds.
  double update_period_;

  /// Last update time.
  gazebo::common::Time last_update_time_;

  /// Keep encoder data.
  geometry_msgs::msg::Pose2D pose_encoder_;

  /// Odometry frame ID
  std::string odometry_frame_;

  /// Last time the encoder was updated
  gazebo::common::Time last_encoder_update_;

  /// Either ENCODER or WORLD
  OdomSource odom_source_;

  /// Keep latest odometry message
  nav_msgs::msg::Odometry odom_;

  /// Robot base frame ID
  std::string robot_base_frame_;

  /// True to publish odometry messages.
  bool publish_odom_;

  /// True to publish wheel-to-base transforms.
  bool publish_wheel_tf_;

  /// True to publish odom-to-world transforms.
  bool publish_odom_tf_;

  ///####################################################################################################
  /// Store number of wheel pairs
  unsigned int num_wheel_pairs_;

  /// Covariance in odometry
  double covariance_[3];
};

///####################################################################################################
GazeboRosOmni3wdDrive::GazeboRosOmni3wdDrive()
: impl_(std::make_unique<GazeboRosOmni3wdDrivePrivate>())
{
}

/// OK
GazeboRosOmni3wdDrive::~GazeboRosOmni3wdDrive()
{
}

/// OK 
void GazeboRosOmni3wdDrive::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  impl_->model_ = _model;

  // Initialize ROS node 
  impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);

  // Dynamic properties
  impl_->max_wheel_accel_ = _sdf->Get<double>("max_wheel_acceleration", 0.0).first;
  impl_->max_wheel_torque_ = _sdf->Get<double>("max_wheel_torque", 5.0).first;

  // Get joints and Kinematic properties
  std::vector<gazebo::physics::JointPtr> left_joints, right_joints, back_joints;

  // Get left wheel joint from sdf
  auto left_joint_elem = _sdf->GetElement("left_joint");
  auto left_joint_name = left_joint_elem->Get<std::string>();
  auto left_joint = _model->GetJoint(left_joint_name);
  if (!left_joint) {
      RCLCPP_ERROR(impl_->ros_node_->get_logger(),
        "Joint [%s] not found, plugin will not work.", left_joint_name.c_str());
      impl_->ros_node_.reset();
      return;
   }
  left_joint->SetParam("fmax", 0, impl_->max_wheel_torque_);
  left_joints.push_back(left_joint);
  RCLCPP_INFO(impl_->ros_node_->get_logger(),"Left wheel found");
  
  // Get right wheel joint from sdf
  auto right_joint_elem = _sdf->GetElement("right_joint");
  auto right_joint_name = right_joint_elem->Get<std::string>();
  auto right_joint = _model->GetJoint(right_joint_name);
  if (!right_joint) {
      RCLCPP_ERROR(impl_->ros_node_->get_logger(),
        "Joint [%s] not found, plugin will not work.", right_joint_name.c_str());
      impl_->ros_node_.reset();
      return;
   }
  right_joint->SetParam("fmax", 0, impl_->max_wheel_torque_);
  right_joints.push_back(right_joint);


  // Get back wheel joint from sdf
  auto back_joint_elem = _sdf->GetElement("back_joint");
  auto back_joint_name = back_joint_elem->Get<std::string>();
  auto back_joint = _model->GetJoint(back_joint_name);
  if (!back_joint) {
      RCLCPP_ERROR(impl_->ros_node_->get_logger(),
        "Joint [%s] not found, plugin will not work.", back_joint_name.c_str());
      impl_->ros_node_.reset();
      return;
   }
  back_joint->SetParam("fmax", 0, impl_->max_wheel_torque_);
  back_joints.push_back(back_joint);

  /*if (left_joints.size() != right_joints.size() || left_joints.size() != impl_->num_wheel_pairs_) {
    RCLCPP_ERROR(impl_->ros_node_->get_logger(),
      "Inconsistent number of joints specified. Plugin will not work.");
    impl_->ros_node_.reset();
    return;
  }*/

  // Add element to joints_ vector
  impl_->joints_.push_back(right_joints[0]);
  impl_->joints_.push_back(left_joints[0]);
  impl_->joints_.push_back(back_joints[0]);

  // Get <wheel_separation> from sdf
  auto wheel_separation = _sdf->Get<double>("wheel_separation",0.2).first;
  impl_->wheel_separation_ = wheel_separation;
  RCLCPP_INFO(impl_->ros_node_->get_logger(),
      "Wheel separation set to [%fm]", impl_->wheel_separation_);

  // Get <wheel_diameter> from sdf
  auto wheel_diameter = _sdf->Get<double>("wheel_diameter",0.15).first;
  impl_->wheel_diameter_ = wheel_diameter;
  RCLCPP_INFO(impl_->ros_node_->get_logger(),
       "Wheel  diameter set to [%fm]", impl_->wheel_diameter_);

  // Initialize vectors. 3 is the number of wheels
  impl_->wheel_speed_instr_.assign(3, 0);
  impl_->desired_wheel_speed_.assign(3, 0);

  // Update rate
  auto update_rate = _sdf->Get<double>("update_rate", 100.0).first;
  if (update_rate > 0.0) {
    impl_->update_period_ = 1.0 / update_rate;
  } else {
    impl_->update_period_ = 0.0;
  }
  impl_->last_update_time_ = _model->GetWorld()->SimTime();

  impl_->cmd_vel_sub_ = impl_->ros_node_->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", rclcpp::QoS(rclcpp::KeepLast(1)),
    std::bind(&GazeboRosOmni3wdDrivePrivate::OnCmdVel, impl_.get(), std::placeholders::_1));

  RCLCPP_INFO(impl_->ros_node_->get_logger(), "Subscribed to [%s]",
    impl_->cmd_vel_sub_->get_topic_name());

  // Odometry
  impl_->odometry_frame_ = _sdf->Get<std::string>("odometry_frame", "odom").first;
  impl_->robot_base_frame_ = _sdf->Get<std::string>("robot_base_frame", "base_footprint").first;
  impl_->odom_source_ = static_cast<GazeboRosOmni3wdDrivePrivate::OdomSource>(
    _sdf->Get<int>("odometry_source", 1).first);

  // Advertise odometry topic
  impl_->publish_odom_ = _sdf->Get<bool>("publish_odom", false).first;
  if (impl_->publish_odom_) {
    impl_->odometry_pub_ = impl_->ros_node_->create_publisher<nav_msgs::msg::Odometry>(
      "odom", rclcpp::QoS(rclcpp::KeepLast(1)));

    RCLCPP_INFO(impl_->ros_node_->get_logger(), "Advertise odometry on [%s]",
      impl_->odometry_pub_->get_topic_name());
  }

  // Create TF broadcaster if needed
  impl_->publish_wheel_tf_ = _sdf->Get<bool>("publish_wheel_tf", false).first;
  impl_->publish_odom_tf_ = _sdf->Get<bool>("publish_odom_tf", false).first;
  if (impl_->publish_wheel_tf_ || impl_->publish_odom_tf_) {
    impl_->transform_broadcaster_ =
      std::make_shared<tf2_ros::TransformBroadcaster>(impl_->ros_node_);

    if (impl_->publish_odom_tf_) {
      RCLCPP_INFO(impl_->ros_node_->get_logger(),
        "Publishing odom transforms between [%s] and [%s]", impl_->odometry_frame_.c_str(),
        impl_->robot_base_frame_.c_str());
    }

    //for (index = 0; index < impl_->num_wheel_pairs_; ++index) {

    if (impl_->publish_wheel_tf_) {
      RCLCPP_INFO(impl_->ros_node_->get_logger(),
        "Publishing wheel transforms between [%s], [%s], [%s] and [%s]",
        impl_->robot_base_frame_.c_str(),
        impl_->joints_[GazeboRosOmni3wdDrivePrivate::LEFT]->GetName().c_str(),
        impl_->joints_[GazeboRosOmni3wdDrivePrivate::RIGHT]->GetName().c_str(),
        impl_->joints_[GazeboRosOmni3wdDrivePrivate::BACK]->GetName().c_str());
    }
  }

  impl_->covariance_[0] = _sdf->Get<double>("covariance_x", 0.00001).first;
  impl_->covariance_[1] = _sdf->Get<double>("covariance_y", 0.00001).first;
  impl_->covariance_[2] = _sdf->Get<double>("covariance_yaw", 0.001).first;

  // Listen to the update event (broadcast every simulation iteration)
  impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&GazeboRosOmni3wdDrivePrivate::OnUpdate, impl_.get(), std::placeholders::_1));
  RCLCPP_INFO(impl_->ros_node_->get_logger(), "Load Finished");
}

// OK
void GazeboRosOmni3wdDrive::Reset()
{
  impl_->last_update_time_ =
    impl_->joints_[GazeboRosOmni3wdDrivePrivate::LEFT]->GetWorld()->SimTime();
  //for (unsigned int i = 0; i < impl_->num_wheel_pairs_; ++i) {
  
  if (impl_->joints_[GazeboRosOmni3wdDrivePrivate::LEFT] &&
    impl_->joints_[GazeboRosOmni3wdDrivePrivate::RIGHT]  &&
    impl_->joints_[GazeboRosOmni3wdDrivePrivate::BACK])
  {
    impl_->joints_[GazeboRosOmni3wdDrivePrivate::LEFT]->SetParam(
      "fmax", 0, impl_->max_wheel_torque_);
    impl_->joints_[GazeboRosOmni3wdDrivePrivate::RIGHT]->SetParam(
      "fmax", 0, impl_->max_wheel_torque_);
    impl_->joints_[GazeboRosOmni3wdDrivePrivate::BACK]->SetParam(
      "fmax", 0, impl_->max_wheel_torque_);
  }
  
  impl_->pose_encoder_.x = 0;
  impl_->pose_encoder_.y = 0;
  impl_->pose_encoder_.theta = 0;
  impl_->target_x_ = 0;
  impl_->target_y_ = 0;
  impl_->target_rot_ = 0;
}

// 
void GazeboRosOmni3wdDrivePrivate::OnUpdate(const gazebo::common::UpdateInfo & _info)
{
  // TO DO
  // wheel_diameter to double
  // Update encoder even if we're going to skip this update
  if (odom_source_ == ENCODER) {
    UpdateOdometryEncoder(_info.simTime);
  }

  double seconds_since_last_update = (_info.simTime - last_update_time_).Double();

  if (seconds_since_last_update < update_period_) {
    return;
  }

  // Update odom message if using ground truth
  if (odom_source_ == WORLD) {
    UpdateOdometryWorld();
  }

  if (publish_odom_) {
    PublishOdometryMsg(_info.simTime);
  }

  if (publish_wheel_tf_) {
    PublishWheelsTf(_info.simTime);
  }

  if (publish_odom_tf_) {
    PublishOdometryTf(_info.simTime);
  }

  // Update robot in case new velocities have been requested
  UpdateWheelVelocities();

  // Current speed
  std::vector<double> current_speed(3);
    current_speed[LEFT] =
      joints_[LEFT]->GetVelocity(0) * (wheel_diameter_ / 2.0);
    current_speed[RIGHT] =
      joints_[RIGHT]->GetVelocity(0) * (wheel_diameter_ / 2.0);
    current_speed[BACK] =
      joints_[BACK]->GetVelocity(0) * (wheel_diameter_ / 2.0);

  // If max_accel == 0, or target speed is reached
  if (max_wheel_accel_ == 0 ||
    (fabs(desired_wheel_speed_[LEFT] - current_speed[LEFT]) < 0.01)   ||
    (fabs(desired_wheel_speed_[RIGHT] - current_speed[RIGHT]) < 0.01) ||
    (fabs(desired_wheel_speed_[BACK] - current_speed[BACK]) < 0.01))
  {
    joints_[LEFT]->SetParam(
      "vel", 0, desired_wheel_speed_[LEFT] / (wheel_diameter_ / 2.0));
    joints_[RIGHT]->SetParam(
      "vel", 0, desired_wheel_speed_[RIGHT] / (wheel_diameter_ / 2.0));
    joints_[BACK]->SetParam(
      "vel", 0, desired_wheel_speed_[BACK] / (wheel_diameter_ / 2.0));
  } else {
    if (desired_wheel_speed_[LEFT] >= current_speed[LEFT]) {
      wheel_speed_instr_[LEFT] += fmin(desired_wheel_speed_[LEFT] -
          current_speed[LEFT], max_wheel_accel_ * seconds_since_last_update);
    } else {
      wheel_speed_instr_[LEFT] += fmax(desired_wheel_speed_[LEFT] -
          current_speed[LEFT], -max_wheel_accel_ * seconds_since_last_update);
    }

    if (desired_wheel_speed_[RIGHT] > current_speed[RIGHT]) {
      wheel_speed_instr_[RIGHT] += fmin(desired_wheel_speed_[RIGHT] -
          current_speed[RIGHT], max_wheel_accel_ * seconds_since_last_update);
    } else {
      wheel_speed_instr_[RIGHT] += fmax(desired_wheel_speed_[RIGHT] -
          current_speed[RIGHT], -max_wheel_accel_ * seconds_since_last_update);
    }

    if (desired_wheel_speed_[BACK] > current_speed[BACK]) {
      wheel_speed_instr_[BACK] += fmin(desired_wheel_speed_[BACK] -
          current_speed[BACK], max_wheel_accel_ * seconds_since_last_update);
    } else {
      wheel_speed_instr_[BACK] += fmax(desired_wheel_speed_[BACK] -
          current_speed[BACK], -max_wheel_accel_ * seconds_since_last_update);
    }

    joints_[LEFT]->SetParam(
      "vel", 0, wheel_speed_instr_[LEFT] / (wheel_diameter_ / 2.0));
    joints_[RIGHT]->SetParam(
      "vel", 0, wheel_speed_instr_[RIGHT] / (wheel_diameter_ / 2.0));
    joints_[BACK]->SetParam(
      "vel", 0, wheel_speed_instr_[BACK] / (wheel_diameter_ / 2.0));
  }

  last_update_time_ = _info.simTime;
}

// OK
void GazeboRosOmni3wdDrivePrivate::UpdateWheelVelocities()
{

  std::lock_guard<std::mutex> scoped_lock(lock_);
  double vx = target_x_;
  double vy = target_y_;
  double va = target_rot_;

  // Matrix
  //  _       _     _                         _  _    _
  // | V_left  |   |-L     cos(30)    -cos(60) ||  w   |
  // |         |   |                           ||      |
  // | V_right | = |-L     -cos(30)   -cos(60) ||  vx  |
  // |         |   |                           ||      |
  // |_V_back _|   |-L     0          1       _||_ vy _|

  // V_left  =   vx * cos(30°) - vy * cos(60°) - L * w =
  //         =   vx * sqrt(3) / 2 l vy * 1/2
  // V_right = - vx * cos(30°) - vy * cos(60°) - L * w=
  //         = - vx * sqrt(3) / 2 + vy * 1/2
  // V_back  = + vy - L * w
  // being L the distance of the wheels from the centre and w the target_rot_

  desired_wheel_speed_[LEFT]  =   vx * sqrt(3) / 2 - vy * 1/2 - wheel_separation_ * va; 
  desired_wheel_speed_[RIGHT] = - vx * sqrt(3) / 2 - vy * 1/2 - wheel_separation_ * va;
  desired_wheel_speed_[BACK]  =                    + vy       - wheel_separation_ * va;
}

// OK
void GazeboRosOmni3wdDrivePrivate::OnCmdVel(const geometry_msgs::msg::Twist::SharedPtr _msg)
{
  std::lock_guard<std::mutex> scoped_lock(lock_);
  target_x_ = _msg->linear.x;
  target_y_ = _msg->linear.y;
  target_rot_ = _msg->angular.z;
}


void GazeboRosOmni3wdDrivePrivate::UpdateOdometryEncoder(const gazebo::common::Time & _current_time)
{
  // joints velocities (rad/s)
  double vl = joints_[LEFT]->GetVelocity(0);
  double vr = joints_[RIGHT]->GetVelocity(0);
  double vb = joints_[BACK]->GetVelocity(0);

  //inverse Matrix
  // _    _         _                           _  _          _
  // | w  |        |  -1/L        -1/L      -1/L ||  V_left   |
  // |    |        |                             ||           |
  // | vx | = 1/3  |  sqrt(3)     -sqrt(3)   0   ||  V_right  |
  // |    |        |                             ||           |
  // |_Vy_|        |_ -1          -1         2  _||_ V_back  _|
  //
  // Note that vl, vr, and vb are angular velocities, while 
  // V_left etc.. are intended as linear (v*r)

  double seconds_since_last_update = (_current_time - last_encoder_update_).Double();
  last_encoder_update_ = _current_time;

  double b = wheel_separation_; // is the L of the matrix
  double r = wheel_diameter_ / 2.0;

  // theta is global
  double dtheta = (1.0/3.0) * r * (-1.0/b) * (vl +vr +vb) * seconds_since_last_update;

  // dx_l and dy_l in local reference frame
  double dx_l = (1.0/3.0) * r * (sqrt(3.0) * vl - sqrt(3.0) * vr) * seconds_since_last_update;
  double dy_l = (1.0/3.0) * r * (-vl -vr + 2.0*vb) * seconds_since_last_update;

  // dx and dy in global reference frame (rotation)
  double approx_theta = pose_encoder_.theta + dtheta/2.0; 
  double dx =   dx_l * cos(approx_theta) + dy_l * sin(approx_theta);
  double dy = - dx_l * sin(approx_theta) + dy_l * cos(approx_theta);

  pose_encoder_.x += dx;
  pose_encoder_.y += dy;
  pose_encoder_.theta += dtheta;

  double w = dtheta / seconds_since_last_update;
  double v = sqrt(dx * dx + dy * dy) / seconds_since_last_update;

  tf2::Quaternion qt;
  tf2::Vector3 vt;
  qt.setRPY(0, 0, pose_encoder_.theta);
  vt = tf2::Vector3(pose_encoder_.x, pose_encoder_.y, 0);

  odom_.pose.pose.position.x = vt.x();
  odom_.pose.pose.position.y = vt.y();
  odom_.pose.pose.position.z = vt.z();

  odom_.pose.pose.orientation.x = qt.x();
  odom_.pose.pose.orientation.y = qt.y();
  odom_.pose.pose.orientation.z = qt.z();
  odom_.pose.pose.orientation.w = qt.w();

  odom_.twist.twist.angular.z = w;
  odom_.twist.twist.linear.x = v;
  odom_.twist.twist.linear.y = 0;
}


void GazeboRosOmni3wdDrivePrivate::UpdateOdometryWorld()
{
  auto pose = model_->WorldPose();
  odom_.pose.pose.position = gazebo_ros::Convert<geometry_msgs::msg::Point>(pose.Pos());
  odom_.pose.pose.orientation = gazebo_ros::Convert<geometry_msgs::msg::Quaternion>(pose.Rot());

  // Get velocity in odom frame
  auto linear = model_->WorldLinearVel();
  odom_.twist.twist.angular.z = model_->WorldAngularVel().Z();

  // Convert velocity to child_frame_id(aka base_footprint)
  float yaw = pose.Rot().Yaw();
  odom_.twist.twist.linear.x = cosf(yaw) * linear.X() + sinf(yaw) * linear.Y();
  odom_.twist.twist.linear.y = cosf(yaw) * linear.Y() - sinf(yaw) * linear.X();
}

// NOT MODIFIED YET
void GazeboRosOmni3wdDrivePrivate::PublishOdometryTf(const gazebo::common::Time & _current_time)
{
  geometry_msgs::msg::TransformStamped msg;
  msg.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(_current_time);
  msg.header.frame_id = odometry_frame_;
  msg.child_frame_id = robot_base_frame_;
  msg.transform.translation =
    gazebo_ros::Convert<geometry_msgs::msg::Vector3>(odom_.pose.pose.position);
  msg.transform.rotation = odom_.pose.pose.orientation;

  transform_broadcaster_->sendTransform(msg);
}

// NOT MODIFIED YET
void GazeboRosOmni3wdDrivePrivate::PublishWheelsTf(const gazebo::common::Time & _current_time)
{
  for (unsigned int i = 0; i < 3; ++i) { // 3 wheels
    auto pose_wheel = joints_[i]->GetChild()->RelativePose();

    geometry_msgs::msg::TransformStamped msg;
    msg.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(_current_time);
    msg.header.frame_id = joints_[i]->GetParent()->GetName();
    msg.child_frame_id = joints_[i]->GetChild()->GetName();
    msg.transform.translation = gazebo_ros::Convert<geometry_msgs::msg::Vector3>(pose_wheel.Pos());
    msg.transform.rotation = gazebo_ros::Convert<geometry_msgs::msg::Quaternion>(pose_wheel.Rot());

    transform_broadcaster_->sendTransform(msg);
  }
}

// NOT MODIFIED YET
void GazeboRosOmni3wdDrivePrivate::PublishOdometryMsg(const gazebo::common::Time & _current_time)
{
  // Set covariance
  odom_.pose.covariance[0] = covariance_[0];
  odom_.pose.covariance[7] = covariance_[1];
  odom_.pose.covariance[14] = 1000000000000.0;
  odom_.pose.covariance[21] = 1000000000000.0;
  odom_.pose.covariance[28] = 1000000000000.0;
  odom_.pose.covariance[35] = covariance_[2];

  odom_.twist.covariance[0] = covariance_[0];
  odom_.twist.covariance[7] = covariance_[1];
  odom_.twist.covariance[14] = 1000000000000.0;
  odom_.twist.covariance[21] = 1000000000000.0;
  odom_.twist.covariance[28] = 1000000000000.0;
  odom_.twist.covariance[35] = covariance_[2];

  // Set header
  odom_.header.frame_id = odometry_frame_;
  odom_.child_frame_id = robot_base_frame_;
  odom_.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(_current_time);

  // Publish
  odometry_pub_->publish(odom_);
}
GZ_REGISTER_MODEL_PLUGIN(GazeboRosOmni3wdDrive)
}  // namespace gazebo_plugins
