/*
    Copyright (c) 2010, Daniel Hewlett, Antons Rebguns
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
        * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
        * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
        * Neither the name of the <organization> nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY Antons Rebguns <email> ''AS IS'' AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL Antons Rebguns <email> BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

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

//#include <algorithm>
//#include <assert.h>

#include <gazebo/common/Time.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo_plugins/gazebo_ros_diff_drive.hpp>
#include <gazebo_ros/node.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sdf/sdf.hh>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

//#include <ignition/math/Angle.hh>
//#include <ignition/math/Pose3.hh>
//#include <ignition/math/Quaternion.hh>
//#include <ignition/math/Vector3.hh>
//#include <builtin_interfaces/msg/time.hpp>
//#include <gazebo_ros/conversions/builtin_interfaces.hpp>

namespace gazebo_plugins
{
class GazeboRosDiffDrivePrivate
{
public:
  ///
  enum OdomSource
  {
    ENCODER = 0,
    WORLD = 1,
  };

  /// A pointer to the GazeboROS node.
  gazebo_ros::Node::SharedPtr ros_node_;
//  void publishOdometry(double step_time);
  void GetWheelVelocities();
//  void publishWheelTF(); /// publishes the wheel tf's
  void UpdateOdometryEncoder();
//  physics::ModelPtr parent;
  gazebo::event::ConnectionPtr update_connection_;
  double wheel_separation_;
  double wheel_diameter_;
  double wheel_torque_;
  double wheel_speed_[2];
  double wheel_accel_;
  double wheel_speed_instr_[2];
  std::vector<gazebo::physics::JointPtr> joints_;
//  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;
//  std::shared_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;
  nav_msgs::msg::Odometry odom_;
  std::mutex lock_;
  std::string robot_namespace_;
  std::string odometry_topic_;
  std::string odometry_frame_;
  std::string robot_base_frame_;
  bool publish_tf_;
  double x_;
  double rot_;
  double update_period_;
  gazebo::common::Time last_update_time_;
  OdomSource odom_source_;
  geometry_msgs::msg::Pose2D pose_encoder_;
  gazebo::common::Time last_odom_update_;
  bool publishWheelTF_;
  bool publishOdomTF_;
};

enum {
  RIGHT,
  LEFT,
};

GazeboRosDiffDrive::GazeboRosDiffDrive()
: impl_(std::make_unique<GazeboRosDiffDrivePrivate>())
{
}

GazeboRosDiffDrive::~GazeboRosDiffDrive()
{
}

void GazeboRosDiffDrive::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
//  impl_->parent = _model;
  impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);

  impl_->ros_node_->get_parameter_or<std::string>("odometryTopic", impl_->odometry_topic_, "odom");
  impl_->ros_node_->get_parameter_or<std::string>("odometryFrame", impl_->odometry_frame_, "odom");
  impl_->ros_node_->get_parameter_or<std::string>("robotBaseFrame", impl_->robot_base_frame_,
    "base_footprint");
  impl_->ros_node_->get_parameter_or<bool>("publishWheelTF", impl_->publishWheelTF_, false);
  impl_->ros_node_->get_parameter_or<bool>("publishOdomTF", impl_->publishOdomTF_, true);
  impl_->ros_node_->get_parameter_or<double>("wheelSeparation", impl_->wheel_separation_, 0.34);
  impl_->ros_node_->get_parameter_or<double>("wheelDiameter", impl_->wheel_diameter_, 0.15);
  impl_->ros_node_->get_parameter_or<double>("wheelAcceleration", impl_->wheel_accel_, 0.0);
  impl_->ros_node_->get_parameter_or<double>("wheelTorque", impl_->wheel_torque_, 5.0);
  std::map<std::string, GazeboRosDiffDrivePrivate::OdomSource> odomOptions;
  odomOptions["encoder"] = GazeboRosDiffDrivePrivate::OdomSource::ENCODER;
  odomOptions["world"] = GazeboRosDiffDrivePrivate::OdomSource::WORLD;
  // TODO(tfoote) restore parameter
  // impl_->ros_node_->get_parameter_or<GazeboRosDiffDrivePrivate::OdomSource>("odometrySource", impl_->odomOptions, WORLD);

  impl_->joints_.resize(2);
  impl_->joints_[LEFT] = _model->GetJoint("leftJoint"); // TODO(tfoote) or underscored left_joint
  impl_->joints_[RIGHT] = _model->GetJoint("rightJoint");
  impl_->joints_[LEFT]->SetParam("fmax", 0, impl_->wheel_torque_);
  impl_->joints_[RIGHT]->SetParam("fmax", 0, impl_->wheel_torque_);

  impl_->publish_tf_ = true;
  if (!_sdf->HasElement("publishTf")) {
    RCLCPP_WARN(impl_->ros_node_->get_logger(),
      "diff_drive", "GazeboRosDiffDrive Plugin (ns = %s) missing <publishTf>, defaults to %d",
      impl_->robot_namespace_.c_str(), impl_->publish_tf_);
  } else {
    impl_->publish_tf_ = _sdf->GetElement("publishTf")->Get<bool>();
  }

  // Initialize update rate stuff
  double update_rate;
  impl_->ros_node_->get_parameter_or<double>("updateRate", update_rate, 100.0);
  if (update_rate > 0.0) {
     impl_->update_period_ = 1.0 / update_rate;
  } else {
    impl_->update_period_ = 0.0;
  }
  impl_->last_update_time_ = _model->GetWorld()->SimTime();

  // Initialize velocity stuff
  impl_->wheel_speed_[RIGHT] = 0;
  impl_->wheel_speed_[LEFT] = 0;

  // Initialize velocity support stuff
  impl_->wheel_speed_instr_[RIGHT] = 0;
  impl_->wheel_speed_instr_[LEFT] = 0;

  impl_->x_ = 0;
  impl_->rot_ = 0;

//  transform_broadcaster_ = std::shared_ptr<tf2_ros::TransformBroadcaster>(new tf2_ros::TransformBroadcaster(impl_->ros_node_));

  // ROS: Subscribe to the velocity command topic (usually "cmd_vel")
  std::string command_topic;
  impl_->ros_node_->get_parameter_or<std::string>("commandTopic", command_topic, "cmd_vel");
  RCLCPP_INFO(impl_->ros_node_->get_logger(),
      "diff_drive", "%s: Try to subscribe to %s", impl_->ros_node_->get_name(),
      command_topic.c_str());

  // TODO(tfoote) equivalent qos settings as below
  // ros::SubscribeOptions so =
  //     ros::SubscribeOptions::create<geometry_msgs::Twist>(command_topic_, 1,
  //             boost::bind(&GazeboRosDiffDrive::cmdVelCallback, this, _1),
  //             ros::VoidPtr(), &queue_);

  impl_->cmd_vel_subscriber_ =
    impl_->ros_node_->create_subscription<geometry_msgs::msg::Twist::SharedPtr>(
    command_topic, std::bind(&GazeboRosDiffDrive::CmdVelCallback, this,
    std::placeholders::_1));
  RCLCPP_INFO(impl_->ros_node_->get_logger(),
      "diff_drive", "%s: Subscribe to %s", impl_->ros_node_->get_name(), command_topic.c_str());

//  if (impl_->publish_tf_)
//  {
//      //TODO(tfoote)mimic qos publisher queue size 1
//      odometry_publisher_ = impl_->ros_node_->create_publisher<nav_msgs::msg::Odometry>(odometry_topic_);
//
//  RCLCPP_INFO(impl_->ros_node_->get_logger(),
//      "diff_drive", "%s: Advertise odom on %s ", impl_->ros_node_->get_name(), odometry_topic_.c_str());
//  }

  // Listen to the update event (broadcast every simulation iteration)
  impl_->update_connection_ =
      gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&GazeboRosDiffDrive::OnUpdate, this));
}

void GazeboRosDiffDrive::Reset()
{
  impl_->last_update_time_ = impl_->joints_[LEFT]->GetWorld()->SimTime();
  impl_->pose_encoder_.x = 0;
  impl_->pose_encoder_.y = 0;
  impl_->pose_encoder_.theta = 0;
  impl_->x_ = 0;
  impl_->rot_ = 0;
  impl_->joints_[LEFT]->SetParam("fmax", 0, impl_->wheel_torque_);
  impl_->joints_[RIGHT]->SetParam("fmax", 0, impl_->wheel_torque_);
}

//void GazeboRosDiffDrive::publishWheelTF()
//{
//    rclcpp::Time current_time = impl_->ros_node_->now();
//    for (int i = 0; i < 2; i++) {
//
//        std::string wheel_frame = impl_->joints_[i]->GetChild()->GetName();
//        std::string wheel_parent_frame = impl_->joints_[i]->GetParent()->GetName();
//
//        ignition::math::Pose3d poseWheel = impl_->joints_[i]->GetChild()->RelativePose();
//
//        geometry_msgs::msg::TransformStamped msg;
//        msg.header.stamp = current_time;
//        msg.header.frame_id = wheel_parent_frame;
//        msg.child_frame_id = wheel_frame;
//        msg.transform.translation.x = poseWheel.Pos().X();
//        msg.transform.translation.y = poseWheel.Pos().Y();
//        msg.transform.translation.z = poseWheel.Pos().Z();
//        msg.transform.rotation.x = poseWheel.Rot().X();
//        msg.transform.rotation.y = poseWheel.Rot().Y();
//        msg.transform.rotation.z = poseWheel.Rot().Z();
//        msg.transform.rotation.w = poseWheel.Rot().W();
//        transform_broadcaster_->sendTransform(msg);
//    }
//}

void GazeboRosDiffDrive::OnUpdate()
{
  // \todo(louise) Check if this has been fixed
  /* force reset SetParam("fmax") since Joint::Reset reset MaxForce to zero at
     https://bitbucket.org/osrf/gazebo/src/8091da8b3c529a362f39b042095e12c94656a5d1/gazebo/physics/Joint.cc?at=gazebo2_2.2.5#cl-331
     (this has been solved in https://bitbucket.org/osrf/gazebo/diff/gazebo/physics/Joint.cc?diff2=b64ff1b7b6ff&at=issue_964)
     and Joint::Reset is called after ModelPlugin::Reset, so we need to set maxForce to wheel_torque other than GazeboRosDiffDrive::Reset
     (this seems to be solved in https://bitbucket.org/osrf/gazebo/commits/ec8801d8683160eccae22c74bf865d59fac81f1e)
  */
  for (int i = 0; i < 2; ++i) {
    if (fabs(impl_->wheel_torque_ - impl_->joints_[i]->GetParam("fmax", 0)) > 1e-6) {
      impl_->joints_[i]->SetParam("fmax", 0, impl_->wheel_torque_);
    }
  }

  if (impl_->odom_source_ == GazeboRosDiffDrivePrivate::OdomSource::ENCODER) {
    impl_->UpdateOdometryEncoder();
  }

  auto current_time = impl_->joints_[LEFT]->GetWorld()->SimTime();
  double seconds_since_last_update = (current_time - impl_->last_update_time_).Double();

  if (seconds_since_last_update > impl_->update_period_) {

//        if (impl_->publish_tf_) publishOdometry(seconds_since_last_update);
//        if (publishWheelTF_) publishWheelTF();

    // Update robot in case new velocities have been requested
    impl_->GetWheelVelocities();

    double current_speed[2];

    current_speed[LEFT] = impl_->joints_[LEFT]->GetVelocity(0)  * (impl_->wheel_diameter_ / 2.0);
    current_speed[RIGHT] = impl_->joints_[RIGHT]->GetVelocity(0) * (impl_->wheel_diameter_ / 2.0);

    // If max_accel == 0, or target speed is reached
    if (impl_->wheel_accel_ == 0 ||
         (fabs(impl_->wheel_speed_[LEFT] - current_speed[LEFT]) < 0.01) ||
         (fabs(impl_->wheel_speed_[RIGHT] - current_speed[RIGHT]) < 0.01)) {
      impl_->joints_[LEFT]->SetParam("vel", 0,
        impl_->wheel_speed_[LEFT]/(impl_->wheel_diameter_ / 2.0));
      impl_->joints_[RIGHT]->SetParam("vel", 0,
        impl_->wheel_speed_[RIGHT]/(impl_->wheel_diameter_ / 2.0));
    } else {
      if (impl_->wheel_speed_[LEFT]>=current_speed[LEFT]) {
        impl_->wheel_speed_instr_[LEFT] += fmin(impl_->wheel_speed_[LEFT]-current_speed[LEFT],
          impl_->wheel_accel_ * seconds_since_last_update);
      } else {
        impl_->wheel_speed_instr_[LEFT] += fmax(impl_->wheel_speed_[LEFT]-current_speed[LEFT],
          -impl_->wheel_accel_ * seconds_since_last_update);
      }

      if (impl_->wheel_speed_[RIGHT]>current_speed[RIGHT]) {
        impl_->wheel_speed_instr_[RIGHT] += fmin(impl_->wheel_speed_[RIGHT]-current_speed[RIGHT],
          impl_->wheel_accel_ * seconds_since_last_update);
      } else {
        impl_->wheel_speed_instr_[RIGHT] += fmax(impl_->wheel_speed_[RIGHT]-current_speed[RIGHT],
          -impl_->wheel_accel_ * seconds_since_last_update);
      }

      // ROS_INFO_NAMED("diff_drive", "actual wheel speed = %lf, issued wheel speed= %lf", current_speed[LEFT], impl_->wheel_speed_[LEFT]);
      // ROS_INFO_NAMED("diff_drive", "actual wheel speed = %lf, issued wheel speed= %lf", current_speed[RIGHT],impl_->wheel_speed_[RIGHT]);

      impl_->joints_[LEFT]->SetParam("vel", 0,
        impl_->wheel_speed_instr_[LEFT] / (impl_->wheel_diameter_ / 2.0));
      impl_->joints_[RIGHT]->SetParam("vel", 0,
        impl_->wheel_speed_instr_[RIGHT] / (impl_->wheel_diameter_ / 2.0));
    }

    impl_->last_update_time_ += gazebo::common::Time(impl_->update_period_);
  }
}

void GazeboRosDiffDrivePrivate::GetWheelVelocities()
{
  std::lock_guard<std::mutex> scoped_lock(lock_);

  double vr = x_;
  double va = rot_;

  wheel_speed_[LEFT] = vr - va * wheel_separation_ / 2.0;
  wheel_speed_[RIGHT] = vr + va * wheel_separation_ / 2.0;
}

void GazeboRosDiffDrive::CmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr _msg)
{
  std::lock_guard<std::mutex> scoped_lock(impl_->lock_);
  impl_->x_ = _msg->linear.x;
  impl_->rot_ = _msg->angular.z;
}

void GazeboRosDiffDrivePrivate::UpdateOdometryEncoder()
{
  double vl = joints_[LEFT]->GetVelocity(0);
  double vr = joints_[RIGHT]->GetVelocity(0);

  auto current_time = joints_[LEFT]->GetWorld()->SimTime();
  double seconds_since_last_update = (current_time - last_odom_update_).Double();
  last_odom_update_ = current_time;

  double b = wheel_separation_;

  // Book: Sigwart 2011 Autonompus Mobile Robots page:337
  double sl = vl * (wheel_diameter_ / 2.0) * seconds_since_last_update;
  double sr = vr * (wheel_diameter_ / 2.0) * seconds_since_last_update;
  double ssum = sl + sr;

  double sdiff = sr - sl;

  double dx = (ssum) /2.0 * cos(pose_encoder_.theta + (sdiff) / (2.0*b));
  double dy = (ssum) /2.0 * sin(pose_encoder_.theta + (sdiff) / (2.0*b));
  double dtheta = (sdiff) /b;

  pose_encoder_.x += dx;
  pose_encoder_.y += dy;
  pose_encoder_.theta += dtheta;

  double w = dtheta/seconds_since_last_update;
  double v = sqrt(dx*dx+dy*dy) /seconds_since_last_update;

  tf2::Quaternion qt;
  tf2::Vector3 vt;
  qt.setRPY(0,0,pose_encoder_.theta);
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

//void GazeboRosDiffDrive::publishOdometry(double step_time)
//{
//
//    rclcpp::Time current_time = impl_->ros_node_->now();
//
//    tf2::Quaternion qt;
//    tf2::Vector3 vt;
//
//    if (impl_->odom_source_ == GazeboRosDiffDrivePrivate::OdomSource::ENCODER) {
//        // getting data form encoder integration
//        qt = tf2::Quaternion(odom_.pose.pose.orientation.x, odom_.pose.pose.orientation.y, odom_.pose.pose.orientation.z, odom_.pose.pose.orientation.w);
//        vt = tf2::Vector3(odom_.pose.pose.position.x, odom_.pose.pose.position.y, odom_.pose.pose.position.z);
//
//    }
//    if (impl_->odom_source_ == WORLD) {
//        // getting data from gazebo world
//        ignition::math::Pose3d pose = parent->WorldPose();
//        qt = tf2::Quaternion(pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z(), pose.Rot().W());
//        vt = tf2::Vector3(pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z());
//
//        odom_.pose.pose.position.x = vt.x();
//        odom_.pose.pose.position.y = vt.y();
//        odom_.pose.pose.position.z = vt.z();
//
//        odom_.pose.pose.orientation.x = qt.x();
//        odom_.pose.pose.orientation.y = qt.y();
//        odom_.pose.pose.orientation.z = qt.z();
//        odom_.pose.pose.orientation.w = qt.w();
//
//        // get velocity in /odom frame
//        ignition::math::Vector3d linear;
//        linear = parent->WorldLinearVel();
//        odom_.twist.twist.angular.z = parent->WorldAngularVel().Z();
//
//        // convert velocity to child_frame_id(aka base_footprint)
//        float yaw = pose.Rot().Yaw();
//        odom_.twist.twist.linear.x = cosf(yaw) * linear.X() + sinf(yaw) * linear.Y();
//        odom_.twist.twist.linear.y = cosf(yaw) * linear.Y() - sinf(yaw) * linear.X();
//    }
//
//    if (publishOdomTF_ == true){
//
//        geometry_msgs::msg::TransformStamped msg;
//        msg.header.stamp = current_time;
//        msg.header.frame_id = odometry_frame_;
//        msg.child_frame_id = robot_base_frame_;
//        msg.transform.translation.x = vt.x();
//        msg.transform.translation.y = vt.y();
//        msg.transform.translation.z = vt.z();
//        msg.transform.rotation.x = qt.x();
//        msg.transform.rotation.y = qt.y();
//        msg.transform.rotation.z = qt.z();
//        msg.transform.rotation.w = qt.w();
//
//        transform_broadcaster_->sendTransform(msg);
//    }
//
//
//    // set covariance
//    odom_.pose.covariance[0] = 0.00001;
//    odom_.pose.covariance[7] = 0.00001;
//    odom_.pose.covariance[14] = 1000000000000.0;
//    odom_.pose.covariance[21] = 1000000000000.0;
//    odom_.pose.covariance[28] = 1000000000000.0;
//    odom_.pose.covariance[35] = 0.001;
//
//
//    // set header
//    odom_.header.stamp = current_time;
//    odom_.header.frame_id = odometry_frame_;
//    odom_.child_frame_id = robot_base_frame_;
//
//    odometry_publisher_->publish(odom_);
//}

GZ_REGISTER_MODEL_PLUGIN(GazeboRosDiffDrive)
}
