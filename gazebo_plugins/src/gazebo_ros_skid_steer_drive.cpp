/*
 * Copyright (c) 2010, Daniel Hewlett, Antons Rebguns
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *      * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *      * Neither the name of the <organization> nor the
 *      names of its contributors may be used to endorse or promote products
 *      derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY Antons Rebguns <email> ''AS IS'' AND ANY
 *  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL Antons Rebguns <email> BE LIABLE FOR ANY
 *  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 **/

/*
 * \file  gazebo_ros_skid_steer_drive.cpp
 *
 * \brief A skid steering drive plugin. Inspired by gazebo_ros_diff_drive and SkidSteerDrivePlugin
 *
 * \author  Zdenek Materna (imaterna@fit.vutbr.cz)
 *
 * $ Id: 06/25/2013 11:23:40 AM materna $
 */


#include <algorithm>
#include <assert.h>

#include <gazebo_plugins/gazebo_ros_skid_steer_drive.h>

#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <sdf/sdf.hh>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/Odometry.h>
#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>

namespace gazebo {

  enum {
    RIGHT_FRONT=0,
    LEFT_FRONT=1,
    RIGHT_REAR=2,
    LEFT_REAR=3,
  };

  GazeboRosSkidSteerDrive::GazeboRosSkidSteerDrive() {
    this->seed = 0;
    x_error = 0;
    y_error = 0;
    yaw_error = 0;
  }

  // Destructor
  GazeboRosSkidSteerDrive::~GazeboRosSkidSteerDrive() {
    delete rosnode_;
    delete transform_broadcaster_;
  }

  // Load the controller
  void GazeboRosSkidSteerDrive::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
    this->parent = _parent;
    this->world = _parent->GetWorld();

    this->robot_namespace_ = "";
    if (!_sdf->HasElement("robotNamespace")) {
      ROS_INFO_NAMED("skid_steer_drive", "GazeboRosSkidSteerDrive Plugin missing <robotNamespace>, defaults to \"%s\"",
          this->robot_namespace_.c_str());
    } else {
      this->robot_namespace_ =
        _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";
    }

    this->broadcast_tf_ = false;
    if (!_sdf->HasElement("broadcastTF")) {
      if (!this->broadcast_tf_)
    	  ROS_INFO_NAMED("skid_steer_drive", "GazeboRosSkidSteerDrive Plugin (ns = %s) missing <broadcastTF>, defaults to false.",this->robot_namespace_.c_str());
      else ROS_INFO_NAMED("skid_steer_drive", "GazeboRosSkidSteerDrive Plugin (ns = %s) missing <broadcastTF>, defaults to true.",this->robot_namespace_.c_str());

    } else {
      this->broadcast_tf_ = _sdf->GetElement("broadcastTF")->Get<bool>();
    }

    // TODO write error if joint doesn't exist!
    this->left_front_joint_name_ = "left_front_joint";
    if (!_sdf->HasElement("leftFrontJoint")) {
      ROS_WARN_NAMED("skid_steer_drive", "GazeboRosSkidSteerDrive Plugin (ns = %s) missing <leftFrontJoint>, defaults to \"%s\"",
          this->robot_namespace_.c_str(), this->left_front_joint_name_.c_str());
    } else {
      this->left_front_joint_name_ = _sdf->GetElement("leftFrontJoint")->Get<std::string>();
    }

    this->right_front_joint_name_ = "right_front_joint";
        if (!_sdf->HasElement("rightFrontJoint")) {
          ROS_WARN_NAMED("skid_steer_drive", "GazeboRosSkidSteerDrive Plugin (ns = %s) missing <rightFrontJoint>, defaults to \"%s\"",
              this->robot_namespace_.c_str(), this->right_front_joint_name_.c_str());
        } else {
          this->right_front_joint_name_ = _sdf->GetElement("rightFrontJoint")->Get<std::string>();
        }

  	this->left_rear_joint_name_ = "left_rear_joint";
  	if (!_sdf->HasElement("leftRearJoint")) {
  	  ROS_WARN_NAMED("skid_steer_drive", "GazeboRosSkidSteerDrive Plugin (ns = %s) missing <leftRearJoint>, defaults to \"%s\"",
  		  this->robot_namespace_.c_str(), this->left_rear_joint_name_.c_str());
  	} else {
  	  this->left_rear_joint_name_ = _sdf->GetElement("leftRearJoint")->Get<std::string>();
  	}

    this->right_rear_joint_name_ = "right_rear_joint";
    if (!_sdf->HasElement("rightRearJoint")) {
      ROS_WARN_NAMED("skid_steer_drive", "GazeboRosSkidSteerDrive Plugin (ns = %s) missing <rightRearJoint>, defaults to \"%s\"",
          this->robot_namespace_.c_str(), this->right_rear_joint_name_.c_str());
    } else {
      this->right_rear_joint_name_ = _sdf->GetElement("rightRearJoint")->Get<std::string>();
    }


    // This assumes that front and rear wheel spacing is identical
    /*this->wheel_separation_ = this->parent->GetJoint(left_front_joint_name_)->GetAnchor(0).Distance(
    		this->parent->GetJoint(right_front_joint_name_)->GetAnchor(0));*/

    this->wheel_separation_ = 0.4;

    if (!_sdf->HasElement("wheelSeparation")) {
      ROS_WARN_NAMED("skid_steer_drive", "GazeboRosSkidSteerDrive Plugin (ns = %s) missing <wheelSeparation>, defaults to value from robot_description: %f",
          this->robot_namespace_.c_str(), this->wheel_separation_);
    } else {
      this->wheel_separation_ =
        _sdf->GetElement("wheelSeparation")->Get<double>();
    }

    // TODO get this from robot_description
    this->wheel_diameter_ = 0.15;
    if (!_sdf->HasElement("wheelDiameter")) {
      ROS_WARN_NAMED("skid_steer_drive", "GazeboRosSkidSteerDrive Plugin (ns = %s) missing <wheelDiameter>, defaults to %f",
          this->robot_namespace_.c_str(), this->wheel_diameter_);
    } else {
      this->wheel_diameter_ = _sdf->GetElement("wheelDiameter")->Get<double>();
    }

    this->torque = 5.0;
    if (!_sdf->HasElement("torque")) {
      ROS_WARN_NAMED("skid_steer_drive", "GazeboRosSkidSteerDrive Plugin (ns = %s) missing <torque>, defaults to %f",
          this->robot_namespace_.c_str(), this->torque);
    } else {
      this->torque = _sdf->GetElement("torque")->Get<double>();
    }

    this->command_topic_ = "cmd_vel";
    if (!_sdf->HasElement("commandTopic")) {
      ROS_WARN_NAMED("skid_steer_drive", "GazeboRosSkidSteerDrive Plugin (ns = %s) missing <commandTopic>, defaults to \"%s\"",
          this->robot_namespace_.c_str(), this->command_topic_.c_str());
    } else {
      this->command_topic_ = _sdf->GetElement("commandTopic")->Get<std::string>();
    }

    this->odometry_topic_ = "odom";
    if (!_sdf->HasElement("odometryTopic")) {
      ROS_WARN_NAMED("skid_steer_drive", "GazeboRosSkidSteerDrive Plugin (ns = %s) missing <odometryTopic>, defaults to \"%s\"",
          this->robot_namespace_.c_str(), this->odometry_topic_.c_str());
    } else {
      this->odometry_topic_ = _sdf->GetElement("odometryTopic")->Get<std::string>();
    }

    this->odometry_frame_ = "odom";
    if (!_sdf->HasElement("odometryFrame")) {
      ROS_WARN_NAMED("skid_steer_drive", "GazeboRosSkidSteerDrive Plugin (ns = %s) missing <odometryFrame>, defaults to \"%s\"",
          this->robot_namespace_.c_str(), this->odometry_frame_.c_str());
    } else {
      this->odometry_frame_ = _sdf->GetElement("odometryFrame")->Get<std::string>();
    }

    this->ground_truth_topic_ = "gt";
    if (!_sdf->HasElement("groundTruthTopic")) {
      ROS_WARN_NAMED("skid_steer_drive", "GazeboRosSkidSteerDrive Plugin (ns = %s) missing <groundTruthTopic>, defaults to \"%s\"",
          this->robot_namespace_.c_str(), this->ground_truth_topic_.c_str());
    } else {
      this->ground_truth_topic_ = _sdf->GetElement("groundTruthTopic")->Get<std::string>();
    }

    this->ground_truth_frame_ = "gt";
    if (!_sdf->HasElement("groundTruthFrame")) {
      ROS_WARN_NAMED("skid_steer_drive", "GazeboRosSkidSteerDrive Plugin (ns = %s) missing <groundTruthFrame>, defaults to \"%s\"",
          this->robot_namespace_.c_str(), this->ground_truth_frame_.c_str());
    } else {
      this->ground_truth_frame_ = _sdf->GetElement("groundTruthFrame")->Get<std::string>();
    }

    this->robot_base_frame_ = "base_footprint";
    if (!_sdf->HasElement("robotBaseFrame")) {
      ROS_WARN_NAMED("skid_steer_drive", "GazeboRosSkidSteerDrive Plugin (ns = %s) missing <robotBaseFrame>, defaults to \"%s\"",
          this->robot_namespace_.c_str(), this->robot_base_frame_.c_str());
    } else {
      this->robot_base_frame_ = _sdf->GetElement("robotBaseFrame")->Get<std::string>();
    }

    this->update_rate_ = 100.0;
    if (!_sdf->HasElement("updateRate")) {
      ROS_WARN_NAMED("skid_steer_drive", "GazeboRosSkidSteerDrive Plugin (ns = %s) missing <updateRate>, defaults to %f",
          this->robot_namespace_.c_str(), this->update_rate_);
    } else {
      this->update_rate_ = _sdf->GetElement("updateRate")->Get<double>();
    }

    this->covariance_x_ = 0.0001;
    if (!_sdf->HasElement("covariance_x")) {
      ROS_WARN_NAMED("skid_steer_drive", "GazeboRosSkidSteerDrive Plugin (ns = %s) missing <covariance_x>, defaults to %f",
          this->robot_namespace_.c_str(), covariance_x_);
    } else {
      covariance_x_ = _sdf->GetElement("covariance_x")->Get<double>();
    }

    this->covariance_y_ = 0.0001;
    if (!_sdf->HasElement("covariance_y")) {
      ROS_WARN_NAMED("skid_steer_drive", "GazeboRosSkidSteerDrive Plugin (ns = %s) missing <covariance_y>, defaults to %f",
          this->robot_namespace_.c_str(), covariance_y_);
    } else {
      covariance_y_ = _sdf->GetElement("covariance_y")->Get<double>();
    }

    this->covariance_yaw_ = 0.01;
    if (!_sdf->HasElement("covariance_yaw")) {
      ROS_WARN_NAMED("skid_steer_drive", "GazeboRosSkidSteerDrive Plugin (ns = %s) missing <covariance_yaw>, defaults to %f",
          this->robot_namespace_.c_str(), covariance_yaw_);
    } else {
      covariance_yaw_ = _sdf->GetElement("covariance_yaw")->Get<double>();
    }

    // Initialize update rate stuff
    if (this->update_rate_ > 0.0) {
      this->update_period_ = 1.0 / this->update_rate_;
    } else {
      this->update_period_ = 0.0;
    }

    // gaussian noise
    if (!_sdf->HasElement("gaussianNoise"))
    {
      ROS_DEBUG_NAMED("skid_steer_drive", "GazeboRosSkidSteerDrive Plugin (ns = %s) <gaussianNoise>, defaults to 0.0");
      this->gaussian_noise_ = 0;
    }
    else
      this->gaussian_noise_ = _sdf->GetElement("gaussianNoise")->Get<double>();

#if GAZEBO_MAJOR_VERSION >= 8
    last_update_time_ = this->world->SimTime();
#else
    last_update_time_ = this->world->GetSimTime();
#endif

    // Initialize velocity stuff
    wheel_speed_[RIGHT_FRONT] = 0;
    wheel_speed_[LEFT_FRONT] = 0;
    wheel_speed_[RIGHT_REAR] = 0;
  	wheel_speed_[LEFT_REAR] = 0;

    x_ = 0;
    rot_ = 0;
    alive_ = true;

    joints[LEFT_FRONT] = this->parent->GetJoint(left_front_joint_name_);
    joints[RIGHT_FRONT] = this->parent->GetJoint(right_front_joint_name_);
    joints[LEFT_REAR] = this->parent->GetJoint(left_rear_joint_name_);
    joints[RIGHT_REAR] = this->parent->GetJoint(right_rear_joint_name_);

    if (!joints[LEFT_FRONT]) {
      char error[200];
      snprintf(error, 200,
          "GazeboRosSkidSteerDrive Plugin (ns = %s) couldn't get left front hinge joint named \"%s\"",
          this->robot_namespace_.c_str(), this->left_front_joint_name_.c_str());
      gzthrow(error);
    }

    if (!joints[RIGHT_FRONT]) {
      char error[200];
      snprintf(error, 200,
          "GazeboRosSkidSteerDrive Plugin (ns = %s) couldn't get right front hinge joint named \"%s\"",
          this->robot_namespace_.c_str(), this->right_front_joint_name_.c_str());
      gzthrow(error);
    }

    if (!joints[LEFT_REAR]) {
	 char error[200];
	 snprintf(error, 200,
		 "GazeboRosSkidSteerDrive Plugin (ns = %s) couldn't get left rear hinge joint named \"%s\"",
		 this->robot_namespace_.c_str(), this->left_rear_joint_name_.c_str());
	 gzthrow(error);
   }

   if (!joints[RIGHT_REAR]) {
	 char error[200];
	 snprintf(error, 200,
		 "GazeboRosSkidSteerDrive Plugin (ns = %s) couldn't get right rear hinge joint named \"%s\"",
		 this->robot_namespace_.c_str(), this->right_rear_joint_name_.c_str());
	 gzthrow(error);
   }

#if GAZEBO_MAJOR_VERSION > 2
    joints[LEFT_FRONT]->SetParam("fmax", 0, torque);
    joints[RIGHT_FRONT]->SetParam("fmax", 0, torque);
    joints[LEFT_REAR]->SetParam("fmax", 0, torque);
    joints[RIGHT_REAR]->SetParam("fmax", 0, torque);
#else
    joints[LEFT_FRONT]->SetMaxForce(0, torque);
    joints[RIGHT_FRONT]->SetMaxForce(0, torque);
    joints[LEFT_REAR]->SetMaxForce(0, torque);
    joints[RIGHT_REAR]->SetMaxForce(0, torque);
#endif

    // Make sure the ROS node for Gazebo has already been initialized
    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM_NAMED("skid_steer_drive", "A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }

    rosnode_ = new ros::NodeHandle(this->robot_namespace_);

    ROS_INFO_NAMED("skid_steer_drive", "Starting GazeboRosSkidSteerDrive Plugin (ns = %s)", this->robot_namespace_.c_str());

    tf_prefix_ = tf::getPrefixParam(*rosnode_);
    transform_broadcaster_ = new tf::TransformBroadcaster();

    // ROS: Subscribe to the velocity command topic (usually "cmd_vel")
    ros::SubscribeOptions so =
      ros::SubscribeOptions::create<geometry_msgs::Twist>(command_topic_, 1,
          boost::bind(&GazeboRosSkidSteerDrive::cmdVelCallback, this, _1),
          ros::VoidPtr(), &queue_);

    cmd_vel_subscriber_ = rosnode_->subscribe(so);

    odometry_publisher_ = rosnode_->advertise<nav_msgs::Odometry>(odometry_topic_, 1);
    ground_truth_publisher_ = rosnode_->advertise<nav_msgs::Odometry>(ground_truth_topic_, 1);

    // start custom queue for diff drive
    this->callback_queue_thread_ =
      boost::thread(boost::bind(&GazeboRosSkidSteerDrive::QueueThread, this));

    // listen to the update event (broadcast every simulation iteration)
    this->update_connection_ =
      event::Events::ConnectWorldUpdateBegin(
          boost::bind(&GazeboRosSkidSteerDrive::UpdateChild, this));

  }

  // Update the controller
  void GazeboRosSkidSteerDrive::UpdateChild() {
#if GAZEBO_MAJOR_VERSION >= 8
    common::Time current_time = this->world->SimTime();
#else
    common::Time current_time = this->world->GetSimTime();
#endif
    double seconds_since_last_update =
      (current_time - last_update_time_).Double();
    if (seconds_since_last_update > update_period_) {

      publishOdometry(seconds_since_last_update);
      publishGroundTruth(seconds_since_last_update);

      // Update robot in case new velocities have been requested
      getWheelVelocities();
#if GAZEBO_MAJOR_VERSION > 2
      joints[LEFT_FRONT]->SetParam("vel", 0, wheel_speed_[LEFT_FRONT] / (wheel_diameter_ / 2.0));
      joints[RIGHT_FRONT]->SetParam("vel", 0, wheel_speed_[RIGHT_FRONT] / (wheel_diameter_ / 2.0));
      joints[LEFT_REAR]->SetParam("vel", 0, wheel_speed_[LEFT_REAR] / (wheel_diameter_ / 2.0));
      joints[RIGHT_REAR]->SetParam("vel", 0, wheel_speed_[RIGHT_REAR] / (wheel_diameter_ / 2.0));
#else
      joints[LEFT_FRONT]->SetVelocity(0, wheel_speed_[LEFT_FRONT] / (wheel_diameter_ / 2.0));
      joints[RIGHT_FRONT]->SetVelocity(0, wheel_speed_[RIGHT_FRONT] / (wheel_diameter_ / 2.0));
      joints[LEFT_REAR]->SetVelocity(0, wheel_speed_[LEFT_REAR] / (wheel_diameter_ / 2.0));
      joints[RIGHT_REAR]->SetVelocity(0, wheel_speed_[RIGHT_REAR] / (wheel_diameter_ / 2.0));
#endif

      last_update_time_+= common::Time(update_period_);

    }
  }

  // Finalize the controller
  void GazeboRosSkidSteerDrive::FiniChild() {
    alive_ = false;
    queue_.clear();
    queue_.disable();
    rosnode_->shutdown();
    callback_queue_thread_.join();
  }

  void GazeboRosSkidSteerDrive::getWheelVelocities() {
    boost::mutex::scoped_lock scoped_lock(lock);

    double vr = x_;
    double va = rot_;

    wheel_speed_[RIGHT_FRONT] = vr + va * wheel_separation_ / 2.0;
    wheel_speed_[RIGHT_REAR] = vr + va * wheel_separation_ / 2.0;

    wheel_speed_[LEFT_FRONT] = vr - va * wheel_separation_ / 2.0;
    wheel_speed_[LEFT_REAR] = vr - va * wheel_separation_ / 2.0;

  }

  void GazeboRosSkidSteerDrive::cmdVelCallback(
      const geometry_msgs::Twist::ConstPtr& cmd_msg) {

    boost::mutex::scoped_lock scoped_lock(lock);
    x_ = cmd_msg->linear.x;
    rot_ = cmd_msg->angular.z;
  }

  void GazeboRosSkidSteerDrive::QueueThread() {
    static const double timeout = 0.01;

    while (alive_ && rosnode_->ok()) {
      queue_.callAvailable(ros::WallDuration(timeout));
    }
  }

  void GazeboRosSkidSteerDrive::publishOdometry(double step_time) {  // Impt
    ros::Time current_time = ros::Time::now();
    std::string odom_frame =
      tf::resolve(tf_prefix_, odometry_frame_);
    std::string base_footprint_frame =
      tf::resolve(tf_prefix_, robot_base_frame_);

    // TODO create some non-perfect odometry!
    // getting data for base_footprint to odom transform
#if GAZEBO_MAJOR_VERSION >= 8
    ignition::math::Pose3d pose = this->parent->WorldPose();
#else
    ignition::math::Pose3d pose = this->parent->GetWorldPose().Ign();
#endif

    tf::Quaternion qt(pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z(), pose.Rot().W());

    if(fabs(x_) > 0 || fabs(rot_) > 0)
    {
      x_error = x_error + GaussianKernel(0, this->gaussian_noise_);
      y_error = y_error + GaussianKernel(0, this->gaussian_noise_);
      yaw_error = yaw_error + GaussianKernel(0, this->gaussian_noise_);
    }

    double r, p, y;
    tf::Matrix3x3(qt).getRPY(r, p, y);
    y = y + yaw_error;
    qt.setRPY(r,p,y);

    tf::Vector3 vt(pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z());

    tf::Transform base_footprint_to_odom(qt, vt);
    if (this->broadcast_tf_) {

    	transform_broadcaster_->sendTransform(
        tf::StampedTransform(base_footprint_to_odom, current_time,
            odom_frame, base_footprint_frame));
    }

    // publish odom topic
    odom_.pose.pose.position.x = pose.Pos().X() + x_error;
    odom_.pose.pose.position.y = pose.Pos().Y() + y_error;

    odom_.pose.pose.orientation.x = qt[0];
    odom_.pose.pose.orientation.y = qt[1];
    odom_.pose.pose.orientation.z = qt[2];
    odom_.pose.pose.orientation.w = qt[3];
    odom_.pose.covariance[0] = this->covariance_x_;
    odom_.pose.covariance[7] = this->covariance_y_;
    odom_.pose.covariance[14] = 1000000000000.0;
    odom_.pose.covariance[21] = 1000000000000.0;
    odom_.pose.covariance[28] = 1000000000000.0;
    odom_.pose.covariance[35] = this->covariance_yaw_;

    // get velocity in /odom frame
    ignition::math::Vector3d linear;
#if GAZEBO_MAJOR_VERSION >= 8
    linear = this->parent->WorldLinearVel();
    odom_.twist.twist.angular.z = this->parent->WorldAngularVel().Z();
#else
    linear = this->parent->GetWorldLinearVel().Ign();
    odom_.twist.twist.angular.z = this->parent->GetWorldAngularVel().Ign().Z();
#endif

    // convert velocity to child_frame_id (aka base_footprint)
    float yaw = pose.Rot().Yaw();
    odom_.twist.twist.linear.x = cosf(yaw) * linear.X() + sinf(yaw) * linear.Y();
    odom_.twist.twist.linear.y = cosf(yaw) * linear.Y() - sinf(yaw) * linear.X();
    odom_.twist.covariance[0] = this->covariance_x_;
    odom_.twist.covariance[7] = this->covariance_y_;
    odom_.twist.covariance[14] = 1000000000000.0;
    odom_.twist.covariance[21] = 1000000000000.0;
    odom_.twist.covariance[28] = 1000000000000.0;
    odom_.twist.covariance[35] = this->covariance_yaw_;

    odom_.header.stamp = current_time;
    odom_.header.frame_id = odom_frame;
    odom_.child_frame_id = base_footprint_frame;

    odometry_publisher_.publish(odom_);
  }

  void GazeboRosSkidSteerDrive::publishGroundTruth(double step_time) { 
    ros::Time current_time = ros::Time::now();
    std::string gt_frame =
      tf::resolve(tf_prefix_, ground_truth_frame_);
    std::string base_footprint_frame =
      tf::resolve(tf_prefix_, robot_base_frame_);

#if GAZEBO_MAJOR_VERSION >= 8
    ignition::math::Pose3d pose = this->parent->WorldPose();
#else
    ignition::math::Pose3d pose = this->parent->GetWorldPose().Ign();
#endif
    tf::Quaternion qt(pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z(), pose.Rot().W());

    tf::Vector3 vt(pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z());

    tf::Transform base_footprint_to_odom(qt, vt);
    if (this->broadcast_tf_) {

      transform_broadcaster_->sendTransform(
        tf::StampedTransform(base_footprint_to_odom, current_time,
            gt_frame, base_footprint_frame));

    }

    // publish gt topic
    ground_truth_.pose.pose.position.x = pose.Pos().X();
    ground_truth_.pose.pose.position.y = pose.Pos().Y();

    ground_truth_.pose.pose.orientation.x = qt[0];
    ground_truth_.pose.pose.orientation.y = qt[1];
    ground_truth_.pose.pose.orientation.z = qt[2];
    ground_truth_.pose.pose.orientation.w = qt[3];
    ground_truth_.pose.covariance[0] = this->covariance_x_;
    ground_truth_.pose.covariance[7] = this->covariance_y_;
    ground_truth_.pose.covariance[14] = 1000000000000.0;
    ground_truth_.pose.covariance[21] = 1000000000000.0;
    ground_truth_.pose.covariance[28] = 1000000000000.0;
    ground_truth_.pose.covariance[35] = this->covariance_yaw_;

    // get velocity in /odom frame
    ignition::math::Vector3d linear;
#if GAZEBO_MAJOR_VERSION >= 8
    linear = this->parent->WorldLinearVel();
    ground_truth_.twist.twist.angular.z = this->parent->WorldAngularVel().Z();
#else
    linear = this->parent->GetWorldLinearVel().Ign();
    ground_truth_.twist.twist.angular.z = this->parent->GetWorldAngularVel().Ign().Z();
#endif

    // convert velocity to child_frame_id (aka base_footprint)
    float yaw = pose.Rot().Yaw();
    ground_truth_.twist.twist.linear.x = cosf(yaw) * linear.X() + sinf(yaw) * linear.Y();
    ground_truth_.twist.twist.linear.y = cosf(yaw) * linear.Y() - sinf(yaw) * linear.X();
    ground_truth_.twist.covariance[0] = this->covariance_x_;
    ground_truth_.twist.covariance[7] = this->covariance_y_;
    ground_truth_.twist.covariance[14] = 1000000000000.0;
    ground_truth_.twist.covariance[21] = 1000000000000.0;
    ground_truth_.twist.covariance[28] = 1000000000000.0;
    ground_truth_.twist.covariance[35] = this->covariance_yaw_;

    ground_truth_.header.stamp = current_time;
    ground_truth_.header.frame_id = gt_frame;
    ground_truth_.child_frame_id = base_footprint_frame;

    ground_truth_publisher_.publish(ground_truth_);
  }

  // Utility for adding noise. (taken from gazebo_ros_p3d)
  double GazeboRosSkidSteerDrive::GaussianKernel(double mu, double sigma) // change this
  {
    // using Box-Muller transform to generate two independent standard
    // normally disbributed normal variables see wikipedia

    // normalized uniform random variable
    double U = static_cast<double>(rand_r(&this->seed)) /
               static_cast<double>(RAND_MAX);

    // normalized uniform random variable
    double V = static_cast<double>(rand_r(&this->seed)) /
               static_cast<double>(RAND_MAX);

    double X = sqrt(-2.0 * ::log(U)) * cos(2.0*M_PI * V);
    // double Y = sqrt(-2.0 * ::log(U)) * sin(2.0*M_PI * V);

    // there are 2 indep. vars, we'll just use X
    // scale to our mu and sigma
    X = sigma * X + mu;
    return X;
  }

  GZ_REGISTER_MODEL_PLUGIN(GazeboRosSkidSteerDrive)
}

//////////////////////////////////////////////////////////////////////////////

