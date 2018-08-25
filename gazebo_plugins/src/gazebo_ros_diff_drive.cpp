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

#include <algorithm>
#include <assert.h>

#include <gazebo_plugins/gazebo_ros_diff_drive.hpp>

#include <ignition/math/Angle.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>
#include <sdf/sdf.hh>

#include <builtin_interfaces/msg/time.hpp>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>


namespace gazebo
{

enum {
    RIGHT,
    LEFT,
};

GazeboRosDiffDrive::GazeboRosDiffDrive() {}

// Destructor
GazeboRosDiffDrive::~GazeboRosDiffDrive() {}

// Load the controller
void GazeboRosDiffDrive::Load ( physics::ModelPtr _parent, sdf::ElementPtr _sdf )
{

    this->parent = _parent;
    gazebo_ros_ = gazebo_ros::Node::Create("gazebo_ros_diff_drive", _sdf);

    // Make sure the ROS node for Gazebo has already been initialized
    // gazebo_ros_->isInitialized(); TODO(tfoote) needs replacement?

    gazebo_ros_->get_parameter_or<std::string> ( "commandTopic", command_topic_, "cmd_vel" );
    gazebo_ros_->get_parameter_or<std::string> ( "odometryTopic", odometry_topic_, "odom" );
    gazebo_ros_->get_parameter_or<std::string> ( "odometryFrame", odometry_frame_, "odom" );
    gazebo_ros_->get_parameter_or<std::string> ( "robotBaseFrame", robot_base_frame_, "base_footprint" );
    gazebo_ros_->get_parameter_or<bool> ( "publishWheelTF", publishWheelTF_, false );
    gazebo_ros_->get_parameter_or<bool> ( "publishOdomTF", publishOdomTF_, true);
    gazebo_ros_->get_parameter_or<bool> ( "publishWheelJointState", publishWheelJointState_, false );
    gazebo_ros_->get_parameter_or<double> ( "wheelSeparation", wheel_separation_, 0.34 );
    gazebo_ros_->get_parameter_or<double> ( "wheelDiameter", wheel_diameter_, 0.15 );
    gazebo_ros_->get_parameter_or<double> ( "wheelAcceleration", wheel_accel, 0.0 );
    gazebo_ros_->get_parameter_or<double> ( "wheelTorque", wheel_torque, 5.0 );
    gazebo_ros_->get_parameter_or<double> ( "updateRate", update_rate_, 100.0 );
    std::map<std::string, OdomSource> odomOptions;
    odomOptions["encoder"] = ENCODER;
    odomOptions["world"] = WORLD;
    // TODO(tfoote) restore parameter
    // gazebo_ros_->get_parameter_or<OdomSource> (  "odometrySource", odomOptions, WORLD );


    joints_.resize ( 2 );
    joints_[LEFT] = parent->GetJoint("leftJoint"); // TODO(tfoote) or undercored left_joint
    joints_[RIGHT] = parent->GetJoint("rightJoint");
    joints_[LEFT]->SetParam ( "fmax", 0, wheel_torque );
    joints_[RIGHT]->SetParam ( "fmax", 0, wheel_torque );



    this->publish_tf_ = true;
    if (!_sdf->HasElement("publishTf")) {
      RCLCPP_WARN(gazebo_ros_->get_logger(),
          "diff_drive", "GazeboRosDiffDrive Plugin (ns = %s) missing <publishTf>, defaults to %d",
          this->robot_namespace_.c_str(), this->publish_tf_);
    } else {
      this->publish_tf_ = _sdf->GetElement("publishTf")->Get<bool>();
    }

    // Initialize update rate stuff
    if ( this->update_rate_ > 0.0 ) this->update_period_ = 1.0 / this->update_rate_;
    else this->update_period_ = 0.0;
#if GAZEBO_MAJOR_VERSION >= 8
    last_update_time_ = parent->GetWorld()->SimTime();
#else
    last_update_time_ = parent->GetWorld()->GetSimTime();
#endif

    // Initialize velocity stuff
    wheel_speed_[RIGHT] = 0;
    wheel_speed_[LEFT] = 0;

    // Initialize velocity support stuff
    wheel_speed_instr_[RIGHT] = 0;
    wheel_speed_instr_[LEFT] = 0;

    x_ = 0;
    rot_ = 0;


    if (this->publishWheelJointState_)
    {
        joint_state_publisher_ = gazebo_ros_->create_publisher<sensor_msgs::msg::JointState>("joint_states");
        // TODO(tfoote) queue size 1000 equivalent qos
        RCLCPP_INFO(gazebo_ros_->get_logger(),
            "diff_drive", "%s: Advertise joint_states", gazebo_ros_->get_name());
    }

    transform_broadcaster_ = std::shared_ptr<tf2_ros::TransformBroadcaster>(new tf2_ros::TransformBroadcaster(gazebo_ros_));

    // ROS: Subscribe to the velocity command topic (usually "cmd_vel")
    RCLCPP_INFO(gazebo_ros_->get_logger(),
        "diff_drive", "%s: Try to subscribe to %s", gazebo_ros_->get_name(), command_topic_.c_str());

    // TODO(tfoote) equivalent qos settings as below
    // ros::SubscribeOptions so =
    //     ros::SubscribeOptions::create<geometry_msgs::Twist>(command_topic_, 1,
    //             boost::bind(&GazeboRosDiffDrive::cmdVelCallback, this, _1),
    //             ros::VoidPtr(), &queue_);


    cmd_vel_subscriber_ = gazebo_ros_->create_subscription<geometry_msgs::msg::Twist::SharedPtr>(
        command_topic_, std::bind(&GazeboRosDiffDrive::cmdVelCallback, this,
        std::placeholders::_1));
    RCLCPP_INFO(gazebo_ros_->get_logger(),
        "diff_drive", "%s: Subscribe to %s", gazebo_ros_->get_name(), command_topic_.c_str());

    if (this->publish_tf_)
    {
        //TODO(tfoote)mimic qos publisher queue size 1
        odometry_publisher_ = gazebo_ros_->create_publisher<nav_msgs::msg::Odometry>(odometry_topic_);
  
    RCLCPP_INFO(gazebo_ros_->get_logger(),
        "diff_drive", "%s: Advertise odom on %s ", gazebo_ros_->get_name(), odometry_topic_.c_str());
    }

    // listen to the update event (broadcast every simulation iteration)
    this->update_connection_ =
        event::Events::ConnectWorldUpdateBegin ( std::bind ( &GazeboRosDiffDrive::UpdateChild, this ) );

}

void GazeboRosDiffDrive::Reset()
{
#if GAZEBO_MAJOR_VERSION >= 8
  last_update_time_ = parent->GetWorld()->SimTime();
#else
  last_update_time_ = parent->GetWorld()->GetSimTime();
#endif
  pose_encoder_.x = 0;
  pose_encoder_.y = 0;
  pose_encoder_.theta = 0;
  x_ = 0;
  rot_ = 0;
  joints_[LEFT]->SetParam ( "fmax", 0, wheel_torque );
  joints_[RIGHT]->SetParam ( "fmax", 0, wheel_torque );
}

void GazeboRosDiffDrive::publishWheelJointState()
{
    rclcpp::Time current_time = gazebo_ros_->now();
    joint_state_.header.stamp = current_time;
    joint_state_.name.resize ( joints_.size() );
    joint_state_.position.resize ( joints_.size() );

    for ( int i = 0; i < 2; i++ ) {
        physics::JointPtr joint = joints_[i];
#if GAZEBO_MAJOR_VERSION >= 8
        double position = joint->Position ( 0 );
#else
        double position = joint->GetAngle ( 0 ).Radian();
#endif
        joint_state_.name[i] = joint->GetName();
        joint_state_.position[i] = position;
    }
    joint_state_publisher_->publish ( joint_state_ );
}

void GazeboRosDiffDrive::publishWheelTF()
{
    rclcpp::Time current_time = gazebo_ros_->now();
    for ( int i = 0; i < 2; i++ ) {

        std::string wheel_frame = joints_[i]->GetChild()->GetName ();
        std::string wheel_parent_frame = joints_[i]->GetParent()->GetName ();

#if GAZEBO_MAJOR_VERSION >= 8
        ignition::math::Pose3d poseWheel = joints_[i]->GetChild()->RelativePose();
#else
        ignition::math::Pose3d poseWheel = joints_[i]->GetChild()->GetRelativePose().Ign();
#endif

        geometry_msgs::msg::TransformStamped msg;
        msg.header.stamp = current_time;
        msg.header.frame_id = wheel_parent_frame;
        msg.child_frame_id = wheel_frame;
        msg.transform.translation.x = poseWheel.Pos().X();
        msg.transform.translation.y = poseWheel.Pos().Y();
        msg.transform.translation.z = poseWheel.Pos().Z();
        msg.transform.rotation.x = poseWheel.Rot().X();
        msg.transform.rotation.y = poseWheel.Rot().Y();
        msg.transform.rotation.z = poseWheel.Rot().Z();
        msg.transform.rotation.w = poseWheel.Rot().W();
        transform_broadcaster_->sendTransform ( msg );
    }
}

// Update the controller
void GazeboRosDiffDrive::UpdateChild()
{

    /* force reset SetParam("fmax") since Joint::Reset reset MaxForce to zero at
       https://bitbucket.org/osrf/gazebo/src/8091da8b3c529a362f39b042095e12c94656a5d1/gazebo/physics/Joint.cc?at=gazebo2_2.2.5#cl-331
       (this has been solved in https://bitbucket.org/osrf/gazebo/diff/gazebo/physics/Joint.cc?diff2=b64ff1b7b6ff&at=issue_964 )
       and Joint::Reset is called after ModelPlugin::Reset, so we need to set maxForce to wheel_torque other than GazeboRosDiffDrive::Reset
       (this seems to be solved in https://bitbucket.org/osrf/gazebo/commits/ec8801d8683160eccae22c74bf865d59fac81f1e)
    */
    for ( int i = 0; i < 2; i++ ) {
      if ( fabs(wheel_torque -joints_[i]->GetParam ( "fmax", 0 )) > 1e-6 ) {
        joints_[i]->SetParam ( "fmax", 0, wheel_torque );
      }
    }


    if ( odom_source_ == ENCODER ) UpdateOdometryEncoder();
#if GAZEBO_MAJOR_VERSION >= 8
    common::Time current_time = parent->GetWorld()->SimTime();
#else
    common::Time current_time = parent->GetWorld()->GetSimTime();
#endif
    double seconds_since_last_update = ( current_time - last_update_time_ ).Double();

    if ( seconds_since_last_update > update_period_ ) {
        if (this->publish_tf_) publishOdometry ( seconds_since_last_update );
        if ( publishWheelTF_ ) publishWheelTF();
        if ( publishWheelJointState_ ) publishWheelJointState();

        // Update robot in case new velocities have been requested
        getWheelVelocities();

        double current_speed[2];

        current_speed[LEFT] = joints_[LEFT]->GetVelocity ( 0 )   * ( wheel_diameter_ / 2.0 );
        current_speed[RIGHT] = joints_[RIGHT]->GetVelocity ( 0 ) * ( wheel_diameter_ / 2.0 );

        if ( wheel_accel == 0 ||
                ( fabs ( wheel_speed_[LEFT] - current_speed[LEFT] ) < 0.01 ) ||
                ( fabs ( wheel_speed_[RIGHT] - current_speed[RIGHT] ) < 0.01 ) ) {
            //if max_accel == 0, or target speed is reached
            joints_[LEFT]->SetParam ( "vel", 0, wheel_speed_[LEFT]/ ( wheel_diameter_ / 2.0 ) );
            joints_[RIGHT]->SetParam ( "vel", 0, wheel_speed_[RIGHT]/ ( wheel_diameter_ / 2.0 ) );
        } else {
            if ( wheel_speed_[LEFT]>=current_speed[LEFT] )
                wheel_speed_instr_[LEFT]+=fmin ( wheel_speed_[LEFT]-current_speed[LEFT],  wheel_accel * seconds_since_last_update );
            else
                wheel_speed_instr_[LEFT]+=fmax ( wheel_speed_[LEFT]-current_speed[LEFT], -wheel_accel * seconds_since_last_update );

            if ( wheel_speed_[RIGHT]>current_speed[RIGHT] )
                wheel_speed_instr_[RIGHT]+=fmin ( wheel_speed_[RIGHT]-current_speed[RIGHT], wheel_accel * seconds_since_last_update );
            else
                wheel_speed_instr_[RIGHT]+=fmax ( wheel_speed_[RIGHT]-current_speed[RIGHT], -wheel_accel * seconds_since_last_update );

            // ROS_INFO_NAMED("diff_drive", "actual wheel speed = %lf, issued wheel speed= %lf", current_speed[LEFT], wheel_speed_[LEFT]);
            // ROS_INFO_NAMED("diff_drive", "actual wheel speed = %lf, issued wheel speed= %lf", current_speed[RIGHT],wheel_speed_[RIGHT]);

            joints_[LEFT]->SetParam ( "vel", 0, wheel_speed_instr_[LEFT] / ( wheel_diameter_ / 2.0 ) );
            joints_[RIGHT]->SetParam ( "vel", 0, wheel_speed_instr_[RIGHT] / ( wheel_diameter_ / 2.0 ) );
        }
        last_update_time_+= common::Time ( update_period_ );
    }
}

// Finalize the controller
void GazeboRosDiffDrive::FiniChild()
{
}

void GazeboRosDiffDrive::getWheelVelocities()
{
    std::lock_guard<std::mutex> scoped_lock ( lock );

    double vr = x_;
    double va = rot_;

    wheel_speed_[LEFT] = vr - va * wheel_separation_ / 2.0;
    wheel_speed_[RIGHT] = vr + va * wheel_separation_ / 2.0;
}

void GazeboRosDiffDrive::cmdVelCallback ( const geometry_msgs::msg::Twist::SharedPtr cmd_msg )
{
    std::lock_guard<std::mutex> scoped_lock ( lock );
    x_ = cmd_msg->linear.x;
    rot_ = cmd_msg->angular.z;
}

void GazeboRosDiffDrive::UpdateOdometryEncoder()
{
    double vl = joints_[LEFT]->GetVelocity ( 0 );
    double vr = joints_[RIGHT]->GetVelocity ( 0 );
#if GAZEBO_MAJOR_VERSION >= 8
    common::Time current_time = parent->GetWorld()->SimTime();
#else
    common::Time current_time = parent->GetWorld()->GetSimTime();
#endif
    double seconds_since_last_update = ( current_time - last_odom_update_ ).Double();
    last_odom_update_ = current_time;

    double b = wheel_separation_;

    // Book: Sigwart 2011 Autonompus Mobile Robots page:337
    double sl = vl * ( wheel_diameter_ / 2.0 ) * seconds_since_last_update;
    double sr = vr * ( wheel_diameter_ / 2.0 ) * seconds_since_last_update;
    double ssum = sl + sr;

    double sdiff = sr - sl;

    double dx = ( ssum ) /2.0 * cos ( pose_encoder_.theta + ( sdiff ) / ( 2.0*b ) );
    double dy = ( ssum ) /2.0 * sin ( pose_encoder_.theta + ( sdiff ) / ( 2.0*b ) );
    double dtheta = ( sdiff ) /b;

    pose_encoder_.x += dx;
    pose_encoder_.y += dy;
    pose_encoder_.theta += dtheta;

    double w = dtheta/seconds_since_last_update;
    double v = sqrt ( dx*dx+dy*dy ) /seconds_since_last_update;

    tf2::Quaternion qt;
    tf2::Vector3 vt;
    qt.setRPY ( 0,0,pose_encoder_.theta );
    vt = tf2::Vector3 ( pose_encoder_.x, pose_encoder_.y, 0 );

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

void GazeboRosDiffDrive::publishOdometry ( double step_time )
{

    rclcpp::Time current_time = gazebo_ros_->now();

    tf2::Quaternion qt;
    tf2::Vector3 vt;

    if ( odom_source_ == ENCODER ) {
        // getting data form encoder integration
        qt = tf2::Quaternion ( odom_.pose.pose.orientation.x, odom_.pose.pose.orientation.y, odom_.pose.pose.orientation.z, odom_.pose.pose.orientation.w );
        vt = tf2::Vector3 ( odom_.pose.pose.position.x, odom_.pose.pose.position.y, odom_.pose.pose.position.z );

    }
    if ( odom_source_ == WORLD ) {
        // getting data from gazebo world
#if GAZEBO_MAJOR_VERSION >= 8
        ignition::math::Pose3d pose = parent->WorldPose();
#else
        ignition::math::Pose3d pose = parent->GetWorldPose().Ign();
#endif
        qt = tf2::Quaternion ( pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z(), pose.Rot().W() );
        vt = tf2::Vector3 ( pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z() );

        odom_.pose.pose.position.x = vt.x();
        odom_.pose.pose.position.y = vt.y();
        odom_.pose.pose.position.z = vt.z();

        odom_.pose.pose.orientation.x = qt.x();
        odom_.pose.pose.orientation.y = qt.y();
        odom_.pose.pose.orientation.z = qt.z();
        odom_.pose.pose.orientation.w = qt.w();

        // get velocity in /odom frame
        ignition::math::Vector3d linear;
#if GAZEBO_MAJOR_VERSION >= 8
        linear = parent->WorldLinearVel();
        odom_.twist.twist.angular.z = parent->WorldAngularVel().Z();
#else
        linear = parent->GetWorldLinearVel().Ign();
        odom_.twist.twist.angular.z = parent->GetWorldAngularVel().Ign().Z();
#endif

        // convert velocity to child_frame_id (aka base_footprint)
        float yaw = pose.Rot().Yaw();
        odom_.twist.twist.linear.x = cosf ( yaw ) * linear.X() + sinf ( yaw ) * linear.Y();
        odom_.twist.twist.linear.y = cosf ( yaw ) * linear.Y() - sinf ( yaw ) * linear.X();
    }

    if (publishOdomTF_ == true){

        geometry_msgs::msg::TransformStamped msg;
        msg.header.stamp = current_time;
        msg.header.frame_id = odometry_frame_;
        msg.child_frame_id = robot_base_frame_;
        msg.transform.translation.x = vt.x();
        msg.transform.translation.y = vt.y();
        msg.transform.translation.z = vt.z();
        msg.transform.rotation.x = qt.x();
        msg.transform.rotation.y = qt.y();
        msg.transform.rotation.z = qt.z();
        msg.transform.rotation.w = qt.w();

        transform_broadcaster_->sendTransform (msg);
    }


    // set covariance
    odom_.pose.covariance[0] = 0.00001;
    odom_.pose.covariance[7] = 0.00001;
    odom_.pose.covariance[14] = 1000000000000.0;
    odom_.pose.covariance[21] = 1000000000000.0;
    odom_.pose.covariance[28] = 1000000000000.0;
    odom_.pose.covariance[35] = 0.001;


    // set header
    odom_.header.stamp = current_time;
    odom_.header.frame_id = odometry_frame_;
    odom_.child_frame_id = robot_base_frame_;

    odometry_publisher_->publish ( odom_ );
}

GZ_REGISTER_MODEL_PLUGIN ( GazeboRosDiffDrive )
}
