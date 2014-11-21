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

#include <gazebo_plugins/gazebo_ros_diff_drive.h>

#include <gazebo/math/gzmath.hh>
#include <sdf/sdf.hh>

#include <ros/ros.h>

namespace gazebo {

enum {
    RIGHT = 0,
    LEFT = 1,
};
enum {
  FORWARD = 0,
  ANGULAR = 1,
};

GazeboRosDiffDrive::GazeboRosDiffDrive() {}

// Destructor
GazeboRosDiffDrive::~GazeboRosDiffDrive() {}

// Load the controller
void GazeboRosDiffDrive::Load ( physics::ModelPtr _parent, sdf::ElementPtr _sdf ) {

    this->parent = _parent;
    gazebo_ros_ = GazeboRosPtr ( new GazeboRos ( _parent, _sdf, "DiffDrive" ) );
    // Make sure the ROS node for Gazebo has already been initialized
    gazebo_ros_->isInitialized();

    gazebo_ros_->getParameter<std::string> ( command_topic_, "commandTopic", "cmd_vel" );
    gazebo_ros_->getParameter<std::string> ( odometry_topic_, "odometryTopic", "odom" );
    gazebo_ros_->getParameter<std::string> ( odometry_frame_, "odometryFrame", "odom" );
    gazebo_ros_->getParameter<std::string> ( trip_recorder_topic_, "tripRecorderTopic", "trip_recorder" );
    gazebo_ros_->getParameter<std::string> ( command_current_topic_, "commandCurrentTopic", "cmd_vel_current" );
    gazebo_ros_->getParameter<std::string> ( command_target_topic_, "commandTargetTopic", "cmd_vel_target" );
    gazebo_ros_->getParameter<std::string> ( robot_base_frame_, "robotBaseFrame", "base_footprint" );
    gazebo_ros_->getParameterBoolean ( publishWheelTF_, "publishWheelTF", false );
    gazebo_ros_->getParameterBoolean ( publishWheelJointState_, "publishWheelJointState", false );


    gazebo_ros_->getParameter<double> ( trip_recorder_scale_, "tripRecorderScale", 1000. );
    gazebo_ros_->getParameter<double> ( wheel_separation_, "wheelSeparation", 0.34 );
    gazebo_ros_->getParameter<double> ( wheel_diameter_, "wheelDiameter", 0.15 );
    gazebo_ros_->getParameter<double> ( accel[FORWARD], "Acceleration_v", 0.0 );
    gazebo_ros_->getParameter<double> ( accel[ANGULAR], "Acceleration_w", 0.0 );
    gazebo_ros_->getParameter<double> ( wheel_torque, "wheelTorque", 5.0 );
    gazebo_ros_->getParameter<double> ( update_rate_, "updateRate", 100.0 );

    std::map<std::string, OdomSource> odomOptions;
    odomOptions["encoder"] = ENCODER;
    odomOptions["world"] = WORLD;
    gazebo_ros_->getParameter<OdomSource> ( odom_source_, "odometrySource", odomOptions, WORLD );

    odometry_frame_resolved_ = gazebo_ros_->resolveTF ( odometry_frame_ );
    robot_base_frame_resolved_ = gazebo_ros_->resolveTF ( robot_base_frame_ );

    joints_.resize ( 2 );
    joints_[LEFT] = gazebo_ros_->getJoint ( parent, "leftJoint", "left_joint" );
    joints_[RIGHT] = gazebo_ros_->getJoint ( parent, "rightJoint", "right_joint" );
    joints_[LEFT]->SetMaxForce ( 0, wheel_torque );
    joints_[RIGHT]->SetMaxForce ( 0, wheel_torque );



    this->publish_tf_ = true;
    if ( !_sdf->HasElement ( "publishTf" ) ) {
        ROS_WARN ( "%s: <publishTf> is missing, using default %d!",
                   gazebo_ros_->info(), this->publish_tf_ );
    } else {
        this->publish_tf_ = _sdf->GetElement ( "publishTf" )->Get<bool>();
    }

    // Initialize update rate stuff
    if ( this->update_rate_ > 0.0 ) this->update_period_ = 1.0 / this->update_rate_;
    else this->update_period_ = 0.0;
    last_update_time_ = parent->GetWorld()->GetSimTime();

    // Initialize velocity stuff
    wheel_speed_[RIGHT] = 0;
    wheel_speed_[LEFT] = 0;

    command_current_.linear.x = 0;
    command_current_.linear.y = 0;
    command_current_.linear.z = 0;
    command_current_.angular.x = 0;
    command_current_.angular.y = 0;
    command_current_.angular.z = 0;
    command_target_.linear.x = 0;
    command_target_.linear.y = 0;
    command_target_.linear.z = 0;
    command_target_.angular.x = 0;
    command_target_.angular.y = 0;
    command_target_.angular.z = 0;
    trip_recorder_ = 0;
    trip_recorder_sub_meter_ = 0.;
    alive_ = true;


    if ( this->publishWheelJointState_ ) {
        joint_state_publisher_ = gazebo_ros_->node()->advertise<sensor_msgs::JointState> ( "joint_states", 1000 );
        ROS_INFO ( "%s: Advertise joint_states!", gazebo_ros_->info() );
    }

    transform_broadcaster_ = boost::shared_ptr<tf::TransformBroadcaster> ( new tf::TransformBroadcaster() );

    // ROS: Subscribe to the velocity command topic (usually "cmd_vel")
    ROS_INFO ( "%s: Try to subscribe to %s!", gazebo_ros_->info(), command_topic_.c_str() );

    ros::SubscribeOptions so =
        ros::SubscribeOptions::create<geometry_msgs::Twist> ( command_topic_, 1,
                boost::bind ( &GazeboRosDiffDrive::cmdVelCallback, this, _1 ),
                ros::VoidPtr(), &queue_ );

    cmd_vel_subscriber_ = gazebo_ros_->node()->subscribe ( so );
    ROS_INFO ( "%s: Subscribe to %s!", gazebo_ros_->info(), command_topic_.c_str() );

    if ( this->publish_tf_ ) {
        odometry_publisher_ = gazebo_ros_->node()->advertise<nav_msgs::Odometry> ( odometry_topic_, 1 );
        ROS_INFO ( "%s: Advertise odom on %s !", gazebo_ros_->info(), odometry_topic_.c_str() );
    }

    trip_recorder_publisher_ = gazebo_ros_->node()->advertise<std_msgs::UInt64> ( trip_recorder_topic_, 1 );
    ROS_INFO ( "%s: Advertise trip recorder on %s scale by %f!", gazebo_ros_->info(), trip_recorder_topic_.c_str(), trip_recorder_scale_ );

    command_current_publisher_ = gazebo_ros_->node()->advertise<geometry_msgs::Twist> ( command_current_topic_, 1 );
    ROS_INFO ( "%s: Advertise current command on %s !", gazebo_ros_->info(), command_current_topic_.c_str() );

    command_target_publisher_ = gazebo_ros_->node()->advertise<geometry_msgs::Twist> ( command_target_topic_, 1 );
    ROS_INFO ( "%s: Advertise target command on %s !", gazebo_ros_->info(), command_target_topic_.c_str() );

    // start custom queue for diff drive
    this->callback_queue_thread_ =
        boost::thread ( boost::bind ( &GazeboRosDiffDrive::QueueThread, this ) );

    // listen to the update event (broadcast every simulation iteration)
    this->update_connection_ =
        event::Events::ConnectWorldUpdateBegin ( boost::bind ( &GazeboRosDiffDrive::UpdateChild, this ) );

}

void GazeboRosDiffDrive::publishWheelJointState() {
    ros::Time current_time = ros::Time::now();

    joint_state_.header.stamp = current_time;
    joint_state_.name.resize ( joints_.size() );
    joint_state_.position.resize ( joints_.size() );

    for ( int i = 0; i < 2; i++ ) {
        physics::JointPtr joint = joints_[i];
        math::Angle angle = joint->GetAngle ( 0 );
        joint_state_.name[i] = joint->GetName();
        joint_state_.position[i] = angle.Radian () ;
    }
    joint_state_publisher_.publish ( joint_state_ );
}

void GazeboRosDiffDrive::publishWheelTF() {
    ros::Time current_time = ros::Time::now();
    for ( int i = 0; i < 2; i++ ) {

        std::string wheel_frame = gazebo_ros_->resolveTF ( joints_[i]->GetChild()->GetName () );
        std::string wheel_parent_frame = gazebo_ros_->resolveTF ( joints_[i]->GetParent()->GetName () );

        math::Pose poseWheel = joints_[i]->GetChild()->GetRelativePose();

        tf::Quaternion qt ( poseWheel.rot.x, poseWheel.rot.y, poseWheel.rot.z, poseWheel.rot.w );
        tf::Vector3 vt ( poseWheel.pos.x, poseWheel.pos.y, poseWheel.pos.z );

        tf::Transform tfWheel ( qt, vt );
        transform_broadcaster_->sendTransform (
            tf::StampedTransform ( tfWheel, current_time, wheel_parent_frame, wheel_frame ) );
    }
}

// Update the controller
void GazeboRosDiffDrive::UpdateChild() {
    updateOdometryEncoder();
    common::Time current_time = parent->GetWorld()->GetSimTime();
    double seconds_since_last_update = ( current_time - last_update_time_ ).Double();
    if ( seconds_since_last_update > update_period_ ) {
        if ( this->publish_tf_ ) publishOdometry ();
        if ( publishWheelTF_ ) publishWheelTF();
        if ( publishWheelJointState_ ) publishWheelJointState();

        // publishes the agent state current and target command as well as distance moved
        publishAgentState();

        // Update robot in case new velocities have been requested
        getWheelVelocities();
		double wheel_l = joints_[LEFT] ->GetVelocity ( 0 ) * ( wheel_diameter_ / 2.0 );
		double wheel_r = joints_[RIGHT]->GetVelocity ( 0 ) * ( wheel_diameter_ / 2.0 );
        double current_vr = (wheel_l + wheel_r)/2.0 ;
        double current_va = (wheel_l - wheel_r)/wheel_diameter_ ;
		double req_vr = vr;
        double req_va = va;	
	
	
        if (/*( accel[FORWARD] == 0 && accel[ANGULAR] == 0)||*/
                (( fabs ( req_vr - current_vr ) < 0.01 ) &&
                 ( fabs ( req_va - current_va ) < 0.01 )) ) {
            //if max_accel == 0, or target speed is reached
            joints_[LEFT]->SetVelocity  ( 0, (req_vr + req_va * wheel_separation_ / 2.0)  / ( wheel_diameter_ / 2.0 ));
            joints_[RIGHT]->SetVelocity ( 0, (req_vr - req_va * wheel_separation_ / 2.0)  / ( wheel_diameter_ / 2.0 ));
        } else {
            if ( req_vr>=current_vr )
                instr_vr += fmin ( req_vr - current_vr,  accel[FORWARD] * seconds_since_last_update );
            else
                instr_vr += fmax ( req_vr - current_vr, -accel[FORWARD] * seconds_since_last_update );

            if ( req_va>=current_va )
                instr_va += fmin ( req_va - current_va,  accel[ANGULAR] * seconds_since_last_update );
            else
                instr_va += fmax ( req_va - current_va, -accel[ANGULAR] * seconds_since_last_update );

             //ROS_INFO("actual wheel speed = %lf, issued wheel speed= %lf", current_vr, req_vr);
             //ROS_INFO("actual wheel speed = %lf, issued wheel speed= %lf", current_va, req_va);

            joints_[LEFT]->SetVelocity  ( 0,(instr_vr + instr_va * wheel_separation_ / 2.0) / ( wheel_diameter_ / 2.0 ));
            joints_[RIGHT]->SetVelocity ( 0,(instr_vr - instr_va * wheel_separation_ / 2.0) / ( wheel_diameter_ / 2.0 ));
        }
        last_update_time_+= common::Time ( update_period_ );
    }
}

// Finalize the controller
void GazeboRosDiffDrive::FiniChild() {
    alive_ = false;
    queue_.clear();
    queue_.disable();
    gazebo_ros_->node()->shutdown();
    callback_queue_thread_.join();
}

void GazeboRosDiffDrive::getWheelVelocities() {
    boost::mutex::scoped_lock scoped_lock ( lock );

    vr = command_target_.linear.x;
    va = command_target_.angular.z;

    wheel_speed_[LEFT] = vr + va * wheel_separation_ / 2.0;
    wheel_speed_[RIGHT] = vr - va * wheel_separation_ / 2.0;
}

void GazeboRosDiffDrive::cmdVelCallback ( const geometry_msgs::Twist::ConstPtr& cmd_msg ) {
    boost::mutex::scoped_lock scoped_lock ( lock );
    command_target_ = *cmd_msg;
}

void GazeboRosDiffDrive::QueueThread() {
    static const double timeout = 0.01;

    while ( alive_ && gazebo_ros_->node()->ok() ) {
        queue_.callAvailable ( ros::WallDuration ( timeout ) );
    }
}

void GazeboRosDiffDrive::updateOdometryEncoder() {
    double vl = joints_[LEFT]->GetVelocity ( 0 );
    double vr = joints_[RIGHT]->GetVelocity ( 0 );
    common::Time current_time = parent->GetWorld()->GetSimTime();
    double seconds_since_last_update = ( current_time - last_odom_update_ ).Double();
    last_odom_update_ = current_time;

    double b = wheel_separation_;

    // Book: Sigwart 2011 Autonomous Mobile Robots page:337
    double sl = vl * ( wheel_diameter_ / 2.0 ) * seconds_since_last_update;
    double sr = vr * ( wheel_diameter_ / 2.0 ) * seconds_since_last_update;
    double theta = ( sl - sr ) /b;


    double dx = ( sl + sr ) /2.0 * cos ( pose_encoder_.theta + ( sl - sr ) / ( 2.0*b ) );
    double dy = ( sl + sr ) /2.0 * sin ( pose_encoder_.theta + ( sl - sr ) / ( 2.0*b ) );
    double dtheta = ( sl - sr ) /b;

    pose_encoder_.x += dx;
    pose_encoder_.y += dy;
    pose_encoder_.theta += dtheta;

    double d = sqrt ( dx*dx + dy*dy );
    trip_recorder_sub_meter_ += d;
    while ( trip_recorder_sub_meter_ >= 1.0 ) {
        trip_recorder_sub_meter_ -= 1.0;
        trip_recorder_++;
    }

    double w = dtheta/seconds_since_last_update;
    double v = sqrt ( dx*dx+dy*dy ) /seconds_since_last_update;

    command_current_.linear.x = v;
    command_current_.linear.y = 0;
    command_current_.linear.z = 0;
    command_current_.angular.x = 0;
    command_current_.angular.y = 0;
    command_current_.angular.z = w;

    if ( odom_source_ == ENCODER ) {
        tf::Quaternion qt;
        tf::Vector3 vt;
        qt.setRPY ( 0,0,pose_encoder_.theta );
        vt = tf::Vector3 ( pose_encoder_.x, pose_encoder_.y, 0 );

        odom_.pose.pose.position.x = vt.x();
        odom_.pose.pose.position.y = vt.y();
        odom_.pose.pose.position.z = vt.z();

        odom_.pose.pose.orientation.x = qt.x();
        odom_.pose.pose.orientation.y = qt.y();
        odom_.pose.pose.orientation.z = qt.z();
        odom_.pose.pose.orientation.w = qt.w();

        odom_.twist.twist.angular.z = w;
        odom_.twist.twist.linear.x = dx/seconds_since_last_update;
        odom_.twist.twist.linear.y = dy/seconds_since_last_update;
    }
}

void GazeboRosDiffDrive::publishAgentState ( ) {

    if ( command_current_publisher_.getNumSubscribers() > 0 ) {
        command_current_publisher_.publish ( command_current_ );
    }
    if ( command_target_publisher_.getNumSubscribers() > 0 ) {
        command_target_publisher_.publish ( command_target_ );
    }
    if ( trip_recorder_publisher_.getNumSubscribers() > 0 ) {
        std_msgs::UInt64 trip;
        trip.data = ( trip_recorder_ * trip_recorder_scale_ ) + ( trip_recorder_sub_meter_ * trip_recorder_scale_ ) + 0.5;
        trip_recorder_publisher_.publish ( trip );
    }
}
void GazeboRosDiffDrive::publishOdometry () {

    ros::Time current_time = ros::Time::now();

    tf::Quaternion qt;
    tf::Vector3 vt;

    if ( odom_source_ == ENCODER ) {
        // getting data form encoder integration
        qt = tf::Quaternion ( odom_.pose.pose.orientation.x, odom_.pose.pose.orientation.y, odom_.pose.pose.orientation.z, odom_.pose.pose.orientation.w );
        vt = tf::Vector3 ( odom_.pose.pose.position.x, odom_.pose.pose.position.y, odom_.pose.pose.position.z );

    }
    if ( odom_source_ == WORLD ) {
        // getting data form gazebo world
        math::Pose pose = parent->GetWorldPose();
        qt = tf::Quaternion ( pose.rot.x, pose.rot.y, pose.rot.z, pose.rot.w );
        vt = tf::Vector3 ( pose.pos.x, pose.pos.y, pose.pos.z );

        odom_.pose.pose.position.x = vt.x();
        odom_.pose.pose.position.y = vt.y();
        odom_.pose.pose.position.z = vt.z();

        odom_.pose.pose.orientation.x = qt.x();
        odom_.pose.pose.orientation.y = qt.y();
        odom_.pose.pose.orientation.z = qt.z();
        odom_.pose.pose.orientation.w = qt.w();

        // get velocity in /odom frame
        math::Vector3 linear;
        linear = parent->GetWorldLinearVel();
        odom_.twist.twist.angular.z = parent->GetWorldAngularVel().z;

        // convert velocity to child_frame_id (aka base_footprint)
        float yaw = pose.rot.GetYaw();
        odom_.twist.twist.linear.x = cosf ( yaw ) * linear.x + sinf ( yaw ) * linear.y;
        odom_.twist.twist.linear.y = cosf ( yaw ) * linear.y - sinf ( yaw ) * linear.x;
    }

    tf::Transform base_footprint_to_odom ( qt, vt );
    transform_broadcaster_->sendTransform (
        tf::StampedTransform ( base_footprint_to_odom, current_time,
                               odometry_frame_resolved_, robot_base_frame_resolved_ ) );


    // set covariance
    odom_.pose.covariance[0] = 0.00001;
    odom_.pose.covariance[7] = 0.00001;
    odom_.pose.covariance[14] = 1000000000000.0;
    odom_.pose.covariance[21] = 1000000000000.0;
    odom_.pose.covariance[28] = 1000000000000.0;
    odom_.pose.covariance[35] = 0.001;


    // set header
    odom_.header.stamp = current_time;
    odom_.header.frame_id = odometry_frame_resolved_;
    odom_.child_frame_id = robot_base_frame_resolved_;

    odometry_publisher_.publish ( odom_ );
}

GZ_REGISTER_MODEL_PLUGIN ( GazeboRosDiffDrive )
}


