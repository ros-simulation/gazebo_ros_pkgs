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
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/Odometry.h>
#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>

namespace gazebo {

enum {
    RIGHT,
    LEFT,
};

GazeboRosDiffDrive::GazeboRosDiffDrive() {}

// Destructor
GazeboRosDiffDrive::~GazeboRosDiffDrive() {
}

// Load the controller
void GazeboRosDiffDrive::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {

    this->parent = _parent;
    this->world = _parent->GetWorld();


    this->robot_namespace_ = this->parent->GetName ();
    if ( !_sdf->HasElement ( "robotNamespace" ) ) {
        ROS_INFO ( "GazeboRosDiffDrive Plugin missing <robotNamespace>, defaults to \"%s\"",
                   this->robot_namespace_.c_str() );
    } else {
        this->robot_namespace_ = _sdf->GetElement ( "robotNamespace" )->Get<std::string>();
        if ( this->robot_namespace_.empty() ) this->robot_namespace_ = this->parent->GetName ();
    }
    if ( !robot_namespace_.empty() ) this->robot_namespace_ += "/";
    rosnode_ = boost::shared_ptr<ros::NodeHandle> ( new ros::NodeHandle ( this->robot_namespace_ ) );

    tf_prefix_ = tf::getPrefixParam(*rosnode_);
    if(tf_prefix_.empty()) {
        tf_prefix_ = robot_namespace_;
        boost::trim_right_if(tf_prefix_,boost::is_any_of("/"));
    }
    ROS_INFO("GazeboRosDiffDrive Plugin (ns = %s)  <tf_prefix_>, set to \"%s\"",
             this->robot_namespace_.c_str(), this->tf_prefix_.c_str());


    this->left_joint_name_ = "left_joint";
    if (!_sdf->HasElement("leftJoint")) {
        ROS_WARN("GazeboRosDiffDrive Plugin (ns = %s) missing <leftJoint>, defaults to \"%s\"",
                 this->robot_namespace_.c_str(), this->left_joint_name_.c_str());
    } else {
        this->left_joint_name_ = _sdf->GetElement("leftJoint")->Get<std::string>();
    }

    this->right_joint_name_ = "right_joint";
    if (!_sdf->HasElement("rightJoint")) {
        ROS_WARN("GazeboRosDiffDrive Plugin (ns = %s) missing <rightJoint>, defaults to \"%s\"",
                 this->robot_namespace_.c_str(), this->right_joint_name_.c_str());
    } else {
        this->right_joint_name_ = _sdf->GetElement("rightJoint")->Get<std::string>();
    }

    this->wheel_separation_ = 0.34;
    if (!_sdf->HasElement("wheelSeparation")) {
        ROS_WARN("GazeboRosDiffDrive Plugin (ns = %s) missing <wheelSeparation>, defaults to %f",
                 this->robot_namespace_.c_str(), this->wheel_separation_);
    } else {
        this->wheel_separation_ =
            _sdf->GetElement("wheelSeparation")->Get<double>();
    }

    this->wheel_diameter_ = 0.15;
    if (!_sdf->HasElement("wheelDiameter")) {
        ROS_WARN("GazeboRosDiffDrive Plugin (ns = %s) missing <wheelDiameter>, defaults to %f",
                 this->robot_namespace_.c_str(), this->wheel_diameter_);
    } else {
        this->wheel_diameter_ = _sdf->GetElement("wheelDiameter")->Get<double>();
    }

    this->torque = 5.0;
    if (!_sdf->HasElement("torque")) {
        ROS_WARN("GazeboRosDiffDrive Plugin (ns = %s) missing <torque>, defaults to %f",
                 this->robot_namespace_.c_str(), this->torque);
    } else {
        this->torque = _sdf->GetElement("torque")->Get<double>();
    }

    this->command_topic_ = "cmd_vel";
    if (!_sdf->HasElement("commandTopic")) {
        ROS_WARN("GazeboRosDiffDrive Plugin (ns = %s) missing <commandTopic>, defaults to \"%s\"",
                 this->robot_namespace_.c_str(), this->command_topic_.c_str());
    } else {
        this->command_topic_ = _sdf->GetElement("commandTopic")->Get<std::string>();
    }

    this->odometry_topic_ = "odom";
    if (!_sdf->HasElement("odometryTopic")) {
        ROS_WARN("GazeboRosDiffDrive Plugin (ns = %s) missing <odometryTopic>, defaults to \"%s\"",
                 this->robot_namespace_.c_str(), this->odometry_topic_.c_str());
    } else {
        this->odometry_topic_ = _sdf->GetElement("odometryTopic")->Get<std::string>();
    }

    this->odometry_frame_ = "odom";
    if (!_sdf->HasElement("odometryFrame")) {
        ROS_WARN("GazeboRosDiffDrive Plugin (ns = %s) missing <odometryFrame>, defaults to \"%s\"",
                 this->robot_namespace_.c_str(), this->odometry_frame_.c_str());
    } else {
        this->odometry_frame_ = _sdf->GetElement("odometryFrame")->Get<std::string>();
    }

    this->robot_base_frame_ = "base_footprint";
    if (!_sdf->HasElement("robotBaseFrame")) {
        ROS_WARN("GazeboRosDiffDrive Plugin (ns = %s) missing <robotBaseFrame>, defaults to \"%s\"",
                 this->robot_namespace_.c_str(), this->robot_base_frame_.c_str());
    } else {
        this->robot_base_frame_ = _sdf->GetElement("robotBaseFrame")->Get<std::string>();
    }

    this->update_rate_ = 100.0;
    if (!_sdf->HasElement("updateRate")) {
        ROS_WARN("GazeboRosDiffDrive Plugin (ns = %s) missing <updateRate>, defaults to %f",
                 this->robot_namespace_.c_str(), this->update_rate_);
    } else {
        this->update_rate_ = _sdf->GetElement("updateRate")->Get<double>();
    }


    this->publishWheelTF_ = false;
    if (_sdf->HasElement("publishWheelTF")) {

        std::string value = _sdf->GetElement("publishWheelTF")->Get<std::string>();
        if(boost::iequals(value, std::string("true"))) {
            this->publishWheelTF_ = true;
        } else if(boost::iequals(value, std::string("false"))) {
            this->publishWheelTF_ = false;
        } else {
            ROS_WARN("GazeboRosDiffDrive Plugin (ns = %s)  <publishWheelTF> was set to '%s', it must be set to 'true' or 'false'!",
                     this->robot_namespace_.c_str(), value.c_str());
        }
    }
    ROS_INFO("GazeboRosDiffDrive Plugin (ns = %s) <publishWheelTF>, set to %s",
             this->robot_namespace_.c_str(), (this->publishWheelTF_?"ture":"false"));

    this->publishWheelJointState_ = false;
    if (_sdf->HasElement("publishWheelJointState")) {
        std::string value = _sdf->GetElement("publishWheelJointState")->Get<std::string>();
        if(boost::iequals(value, std::string("true"))) {
            this->publishWheelJointState_ = true;
        } else if(boost::iequals(value, std::string("false"))) {
            this->publishWheelJointState_ = false;
        } else {
            ROS_WARN("GazeboRosDiffDrive Plugin (ns = %s)  <publishWheelTF> was set to '%s', it must be set to 'true' or 'false'!",
                     this->robot_namespace_.c_str(), value.c_str());
        }
    }
    this->max_accel=0;
    if (!_sdf->HasElement("max_accel")) {
        ROS_WARN("GazeboRosDiffDrive Plugin (ns = %s) missing <max_accel>, defaults to %f",
                 this->robot_namespace_.c_str(), this->max_accel);
    } else {
        this->max_accel = _sdf->GetElement("max_accel")->Get<double>();
    }

    ROS_INFO("GazeboRosDiffDrive Plugin (ns = %s) <publishWheelJointState>, set to %s",
             this->robot_namespace_.c_str(), (this->publishWheelJointState_?"ture":"false"));

    // Initialize update rate stuff
    if (this->update_rate_ > 0.0) {
        this->update_period_ = 1.0 / this->update_rate_;
    } else {
        this->update_period_ = 0.0;
    }
    last_update_time_ = this->world->GetSimTime();

    // Initialize velocity stuff
    wheel_speed_[RIGHT] = 0;
    wheel_speed_[LEFT] = 0;

    x_ = 0;
    rot_ = 0;
    alive_ = true;

    joints_.resize(2);
    joints_[LEFT] = this->parent->GetJoint(left_joint_name_);
    joints_[RIGHT] = this->parent->GetJoint(right_joint_name_);

    if (!joints_[LEFT]) {
        char error[200];
        snprintf(error, 200,
                 "GazeboRosDiffDrive Plugin (ns = %s) couldn't get left hinge joint named \"%s\"",
                 this->robot_namespace_.c_str(), this->left_joint_name_.c_str());
        gzthrow(error);
    }
    if (!joints_[RIGHT]) {
        char error[200];
        snprintf(error, 200,
                 "GazeboRosDiffDrive Plugin (ns = %s) couldn't get right hinge joint named \"%s\"",
                 this->robot_namespace_.c_str(), this->right_joint_name_.c_str());
        gzthrow(error);
    }

    joints_[LEFT]->SetMaxForce(0, torque);
    joints_[RIGHT]->SetMaxForce(0, torque);

    // Make sure the ROS node for Gazebo has already been initialized
    if (!ros::isInitialized())
    {
        ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                         << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
        return;
    }

    rosnode_ = boost::shared_ptr<ros::NodeHandle> ( new ros::NodeHandle ( this->robot_namespace_ ) );
    ROS_INFO("Starting GazeboRosDiffDrive Plugin (ns = %s)!", this->robot_namespace_.c_str());

    if(this->publishWheelJointState_) {
        joint_state_publisher_ = rosnode_->advertise<sensor_msgs::JointState> ( "joint_states",1000 );
    }

    transform_broadcaster_ = boost::shared_ptr<tf::TransformBroadcaster> ( new tf::TransformBroadcaster() );

    // ROS: Subscribe to the velocity command topic (usually "cmd_vel")
    ros::SubscribeOptions so =
        ros::SubscribeOptions::create<geometry_msgs::Twist>(command_topic_, 1,
                boost::bind(&GazeboRosDiffDrive::cmdVelCallback, this, _1),
                ros::VoidPtr(), &queue_);

    cmd_vel_subscriber_ = rosnode_->subscribe(so);

    odometry_publisher_ = rosnode_->advertise<nav_msgs::Odometry>(odometry_topic_, 1);

    // start custom queue for diff drive
    this->callback_queue_thread_ =
        boost::thread(boost::bind(&GazeboRosDiffDrive::QueueThread, this));

    // listen to the update event (broadcast every simulation iteration)
    this->update_connection_ =
        event::Events::ConnectWorldUpdateBegin(
            boost::bind(&GazeboRosDiffDrive::UpdateChild, this));

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
    for(int i = 0; i < 2; i++) {
        std::string wheel_frame =
            tf::resolve(tf_prefix_, joints_[i]->GetChild()->GetName ());
        std::string wheel_parent_frame =
            tf::resolve(tf_prefix_, joints_[i]->GetParent()->GetName ());

        math::Pose poseWheel = joints_[i]->GetChild()->GetRelativePose();

        tf::Quaternion qt(poseWheel.rot.x, poseWheel.rot.y, poseWheel.rot.z, poseWheel.rot.w);
        tf::Vector3 vt(poseWheel.pos.x, poseWheel.pos.y, poseWheel.pos.z);

        tf::Transform tfWheel(qt, vt);
        transform_broadcaster_->sendTransform(
            tf::StampedTransform(tfWheel, current_time,
                                 wheel_parent_frame, wheel_frame));
    }
}

// Update the controller
void GazeboRosDiffDrive::UpdateChild() {
    common::Time current_time = this->world->GetSimTime();
    double seconds_since_last_update =
        (current_time - last_update_time_).Double();
    if (seconds_since_last_update > update_period_) {

        publishOdometry(seconds_since_last_update);
        if(publishWheelTF_) publishWheelTF();
        if(publishWheelJointState_) publishWheelJointState();

        // Update robot in case new velocities have been requested
        getWheelVelocities();


        double current_speed[2];

        current_speed[LEFT] = joints_[LEFT]->GetVelocity(0)   * (wheel_diameter_ / 2.0);
        current_speed[RIGHT] = joints_[RIGHT]->GetVelocity(0) * (wheel_diameter_ / 2.0);

        if(max_accel==0
                || (fabs(wheel_speed_[LEFT] - current_speed[LEFT]) < 0.01)
                || (fabs(wheel_speed_[RIGHT] - current_speed[RIGHT]) < 0.01)) {
            //if max_accel==0, or target speed is reached
            joints_[LEFT]->SetVelocity(0, wheel_speed_[LEFT]/ (wheel_diameter_ / 2.0));
            joints_[RIGHT]->SetVelocity(0, wheel_speed_[RIGHT]/ (wheel_diameter_ / 2.0));
        }
        else {

            if(wheel_speed_[LEFT]>=current_speed[LEFT])
                wheel_speed_instr_[LEFT]+=fmin(wheel_speed_[LEFT]-current_speed[LEFT],  max_accel * seconds_since_last_update);
            else
                wheel_speed_instr_[LEFT]+=fmax(wheel_speed_[LEFT]-current_speed[LEFT], -max_accel * seconds_since_last_update);

            if(wheel_speed_[RIGHT]>current_speed[RIGHT])
                wheel_speed_instr_[RIGHT]+=fmin(wheel_speed_[RIGHT]-current_speed[RIGHT], max_accel * seconds_since_last_update);
            else
                wheel_speed_instr_[RIGHT]+=fmax(wheel_speed_[RIGHT]-current_speed[RIGHT], -max_accel * seconds_since_last_update);

            // ROS_INFO("actual wheel speed = %lf, issued wheel speed= %lf", current_speed[LEFT], wheel_speed_[LEFT]);
            // ROS_INFO("actual wheel speed = %lf, issued wheel speed= %lf", current_speed[RIGHT],wheel_speed_[RIGHT]);

            joints_[LEFT]->SetVelocity(0,wheel_speed_instr_[LEFT] / (wheel_diameter_ / 2.0));
            joints_[RIGHT]->SetVelocity(0,wheel_speed_instr_[RIGHT] / (wheel_diameter_ / 2.0));
        }

        last_update_time_+= common::Time(update_period_);

    }
}

// Finalize the controller
void GazeboRosDiffDrive::FiniChild() {
    alive_ = false;
    queue_.clear();
    queue_.disable();
    rosnode_->shutdown();
    callback_queue_thread_.join();
}

void GazeboRosDiffDrive::getWheelVelocities() {
    boost::mutex::scoped_lock scoped_lock(lock);

    double vr = x_;
    double va = rot_;

    wheel_speed_[LEFT] = vr + va * wheel_separation_ / 2.0;
    wheel_speed_[RIGHT] = vr - va * wheel_separation_ / 2.0;
}

void GazeboRosDiffDrive::cmdVelCallback(
    const geometry_msgs::Twist::ConstPtr& cmd_msg) {

    boost::mutex::scoped_lock scoped_lock(lock);
    x_ = cmd_msg->linear.x;
    rot_ = cmd_msg->angular.z;
}

void GazeboRosDiffDrive::QueueThread() {
    static const double timeout = 0.01;

    while (alive_ && rosnode_->ok()) {
        queue_.callAvailable(ros::WallDuration(timeout));
    }
}

void GazeboRosDiffDrive::publishOdometry(double step_time) {
    ros::Time current_time = ros::Time::now();
    std::string odom_frame =
        tf::resolve(tf_prefix_, odometry_frame_);
    std::string base_footprint_frame =
        tf::resolve(tf_prefix_, robot_base_frame_);

    // getting data for base_footprint to odom transform
    math::Pose pose = this->parent->GetWorldPose();

    tf::Quaternion qt(pose.rot.x, pose.rot.y, pose.rot.z, pose.rot.w);
    tf::Vector3 vt(pose.pos.x, pose.pos.y, pose.pos.z);

    tf::Transform base_footprint_to_odom(qt, vt);
    transform_broadcaster_->sendTransform(
        tf::StampedTransform(base_footprint_to_odom, current_time,
                             odom_frame, base_footprint_frame));

    // publish odom topic
    odom_.pose.pose.position.x = pose.pos.x;
    odom_.pose.pose.position.y = pose.pos.y;

    odom_.pose.pose.orientation.x = pose.rot.x;
    odom_.pose.pose.orientation.y = pose.rot.y;
    odom_.pose.pose.orientation.z = pose.rot.z;
    odom_.pose.pose.orientation.w = pose.rot.w;
    odom_.pose.covariance[0] = 0.00001;
    odom_.pose.covariance[7] = 0.00001;
    odom_.pose.covariance[14] = 1000000000000.0;
    odom_.pose.covariance[21] = 1000000000000.0;
    odom_.pose.covariance[28] = 1000000000000.0;
    odom_.pose.covariance[35] = 0.001;

    // get velocity in /odom frame
    math::Vector3 linear;
    linear = this->parent->GetWorldLinearVel();
    odom_.twist.twist.angular.z = this->parent->GetWorldAngularVel().z;

    // convert velocity to child_frame_id (aka base_footprint)
    float yaw = pose.rot.GetYaw();
    odom_.twist.twist.linear.x = cosf(yaw) * linear.x + sinf(yaw) * linear.y;
    odom_.twist.twist.linear.y = cosf(yaw) * linear.y - sinf(yaw) * linear.x;

    odom_.header.stamp = current_time;
    odom_.header.frame_id = odom_frame;
    odom_.child_frame_id = base_footprint_frame;

    odometry_publisher_.publish(odom_);
}

GZ_REGISTER_MODEL_PLUGIN(GazeboRosDiffDrive)
}

