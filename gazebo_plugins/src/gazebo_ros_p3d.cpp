/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003
 *     Nate Koenig & Andrew Howard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/*
 * Desc: 3D position interface for ground truth.
 * Author: Sachin Chitta and John Hsu
 * Date: 1 June 2008
 * SVN info: $Id$
 */

#include <gazebo_plugins/gazebo_ros_p3d.h>

#include "tf/tf.h"

namespace gazebo
{


////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosP3D::GazeboRosP3D()
{
  this->p3d_connect_count_ = 0;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosP3D::~GazeboRosP3D()
{
  event::Events::DisconnectWorldUpdateBegin(this->update_connection_);
  // Finalize the controller
  this->rosnode_->shutdown();
  this->p3d_queue_.clear();
  this->p3d_queue_.disable();
  this->callback_queue_thread_.join();
  delete this->rosnode_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosP3D::Load( physics::ModelPtr _parent, sdf::ElementPtr _sdf )
{
  // Get the world name.
  this->world_ = _parent->GetWorld();
  this->model_ = _parent;

  // load parameters
  this->robot_namespace_ = "";
  if (_sdf->HasElement("robotNamespace"))
    this->robot_namespace_ = _sdf->GetElement("robotNamespace")->GetValueString() + "/";

  if (!_sdf->HasElement("bodyName"))
  {
    ROS_FATAL("p3d plugin missing <bodyName>, cannot proceed");
    return;
  }
  else
    this->link_name_ = _sdf->GetElement("bodyName")->GetValueString();

  this->link_ = _parent->GetLink(this->link_name_);
  if (!this->link_)
  {
    ROS_FATAL("gazebo_ros_p3d plugin error: bodyName: %s does not exist\n",this->link_name_.c_str());
    return;
  }

  if (!_sdf->HasElement("topicName"))
  {
    ROS_FATAL("p3d plugin missing <topicName>, cannot proceed");
    return;
  }
  else
    this->topic_name_ = _sdf->GetElement("topicName")->GetValueString();

  if (!_sdf->HasElement("frameName"))
  {
    ROS_INFO("p3d plugin missing <frameName>, defaults to world");
    this->frame_name_ = "world";
  }
  else
    this->frame_name_ = _sdf->GetElement("frameName")->GetValueString();

  if (!_sdf->HasElement("xyzOffset"))
  {
    ROS_INFO("p3d plugin missing <xyzOffset>, defaults to 0s");
    this->offset_.pos = math::Vector3(0,0,0);
  }
  else
    this->offset_.pos = _sdf->GetElement("xyzOffset")->GetValueVector3();

  if (!_sdf->HasElement("rpyOffset"))
  {
    ROS_INFO("p3d plugin missing <rpyOffset>, defaults to 0s");
    this->offset_.rot = math::Vector3(0,0,0);
  }
  else
    this->offset_.rot = _sdf->GetElement("rpyOffset")->GetValueVector3();

  if (!_sdf->HasElement("gaussianNoise"))
  {
    ROS_INFO("p3d plugin missing <gaussianNoise>, defaults to 0.0");
    this->gaussian_noise_ = 0;
  }
  else
    this->gaussian_noise_ = _sdf->GetElement("gaussianNoise")->GetValueDouble();

  if (!_sdf->HasElement("updateRate"))
  {
    ROS_INFO("p3d plugin missing <updateRate>, defaults to 0.0 (as fast as possible)");
    this->update_rate_ = 0;
  }
  else
    this->update_rate_ = _sdf->GetElement("updateRate")->GetValueDouble();

  if (!ros::isInitialized())
  {
    int argc = 0;
    char** argv = NULL;
    ros::init(argc,argv,"gazebo",ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
  }

  this->rosnode_ = new ros::NodeHandle(this->robot_namespace_);

  // resolve tf prefix
  std::string prefix;
  this->rosnode_->getParam(std::string("tf_prefix"), prefix);
  this->tf_frame_name_ = tf::resolve(prefix, this->frame_name_);

  if (this->topic_name_ != "")
  {
    ros::AdvertiseOptions p3d_ao = ros::AdvertiseOptions::create<nav_msgs::Odometry>(
    this->topic_name_,1,
    boost::bind( &GazeboRosP3D::P3DConnect,this),
    boost::bind( &GazeboRosP3D::P3DDisconnect,this), ros::VoidPtr(), &this->p3d_queue_);
    this->pub_ = this->rosnode_->advertise(p3d_ao);
  }
  
  this->last_time_ = this->world_->GetSimTime();
  // initialize body
  this->last_vpos_ = this->link_->GetWorldLinearVel(); // get velocity in gazebo frame
  this->last_veul_ = this->link_->GetWorldAngularVel(); // get velocity in gazebo frame
  this->apos_ = 0;
  this->aeul_ = 0;

  /// if frameName specified is "world", "/map" or "map" report back inertial values in the gazebo world
  if (this->frame_name_ != "world" && this->frame_name_ != "/map" && this->frame_name_ != "map")
  {
    this->reference_link_ = this->model_->GetLink(this->frame_name_);
    if (!this->reference_link_)
    {
      ROS_ERROR("gazebo_ros_p3d plugin: frameName: %s does not exist, will not publish pose\n",this->frame_name_.c_str());
      return;
    }
  }

  // init reference frame state
  if (this->reference_link_)
  {
    ROS_DEBUG("got body %s",this->reference_link_->GetName().c_str());
    this->frame_apos_ = 0;
    this->frame_aeul_ = 0;
    this->last_frame_vpos_ = this->reference_link_->GetWorldLinearVel(); // get velocity in gazebo frame
    this->last_frame_veul_ = this->reference_link_->GetWorldAngularVel(); // get velocity in gazebo frame
  }


  // start custom queue for p3d
  this->callback_queue_thread_ = boost::thread( boost::bind( &GazeboRosP3D::P3DQueueThread,this ) );
  
  // New Mechanism for Updating every World Cycle
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GazeboRosP3D::UpdateChild, this));
}

////////////////////////////////////////////////////////////////////////////////
// Increment count
void GazeboRosP3D::P3DConnect()
{
  this->p3d_connect_count_++;
}
////////////////////////////////////////////////////////////////////////////////
// Decrement count
void GazeboRosP3D::P3DDisconnect()
{
  this->p3d_connect_count_--;
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosP3D::UpdateChild()
{
  if (!this->link_)
    return;

  common::Time cur_time = this->world_->GetSimTime();

  // rate control
  if (this->update_rate_ > 0 && (cur_time-this->last_time_).Double() < (1.0/this->update_rate_))
    return;

  if (this->p3d_connect_count_ > 0)
  {


    // differentiate to get accelerations
    double tmp_dt = cur_time.Double() - this->last_time_.Double();
    if (tmp_dt != 0)
    {
      this->lock.lock();

      if (this->topic_name_ != "")
      {
        // copy data into pose message
        this->pose_msg_.header.frame_id = this->tf_frame_name_;  // @todo: should this be changeable?
        this->pose_msg_.header.stamp.sec = cur_time.sec;
        this->pose_msg_.header.stamp.nsec = cur_time.nsec;


        math::Pose pose, frame_pose;
        math::Vector3 frame_vpos;
        math::Vector3 frame_veul;

        // get inertial Rates
        math::Vector3 vpos = this->link_->GetWorldLinearVel(); // get velocity in gazebo frame
        math::Vector3 veul = this->link_->GetWorldAngularVel(); // get velocity in gazebo frame

        // Get Pose/Orientation
        pose = this->link_->GetWorldPose(); // - this->link_->GetCoMPose();

        // Apply Reference Frame
        if (this->reference_link_)
        {
          // convert to relative pose
          frame_pose = this->reference_link_->GetWorldPose(); // - this->link_->GetCoMPose();
          pose.pos = pose.pos - frame_pose.pos;
          pose.pos = frame_pose.rot.RotateVectorReverse(pose.pos);
          pose.rot *= frame_pose.rot.GetInverse();
          // convert to relative rates
          frame_vpos = this->reference_link_->GetWorldLinearVel(); // get velocity in gazebo frame
          frame_veul = this->reference_link_->GetWorldAngularVel(); // get velocity in gazebo frame
          vpos = frame_pose.rot.RotateVector(vpos - frame_vpos);
          veul = frame_pose.rot.RotateVector(veul - frame_veul);
        }

        // Apply Constant Offsets
        // apply xyz offsets and get position and rotation components
        pose.pos = pose.pos + this->offset_.pos;
        // apply rpy offsets
        pose.rot = this->offset_.rot*pose.rot;
        pose.rot.Normalize();

        // compute accelerations (not used)
        this->apos_ = (this->last_vpos_ - vpos) / tmp_dt;
        this->aeul_ = (this->last_veul_ - veul) / tmp_dt;
        this->last_vpos_ = vpos;
        this->last_veul_ = veul;

        this->frame_apos_ = (this->last_frame_vpos_ - frame_vpos) / tmp_dt;
        this->frame_aeul_ = (this->last_frame_veul_ - frame_veul) / tmp_dt;
        this->last_frame_vpos_ = frame_vpos;
        this->last_frame_veul_ = frame_veul;

        // Fill out messages
        this->pose_msg_.pose.pose.position.x    = pose.pos.x;
        this->pose_msg_.pose.pose.position.y    = pose.pos.y;
        this->pose_msg_.pose.pose.position.z    = pose.pos.z;

        this->pose_msg_.pose.pose.orientation.x = pose.rot.x;
        this->pose_msg_.pose.pose.orientation.y = pose.rot.y;
        this->pose_msg_.pose.pose.orientation.z = pose.rot.z;
        this->pose_msg_.pose.pose.orientation.w = pose.rot.w;

        this->pose_msg_.twist.twist.linear.x  = vpos.x + this->GaussianKernel(0,this->gaussian_noise_) ;
        this->pose_msg_.twist.twist.linear.y  = vpos.y + this->GaussianKernel(0,this->gaussian_noise_) ;
        this->pose_msg_.twist.twist.linear.z  = vpos.z + this->GaussianKernel(0,this->gaussian_noise_) ;
        // pass euler angular rates
        this->pose_msg_.twist.twist.angular.x = veul.x + this->GaussianKernel(0,this->gaussian_noise_) ;
        this->pose_msg_.twist.twist.angular.y = veul.y + this->GaussianKernel(0,this->gaussian_noise_) ;
        this->pose_msg_.twist.twist.angular.z = veul.z + this->GaussianKernel(0,this->gaussian_noise_) ;

        // fill in covariance matrix
        /// @todo: let user set separate linear and angular covariance values.
        double gn2 = this->gaussian_noise_*this->gaussian_noise_;
        this->pose_msg_.pose.covariance[0] = gn2;
        this->pose_msg_.pose.covariance[7] = gn2;
        this->pose_msg_.pose.covariance[14] = gn2;
        this->pose_msg_.pose.covariance[21] = gn2;
        this->pose_msg_.pose.covariance[28] = gn2;
        this->pose_msg_.pose.covariance[35] = gn2;

        this->pose_msg_.twist.covariance[0] = gn2;
        this->pose_msg_.twist.covariance[7] = gn2;
        this->pose_msg_.twist.covariance[14] = gn2;
        this->pose_msg_.twist.covariance[21] = gn2;
        this->pose_msg_.twist.covariance[28] = gn2;
        this->pose_msg_.twist.covariance[35] = gn2;

        // publish to ros
        this->pub_.publish(this->pose_msg_);
      }

      this->lock.unlock();

      // save last time stamp
      this->last_time_ = cur_time;
    }
  }
}

//////////////////////////////////////////////////////////////////////////////
// Utility for adding noise
double GazeboRosP3D::GaussianKernel(double mu,double sigma)
{
  // using Box-Muller transform to generate two independent standard normally disbributed normal variables
  // see wikipedia
  double U = (double)rand()/(double)RAND_MAX; // normalized uniform random variable
  double V = (double)rand()/(double)RAND_MAX; // normalized uniform random variable
  double X = sqrt(-2.0 * ::log(U)) * cos( 2.0*M_PI * V);
  //double Y = sqrt(-2.0 * ::log(U)) * sin( 2.0*M_PI * V); // the other indep. normal variable
  // we'll just use X
  // scale to our mu and sigma
  X = sigma * X + mu;
  return X;
}

////////////////////////////////////////////////////////////////////////////////
// Put laser data to the interface
void GazeboRosP3D::P3DQueueThread()
{
  static const double timeout = 0.01;

  while (this->rosnode_->ok())
  {
    this->p3d_queue_.callAvailable(ros::WallDuration(timeout));
  }
}

GZ_REGISTER_MODEL_PLUGIN(GazeboRosP3D);

}
