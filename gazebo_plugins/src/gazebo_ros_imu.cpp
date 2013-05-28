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

#include <gazebo_plugins/gazebo_ros_imu.h>

namespace gazebo
{

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosIMU::GazeboRosIMU()
{
  this->imu_connect_count_ = 0;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosIMU::~GazeboRosIMU()
{
  event::Events::DisconnectWorldUpdateStart(this->update_connection_);
  // Finalize the controller
  this->rosnode_->shutdown();
  this->callback_queue_thread_.join();
  delete this->rosnode_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosIMU::Load( physics::ModelPtr _parent, sdf::ElementPtr _sdf )
{
  // Get the world name.
  this->world_ = _parent->GetWorld();

  // load parameters
  this->robot_namespace_ = "";
  if (_sdf->HasElement("robotNamespace"))
    this->robot_namespace_ = _sdf->GetElement("robotNamespace")->GetValueString() + "/";

  if (!_sdf->HasElement("serviceName"))
  {
    ROS_INFO("imu plugin missing <serviceName>, defaults to /default_imu");
    this->service_name_ = "/default_imu";
  }
  else
    this->service_name_ = _sdf->GetElement("serviceName")->GetValueString();

  if (!_sdf->HasElement("topicName"))
  {
    ROS_INFO("imu plugin missing <topicName>, defaults to /default_imu");
    this->topic_name_ = "/default_imu";
  }
  else
    this->topic_name_ = _sdf->GetElement("topicName")->GetValueString();

  if (!_sdf->HasElement("gaussianNoise"))
  {
    ROS_INFO("imu plugin missing <gaussianNoise>, defaults to 0.0");
    this->gaussian_noise_ = 0;
  }
  else
    this->gaussian_noise_ = _sdf->GetElement("gaussianNoise")->GetValueDouble();

  if (!_sdf->HasElement("bodyName"))
  {
    ROS_FATAL("imu plugin missing <bodyName>, cannot proceed");
    return;
  }
  else
    this->link_name_ = _sdf->GetElement("bodyName")->GetValueString();

  if (!_sdf->HasElement("xyzOffset"))
  {
    ROS_INFO("imu plugin missing <xyzOffset>, defaults to 0s");
    this->offset_.pos = math::Vector3(0,0,0);
  }
  else
    this->offset_.pos = _sdf->GetElement("xyzOffset")->GetValueVector3();

  if (!_sdf->HasElement("rpyOffset"))
  {
    ROS_INFO("imu plugin missing <rpyOffset>, defaults to 0s");
    this->offset_.rot = math::Vector3(0,0,0);
  }
  else
    this->offset_.rot = _sdf->GetElement("rpyOffset")->GetValueVector3();


  // start ros node
  if (!ros::isInitialized())
  {
    int argc = 0;
    char** argv = NULL;
    ros::init(argc,argv,"gazebo",ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
  }

  this->rosnode_ = new ros::NodeHandle(this->robot_namespace_);

  // assert that the body by link_name_ exists
  this->link = boost::shared_dynamic_cast<physics::Link>(this->world_->GetEntity(this->link_name_));
  if (!this->link)
  {
    ROS_FATAL("gazebo_ros_imu plugin error: bodyName: %s does not exist\n",this->link_name_.c_str());
    return;
  }

  // if topic name specified as empty, do not publish (then what is this plugin good for?)
  if (this->topic_name_ != "")
  {
    ros::AdvertiseOptions ao = ros::AdvertiseOptions::create<sensor_msgs::Imu>(
      this->topic_name_,1,
      boost::bind( &GazeboRosIMU::IMUConnect,this),
      boost::bind( &GazeboRosIMU::IMUDisconnect,this), ros::VoidPtr(), &this->imu_queue_);
    this->pub_ = this->rosnode_->advertise(ao);

    // advertise services on the custom queue
    ros::AdvertiseServiceOptions aso = ros::AdvertiseServiceOptions::create<std_srvs::Empty>(
        this->service_name_,boost::bind( &GazeboRosIMU::ServiceCallback, this, _1, _2 ), ros::VoidPtr(), &this->imu_queue_);
    this->srv_ = this->rosnode_->advertiseService(aso);
  }

  // Initialize the controller
  this->last_time_ = this->world_->GetSimTime();
  //this->initial_pose_ = this->link->GetPose(); // get initial pose of the local link
  this->last_vpos_ = this->link->GetWorldLinearVel(); // get velocity in gazebo frame
  this->last_veul_ = this->link->GetWorldAngularVel(); // get velocity in gazebo frame
  this->apos_ = 0;
  this->aeul_ = 0;

  // start custom queue for imu
  this->callback_queue_thread_ = boost::thread( boost::bind( &GazeboRosIMU::IMUQueueThread,this ) );


  // New Mechanism for Updating every World Cycle
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->update_connection_ = event::Events::ConnectWorldUpdateStart(
      boost::bind(&GazeboRosIMU::UpdateChild, this));
}

////////////////////////////////////////////////////////////////////////////////
// returns true always, imu is always calibrated in sim
bool GazeboRosIMU::ServiceCallback(std_srvs::Empty::Request &req,
                                        std_srvs::Empty::Response &res)
{
  return true;
}

////////////////////////////////////////////////////////////////////////////////
// Increment count
void GazeboRosIMU::IMUConnect()
{
  this->imu_connect_count_++;
}
////////////////////////////////////////////////////////////////////////////////
// Decrement count
void GazeboRosIMU::IMUDisconnect()
{
  this->imu_connect_count_--;
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosIMU::UpdateChild()
{
  if ((this->imu_connect_count_ > 0 && this->topic_name_ != ""))
  {
    math::Pose pose;
    math::Quaternion rot;
    math::Vector3 pos;

    // Get Pose/Orientation ///@todo: verify correctness
    pose = this->link->GetWorldPose(); // - this->link->GetCoMPose();
    // apply xyz offsets and get position and rotation components
    pos = pose.pos + this->offset_.pos;
    rot = pose.rot;
    // std::cout << " --------- GazeboRosIMU rot " << rot.x << ", " << rot.y << ", " << rot.z << ", " << rot.u << std::endl;

    // apply rpy offsets
    rot = this->offset_.rot*rot;
    rot.Normalize();

    common::Time cur_time = this->world_->GetSimTime();
    
    // get Rates
    math::Vector3 vpos = this->link->GetWorldLinearVel(); // get velocity in gazebo frame
    math::Vector3 veul = this->link->GetWorldAngularVel(); // get velocity in gazebo frame

    // differentiate to get accelerations
    double tmp_dt = this->last_time_.Double() - cur_time.Double();
    if (tmp_dt != 0)
    {
      this->apos_ = (this->last_vpos_ - vpos) / tmp_dt;
      this->aeul_ = (this->last_veul_ - veul) / tmp_dt;
      this->last_vpos_ = vpos;
      this->last_veul_ = veul;
    }

    // copy data into pose message
    this->imu_msg_.header.frame_id = this->link_name_;
    this->imu_msg_.header.stamp.sec = cur_time.sec;
    this->imu_msg_.header.stamp.nsec = cur_time.nsec;

    // orientation quaternion

    // uncomment this if we are reporting orientation in the local frame
    // not the case for our imu definition
    // // apply fixed orientation offsets of initial pose
    // rot = this->initial_pose_.rot*rot;
    // rot.Normalize();

    this->imu_msg_.orientation.x = rot.x;
    this->imu_msg_.orientation.y = rot.y;
    this->imu_msg_.orientation.z = rot.z;
    this->imu_msg_.orientation.w = rot.w;

    // pass euler angular rates
    math::Vector3 linear_velocity(veul.x + this->GaussianKernel(0,this->gaussian_noise_)
                           ,veul.y + this->GaussianKernel(0,this->gaussian_noise_)
                           ,veul.z + this->GaussianKernel(0,this->gaussian_noise_));
    // rotate into local frame
    // @todo: deal with offsets!
    linear_velocity = rot.RotateVector(linear_velocity);
    this->imu_msg_.angular_velocity.x    = linear_velocity.x;
    this->imu_msg_.angular_velocity.y    = linear_velocity.y;
    this->imu_msg_.angular_velocity.z    = linear_velocity.z;

    // pass accelerations
    math::Vector3 linear_acceleration(apos_.x + this->GaussianKernel(0,this->gaussian_noise_)
                                     ,apos_.y + this->GaussianKernel(0,this->gaussian_noise_)
                                     ,apos_.z + this->GaussianKernel(0,this->gaussian_noise_));
    // rotate into local frame
    // @todo: deal with offsets!
    linear_acceleration = rot.RotateVector(linear_acceleration);
    this->imu_msg_.linear_acceleration.x    = linear_acceleration.x;
    this->imu_msg_.linear_acceleration.y    = linear_acceleration.y;
    this->imu_msg_.linear_acceleration.z    = linear_acceleration.z;

    // fill in covariance matrix
    /// @todo: let user set separate linear and angular covariance values.
    /// @todo: apply appropriate rotations from frame_pose
    double gn2 = this->gaussian_noise_*this->gaussian_noise_;
    this->imu_msg_.orientation_covariance[0] = gn2;
    this->imu_msg_.orientation_covariance[4] = gn2;
    this->imu_msg_.orientation_covariance[8] = gn2;
    this->imu_msg_.angular_velocity_covariance[0] = gn2;
    this->imu_msg_.angular_velocity_covariance[4] = gn2;
    this->imu_msg_.angular_velocity_covariance[8] = gn2;
    this->imu_msg_.linear_acceleration_covariance[0] = gn2;
    this->imu_msg_.linear_acceleration_covariance[4] = gn2;
    this->imu_msg_.linear_acceleration_covariance[8] = gn2;

    this->lock_.lock();
    // publish to ros
    if (this->imu_connect_count_ > 0 && this->topic_name_ != "")
        this->pub_.publish(this->imu_msg_);
    this->lock_.unlock();

    // save last time stamp
    this->last_time_ = cur_time;
  }
}


//////////////////////////////////////////////////////////////////////////////
// Utility for adding noise
double GazeboRosIMU::GaussianKernel(double mu,double sigma)
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
void GazeboRosIMU::IMUQueueThread()
{
  static const double timeout = 0.01;

  while (this->rosnode_->ok())
  {
    this->imu_queue_.callAvailable(ros::WallDuration(timeout));
  }
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboRosIMU)
}
