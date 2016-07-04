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
/*
 * Desc: 3D position interface for ground truth.
 * Author: Sachin Chitta and John Hsu
 * Date: 1 June 2008
 */

#include <gazebo_plugins/gazebo_ros_imu.h>

namespace gazebo
{
// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboRosIMU)

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosIMU::GazeboRosIMU()
{
  this->seed = 0;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosIMU::~GazeboRosIMU()
{
  event::Events::DisconnectWorldUpdateBegin(this->update_connection_);
  // Finalize the controller
  this->rosnode_->shutdown();
  this->callback_queue_thread_.join();
  delete this->rosnode_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosIMU::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  // save pointers
  this->world_ = _parent->GetWorld();
  this->sdf = _sdf;

  // ros callback queue for processing subscription
  this->deferred_load_thread_ = boost::thread(
    boost::bind(&GazeboRosIMU::LoadThread, this));
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosIMU::LoadThread()
{
  // load parameters
  this->robot_namespace_ = "";
  if (this->sdf->HasElement("robotNamespace"))
    this->robot_namespace_ = this->sdf->Get<std::string>("robotNamespace") + "/";

  if (!this->sdf->HasElement("serviceName"))
  {
    ROS_INFO("imu plugin missing <serviceName>, defaults to /default_imu");
    this->service_name_ = "/default_imu";
  }
  else
    this->service_name_ = this->sdf->Get<std::string>("serviceName");

  if (!this->sdf->HasElement("topicName"))
  {
    ROS_INFO("imu plugin missing <topicName>, defaults to /default_imu");
    this->topic_name_ = "/default_imu";
  }
  else
    this->topic_name_ = this->sdf->Get<std::string>("topicName");

  if (!this->sdf->HasElement("gaussianNoise"))
  {
    ROS_INFO("imu plugin missing <gaussianNoise>, defaults to 0.0");
    this->gaussian_noise_ = 0.0;
  }
  else
    this->gaussian_noise_ = this->sdf->Get<double>("gaussianNoise");

  if (!this->sdf->HasElement("bodyName"))
  {
    ROS_FATAL("imu plugin missing <bodyName>, cannot proceed");
    return;
  }
  else
    this->link_name_ = this->sdf->Get<std::string>("bodyName");

  if (!this->sdf->HasElement("xyzOffset"))
  {
    ROS_INFO("imu plugin missing <xyzOffset>, defaults to 0s");
    this->offset_.pos = math::Vector3(0, 0, 0);
  }
  else
    this->offset_.pos = this->sdf->Get<math::Vector3>("xyzOffset");

  if (!this->sdf->HasElement("rpyOffset"))
  {
    ROS_INFO("imu plugin missing <rpyOffset>, defaults to 0s");
    this->offset_.rot = math::Vector3(0, 0, 0);
  }
  else
    this->offset_.rot = this->sdf->Get<math::Vector3>("rpyOffset");
  
  if (!this->sdf->HasElement("updateRate"))
  {
    ROS_DEBUG("imu plugin missing <updateRate>, defaults to 0.0"
             " (as fast as possible)");
    this->update_rate_ = 0.0;
  }
  else
    this->update_rate_ = this->sdf->GetElement("updateRate")->Get<double>();

  if (!this->sdf->HasElement("frameName"))
  {
    ROS_INFO("imu plugin missing <frameName>, defaults to <bodyName>");
    this->frame_name_ = link_name_;
  }
  else
    this->frame_name_ = this->sdf->Get<std::string>("frameName");

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  this->rosnode_ = new ros::NodeHandle(this->robot_namespace_);

  // publish multi queue
  this->pmq.startServiceThread();

  // assert that the body by link_name_ exists
  this->link = boost::dynamic_pointer_cast<physics::Link>(
    this->world_->GetEntity(this->link_name_));
  if (!this->link)
  {
    ROS_FATAL("gazebo_ros_imu plugin error: bodyName: %s does not exist\n",
      this->link_name_.c_str());
    return;
  }

  // if topic name specified as empty, do not publish
  if (this->topic_name_ != "")
  {
    this->pub_Queue = this->pmq.addPub<sensor_msgs::Imu>();
    this->pub_ = this->rosnode_->advertise<sensor_msgs::Imu>(
      this->topic_name_, 1);

    // advertise services on the custom queue
    ros::AdvertiseServiceOptions aso =
      ros::AdvertiseServiceOptions::create<std_srvs::Empty>(
      this->service_name_, boost::bind(&GazeboRosIMU::ServiceCallback,
      this, _1, _2), ros::VoidPtr(), &this->imu_queue_);
    this->srv_ = this->rosnode_->advertiseService(aso);
  }

  // Initialize the controller
  this->last_time_ = this->world_->GetSimTime();

  // this->initial_pose_ = this->link->GetPose();
  this->last_vpos_ = this->link->GetWorldLinearVel();
  this->last_veul_ = this->link->GetWorldAngularVel();
  this->apos_ = 0;
  this->aeul_ = 0;

  // start custom queue for imu
  this->callback_queue_thread_ =
    boost::thread(boost::bind(&GazeboRosIMU::IMUQueueThread, this));


  // New Mechanism for Updating every World Cycle
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
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
// Update the controller
void GazeboRosIMU::UpdateChild()
{
  common::Time cur_time = this->world_->GetSimTime();
  
  // rate control
  if (this->update_rate_ > 0 &&
      (cur_time - this->last_time_).Double() < (1.0 / this->update_rate_))
    return;
    
  if ((this->pub_.getNumSubscribers() > 0 && this->topic_name_ != ""))
  {
    math::Pose pose;
    math::Quaternion rot;
    math::Vector3 pos;

    // Get Pose/Orientation ///@todo: verify correctness
    pose = this->link->GetWorldPose();
    // apply xyz offsets and get position and rotation components
    pos = pose.pos + this->offset_.pos;
    rot = pose.rot;

    // apply rpy offsets
    rot = this->offset_.rot*rot;
    rot.Normalize();

    // get Rates
    math::Vector3 vpos = this->link->GetWorldLinearVel();
    math::Vector3 veul = this->link->GetWorldAngularVel();

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
    this->imu_msg_.header.frame_id = this->frame_name_;
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
    math::Vector3 linear_velocity(
      veul.x + this->GaussianKernel(0, this->gaussian_noise_),
      veul.y + this->GaussianKernel(0, this->gaussian_noise_),
      veul.z + this->GaussianKernel(0, this->gaussian_noise_));
    // rotate into local frame
    // @todo: deal with offsets!
    linear_velocity = rot.RotateVector(linear_velocity);
    this->imu_msg_.angular_velocity.x    = linear_velocity.x;
    this->imu_msg_.angular_velocity.y    = linear_velocity.y;
    this->imu_msg_.angular_velocity.z    = linear_velocity.z;

    // pass accelerations
    math::Vector3 linear_acceleration(
      apos_.x + this->GaussianKernel(0, this->gaussian_noise_),
      apos_.y + this->GaussianKernel(0, this->gaussian_noise_),
      apos_.z + this->GaussianKernel(0, this->gaussian_noise_));
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

    {
      boost::mutex::scoped_lock lock(this->lock_);
      // publish to ros
      if (this->pub_.getNumSubscribers() > 0 && this->topic_name_ != "")
          this->pub_Queue->push(this->imu_msg_, this->pub_);
    }

    // save last time stamp
    this->last_time_ = cur_time;
  }
}


//////////////////////////////////////////////////////////////////////////////
// Utility for adding noise
double GazeboRosIMU::GaussianKernel(double mu, double sigma)
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
}
