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
 * Desc: Ros Block Laser controller.
 * Author: Nathan Koenig
 * Date: 01 Feb 2007
 */

#include <algorithm>
#include <assert.h>

#include <gazebo_plugins/gazebo_ros_block_laser.h>
#include <gazebo_plugins/gazebo_ros_utils.h>

#include <gazebo/physics/World.hh>
#include <gazebo/physics/HingeJoint.hh>
#include <gazebo/sensors/Sensor.hh>
#include <sdf/sdf.hh>
#include <sdf/Param.hh>
#include <gazebo/common/Exception.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/transport/Node.hh>

#include <geometry_msgs/Point32.h>
#include <sensor_msgs/ChannelFloat32.h>

#include <tf/tf.h>

#define EPSILON_DIFF 0.000001

namespace gazebo
{
// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboRosBlockLaser)

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosBlockLaser::GazeboRosBlockLaser()
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosBlockLaser::~GazeboRosBlockLaser()
{
  ////////////////////////////////////////////////////////////////////////////////
  // Finalize the controller / Custom Callback Queue
  this->laser_queue_.clear();
  this->laser_queue_.disable();
  this->rosnode_->shutdown();
  this->callback_laser_queue_thread_.join();

  delete this->rosnode_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosBlockLaser::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{
  // load plugin
  RayPlugin::Load(_parent, _sdf);

  // Get then name of the parent sensor
  this->parent_sensor_ = _parent;

  // Get the world name.
  std::string worldName = _parent->GetWorldName();
  this->world_ = physics::get_world(worldName);

  last_update_time_ = this->world_->GetSimTime();

  this->node_ = transport::NodePtr(new transport::Node());
  this->node_->Init(worldName);

  GAZEBO_SENSORS_USING_DYNAMIC_POINTER_CAST;
  this->parent_ray_sensor_ = dynamic_pointer_cast<sensors::RaySensor>(this->parent_sensor_);

  if (!this->parent_ray_sensor_)
    gzthrow("GazeboRosBlockLaser controller requires a Ray Sensor as its parent");

  this->robot_namespace_ = "";
  if (_sdf->HasElement("robotNamespace"))
    this->robot_namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";

  if (!_sdf->HasElement("frameName"))
  {
    ROS_INFO("Block laser plugin missing <frameName>, defaults to /world");
    this->frame_name_ = "/world";
  }
  else
    this->frame_name_ = _sdf->GetElement("frameName")->Get<std::string>();

  if (!_sdf->HasElement("topicName"))
  {
    ROS_INFO("Block laser plugin missing <topicName>, defaults to /world");
    this->topic_name_ = "/world";
  }
  else
    this->topic_name_ = _sdf->GetElement("topicName")->Get<std::string>();

  if (!_sdf->HasElement("gaussianNoise"))
  {
    ROS_INFO("Block laser plugin missing <gaussianNoise>, defaults to 0.0");
    this->gaussian_noise_ = 0;
  }
  else
    this->gaussian_noise_ = _sdf->GetElement("gaussianNoise")->Get<double>();

  if (!_sdf->HasElement("hokuyoMinIntensity"))
  {
    ROS_INFO("Block laser plugin missing <hokuyoMinIntensity>, defaults to 101");
    this->hokuyo_min_intensity_ = 101;
  }
  else
    this->hokuyo_min_intensity_ = _sdf->GetElement("hokuyoMinIntensity")->Get<double>();

  ROS_INFO("INFO: gazebo_ros_laser plugin should set minimum intensity to %f due to cutoff in hokuyo filters." , this->hokuyo_min_intensity_);

  if (!_sdf->HasElement("updateRate"))
  {
    ROS_INFO("Block laser plugin missing <updateRate>, defaults to 0");
    this->update_rate_ = 0;
  }
  else
    this->update_rate_ = _sdf->GetElement("updateRate")->Get<double>();
  // FIXME:  update the update_rate_


  this->laser_connect_count_ = 0;

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  this->rosnode_ = new ros::NodeHandle(this->robot_namespace_);

  // resolve tf prefix
  std::string prefix;
  this->rosnode_->getParam(std::string("tf_prefix"), prefix);
  this->frame_name_ = tf::resolve(prefix, this->frame_name_);


  point_step_ = 16;
  cloud_msg_.header.frame_id = frame_name_;
  cloud_msg_.fields.resize(4);
  cloud_msg_.fields[0].name = "x";
  cloud_msg_.fields[0].offset = 0;
  cloud_msg_.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
  cloud_msg_.fields[0].count = 1;
  cloud_msg_.fields[1].name = "y";
  cloud_msg_.fields[1].offset = 4;
  cloud_msg_.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
  cloud_msg_.fields[1].count = 1;
  cloud_msg_.fields[2].name = "z";
  cloud_msg_.fields[2].offset = 8;
  cloud_msg_.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
  cloud_msg_.fields[2].count = 1;
  cloud_msg_.fields[3].name = "intensity";
  cloud_msg_.fields[3].offset = 12;
  cloud_msg_.fields[3].datatype = sensor_msgs::PointField::FLOAT32;
  cloud_msg_.fields[3].count = 1;
  // cloud_msg_.fields[4].name = "ring";
  // cloud_msg_.fields[4].offset = 20;
  // cloud_msg_.fields[4].datatype = sensor_msgs::PointField::UINT16;
  // cloud_msg_.fields[4].count = 1;
  cloud_msg_.point_step = point_step_;

  // set size of cloud message, starts at 0!! FIXME: not necessary
  // this->cloud_msg_.points.clear();
  // this->cloud_msg_.channels.clear();
  // this->cloud_msg_.channels.push_back(sensor_msgs::ChannelFloat32());

  if (this->topic_name_ != "")
  {
    // Custom Callback Queue
    ros::AdvertiseOptions ao = ros::AdvertiseOptions::create<sensor_msgs::PointCloud2>(
      this->topic_name_,1,
      boost::bind( &GazeboRosBlockLaser::LaserConnect,this),
      boost::bind( &GazeboRosBlockLaser::LaserDisconnect,this), ros::VoidPtr(), &this->laser_queue_);
    this->pub_ = this->rosnode_->advertise(ao);
  }


  // Initialize the controller

  // sensor generation off by default
  this->parent_ray_sensor_->SetActive(false);
  // start custom queue for laser
  this->callback_laser_queue_thread_ = boost::thread( boost::bind( &GazeboRosBlockLaser::LaserQueueThread,this ) );

}

////////////////////////////////////////////////////////////////////////////////
// Increment count
void GazeboRosBlockLaser::LaserConnect()
{
  this->laser_connect_count_++;
  this->parent_ray_sensor_->SetActive(true);
}
////////////////////////////////////////////////////////////////////////////////
// Decrement count
void GazeboRosBlockLaser::LaserDisconnect()
{
  this->laser_connect_count_--;

  if (this->laser_connect_count_ == 0)
    this->parent_ray_sensor_->SetActive(false);
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosBlockLaser::OnNewLaserScans()
{
  if (this->topic_name_ != "")
  {
    common::Time sensor_update_time = this->parent_sensor_->GetLastUpdateTime();
    if (last_update_time_ < sensor_update_time)
    {
      this->PutLaserData(sensor_update_time);
      last_update_time_ = sensor_update_time;
    }
  }
  else
  {
    ROS_INFO("gazebo_ros_block_laser topic name not set");
  }
}

////////////////////////////////////////////////////////////////////////////////
// Put laser data to the interface
void GazeboRosBlockLaser::PutLaserData(common::Time &_updateTime)
{

  //clock_t start = clock();
  this->parent_ray_sensor_->SetActive(false);

  int vertialRangeCnt = parent_ray_sensor_->GetLaserShape()->GetVerticalSampleCount();
  int rangeCnt = parent_ray_sensor_->GetLaserShape()->GetSampleCount();
  if(vertialRangeCnt*rangeCnt == 0) return;

  
  double maxAngle = parent_ray_sensor_->GetAngleMax().Radian();
  double minAngle = parent_ray_sensor_->GetAngleMin().Radian();
  
  double vertialMaxAngle = parent_ray_sensor_->GetVerticalAngleMax().Radian();
  double vertialMinAngle = parent_ray_sensor_->GetVerticalAngleMin().Radian();
  

  // std::cout << "count: " << vertialRangeCnt << "  " 
  //      << rangeCnt << "  " 
  //      << vertialRangeCnt*rangeCnt << std::endl;
  double delta_angle = (maxAngle - minAngle )/rangeCnt;
  double delta_vertial_angle = (vertialMaxAngle - vertialMinAngle)/vertialRangeCnt;
  
  double maxRange = parent_ray_sensor_->GetRangeMax();
  double minRange = parent_ray_sensor_->GetRangeMin();
  
  cloud_msg_.header.stamp.sec  = (world->GetSimTime()).sec;
  cloud_msg_.header.stamp.nsec = (world->GetSimTime()).nsec;
  
  cloud_msg_.data.resize(vertialRangeCnt*rangeCnt*point_step_);

  cloud_msg_.width = rangeCnt;
  cloud_msg_.height = vertialRangeCnt;

  cloud_msg_.is_bigendian = false;
  cloud_msg_.is_dense = true;

  std::vector<double> ranges;
  parent_ray_sensor_->GetRanges(ranges);

  float x, y, z, intensity;
  double ray;
  float * ptr = (float*)( cloud_msg_.data.data() );
  
  for(int i = 0; i < vertialRangeCnt; ++i) 
  {
    double vertialAngle = vertialMaxAngle - delta_vertial_angle*i; 
    double angle = minAngle;
    double cos_va = cos(vertialAngle);
    double sin_va = sin(vertialAngle);
    int range_id = i*rangeCnt; 
    for(int j = 0; j < rangeCnt; j++)
    { 
      // ray = parent_ray_sensor_->GetLaserShape()->GetRange(range_id);
      ray = ranges[range_id];
      //intensity = parent_ray_sensor_->GetLaserShape()->GetRetro(i*rangeCnt + j);
      //ray = sensor_model_(ray, dt); //add noise

       x = ray*cos_va*cos(angle);
       y = ray*cos_va*sin(angle);
       z = ray*sin_va;

       *(ptr + 0)  = x;
       *(ptr + 1)  = y;
       *(ptr + 2)  = z;
       *(ptr + 3) = intensity;
       ptr += cloud_msg_.fields.size();
       angle += delta_angle;
       range_id++;
    }

  }

  this->parent_ray_sensor_->SetActive(true);

  // send data out via ros message
  this->pub_.publish(this->cloud_msg_);



}


//////////////////////////////////////////////////////////////////////////////
// Utility for adding noise
double GazeboRosBlockLaser::GaussianKernel(double mu,double sigma)
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

// Custom Callback Queue
////////////////////////////////////////////////////////////////////////////////
// custom callback queue thread
void GazeboRosBlockLaser::LaserQueueThread()
{
  static const double timeout = 0.01;

  while (this->rosnode_->ok())
  {
    this->laser_queue_.callAvailable(ros::WallDuration(timeout));
  }
}

void GazeboRosBlockLaser::OnStats( const boost::shared_ptr<msgs::WorldStatistics const> &_msg)
{
  this->sim_time_  = msgs::Convert( _msg->sim_time() );

  math::Pose pose;
  pose.pos.x = 0.5*sin(0.01*this->sim_time_.Double());
  gzdbg << "plugin simTime [" << this->sim_time_.Double() << "] update pose [" << pose.pos.x << "]\n";
}


}

