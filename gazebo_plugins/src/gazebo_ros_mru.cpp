/*
 * Copyright (C) 2021 Roland Arsenault roland@ccom.unh.edu
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

#include <gazebo_plugins/gazebo_ros_mru.h>

#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/GpsSensor.hh>
#include <gazebo/sensors/ImuSensor.hh>
#include <gazebo/sensors/SensorManager.hh>
#include <gazebo/sensors/GaussianNoiseModel.hh>

#include <ros/ros.h>

GZ_REGISTER_SENSOR_PLUGIN(gazebo::GazeboRosMRU)

void gazebo::GazeboRosMRU::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
  gps_sensor_ = std::dynamic_pointer_cast<sensors::GpsSensor>(_sensor);
  if (!gps_sensor_)
  {
    ROS_WARN_STREAM("gazebo_ros_mru plugin did not receive a sensor of type \"gps\".");
    return;
  }

  world_ = physics::get_world(_sensor->WorldName());
  if(world_)
    link_ = world_->EntityByName(_sensor->ParentName());
  if (!link_)
  {
    ROS_WARN_STREAM("gazebo_ros_mru plugin did not find link entity.");
    return;
  }
  
  // Look for the name of the optional IMU. 
  if (_sdf->HasElement("imuSensorName"))
    imu_sensor_name_ = _sdf->Get<std::string>("imuSensorName");
  
  fix_gazebo_wsu_bug_ = true;
  if (_sdf->HasElement("fixWSUBug"))
    imu_sensor_name_ = _sdf->Get<bool>("fixWSUBug");
  
  // Initialize ROS
  gazebo_ros_ = GazeboRosPtr ( new GazeboRos ( _sensor, _sdf, "MRU" ) );
  gazebo_ros_->isInitialized();

  // Fetch the publish topics
  std::string position_topic, orientation_topic, velocity_topic;
  gazebo_ros_->getParameter<std::string> (position_topic, "positionTopic", "position" );
  gazebo_ros_->getParameter<std::string> (orientation_topic, "orientationTopic", "orientation" );
  gazebo_ros_->getParameter<std::string> (velocity_topic, "velocityTopic", "velocity" );
  
  // Setup the ROS publishers
  position_publisher_ = gazebo_ros_->node()->advertise<sensor_msgs::NavSatFix>(position_topic, 10);
  velocity_publisher_ = gazebo_ros_->node()->advertise<geometry_msgs::TwistWithCovarianceStamped>(velocity_topic, 10);
  // Only setup an orientation publisher if we expect an IMU.
  if(!imu_sensor_name_.empty())
    orientation_publisher_ = gazebo_ros_->node()->advertise<sensor_msgs::Imu>(orientation_topic, 10);
  
  std::string frame_id;
  gazebo_ros_->getParameter<std::string> (frame_id, "frameId", "mru" );

  position_message_.header.frame_id = frame_id;
  orientation_message_.header.frame_id = frame_id;
  velocity_message_.header.frame_id = frame_id;
 
  position_message_.position_covariance[0] = EstimateVariance(gps_sensor_->Noise(sensors::GPS_POSITION_LONGITUDE_NOISE_METERS));
  position_message_.position_covariance[4] = EstimateVariance(gps_sensor_->Noise(sensors::GPS_POSITION_LATITUDE_NOISE_METERS));
  position_message_.position_covariance[8] = EstimateVariance(gps_sensor_->Noise(sensors::GPS_POSITION_ALTITUDE_NOISE_METERS));
  if(position_message_.position_covariance[0] && position_message_.position_covariance[4] && position_message_.position_covariance[8])
    position_message_.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

  velocity_message_.twist.covariance[0] = EstimateVariance(gps_sensor_->Noise(sensors::GPS_VELOCITY_LONGITUDE_NOISE_METERS));
  velocity_message_.twist.covariance[7] = EstimateVariance(gps_sensor_->Noise(sensors::GPS_VELOCITY_LATITUDE_NOISE_METERS));
  velocity_message_.twist.covariance[14] = EstimateVariance(gps_sensor_->Noise(sensors::GPS_VELOCITY_ALTITUDE_NOISE_METERS));
  
  gps_connection_ = gps_sensor_->ConnectUpdated(std::bind(&gazebo::GazeboRosMRU::OnGpsUpdate, this));
}

double gazebo::GazeboRosMRU::EstimateVariance(sensors::NoisePtr _noise)
{
  if(!_noise)
    return 0.0;
  if(_noise->GetNoiseType() != sensors::Noise::GAUSSIAN)
    return 0.0;
  sensors::GaussianNoiseModelPtr gaussian_noise = std::dynamic_pointer_cast<sensors::GaussianNoiseModel>(_noise);
  if(!gaussian_noise)
    return 0.0;
  return pow(gaussian_noise->GetMean(),2) + pow(gaussian_noise->GetStdDev(),2) + pow(gaussian_noise->GetBias(),2);
  // GetDynamicBiasStdDev seems to be declared, but not implemented in gazebo. + pow(gaussian_noise->GetDynamicBiasStdDev(),2);
}

void gazebo::GazeboRosMRU::LookForIMU()
{
  for(auto s: sensors::SensorManager::Instance()->GetSensors())
    if(s->Name() == imu_sensor_name_ && s->ParentName() == gps_sensor_->ParentName())
    {
      imu_sensor_ = std::dynamic_pointer_cast<sensors::ImuSensor>(s);
      if (!imu_sensor_)
        ROS_WARN_STREAM("Sensor " << s->ScopedName() << " is not an imu sensor.");
      else
      {
        if(imu_sensor_->UpdateRate() != gps_sensor_->UpdateRate())
        {
          ROS_WARN_STREAM("Setting imu rate from " << imu_sensor_->UpdateRate() << " to " << gps_sensor_->UpdateRate());
          imu_sensor_->SetUpdateRate(gps_sensor_->UpdateRate());
        }
        
        // make sure we are getting world referenced data
        imu_sensor_->SetWorldToReferenceOrientation(ignition::math::Quaterniond::Identity);
        
        orientation_message_.angular_velocity_covariance[0] = EstimateVariance(imu_sensor_->Noise(sensors::IMU_ANGVEL_X_NOISE_RADIANS_PER_S));
        orientation_message_.angular_velocity_covariance[4] = EstimateVariance(imu_sensor_->Noise(sensors::IMU_ANGVEL_Y_NOISE_RADIANS_PER_S));
        orientation_message_.angular_velocity_covariance[8] = EstimateVariance(imu_sensor_->Noise(sensors::IMU_ANGVEL_Z_NOISE_RADIANS_PER_S));
        
        orientation_message_.linear_acceleration_covariance[0] = EstimateVariance(imu_sensor_->Noise(sensors::IMU_LINACC_X_NOISE_METERS_PER_S_SQR));
        orientation_message_.linear_acceleration_covariance[4] = EstimateVariance(imu_sensor_->Noise(sensors::IMU_LINACC_Y_NOISE_METERS_PER_S_SQR));
        orientation_message_.linear_acceleration_covariance[8] = EstimateVariance(imu_sensor_->Noise(sensors::IMU_LINACC_Z_NOISE_METERS_PER_S_SQR));
        
        imu_connection_ = imu_sensor_->ConnectUpdated(std::bind(&gazebo::GazeboRosMRU::OnImuUpdate, this));
      } 
    }
}

void gazebo::GazeboRosMRU::OnGpsUpdate()
{
  if (!imu_sensor_ && !imu_sensor_name_.empty())
    LookForIMU();

  common::Time current_time = gps_sensor_->LastUpdateTime();
  position_message_.header.stamp.sec = current_time.sec;
  position_message_.header.stamp.nsec = current_time.nsec;
  if(fix_gazebo_wsu_bug_)
  {
    common::SphericalCoordinatesPtr spherical_coords = world_->SphericalCoords();
    auto delta_lat = gps_sensor_->Latitude() - spherical_coords->LatitudeReference();
    auto delta_lon = gps_sensor_->Longitude() - spherical_coords->LongitudeReference();
    position_message_.latitude = (spherical_coords->LatitudeReference()-delta_lat).Degree();
    position_message_.longitude = (spherical_coords->LongitudeReference()-delta_lon).Degree();
  }
  else
  {
    position_message_.latitude = gps_sensor_->Latitude().Degree();
    position_message_.longitude = gps_sensor_->Longitude().Degree();
  }
  position_message_.altitude = gps_sensor_->Altitude();
  
  position_publisher_.publish(position_message_);
  
  velocity_message_.header.stamp.sec = current_time.sec;
  velocity_message_.header.stamp.nsec = current_time.nsec;
  velocity_message_.twist.twist.linear.x = gps_sensor_->VelocityEast();
  velocity_message_.twist.twist.linear.y = gps_sensor_->VelocityNorth();
  if(fix_gazebo_wsu_bug_)
  {
    velocity_message_.twist.twist.linear.x = -velocity_message_.twist.twist.linear.x;
    velocity_message_.twist.twist.linear.y = -velocity_message_.twist.twist.linear.y;
  }
  velocity_message_.twist.twist.linear.z = gps_sensor_->VelocityUp();
  
  if(imu_sensor_)
  {
    imu_sensor_->SetActive(gps_sensor_->IsActive());
    imu_sensor_->Update(true);
    
    orientation_message_.header.stamp.sec = current_time.sec;
    orientation_message_.header.stamp.nsec = current_time.nsec;
    
    orientation_message_.orientation.x = imu_sensor_->Orientation().X();
    orientation_message_.orientation.y = imu_sensor_->Orientation().Y();
    orientation_message_.orientation.z = imu_sensor_->Orientation().Z();
    orientation_message_.orientation.w = imu_sensor_->Orientation().W();
    
    orientation_message_.angular_velocity.x = imu_sensor_->AngularVelocity().X();
    orientation_message_.angular_velocity.y = imu_sensor_->AngularVelocity().Y();
    orientation_message_.angular_velocity.z = imu_sensor_->AngularVelocity().Z();
    
    orientation_message_.linear_acceleration.x = imu_sensor_->LinearAcceleration().X();
    orientation_message_.linear_acceleration.y = imu_sensor_->LinearAcceleration().Y();
    orientation_message_.linear_acceleration.z = imu_sensor_->LinearAcceleration().Z();
    
    orientation_publisher_.publish(orientation_message_);
    
    velocity_message_.twist.twist.angular = orientation_message_.angular_velocity;
  }
  
  velocity_publisher_.publish(velocity_message_);
}

void gazebo::GazeboRosMRU::OnImuUpdate()
{
  imu_sensor_->SetActive(gps_sensor_->IsActive());
}
