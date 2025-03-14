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
#ifndef GAZEBO_ROS_MRU_H
#define GAZEBO_ROS_MRU_H

#include <gazebo/common/Plugin.hh>
#include <gazebo_plugins/gazebo_ros_utils.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>

namespace gazebo
{
  /// Wrapper that combines GPS and IMU sensor messages to simulate a
  /// Motion Reference Unit. The set of messages are synchronized using
  /// the same timestamp.
  /// Can also wrap a simple GPS by omiting the IMU sensor.
  class GazeboRosMRU : public SensorPlugin
  {
    /// Loads the sensors. It is expected that the plugin is attached to
    /// a GpsSensor and an optional ImuSensor is also attached as a sibling
    /// of the GpsSensor.
    public: void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);
    
    private: void OnGpsUpdate();
    private: void OnImuUpdate();
    
    private: void LookForIMU();
    private: event::ConnectionPtr _connection_;

    /// Estimates a signal variance based on the parameters of a GaussianNoiseModel.
    /// Returns 0.0 if _noise is null or not a GaussianNoiseModel.
    private: static double EstimateVariance(sensors::NoisePtr _noise);
    
    private: sensors::GpsSensorPtr gps_sensor_;
    private: event::ConnectionPtr gps_connection_;
    
    private: std::string imu_sensor_name_;
    private: sensors::ImuSensorPtr imu_sensor_;
    private: event::ConnectionPtr imu_connection_;
    
    private: physics::WorldPtr world_;
    private: physics::EntityPtr link_;
    
    // A bug in Gazebo wrongly calulates spherical positions
    // Some worlds may use a workaround while other don't so
    // use a parameter to idicate is the plugin should compensate
    // https://github.com/osrf/gazebo/issues/2022
    private: bool fix_gazebo_wsu_bug_;
    
    private: GazeboRosPtr gazebo_ros_;
    private: ros::Publisher position_publisher_;
    private: sensor_msgs::NavSatFix position_message_; 
    private: ros::Publisher orientation_publisher_;
    private: sensor_msgs::Imu orientation_message_;
    private: ros::Publisher velocity_publisher_;
    private: geometry_msgs::TwistWithCovarianceStamped velocity_message_;
    
    
  };
}

#endif //GAZEBO_ROS_MRU_H
