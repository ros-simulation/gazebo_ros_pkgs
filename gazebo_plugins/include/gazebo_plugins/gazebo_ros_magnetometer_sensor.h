/*
 * Copyright 2020 Open Source Robotics Foundation
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
 * Desc: magnetometer ros interface.
 * Author: Johannes Bier
 * Date: 11 February 2020
 */

#ifndef GAZEBO_ROS_MAGNETOMETER_SENSOR_H
#define GAZEBO_ROS_MAGNETOMETER_SENSOR_H

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/UpdateInfo.hh>
#include <gazebo/sensors/MagnetometerSensor.hh>

#include <ros/ros.h>
#include <sensor_msgs/MagneticField.h>

namespace gazebo
{
  namespace sensors
  {
    class MagnetometerSensor;
  }
  /**
  @anchor GazeboRosMagnetometerSensor
  \ref GazeboRosMagnetometerSensor is a plugin to simulate a magnetic field sensor. Some things to note:
  - inheritance from SensorPlugin,
  - measurements are given by gazebo MagnetometerSensor instead of being computed by the ros plugin
  */
  /** @brief Gazebo Ros magnetometer sensor plugin. */
  class GazeboRosMagnetometerSensor : public SensorPlugin
  {
  public:
    /// \brief Constructor.
    GazeboRosMagnetometerSensor();
    /// \brief Destructor.
    virtual ~GazeboRosMagnetometerSensor();
    /// \brief Load the sensor.
    /// \param sensor_ pointer to the sensor.
    /// \param sdf_ pointer to the sdf config file.
    virtual void Load(sensors::SensorPtr sensor_, sdf::ElementPtr sdf_);

  protected:
    /// \brief Update the sensor.
    virtual void UpdateChild(const gazebo::common::UpdateInfo &/*_info*/);

  private:
    /// \brief Load the parameters from the sdf file.
    bool LoadParameters();
    
    /// \brief Ros NodeHandle pointer.
    ros::NodeHandle* node;
    /// \brief Ros Publisher for imu data.
    ros::Publisher magnetometer_data_publisher;
    /// \brief Ros IMU message.
    sensor_msgs::MagneticField magnetometer_msg;

    /// \brief last time on which the data was published.
    common::Time last_time;
    /// \brief Pointer to the update event connection.
    gazebo::event::ConnectionPtr connection;
    /// \brief Pointer to the sensor.
    sensors::MagnetometerSensor* sensor;
    /// \brief Pointer to the sdf config file.
    sdf::ElementPtr sdf;

    //loaded parameters
    /// \brief The data is published on the topic named: /robot_namespace/topic_name.
    std::string robot_namespace;
    /// \brief The data is published on the topic named: /robot_namespace/topic_name.
    std::string topic_name;
    /// \brief Name of the link of the IMU.
    std::string body_name;
    /// \brief Sensor update rate.
    double update_rate;
  };
}

#endif //GAZEBO_ROS_MAGNETOMETER_SENSOR_H
