// Copyright 2018 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#ifndef GAZEBO_PLUGINS__GAZEBO_ROS_IMU_SENSOR_HPP_
#define GAZEBO_PLUGINS__GAZEBO_ROS_IMU_SENSOR_HPP_

#include <gazebo/plugins/ImuSensorPlugin.hh>
#include <sensor_msgs/msg/imu.hpp>

#include <string>
#include <memory>

namespace gazebo_plugins
{

class GazeboRosImuSensorPrivate;

/// Plugin to attach to a gazebo IMU sensor and publish ROS message of output
/**
  Example Usage:
  \code{.xml}
    <sensor name="my_imu" type="imu">
      <!-- ensure the sensor is active (required) -->
      <always_on>true</always_on>
      <update_rate>30</update_rate>
      <plugin name="my_imu_plugin" filename="libgazebo_ros_imu_sensor.so">
        <ros>
          <!-- publish to /imu/data -->
          <namespace>/imu</namespace>
          <remapping>~/out:=data</remapping>
        </ros>
      </plugin>
    </sensor>
  \endcode
*/
class GazeboRosImuSensor : public gazebo::SensorPlugin
{
public:
  /// Constructor.
  GazeboRosImuSensor();
  /// Destructor.
  virtual ~GazeboRosImuSensor();

  // Documentation Inherited
  void Load(gazebo::sensors::SensorPtr _sensor, sdf::ElementPtr _sdf) override;

private:
  /// Private data pointer
  std::unique_ptr<GazeboRosImuSensorPrivate> impl_;
};

}  // namespace gazebo_plugins

#endif  // GAZEBO_PLUGINS__GAZEBO_ROS_IMU_SENSOR_HPP_
