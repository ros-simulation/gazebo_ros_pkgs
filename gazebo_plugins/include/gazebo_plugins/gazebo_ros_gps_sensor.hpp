// Copyright 2019 Open Source Robotics Foundation, Inc.
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


#ifndef GAZEBO_PLUGINS__GAZEBO_ROS_GPS_SENSOR_HPP_
#define GAZEBO_PLUGINS__GAZEBO_ROS_GPS_SENSOR_HPP_

#include <gazebo/common/Plugin.hh>
#include <gazebo/sensors/GpsSensor.hh>
#include <gazebo/common/Events.hh>

#include <memory>

namespace gazebo_plugins
{

class GazeboRosGpsSensorPrivate;

/// Plugin to attach to a gazebo GPS sensor and publish ROS message of output
/**
  Example Usage:
  \code{.xml}
    <sensor name="my_gps" type="gps">
      <!-- ensure the sensor is active (required) -->
      <always_on>true</always_on>
      <update_rate>30</update_rate>
      <plugin name="my_gps_plugin" filename="libgazebo_ros_gps_sensor.so">
        <ros>
          <!-- publish to /gps/data -->
          <namespace>/gps</namespace>
          <remapping>~/out:=data</remapping>
        </ros>
      </plugin>
    </sensor>
  \endcode
*/
class GazeboRosGpsSensor : public gazebo::SensorPlugin
{
public:
  /// Constructor.
  GazeboRosGpsSensor();
  /// Destructor.
  virtual ~GazeboRosGpsSensor();

  // Documentation Inherited
  void Load(gazebo::sensors::SensorPtr _sensor, sdf::ElementPtr _sdf) override;

private:
  /// Private data pointer
  std::unique_ptr<GazeboRosGpsSensorPrivate> impl_;
};

}  // namespace gazebo_plugins

#endif  // GAZEBO_PLUGINS__GAZEBO_ROS_GPS_SENSOR_HPP_
