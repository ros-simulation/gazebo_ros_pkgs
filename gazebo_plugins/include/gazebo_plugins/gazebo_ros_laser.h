/*
 * Copyright 2013-2018 Open Source Robotics Foundation
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

#ifndef GAZEBO_ROS_LASER_HH
#define GAZEBO_ROS_LASER_HH

#include <gazebo_plugins/gazebo_ros_ray_sensor.h>

namespace gazebo
{
  /// Deprecated plugin, simply loads GazeboRosRaySensor with different configuration for backwards compatibility
  class GazeboRosLaser : public GazeboRosRaySensor
  {
    /// \brief Constructor
    public: GazeboRosLaser();

    /// \brief Destructor
    public: ~GazeboRosLaser();

    /// \brief Override the default behavior for resolving tf frame to provide backwards compatibility
    protected: virtual std::string resolveTF(const std::string& _frame, const std::string& _robot_namespace, ros::NodeHandle& _nh) override;

    /// \brief Load the plugin
    public: void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);
  };
}
#endif
