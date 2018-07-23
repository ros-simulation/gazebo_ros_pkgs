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

#include <tf/transform_listener.h>

#include <gazebo_plugins/gazebo_ros_block_laser.h>

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
}

std::string GazeboRosBlockLaser::resolveTF(const std::string& _frame, const std::string& _robot_namespace, ros::NodeHandle& _nh)
{
  std::string prefix;
  _nh.getParam(std::string("tf_prefix"), prefix);
  return tf::resolve(prefix, _frame);
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosBlockLaser::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{
  // Warn user that this plugin is deprecated
  ROS_WARN_NAMED("laser", "gazebo_ros_block_laser is deprecated. Please use gazebo_ros_ray_sensor instead");

  // For API compatibility, gazebo_ros_block_laser should have output type PointCloud
  if (!_sdf->HasElement("outputType"))
  {
    // Set outputType to sensor_msgs/PointCloud
    sdf::ElementPtr element = std::make_shared<sdf::Element>();
    element->SetName("outputType");
    element->AddValue("string", "", false, "");
    element->Set<std::string>(ros::message_traits::DataType<sensor_msgs::PointCloud>::value());
    _sdf->AddElementDescription(element);
    sdf::ElementPtr my_element = _sdf->GetElement("outputType");
  }

  // Load parent plugin
  this->GazeboRosRaySensor::Load(_parent, _sdf);
}

}
