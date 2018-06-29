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

#include <gazebo_plugins/gazebo_ros_openni_kinect.h>

namespace gazebo
{
// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboRosOpenniKinect)

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosOpenniKinect::GazeboRosOpenniKinect() : GazeboRosDepthCamera()
{

}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosOpenniKinect::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{
  // Warn user that this plugin is deprecated and to migrate to depth camera
  ROS_WARN_NAMED("depth_camera", "gazebo_ros_openni_kinect is deprecated. Please use gazebo_ros_depth_camera instead");

  // For API compatibility, openni_kinect pointCloudCutoffMax must default to 5 while depth camera
  // defautls to -1 (no limit).
  if (!_sdf->HasElement("pointCloudCutoffMax"))
  {
    ROS_INFO_NAMED("depth_camera", "Tag <pointCloudCutoffMax> not set, defaulting to 5.0");
    // Create element description for this tag
    sdf::ElementPtr element = std::make_shared<sdf::Element>();
    element->SetName("pointCloudCutoffMax");
    element->AddValue("double", "5.0", false, "");
    element->Set<double>(5.0);
    _sdf->AddElementDescription(element);
    // Force the element to be generated so depth camera doesn't go with its default
    sdf::ElementPtr my_element = _sdf->GetElement("pointCloudCutoffMax");
  }

  // Load parent plugin
  this->GazeboRosDepthCamera::Load(_parent, _sdf);
}

}
