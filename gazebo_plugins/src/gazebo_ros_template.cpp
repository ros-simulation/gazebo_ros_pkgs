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
 * Desc: 
 * Author: 
 * Date: 
 */

#include <gazebo_plugins/gazebo_ros_template.h>
#include <ros/ros.h>

namespace gazebo
{
// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboRosTemplate);

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosTemplate::GazeboRosTemplate()
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosTemplate::~GazeboRosTemplate()
{
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosTemplate::Load( physics::ModelPtr _parent, sdf::ElementPtr _sdf )
{
  // Make sure the ROS node for Gazebo has already been initalized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosTemplate::UpdateChild()
{
}

}
