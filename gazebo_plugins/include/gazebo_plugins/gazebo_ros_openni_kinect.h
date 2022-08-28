/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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

/* DEPRECATED.
 * Instead use, GazeboRosDepthCamera
 */

#ifndef GAZEBO_ROS_OPENNI_KINECT_HH
#define GAZEBO_ROS_OPENNI_KINECT_HH

// Base plugin
#include <gazebo_plugins/gazebo_ros_depth_camera.h>

namespace gazebo
{
  class GazeboRosOpenniKinect : public GazeboRosDepthCamera
  {
    public: GazeboRosOpenniKinect();
    public: virtual void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);
  };
}
#endif

