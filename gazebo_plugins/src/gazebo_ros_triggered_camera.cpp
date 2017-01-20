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
 @mainpage
   Desc: GazeboRosTriggeredCamera plugin for simulating cameras in Gazebo
   Author: John Hsu
   Date: 24 Sept 2008
*/

#include "gazebo_plugins/gazebo_ros_triggered_camera.h"

#include <float.h>
#include <string>

#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo/sensors/SensorTypes.hh>

namespace gazebo
{
// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboRosTriggeredCamera)

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosTriggeredCamera::GazeboRosTriggeredCamera()
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosTriggeredCamera::~GazeboRosTriggeredCamera()
{
  ROS_DEBUG_STREAM_NAMED("camera","Unloaded");
}

void GazeboRosTriggeredCamera::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{
  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  CameraPlugin::Load(_parent, _sdf);
  // copying from CameraPlugin into GazeboRosTriggeredCameraUtils
  this->parentSensor_ = this->parentSensor;
  this->width_ = this->width;
  this->height_ = this->height;
  this->depth_ = this->depth;
  this->format_ = this->format;
  this->camera_ = this->camera;

  GazeboRosCameraUtils::Load(_parent, _sdf);
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosTriggeredCamera::OnNewFrame(const unsigned char *_image,
    unsigned int _width, unsigned int _height, unsigned int _depth,
    const std::string &_format)
{
# if GAZEBO_MAJOR_VERSION >= 7
  this->sensor_update_time_ = this->parentSensor_->LastUpdateTime();
# else
  this->sensor_update_time_ = this->parentSensor_->GetLastUpdateTime();
# endif

  if ((*this->image_connect_count_) > 0)
  {
    this->PutCameraData(_image);
    this->PublishCameraInfo();
  }
  this->SetCameraEnabled(false);
}

void GazeboRosTriggeredCamera::TriggerCamera()
{
  this->preRenderConnection_ =
      event::Events::ConnectPreRender(
          std::bind(&GazeboRosTriggeredCamera::PreRender, this));
}

void GazeboRosTriggeredCamera::PreRender()
{
  this->SetCameraEnabled(true);
  this->preRenderConnection_.reset();
}

void GazeboRosTriggeredCamera::SetCameraEnabled(const bool _enabled)
{
  this->parentSensor_->SetActive(_enabled);
  this->parentSensor_->SetUpdateRate(_enabled ? 0.0 : DBL_MIN);
}

}
