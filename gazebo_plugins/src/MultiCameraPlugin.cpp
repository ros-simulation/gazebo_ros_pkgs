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
#include <gazebo/sensors/DepthCameraSensor.hh>
#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo_plugins/MultiCameraPlugin.h>
#include <gazebo_plugins/gazebo_ros_utils.h>

using namespace gazebo;
GZ_REGISTER_SENSOR_PLUGIN(MultiCameraPlugin)

/////////////////////////////////////////////////
MultiCameraPlugin::MultiCameraPlugin() : SensorPlugin()
{
}

/////////////////////////////////////////////////
MultiCameraPlugin::~MultiCameraPlugin()
{
  this->parentSensor.reset();
  this->camera.clear();
}

/////////////////////////////////////////////////
void MultiCameraPlugin::Load(sensors::SensorPtr _sensor,
  sdf::ElementPtr /*_sdf*/)
{
  if (!_sensor)
    gzerr << "Invalid sensor pointer.\n";

  GAZEBO_SENSORS_USING_DYNAMIC_POINTER_CAST;
  this->parentSensor =
    dynamic_pointer_cast<sensors::MultiCameraSensor>(_sensor);

  if (!this->parentSensor)
  {
    gzerr << "MultiCameraPlugin requires a CameraSensor.\n";
    if (dynamic_pointer_cast<sensors::DepthCameraSensor>(_sensor))
      gzmsg << "It is a depth camera sensor\n";
    if (dynamic_pointer_cast<sensors::CameraSensor>(_sensor))
      gzmsg << "It is a camera sensor\n";
  }

  if (!this->parentSensor)
  {
    gzerr << "MultiCameraPlugin not attached to a camera sensor\n";
    return;
  }

  for (unsigned int i = 0; i < this->parentSensor->CameraCount(); ++i)
  {
    this->camera.push_back(this->parentSensor->Camera(i));

    // save camera attributes
    this->width.push_back(this->camera[i]->ImageWidth());
    this->height.push_back(this->camera[i]->ImageHeight());
    this->depth.push_back(this->camera[i]->ImageDepth());
    this->format.push_back(this->camera[i]->ImageFormat());

    std::string cameraName = this->parentSensor->Camera(i)->Name();
    // gzdbg << "camera(" << i << ") name [" << cameraName << "]\n";

    if ( this->parentSensor->CameraCount() <= 2 ) {
      if (cameraName.find("left") == std::string::npos &&
          cameraName.find("right") == std::string::npos   ) {
        // legacy: if only two cameras, must have "left" or "right"
        //  in name to be added; otherwise ignored
        continue;
      }
    }
    
    ROS_INFO_NAMED("MultiCameraPlugin", "%s loaded with index %d",
                   cameraName.c_str(), i);
    
    this->newFrameConnection.push_back
      (this->camera[i]->ConnectNewImageFrame
       (boost::bind(&MultiCameraPlugin::OnNewFrame,
                    this, i, _1, _2, _3, _4, _5)));
  }

  this->parentSensor->SetActive(true);
}

/////////////////////////////////////////////////
void MultiCameraPlugin::OnNewFrame(const unsigned int camNumber,
                                   const unsigned char * /*_image*/,
                                   unsigned int /*_width*/,
                                   unsigned int /*_height*/,
                                   unsigned int /*_depth*/,
                                   const std::string &/*_format*/)
{
}
