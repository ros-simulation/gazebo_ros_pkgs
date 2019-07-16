// Copyright 2019 Open Source Robotics Foundation
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

#include <gazebo_plugins/MultiCameraPlugin.hpp>
#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo/sensors/DepthCameraSensor.hh>

#include <string>

GZ_REGISTER_SENSOR_PLUGIN(gazebo::MultiCameraPlugin)

/////////////////////////////////////////////////
gazebo::MultiCameraPlugin::MultiCameraPlugin()
: SensorPlugin()
{
}

/////////////////////////////////////////////////
gazebo::MultiCameraPlugin::~MultiCameraPlugin()
{
  for (auto conn : this->newFrameConnection) {
    conn.reset();
  }
  this->newFrameConnection.clear();

  this->parentSensor.reset();
  this->camera.clear();
}

/////////////////////////////////////////////////
void gazebo::MultiCameraPlugin::Load(gazebo::sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/)
{
  if (!_sensor) {
    gzerr << "Invalid sensor pointer.\n";
  }

  this->parentSensor = std::dynamic_pointer_cast<sensors::MultiCameraSensor>(_sensor);

  if (!this->parentSensor) {
    gzerr << "MultiCameraPlugin requires a CameraSensor.\n";
    if (std::dynamic_pointer_cast<sensors::DepthCameraSensor>(_sensor)) {
      gzmsg << "It is a depth camera sensor\n";
    }
    if (std::dynamic_pointer_cast<sensors::CameraSensor>(_sensor)) {
      gzmsg << "It is a camera sensor\n";
    }
  }

  if (!this->parentSensor) {
    gzerr << "MultiCameraPlugin not attached to a camera sensor\n";
    return;
  }

  for (unsigned int i = 0; i < this->parentSensor->CameraCount(); ++i) {
    this->camera.push_back(this->parentSensor->Camera(i));

    // save camera attributes
    this->width.push_back(this->camera[i]->ImageWidth());
    this->height.push_back(this->camera[i]->ImageHeight());
    this->depth.push_back(this->camera[i]->ImageDepth());
    this->format.push_back(this->camera[i]->ImageFormat());

    std::string cameraName = this->parentSensor->Camera(i)->Name();
    // gzdbg << "camera(" << i << ") name [" << cameraName << "]\n";

    this->newFrameConnection.push_back(this->camera[i]->ConnectNewImageFrame(
        std::bind(&MultiCameraPlugin::OnNewMultiFrame,
        this, std::placeholders::_1, std::placeholders::_2,
        std::placeholders::_3, std::placeholders::_4, std::placeholders::_5, i)));
  }

  this->parentSensor->SetActive(true);
}

/////////////////////////////////////////////////
void gazebo::MultiCameraPlugin::OnNewMultiFrame(
  const unsigned char * /*_image*/,
  unsigned int /*_width*/,
  unsigned int /*_height*/,
  unsigned int /*_depth*/,
  const std::string & /*_format*/,
  int /*camera_num*/)
{
  /*rendering::Camera::SaveFrame(_image, this->width,
    this->height, this->depth, this->format,
    "/tmp/camera/me.jpg");
    */
}
