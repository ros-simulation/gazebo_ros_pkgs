/*
 * Copyright 2017 Open Source Robotics Foundation
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

#include <string>

#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/MultiCameraSensor.hh>
#include <gazebo/sensors/SensorTypes.hh>

#include "gazebo_plugins/gazebo_ros_triggered_multicamera.h"

namespace gazebo
{
// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboRosTriggeredMultiCamera)

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosTriggeredMultiCamera::GazeboRosTriggeredMultiCamera()
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosTriggeredMultiCamera::~GazeboRosTriggeredMultiCamera()
{
}

void GazeboRosTriggeredMultiCamera::Load(sensors::SensorPtr _parent,
  sdf::ElementPtr _sdf)
{
  MultiCameraPlugin::Load(_parent, _sdf);

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  // initialize shared_ptr members
  this->image_connect_count_ = boost::shared_ptr<int>(new int(0));
  this->image_connect_count_lock_ = boost::shared_ptr<boost::mutex>(new boost::mutex);
  this->was_active_ = boost::shared_ptr<bool>(new bool(false));

  const double hackBaselineSdf = (_sdf->HasElement("hackBaseline") ?
                                  _sdf->Get<double>("hackBaseline") : 0);

  std::vector<std::string> cameraSuffixes;
  if (_sdf->HasElement("cameraSuffixes"))
  {
    const auto suffixes = _sdf->Get<std::string>("cameraSuffixes");
    if (!suffixes.empty())
    {
      boost::split(cameraSuffixes, suffixes, boost::is_any_of(","));
      if (cameraSuffixes.size() != this->camera.size())
      {
        ROS_FATAL_STREAM_NAMED("triggered_multicamera",
          "The multicamera plugin "
          " has different number of cameras than there are items in "
          "<cameraSuffixes>. Either leave <cameraSuffixes> empty, or put there "
          "the same number of comma-delimited strings as there are cameras.");
        return;
      }
    }
  }
    // backwards compatibility with the two-camera-only version
  else if (this->camera.size() == 2 &&
    this->camera[0]->Name().find("left") != std::string::npos &&
    this->camera[1]->Name().find("right") != std::string::npos)
  {
    cameraSuffixes = { "/left", "/right" };
  }
  else if (this->camera.size() == 2 &&
    this->camera[0]->Name().find("right") != std::string::npos &&
    this->camera[1]->Name().find("left") != std::string::npos)
  {
    cameraSuffixes = { "/right", "/left" };
  }

  std::vector<std::string> frameNames;
  const auto frameName = (_sdf->HasElement("frameName") ?
                          _sdf->Get<std::string>("frameName") : "world");
  // read <frameName> as a comma-separated list of camera frames
  boost::split(frameNames, frameName, boost::is_any_of(","));
  // if only one frame name was entered, use it for all cameras
  if (this->camera.size() > 1 && frameNames.size() == 1)
  {
    for (size_t i = 0; i < this->camera.size() - 1; ++i)
      frameNames.push_back(frameNames[0]);
  }

  // copying from CameraPlugin into GazeboRosCameraUtils
  for (unsigned i = 0; i < this->camera.size(); ++i)
  {
    GazeboRosTriggeredCamera * cam = new GazeboRosTriggeredCamera();
    cam->parentSensor_ = this->parentSensor;
    cam->width_   = this->width[i];
    cam->height_  = this->height[i];
    cam->depth_   = this->depth[i];
    cam->format_  = this->format[i];
    cam->camera_  = this->camera[i];
    // Set up a shared connection counter
    cam->image_connect_count_ = this->image_connect_count_;
    cam->image_connect_count_lock_ = this->image_connect_count_lock_;
    cam->was_active_ = this->was_active_;

    double hackBaseline = 0.0;
    if (this->camera[i]->Name().find("right") != std::string::npos)
      hackBaseline = hackBaselineSdf;

    const auto cameraSuffix = (cameraSuffixes.empty() ?
      ("/" + this->camera[i]->Name()) : cameraSuffixes[i]);

    cam->Load(_parent, _sdf, cameraSuffix, hackBaseline);
    cam->frame_name_ = frameNames[i]; // overwrite the SDF-parsed frame name

    this->triggered_cameras.push_back(cam);
  }
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosTriggeredMultiCamera::OnNewFrame(const unsigned char *_image,
    const size_t _camNumber,
    unsigned int _width, unsigned int _height, unsigned int _depth,
    const std::string &_format)
{
  GazeboRosTriggeredCamera * cam = this->triggered_cameras[_camNumber];
  cam->OnNewFrame(_image, _width, _height, _depth, _format);
}
}
