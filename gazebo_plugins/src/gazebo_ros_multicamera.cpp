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
 * Desc: Syncronizes shutters across multiple cameras
 * Author: John Hsu
 * Date: 10 June 2013
 */

#include <string>

#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/MultiCameraSensor.hh>
#include <gazebo/sensors/SensorTypes.hh>

#include "gazebo_plugins/gazebo_ros_multicamera.h"

namespace gazebo
{
// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboRosMultiCamera)

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosMultiCamera::GazeboRosMultiCamera()
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosMultiCamera::~GazeboRosMultiCamera()
{
}

void GazeboRosMultiCamera::Load(sensors::SensorPtr _parent,
  sdf::ElementPtr _sdf)
{
  MultiCameraPlugin::Load(_parent, _sdf);

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM_NAMED("multicamera", "A ROS node for Gazebo has not been initialized, unable to load plugin. "
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
        ROS_FATAL_STREAM_NAMED("multicamera", "The multicamera plugin "
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
    GazeboRosCameraUtils* util = new GazeboRosCameraUtils();
    util->parentSensor_ = this->parentSensor;
    util->width_   = this->width[i];
    util->height_  = this->height[i];
    util->depth_   = this->depth[i];
    util->format_  = this->format[i];
    util->camera_  = this->camera[i];
    // Set up a shared connection counter
    util->image_connect_count_ = this->image_connect_count_;
    util->image_connect_count_lock_ = this->image_connect_count_lock_;
    util->was_active_ = this->was_active_;

    double hackBaseline = 0.0;
    if (this->camera[i]->Name().find("right") != std::string::npos)
      hackBaseline = hackBaselineSdf;

    const auto cameraSuffix = (cameraSuffixes.empty() ?
      ("/" + this->camera[i]->Name()) : cameraSuffixes[i]);

    util->Load(_parent, _sdf, cameraSuffix, hackBaseline);
    util->frame_name_ = frameNames[i]; // overwrite the SDF-parsed frame name

    this->utils.push_back(util);
  }
}

// Update the controller
void GazeboRosMultiCamera::OnNewFrame(const unsigned char *_image,
    const size_t _camNumber,
    unsigned int /*_width*/, unsigned int /*_height*/, unsigned int /*_depth*/,
    const std::string &/*_format*/)
{
  GazeboRosCameraUtils* util = this->utils[_camNumber];

# if GAZEBO_MAJOR_VERSION >= 7
  common::Time sensor_update_time = util->parentSensor_->LastMeasurementTime();
# else
  common::Time sensor_update_time = util->parentSensor_->GetLastMeasurementTime();
# endif

  if (util->parentSensor_->IsActive())
  {
    if (sensor_update_time - util->last_update_time_ >= util->update_period_)
    {
      util->PutCameraData(_image, sensor_update_time);
      util->PublishCameraInfo(sensor_update_time);
      util->last_update_time_ = sensor_update_time;
    }
  }
}
}
