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

#ifndef GAZEBO_PLUGINS__MULTI_CAMERA_PLUGIN_HPP_
#define GAZEBO_PLUGINS__MULTI_CAMERA_PLUGIN_HPP_

#include <gazebo/common/Plugin.hh>
#include <gazebo/sensors/MultiCameraSensor.hh>
#include <gazebo/util/system.hh>

#include <memory>
#include <string>
#include <vector>

namespace gazebo_plugins
{
class MultiCameraPluginPrivate;

class GAZEBO_VISIBLE MultiCameraPlugin : public gazebo::SensorPlugin
{
public:
  /// Constructor
  MultiCameraPlugin();

  /// Destructor
  virtual ~MultiCameraPlugin();

protected:
  // Documentation inherited
  virtual void Load(gazebo::sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);

  /// Callback when multi camera produces a new image.
  /*
   * \details This is called at the camera's update rate.
   * \details Not called when the camera isn't active. For a triggered camera, it will only be
   * called after triggered.
   * \param[in] _image Image
   * \param[in] _width Image width
   * \param[in] _height Image height
   * \param[in] _depth Image depth
   * \param[in] _format Image format
   * \param[in] _camera_num Index number of camera
   */
  virtual void OnNewMultiFrame(
    const unsigned char * _image,
    unsigned int _width, unsigned int _height,
    unsigned int _depth, const std::string & _format, const int _camera_num);

  // Pointer to multicamera sensor
  gazebo::sensors::MultiCameraSensorPtr parent_sensor_;

  // Store dimensions of camera image
  std::vector<unsigned int> width_, height_, depth_;

  // Store image format_
  std::vector<std::string> format_;

  // Pointer to multicamera
  std::vector<gazebo::rendering::CameraPtr> camera_;

private:
  /// Private data pointer
  std::unique_ptr<MultiCameraPluginPrivate> impl_;
};
}  // namespace gazebo_plugins

#endif  // GAZEBO_PLUGINS__MULTI_CAMERA_PLUGIN_HPP_
