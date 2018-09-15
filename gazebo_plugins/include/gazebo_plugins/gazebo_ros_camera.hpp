// Copyright 2012 Open Source Robotics Foundation
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

#ifndef GAZEBO_PLUGINS__GAZEBO_ROS_CAMERA_HPP_
#define GAZEBO_PLUGINS__GAZEBO_ROS_CAMERA_HPP_

#include <gazebo/plugins/CameraPlugin.hh>
#include <std_msgs/msg/empty.hpp>

#include <memory>
#include <string>

namespace gazebo_plugins
{
class GazeboRosCameraPrivate;

/// A plugin that publishes raw images and camera info for generic camera sensors.
class GazeboRosCamera : public gazebo::CameraPlugin
{
public:
  /// Constructor
  GazeboRosCamera();

  /// Destructor
  ~GazeboRosCamera();

protected:
  // Documentation inherited
  void Load(gazebo::sensors::SensorPtr _sensor, sdf::ElementPtr _sdf) override;

  /// Callback when camera produces a new image.
  /*
   * \details This is called at the camera's update rate.
   * \details Not called when the camera isn't active. For a triggered camera, it will only be
   * called after triggered.
   * \param[in] _image
   * \param[in] _width
   * \param[in] _height
   * \param[in] _depth
   * \param[in] _format
   */
  virtual void OnNewFrame(const unsigned char *_image,
                          unsigned int _width, unsigned int _height,
                          unsigned int _depth, const std::string &_format) override;

  /// Callback when camera is triggered.
  void OnTrigger(const std_msgs::msg::Empty::SharedPtr _dummy);

  /// Callback on pre-render event.
  void PreRender();

  /// Enables or disables the camera so it produces messages or not.
  /// param[in] _enabled True to enable.
  void SetCameraEnabled(const bool _enabled);

private:
  /// Private data pointer
  std::unique_ptr<GazeboRosCameraPrivate> impl_;
};
}  // namespace gazebo_plugins

#endif  // GAZEBO_PLUGINS__GAZEBO_ROS_CAMERA_HPP_

