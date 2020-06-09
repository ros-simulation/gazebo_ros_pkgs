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

#include <gazebo_plugins/multi_camera_plugin.hpp>
#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo/sensors/DepthCameraSensor.hh>
#include <gazebo/rendering/Camera.hh>

#include <memory>
#include <string>
#include <vector>

namespace gazebo_plugins
{
class MultiCameraPluginPrivate
{
public:
  /// Connects to pre-render events.
  std::vector<gazebo::event::ConnectionPtr> new_frame_connection_;
};

/////////////////////////////////////////////////
MultiCameraPlugin::MultiCameraPlugin()
: SensorPlugin(),
  impl_(std::make_unique<MultiCameraPluginPrivate>())
{
}

/////////////////////////////////////////////////
MultiCameraPlugin::~MultiCameraPlugin()
{
  for (auto conn : impl_->new_frame_connection_) {
    conn.reset();
  }
  impl_->new_frame_connection_.clear();

  parent_sensor_.reset();
  camera_.clear();
}

/////////////////////////////////////////////////
void MultiCameraPlugin::Load(gazebo::sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/)
{
  if (!_sensor) {
    gzerr << "Invalid sensor pointer.\n";
  }

  parent_sensor_ = std::dynamic_pointer_cast<gazebo::sensors::MultiCameraSensor>(_sensor);

  if (!parent_sensor_) {
    gzerr << "MultiCameraPlugin requires a CameraSensor.\n";
    if (std::dynamic_pointer_cast<gazebo::sensors::DepthCameraSensor>(_sensor)) {
      gzmsg << "It is a depth camera sensor\n";
    }
    if (std::dynamic_pointer_cast<gazebo::sensors::CameraSensor>(_sensor)) {
      gzmsg << "It is a camera sensor\n";
    }
  }

  if (!parent_sensor_) {
    gzerr << "MultiCameraPlugin not attached to a camera sensor\n";
    return;
  }
  for (unsigned int i = 0; i < parent_sensor_->CameraCount(); ++i) {
    camera_.push_back(parent_sensor_->Camera(i));

    // save camera attributes
    width_.push_back(camera_[i]->ImageWidth());
    height_.push_back(camera_[i]->ImageHeight());
    depth_.push_back(camera_[i]->ImageDepth());
    format_.push_back(camera_[i]->ImageFormat());

    impl_->new_frame_connection_.push_back(
      camera_[i]->ConnectNewImageFrame(
        std::bind(
          &MultiCameraPlugin::OnNewMultiFrame,
          this, std::placeholders::_1, std::placeholders::_2,
          std::placeholders::_3, std::placeholders::_4, std::placeholders::_5, i)));
  }

  parent_sensor_->SetActive(true);
}

/////////////////////////////////////////////////
void MultiCameraPlugin::OnNewMultiFrame(
  const unsigned char * /*_image*/,
  unsigned int /*_width*/,
  unsigned int /*_height*/,
  unsigned int /*_depth*/,
  const std::string & /*_format*/,
  const int /*_camera_num*/)
{
}
GZ_REGISTER_SENSOR_PLUGIN(MultiCameraPlugin)
}  // namespace gazebo_plugins
