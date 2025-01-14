// Copyright 2018 Open Source Robotics Foundation, Inc.
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

#include <gazebo/common/CommonIface.hh>
#include <gazebo_ros/utils.hpp>
#include <gazebo/sensors/GaussianNoiseModel.hh>

#include <cstring>
#include <string>

namespace gazebo_ros
{

double NoiseVariance(const gazebo::sensors::Noise & _noise)
{
  if (gazebo::sensors::Noise::NoiseType::GAUSSIAN == _noise.GetNoiseType()) {
    auto gm = dynamic_cast<const gazebo::sensors::GaussianNoiseModel &>(_noise);
    return gm.GetStdDev() * gm.GetStdDev();
  } else if (gazebo::sensors::Noise::NoiseType::NONE == _noise.GetNoiseType()) {
    return 0.;
  }
  return -1.;
}

double NoiseVariance(const gazebo::sensors::NoisePtr & _noise_ptr)
{
  if (!_noise_ptr) {return 0.;}
  return NoiseVariance(*_noise_ptr);
}

std::string ScopedNameBase(const std::string & str)
{
  // Get index of last :: scope marker
  auto idx = str.rfind("::");
  // If not found or at end, return original string
  if (std::string::npos == idx || (idx + 2) >= str.size()) {
    return str;
  }
  // Otherwise return part after last scope marker
  return str.substr(idx + 2);
}

std::string SensorFrameID(const gazebo::sensors::Sensor & _sensor, const sdf::Element & _sdf)
{
  // Return frame from sdf tag if present
  if (_sdf.HasElement("frame_name")) {
    return _sdf.Get<std::string>("frame_name");
  }

  // Otherwise return the unscoped parent link name
  return gazebo_ros::ScopedNameBase(_sensor.ParentName());
}

Throttler::Throttler(const double _hz)
: period_(1.0 / _hz),
  last_time_(0, 0)
{
}

bool Throttler::IsReady(const gazebo::common::Time & _now)
{
  // If time went backwards, reset
  if (_now < last_time_) {
    last_time_ = _now;
    return true;
  }

  // If not enough time has passed, return false
  if (period_ > 0 && (_now - last_time_).Double() < period_) {
    return false;
  }

  // Enough time has passed, set last time to now and return true
  last_time_ = _now;
  return true;
}

bool ShouldDisplayEOLNotice()
{
  // Gazebo 11.15.0 and later contain the EOL warning in the core, so there's
  // no need to add the notice here.
  if (GAZEBO_MAJOR_VERSION == 11 && GAZEBO_MINOR_VERSION >= 15) {
    return false;
  }

  const char * ignoreEol = gazebo::common::getEnv("GAZEBO_SUPPRESS_EOL_WARNING");
  return ignoreEol == nullptr || strncmp(ignoreEol, "1", 1) != 0;
}

}  // namespace gazebo_ros
