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

#include <gazebo_ros/utils.hpp>
#include <gazebo/sensors/GaussianNoiseModel.hh>

#include <string>

namespace gazebo_ros
{

double NoiseVariance(const gazebo::sensors::Noise & _noise)
{
  if (gazebo::sensors::Noise::NoiseType::GAUSSIAN == _noise.GetNoiseType()) {
    auto gm = dynamic_cast<const gazebo::sensors::GaussianNoiseModel &>(_noise);
    return gm.GetStdDev() * gm.GetStdDev();
  }
  return 0.;
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

}  // namespace gazebo_ros
