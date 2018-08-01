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

#ifndef GAZEBO_ROS__UTILS_HPP_
#define GAZEBO_ROS__UTILS_HPP_

#include <gazebo/sensors/Noise.hh>

#include <string>

namespace gazebo_ros
{

/// Get the variance of a gazebo sensor noise model
/// \param[in] _noise The gazebo noise model
/// \return The square of the standard deviation if the model is gaussian, otherwise 0
double NoiseVariance(const gazebo::sensors::Noise & _noise);

/// Gets the base name of a gazebo scoped name
/// \details Example: given "my_world::my_robot::my_link", returns "my_link"
/// \param[in] str Input scoped name, see example
/// \return Input string with all base scopes removed, see example
/// \todo Deprecate once with is implemented in gazebo/ignition/sdf.
///       See: https://bitbucket.org/osrf/gazebo/issues/1735/add-helper-functions-to-handle-scoped
std::string ScopedNameBase(const std::string & str);

}  // namespace gazebo_ros
#endif  // GAZEBO_ROS__UTILS_HPP_
