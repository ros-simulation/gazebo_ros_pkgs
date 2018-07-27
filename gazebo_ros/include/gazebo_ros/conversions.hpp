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

#ifndef GAZEBO_ROS__CONVERSIONS_HPP_
#define GAZEBO_ROS__CONVERSIONS_HPP_

#include <geometry_msgs/msg/vector3.hpp>
#include <ignition/math/Vector3.hh>

namespace gazebo_ros
{
/// Generic conversion from a ROS geometry vector message to another type.
/// \param[in] in Input message.
/// \return Conversion result
/// \tparam OUT Output type
template<class OUT>
OUT Convert(const geometry_msgs::msg::Vector3 & in)
{
  return OUT();
}

/// \brief Specialized conversion from a ROS vector message to an Ignition Math vector.
/// \param[in] msg ROS message to convert.
/// \return An Ignition Math vector.
template<>
ignition::math::Vector3d Convert(const geometry_msgs::msg::Vector3 & msg)
{
  ignition::math::Vector3d vec;
  vec.X(msg.x);
  vec.Y(msg.y);
  vec.Z(msg.z);
  return vec;
}

/// Generic conversion from an Ignition Math vector to another type.
/// \param[in] in Input vector.
/// \return Conversion result
/// \tparam OUT Output type
template<class OUT>
OUT Convert(const ignition::math::Vector3d & in)
{
  return OUT();
}

/// \brief Specialized conversion from an Ignition Math vector to a ROS message.
/// \param[in] vec Ignition vector to convert.
/// \return ROS geometry vector message
template<>
geometry_msgs::msg::Vector3 Convert(const ignition::math::Vector3d & vec)
{
  geometry_msgs::msg::Vector3 msg;
  msg.x = vec.X();
  msg.y = vec.Y();
  msg.z = vec.Z();
  return msg;
}
}  // namespace gazebo_ros
#endif  // GAZEBO_ROS__CONVERSIONS_HPP_
