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

#ifndef GAZEBO_ROS__CONVERSIONS__GEOMETRY_MSGS_HPP_
#define GAZEBO_ROS__CONVERSIONS__GEOMETRY_MSGS_HPP_

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>

#include "gazebo_ros/conversions/generic.hpp"

namespace gazebo_ros
{

/// Generic conversion from a ROS geometry vector message to another type.
/// \param[in] in Input message.
/// \return Conversion result
/// \tparam OUT Output type
template<class OUT>
OUT Convert(const geometry_msgs::msg::Vector3 & in)
{
  OUT::ConversionNotImplemented;
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

/// Generic conversion from a ROS geometry point32 message to another type.
/// \param[in] in Input message.
/// \return Conversion result
/// \tparam OUT Output type
template<class OUT>
OUT Convert(const geometry_msgs::msg::Point32 & in)
{
  OUT::ConversionNotImplemented;
}

/// \brief Specialized conversion from a ROS point32 message to an Ignition Math vector.
/// \param[in] in ROS message to convert.
/// \return An Ignition Math vector.
template<>
ignition::math::Vector3d Convert(const geometry_msgs::msg::Point32 & in)
{
  ignition::math::Vector3d vec;
  vec.X(in.x);
  vec.Y(in.y);
  vec.Z(in.z);
  return vec;
}

/// Generic conversion from a ROS geometry point message to another type.
/// \param[in] in Input message.
/// \return Conversion result
/// \tparam OUT Output type
template<class OUT>
OUT Convert(const geometry_msgs::msg::Point &)
{
  OUT::ConversionNotImplemented;
}

/// TODO(louise) This may already exist somewhere else, since it's within the same lib
/// \brief Specialized conversion from a ROS point message to a ROS vector message.
/// \param[in] in ROS message to convert.
/// \return A ROS vector message.
template<>
geometry_msgs::msg::Vector3 Convert(const geometry_msgs::msg::Point & in)
{
  geometry_msgs::msg::Vector3 msg;
  msg.x = in.x;
  msg.y = in.y;
  msg.z = in.z;
  return msg;
}

/// \brief Specialized conversion from a ROS point message to an Ignition math vector.
/// \param[in] in ROS message to convert.
/// \return A ROS vector message.
template<>
ignition::math::Vector3d Convert(const geometry_msgs::msg::Point & in)
{
  ignition::math::Vector3d out;
  out.X(in.x);
  out.Y(in.y);
  out.Z(in.z);
  return out;
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

/// \brief Specialized conversion from an Ignition Math vector to a ROS message.
/// \param[in] vec Ignition vector to convert.
/// \return ROS geometry point message
template<>
geometry_msgs::msg::Point Convert(const ignition::math::Vector3d & vec)
{
  geometry_msgs::msg::Point msg;
  msg.x = vec.X();
  msg.y = vec.Y();
  msg.z = vec.Z();
  return msg;
}

/// \brief Specialized conversion from an Ignition Math Quaternion to a ROS message.
/// \param[in] in Ignition Quaternion to convert.
/// \return ROS geometry quaternion message
template<>
geometry_msgs::msg::Quaternion Convert(const ignition::math::Quaterniond & in)
{
  geometry_msgs::msg::Quaternion msg;
  msg.x = in.X();
  msg.y = in.Y();
  msg.z = in.Z();
  msg.w = in.W();
  return msg;
}

/// Generic conversion from a ROS Quaternion message to another type
/// \param[in] in Input quaternion
/// \return Conversion result
/// \tparam OUT Output type
template<class OUT>
OUT Convert(const geometry_msgs::msg::Quaternion & in)
{
  OUT::ConversionNotImplemented;
}

/// \brief Specialized conversion from a ROS quaternion message to ignition quaternion
/// \param[in] in Input quaternion message
/// \return Ignition math quaternion with same values as the input message
template<>
ignition::math::Quaterniond Convert(const geometry_msgs::msg::Quaternion & in)
{
  return ignition::math::Quaterniond(in.w, in.x, in.y, in.z);
}

}  // namespace gazebo_ros
#endif  // GAZEBO_ROS__CONVERSIONS__GEOMETRY_MSGS_HPP_
