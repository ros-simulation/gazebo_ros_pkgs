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

#include <builtin_interfaces/msg/time.hpp>
#include <gazebo/common/Time.hh>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>
#include <rclcpp/time.hpp>

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

/// Generic conversion from an Ignition Math quaternion to another type.
/// \param[in] in Input quaternion
/// \return Conversion result
/// \tparam OUT Output type
template<class OUT>
OUT Convert(const ignition::math::Quaterniond & in)
{
  return OUT();
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
  return OUT();
}

/// \brief Specialized conversion from a ROS quaternion message to ignition quaternion
/// \param[in] in Input quaternion message
/// \return Ignition math quaternion with same values as the input message
template<>
ignition::math::Quaterniond Convert(const geometry_msgs::msg::Quaternion & in)
{
  return ignition::math::Quaterniond(in.w, in.x, in.y, in.z);
}

/// Generic conversion from an Gazebo Time object to another type.
/// \param[in] in Input time;
/// \return Conversion result
/// \tparam OUT Output type
template<class OUT>
OUT Convert(const gazebo::common::Time & in)
{
  return OUT();
}

/// \brief Specialized conversion from an Gazebo Time to a RCLCPP Time.
/// \param[in] in Gazebo Time to convert.
/// \return A rclcpp::Time object with the same value as in
template<>
rclcpp::Time Convert(const gazebo::common::Time & in)
{
  return rclcpp::Time(in.sec, in.nsec, rcl_clock_type_t::RCL_ROS_TIME);
}

/// \brief Specialized conversion from an Gazebo Time to a ROS Time message.
/// \param[in] in Gazebo Time to convert.
/// \return A ROS Time message with the same value as in
template<>
builtin_interfaces::msg::Time Convert(const gazebo::common::Time & in)
{
  builtin_interfaces::msg::Time time;
  time.sec = in.sec;
  time.nanosec = in.nsec;
  return time;
}

}  // namespace gazebo_ros
#endif  // GAZEBO_ROS__CONVERSIONS_HPP_
