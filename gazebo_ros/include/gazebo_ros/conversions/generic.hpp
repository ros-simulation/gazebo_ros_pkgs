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

#ifndef GAZEBO_ROS__CONVERSIONS__GENERIC_HPP_
#define GAZEBO_ROS__CONVERSIONS__GENERIC_HPP_

#include <math.h>

#include <gazebo/common/Time.hh>
#include <gazebo/msgs/time.pb.h>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>
#include <rclcpp/logging.hpp>
#include <rclcpp/time.hpp>

namespace gazebo_ros
{
/// \brief Common logger for all conversions
static rclcpp::Logger conversions_logger = rclcpp::get_logger("gazebo_ros_conversions");

/// Generic conversion from an Ignition Math vector to another type.
/// \param[in] in Input vector.
/// \return Conversion result
/// \tparam T Output type
template<class T>
inline
T Convert(const ignition::math::Vector3d &)
{
  T::ConversionNotImplemented;
}

/// Generic conversion from an Ignition Math quaternion to another type.
/// \param[in] in Input quaternion
/// \return Conversion result
/// \tparam T Output type
template<class T>
inline
T Convert(const ignition::math::Quaterniond &)
{
  T::ConversionNotImplemented;
}

/// Generic conversion from an Ignition Math pose3d to another type.
/// \param[in] in Input pose3d
/// \return Conversion result
/// \tparam T Output type
template<class T>
inline
T Convert(const ignition::math::Pose3d &)
{
  T::ConversionNotImplemented;
}

/// Generic conversion from an Gazebo Time object to another type.
/// \param[in] in Input time;
/// \return Conversion result
/// \tparam T Output type
template<class T>
inline
T Convert(const gazebo::common::Time &)
{
  T::ConversionNotImplemented;
}

/// \brief Specialized conversion from an Gazebo Time to a RCLCPP Time.
/// \param[in] in Gazebo Time to convert.
/// \return A rclcpp::Time object with the same value as in
template<>
inline
rclcpp::Time Convert(const gazebo::common::Time & in)
{
  return rclcpp::Time(in.sec, in.nsec, rcl_clock_type_t::RCL_ROS_TIME);
}

/// Generic conversion from an Gazebo Time message to another type.
/// \param[in] in Input time
/// \return Conversion result
/// \tparam T Output type
template<class T>
inline
T Convert(const gazebo::msgs::Time &)
{
  T::ConversionNotImplemented;
}

}  // namespace gazebo_ros
#endif  // GAZEBO_ROS__CONVERSIONS__GENERIC_HPP_
