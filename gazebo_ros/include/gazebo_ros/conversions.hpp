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
#include <gazebo/msgs/laserscan_stamped.pb.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <string>
#include <algorithm>
#include <limits>

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

/// Generic conversion from an Gazebo Laser Scan message to another type.
/// \param[in] in Input message;
/// \param[in] min_intensity The minimum intensity value to clip the output intensities
/// \return Conversion result
/// \tparam OUT Output type
template<class OUT>
OUT Convert(const gazebo::msgs::LaserScanStamped & in, double min_intensity = 0.0)
{
  return OUT();
}

/// \brief Specialized conversion from an Gazebo Laser Scan to a ROS Laser Scan.
/// \param[in] in Input message;
/// \param[in] min_intensity The minimum intensity value to clip the output intensities
/// \return A ROS Laser Scan message with the same data as the input message
/// \note If multiple vertical rays are present, the LaserScan will be the
///       horizontal scan in the center of the vertical range
template<>
sensor_msgs::msg::LaserScan Convert(const gazebo::msgs::LaserScanStamped & in, double min_intensity)
{
  sensor_msgs::msg::LaserScan ls;
  ls.header.stamp.sec = in.time().sec();
  ls.header.stamp.nanosec = in.time().nsec();
  ls.angle_min = in.scan().angle_min();
  ls.angle_max = in.scan().angle_max();
  ls.angle_increment = in.scan().angle_step();
  ls.time_increment = 0;
  ls.scan_time = 0;
  ls.range_min = in.scan().range_min();
  ls.range_max = in.scan().range_max();

  auto count = in.scan().count();
  auto vertical_count = in.scan().vertical_count();

  // If there are multiple vertical beams, use the one in the middle
  size_t start = (vertical_count / 2) * count;

  // Copy ranges into ROS message
  ls.ranges.reserve(count);
  std::copy(
    in.scan().ranges().begin() + start,
    in.scan().ranges().begin() + start + count,
    ls.ranges.begin());

  // Copy intensities into ROS message, clipping at min_intensity
  ls.intensities.reserve(count);
  std::transform(
    in.scan().intensities().begin() + start,
    in.scan().intensities().begin() + start + count,
    ls.intensities.begin(), [min_intensity](double i) -> double {
      return i ? i > min_intensity : min_intensity;
    });

  return ls;
}

/// \brief Specialized conversion from an Gazebo Laser Scan to a ROS PointCloud.
/// \param[in] in Input message;
/// \param[in] min_intensity The minimum intensity value to clip the output intensities
/// \return A ROS PointCloud message with the same data as the input message
template<>
sensor_msgs::msg::PointCloud Convert(
  const gazebo::msgs::LaserScanStamped & in,
  double min_intensity)
{
  // Create message to send
  sensor_msgs::msg::PointCloud pc;

  // Fill header
  pc.header.stamp.sec = in.time().sec();
  pc.header.stamp.nanosec = in.time().nsec();

  // Cache values that are repeatedly used
  auto count = in.scan().count();
  auto vertical_count = in.scan().vertical_count();
  auto angle_step = in.scan().angle_step();
  auto vertical_angle_step = in.scan().vertical_angle_step();

  // Setup point cloud fields
  pc.points.reserve(count * vertical_count);
  pc.channels.push_back(sensor_msgs::msg::ChannelFloat32());
  pc.channels[0].values.reserve(count * vertical_count);
  pc.channels[0].name = "intensity";

  // Current index in range array
  size_t range_index = 0;
  // Angles of ray currently processing, azimuth is horizontal, inclination is vertical
  double azimuth, inclination;
  // Index in vertical and horizontal loops
  size_t i, j;

  // Fill pointcloud with laser scan data, converting spherical to Cartesian
  for (j = 0, inclination = in.scan().vertical_angle_min();
    j < vertical_count;
    ++j, inclination += vertical_angle_step)
  {
    double c_inclination = cos(inclination);
    double s_inclination = sin(inclination);
    for (i = 0, azimuth = in.scan().angle_min();
      i < count;
      ++i, azimuth += angle_step, ++range_index)
    {
      double c_azimuth = cos(azimuth);
      double s_azimuth = sin(azimuth);

      double r = in.scan().ranges(range_index);
      double intensity = in.scan().intensities(range_index);
      if (intensity < min_intensity) {
        intensity = min_intensity;
      }

      // Convert spherical coordinates to Cartesian for pointcloud
      // See https://en.wikipedia.org/wiki/Spherical_coordinate_system
      geometry_msgs::msg::Point32 point;
      point.x = r * c_inclination * c_azimuth;
      point.y = r * c_inclination * s_azimuth;
      point.z = r * s_inclination;
      pc.points.push_back(point);
      pc.channels[0].values.push_back(intensity);
    }
  }

  return pc;
}

/// \brief Specialized conversion from an Gazebo Laser Scan to a ROS PointCloud2.
/// \param[in] in Input message;
/// \param[in] min_intensity The minimum intensity value to clip the output intensities
/// \return A ROS PointCloud2 message with the same data as the input message
template<>
sensor_msgs::msg::PointCloud2 Convert(
  const gazebo::msgs::LaserScanStamped & in,
  double min_intensity)
{
  // Create message to send
  sensor_msgs::msg::PointCloud2 pc;

  // Fill header
  pc.header.stamp.sec = in.time().sec();
  pc.header.stamp.nanosec = in.time().nsec();

  // Cache values that are repeatedly used
  auto count = in.scan().count();
  auto vertical_count = in.scan().vertical_count();
  auto angle_step = in.scan().angle_step();
  auto vertical_angle_step = in.scan().vertical_angle_step();

  // Create fields in pointcloud
  sensor_msgs::PointCloud2Modifier pcd_modifier(pc);
  pcd_modifier.setPointCloud2Fields(4,
    "x", 1, sensor_msgs::msg::PointField::FLOAT32,
    "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32,
    "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
  pcd_modifier.resize(vertical_count * count);
  pc.is_dense = true;
  sensor_msgs::PointCloud2Iterator<float> iter_x(pc, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(pc, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(pc, "z");
  sensor_msgs::PointCloud2Iterator<float> iter_intensity(pc, "intensity");

  // Current index in range array
  size_t range_index = 0;
  // Angles of ray currently processing, azimuth is horizontal, inclination is vertical
  double azimuth, inclination;
  // Index in vertical and horizontal loops
  size_t i, j;

  // Fill pointcloud with laser scan data, converting spherical to Cartesian
  for (j = 0, inclination = in.scan().vertical_angle_min();
    j < vertical_count;
    ++j, inclination += vertical_angle_step)
  {
    double c_inclination = cos(inclination);
    double s_inclination = sin(inclination);
    for (i = 0, azimuth = in.scan().angle_min();
      i < count;
      ++i, azimuth += angle_step, ++range_index,
      ++iter_x, ++iter_y, ++iter_z, ++iter_intensity)
    {
      double c_azimuth = cos(azimuth);
      double s_azimuth = sin(azimuth);

      double r = in.scan().ranges(range_index);
      double intensity = in.scan().intensities(range_index);
      if (intensity < min_intensity) {
        intensity = min_intensity;
      }

      // Convert spherical coordinates to Cartesian for pointcloud
      // See https://en.wikipedia.org/wiki/Spherical_coordinate_system
      *iter_x = r * c_inclination * c_azimuth;
      *iter_y = r * c_inclination * s_azimuth;
      *iter_z = r * s_inclination;
      *iter_intensity = intensity;
    }
  }

  return pc;
}

/// \brief Specialized conversion from an Gazebo Laser Scan to a ROS PointCloud2.
/// \param[in] in Input message;
/// \param[in] min_intensity Ignored.
/// \return A ROS Range message with minimum range of the rays in the laser scan
template<>
sensor_msgs::msg::Range Convert(const gazebo::msgs::LaserScanStamped & in, double min_intensity)
{
  (void) min_intensity;

  // Create message
  sensor_msgs::msg::Range range_msg;

  // Set stamp from header
  range_msg.header.stamp.sec = in.time().sec();
  range_msg.header.stamp.nanosec = in.time().nsec();

  double horizontal_fov = in.scan().angle_max() - in.scan().angle_min();
  double vertical_fov = in.scan().vertical_angle_max() - in.scan().vertical_angle_min();
  range_msg.field_of_view = std::max(horizontal_fov, vertical_fov);
  range_msg.min_range = in.scan().range_min();
  range_msg.max_range = in.scan().range_max();

  // Set range to the minimum of the ray ranges
  // For single rays, this will just be the range of the ray
  range_msg.range = std::numeric_limits<sensor_msgs::msg::Range::_range_type>::max();
  for (double range : in.scan().ranges()) {
    if (range < range_msg.range) {
      range_msg.range = range;
    }
  }
  return range_msg;
}

}  // namespace gazebo_ros
#endif  // GAZEBO_ROS__CONVERSIONS_HPP_
