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

#ifndef GAZEBO_ROS__CONVERSIONS__SENSOR_MSGS_HPP_
#define GAZEBO_ROS__CONVERSIONS__SENSOR_MSGS_HPP_

#include <math.h>

#include <gazebo/msgs/laserscan_stamped.pb.h>
#include <geometry_msgs/msg/point32.hpp>
#include <rclcpp/logging.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <algorithm>
#include <limits>

#include "gazebo_ros/conversions/builtin_interfaces.hpp"
#include "gazebo_ros/conversions/generic.hpp"

namespace gazebo_ros
{
/// Generic conversion from an Gazebo Laser Scan message to another type.
/// \param[in] in Input message;
/// \param[in] min_intensity The minimum intensity value to clip the output intensities
/// \return Conversion result
/// \tparam T Output type
template<class T>
inline
T Convert(const gazebo::msgs::LaserScanStamped &, double min_intensity = 0.0)
{
  (void)min_intensity;
  T::ConversionNotImplemented;
}

/// \brief Specialized conversion from an Gazebo Laser Scan to a ROS Laser Scan.
/// \param[in] in Input message;
/// \param[in] min_intensity The minimum intensity value to clip the output intensities
/// \return A ROS Laser Scan message with the same data as the input message
/// \note If multiple vertical rays are present, the LaserScan will be the
///       horizontal scan in the center of the vertical range
template<>
inline
sensor_msgs::msg::LaserScan Convert(const gazebo::msgs::LaserScanStamped & in, double min_intensity)
{
  sensor_msgs::msg::LaserScan ls;
  ls.header.stamp = Convert<builtin_interfaces::msg::Time>(in.time());
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
  ls.ranges.resize(count);
  std::copy(
    in.scan().ranges().begin() + start,
    in.scan().ranges().begin() + start + count,
    ls.ranges.begin());

  // Copy intensities into ROS message, clipping at min_intensity
  ls.intensities.resize(count);
  std::transform(
    in.scan().intensities().begin() + start,
    in.scan().intensities().begin() + start + count,
    ls.intensities.begin(), [min_intensity](double i) -> double {
      return i > min_intensity ? i : min_intensity;
    });

  return ls;
}

/// \brief Specialized conversion from an Gazebo Laser Scan to a ROS PointCloud.
/// \param[in] in Input message;
/// \param[in] min_intensity The minimum intensity value to clip the output intensities
/// \return A ROS PointCloud message with the same data as the input message
template<>
inline
sensor_msgs::msg::PointCloud Convert(
  const gazebo::msgs::LaserScanStamped & in,
  double min_intensity)
{
  // Create message to send
  sensor_msgs::msg::PointCloud pc;

  // Fill header
  pc.header.stamp = Convert<builtin_interfaces::msg::Time>(in.time());

  // Cache values that are repeatedly used
  auto count = in.scan().count();
  auto vertical_count = in.scan().vertical_count();
  auto angle_step = in.scan().angle_step();
  auto vertical_angle_step = in.scan().vertical_angle_step();

  // Gazebo sends an infinite vertical step if the number of samples is 1
  // Surprisingly, not setting the <vertical> tag results in nan instead of inf, which is ok
  if (std::isinf(vertical_angle_step)) {
    RCLCPP_WARN_ONCE(conversions_logger, "Infinite angle step results in wrong PointCloud");
  }

  // Setup point cloud fields
  pc.points.reserve(count * vertical_count);
  pc.channels.push_back(sensor_msgs::msg::ChannelFloat32());
  pc.channels[0].values.reserve(count * vertical_count);
  pc.channels[0].name = "intensity";

  // Iterators to range and intensities
  auto range_iter = in.scan().ranges().begin();
  auto intensity_iter = in.scan().intensities().begin();

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
      ++i, azimuth += angle_step, ++range_iter, ++intensity_iter)
    {
      double c_azimuth = cos(azimuth);
      double s_azimuth = sin(azimuth);

      double r = *range_iter;
      // Skip NaN / inf points
      if (!std::isfinite(r)) {
        continue;
      }

      // Get intensity, clipping at min_intensity
      double intensity = *intensity_iter;
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
inline
sensor_msgs::msg::PointCloud2 Convert(
  const gazebo::msgs::LaserScanStamped & in,
  double min_intensity)
{
  // Create message to send
  sensor_msgs::msg::PointCloud2 pc;

  // Pointcloud will be dense, unordered
  pc.height = 1;
  pc.is_dense = true;

  // Fill header
  pc.header.stamp = Convert<builtin_interfaces::msg::Time>(in.time());

  // Cache values that are repeatedly used
  auto count = in.scan().count();
  auto vertical_count = in.scan().vertical_count();
  auto angle_step = in.scan().angle_step();
  auto vertical_angle_step = in.scan().vertical_angle_step();

  // Gazebo sends an infinite vertical step if the number of samples is 1
  // Surprisingly, not setting the <vertical> tag results in nan instead of inf, which is ok
  if (std::isinf(vertical_angle_step)) {
    RCLCPP_WARN_ONCE(conversions_logger, "Infinite angle step results in wrong PointCloud2");
  }

  // Create fields in pointcloud
  sensor_msgs::PointCloud2Modifier pcd_modifier(pc);
  pcd_modifier.setPointCloud2Fields(
    4,
    "x", 1, sensor_msgs::msg::PointField::FLOAT32,
    "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32,
    "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
  pcd_modifier.resize(vertical_count * count);
  sensor_msgs::PointCloud2Iterator<float> iter_x(pc, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(pc, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(pc, "z");
  sensor_msgs::PointCloud2Iterator<float> iter_intensity(pc, "intensity");

  // Iterators to range and intensities
  auto range_iter = in.scan().ranges().begin();
  auto intensity_iter = in.scan().intensities().begin();

  // Number of points actually added
  size_t points_added = 0;

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
      ++i, azimuth += angle_step, ++range_iter, ++intensity_iter)
    {
      double c_azimuth = cos(azimuth);
      double s_azimuth = sin(azimuth);

      double r = *range_iter;
      // Skip NaN / inf points
      if (!std::isfinite(r)) {
        continue;
      }

      // Get intensity, clipping at min_intensity
      double intensity = *intensity_iter;
      if (intensity < min_intensity) {
        intensity = min_intensity;
      }

      // Convert spherical coordinates to Cartesian for pointcloud
      // See https://en.wikipedia.org/wiki/Spherical_coordinate_system
      *iter_x = r * c_inclination * c_azimuth;
      *iter_y = r * c_inclination * s_azimuth;
      *iter_z = r * s_inclination;
      *iter_intensity = intensity;

      // Increment ouput iterators
      ++points_added;
      ++iter_x;
      ++iter_y;
      ++iter_z;
      ++iter_intensity;
    }
  }

  pcd_modifier.resize(points_added);

  return pc;
}

/// \brief Specialized conversion from an Gazebo Laser Scan to a ROS PointCloud2.
/// \param[in] in Input message;
/// \param[in] min_intensity Ignored.
/// \return A ROS Range message with minimum range of the rays in the laser scan
template<>
inline
sensor_msgs::msg::Range Convert(const gazebo::msgs::LaserScanStamped & in, double min_intensity)
{
  (void) min_intensity;

  // Create message
  sensor_msgs::msg::Range range_msg;

  // Set stamp from header
  range_msg.header.stamp = Convert<builtin_interfaces::msg::Time>(in.time());

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
#endif  // GAZEBO_ROS__CONVERSIONS__SENSOR_MSGS_HPP_
