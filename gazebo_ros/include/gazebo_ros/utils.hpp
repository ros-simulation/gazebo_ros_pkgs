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

#include <gazebo/common/Time.hh>
#include <gazebo/sensors/Noise.hh>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo_ros/utils_visibility_control.h>

#include <string>

#ifdef _WIN32

#include <chrono>
#include <thread>
#define usleep(usec) (std::this_thread::sleep_for(std::chrono::microseconds(usec)))

#endif  // _WIN32

namespace gazebo_ros
{

/// Get the variance of a gazebo sensor noise model
/// \param[in] _noise The gazebo noise model
/// \return If the model is Gaussian, return the square of the standard deviation.
/// \return If the model is no noise, return 0.
/// \return If the model is custom, return -1
GAZEBO_ROS_UTILS_PUBLIC
double NoiseVariance(const gazebo::sensors::Noise & _noise);

/// Get the variance of a gazebo sensor noise model
/// \param[in] _noise_ptr Shared pointer to the gazebo noise model
/// \return If the pointer is nullptr, return 0.
/// \return Otherwise, returns the same as @ref NoiseVariance(const gazebo::sensors::Noise &).
GAZEBO_ROS_UTILS_PUBLIC
double NoiseVariance(const gazebo::sensors::NoisePtr & _noise_ptr);

/// Gets the base name of a gazebo scoped name
/// \details Example: given "my_world::my_robot::my_link", returns "my_link"
/// \param[in] str Input scoped name, see example
/// \return Input string with all base scopes removed, see example
/// \todo Deprecate once with is implemented in gazebo/ignition/sdf.
///       See: https://bitbucket.org/osrf/gazebo/issues/1735/add-helper-functions-to-handle-scoped
GAZEBO_ROS_UTILS_PUBLIC
std::string ScopedNameBase(const std::string & str);

/// Selects an appropriate tf frame id for a gazebo sensor
/// \details Either returns the name of the sensor's parent link (without the scope)
//           or the value of the sdf tag below if it is present.
/// \code{.xml}
/// <frame_name>my_tf_frame</frame_name>
/// \endcode
/// \param[in] _sensor The gazebo sensor which the frame should be in
/// \param[in] _sdf SDF pointer which may contain a tag to override the frame id
/// \return The string representing the tf frame of the sensor
GAZEBO_ROS_UTILS_PUBLIC
std::string SensorFrameID(const gazebo::sensors::Sensor & _sensor, const sdf::Element & _sdf);

/// Helper class used to throttle something to a given rate
class GAZEBO_ROS_UTILS_PUBLIC Throttler
{
public:
  /// Create a throttler with a frequency
  /// \param[in] _hz Frequency at which IsReady will return true, in hertz
  explicit Throttler(const double _hz);
  /// Check if the enough time has elapsed since the last time IsReady returned true
  /// \param[in] _time The current time.
  /// \return false if not enough time has elapsed from the last time IsReady was true
  /// \return true if enough time has elapsed since the last success, or it is the first time.
  bool IsReady(const gazebo::common::Time & _time);

private:
  /// The time between calls to @IsReady returning true
  double period_;
  /// The last time @IsReady returned true
  gazebo::common::Time last_time_;
};

/// Determines if the Gazebo-classic EOL notice should be displayed
GAZEBO_ROS_UTILS_PUBLIC
bool ShouldDisplayEOLNotice();

}  // namespace gazebo_ros
#endif  // GAZEBO_ROS__UTILS_HPP_
