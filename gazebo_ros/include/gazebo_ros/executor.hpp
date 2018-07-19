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

#ifndef GAZEBO_ROS__EXECUTOR_HPP_
#define GAZEBO_ROS__EXECUTOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <gazebo/common/Events.hh>

namespace gazebo_ros
{

/// Executor run in a separate thread to handle events from all #gazebo_ros::Node instances
/**
 * \class Executor executor.hpp <gazebo_ros/executor.hpp>
 */
class Executor : public rclcpp::executors::MultiThreadedExecutor
{
public:
  /// Create an instance and start the internal thread
  Executor();

  /// Stops the internal executor and joins with the internal thread
  virtual ~Executor();

private:
  /// Thread where the executor spins until destruction
  std::thread spin_thread_;

  /// Spin executor, called in #spin_thread_
  void run();

  /// Shutdown ROS, called when gazebo sigint handle arrives so ROS is shutdown cleanly
  void shutdown();

  /// Connection to gazebo sigint event
  gazebo::event::ConnectionPtr sigint_handle_;
};
}  // namespace gazebo_ros
#endif  // GAZEBO_ROS__EXECUTOR_HPP_
