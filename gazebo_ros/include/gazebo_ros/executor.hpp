// Copyright 2014 Open Source Robotics Foundation, Inc.
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

#include <rclcpp/executor.hpp>
#include <gazebo/common/Events.hh>

#include <chrono>
#include <memory>
#include <mutex>
#include <set>
#include <thread>

namespace gazebo_ros
{

class Executor : public rclcpp::Executor
{
public:
  /// Create an instance and start the internal thread
  Executor();

  /// Stops the internal executor and joins with the internal thread
  virtual ~Executor();

  /// Add a node to the executor.
  /**
   * This is a thread-safe wrapper around rclcpp::Executor::add_node.
   *
   * \see rclcpp::Executor::add_node
   */
  virtual void
  add_node(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr, bool notify = true);

  /// Convenience function which takes Node and forwards NodeBaseInterface.
  /**
   * This is a thread-safe wrapper around rclcpp::Executor::add_node.
   *
   * \see rclcpp::Executor::add_node
   */
  virtual void add_node(std::shared_ptr<rclcpp::Node> node_ptr, bool notify = true);

  /// Remove a node from the executor.
  /**
   * This is a thread-safe wrapper around rclcpp::Executor::remove_node.
   *
   * \see rclcpp::Executor::remove_node
   */
  virtual void
  remove_node(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr, bool notify = true);

  /// Convenience function which takes Node and forwards NodeBaseInterface.
  /**
   * This is a thread-safe wrapper around rclcpp::Executor::remove_node.
   *
   * \see rclcpp::Executor::remove_node
   */
  virtual void remove_node(std::shared_ptr<rclcpp::Node> node_ptr, bool notify = true);

  /**
   * \sa rclcpp::Executor:spin() for more details
   * \throws std::runtime_error when spin() called while already spinning
   */
  void spin() override;

private:
  /// Spin executor, called in #spin_thread_
  void run();

  /// Shutdown ROS, called when gazebo sigint handle arrives so ROS is shutdown cleanly
  void shutdown();

  /// Thread where the executor spins until destruction
  std::thread spin_thread_;

  std::mutex wait_mutex_;
  size_t number_of_threads_;
  std::chrono::nanoseconds next_exec_timeout_;
  std::set<rclcpp::TimerBase::SharedPtr> scheduled_timers_;

  /// Connection to gazebo sigint event
  gazebo::event::ConnectionPtr sigint_handle_;
};

}  // namespace gazebo_ros

#endif  // GAZEBO_ROS__EXECUTOR_HPP_
