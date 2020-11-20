// Copyright 2015 Open Source Robotics Foundation, Inc.
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

#include <gazebo_ros/executor.hpp>

#include <chrono>
#include <functional>
#include <memory>
#include <thread>
#include <vector>

#include "rclcpp/node.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp/scope_exit.hpp"

namespace gazebo_ros
{

Executor::Executor()
: next_exec_timeout_(std::chrono::nanoseconds(100))
{
  number_of_threads_ = std::thread::hardware_concurrency();
  if (number_of_threads_ == 0) {
    number_of_threads_ = 1;
  }

  spin_thread_ = std::thread(std::bind(&Executor::spin, this));

  sigint_handle_ = gazebo::event::Events::ConnectSigInt(std::bind(&Executor::shutdown, this));
}

Executor::~Executor()
{
  // If ros was not already shutdown by SIGINT handler, do it now
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  spin_thread_.join();
}

void Executor::shutdown()
{
  rclcpp::shutdown();
}

void
Executor::add_node(
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr, bool notify)
{
  std::lock_guard<std::mutex> wait_lock(wait_mutex_);
  rclcpp::Executor::add_node(node_ptr, notify);
}

void
Executor::add_node(std::shared_ptr<rclcpp::Node> node_ptr, bool notify)
{
  this->add_node(node_ptr->get_node_base_interface(), notify);
}

void
Executor::remove_node(
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr, bool notify)
{
  std::lock_guard<std::mutex> wait_lock(wait_mutex_);
  rclcpp::Executor::remove_node(node_ptr, notify);
}

void
Executor::remove_node(std::shared_ptr<rclcpp::Node> node_ptr, bool notify)
{
  this->remove_node(node_ptr->get_node_base_interface(), notify);
}

void
Executor::spin()
{
  if (spinning.exchange(true)) {
    throw std::runtime_error("spin() called while already spinning");
  }
  RCLCPP_SCOPE_EXIT(this->spinning.store(false); );
  std::vector<std::thread> threads;
  size_t thread_id = 0;
  {
    for (; thread_id < number_of_threads_ - 1; ++thread_id) {
      auto func = std::bind(&Executor::run, this);
      threads.emplace_back(func);
    }
  }

  run();
  for (auto & thread : threads) {
    thread.join();
  }
}

void
Executor::run()
{
  while (rclcpp::ok(this->context_) && spinning.load()) {
    rclcpp::AnyExecutable any_exec;
    std::lock_guard<std::mutex> wait_lock(wait_mutex_);
    if (!rclcpp::ok(this->context_) || !spinning.load()) {
      return;
    }

    if (!get_next_executable(any_exec, next_exec_timeout_)) {
      continue;
    }

    if (any_exec.timer) {
      // Guard against multiple threads getting the same timer.
      if (scheduled_timers_.count(any_exec.timer) != 0) {
        // Make sure that any_exec's callback group is reset before
        // the lock is released.
        if (any_exec.callback_group) {
          any_exec.callback_group->can_be_taken_from().store(true);
        }
        continue;
      }
      scheduled_timers_.insert(any_exec.timer);
    }

    execute_any_executable(any_exec);

    if (any_exec.timer) {
      auto it = scheduled_timers_.find(any_exec.timer);
      if (it != scheduled_timers_.end()) {
        scheduled_timers_.erase(it);
      }
    }
    // Clear the callback_group to prevent the AnyExecutable destructor from
    // resetting the callback group `can_be_taken_from`
    any_exec.callback_group.reset();
  }
}

}  // namespace gazebo_ros
