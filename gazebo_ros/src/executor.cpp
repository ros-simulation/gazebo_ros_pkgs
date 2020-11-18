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

#include <gazebo_ros/executor.hpp>

#include <rclcpp/node.hpp>

#include <chrono>
#include <iostream>
#include <memory>
#include <mutex>
#include <thread>

namespace gazebo_ros
{

Executor::Executor()
: spin_thread_(std::bind(&Executor::run, this))
{
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

void
Executor::add_node(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr, bool notify)
{
  std::lock_guard<std::mutex> lock(spin_mutex_);
  rclcpp::Executor::add_node(node_ptr, notify);
}

void
Executor::add_node(std::shared_ptr<rclcpp::Node> node_ptr, bool notify)
{
  this->add_node(node_ptr->get_node_base_interface(), notify);
}

void
Executor::remove_node(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr, bool notify)
{
  std::lock_guard<std::mutex> lock(spin_mutex_);
  rclcpp::Executor::remove_node(node_ptr, notify);
}

void
Executor::remove_node(std::shared_ptr<rclcpp::Node> node_ptr, bool notify)
{
  this->remove_node(node_ptr->get_node_base_interface(), notify);
}

void Executor::run()
{
  while (rclcpp::ok()) {
    std::lock_guard<std::mutex> lock(spin_mutex_);
    spin_once(std::chrono::milliseconds(100));
  }
}

void Executor::shutdown()
{
  rclcpp::shutdown();
}

}  // namespace gazebo_ros
