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

#include <iostream>

namespace gazebo_ros
{

Executor::Executor()
: spin_thread_(std::bind(&Executor::run, this))
{
  using namespace std::chrono_literals;
  sigint_handle_ = gazebo::event::Events::ConnectSigInt(std::bind(&Executor::shutdown, this));
  while (!this->spinning) {
    // TODO(ivanpauno): WARN Terrible hack here!!!!
    // We cannot call rclcpp::shutdown asynchronously, because it generates exceptions that
    // cannot be caught properly (see https://github.com/ros2/rclcpp/issues/1139).
    // Executor::cancel() doesn't cause this problem, but it has a race.
    // Wait until the launched thread starts spinning to avoid the race ...
    std::this_thread::sleep_for(100ms);
  }
}

Executor::~Executor()
{
  // If ros was not already shutdown by SIGINT handler, do it now
  this->shutdown();
  spin_thread_.join();
}

void Executor::run()
{
  spin();
}

void Executor::shutdown()
{
  if (this->spinning) {
    this->cancel();
  }
}

}  // namespace gazebo_ros
