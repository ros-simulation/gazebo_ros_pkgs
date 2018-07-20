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

#include "gazebo_ros/gazebo_ros_init.hpp"

#include <rclcpp/rclcpp.hpp>
#include <gazebo/common/Plugin.hh>

GazeboRosInit::GazeboRosInit()
{
}

GazeboRosInit::~GazeboRosInit()
{
}

void GazeboRosInit::Load(int argc, char ** argv)
{
  // Initialize ROS with arguments
  rclcpp::init(argc, argv);
  RCLCPP_INFO(rclcpp::get_logger("gazebo_ros_init"), "ROS has been initialized.");
}

GZ_REGISTER_SYSTEM_PLUGIN(GazeboRosInit)
