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

#ifndef GAZEBO_ROS_PLUGIN_EXAMPLE_HPP_
#define GAZEBO_ROS_PLUGIN_EXAMPLE_HPP_

#include <gazebo/common/common.hh>

#include <std_msgs/msg/string.hpp>
#include <gazebo_ros/node.hpp>

#include <memory>

/// Simple example of a gazebo plugin which uses a ROS2 node with gazebo_ros::Node
class GazeboRosNodeExample : public gazebo::SystemPlugin
{
public:
  GazeboRosNodeExample();
  virtual ~GazeboRosNodeExample();
  /// Called by gazebo to load plugin. Creates #node_, #timer_, and #pub_
  void Load(int argc, char ** argv);

private:
  /// ROS node used for publisher and timer
  gazebo_ros::Node::SharedPtr node_;
  /// Timer called to publish a message every second
  std::shared_ptr<rclcpp::TimerBase> timer_;
  /// Example publisher
  std::shared_ptr<rclcpp::Publisher<std_msgs::msg::String>> pub_;
};

#endif  // GAZEBO_ROS_PLUGIN_EXAMPLE_HPP_
