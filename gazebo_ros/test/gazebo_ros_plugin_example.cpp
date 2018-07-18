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

#include "gazebo_ros_plugin_example.hpp"

GazeboRosNodeExample::GazeboRosNodeExample()
{
}

GazeboRosNodeExample::~GazeboRosNodeExample()
{
}

void GazeboRosNodeExample::Load(int argc, char ** argv)
{
  // Initialize ROS with arguments
  gazebo_ros::Node::InitROS(argc, argv);

  // Create the ROS node
  node_ = gazebo_ros::Node::Create("gazebo_ros_node_example");

  // Create a publisher
  pub_ = node_->create_publisher<std_msgs::msg::String>("test");

  // Run lambda every 1 second
  using namespace std::chrono_literals;
  timer_ = node_->create_wall_timer(1s,
      [this]() {
        // Create string message
        auto msg = std_msgs::msg::String();
        msg.data = "Hello world";

        // Warn with this node's name (to test logging)
        RCLCPP_WARN(node_->get_logger(), "Publishing");

        // Publish message
        this->pub_->publish(msg);
      });
}

GZ_REGISTER_SYSTEM_PLUGIN(GazeboRosNodeExample)
