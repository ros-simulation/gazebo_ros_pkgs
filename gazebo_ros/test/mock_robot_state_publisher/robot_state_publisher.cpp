// Copyright 2020 Open Source Robotics Foundation, Inc.
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

#include <chrono>
#include <iostream>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("mock_robot_state_publisher");
  auto publisher = node->create_publisher<std_msgs::msg::String>(
    "/mock_robot_description", rclcpp::QoS(1).transient_local());

  std_msgs::msg::String message;
  message.data = "<?xml version='1.0' ?>"
    "<sdf version='1.5'>"
    "<model name='ignored'>"
    "<static>true</static>"
    "<link name='link'>"
    "<visual name='visual'>"
    "<geometry>"
    "<sphere><radius>1.0</radius></sphere>"
    "</geometry>"
    "</visual>"
    "</link>"
    "<plugin name='test_xmlns' filename='not-found'"
    "        xmlns:test='http://example.org/schema'>"
    "<test:parameter>content</test:parameter>"
    "</plugin>"
    "</model>"
    "</sdf>";

  RCLCPP_INFO(node->get_logger(), "Publishing on /mock_robot_description");
  publisher->publish(message);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
