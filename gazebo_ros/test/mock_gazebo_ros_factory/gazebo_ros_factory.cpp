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
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "gazebo_msgs/srv/spawn_entity.hpp"

using namespace std::chrono_literals;

void SpawnEntityRequest(
  gazebo_msgs::srv::SpawnEntity::Request::SharedPtr req,
  gazebo_msgs::srv::SpawnEntity::Response::SharedPtr res)
{
  const std::string xmlnsParam = "<test:parameter>content</test:parameter>";
  if (std::string::npos == req->xml.find(xmlnsParam)) {
    res->status_message = "Unable to find '" + xmlnsParam + "' in " + req->xml;
    res->success = false;
    return;
  }
  res->status_message = "Success";
  res->success = true;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("mock_gazebo_ros_factory");
  auto service = node->create_service<gazebo_msgs::srv::SpawnEntity>(
    "/spawn_entity",
    std::bind(
      &SpawnEntityRequest, std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(node->get_logger(), "Listening to /spawn_entity");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
