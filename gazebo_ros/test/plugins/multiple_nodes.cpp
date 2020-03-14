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

#include <gazebo/common/Plugin.hh>

#include <std_msgs/msg/string.hpp>
#include <gazebo_ros/node.hpp>

#include <memory>

/// Simple example of a gazebo system plugin which uses a ROS2 node with gazebo_ros::Node.
class MultipleNodes : public gazebo::SystemPlugin
{
public:
  /// Called by gazebo to load plugin. Creates #node, #timer_, and #pub
  /// \param[in] argc Argument count.
  /// \param[in] argv Argument values.
  void Load(int argc, char ** argv);

private:
  /// Timer called to publish a message every second
  std::shared_ptr<rclcpp::TimerBase> timer_;
};

void MultipleNodes::Load(int argc, char ** argv)
{
  // Initialize ROS with arguments
  rclcpp::init(argc, argv);

  // Create GazeboNodes (may be referencing the same ROS 2 node underneath)
  auto nodeA = gazebo_ros::Node::Get();
  assert(nullptr != nodeA);

  auto nodeB = gazebo_ros::Node::Get();
  assert(nullptr != nodeB);

  // Create publishers
  auto pubA = nodeA->create_publisher<std_msgs::msg::String>(
    "testA",
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local());
  auto pubB = nodeB->create_publisher<std_msgs::msg::String>(
    "testB",
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local());

  // Run lambdas every 1 second
  using namespace std::chrono_literals;
  timer_ = nodeA->create_wall_timer(
    1s,
    [nodeA, nodeB, pubA, pubB]() {
      // Create string message
      auto msg = std_msgs::msg::String();
      msg.data = "Hello world";

      // Warn with this node's name (to test logging)
      RCLCPP_WARN(nodeA->get_logger(), "Publishing A");

      // Publish message
      pubA->publish(msg);

      RCLCPP_WARN(nodeB->get_logger(), "Publishing B");
      pubB->publish(msg);
    });
}

GZ_REGISTER_SYSTEM_PLUGIN(MultipleNodes)
