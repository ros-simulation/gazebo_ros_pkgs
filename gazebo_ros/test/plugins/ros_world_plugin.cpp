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

/// Simple example of a gazebo world plugin which uses a ROS2 node with gazebo_ros::Node.
class RosWorldPlugin : public gazebo::WorldPlugin
{
public:
  /// Called by gazebo to load plugin. Creates #node, #timer_, and #pub
  /// \param[in] world Pointer to gazebo world this plugin is attached to
  /// \param[in] sdf SDF of plugin
  void Load(gazebo::physics::WorldPtr world, sdf::ElementPtr sdf);

private:
  /// Timer called to publish a message every second
  std::shared_ptr<rclcpp::TimerBase> timer_;
};

void RosWorldPlugin::Load(gazebo::physics::WorldPtr, sdf::ElementPtr _sdf)
{
  // It should be ok to create a node without calling init first.
  auto node = gazebo_ros::Node::Get(_sdf);
  assert(nullptr != node);

  // Create a publisher
  auto pub = node->create_publisher<std_msgs::msg::String>(
    "test",
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local());

  // Run lambda every 1 second
  using namespace std::chrono_literals;
  timer_ = node->create_wall_timer(
    1s,
    [node, pub]() {
      // Create string message
      auto msg = std_msgs::msg::String();
      msg.data = "Hello world, literally";

      // Warn with this node's name (to test logging)
      RCLCPP_WARN(node->get_logger(), "Publishing");

      // Publish message
      pub->publish(msg);
    });
}

GZ_REGISTER_WORLD_PLUGIN(RosWorldPlugin)
