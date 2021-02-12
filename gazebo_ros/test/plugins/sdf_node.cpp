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
#include <rclcpp/exceptions.hpp>

#include <memory>
#include <string>

/// Simple example of a gazebo world plugin which uses a ROS2 node with gazebo_ros::Node.
class SDFNode : public gazebo::WorldPlugin
{
public:
  /// Called by gazebo to load plugin. Creates #node, #timer_, and #pub
  /// \param[in] world Pointer to the gazebo world this plugin is attached to
  /// \param[in] sdf SDF of the plugin this is attached to
  void Load(gazebo::physics::WorldPtr world, sdf::ElementPtr sdf);

private:
  /// Timer called to publish a message every second
  std::shared_ptr<rclcpp::TimerBase> timer_;
};

void SDFNode::Load(gazebo::physics::WorldPtr, sdf::ElementPtr _sdf)
{
  // It should be ok to create a node without calling init first.
  auto node = gazebo_ros::Node::Get(_sdf);
  assert(nullptr != node);

  node->declare_parameter("declared_string", "overridden");
  node->declare_parameter("declared_int", 123);
  node->declare_parameter("missing_type", "declared");

  const char * node_name = node->get_name();
  if (strcmp("sdf_node_name", node_name)) {
    RCLCPP_ERROR(node->get_logger(), "Node name not taken from SDF");
    return;
  }

  // Attempt to get the parameters set by the SDF.
  // If any of them fail, messages will not publish so test will fail

  // Create a publisher
  auto pub = node->create_publisher<std_msgs::msg::String>(
    "test",
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local());

  {
    auto param = node->get_parameter("declared_string");
    if (rclcpp::PARAMETER_STRING != param.get_type()) {
      RCLCPP_ERROR(node->get_logger(), "Wrong parameter type, not string");
      return;
    }
    if ("from SDF" != param.get_value<std::string>()) {
      RCLCPP_ERROR(node->get_logger(), "Wrong parameter value");
      return;
    }
  }

  {
    auto param = node->get_parameter("declared_int");
    if (rclcpp::PARAMETER_INTEGER != param.get_type()) {
      RCLCPP_ERROR(node->get_logger(), "Wrong parameter type, not int");
      return;
    }
    if (42 != param.get_value<int>()) {
      RCLCPP_ERROR(node->get_logger(), "Wrong parameter value");
      return;
    }
  }

  bool caught{false};
  try {
    node->get_parameter("non_declared_bool");
  } catch (const rclcpp::exceptions::ParameterNotDeclaredException &) {
    caught = true;
  }
  if (!caught) {
    RCLCPP_ERROR(node->get_logger(), "Failed to throw exception");
    return;
  }

  caught = false;
  try {
    node->get_parameter("non_declared_double");
  } catch (const rclcpp::exceptions::ParameterNotDeclaredException &) {
    caught = true;
  }
  if (!caught) {
    RCLCPP_ERROR(node->get_logger(), "Failed to throw exception");
    return;
  }

  {
    auto param = node->get_parameter("missing_type");
    if (rclcpp::PARAMETER_STRING != param.get_type()) {
      RCLCPP_ERROR(node->get_logger(), "Wrong parameter type, not int");
      return;
    }
    if ("declared" != param.get_value<std::string>()) {
      RCLCPP_ERROR(node->get_logger(), "Wrong parameter value");
      return;
    }
  }

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

GZ_REGISTER_WORLD_PLUGIN(SDFNode)
