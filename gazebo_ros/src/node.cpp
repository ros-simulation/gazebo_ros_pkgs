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

#include <gazebo/common/CommonIface.hh>

#include <gazebo_ros/node.hpp>

#include <rcl/arguments.h>

#include <sdf/Param.hh>

#include <memory>
#include <string>
#include <vector>

namespace gazebo_ros
{

std::weak_ptr<Executor> Node::static_executor_;
std::weak_ptr<Node> Node::static_node_;
std::mutex Node::lock_;

Node::~Node()
{
  executor_->remove_node(get_node_base_interface());
}

Node::SharedPtr Node::Get(sdf::ElementPtr sdf)
{
  return Get(sdf, "/");
}

Node::SharedPtr Node::Get(
  sdf::ElementPtr sdf,
  const gazebo::physics::ModelPtr & parent)
{
  std::string modelName;
  if (parent) {
    modelName = parent->GetName();
  }

  return Get(sdf, "/" + modelName);
}

Node::SharedPtr Node::Get(
  sdf::ElementPtr sdf,
  const gazebo::sensors::SensorPtr & parent)
{
  std::string modelName;
  std::vector<std::string> values;
  std::string scopedName = parent->ScopedName();
  values = gazebo::common::split(scopedName, "::");
  if (values.size() < 2) {
    modelName = "";
  } else {
    // the second element is the model name; the first one is the world name
    modelName = values[1];
  }

  return Get(sdf, "/" + modelName);
}

Node::SharedPtr Node::Get(
  sdf::ElementPtr sdf,
  const gazebo::rendering::VisualPtr & parent)
{
  std::string modelName;
  if (parent) {
    modelName = parent->GetRootVisual()->Name();
  }

  return Get(sdf, "/" + modelName);
}

Node::SharedPtr Node::Get(
  sdf::ElementPtr sdf,
  const std::string & defaultNamespace)
{
  // Initialize arguments
  std::string name = "";
  std::vector<std::string> arguments;
  std::vector<rclcpp::Parameter> parameter_overrides;

  // Get the name of the plugin as the name for the node.
  if (!sdf->HasAttribute("name")) {
    RCLCPP_WARN(internal_logger(), "Name of plugin not found.");
  }
  name = sdf->Get<std::string>("name");

  // Get inner <ros> element if full plugin sdf was passed in
  if (sdf->HasElement("ros")) {
    sdf = sdf->GetElement("ros");
  }

  // Legacy namespace
  // True to default to the root (/) namespace if <namespace> is not specified
  // False to use the model name as the namespace if <namespace> is not
  // specified.
  // todo(anyone) change this to false in humble
  bool legacyNamespace = true;
  if (sdf->HasElement("legacy_namespace")) {
    legacyNamespace = sdf->Get<bool>("legacy_namespace");
  }
  std::string ns =
    (legacyNamespace || defaultNamespace.empty() ||
    defaultNamespace[0] != '/') ? "/" : defaultNamespace;

  // Set namespace if tag is present
  if (sdf->HasElement("namespace")) {
    ns = sdf->GetElement("namespace")->Get<std::string>();
    // prevent exception: namespace must be absolute, it must lead with a '/'
    if (ns.empty() || ns[0] != '/') {
      ns = '/' + ns;
    }
  }

  // Get list of arguments from SDF
  if (sdf->HasElement("argument")) {
    sdf::ElementPtr argument_sdf = sdf->GetElement("argument");

    while (argument_sdf) {
      std::string argument = argument_sdf->Get<std::string>();
      arguments.push_back(argument);
      argument_sdf = argument_sdf->GetNextElement("argument");
    }
  }

  // Get list of remapping rules from SDF
  if (sdf->HasElement("remapping")) {
    sdf::ElementPtr argument_sdf = sdf->GetElement("remapping");

    arguments.push_back(RCL_ROS_ARGS_FLAG);
    while (argument_sdf) {
      std::string argument = argument_sdf->Get<std::string>();
      arguments.push_back(RCL_REMAP_FLAG);
      arguments.push_back(argument);
      argument_sdf = argument_sdf->GetNextElement("remapping");
    }
  }

  // Convert each parameter tag to a ROS parameter
  if (sdf->HasElement("parameter")) {
    sdf::ElementPtr parameter_sdf = sdf->GetElement("parameter");
    while (parameter_sdf) {
      auto param = sdf_to_ros_parameter(parameter_sdf);
      if (rclcpp::ParameterType::PARAMETER_NOT_SET != param.get_type()) {
        parameter_overrides.push_back(param);
      }
      parameter_sdf = parameter_sdf->GetNextElement("parameter");
    }
  }

  rclcpp::NodeOptions node_options;
  node_options.arguments(arguments);
  node_options.parameter_overrides(parameter_overrides);

  // Create node with parsed arguments
  std::shared_ptr<gazebo_ros::Node> node = CreateWithArgs(name, ns, node_options);

  // Parse the qos tag
  node->qos_ = gazebo_ros::QoS(sdf, name, ns, node_options);

  return node;
}

Node::SharedPtr Node::Get()
{
  Node::SharedPtr node = static_node_.lock();

  if (!node) {
    rclcpp::NodeOptions node_options;
    node_options.allow_undeclared_parameters(true);
    node = CreateWithArgs("gazebo", "", node_options);
    static_node_ = node;
  }

  return node;
}

rclcpp::Parameter Node::sdf_to_ros_parameter(sdf::ElementPtr const & sdf)
{
  if (!sdf->HasAttribute("name")) {
    RCLCPP_WARN(
      internal_logger(),
      "Ignoring parameter because it has no attribute 'name'. Tag: %s", sdf->ToString("").c_str());
    return rclcpp::Parameter();
  }
  if (!sdf->HasAttribute("type")) {
    RCLCPP_WARN(
      internal_logger(),
      "Ignoring parameter because it has no attribute 'type'. Tag: %s", sdf->ToString("").c_str());
    return rclcpp::Parameter();
  }

  std::string name = sdf->Get<std::string>("name");
  std::string type = sdf->Get<std::string>("type");

  if ("int" == type) {
    return rclcpp::Parameter(name, sdf->Get<int>());
  } else if ("double" == type || "float" == type) {
    return rclcpp::Parameter(name, sdf->Get<double>());
  } else if ("bool" == type) {
    return rclcpp::Parameter(name, sdf->Get<bool>());
  } else if ("string" == type) {
    return rclcpp::Parameter(name, sdf->Get<std::string>());
  } else {
    RCLCPP_WARN(
      internal_logger(),
      "Ignoring parameter because attribute 'type' is invalid. Tag: %s", sdf->ToString("").c_str());
    return rclcpp::Parameter();
  }
}

rclcpp::Logger Node::internal_logger()
{
  return rclcpp::get_logger("gazebo_ros_node");
}

}  // namespace gazebo_ros
