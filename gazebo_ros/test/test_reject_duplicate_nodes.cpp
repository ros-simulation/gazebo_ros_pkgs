// Copyright 2022 Open Source Robotics Foundation, Inc.
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

#include <gtest/gtest.h>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>

TEST(TestRejectDuplicateNodes, PluginsSameName)
{
  // sdf model with same plugin name
  auto sdf_str =
    "<?xml version='1.0' ?>"
    "<sdf version='1.6'>"
    "<world name='default'>"
    "<model name='model_1'>"
    "<plugin name='node_1' filename='libnode_name.so'/>"
    "</model>"
    "<model name='model_2'>"
    "<plugin name='node_1' filename='libnode_name.so'/>"
    "</model>"
    "</world>"
    "</sdf>";

  sdf::SDF sdf;
  sdf.SetFromString(sdf_str);
  auto plugin_sdf_1 = sdf.Root()->GetElement("world")->GetFirstElement()->GetElement("plugin");
  gazebo_ros::Node::SharedPtr node_1 = gazebo_ros::Node::Get(plugin_sdf_1);
  ASSERT_NE(nullptr, node_1);

  // check if a node with the same name exists
  auto plugin_sdf_2 =
    sdf.Root()->GetElement("world")->GetElement("model")->GetNextElement()->GetElement("plugin");
  gazebo_ros::Node::SharedPtr node_2 = gazebo_ros::Node::Get(plugin_sdf_2);
  ASSERT_TRUE(node_2 == nullptr);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
