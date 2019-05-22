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

#include <gtest/gtest.h>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>

TEST(TestNode, StaticNode)
{
  // Create the static node
  auto node_1 = gazebo_ros::Node::Get();
  ASSERT_NE(nullptr, node_1);
  EXPECT_STREQ("gazebo", node_1->get_name());

  std::stringstream address_1;
  address_1 << node_1;

  // The next time we get it, it's the same node
  auto node_2 = gazebo_ros::Node::Get();
  ASSERT_NE(nullptr, node_2);
  EXPECT_STREQ("gazebo", node_2->get_name());
  EXPECT_EQ(node_1, node_2);

  std::stringstream address_2;
  address_2 << node_2;
  EXPECT_EQ(address_1.str(), address_2.str());

  // Reset both, the node should be destroyed
  node_1.reset();
  node_2.reset();

  // Create a new static node and check it is different
  auto node_3 = gazebo_ros::Node::Get();
  ASSERT_NE(nullptr, node_3);
  EXPECT_STREQ("gazebo", node_3->get_name());

  std::stringstream address_3;
  address_3 << node_3;
  EXPECT_NE(address_1.str(), address_3.str());
}

TEST(TestNode, GetSdf)
{
  // Create a node
  auto sdf_str_1 =
    "<?xml version='1.0' ?>"
    "<sdf version='1.6'>"
    "<world name='default'>"
    "<plugin name='node_1' filename='libnode_name.so'/>"
    "</world>"
    "</sdf>";
  sdf::SDF sdf_1;
  sdf_1.SetFromString(sdf_str_1);
  auto plugin_sdf_1 = sdf_1.Root()->GetElement("world")->GetElement("plugin");

  auto node_1 = gazebo_ros::Node::Get(plugin_sdf_1);
  ASSERT_NE(nullptr, node_1);
  EXPECT_STREQ("node_1", node_1->get_name());

  // TODO(anyone) Fix this test, see
  // https://github.com/ros-simulation/gazebo_ros_pkgs/issues/855

  // Create another node
  // auto sdf_str_2 =
  //   "<?xml version='1.0' ?>"
  //   "<sdf version='1.6'>"
  //   "<world name='default'>"
  //   "<plugin name='node_2' filename='libnode_name.so'/>"
  //   "</world>"
  //   "</sdf>";
  // sdf::SDF sdf_2;
  // sdf_2.SetFromString(sdf_str_2);
  // auto plugin_sdf_2 = sdf_2.Root()->GetElement("world")->GetElement("plugin");

  // auto node_2 = gazebo_ros::Node::Get(plugin_sdf_2);
  // ASSERT_NE(nullptr, node_2);
  // EXPECT_STREQ("node_2", node_2->get_name());
  // EXPECT_NE(node_1, node_2);

  // Reset both
  // node_1.reset();
  // node_2.reset();

  // // Create another node
  // auto sdf_str_3 =
  //   "<?xml version='1.0' ?>"
  //   "<sdf version='1.6'>"
  //   "<world name='default'>"
  //   "<plugin name='node_3' filename='libnode_name.so'/>"
  //   "</world>"
  //   "</sdf>";
  // sdf::SDF sdf_3;
  // sdf_3.SetFromString(sdf_str_3);
  // auto plugin_sdf_3 = sdf_3.Root()->GetElement("world")->GetElement("plugin");
  // auto node_3 = gazebo_ros::Node::Get(plugin_sdf_3);
  // ASSERT_NE(nullptr, node_3);
  // EXPECT_STREQ("node_3", node_3->get_name());
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
