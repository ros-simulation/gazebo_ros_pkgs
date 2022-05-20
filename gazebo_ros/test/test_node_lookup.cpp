//
// Created by deepanshu on 5/20/22.
//

#include <gtest/gtest.h>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>

TEST(TestNodeLookUp, NodesSameName)
{
  // create first node
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

  gazebo_ros::Node::SharedPtr node_1 = gazebo_ros::Node::Get(plugin_sdf_1);
  ASSERT_NE(nullptr, node_1);

  // create second node with same name as first
  auto sdf_str_2 =
      "<?xml version='1.0' ?>"
      "<sdf version='1.6'>"
      "<world name='default'>"
      "<plugin name='node_1' filename='libnode_name.so'/>"
      "</world>"
      "</sdf>";
  sdf::SDF sdf_2;
  sdf_2.SetFromString(sdf_str_2);
  auto plugin_sdf_2 = sdf_1.Root()->GetElement("world")->GetElement("plugin");

  gazebo_ros::Node::SharedPtr node_2 = gazebo_ros::Node::Get(plugin_sdf_1);
  ASSERT_TRUE(node_2 == nullptr);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}