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

#include <gtest/gtest.h>
#include <gazebo_ros/qos.hpp>

TEST(TestQoS, ValidConstruction)
{
  // No SDF
  {
    gazebo_ros::QoS qos;
  }
  // No <qos> element
  {
    auto sdf_str =
      "<?xml version='1.0' ?>"
      "<sdf version='1.6'>"
      "  <world name='default'>"
      "  </world>"
      "</sdf>";
    sdf::SDF sdf;
    sdf.SetFromString(sdf_str);
    gazebo_ros::QoS qos(sdf.Root(), "test_node", "/", rclcpp::NodeOptions());

    // We should get the default QoS for an arbitrary topic
    rclcpp::QoS some_qos = qos.get_publisher_qos("some_random_test_topic", rclcpp::QoS(10));
    EXPECT_EQ(rclcpp::QoS(10), some_qos);
  }
  // Empty <qos> element
  {
    auto sdf_str =
      "<?xml version='1.0' ?>"
      "<sdf version='1.6'>"
      "  <world name='default'>"
      "    <plugin name='node_1' filename='libnode_name.so'>"
      "      <qos />"
      "    </plugin>"
      "  </world>"
      "</sdf>";
    sdf::SDF sdf;
    sdf.SetFromString(sdf_str);
    gazebo_ros::QoS qos(
      sdf.Root()->GetElement("world")->GetElement("plugin"),
      "test_node",
      "/",
      rclcpp::NodeOptions());
    // We should get the default QoS for an arbitrary topic
    rclcpp::QoS some_qos = qos.get_publisher_qos("foo/bar/baz", rclcpp::QoS(10));
    EXPECT_EQ(rclcpp::QoS(10), some_qos);
  }
  // <qos> element with some content
  {
    rclcpp::QoS expected_sub_qos(10);
    expected_sub_qos.reliable();
    expected_sub_qos.transient_local();

    auto sdf_str =
      "<?xml version='1.0' ?>"
      "<sdf version='1.6'>"
      "  <world name='default'>"
      "    <plugin name='node_1' filename='libnode_name.so'>"
      "      <ros>"
      "        <qos>"
      "          <topic name='foo'>"
      "            <subscription>"
      "              <reliability>reliable</reliability>"
      "              <durability>transient_local</durability>"
      "            </subscription>"
      "          </topic>"
      "        </qos>"
      "      </ros>"
      "    </plugin>"
      "  </world>"
      "</sdf>";
    sdf::SDF sdf;
    sdf.SetFromString(sdf_str);
    gazebo_ros::QoS qos(
      sdf.Root()->GetElement("world")->GetElement("plugin")->GetElement("ros"),
      "test_node",
      "/",
      rclcpp::NodeOptions());

    rclcpp::QoS sub_qos = qos.get_subscription_qos("foo");
    EXPECT_EQ(expected_sub_qos, sub_qos);
    // Publisher should have default QoS
    rclcpp::QoS pub_qos = qos.get_publisher_qos("foo", rclcpp::QoS(10));
    EXPECT_EQ(rclcpp::QoS(10), pub_qos);
  }
  // <qos> element with all possible overrides
  {
    rclcpp::QoS expected_pub_qos(10);
    expected_pub_qos.best_effort();
    expected_pub_qos.transient_local();
    expected_pub_qos.keep_all();
    expected_pub_qos.deadline(std::chrono::milliseconds(123));
    expected_pub_qos.lifespan(std::chrono::milliseconds(42));
    expected_pub_qos.liveliness_lease_duration(std::chrono::milliseconds(1000));
    expected_pub_qos.liveliness(RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC);
    rclcpp::QoS expected_sub_qos(10);
    expected_sub_qos.reliable();
    expected_sub_qos.durability_volatile();
    expected_sub_qos.keep_last(3u);
    expected_sub_qos.deadline(std::chrono::milliseconds(321));
    expected_sub_qos.lifespan(std::chrono::milliseconds(24));
    expected_sub_qos.liveliness_lease_duration(std::chrono::milliseconds(1));
    expected_sub_qos.liveliness(RMW_QOS_POLICY_LIVELINESS_AUTOMATIC);

    auto sdf_str =
      "<?xml version='1.0' ?>"
      "<sdf version='1.6'>"
      "  <world name='default'>"
      "    <plugin name='node_1' filename='libnode_name.so'>"
      "      <ros>"
      "        <qos>"
      "          <topic name='foo'>"
      "            <publisher>"
      "              <reliability>best_effort</reliability>"
      "              <durability>transient_local</durability>"
      "              <history>keep_all</history>"
      "              <deadline>123</deadline>"
      "              <lifespan>42</lifespan>"
      "              <liveliness_lease_duration>1000</liveliness_lease_duration>"
      "              <liveliness>manual_by_topic</liveliness>"
      "            </publisher>"
      "            <subscription>"
      "              <reliability>reliable</reliability>"
      "              <durability>volatile</durability>"
      "              <history depth='3'>keep_last</history>"
      "              <deadline>321</deadline>"
      "              <lifespan>24</lifespan>"
      "              <liveliness_lease_duration>1</liveliness_lease_duration>"
      "              <liveliness>automatic</liveliness>"
      "            </subscription>"
      "          </topic>"
      "        </qos>"
      "      </ros>"
      "    </plugin>"
      "  </world>"
      "</sdf>";
    sdf::SDF sdf;
    sdf.SetFromString(sdf_str);
    gazebo_ros::QoS qos(
      sdf.Root()->GetElement("world")->GetElement("plugin")->GetElement("ros"),
      "test_node",
      "/",
      rclcpp::NodeOptions());

    rclcpp::QoS pub_qos = qos.get_publisher_qos("foo");
    EXPECT_EQ(expected_pub_qos, pub_qos);
    rclcpp::QoS sub_qos = qos.get_subscription_qos("foo");
    EXPECT_EQ(expected_sub_qos, sub_qos);
  }
  // Multiple <topic> elements
  {
    rclcpp::QoS expected_sub_qos(10);
    expected_sub_qos.best_effort();
    expected_sub_qos.durability_volatile();
    rclcpp::QoS expected_pub_qos(10);
    expected_pub_qos.reliable();
    expected_pub_qos.transient_local();

    auto sdf_str =
      "<?xml version='1.0' ?>"
      "<sdf version='1.6'>"
      "  <world name='default'>"
      "    <plugin name='node_1' filename='libnode_name.so'>"
      "      <ros>"
      "       <qos>"
      "         <topic name='foo'>"
      "           <subscription>"
      "             <reliability>best_effort</reliability>"
      "             <durability>volatile</durability>"
      "           </subscription>"
      "         </topic>"
      "         <topic name='foo'>"
      "           <publisher>"
      "             <reliability>reliable</reliability>"
      "             <durability>transient_local</durability>"
      "           </publisher>"
      "         </topic>"
      "       </qos>"
      "      </ros>"
      "    </plugin>"
      "  </world>"
      "</sdf>";
    sdf::SDF sdf;
    sdf.SetFromString(sdf_str);
    gazebo_ros::QoS qos(
      sdf.Root()->GetElement("world")->GetElement("plugin")->GetElement("ros"),
      "test_node",
      "/",
      rclcpp::NodeOptions());

    rclcpp::QoS sub_qos = qos.get_subscription_qos("foo");
    EXPECT_EQ(expected_sub_qos, sub_qos);
    rclcpp::QoS pub_qos = qos.get_publisher_qos("foo");
    EXPECT_EQ(expected_pub_qos, pub_qos);
  }
}

TEST(TestQoS, InvalidConstruction)
{
  // Missing topic name
  {
    auto sdf_str =
      "<?xml version='1.0' ?>"
      "<sdf version='1.6'>"
      "  <world name='default'>"
      "    <plugin name='node_1' filename='libnode_name.so'>"
      "      <qos>"
      "        <topic>"
      "        </topic>"
      "      </qos>"
      "    </plugin>"
      "  </world>"
      "</sdf>";
    sdf::SDF sdf;
    sdf.SetFromString(sdf_str);
    EXPECT_THROW(
      gazebo_ros::QoS qos(
        sdf.Root()->GetElement("world")->GetElement("plugin"),
        "test_node",
        "/",
        rclcpp::NodeOptions()),
      gazebo_ros::InvalidQoSException);
  }
  // Invalid <reliability> value
  {
    auto sdf_str =
      "<?xml version='1.0' ?>"
      "<sdf version='1.6'>"
      "  <world name='default'>"
      "    <plugin name='node_1' filename='libnode_name.so'>"
      "      <qos>"
      "        <topic>"
      "          <publisher>"
      "            <reliability>foo</reliability>"
      "          </publisher>"
      "        </topic>"
      "      </qos>"
      "    </plugin>"
      "  </world>"
      "</sdf>";
    sdf::SDF sdf;
    sdf.SetFromString(sdf_str);
    EXPECT_THROW(
      gazebo_ros::QoS qos(
        sdf.Root()->GetElement("world")->GetElement("plugin"),
        "test_node",
        "/",
        rclcpp::NodeOptions()),
      gazebo_ros::InvalidQoSException);
  }
  // Missing depth attribute
  {
    auto sdf_str =
      "<?xml version='1.0' ?>"
      "<sdf version='1.6'>"
      "  <world name='default'>"
      "    <plugin name='node_1' filename='libnode_name.so'>"
      "      <qos>"
      "        <topic>"
      "          <publisher>"
      "            <history>keep_last</history>"
      "          </publisher>"
      "        </topic>"
      "      </qos>"
      "    </plugin>"
      "  </world>"
      "</sdf>";
    sdf::SDF sdf;
    sdf.SetFromString(sdf_str);
    EXPECT_THROW(
      gazebo_ros::QoS qos(
        sdf.Root()->GetElement("world")->GetElement("plugin"),
        "test_node",
        "/",
        rclcpp::NodeOptions()),
      gazebo_ros::InvalidQoSException);
  }
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
