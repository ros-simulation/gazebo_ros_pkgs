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

#include <gazebo/common/Time.hh>
#include <gazebo/test/ServerFixture.hh>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <memory>

#define tol 10e-2

using namespace std::literals::chrono_literals; // NOLINT

class GazeboRosP3dTest : public gazebo::ServerFixture
{
};

TEST_F(GazeboRosP3dTest, Publishing)
{
  // Load test world and start paused
  this->Load("worlds/gazebo_ros_p3d.world", true);

  // World
  auto world = gazebo::physics::get_world();
  ASSERT_NE(nullptr, world);

  // Model
  auto model = world->ModelByName("the_model");
  ASSERT_NE(nullptr, model);

  // Link
  auto link = model->GetLink("box_link");
  ASSERT_NE(nullptr, link);

  // Reference link
  auto ref_link = model->GetLink("sphere_link");
  ASSERT_NE(nullptr, ref_link);

  // Step a bit for model to settle
  world->Step(100);

  // Create node and executor
  auto node = std::make_shared<rclcpp::Node>("gazebo_ros_p3d_test");
  ASSERT_NE(nullptr, node);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  // Create subscriber
  nav_msgs::msg::Odometry::SharedPtr latest_msg{nullptr};
  auto sub = node->create_subscription<nav_msgs::msg::Odometry>(
    "test/p3d_test", rclcpp::SensorDataQoS(),
    [&latest_msg](const nav_msgs::msg::Odometry::SharedPtr _msg) {
      latest_msg = _msg;
    });

  // Wait for a message
  world->Step(1000);
  unsigned int max_sleep{30u};
  for (auto sleep = 0u; !latest_msg && sleep < max_sleep; ++sleep) {
    executor.spin_once(100ms);
    gazebo::common::Time::MSleep(100);
  }

  // Check message
  ASSERT_NE(nullptr, latest_msg);
  EXPECT_EQ("sphere_link", latest_msg->header.frame_id);
  EXPECT_EQ("box_link", latest_msg->child_frame_id);

  EXPECT_NEAR(
    latest_msg->pose.pose.position.x,
    -ref_link->WorldPose().Pos().X() + link->WorldPose().Pos().X() + 10, tol);
  EXPECT_NEAR(
    latest_msg->pose.pose.position.y,
    -ref_link->WorldPose().Pos().Y() + link->WorldPose().Pos().Y() + 10, tol);
  EXPECT_NEAR(
    latest_msg->pose.pose.position.z,
    -ref_link->WorldPose().Pos().Z() + link->WorldPose().Pos().Z() + 10, tol);
  EXPECT_NEAR(latest_msg->pose.pose.orientation.x, link->WorldPose().Rot().X(), tol);
  EXPECT_NEAR(latest_msg->pose.pose.orientation.y, link->WorldPose().Rot().Y(), tol);
  EXPECT_NEAR(latest_msg->pose.pose.orientation.z, link->WorldPose().Rot().Z(), tol);

  EXPECT_NEAR(0.0, latest_msg->twist.twist.linear.x, tol);
  EXPECT_NEAR(0.0, latest_msg->twist.twist.linear.y, tol);
  EXPECT_NEAR(0.0, latest_msg->twist.twist.linear.z, tol);
  EXPECT_NEAR(0.0, latest_msg->twist.twist.angular.x, tol);
  EXPECT_NEAR(0.0, latest_msg->twist.twist.angular.y, tol);
  EXPECT_NEAR(0.0, latest_msg->twist.twist.angular.z, tol);
}

TEST_F(GazeboRosP3dTest, DeprecatedOffsetElements)
{
  // Load test world and start paused
  this->Load("worlds/gazebo_ros_p3d.world", true);

  // World
  auto world = gazebo::physics::get_world();
  ASSERT_NE(nullptr, world);

  // Model
  auto model = world->ModelByName("the_model");
  ASSERT_NE(nullptr, model);

  // Link
  auto link = model->GetLink("box_link");
  ASSERT_NE(nullptr, link);

  // Reference link
  auto ref_link = model->GetLink("sphere_link");
  ASSERT_NE(nullptr, ref_link);

  // Step a bit for model to settle
  world->Step(100);

  // Create node and executor
  auto node = std::make_shared<rclcpp::Node>("gazebo_ros_p3d_test");
  ASSERT_NE(nullptr, node);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  // Create subscriber
  nav_msgs::msg::Odometry::SharedPtr latest_msg{nullptr};
  auto sub = node->create_subscription<nav_msgs::msg::Odometry>(
    "test_deprecated/p3d_test_deprecated", rclcpp::SensorDataQoS(),
    [&latest_msg](const nav_msgs::msg::Odometry::SharedPtr _msg) {
      latest_msg = _msg;
    });

  // Wait for a message
  world->Step(1000);
  unsigned int max_sleep{30u};
  for (auto sleep = 0u; !latest_msg && sleep < max_sleep; ++sleep) {
    executor.spin_once(100ms);
    gazebo::common::Time::MSleep(100);
  }

  // Check message
  ASSERT_NE(nullptr, latest_msg);
  EXPECT_EQ("sphere_link", latest_msg->header.frame_id);
  EXPECT_EQ("box_link", latest_msg->child_frame_id);

  EXPECT_NEAR(
    latest_msg->pose.pose.position.x,
    -ref_link->WorldPose().Pos().X() + link->WorldPose().Pos().X() + 5, tol);
  EXPECT_NEAR(
    latest_msg->pose.pose.position.y,
    -ref_link->WorldPose().Pos().Y() + link->WorldPose().Pos().Y() + 5, tol);
  EXPECT_NEAR(
    latest_msg->pose.pose.position.z,
    -ref_link->WorldPose().Pos().Z() + link->WorldPose().Pos().Z() + 5, tol);
  EXPECT_NEAR(latest_msg->pose.pose.orientation.x, link->WorldPose().Rot().X(), tol);
  EXPECT_NEAR(latest_msg->pose.pose.orientation.y, link->WorldPose().Rot().Y(), tol);
  EXPECT_NEAR(latest_msg->pose.pose.orientation.z, link->WorldPose().Rot().Z(), tol);

  EXPECT_NEAR(0.0, latest_msg->twist.twist.linear.x, tol);
  EXPECT_NEAR(0.0, latest_msg->twist.twist.linear.y, tol);
  EXPECT_NEAR(0.0, latest_msg->twist.twist.linear.z, tol);
  EXPECT_NEAR(0.0, latest_msg->twist.twist.angular.x, tol);
  EXPECT_NEAR(0.0, latest_msg->twist.twist.angular.y, tol);
  EXPECT_NEAR(0.0, latest_msg->twist.twist.angular.z, tol);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
