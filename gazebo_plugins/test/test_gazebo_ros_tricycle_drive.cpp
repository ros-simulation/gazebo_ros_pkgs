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
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <memory>

#define tol 10e-2

using namespace std::literals::chrono_literals; // NOLINT

class GazeboRosTricycleDriveTest : public gazebo::ServerFixture
{
};

TEST_F(GazeboRosTricycleDriveTest, Publishing)
{
  // Load test world and start paused
  this->Load("worlds/gazebo_ros_tricycle_drive.world", true);

  // World
  auto world = gazebo::physics::get_world();
  ASSERT_NE(nullptr, world);

  // Model
  auto tricycle = world->ModelByName("tricycle");
  ASSERT_NE(nullptr, tricycle);

  // Step a bit for model to settle
  world->Step(100);

  // Check model state
  EXPECT_NEAR(0.0, tricycle->WorldPose().Pos().X(), tol);
  EXPECT_NEAR(0.0, tricycle->WorldPose().Pos().Y(), tol);
  EXPECT_NEAR(0.0, tricycle->WorldPose().Rot().Yaw(), tol);
  EXPECT_NEAR(0.0, tricycle->WorldLinearVel().X(), tol);
  EXPECT_NEAR(0.0, tricycle->WorldAngularVel().Z(), tol);

  // Create node and executor
  auto node = std::make_shared<rclcpp::Node>("gazebo_ros_diff_drive_test");
  ASSERT_NE(nullptr, node);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  // Create subscriber
  nav_msgs::msg::Odometry::SharedPtr latestMsg;
  auto sub = node->create_subscription<nav_msgs::msg::Odometry>(
    "test/odom_test", rclcpp::QoS(rclcpp::KeepLast(1)),
    [&latestMsg](const nav_msgs::msg::Odometry::SharedPtr _msg) {
      latestMsg = _msg;
    });

  // Send command
  auto pub = node->create_publisher<geometry_msgs::msg::Twist>(
    "test/cmd_test", rclcpp::QoS(rclcpp::KeepLast(1)));

  auto msg = geometry_msgs::msg::Twist();
  msg.linear.x = 0.5;
  msg.angular.z = 0.05;

  double sleep = 0;
  double max_sleep = 100;
  for (; sleep < max_sleep; ++sleep) {
    pub->publish(msg);
    executor.spin_once(100ms);
    world->Step(100);
  }

  // Check message
  ASSERT_NE(nullptr, latestMsg);
  EXPECT_EQ("odom_frame_test", latestMsg->header.frame_id);
  EXPECT_LT(-tol, latestMsg->pose.pose.position.x);
  EXPECT_LT(-tol, latestMsg->pose.pose.orientation.z);

  // Check movement
  EXPECT_LT(-tol, tricycle->WorldPose().Pos().X());
  EXPECT_LT(-tol, tricycle->WorldPose().Rot().Yaw());
  EXPECT_NEAR(0.5, tricycle->WorldLinearVel().X(), tol);
  EXPECT_NEAR(0.05, tricycle->WorldAngularVel().Z(), tol);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
