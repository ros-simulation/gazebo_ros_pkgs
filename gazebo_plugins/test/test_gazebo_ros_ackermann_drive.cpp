// Copyright 2019 Open Source Robotics Foundation, Inc.
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

#include <memory>

#include "gazebo/common/Time.hh"
#include "gazebo/test/ServerFixture.hh"

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

#define tol 10e-1

using namespace std::literals::chrono_literals; // NOLINT

class GazeboRosAckermannDriveTest : public gazebo::ServerFixture
{
};

TEST_F(GazeboRosAckermannDriveTest, Publishing)
{
  // Load test world and start paused
  this->Load("worlds/gazebo_ros_ackermann_drive.world", true);

  // World
  auto world = gazebo::physics::get_world();
  ASSERT_NE(nullptr, world);

  // Model
  auto vehicle = world->ModelByName("vehicle");
  ASSERT_NE(nullptr, vehicle);

  // Create node and executor
  auto node = std::make_shared<rclcpp::Node>("gazebo_ros_ackermann_drive_test");
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

  // Step a bit for model to settle
  world->Step(100);
  executor.spin_once(100ms);

  // Check model state
  EXPECT_NEAR(0.0, vehicle->WorldPose().Pos().X(), tol);
  EXPECT_NEAR(0.0, vehicle->WorldPose().Pos().Y(), tol);
  EXPECT_NEAR(0.0, vehicle->WorldPose().Rot().Yaw(), tol);
  EXPECT_NEAR(0.0, vehicle->WorldLinearVel().X(), tol);
  EXPECT_NEAR(0.0, vehicle->WorldAngularVel().Z(), tol);

  // Send command
  auto pub = node->create_publisher<geometry_msgs::msg::Twist>(
    "test/cmd_test", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local());

  auto msg = geometry_msgs::msg::Twist();
  msg.linear.x = 5.0;
  msg.angular.z = 0.1;
  pub->publish(msg);

  // Wait for it to be processed
  int sleep{0};
  int maxSleep{300};

  double yaw = static_cast<float>(vehicle->WorldPose().Rot().Yaw());
  auto linear_vel = vehicle->WorldLinearVel();
  double linear_vel_x = cosf(yaw) * linear_vel.X() + sinf(yaw) * linear_vel.Y();

  for (; sleep < maxSleep && (linear_vel_x < 4.0 ||
    vehicle->WorldAngularVel().Z() < 0.0); ++sleep)
  {
    yaw = static_cast<float>(vehicle->WorldPose().Rot().Yaw());
    linear_vel = vehicle->WorldLinearVel();
    linear_vel_x = cosf(yaw) * linear_vel.X() + sinf(yaw) * linear_vel.Y();
    world->Step(100);
    executor.spin_once(100ms);
    gazebo::common::Time::MSleep(100);
  }
  EXPECT_NE(sleep, maxSleep);

  // Check message
  ASSERT_NE(nullptr, latestMsg);
  EXPECT_EQ("odom_test", latestMsg->header.frame_id);
  EXPECT_LT(0.0, latestMsg->pose.pose.position.x);
  EXPECT_LT(0.0, latestMsg->pose.pose.orientation.z);

  // Check movement
  yaw = static_cast<float>(vehicle->WorldPose().Rot().Yaw());
  linear_vel = vehicle->WorldLinearVel();
  linear_vel_x = cosf(yaw) * linear_vel.X() + sinf(yaw) * linear_vel.Y();
  EXPECT_LT(0.0, vehicle->WorldPose().Pos().X());
  EXPECT_LT(0.0, yaw);
  EXPECT_NEAR(5.0, linear_vel_x, tol);
  EXPECT_NEAR(0.1, vehicle->WorldAngularVel().Z(), tol);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
