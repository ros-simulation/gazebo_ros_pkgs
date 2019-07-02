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

#include <gazebo/test/ServerFixture.hh>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float32.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>

#define tol 10e-2

using namespace std::literals::chrono_literals; // NOLINT

class GazeboRosHarnessTest : public gazebo::ServerFixture
{
};

TEST_F(GazeboRosHarnessTest, Publishing)
{
  // Load test world and start paused
  this->Load("worlds/gazebo_ros_harness.world", true);

  // World
  auto world = gazebo::physics::get_world();
  ASSERT_NE(nullptr, world);

  // Model
  auto box = world->ModelByName("box");
  ASSERT_NE(nullptr, box);

  // Link
  auto link = box->GetLink("link");
  ASSERT_NE(nullptr, link);

  // Step a bit for model to settle
  world->Step(100);

  // Check model state
  EXPECT_NEAR(0.0, link->WorldPose().Pos().X(), tol);
  EXPECT_NEAR(0.0, link->WorldPose().Pos().Y(), tol);
  EXPECT_NEAR(3.0, link->WorldPose().Pos().Z(), tol);

  // Create node and executor
  auto node = std::make_shared<rclcpp::Node>("gazebo_ros_harness_test");
  ASSERT_NE(nullptr, node);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  // Send velocity
  auto vel_pub = node->create_publisher<std_msgs::msg::Float32>(
    "test/velocity_test", rclcpp::QoS(rclcpp::KeepLast(1)));

  auto vel_msg = std_msgs::msg::Float32();
  vel_msg.data = -1.0;

  double sleep = 0;
  double max_sleep = 100;
  for (; sleep < max_sleep; ++sleep) {
    vel_pub->publish(vel_msg);
    executor.spin_once(100ms);
    world->Step(100);
  }

  // Check model state
  EXPECT_NEAR(0.0, link->WorldPose().Pos().X(), tol);
  EXPECT_NEAR(0.0, link->WorldPose().Pos().Y(), tol);
  EXPECT_NEAR(1.5, link->WorldPose().Pos().Z(), tol);


  // Send detach command
  auto detach_pub = node->create_publisher<std_msgs::msg::Empty>(
    "test/detach_test", rclcpp::QoS(rclcpp::KeepLast(1)));

  auto detach_msg = std_msgs::msg::Empty();

  sleep = 0;
  max_sleep = 10;
  for (; sleep < max_sleep && link->WorldPose().Pos().Z() != 0.5; ++sleep) {
    detach_pub->publish(detach_msg);
    executor.spin_once(100ms);
    world->Step(100);
  }

  // Check model state
  EXPECT_NEAR(0.0, link->WorldPose().Pos().X(), tol);
  EXPECT_NEAR(0.0, link->WorldPose().Pos().Y(), tol);
  EXPECT_NEAR(0.5, link->WorldPose().Pos().Z(), tol);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
