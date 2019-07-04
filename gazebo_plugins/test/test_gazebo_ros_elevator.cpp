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
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <memory>
#include <string>

#define tol 10e-2

using namespace std::literals::chrono_literals; // NOLINT

class GazeboRosElevatorTest : public gazebo::ServerFixture
{
};

TEST_F(GazeboRosElevatorTest, Publishing)
{
  // Load test world and start paused
  this->Load("worlds/gazebo_ros_elevator.world", true);

  // World
  auto world = gazebo::physics::get_world();
  ASSERT_NE(nullptr, world);

  // Model
  auto elevator = world->ModelByName("elevator");
  ASSERT_NE(nullptr, elevator);

  // Link
  auto link = elevator->GetLink("link");
  ASSERT_NE(nullptr, link);

  // Step a bit for model to settle
  world->Step(100);

  // Check elevator state
  EXPECT_NEAR(0.0, link->WorldPose().Pos().X(), tol);
  EXPECT_NEAR(0.0, link->WorldPose().Pos().Y(), tol);
  EXPECT_NEAR(0.075, link->WorldPose().Pos().Z(), tol);

  // Create node and executor
  auto node = std::make_shared<rclcpp::Node>("gazebo_ros_elevator_test");
  ASSERT_NE(nullptr, node);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  // Publisher for elevator command
  auto pub = node->create_publisher<std_msgs::msg::String>(
    "test/elevator_test", rclcpp::QoS(rclcpp::KeepLast(1)));

  auto msg = std_msgs::msg::String();
  msg.data = "1";

  double sleep = 0;
  double max_sleep = 100;
  for (; sleep < max_sleep; ++sleep) {
    pub->publish(msg);
    executor.spin_once(100ms);
    world->Step(100);
  }

  // Check model state
  EXPECT_NEAR(0.0, link->WorldPose().Pos().X(), tol);
  EXPECT_NEAR(0.0, link->WorldPose().Pos().Y(), tol);
  EXPECT_NEAR(3.075, link->WorldPose().Pos().Z(), tol);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
