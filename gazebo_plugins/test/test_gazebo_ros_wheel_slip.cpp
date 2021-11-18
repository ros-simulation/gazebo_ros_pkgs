// Copyright 2021 Open Source Robotics Foundation, Inc.
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
#include <gazebo_msgs/msg/instant_slip.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <memory>
#include <cmath>

#define tol 10e-4

using namespace std::literals::chrono_literals; // NOLINT

class GazeboRosWheelSlipTest : public gazebo::ServerFixture
{
};

TEST_F(GazeboRosWheelSlipTest, Publishing)
{
  // Load test world and start paused
  this->Load("worlds/gazebo_ros_wheel_slip.world", true);

  // World
  auto world = gazebo::physics::get_world();
  ASSERT_NE(nullptr, world);

  // Create node and executor
  auto node = std::make_shared<rclcpp::Node>("gazebo_ros_joint_state_publisher_test");
  ASSERT_NE(nullptr, node);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  // Create subscriber
  gazebo_msgs::msg::InstantSlip::SharedPtr latestMsg;
  auto sub = node->create_subscription<gazebo_msgs::msg::InstantSlip>(
    "trisphere_cycle_slip/wheel_slip", rclcpp::QoS(1),
    [&latestMsg](const gazebo_msgs::msg::InstantSlip::SharedPtr msg) {
      latestMsg = msg;
    });

  // Spin until we get a message or timeout
  auto startTime = std::chrono::steady_clock::now();
  while (latestMsg == nullptr &&
    (std::chrono::steady_clock::now() - startTime) < std::chrono::seconds(5))
  {
    world->Step(100);
    executor.spin_once(100ms);
    gazebo::common::Time::MSleep(100);
  }

  // Check that we receive the latest joint state
  ASSERT_NE(nullptr, latestMsg);

  ASSERT_LT(0u, latestMsg->name.size());
  ASSERT_GT(4u, latestMsg->name.size());
  ASSERT_EQ(latestMsg->name.size(), latestMsg->lateral_slip.size());
  ASSERT_EQ(latestMsg->name.size(), latestMsg->longitudinal_slip.size());

  EXPECT_NEAR(0.0, latestMsg->lateral_slip[0], tol);
  ASSERT_LT(0.0, fabs(latestMsg->longitudinal_slip[0]));
  ASSERT_GT(0.6, fabs(latestMsg->longitudinal_slip[0]));
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
