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
#include <gazebo_ros/node.hpp>
#include <gazebo_ros/testing_utils.hpp>
#include <rclcpp/rclcpp.hpp>
#include <gazebo_msgs/msg/contacts_state.hpp>

#include <memory>

#define tol 10e-2

/// Tests the gazebo_ros_bumper plugin
class GazeboRosBumperTest : public gazebo::ServerFixture
{
};

TEST_F(GazeboRosBumperTest, BumperMessageCorrect)
{
  // Load test world and start paused
  this->Load("worlds/gazebo_ros_bumper.world", true);

  // World
  auto world = gazebo::physics::get_world();
  ASSERT_NE(nullptr, world);

  // Model
  auto ball = world->ModelByName("ball");
  ASSERT_NE(nullptr, ball);

  // Make sure sensor is loaded
  auto link = ball->GetLink("link");
  ASSERT_EQ(link->GetSensorCount(), 1u);

  // Create node / executor for receiving bumper contact message
  auto node = std::make_shared<rclcpp::Node>("test_gazebo_ros_bumper");
  ASSERT_NE(nullptr, node);

  gazebo_msgs::msg::ContactsState::SharedPtr msg = nullptr;
  auto sub =
    node->create_subscription<gazebo_msgs::msg::ContactsState>(
    "/test/bumper_test", rclcpp::SensorDataQoS(),
    [&msg](gazebo_msgs::msg::ContactsState::SharedPtr _msg) {
      msg = _msg;
    });

  // Step until a contact message will have been published
  int sleep{0};
  int max_sleep{30};
  while (sleep < max_sleep && nullptr == msg) {
    world->Step(100);
    rclcpp::spin_some(node);
    gazebo::common::Time::MSleep(100);
    sleep++;
  }
  EXPECT_LT(0u, sub->get_publisher_count());
  EXPECT_LT(sleep, max_sleep);
  ASSERT_NE(nullptr, msg);

  EXPECT_NEAR(0.0, msg->states[0].total_wrench.force.x, tol);
  EXPECT_NEAR(0.0, msg->states[0].total_wrench.force.y, tol);
  // force z = body mass * gravity_acceleration
  EXPECT_NEAR(0.25, msg->states[0].total_wrench.force.z, tol);
  EXPECT_NEAR(0.0, msg->states[0].total_wrench.torque.x, tol);
  EXPECT_NEAR(0.0, msg->states[0].total_wrench.torque.y, tol);
  EXPECT_NEAR(0.0, msg->states[0].total_wrench.torque.z, tol);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
