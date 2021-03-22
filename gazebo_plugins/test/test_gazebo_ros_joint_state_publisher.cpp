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
#include <sensor_msgs/msg/joint_state.hpp>

#include <chrono>
#include <memory>

#define tol 10e-10

using namespace std::literals::chrono_literals; // NOLINT

class GazeboRosJointStatePublisherTest : public gazebo::ServerFixture
{
};

TEST_F(GazeboRosJointStatePublisherTest, Publishing)
{
  // Load test world and start paused
  this->Load("worlds/gazebo_ros_joint_state_publisher.world", true);

  // World
  auto world = gazebo::physics::get_world();
  ASSERT_NE(nullptr, world);

  // Model
  auto revoluter = world->ModelByName("revoluter");
  ASSERT_NE(nullptr, revoluter);

  // Joint
  auto hinge = revoluter->GetJoint("hinge");
  ASSERT_NE(nullptr, hinge);

  // Check joint state
  EXPECT_NEAR(0.0, hinge->Position(), tol);
  EXPECT_NEAR(0.0, hinge->GetVelocity(0), tol);

  // Create node and executor
  auto node = std::make_shared<rclcpp::Node>("gazebo_ros_joint_state_publisher_test");
  ASSERT_NE(nullptr, node);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  // Create subscriber
  sensor_msgs::msg::JointState::SharedPtr latestMsg;
  auto sub = node->create_subscription<sensor_msgs::msg::JointState>(
    "test/joint_states_test", rclcpp::QoS(1),
    [&latestMsg](const sensor_msgs::msg::JointState::SharedPtr msg) {
      latestMsg = msg;
    });

  // Spin until we get a message or timeout
  auto startTime = std::chrono::steady_clock::now();
  while (latestMsg == nullptr &&
    (std::chrono::steady_clock::now() - startTime) < std::chrono::seconds(2))
  {
    world->Step(100);
    executor.spin_once(100ms);
    gazebo::common::Time::MSleep(100);
  }

  // Check that we receive the latest joint state
  ASSERT_NE(nullptr, latestMsg);

  ASSERT_EQ(1u, latestMsg->name.size());
  ASSERT_EQ(1u, latestMsg->position.size());
  ASSERT_EQ(1u, latestMsg->velocity.size());

  EXPECT_EQ("hinge", latestMsg->name[0]);
  EXPECT_NEAR(hinge->Position(), latestMsg->position[0], tol);
  EXPECT_NEAR(hinge->GetVelocity(0), latestMsg->velocity[0], tol);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
