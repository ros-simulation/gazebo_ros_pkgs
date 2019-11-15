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

#include <memory>

#define tol 10e-10

using namespace std::literals::chrono_literals; // NOLINT

class GazeboRosJointEffortTest : public gazebo::ServerFixture
{
};

TEST_F(GazeboRosJointEffortTest, Publishing)
{
  // Load test world and start paused
  this->Load("worlds/gazebo_ros_joint_effort.world", true);

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
  auto node = std::make_shared<rclcpp::Node>("gazebo_ros_joint_effort_test");
  ASSERT_NE(nullptr, node);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  // Step a bit before starting
  world->Step(500);
  executor.spin_once(500ms);
  gazebo::common::Time::MSleep(100);

  // Create publisher
  auto joint_cmd = sensor_msgs::msg::JointState();
  auto pub = node->create_publisher<sensor_msgs::msg::JointState>(
    "/test/joint_efforts", rclcpp::QoS(rclcpp::KeepLast(1)));
  joint_cmd.header.frame_id = "world";
  joint_cmd.name.push_back("hinge");
  joint_cmd.effort.push_back(5);

  pub->publish(joint_cmd);

  unsigned int sleep = 0;
  unsigned int max_sleep = 30;
  while (sleep < max_sleep) {
    world->Step(100);
    gazebo::common::Time::MSleep(100);
    executor.spin_once(100ms);
    pub->publish(joint_cmd);
    sleep++;
  }
  ASSERT_GT(hinge->GetVelocity(0), 0);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
