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
#include <gazebo_msgs/srv/apply_joint_effort.hpp>
#include <gazebo_msgs/srv/joint_request.hpp>
#include <rclcpp/rclcpp.hpp>
#include <memory>

#define tol 10e-2

class GazeboRosJointEffortTest : public gazebo::ServerFixture
{
};

TEST_F(GazeboRosJointEffortTest, JointEffortTest)
{
  // Load test world
  this->LoadArgs(
    "worlds/gazebo_ros_joint_effort_test.world -u --verbose -s libgazebo_ros_force_system.so");

  // World
  auto world = gazebo::physics::get_world();
  ASSERT_NE(nullptr, world);

  // Create ROS clients
  auto node = std::make_shared<rclcpp::Node>("gazebo_ros_joint_effort_test");
  ASSERT_NE(nullptr, node);

  auto apply_joint_effort =
    node->create_client<gazebo_msgs::srv::ApplyJointEffort>("apply_joint_effort");
  ASSERT_NE(nullptr, apply_joint_effort);
  EXPECT_TRUE(apply_joint_effort->wait_for_service(std::chrono::seconds(1)));

  auto clear_joint_efforts =
    node->create_client<gazebo_msgs::srv::JointRequest>("clear_joint_efforts");
  ASSERT_NE(nullptr, clear_joint_efforts);
  EXPECT_TRUE(clear_joint_efforts->wait_for_service(std::chrono::seconds(1)));

  auto apply_request = std::make_shared<gazebo_msgs::srv::ApplyJointEffort::Request>();
  apply_request->joint_name = "upper_joint";
  apply_request->effort = 10.0;
  apply_request->duration = rclcpp::Duration(-1, 0);

  auto apply_response_future = apply_joint_effort->async_send_request(apply_request);
  EXPECT_EQ(
    rclcpp::FutureReturnCode::SUCCESS,
    rclcpp::spin_until_future_complete(node, apply_response_future));

  auto apply_response = apply_response_future.get();
  ASSERT_NE(nullptr, apply_response);
  EXPECT_TRUE(apply_response->success);

  world->Step(1000);

  auto entity = world->ModelByName("single_pendulum_with_base")->GetJoint("upper_joint");
  EXPECT_NEAR(entity->GetForce(0), 10, tol);

  auto clear_request = std::make_shared<gazebo_msgs::srv::JointRequest::Request>();
  clear_request->joint_name = "upper_joint";

  auto clear_response_future = clear_joint_efforts->async_send_request(clear_request);
  EXPECT_EQ(
    rclcpp::FutureReturnCode::SUCCESS,
    rclcpp::spin_until_future_complete(node, clear_response_future));

  auto clear_response = clear_response_future.get();
  ASSERT_NE(nullptr, clear_response);

  world->Step(1000);

  EXPECT_NEAR(entity->GetForce(0), 10, tol);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
