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
#include <gazebo_msgs/srv/apply_link_wrench.hpp>
#include <gazebo_msgs/srv/link_request.hpp>
#include <rclcpp/rclcpp.hpp>
#include <memory>

#define tol 10e-2

class GazeboRosLinkWrenchTest : public gazebo::ServerFixture
{
};

TEST_F(GazeboRosLinkWrenchTest, LinkWrenchTest)
{
  // Load test world
  this->LoadArgs(
    "worlds/gazebo_ros_link_wrench_test.world -u --verbose -s libgazebo_ros_force_system.so");

  // World
  auto world = gazebo::physics::get_world();
  ASSERT_NE(nullptr, world);

  // Create ROS clients
  auto node = std::make_shared<rclcpp::Node>("gazebo_ros_link_wrench_test");
  ASSERT_NE(nullptr, node);

  auto apply_link_wrench =
    node->create_client<gazebo_msgs::srv::ApplyLinkWrench>("apply_link_wrench");
  ASSERT_NE(nullptr, apply_link_wrench);
  EXPECT_TRUE(apply_link_wrench->wait_for_service(std::chrono::seconds(1)));

  auto clear_link_wrenches =
    node->create_client<gazebo_msgs::srv::LinkRequest>("clear_link_wrenches");
  ASSERT_NE(nullptr, clear_link_wrenches);
  EXPECT_TRUE(clear_link_wrenches->wait_for_service(std::chrono::seconds(1)));

  auto apply_request = std::make_shared<gazebo_msgs::srv::ApplyLinkWrench::Request>();
  apply_request->link_name = "box::base";
  apply_request->wrench.force.x = 10;
  apply_request->wrench.force.y = 10;
  apply_request->wrench.force.z = 10;
  apply_request->wrench.torque.x = 10;
  apply_request->wrench.torque.y = 10;
  apply_request->wrench.torque.z = 10;
  apply_request->duration = rclcpp::Duration(-1, 0);

  auto apply_response_future = apply_link_wrench->async_send_request(apply_request);
  EXPECT_EQ(
    rclcpp::FutureReturnCode::SUCCESS,
    rclcpp::spin_until_future_complete(node, apply_response_future));

  auto apply_response = apply_response_future.get();
  ASSERT_NE(nullptr, apply_response);
  EXPECT_TRUE(apply_response->success);

  world->Step(1000);

  auto entity = boost::dynamic_pointer_cast<gazebo::physics::Link>(world->EntityByName("base"));
  EXPECT_NEAR(entity->WorldForce().X(), 10, tol);
  EXPECT_NEAR(entity->WorldForce().Y(), 10, tol);
  EXPECT_NEAR(entity->WorldForce().Z(), 10, tol);
  EXPECT_NEAR(entity->WorldTorque().X(), 10, tol);
  EXPECT_NEAR(entity->WorldTorque().Y(), 10, tol);
  EXPECT_NEAR(entity->WorldTorque().Z(), 10, tol);

  auto clear_request = std::make_shared<gazebo_msgs::srv::LinkRequest::Request>();
  clear_request->link_name = "box::base";

  auto clear_response_future = clear_link_wrenches->async_send_request(clear_request);
  EXPECT_EQ(
    rclcpp::FutureReturnCode::SUCCESS,
    rclcpp::spin_until_future_complete(node, clear_response_future));

  auto clear_response = clear_response_future.get();
  ASSERT_NE(nullptr, clear_response);

  world->Step(1000);

  EXPECT_NEAR(entity->WorldForce().X(), 0, tol);
  EXPECT_NEAR(entity->WorldForce().Y(), 0, tol);
  EXPECT_NEAR(entity->WorldForce().Z(), 0, tol);
  EXPECT_NEAR(entity->WorldTorque().X(), 0, tol);
  EXPECT_NEAR(entity->WorldTorque().Y(), 0, tol);
  EXPECT_NEAR(entity->WorldTorque().Z(), 0, tol);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
