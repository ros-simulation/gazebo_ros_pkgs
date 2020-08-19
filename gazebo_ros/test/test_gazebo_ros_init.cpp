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

#include <gazebo/test/ServerFixture.hh>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>
#include <memory>

class GazeboRosInitTest : public gazebo::ServerFixture
{
};

// Since the plugin calls rclcpp:init, and that can be called only once, we can only run one test
TEST_F(GazeboRosInitTest, Commands)
{
  // Load empty world with init plugin and start paused
  this->LoadArgs("-u --verbose -s libgazebo_ros_init.so worlds/free_fall.world");

  // World
  auto world = gazebo::physics::get_world();
  ASSERT_NE(nullptr, world);

  // Model
  auto model = world->ModelByName("box");
  ASSERT_NE(nullptr, model);

  // ROS node
  auto node = std::make_shared<rclcpp::Node>("gazebo_ros_init_test");
  ASSERT_NE(nullptr, node);

  // Pause / unpause simulation
  {
    // Check world is paused
    EXPECT_TRUE(world->IsPaused());

    // Create unpause client
    auto unpause_physics_client = node->create_client<std_srvs::srv::Empty>("unpause_physics");
    ASSERT_NE(nullptr, unpause_physics_client);
    EXPECT_TRUE(unpause_physics_client->wait_for_service(std::chrono::seconds(1)));

    // Request
    auto request = std::make_shared<std_srvs::srv::Empty::Request>();

    auto response_future = unpause_physics_client->async_send_request(request);
    EXPECT_EQ(
      rclcpp::FutureReturnCode::SUCCESS,
      rclcpp::spin_until_future_complete(node, response_future));

    auto response = response_future.get();
    ASSERT_NE(nullptr, response);

    // Check world is unpaused
    EXPECT_FALSE(world->IsPaused());

    // Create pause client
    auto pause_physics_client = node->create_client<std_srvs::srv::Empty>("pause_physics");
    ASSERT_NE(nullptr, pause_physics_client);
    EXPECT_TRUE(pause_physics_client->wait_for_service(std::chrono::seconds(1)));

    // Request
    response_future = pause_physics_client->async_send_request(request);
    EXPECT_EQ(
      rclcpp::FutureReturnCode::SUCCESS,
      rclcpp::spin_until_future_complete(node, response_future));

    response = response_future.get();
    ASSERT_NE(nullptr, response);

    // Check world is paused
    EXPECT_TRUE(world->IsPaused());
  }

  // Reset
  {
    // Step the world so the box falls a bit
    world->Step(1000);

    // Check time increases and model moves
    EXPECT_GE(world->SimTime(), gazebo::common::Time(1.0));
    EXPECT_LT(model->WorldPose().Pos().Z(), 0.0);

    // Create reset simulation client
    auto reset_simulation_client = node->create_client<std_srvs::srv::Empty>("reset_simulation");
    ASSERT_NE(nullptr, reset_simulation_client);
    EXPECT_TRUE(reset_simulation_client->wait_for_service(std::chrono::seconds(1)));

    // Request
    auto request = std::make_shared<std_srvs::srv::Empty::Request>();

    auto response_future = reset_simulation_client->async_send_request(request);
    EXPECT_EQ(
      rclcpp::FutureReturnCode::SUCCESS,
      rclcpp::spin_until_future_complete(node, response_future));

    auto response = response_future.get();
    ASSERT_NE(nullptr, response);

    // Check time and model pose
    EXPECT_EQ(gazebo::common::Time(0.0), world->SimTime());
    EXPECT_GT(model->WorldPose().Pos().Z(), 0.0);

    // Step the world
    world->Step(1000);

    // Check time increases and model moves
    EXPECT_GE(world->SimTime(), gazebo::common::Time(1.0));
    EXPECT_LT(model->WorldPose().Pos().Z(), 0.0);

    // Create reset client
    auto reset_world_client = node->create_client<std_srvs::srv::Empty>("reset_world");
    ASSERT_NE(nullptr, reset_world_client);
    EXPECT_TRUE(reset_world_client->wait_for_service(std::chrono::seconds(1)));

    // Request
    request = std::make_shared<std_srvs::srv::Empty::Request>();

    response_future = reset_world_client->async_send_request(request);
    EXPECT_EQ(
      rclcpp::FutureReturnCode::SUCCESS,
      rclcpp::spin_until_future_complete(node, response_future));

    response = response_future.get();
    ASSERT_NE(nullptr, response);

    // Check model pose was reset, but not time
    EXPECT_GE(world->SimTime(), gazebo::common::Time(1.0));
    EXPECT_GT(model->WorldPose().Pos().Z(), 0.0);
  }

  auto reset_world_client = node->create_client<std_srvs::srv::Empty>("reset_world");
  ASSERT_NE(nullptr, reset_world_client);
  EXPECT_TRUE(reset_world_client->wait_for_service(std::chrono::seconds(1)));
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
