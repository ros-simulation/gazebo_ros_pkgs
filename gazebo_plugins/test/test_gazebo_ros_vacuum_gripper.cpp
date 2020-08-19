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
#include <std_srvs/srv/set_bool.hpp>

#include <memory>

#define tol 10e-2

class GazeboRosVacuumGripperTest : public gazebo::ServerFixture
{
};

TEST_F(GazeboRosVacuumGripperTest, VacuumGripperServiceTest)
{
  // Load test world and start paused
  this->Load("worlds/gazebo_ros_vacuum_gripper.world", true);

  // World
  auto world = gazebo::physics::get_world();
  ASSERT_NE(nullptr, world);

  // Box
  auto box = world->ModelByName("box");
  ASSERT_NE(nullptr, box);

  // Ball1
  auto ball1 = world->ModelByName("ball1");
  ASSERT_NE(nullptr, ball1);

  // Ball2
  auto ball2 = world->ModelByName("ball2");
  ASSERT_NE(nullptr, ball2);

  // Check plugin was loaded
  world->Step(100);
  EXPECT_EQ(1u, box->GetPluginCount());

  // Check ball1 position
  EXPECT_NEAR(0.6, ball1->WorldPose().Pos().X(), tol);
  EXPECT_NEAR(0.0, ball1->WorldPose().Pos().Y(), tol);

  // Check ball2 position
  EXPECT_NEAR(-0.6, ball2->WorldPose().Pos().X(), tol);
  EXPECT_NEAR(0.0, ball2->WorldPose().Pos().Y(), tol);

  // Create ROS publisher
  auto node = std::make_shared<rclcpp::Node>("gazebo_ros_vacuum_gripper_test");
  ASSERT_NE(nullptr, node);

  auto client = node->create_client<std_srvs::srv::SetBool>("test/switch_test");
  ASSERT_NE(nullptr, client);
  EXPECT_TRUE(client->wait_for_service(std::chrono::seconds(1)));

  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = true;
  auto response_future = client->async_send_request(request);
  EXPECT_EQ(
    rclcpp::FutureReturnCode::SUCCESS,
    rclcpp::spin_until_future_complete(node, response_future));

  unsigned int sleep = 0;
  unsigned int max_sleep = 200;
  while (sleep < max_sleep &&
    (ball1->WorldPose().Pos().X() > 0.1 || ball2->WorldPose().Pos().Y() < -0.1))
  {
    gazebo::common::Time::MSleep(100);
    world->Step(100);
    sleep++;
  }

  // Check balls moved
  EXPECT_LT(sleep, max_sleep);
  EXPECT_LT(ball1->WorldPose().Pos().X(), 0.1);
  EXPECT_NE(ball1->WorldPose().Pos().Y(), 0);
  EXPECT_GT(ball2->WorldPose().Pos().X(), -0.1);
  EXPECT_NE(ball2->WorldPose().Pos().Y(), 0);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
