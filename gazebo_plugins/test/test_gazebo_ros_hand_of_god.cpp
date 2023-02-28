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
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>

#include <memory>

#define tol 10e-2

class GazeboRosHandOfGodTest : public gazebo::ServerFixture
{
};

TEST_F(GazeboRosHandOfGodTest, HandOfGodTransform)
{
  // Load test world and start paused
  this->Load("worlds/gazebo_ros_hand_of_god.world", true);

  // World
  auto world = gazebo::physics::get_world();
  ASSERT_NE(nullptr, world);

  // Box
  auto box = world->ModelByName("box");
  ASSERT_NE(nullptr, box);

  // Link
  auto link = box->GetLink("link");
  ASSERT_NE(nullptr, link);

  world->Step(100);

  // Check box is at world origin
  EXPECT_NEAR(0.0, box->WorldPose().Pos().X(), tol);
  EXPECT_NEAR(0.0, box->WorldPose().Pos().Y(), tol);
  // Height of box's center
  EXPECT_NEAR(0.5, box->WorldPose().Pos().Z(), tol);

  // Create ROS node
  auto node = std::make_shared<rclcpp::Node>("gazebo_ros_hand_of_god_test");
  ASSERT_NE(nullptr, node);

  auto tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(node);

  geometry_msgs::msg::TransformStamped msg;
  msg.header.frame_id = "world";
  msg.child_frame_id = "link_desired";
  msg.header.stamp = node->now();
  msg.transform.translation.z = 10;
  msg.transform.rotation.w = 1;

  unsigned int sleep = 0;
  unsigned int max_sleep = 30;
  while (sleep < max_sleep) {
    tf_broadcaster->sendTransform(msg);
    gazebo::common::Time::MSleep(100);
    world->Step(100);
    sleep++;
  }

  // Check box moved
  EXPECT_NEAR(0, box->WorldPose().Pos().X(), tol);
  EXPECT_NEAR(0, box->WorldPose().Pos().Y(), tol);
  EXPECT_NEAR(10, box->WorldPose().Pos().Z(), tol);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
