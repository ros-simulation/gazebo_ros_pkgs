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
#include <gazebo_ros/node.hpp>
#include <gazebo_ros/testing_utils.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>

#include <memory>

#define tol 10e-4
#define tol_vel 10e-3

/// Tests the gazebo_ros_gps_sensor plugin
class GazeboRosGpsSensorTest : public gazebo::ServerFixture
{
};

TEST_F(GazeboRosGpsSensorTest, GpsMessageCorrect)
{
  // Load test world and start paused
  this->Load("worlds/gazebo_ros_gps_sensor.world", true);

  // World
  auto world = gazebo::physics::get_world();
  ASSERT_NE(nullptr, world);

  // Get the model with the attached GPS
  auto box = world->ModelByName("box");
  auto link = box->GetLink("link");
  ASSERT_NE(nullptr, box);

  // Make sure sensor is loaded
  ASSERT_EQ(link->GetSensorCount(), 1u);

  // Create node / executor for receiving gps message
  auto node = std::make_shared<rclcpp::Node>("test_gazebo_ros_gps_sensor");
  ASSERT_NE(nullptr, node);

  sensor_msgs::msg::NavSatFix::SharedPtr msg = nullptr;
  auto sub = node->create_subscription<sensor_msgs::msg::NavSatFix>(
    "/gps/data", rclcpp::SensorDataQoS(),
    [&msg](sensor_msgs::msg::NavSatFix::SharedPtr _msg) {
      msg = _msg;
    });
  geometry_msgs::msg::Vector3Stamped::SharedPtr msg_vel = nullptr;
  auto sub_vel = node->create_subscription<geometry_msgs::msg::Vector3Stamped>(
    "/gps/velocity", rclcpp::SensorDataQoS(),
    [&msg_vel](geometry_msgs::msg::Vector3Stamped::SharedPtr _msg) {
      msg_vel = _msg;
    });

  world->Step(1);
  EXPECT_NEAR(0.0, box->WorldPose().Pos().X(), tol);
  EXPECT_NEAR(0.0, box->WorldPose().Pos().Y(), tol);
  EXPECT_NEAR(0.5, box->WorldPose().Pos().Z(), tol);
  EXPECT_NEAR(0.0, box->WorldLinearVel().X(), tol_vel);
  EXPECT_NEAR(0.0, box->WorldLinearVel().Y(), tol_vel);
  EXPECT_NEAR(0.0, box->WorldLinearVel().Z(), tol_vel);

  // Step until a gps message will have been published
  int sleep{0};
  int max_sleep{1000};
  while (sleep < max_sleep && (nullptr == msg || nullptr == msg_vel)) {
    world->Step(100);
    rclcpp::spin_some(node);
    gazebo::common::Time::MSleep(100);
    sleep++;
  }
  EXPECT_LT(0u, sub->get_publisher_count());
  EXPECT_LT(sleep, max_sleep);
  ASSERT_NE(nullptr, msg);
  ASSERT_NE(nullptr, msg_vel);

  // Get the initial gps output when the box is at rest
  auto pre_movement_msg = std::make_shared<sensor_msgs::msg::NavSatFix>(*msg);
  auto pre_movement_msg_vel = std::make_shared<geometry_msgs::msg::Vector3Stamped>(*msg_vel);
  ASSERT_NE(nullptr, pre_movement_msg);
  EXPECT_NEAR(0.0, pre_movement_msg->latitude, tol);
  EXPECT_NEAR(0.0, pre_movement_msg->longitude, tol);
  EXPECT_NEAR(0.5, pre_movement_msg->altitude, tol);
  EXPECT_NEAR(0.0, pre_movement_msg_vel->vector.x, tol_vel);
  EXPECT_NEAR(0.0, pre_movement_msg_vel->vector.y, tol_vel);
  EXPECT_NEAR(0.0, pre_movement_msg_vel->vector.z, tol_vel);

  // Change the position and velocity of the link
  // and step a few times to wait the ros message to be received
  msg = nullptr;
  msg_vel = nullptr;
  ignition::math::Pose3d box_pose;
  box_pose.Pos() = {100.0, 200.0, 300.0};
  link->SetWorldPose(box_pose);
  link->SetLinearVel({15.0, 10.0, 0.0});

  sleep = 0;
  while (sleep < max_sleep && (nullptr == msg || msg->altitude < 150 || nullptr == msg_vel)) {
    world->Step(50);
    rclcpp::spin_some(node);
    gazebo::common::Time::MSleep(100);
    sleep++;
  }

  // Check that GPS output reflects the position change
  auto post_movement_msg = std::make_shared<sensor_msgs::msg::NavSatFix>(*msg);
  auto post_movement_msg_vel = std::make_shared<geometry_msgs::msg::Vector3Stamped>(*msg_vel);
  ASSERT_NE(nullptr, post_movement_msg);
  ASSERT_NE(nullptr, post_movement_msg_vel);
  EXPECT_GT(post_movement_msg->latitude, 0.0);
  EXPECT_GT(post_movement_msg->longitude, 0.0);
  EXPECT_NEAR(300, post_movement_msg->altitude, 1);
  EXPECT_NEAR(15.0, post_movement_msg_vel->vector.x, tol_vel);
  EXPECT_NEAR(10.0, post_movement_msg_vel->vector.y, tol_vel);
  EXPECT_NEAR(0.0, post_movement_msg_vel->vector.z, 0.1);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
