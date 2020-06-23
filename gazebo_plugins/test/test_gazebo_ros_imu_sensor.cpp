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
#include <gazebo_ros/conversions/geometry_msgs.hpp>
#include <gazebo_ros/node.hpp>
#include <gazebo_ros/testing_utils.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <memory>

/// Tests the gazebo_ros_imu_sensor plugin
class GazeboRosImuSensorTest : public gazebo::ServerFixture
{
};

TEST_F(GazeboRosImuSensorTest, ImuMessageCorrect)
{
  // Load test world and start paused
  this->Load("worlds/gazebo_ros_imu_sensor.world", true);

  // World
  auto world = gazebo::physics::get_world();
  ASSERT_NE(nullptr, world);

  // Get the model with the attached IMU
  auto box = world->ModelByName("box");
  auto link = box->GetLink("link");
  ASSERT_NE(nullptr, box);

  // Make sure sensor is loaded
  ASSERT_EQ(link->GetSensorCount(), 1u);

  // Create node / executor for receiving imu message
  auto node = std::make_shared<rclcpp::Node>("test_gazebo_ros_imu_sensor");
  ASSERT_NE(nullptr, node);

  sensor_msgs::msg::Imu::SharedPtr msg = nullptr;
  auto sub = node->create_subscription<sensor_msgs::msg::Imu>(
    "/imu/data", rclcpp::SensorDataQoS(),
    [&msg](sensor_msgs::msg::Imu::SharedPtr _msg) {
      msg = _msg;
    });

  // Step until an imu message will have been published
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

  // Get the initial imu output when the box is at rest
  auto pre_movement_msg = std::make_shared<sensor_msgs::msg::Imu>(*msg);
  ASSERT_NE(nullptr, pre_movement_msg);
  auto pre_movement_yaw =
    gazebo_ros::Convert<ignition::math::Quaterniond>(pre_movement_msg->orientation).Euler().Z();
  EXPECT_LT(pre_movement_yaw, 0.05);
  EXPECT_LT(pre_movement_msg->linear_acceleration.x, 0.5);
  EXPECT_LT(pre_movement_msg->angular_velocity.z, 0.1);

  // Apply a force + torque and collect a new message
  msg = nullptr;
  for (unsigned int i = 0; i < 5; i++) {
    // Small steps so the force is continually applied
    link->SetForce({500.0, 0.0, 0.0});
    link->SetTorque({0.0, 0.0, 500.0});
    world->Step(50);
  }
  rclcpp::spin_some(node);

  // Check that IMU output reflects state changes due to applied force
  auto post_movement_msg = std::make_shared<sensor_msgs::msg::Imu>(*msg);
  ASSERT_NE(nullptr, post_movement_msg);
  auto post_movement_yaw =
    gazebo_ros::Convert<ignition::math::Quaterniond>(post_movement_msg->orientation).Euler().Z();
  EXPECT_GT(post_movement_yaw, 0.05);
  // The linear acceleration reported by Gazebo flips signs, so we take the absolute value
  EXPECT_GT(std::abs(post_movement_msg->linear_acceleration.x), 1.0);
  EXPECT_GT(post_movement_msg->angular_velocity.z, 1.0);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
