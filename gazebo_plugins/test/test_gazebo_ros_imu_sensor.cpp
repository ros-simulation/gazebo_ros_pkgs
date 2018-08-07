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
#include <gazebo_ros/testing_utils.hpp>
#include <gazebo_ros/node.hpp>
#include <gazebo_ros/conversions.hpp>
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
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  // Helper function to increment gazebo until a new imu message is received
  // TODO(ironmig): make more generic and put in gazebo_ros testing_utils
  auto step_for_msg =
    [&executor, &world, &node](size_t max_iterations = 100,
      size_t steps_per_iteration = 10) -> sensor_msgs::msg::Imu::SharedPtr {
      sensor_msgs::msg::Imu::SharedPtr msg = nullptr;
      auto sub =
        node->create_subscription<sensor_msgs::msg::Imu>("/imu/data",
          [&msg](sensor_msgs::msg::Imu::SharedPtr _msg) {
            msg = _msg;
          });
      for (size_t i = 0; i < max_iterations && !msg; ++i) {
        world->Step(steps_per_iteration);
        using namespace std::literals::chrono_literals;
        executor.spin_once(100ms);
      }
      return msg;
    };

  // Get the initial imu output when the box is still
  auto first_msg = step_for_msg(10, 100);
  ASSERT_NE(nullptr, first_msg);
  auto first_yaw =
    gazebo_ros::Convert<ignition::math::Quaterniond>(first_msg->orientation).Euler().Z();

  // Apply a force + torque and collect a new message
  link->SetForce({500.0, 0.0, 0.0});
  link->SetTorque({0.0, 0.0, 200.0});
  auto second_msg = step_for_msg(100, 10);
  ASSERT_NE(nullptr, second_msg);
  auto second_yaw =
    gazebo_ros::Convert<ignition::math::Quaterniond>(second_msg->orientation).Euler().Z();

  // Check that IMU output reflects state changes due to applied force
  EXPECT_GT(second_msg->linear_acceleration.x, first_msg->linear_acceleration.x);
  EXPECT_GT(second_msg->angular_velocity.z, first_msg->angular_velocity.z);
  EXPECT_GT(second_yaw, first_yaw);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
