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

#include <gazebo/common/Time.hh>
#include <gazebo/test/ServerFixture.hh>
#include <gazebo_ros/conversions/geometry_msgs.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>

#include <memory>

#define tol 10e-2

using namespace std::literals::chrono_literals; // NOLINT

class GazeboRosFTSensorTest : public gazebo::ServerFixture
{
};

TEST_F(GazeboRosFTSensorTest, Publishing)
{
  // Load test world and start paused
  this->Load("worlds/gazebo_ros_ft_sensor.world", true);

  // World
  auto world = gazebo::physics::get_world();

  ASSERT_NE(nullptr, world);

  // Model
  auto model = world->ModelByName("box");
  ASSERT_NE(nullptr, model);

  // Link
  auto link = model->GetLink("link");
  ASSERT_NE(nullptr, link);

  // Step a bit for model to settle
  world->Step(100);

  // Create node and executor
  auto node = std::make_shared<rclcpp::Node>("gazebo_ros_ft_sensor_test");
  ASSERT_NE(nullptr, node);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  // Create subscriber
  geometry_msgs::msg::WrenchStamped::SharedPtr latest_msg{nullptr};
  auto sub = node->create_subscription<geometry_msgs::msg::WrenchStamped>(
    "test/ft_sensor_test", rclcpp::QoS(rclcpp::KeepLast(1)),
    [&latest_msg](const geometry_msgs::msg::WrenchStamped::SharedPtr _msg) {
      latest_msg = _msg;
    });

  // Wait for a message
  world->Step(1000);
  unsigned int max_sleep{30u};
  for (auto sleep = 0u; !latest_msg && sleep < max_sleep; ++sleep) {
    executor.spin_once(100ms);
    gazebo::common::Time::MSleep(1000);
  }

  // Check message
  ASSERT_NE(nullptr, latest_msg);
  EXPECT_EQ("test_world", latest_msg->header.frame_id);

  // The box is under free fall because of no ground plane.
  // Therefore the only force acting should be gravity.
  EXPECT_DOUBLE_EQ(0.0, latest_msg->wrench.force.x);
  EXPECT_DOUBLE_EQ(0.0, latest_msg->wrench.force.y);
  EXPECT_DOUBLE_EQ(-9.8, latest_msg->wrench.force.z);
  EXPECT_DOUBLE_EQ(0.0, latest_msg->wrench.torque.x);
  EXPECT_DOUBLE_EQ(0.0, latest_msg->wrench.torque.y);
  EXPECT_DOUBLE_EQ(0.0, latest_msg->wrench.torque.z);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
