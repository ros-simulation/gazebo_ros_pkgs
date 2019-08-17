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
#include <geometry_msgs/msg/wrench.hpp>

#include <memory>

#define tol 10e-10

enum AXIS
{
  X, Y, Z
};

class GazeboRosForceTest : public gazebo::ServerFixture
{
public:
  void ApplyForceTorque(AXIS force_axis, double init_rotation = 0)
  {
    // World
    auto world = gazebo::physics::get_world();
    ASSERT_NE(nullptr, world);

    // Box
    auto box = world->ModelByName("box");
    ASSERT_NE(nullptr, box);

    // Check plugin was loaded
    world->Step(100);
    EXPECT_EQ(1u, box->GetPluginCount());

    // Check box is at world origin
    EXPECT_NEAR(0.0, box->WorldPose().Pos().X(), tol);
    EXPECT_NEAR(init_rotation, box->WorldPose().Rot().Yaw(), tol);

    // Create ROS publisher
    auto node = std::make_shared<rclcpp::Node>("gazebo_ros_force_test");
    ASSERT_NE(nullptr, node);

    auto pub = node->create_publisher<geometry_msgs::msg::Wrench>(
      "test/force_test", rclcpp::QoS(rclcpp::KeepLast(1)));

    // Wait for subscriber to come up
    unsigned int sleep = 0;
    unsigned int max_sleep = 30;
    while (sleep < max_sleep && pub->get_subscription_count() == 0u) {
      gazebo::common::Time::MSleep(100);
      sleep++;
    }
    EXPECT_LT(0u, pub->get_subscription_count());
    EXPECT_LT(sleep, max_sleep);

    // Apply force
    auto msg = geometry_msgs::msg::Wrench();
    switch (force_axis) {
      case AXIS::Y:
        msg.force.y = 10.0;
        break;
      case AXIS::Z:
        msg.force.z = 10.0;
        break;
      default:
        msg.force.x = 10.0;
        break;
    }
    msg.torque.z = 10.0;
    pub->publish(msg);

    // Wait until box moves
    sleep = 0;
    double target_dist{0.1};
    while (sleep < max_sleep && box->WorldPose().Pos().X() < target_dist) {
      world->Step(100);
      gazebo::common::Time::MSleep(100);
      sleep++;
    }

    // Check box moved
    EXPECT_LT(sleep, max_sleep);
    EXPECT_LT(target_dist, box->WorldPose().Pos().X());
    EXPECT_LT(target_dist + init_rotation, box->WorldPose().Rot().Yaw());
  }
};

TEST_F(GazeboRosForceTest, ApplyForceTorqueWorld)
{
  // Load test world and start paused
  this->Load("worlds/gazebo_ros_force.world", true);

  this->ApplyForceTorque(AXIS::X);
}

TEST_F(GazeboRosForceTest, ApplyForceTorqueLink)
{
  // Load test world and start paused
  this->Load("worlds/gazebo_ros_force_link.world", true);

  this->ApplyForceTorque(AXIS::Y, -M_PI / 2);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
