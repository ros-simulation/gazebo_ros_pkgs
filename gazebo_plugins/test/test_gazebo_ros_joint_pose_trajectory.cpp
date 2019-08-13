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
#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <memory>

#define tol 10e-2

using namespace std::literals::chrono_literals; // NOLINT

class GazeboRosJointPoseTrajectoryTest : public gazebo::ServerFixture
{
};

TEST_F(GazeboRosJointPoseTrajectoryTest, Publishing)
{
  // Load test world and start paused
  this->Load("worlds/gazebo_ros_joint_pose_trajectory.world", true);

  // World
  auto world = gazebo::physics::get_world();
  ASSERT_NE(nullptr, world);

  // Model
  auto model = world->ModelByName("double_pendulum_with_base");
  ASSERT_NE(nullptr, model);

  // Joint
  auto joint = model->GetJoint("upper_joint");
  ASSERT_NE(nullptr, joint);

  // Create node and executor
  auto node = std::make_shared<rclcpp::Node>("gazebo_ros_joint_pose_trajectory_test");
  ASSERT_NE(nullptr, node);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  // Create publisher
  auto pub = node->create_publisher<trajectory_msgs::msg::JointTrajectory>(
    "test/set_trajectory_test", rclcpp::QoS(rclcpp::KeepLast(1)));

  auto joint_cmd = trajectory_msgs::msg::JointTrajectory();
  joint_cmd.header.frame_id = "world";
  joint_cmd.joint_names.push_back("upper_joint");
  joint_cmd.points.resize(1);
  joint_cmd.points[0].positions.push_back(-1.57);

  pub->publish(joint_cmd);

  unsigned int sleep = 0;
  unsigned int max_sleep = 30;
  while (sleep < max_sleep) {
    world->Step(100);
    gazebo::common::Time::MSleep(100);
    executor.spin_once(100ms);
    pub->publish(joint_cmd);
    sleep++;
  }

  ASSERT_NEAR(joint->Position(0), -1.57, tol);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
