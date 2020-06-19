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

#include <gtest/gtest.h>

#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/conversions/geometry_msgs.hpp>

TEST(TestConversions, Vector3)
{
  // Ign to ROS
  ignition::math::Vector3d vec(1.0, 2.0, 3.0);
  auto msg = gazebo_ros::Convert<geometry_msgs::msg::Vector3>(vec);
  EXPECT_EQ(1.0, msg.x);
  EXPECT_EQ(2.0, msg.y);
  EXPECT_EQ(3.0, msg.z);
  // ROS to Ign
  vec = gazebo_ros::Convert<ignition::math::Vector3d>(msg);
  EXPECT_EQ(1.0, vec.X());
  EXPECT_EQ(2.0, vec.Y());
  EXPECT_EQ(3.0, vec.Z());
}

TEST(TestConversions, Quaternion)
{
  // Ign to ROS
  ignition::math::Quaterniond quat(1.0, 0.2, 0.4, 0.6);
  auto quat_msg = gazebo_ros::Convert<geometry_msgs::msg::Quaternion>(quat);
  EXPECT_EQ(0.2, quat_msg.x);
  EXPECT_EQ(0.4, quat_msg.y);
  EXPECT_EQ(0.6, quat_msg.z);
  EXPECT_EQ(1.0, quat_msg.w);

  auto ign_quat = gazebo_ros::Convert<ignition::math::Quaterniond>(quat_msg);
  EXPECT_EQ(0.2, ign_quat.X());
  EXPECT_EQ(0.4, ign_quat.Y());
  EXPECT_EQ(0.6, ign_quat.Z());
  EXPECT_EQ(1.0, ign_quat.W());
}

TEST(TestConversions, Pose)
{
  // Ign to ROS Pose
  ignition::math::Pose3d pose(0.6, 0.5, 0.7, 0.9, 0.0, 0.3, -0.1);
  auto pose_msg = gazebo_ros::Convert<geometry_msgs::msg::Pose>(pose);
  EXPECT_EQ(0.6, pose_msg.position.x);
  EXPECT_EQ(0.5, pose_msg.position.y);
  EXPECT_EQ(0.7, pose_msg.position.z);
  EXPECT_EQ(0.9, pose_msg.orientation.w);
  EXPECT_EQ(0.0, pose_msg.orientation.x);
  EXPECT_EQ(0.3, pose_msg.orientation.y);
  EXPECT_EQ(-0.1, pose_msg.orientation.z);

  // ROS Pose to Ign
  auto pose_ign = gazebo_ros::Convert<ignition::math::Pose3d>(pose_msg);
  EXPECT_EQ(0.6, pose_ign.Pos().X());
  EXPECT_EQ(0.5, pose_ign.Pos().Y());
  EXPECT_EQ(0.7, pose_ign.Pos().Z());
  EXPECT_EQ(0.9, pose_ign.Rot().W());
  EXPECT_EQ(0.0, pose_ign.Rot().X());
  EXPECT_EQ(0.3, pose_ign.Rot().Y());
  EXPECT_EQ(-0.1, pose_ign.Rot().Z());

  // ROS Pose to ROS Transform
  auto transform_msg = gazebo_ros::Convert<geometry_msgs::msg::Transform>(pose_msg);
  EXPECT_EQ(0.6, transform_msg.translation.x);
  EXPECT_EQ(0.5, transform_msg.translation.y);
  EXPECT_EQ(0.7, transform_msg.translation.z);
  EXPECT_EQ(0.9, transform_msg.rotation.w);
  EXPECT_EQ(0.0, transform_msg.rotation.x);
  EXPECT_EQ(0.3, transform_msg.rotation.y);
  EXPECT_EQ(-0.1, transform_msg.rotation.z);
}

TEST(TestConversions, Time)
{
  // Gazebo time
  {
    gazebo::common::Time time(200, 100);

    // to rclcpp
    auto rostime = gazebo_ros::Convert<rclcpp::Time>(time);
    EXPECT_EQ(200E9 + 100u, rostime.nanoseconds());

    // to ros message
    auto time_msg = gazebo_ros::Convert<builtin_interfaces::msg::Time>(time);
    EXPECT_EQ(200, time_msg.sec);
    EXPECT_EQ(100u, time_msg.nanosec);

    // to Gazebo time
    auto gazebo_time = gazebo_ros::Convert<gazebo::common::Time>(time_msg);
    EXPECT_EQ(200, gazebo_time.sec);
    EXPECT_EQ(100, gazebo_time.nsec);
  }

  // Gazebo msg
  {
    gazebo::msgs::Time time;
    time.set_sec(200);
    time.set_nsec(100);

    // to ros message
    auto time_msg = gazebo_ros::Convert<builtin_interfaces::msg::Time>(time);
    EXPECT_EQ(200, time_msg.sec);
    EXPECT_EQ(100u, time_msg.nanosec);
  }

  // ROS Duration
  {
    auto duration = builtin_interfaces::msg::Duration();
    duration.sec = 200;
    duration.nanosec = 100u;

    // to Gazebo time
    auto gazebo_time = gazebo_ros::Convert<gazebo::common::Time>(duration);
    EXPECT_EQ(200, gazebo_time.sec);
    EXPECT_EQ(100, gazebo_time.nsec);
  }
}
