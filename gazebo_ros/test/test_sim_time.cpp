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
#include <gazebo_ros/testing_utils.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

#include <memory>
#include <string>
#include <vector>

using namespace std::literals::chrono_literals;

/// Tests the simtime published to /clock by gazebo_ros_init
class TestSimTime : public ::testing::Test
{
public:
  TestSimTime() {}
  void SetUp() override;
  void TearDown() override;

protected:
  std::unique_ptr<gazebo_ros::GazeboProcess> gazebo_process_;
};

void TestSimTime::SetUp()
{
  gazebo_process_ = std::make_unique<gazebo_ros::GazeboProcess>(
    std::vector<const char *>{"-s", "libgazebo_ros_init.so"});
  ASSERT_GT(gazebo_process_->Run(), 0);
}

void TestSimTime::TearDown()
{
  ASSERT_GE(gazebo_process_->Terminate(), 0);
  gazebo_process_.reset();
}

TEST_F(TestSimTime, TestClock)
{
  using ClockMsg = rosgraph_msgs::msg::Clock;

  // Create a node which will use sim time
  auto node = std::make_shared<rclcpp::Node>("test_sim_time");
  node->set_parameter({"use_sim_time", true});

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::vector<ClockMsg::SharedPtr> msgs;
  std::vector<rclcpp::Time> times;
  auto sub = node->create_subscription<ClockMsg>(
    "/clock", rclcpp::SystemDefaultsQoS(),
    [&](ClockMsg::SharedPtr _msg) {
      msgs.push_back(_msg);
      times.push_back(node->now());
    });

  unsigned int sleep{0};
  unsigned int max_sleep{100};
  while (msgs.size() < 5 && sleep++ < max_sleep) {
    executor.spin_once(100ms);
  }
  EXPECT_LT(sleep, max_sleep);
  EXPECT_EQ(5u, msgs.size());
  EXPECT_EQ(5u, times.size());

  // Check node starts at time zero
  for (int i = 0; i < 5; ++i) {
    // The SIM time should be close to zero (start of simulation)
    EXPECT_LT(rclcpp::Time(msgs[i]->clock), rclcpp::Time(1, 0, RCL_ROS_TIME));
    EXPECT_LT(times[i], rclcpp::Time(1, 0, RCL_ROS_TIME));

    // Time should go forward
    if (i > 0) {
      EXPECT_GT(rclcpp::Time(msgs[i]->clock), rclcpp::Time(msgs[i - 1]->clock)) <<
        rclcpp::Time(msgs[i]->clock).nanoseconds() << "    " <<
        rclcpp::Time(msgs[i - 1]->clock).nanoseconds();
      EXPECT_GT(times[i], times[i - 1]) << times[i].nanoseconds() << "    " <<
        times[i - 1].nanoseconds();

      // The message from the clock should be close to the node's time
      // We can't guarantee they match exactly because the node may process clock messages late
      EXPECT_NEAR(times[i].nanoseconds(), rclcpp::Time(msgs[i]->clock).nanoseconds(), 150000000);
    }
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
