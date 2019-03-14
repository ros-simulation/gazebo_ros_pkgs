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

#include <gazebo/common/Time.hh>
#include <gazebo/test/ServerFixture.hh>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <memory>

using namespace std::literals::chrono_literals; // NOLINT

class GazeboRosVideoTest : public gazebo::ServerFixture
{
public:
  void TearDown() override
  {
    // Make sure they're destroyed even if test fails by ASSERT
    ServerFixture::TearDown();
  }
};

TEST_F(GazeboRosVideoTest, VideoSubscribeTest)
{
  // Load test world and start paused
  this->Load("worlds/gazebo_ros_video.world", true);

  // World
  auto world = gazebo::physics::get_world();
  ASSERT_NE(nullptr, world);

  // Model box
  auto box = world->ModelByName("box_display");
  ASSERT_NE(nullptr, box);

  // Step a bit for model to settle
  world->Step(100);

  // Create node and executor
  auto node = std::make_shared<rclcpp::Node>("gazebo_ros_video_test");
  ASSERT_NE(nullptr, node);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  // Send image
  auto pub = node->create_publisher<sensor_msgs::msg::Image>("test/video_test");

  auto msg = sensor_msgs::msg::Image();
  msg.height = 120;
  msg.width = 160;
  msg.encoding = "rgb8";
  msg.step = 3 * msg.width;
  for (unsigned int j = 0; j < msg.width * msg.height; j++) {
    msg.data.push_back(255);
    msg.data.push_back(255);
    msg.data.push_back(255);
  }
  pub->publish(msg);
  executor.spin_once(100ms);
  // Wait for it to be processed
  world->Step(1000);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
