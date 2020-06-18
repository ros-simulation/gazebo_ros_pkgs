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

class GazeboRosVideoTest : public gazebo::RenderingFixture
{
public:
  void TearDown() override
  {
    // Make sure they're destroyed even if test fails by ASSERT
    RenderingFixture::TearDown();
  }
};

TEST_F(GazeboRosVideoTest, VideoSubscribeTest)
{
  // Load test world and start paused
  this->Load("worlds/gazebo_ros_video.world", true);

  // Get the scene
  auto scene = gazebo::rendering::get_scene();
  ASSERT_NE(nullptr, scene);

  // Trigger render events until model is loaded
  int sleep = 0;
  int max_sleep = 30;
  gazebo::rendering::VisualPtr model_vis;
  gazebo::rendering::VisualPtr link_vis;
  gazebo::rendering::VisualPtr visual_vis;
  gazebo::rendering::VisualPtr video_vis;
  for (; !model_vis && sleep < max_sleep; ++sleep) {
    gazebo::event::Events::preRender();
    gazebo::event::Events::render();
    gazebo::event::Events::postRender();
    model_vis = scene->GetVisual("box_display");
    link_vis = scene->GetVisual("box_display::base_link");
    visual_vis = scene->GetVisual("box_display::base_link::visual");
    video_vis = scene->GetVisual(
      "box_display::base_link::visual::video_visual::display_video_controller");
    gazebo::common::Time::MSleep(100);
  }
  EXPECT_LT(sleep, max_sleep);
  EXPECT_NE(nullptr, model_vis);
  EXPECT_NE(nullptr, link_vis);
  EXPECT_NE(nullptr, visual_vis);
  EXPECT_NE(nullptr, video_vis);

  // Create node and executor
  auto node = std::make_shared<rclcpp::Node>("gazebo_ros_video_test");
  ASSERT_NE(nullptr, node);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  // Send image
  auto pub = node->create_publisher<sensor_msgs::msg::Image>(
    "test/video_test", rclcpp::QoS(rclcpp::KeepLast(1)));

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

  // Give iterations for it to be processed. Right now we don't have a quick way to verify the
  // output, so we just make sure there's no crash
  sleep = 0;
  max_sleep = 10;
  for (; sleep < max_sleep; ++sleep) {
    pub->publish(msg);
    executor.spin_once(100ms);

    gazebo::event::Events::preRender();
    gazebo::event::Events::render();
    gazebo::event::Events::postRender();

    EXPECT_NE(nullptr, scene->GetVisual("box_display"));
    EXPECT_NE(
      nullptr, scene->GetVisual(
        "box_display::base_link::visual::video_visual::display_video_controller"));
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
