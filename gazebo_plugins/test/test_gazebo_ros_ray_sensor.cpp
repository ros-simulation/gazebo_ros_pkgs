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
#include <sensor_msgs/msg/range.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <atomic>

/// Tests the gazebo_ros_ray_sensor plugin
class TestGazeboRosRaySensor : public ::testing::Test
{
public:
  TestGazeboRosRaySensor() {}

  // Docmentation inherited
  static void SetUpTestCase();
  // Documentation inherited
  static void TearDownTestCase();

protected:
  /// command line arguments to pass to gzserver
  static const std::vector<const char *> args;
  /// Wrapper for gzserver run with plugin
  static std::unique_ptr<gazebo_ros::GazeboProcess> gazebo_process_;
  /// ROS node used to verify output of plugin
  static rclcpp::Node::SharedPtr node_;
  /// ROS executor to call node_'s callbacks
  static rclcpp::executor::Executor::SharedPtr executor_;

  /// Get a message from a topic or nullptr of none received before timeout
  /// \return The first message received on that topic or nullptr
  template<typename T>
  static typename T::SharedPtr get_message(const std::string & topic, rclcpp::Duration timeout);
};

using namespace std::literals::chrono_literals;

// Initialize statics
const std::vector<const char *>
TestGazeboRosRaySensor::args = {"worlds/gazebo_ros_ray_sensor.world"};

std::unique_ptr<gazebo_ros::GazeboProcess> TestGazeboRosRaySensor::gazebo_process_ = nullptr;
rclcpp::Node::SharedPtr TestGazeboRosRaySensor::node_ = nullptr;
rclcpp::executor::Executor::SharedPtr TestGazeboRosRaySensor::executor_ = nullptr;

void TestGazeboRosRaySensor::SetUpTestCase()
{
  // Start gzserver in background
  gazebo_process_ = std::make_unique<gazebo_ros::GazeboProcess>(args);
  ASSERT_GT(gazebo_process_->run(), 0);

  // Create node and executor
  node_ = rclcpp::Node::make_shared("test_gazebo_ros_node");
  executor_ = rclcpp::executors::SingleThreadedExecutor::make_shared();
  executor_->add_node(node_);
}

void TestGazeboRosRaySensor::TearDownTestCase()
{
  // Terminate gzserver
  ASSERT_GE(gazebo_process_->terminate(), 0);
  gazebo_process_.reset();

  node_.reset();
  executor_.reset();
}

template<typename T>
typename T::SharedPtr
TestGazeboRosRaySensor::get_message(const std::string & _topic, rclcpp::Duration _timeout)
{
  std::atomic_bool msg_received(false);

  typename T::SharedPtr msg = nullptr;

  auto sub = node_->create_subscription<T>(_topic,
      [&msg_received, &msg](typename T::SharedPtr _msg) {
        (void) msg;
        // If this is the first message from this topic, increment the counter
        if (!msg_received.exchange(true)) {
          msg = _msg;
        }
      });

  // Wait until message is received or timeout occurs
  using namespace std::literals::chrono_literals;
  auto timeout = node_->now() + _timeout;
  while (false == msg_received && node_->now() < timeout) {
    executor_->spin_once(200ms);
  }

  return msg;
}

TEST_F(TestGazeboRosRaySensor, TestRange)
{
  auto range = get_message<sensor_msgs::msg::Range>("/ray/range", rclcpp::Duration(20s));
  ASSERT_NE(range, nullptr);
  EXPECT_EQ(range->header.frame_id, "ray_link");
  EXPECT_EQ(range->radiation_type, sensor_msgs::msg::Range::INFRARED);
  EXPECT_NEAR(range->field_of_view, 1.0472, 1E-4);
  EXPECT_NEAR(range->min_range, 0.05, 1E-4);
  EXPECT_NEAR(range->max_range, 50.0, 1E-4);
  EXPECT_NEAR(range->range, 1.45, 1E-4);
}

TEST_F(TestGazeboRosRaySensor, TestPointCloud)
{
  // TODO(ironmig): verify content of message
  auto pc = get_message<sensor_msgs::msg::PointCloud>("/ray/pointcloud", rclcpp::Duration(20s));
  ASSERT_NE(pc, nullptr);
}

TEST_F(TestGazeboRosRaySensor, TestPointCloud2)
{
  // TODO(ironmig): verify content of message
  auto pc = get_message<sensor_msgs::msg::PointCloud2>("/ray/pointcloud2", rclcpp::Duration(20s));
  ASSERT_NE(pc, nullptr);
}

TEST_F(TestGazeboRosRaySensor, TestLaserScan)
{
  // TODO(ironmig): verify content of message
  auto ls = get_message<sensor_msgs::msg::LaserScan>("/ray/laserscan", rclcpp::Duration(20s));
  ASSERT_NE(ls, nullptr);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
