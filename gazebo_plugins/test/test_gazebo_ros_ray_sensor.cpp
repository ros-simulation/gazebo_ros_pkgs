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
#include <gazebo/common/Events.hh>
#include <gazebo/test/ServerFixture.hh>
#include <gazebo_ros/conversions/geometry_msgs.hpp>
#include <gazebo_ros/conversions/sensor_msgs.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/range.hpp>

#include <memory>
#include <vector>
#include <algorithm>

using namespace std::literals::chrono_literals; // NOLINT

/// Tests the gazebo_ros_ray_sensor plugin
class GazeboRosRaySensorTest : public gazebo::ServerFixture
{
protected:
  using position_t = ignition::math::Vector3d;
  using positions_t = std::vector<position_t>;

  /// Maximum error distance a point and the ground truth
  static constexpr double POINT_DISTANCE_TOL = 0.06;

  /// Tollerance for error in doubles vs ground truth
  static constexpr double ROUNDING_ERROR_TOL = 1E-2;

  /// Verify that a point is close to one of the ground truth points
  bool VerifyPoint(const positions_t & positions, const ignition::math::Vector3d & point)
  {
    for (auto real_point : positions) {
      if (real_point.Distance(point) < POINT_DISTANCE_TOL) {return true;}
    }
    return false;
  }
};

TEST_F(GazeboRosRaySensorTest, CorrectOutput)
{
  // Load test world and start paused
  this->Load("worlds/gazebo_ros_ray_sensor.world", true);

  // World
  auto world = gazebo::physics::get_world();
  ASSERT_NE(nullptr, world);

  positions_t positions;
  std::vector<double> angles;
  std::vector<double> ranges;

  // Store ground truth for verifying pointcloud / laserscan
  for (auto model : world->Models()) {
    // Only store the objects of interest (spheres)
    if (model->GetName().find("sphere") != 0) {
      continue;
    }

    // Push object position into vector for ground truth
    auto position = model->WorldPose().Pos();
    positions.push_back(position);

    // If object is in ground plane, store it in ground truth for LaserScan type
    if (position.Z() == 0.) {
      double angle = atan2(position.Y(), position.X());
      double range = position.Length();
      angles.push_back(angle);
      ranges.push_back(range);
    }
  }

  // Range message should return closest ray (minus radius of sphere)
  double min_range = *std::min_element(ranges.begin(), ranges.end()) - 0.05;

  // Create node and executor
  auto node = std::make_shared<rclcpp::Node>("gazebo_ros_joint_state_publisher_test");
  ASSERT_NE(nullptr, node);

  // Convienence function to subscribe to a topic and store it to a variable
  #define SUBSCRIBE_SETTER(msg, topic) \
  node->create_subscription<decltype(msg)::element_type>( \
    topic, rclcpp::SensorDataQoS(), \
    [&msg](decltype(msg) _msg) { \
      msg = _msg; \
    })

  // Create subscribe setter for each output type
  sensor_msgs::msg::LaserScan::SharedPtr ls = nullptr;
  sensor_msgs::msg::PointCloud::SharedPtr pc = nullptr;
  sensor_msgs::msg::PointCloud2::SharedPtr pc2 = nullptr;
  sensor_msgs::msg::Range::SharedPtr range = nullptr;
  auto ls_sub = SUBSCRIBE_SETTER(ls, "/ray/laserscan");
  auto pc_sub = SUBSCRIBE_SETTER(pc, "/ray/pointcloud");
  auto pc2_sub = SUBSCRIBE_SETTER(pc2, "/ray/pointcloud2");
  auto range_sub = SUBSCRIBE_SETTER(range, "/ray/range");

  #undef SUBSCRIBE_SETTER

  // Step world enough for one message to be published
  world->Step(400);

  // Receive messages
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin_some();

  // Ensure every message was received
  ASSERT_NE(ls, nullptr);
  ASSERT_NE(pc, nullptr);
  ASSERT_NE(pc2, nullptr);
  ASSERT_NE(range, nullptr);

  // Range message verification
  EXPECT_EQ(range->header.frame_id, "ray_link");
  EXPECT_EQ(range->radiation_type, sensor_msgs::msg::Range::INFRARED);
  EXPECT_NEAR(range->field_of_view, 1.0472, ROUNDING_ERROR_TOL);
  EXPECT_NEAR(range->min_range, 0.05, ROUNDING_ERROR_TOL);
  EXPECT_NEAR(range->max_range, 50.0, ROUNDING_ERROR_TOL);
  EXPECT_NEAR(range->range, min_range, ROUNDING_ERROR_TOL);

  // PointCloud verification
  EXPECT_EQ(pc->header.frame_id, "ray_link");
  ASSERT_EQ(pc->channels.size(), 1u);
  ASSERT_EQ(pc->points.size(), pc->channels[0].values.size());
  auto point = pc->points.begin();
  auto intensity = pc->channels[0].values.begin();
  for (; point != pc->points.end();
    ++point, ++intensity)
  {
    EXPECT_TRUE(VerifyPoint(positions, gazebo_ros::Convert<position_t>(*point)));
    EXPECT_NEAR(*intensity, 80, ROUNDING_ERROR_TOL);
  }

  // PointCloud2 verification
  EXPECT_EQ(pc2->header.frame_id, "ray_link");
  auto pc2_iter_x = sensor_msgs::PointCloud2Iterator<float>(*pc2, "x");
  auto pc2_iter_y = sensor_msgs::PointCloud2Iterator<float>(*pc2, "y");
  auto pc2_iter_z = sensor_msgs::PointCloud2Iterator<float>(*pc2, "z");
  auto pc2_iter_intensity = sensor_msgs::PointCloud2Iterator<float>(*pc2, "intensity");
  for (; pc2_iter_x != pc2_iter_x.end();
    ++pc2_iter_x, ++pc2_iter_y, ++pc2_iter_z, ++pc2_iter_intensity)
  {
    auto point = position_t(*pc2_iter_x, *pc2_iter_y, *pc2_iter_z);
    EXPECT_TRUE(VerifyPoint(positions, point));
    EXPECT_NEAR(*pc2_iter_intensity, 80, ROUNDING_ERROR_TOL);
    // TODO(anyone): verify intensity
  }

  // LaserScan verification
  EXPECT_EQ(ls->header.frame_id, "ray_link");
  EXPECT_NEAR(ls->angle_min, -0.5236, ROUNDING_ERROR_TOL);
  EXPECT_NEAR(ls->angle_max, 0.5236, ROUNDING_ERROR_TOL);
  EXPECT_NEAR(ls->range_min, 0.05, ROUNDING_ERROR_TOL);
  EXPECT_NEAR(ls->range_max, 50.0, ROUNDING_ERROR_TOL);
  EXPECT_NEAR(
    ls->angle_increment, (ls->angle_max - ls->angle_min) / ls->ranges.size(), ROUNDING_ERROR_TOL);

  // Ensure each ground truth range is found in the laserscan
  for (size_t i = 0; i < ranges.size(); ++i) {
    double range = ranges[i];
    double angle = angles[i];
    // Check for a correct range at roughly the ground truth angle
    int idx = (angle - ls->angle_min) / ls->angle_increment;
    if (idx < 0) {idx = 0;}
    if (idx > static_cast<int>(ls->ranges.size())) {idx = ls->ranges.size() - 1;}
    EXPECT_NEAR(ls->ranges[idx], range, POINT_DISTANCE_TOL);
    EXPECT_NEAR(ls->intensities[idx], 80, ROUNDING_ERROR_TOL);
  }

  // Artificialy trigger sigInt
  // TODO(anyone): investigate why this is needed to avoid deadlock at shutdown
  gazebo::event::Events::sigInt();
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
