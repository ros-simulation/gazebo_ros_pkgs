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
#include <gazebo_msgs/srv/delete_entity.hpp>
#include <gazebo_msgs/srv/spawn_entity.hpp>
#include <rclcpp/rclcpp.hpp>
#include <memory>

class GazeboRosFactoryTest : public gazebo::ServerFixture
{
};

// Since the plugin calls rclcpp:init, and that can be called only once, we can only run one test
TEST_F(GazeboRosFactoryTest, SpawnDelete)
{
  // Load empty world with factory plugin and start paused
  this->LoadArgs("-u --verbose -s libgazebo_ros_factory.so");

  // World
  auto world = gazebo::physics::get_world();
  ASSERT_NE(nullptr, world);

  // Create ROS clients
  auto node = std::make_shared<rclcpp::Node>("gazebo_ros_factory_test");
  ASSERT_NE(nullptr, node);

  auto spawn_client = node->create_client<gazebo_msgs::srv::SpawnEntity>("spawn_entity");
  ASSERT_NE(nullptr, spawn_client);
  EXPECT_TRUE(spawn_client->wait_for_service(std::chrono::seconds(1)));

  auto delete_client = node->create_client<gazebo_msgs::srv::DeleteEntity>("delete_entity");
  ASSERT_NE(nullptr, delete_client);
  EXPECT_TRUE(delete_client->wait_for_service(std::chrono::seconds(1)));

  // Spawn SDF model
  {
    // Check it has no box model
    EXPECT_EQ(nullptr, world->ModelByName("sdf_box"));

    // Request spawn
    auto request = std::make_shared<gazebo_msgs::srv::SpawnEntity::Request>();
    request->name = "sdf_box";
    request->initial_pose.position.x = 1.0;
    request->initial_pose.position.y = 2.0;
    request->initial_pose.position.z = 3.0;
    request->xml =
      "<?xml version='1.0' ?>"
      "<sdf version='1.5'>"
      "<model name='ignored'>"
      "<static>true</static>"
      "<link name='link'>"
      "<visual name='visual'>"
      "<geometry>"
      "<sphere><radius>1.0</radius></sphere>"
      "</geometry>"
      "</visual>"
      "</link>"
      "</model>"
      "</sdf>";

    auto response_future = spawn_client->async_send_request(request);
    EXPECT_EQ(rclcpp::executor::FutureReturnCode::SUCCESS,
      rclcpp::spin_until_future_complete(node, response_future));

    auto response = response_future.get();
    ASSERT_NE(nullptr, response);
    EXPECT_TRUE(response->success);

    // Check it was spawned with the correct name
    ASSERT_NE(nullptr, world->ModelByName("sdf_box"));
    EXPECT_EQ(world->ModelByName("sdf_box")->WorldPose(), ignition::math::Pose3d(1, 2, 3, 0, 0, 0));
  }

  // Spawn URDF model
  {
    // Check it has no box model
    EXPECT_EQ(nullptr, world->ModelByName("urdf_box"));

    // Request spawn
    auto request = std::make_shared<gazebo_msgs::srv::SpawnEntity::Request>();
    request->initial_pose.position.x = 1.0;
    request->initial_pose.position.y = 2.0;
    request->initial_pose.position.z = 3.0;
    request->reference_frame = "sdf_box";
    request->xml =
      "<?xml version='1.0' ?>"
      "<robot name='urdf_box'>"
      "<link name='link'>"
      "<visual>"
      "<geometry>"
      "<sphere radius='1.0'/>"
      "</geometry>"
      "</visual>"
      "<inertial>"
      "<mass value='1'/>"
      "<inertia ixx='1' ixy='0.0' ixz='0.0' iyy='1' iyz='0.0' izz='1'/>"
      "</inertial>"
      "</link>"
      "</robot>";

    auto response_future = spawn_client->async_send_request(request);
    EXPECT_EQ(rclcpp::executor::FutureReturnCode::SUCCESS,
      rclcpp::spin_until_future_complete(node, response_future));

    auto response = response_future.get();
    ASSERT_NE(nullptr, response);
    EXPECT_TRUE(response->success);

    // Name was not set, so we wait for spawn
    unsigned int sleep{0};
    unsigned int max_sleep{30};
    while (sleep++ < max_sleep && !world->ModelByName("urdf_box")) {
      usleep(100000);
    }

    // Check it was spawned with the correct name
    ASSERT_NE(nullptr, world->ModelByName("urdf_box"));
    EXPECT_EQ(world->ModelByName("urdf_box")->WorldPose(),
      ignition::math::Pose3d(2, 4, 6, 0, 0, 0));
  }

  // Spawn SDF light
  {
    // Check it has no dir light
    EXPECT_EQ(nullptr, world->LightByName("dir"));

    // Request spawn
    auto request = std::make_shared<gazebo_msgs::srv::SpawnEntity::Request>();
    request->name = "dir";
    request->initial_pose.position.z = 10.0;
    request->reference_frame = "world";
    request->xml =
      "<?xml version='1.0' ?>"
      "<sdf version='1.5'>"
      "<light name='ignored' type='directional'/>"
      "</sdf>";

    auto response_future = spawn_client->async_send_request(request);
    EXPECT_EQ(rclcpp::executor::FutureReturnCode::SUCCESS,
      rclcpp::spin_until_future_complete(node, response_future));

    auto response = response_future.get();
    ASSERT_NE(nullptr, response);
    EXPECT_TRUE(response->success);

    // Check it was spawned with the correct name
    ASSERT_NE(nullptr, world->LightByName("dir"));
    EXPECT_EQ(world->LightByName("dir")->WorldPose(),
      ignition::math::Pose3d(0, 0, 10, 0, 0, 0));
  }

  // Delete model
  {
    // Check model exists
    EXPECT_NE(nullptr, world->ModelByName("ground_plane"));

    // Request delete
    auto request = std::make_shared<gazebo_msgs::srv::DeleteEntity::Request>();
    request->name = "ground_plane";

    auto response_future = delete_client->async_send_request(request);
    EXPECT_EQ(rclcpp::executor::FutureReturnCode::SUCCESS,
      rclcpp::spin_until_future_complete(node, response_future));

    auto response = response_future.get();
    ASSERT_NE(nullptr, response);
    EXPECT_TRUE(response->success);

    // Check it was deleted
    EXPECT_EQ(nullptr, world->ModelByName("ground_plane"));
  }

  // Delete light
  {
    // Check light exists
    EXPECT_NE(nullptr, world->LightByName("sun"));

    // Request delete
    auto request = std::make_shared<gazebo_msgs::srv::DeleteEntity::Request>();
    request->name = "sun";

    auto response_future = delete_client->async_send_request(request);
    EXPECT_EQ(rclcpp::executor::FutureReturnCode::SUCCESS,
      rclcpp::spin_until_future_complete(node, response_future));

    auto response = response_future.get();
    ASSERT_NE(nullptr, response);
    EXPECT_TRUE(response->success);

    // Check it was deleted
    EXPECT_EQ(nullptr, world->LightByName("sun"));
  }
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
