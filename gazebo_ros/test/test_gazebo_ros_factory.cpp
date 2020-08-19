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
#include <gazebo_msgs/srv/get_model_list.hpp>
#include <gazebo_msgs/srv/delete_entity.hpp>
#include <gazebo_msgs/srv/spawn_entity.hpp>
#include <rclcpp/rclcpp.hpp>
#include <memory>

class GazeboRosFactoryTest : public gazebo::ServerFixture
{
};

// Since the plugin calls rclcpp:init, and that can be called only once, we can only run one test
TEST_F(GazeboRosFactoryTest, SpawnDeleteList)
{
  // Load empty world with factory plugin and start paused
  this->LoadArgs("-u --verbose -s libgazebo_ros_factory.so");

  // World
  auto world = gazebo::physics::get_world();
  ASSERT_NE(nullptr, world);

  // Create ROS clients
  auto node = std::make_shared<rclcpp::Node>("gazebo_ros_factory_test");
  ASSERT_NE(nullptr, node);

  auto model_list_client = node->create_client<gazebo_msgs::srv::GetModelList>("get_model_list");
  ASSERT_NE(nullptr, model_list_client);
  EXPECT_TRUE(model_list_client->wait_for_service(std::chrono::seconds(1)));

  auto spawn_client = node->create_client<gazebo_msgs::srv::SpawnEntity>("spawn_entity");
  ASSERT_NE(nullptr, spawn_client);
  EXPECT_TRUE(spawn_client->wait_for_service(std::chrono::seconds(1)));

  auto delete_client = node->create_client<gazebo_msgs::srv::DeleteEntity>("delete_entity");
  ASSERT_NE(nullptr, delete_client);
  EXPECT_TRUE(delete_client->wait_for_service(std::chrono::seconds(1)));

  // Get Model List (Spawn two sdf models and check result)
  {
    // Model 1
    // Check it has no box1 model
    EXPECT_EQ(nullptr, world->ModelByName("sdf_box1"));

    // Request spawn box1
    auto request1 = std::make_shared<gazebo_msgs::srv::SpawnEntity::Request>();
    request1->name = "sdf_box1";
    request1->initial_pose.position.x = -1.0;
    request1->initial_pose.position.y = -1.0;
    request1->initial_pose.position.z = 0.0;
    request1->xml =
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

    auto response_future1 = spawn_client->async_send_request(request1);
    EXPECT_EQ(
      rclcpp::FutureReturnCode::SUCCESS,
      rclcpp::spin_until_future_complete(node, response_future1));

    auto response1 = response_future1.get();
    ASSERT_NE(nullptr, response1);
    EXPECT_TRUE(response1->success);

    // Model 2
    // Check it has no box2 model
    EXPECT_EQ(nullptr, world->ModelByName("sdf_box2"));

    // Request spawn box2
    auto request2 = std::make_shared<gazebo_msgs::srv::SpawnEntity::Request>();
    request2->name = "sdf_box2";
    request2->initial_pose.position.x = 1.5;
    request2->initial_pose.position.y = 1.5;
    request2->initial_pose.position.z = 0.0;
    request2->xml =
      "<?xml version='1.0' ?>"
      "<sdf version='1.5'>"
      "<model name='ignored'>"
      "<static>true</static>"
      "<link name='link'>"
      "<visual name='visual'>"
      "<geometry>"
      "<sphere><radius>2.0</radius></sphere>"
      "</geometry>"
      "</visual>"
      "</link>"
      "</model>"
      "</sdf>";

    auto response_future2 = spawn_client->async_send_request(request2);
    EXPECT_EQ(
      rclcpp::FutureReturnCode::SUCCESS,
      rclcpp::spin_until_future_complete(node, response_future2));

    auto response2 = response_future2.get();
    ASSERT_NE(nullptr, response2);
    EXPECT_TRUE(response2->success);

    // Check GetModelList
    auto request3 = std::make_shared<gazebo_msgs::srv::GetModelList::Request>();

    auto response_future3 = model_list_client->async_send_request(request3);
    EXPECT_EQ(
      rclcpp::FutureReturnCode::SUCCESS,
      rclcpp::spin_until_future_complete(node, response_future3));

    auto response3 = response_future3.get();
    ASSERT_NE(nullptr, response3);
    EXPECT_TRUE(response3->success);

    EXPECT_EQ(response3->model_names[0], "ground_plane");
    EXPECT_EQ(response3->model_names[1], "sdf_box1");
    EXPECT_EQ(response3->model_names[2], "sdf_box2");
  }

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
    EXPECT_EQ(
      rclcpp::FutureReturnCode::SUCCESS,
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
      "<gazebo>"
      "  <plugin filename='fake_plugin.so' name='fake'>"
      "  </plugin>"
      "</gazebo>"
      "</robot>";

    auto response_future = spawn_client->async_send_request(request);
    EXPECT_EQ(
      rclcpp::FutureReturnCode::SUCCESS,
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
    EXPECT_EQ(
      world->ModelByName("urdf_box")->WorldPose(),
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
    EXPECT_EQ(
      rclcpp::FutureReturnCode::SUCCESS,
      rclcpp::spin_until_future_complete(node, response_future));

    auto response = response_future.get();
    ASSERT_NE(nullptr, response);
    EXPECT_TRUE(response->success);

    // Check it was spawned with the correct name
    ASSERT_NE(nullptr, world->LightByName("dir"));
    EXPECT_EQ(
      world->LightByName("dir")->WorldPose(),
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
    EXPECT_EQ(
      rclcpp::FutureReturnCode::SUCCESS,
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
    EXPECT_EQ(
      rclcpp::FutureReturnCode::SUCCESS,
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
