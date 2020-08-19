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
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

#include <gazebo_msgs/msg/link_states.hpp>
#include <gazebo_msgs/msg/model_states.hpp>
#include <gazebo_msgs/srv/get_entity_state.hpp>
#include <gazebo_msgs/srv/set_entity_state.hpp>
#include <gazebo_ros/conversions/geometry_msgs.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>

#define tol 10e-2

using namespace std::literals::chrono_literals; // NOLINT

class GazeboRosStateTest : public gazebo::ServerFixture
{
public:
  // Documentation inherited
  void SetUp() override;

  /// Helper function to call get state service
  void GetState(
    const std::string & _entity,
    const ignition::math::Pose3d & _pose,
    const ignition::math::Vector3d & _lin_vel = ignition::math::Vector3d::Zero,
    const ignition::math::Vector3d & _ang_vel = ignition::math::Vector3d::Zero);

  /// Helper function to call set state service
  void SetState(
    const std::string & _entity,
    const ignition::math::Pose3d & _pose,
    const ignition::math::Vector3d & _lin_vel = ignition::math::Vector3d::Zero,
    const ignition::math::Vector3d & _ang_vel = ignition::math::Vector3d::Zero);

  gazebo::physics::WorldPtr world_;
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<rclcpp::Client<gazebo_msgs::srv::GetEntityState>> get_state_client_;
  std::shared_ptr<rclcpp::Client<gazebo_msgs::srv::SetEntityState>> set_state_client_;
  rclcpp::Subscription<gazebo_msgs::msg::LinkStates>::SharedPtr link_states_sub_;
  rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr model_states_sub_;
};

void GazeboRosStateTest::SetUp()
{
  // Load world with state plugin and start paused
  this->Load("worlds/gazebo_ros_state_test.world", true);

  // World
  world_ = gazebo::physics::get_world();
  ASSERT_NE(nullptr, world_);

  // Create ROS clients
  node_ = std::make_shared<rclcpp::Node>("gazebo_ros_state_test");
  ASSERT_NE(nullptr, node_);

  get_state_client_ =
    node_->create_client<gazebo_msgs::srv::GetEntityState>("test/get_entity_state");
  ASSERT_NE(nullptr, get_state_client_);
  EXPECT_TRUE(get_state_client_->wait_for_service(std::chrono::seconds(1)));

  set_state_client_ =
    node_->create_client<gazebo_msgs::srv::SetEntityState>("test/set_entity_state");
  ASSERT_NE(nullptr, set_state_client_);
  EXPECT_TRUE(set_state_client_->wait_for_service(std::chrono::seconds(1)));
}

void GazeboRosStateTest::GetState(
  const std::string & _entity,
  const ignition::math::Pose3d & _pose,
  const ignition::math::Vector3d & _lin_vel,
  const ignition::math::Vector3d & _ang_vel)
{
  auto entity = world_->EntityByName(_entity);
  ASSERT_NE(nullptr, entity);

  auto request = std::make_shared<gazebo_msgs::srv::GetEntityState::Request>();
  request->name = _entity;

  auto response_future = get_state_client_->async_send_request(request);
  EXPECT_EQ(
    rclcpp::FutureReturnCode::SUCCESS,
    rclcpp::spin_until_future_complete(node_, response_future));

  auto response = response_future.get();
  ASSERT_NE(nullptr, response);
  EXPECT_TRUE(response->success);

  EXPECT_NEAR(_pose.Pos().X(), response->state.pose.position.x, tol) << _entity;
  EXPECT_NEAR(_pose.Pos().Y(), response->state.pose.position.y, tol) << _entity;
  EXPECT_NEAR(_pose.Pos().Z(), response->state.pose.position.z, tol) << _entity;

  EXPECT_NEAR(_pose.Rot().X(), response->state.pose.orientation.x, tol) << _entity;
  EXPECT_NEAR(_pose.Rot().Y(), response->state.pose.orientation.y, tol) << _entity;
  EXPECT_NEAR(_pose.Rot().Z(), response->state.pose.orientation.z, tol) << _entity;
  EXPECT_NEAR(_pose.Rot().W(), response->state.pose.orientation.w, tol) << _entity;

  EXPECT_NEAR(_lin_vel.X(), response->state.twist.linear.x, tol) << _entity;
  EXPECT_NEAR(_lin_vel.Y(), response->state.twist.linear.y, tol) << _entity;
  EXPECT_NEAR(_lin_vel.Z(), response->state.twist.linear.z, tol) << _entity;

  EXPECT_NEAR(_ang_vel.X(), response->state.twist.angular.x, tol) << _entity;
  EXPECT_NEAR(_ang_vel.Y(), response->state.twist.angular.y, tol) << _entity;
  EXPECT_NEAR(_ang_vel.Z(), response->state.twist.angular.z, tol) << _entity;
}

void GazeboRosStateTest::SetState(
  const std::string & _entity,
  const ignition::math::Pose3d & _pose,
  const ignition::math::Vector3d & _lin_vel,
  const ignition::math::Vector3d & _ang_vel)
{
  auto request = std::make_shared<gazebo_msgs::srv::SetEntityState::Request>();
  request->state.name = _entity;
  request->state.pose.position = gazebo_ros::Convert<geometry_msgs::msg::Point>(_pose.Pos());
  request->state.pose.orientation =
    gazebo_ros::Convert<geometry_msgs::msg::Quaternion>(_pose.Rot());
  request->state.twist.linear = gazebo_ros::Convert<geometry_msgs::msg::Vector3>(_lin_vel);
  request->state.twist.angular = gazebo_ros::Convert<geometry_msgs::msg::Vector3>(_ang_vel);

  auto response_future = set_state_client_->async_send_request(request);
  EXPECT_EQ(
    rclcpp::FutureReturnCode::SUCCESS,
    rclcpp::spin_until_future_complete(node_, response_future));

  auto response = response_future.get();
  ASSERT_NE(nullptr, response);
  EXPECT_TRUE(response->success);
}

TEST_F(GazeboRosStateTest, GetSet)
{
  // Get / set model state
  {
    // Get initial state
    this->GetState("boxes", ignition::math::Pose3d(0, 0, 0.5, 0, 0, 0));

    // Set new state
    this->SetState(
      "boxes", ignition::math::Pose3d(1.0, 2.0, 10.0, 0, 0, 0),
      ignition::math::Vector3d(4.0, 0, 0), ignition::math::Vector3d::Zero);

    // Check new state
    this->GetState(
      "boxes", ignition::math::Pose3d(1.0, 2.0, 10.0, 0, 0, 0),
      ignition::math::Vector3d(4.0, 0, 0), ignition::math::Vector3d::Zero);
  }

  // Get / set light state
  {
    // Get initial state
    this->GetState("sun", ignition::math::Pose3d(0, 0, 10, 0, 0, 0));

    // Set new state
    this->SetState("sun", ignition::math::Pose3d(1.0, 2.0, 3.0, 0.1, 0.2, 0.3));

    // Check new state
    this->GetState("sun", ignition::math::Pose3d(1.0, 2.0, 3.0, 0.1, 0.2, 0.3));
  }

  // Get / set link state
  {
    // Get initial state - note that is was moved with the model
    this->GetState(
      "boxes::top", ignition::math::Pose3d(1.0, 2.0, 11.25, 0, 0, 0),
      ignition::math::Vector3d(4.0, 0, 0), ignition::math::Vector3d::Zero);

    // Set new state
    this->SetState(
      "boxes::top", ignition::math::Pose3d(10, 20, 30, 0.1, 0, 0),
      ignition::math::Vector3d(1.0, 2.0, 3.0), ignition::math::Vector3d(0.0, 0.0, 4.0));

    // Check new state
    this->GetState(
      "boxes::top", ignition::math::Pose3d(10, 20, 30, 0.1, 0, 0),
      ignition::math::Vector3d(1.0, 2.0, 3.0), ignition::math::Vector3d(0.0, 0.0, 4.0));
  }

  // Model states
  {
    // Set new state
    this->SetState("boxes", ignition::math::Pose3d(1, 2, 0.5, 0, 0, 0));

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node_);
    gazebo_msgs::msg::ModelStates::SharedPtr model_states_msg{nullptr};
    model_states_sub_ = node_->create_subscription<gazebo_msgs::msg::ModelStates>(
      "test/model_states_test", rclcpp::SystemDefaultsQoS(),
      [&model_states_msg](gazebo_msgs::msg::ModelStates::SharedPtr _msg) {
        model_states_msg = _msg;
      });

    // Wait for a message
    world_->Step(1000);

    // Wait for it to be processed
    int sleep{0};
    int maxSleep{300};

    for (; sleep < maxSleep && !model_states_msg; ++sleep) {
      executor.spin_once(100ms);
      gazebo::common::Time::MSleep(100);
    }
    EXPECT_NE(sleep, maxSleep);
    EXPECT_NE(model_states_msg, nullptr);
    EXPECT_NEAR(model_states_msg->pose[1].position.x, 1.0, tol);
    EXPECT_NEAR(model_states_msg->pose[1].position.y, 2.0, tol);
    EXPECT_NEAR(model_states_msg->pose[1].position.z, 0.5, tol);
    EXPECT_NEAR(model_states_msg->pose[1].orientation.x, 0.0, tol);
    EXPECT_NEAR(model_states_msg->pose[1].orientation.y, 0.0, tol);
    EXPECT_NEAR(model_states_msg->pose[1].orientation.z, 0.0, tol);
    EXPECT_NEAR(model_states_msg->pose[1].orientation.w, 1.0, tol);
  }

  // Link states
  {
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node_);
    gazebo_msgs::msg::LinkStates::SharedPtr link_states_msg{nullptr};
    link_states_sub_ = node_->create_subscription<gazebo_msgs::msg::LinkStates>(
      "test/link_states_test", rclcpp::SystemDefaultsQoS(),
      [&link_states_msg](gazebo_msgs::msg::LinkStates::SharedPtr _msg) {
        link_states_msg = _msg;
      });

    // Wait for a message
    world_->Step(1000);

    // Wait for it to be processed
    int sleep{0};
    int maxSleep{300};

    for (; sleep < maxSleep && !link_states_msg; ++sleep) {
      executor.spin_once(100ms);
      gazebo::common::Time::MSleep(100);
    }
    EXPECT_NE(sleep, maxSleep);
    EXPECT_NE(link_states_msg, nullptr);
    EXPECT_NEAR(link_states_msg->pose[1].position.x, 1.0, tol);
    EXPECT_NEAR(link_states_msg->pose[1].position.y, 2.0, tol);
    EXPECT_NEAR(link_states_msg->pose[1].position.z, 0.5, tol);
    EXPECT_NEAR(link_states_msg->pose[1].orientation.x, 0.0, tol);
    EXPECT_NEAR(link_states_msg->pose[1].orientation.y, 0.0, tol);
    EXPECT_NEAR(link_states_msg->pose[1].orientation.z, 0.0, tol);
    EXPECT_NEAR(link_states_msg->pose[1].orientation.w, 1.0, tol);
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
