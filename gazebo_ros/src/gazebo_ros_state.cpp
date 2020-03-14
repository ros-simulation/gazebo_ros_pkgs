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

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Entity.hh>
#include <gazebo/physics/Light.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo_msgs/msg/link_states.hpp>
#include <gazebo_msgs/msg/model_states.hpp>
#include <gazebo_msgs/srv/get_entity_state.hpp>
#include <gazebo_msgs/srv/set_entity_state.hpp>
#include <gazebo_ros/node.hpp>

#include <memory>

#include "gazebo_ros/conversions/builtin_interfaces.hpp"
#include "gazebo_ros/conversions/geometry_msgs.hpp"
#include "gazebo_ros/gazebo_ros_state.hpp"

namespace gazebo_ros
{

class GazeboRosStatePrivate
{
public:
  /// Callback when world is updated.
  /// \param[in] _info Updated simulation info.
  void OnUpdate(const gazebo::common::UpdateInfo & _info);

  /// \brief Callback for get entity state service.
  /// \param[in] req Request
  /// \param[out] res Response
  void GetEntityState(
    gazebo_msgs::srv::GetEntityState::Request::SharedPtr _req,
    gazebo_msgs::srv::GetEntityState::Response::SharedPtr _res);

  /// \brief Callback for set entity state service.
  /// \param[in] req Request
  /// \param[out] res Response
  void SetEntityState(
    gazebo_msgs::srv::SetEntityState::Request::SharedPtr _req,
    gazebo_msgs::srv::SetEntityState::Response::SharedPtr _res);

  /// \brief World pointer from Gazebo.
  gazebo::physics::WorldPtr world_;

  /// ROS node for communication, managed by gazebo_ros.
  gazebo_ros::Node::SharedPtr ros_node_;

  /// \brief ROS service to handle requests for entity states.
  rclcpp::Service<gazebo_msgs::srv::GetEntityState>::SharedPtr get_entity_state_service_;

  /// \brief ROS service to handle requests to set entity states.
  rclcpp::Service<gazebo_msgs::srv::SetEntityState>::SharedPtr set_entity_state_service_;

  /// \brief ROS publisher to publish model_states.
  rclcpp::Publisher<gazebo_msgs::msg::ModelStates>::SharedPtr model_states_pub_;

  /// \brief ROS publisher to publish link_states.
  rclcpp::Publisher<gazebo_msgs::msg::LinkStates>::SharedPtr link_states_pub_;

  /// \brief Connection to world update event, called at every iteration
  gazebo::event::ConnectionPtr world_update_event_;

  /// \brief Update period in seconds.
  double update_period_;

  /// \brief Last update time.
  gazebo::common::Time last_update_time_;
};

GazeboRosState::GazeboRosState()
: impl_(std::make_unique<GazeboRosStatePrivate>())
{
}

GazeboRosState::~GazeboRosState()
{
}

void GazeboRosState::Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
  impl_->world_ = _world;

  impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);

  impl_->get_entity_state_service_ =
    impl_->ros_node_->create_service<gazebo_msgs::srv::GetEntityState>(
    "get_entity_state", std::bind(
      &GazeboRosStatePrivate::GetEntityState, impl_.get(),
      std::placeholders::_1, std::placeholders::_2));

  impl_->set_entity_state_service_ =
    impl_->ros_node_->create_service<gazebo_msgs::srv::SetEntityState>(
    "set_entity_state", std::bind(
      &GazeboRosStatePrivate::SetEntityState, impl_.get(),
      std::placeholders::_1, std::placeholders::_2));

  impl_->model_states_pub_ = impl_->ros_node_->create_publisher<gazebo_msgs::msg::ModelStates>(
    "model_states", rclcpp::QoS(rclcpp::KeepLast(1)));

  RCLCPP_INFO(
    impl_->ros_node_->get_logger(), "Publishing states of gazebo models at [%s]",
    impl_->model_states_pub_->get_topic_name());

  impl_->link_states_pub_ = impl_->ros_node_->create_publisher<gazebo_msgs::msg::LinkStates>(
    "link_states", rclcpp::QoS(rclcpp::KeepLast(1)));

  RCLCPP_INFO(
    impl_->ros_node_->get_logger(), "Publishing states of gazebo links at [%s]",
    impl_->link_states_pub_->get_topic_name());

  // Get a callback when world is updated
  impl_->world_update_event_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&GazeboRosStatePrivate::OnUpdate, impl_.get(), std::placeholders::_1));

  // Update rate
  auto update_rate = _sdf->Get<double>("update_rate", 100.0).first;
  if (update_rate > 0.0) {
    impl_->update_period_ = 1.0 / update_rate;
  } else {
    impl_->update_period_ = 0.0;
  }
  impl_->last_update_time_ = _world->SimTime();
}

void GazeboRosStatePrivate::OnUpdate(const gazebo::common::UpdateInfo & _info)
{
  double seconds_since_last_update = (_info.simTime - last_update_time_).Double();

  if (seconds_since_last_update < update_period_) {
    return;
  }

  gazebo_msgs::msg::ModelStates model_states;
  gazebo_msgs::msg::LinkStates link_states;

  // fill model_states
  for (const auto & model : world_->Models()) {
    auto pose = gazebo_ros::Convert<geometry_msgs::msg::Pose>(model->WorldPose());

    model_states.pose.push_back(pose);
    model_states.name.push_back(model->GetName());

    geometry_msgs::msg::Twist twist;
    twist.linear = gazebo_ros::Convert<geometry_msgs::msg::Vector3>(model->WorldLinearVel());
    twist.angular = gazebo_ros::Convert<geometry_msgs::msg::Vector3>(model->WorldAngularVel());
    model_states.twist.push_back(twist);

    for (unsigned int j = 0; j < model->GetChildCount(); ++j) {
      auto link = boost::dynamic_pointer_cast<gazebo::physics::Link>(model->GetChild(j));

      if (link) {
        link_states.name.push_back(link->GetScopedName());

        pose = gazebo_ros::Convert<geometry_msgs::msg::Pose>(link->WorldPose());
        link_states.pose.push_back(pose);
        twist.linear = gazebo_ros::Convert<geometry_msgs::msg::Vector3>(link->WorldLinearVel());
        twist.angular = gazebo_ros::Convert<geometry_msgs::msg::Vector3>(link->WorldAngularVel());
        link_states.twist.push_back(twist);
      }
    }
  }
  model_states_pub_->publish(model_states);
  link_states_pub_->publish(link_states);

  last_update_time_ = _info.simTime;
}

void GazeboRosStatePrivate::GetEntityState(
  gazebo_msgs::srv::GetEntityState::Request::SharedPtr _req,
  gazebo_msgs::srv::GetEntityState::Response::SharedPtr _res)
{
  // Target entity
  auto entity = world_->EntityByName(_req->name);
  if (!entity) {
    _res->success = false;

    RCLCPP_ERROR(
      ros_node_->get_logger(),
      "GetEntityState: entity [%s] does not exist", _req->name.c_str());
    return;
  }

  auto entity_pose = entity->WorldPose();
  auto entity_lin_vel = entity->WorldLinearVel();
  auto entity_ang_vel = entity->WorldAngularVel();

  // Frame of reference
  auto frame = world_->EntityByName(_req->reference_frame);
  if (frame) {
    auto frame_pose = frame->WorldPose();
    auto frame_lin_vel = frame->WorldLinearVel();
    auto frame_ang_vel = frame->WorldAngularVel();

    entity_pose = entity_pose - frame_pose;

    // convert to relative
    entity_lin_vel = frame_pose.Rot().RotateVectorReverse(entity_lin_vel - frame_lin_vel);
    entity_ang_vel = frame_pose.Rot().RotateVectorReverse(entity_ang_vel - frame_ang_vel);
  } else if (_req->reference_frame == "" || _req->reference_frame == "world") {
    RCLCPP_DEBUG(
      ros_node_->get_logger(),
      "GetEntityState: reference_frame is empty/world, using inertial frame");
  } else {
    _res->success = false;

    RCLCPP_ERROR(
      ros_node_->get_logger(),
      "GetEntityState: reference entity [%s] not found, did you forget to scope the entity name?",
      _req->name.c_str());
    return;
  }

  // Fill in response
  _res->header.stamp = Convert<builtin_interfaces::msg::Time>(world_->SimTime());
  _res->header.frame_id = _req->reference_frame;

  _res->state.pose.position = Convert<geometry_msgs::msg::Point>(entity_pose.Pos());
  _res->state.pose.orientation = Convert<geometry_msgs::msg::Quaternion>(entity_pose.Rot());

  _res->state.twist.linear = Convert<geometry_msgs::msg::Vector3>(entity_lin_vel);
  _res->state.twist.angular = Convert<geometry_msgs::msg::Vector3>(entity_ang_vel);

  _res->success = true;
}

void GazeboRosStatePrivate::SetEntityState(
  gazebo_msgs::srv::SetEntityState::Request::SharedPtr _req,
  gazebo_msgs::srv::SetEntityState::Response::SharedPtr _res)
{
  auto entity_pos = Convert<ignition::math::Vector3d>(_req->state.pose.position);
  auto entity_rot = Convert<ignition::math::Quaterniond>(_req->state.pose.orientation);

  // Eliminate invalid rotation (0, 0, 0, 0)
  entity_rot.Normalize();

  ignition::math::Pose3d entity_pose(entity_pos, entity_rot);
  auto entity_lin_vel = Convert<ignition::math::Vector3d>(_req->state.twist.linear);
  auto entity_ang_vel = Convert<ignition::math::Vector3d>(_req->state.twist.angular);

  // Target entity
  auto entity = world_->EntityByName(_req->state.name);
  if (!entity) {
    _res->success = false;

    RCLCPP_ERROR(
      ros_node_->get_logger(),
      "SetEntityState: entity [%s] does not exist", _req->state.name.c_str());
    return;
  }

  // Frame of reference
  auto frame = world_->EntityByName(_req->state.reference_frame);
  if (frame) {
    auto frame_pose = frame->WorldPose();
    entity_pose = entity_pose + frame_pose;

    // Velocities should be commanded in the requested reference
    // frame, so we need to translate them to the world frame
    entity_lin_vel = frame_pose.Rot().RotateVector(entity_lin_vel);
    entity_ang_vel = frame_pose.Rot().RotateVector(entity_ang_vel);
  } else if (_req->state.reference_frame == "" || _req->state.reference_frame == "world") {
    RCLCPP_DEBUG(
      ros_node_->get_logger(),
      "SetEntityState: reference_frame is empty/world, using inertial frame");
  } else {
    _res->success = false;

    RCLCPP_ERROR(
      ros_node_->get_logger(),
      "GetEntityState: reference entity [%s] not found, did you forget to scope the entity name?",
      _req->state.name.c_str());
    return;
  }

  // Set state
  auto model = boost::dynamic_pointer_cast<gazebo::physics::Model>(entity);
  auto link = boost::dynamic_pointer_cast<gazebo::physics::Link>(entity);
  auto light = boost::dynamic_pointer_cast<gazebo::physics::Light>(entity);

  bool is_paused = world_->IsPaused();

  world_->SetPaused(true);
  entity->SetWorldPose(entity_pose);
  world_->SetPaused(is_paused);

  if (model) {
    model->SetLinearVel(entity_lin_vel);
    model->SetAngularVel(entity_ang_vel);
  } else if (link) {
    link->SetLinearVel(entity_lin_vel);
    link->SetAngularVel(entity_ang_vel);
  }

  // Fill response
  _res->success = true;
}

GZ_REGISTER_WORLD_PLUGIN(GazeboRosState)

}  // namespace gazebo_ros
