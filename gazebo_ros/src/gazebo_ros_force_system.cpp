// Copyright 2019 Open Source Robotics Foundation, Inc.
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

#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/PhysicsIface.hh>
#include <gazebo/physics/World.hh>
#include <gazebo_msgs/srv/apply_link_wrench.hpp>
#include <gazebo_msgs/srv/apply_joint_effort.hpp>
#include <gazebo_msgs/srv/joint_request.hpp>
#include <gazebo_msgs/srv/link_request.hpp>
#include <gazebo_ros/node.hpp>

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "gazebo_ros/conversions/builtin_interfaces.hpp"
#include "gazebo_ros/gazebo_ros_force_system.hpp"

namespace gazebo_ros
{
class GazeboRosForceSystemPrivate
{
public:
  /// Apply link wrench task
  struct LinkWrenchTask
  {
    gazebo::physics::LinkPtr link;
    ignition::math::Vector3d force;
    ignition::math::Vector3d torque;
    rclcpp::Time start_time;
    rclcpp::Duration duration;

    LinkWrenchTask()
    : duration(0, 0) {}
  };

  /// Apply joint effort task
  struct JointEffortTask
  {
    gazebo::physics::JointPtr joint;
    double force;
    rclcpp::Time start_time;
    rclcpp::Duration duration;

    JointEffortTask()
    : duration(0, 0) {}
  };

  /// Callback when a world is created.
  /// \param[in] _world_name The world's name
  void OnWorldCreated(const std::string & _world_name);

  /// \brief Callback for apply joint effort service.
  /// \param[in] req Request
  /// \param[out] res Response
  void ApplyJointEffort(
    gazebo_msgs::srv::ApplyJointEffort::Request::SharedPtr _req,
    gazebo_msgs::srv::ApplyJointEffort::Response::SharedPtr _res);

  /// \brief Callback for apply link wrench service.
  /// \param[in] req Request
  /// \param[out] res Response
  void ApplyLinkWrench(
    gazebo_msgs::srv::ApplyLinkWrench::Request::SharedPtr _req,
    gazebo_msgs::srv::ApplyLinkWrench::Response::SharedPtr _res);

  /// \brief Callback for clear link wrenches service.
  /// \param[in] req Request
  /// \param[out] res Response
  void ClearLinkWrenches(
    gazebo_msgs::srv::LinkRequest::Request::SharedPtr _req,
    gazebo_msgs::srv::LinkRequest::Response::SharedPtr _res);

  /// \brief Callback for clear joint efforts service.
  /// \param[in] req Request
  /// \param[out] res Response
  void ClearJointEfforts(
    gazebo_msgs::srv::JointRequest::Request::SharedPtr _req,
    gazebo_msgs::srv::JointRequest::Response::SharedPtr _res);

  /// \bried Helper to shift wrench from reference frame to target frame
  /// \param[in] reference_force Reference Frame Force
  /// \param[in] reference_torque Reference Frame Torque
  /// \param[in] target_to_reference Reference for transform
  /// \param[out] target_force Shifted Force
  /// \param[out] target_torque Shifted Torque
  void TransformWrench(
    ignition::math::Vector3d & target_force, ignition::math::Vector3d & target_torque,
    const ignition::math::Vector3d & reference_force,
    const ignition::math::Vector3d & reference_torque,
    const ignition::math::Pose3d & target_to_reference
  );

  /// Execute link wrench and joint effort tasks.
  /// \param[in] _info World update information.
  void TaskExecutor(const gazebo::common::UpdateInfo & _info);

  /// \brief World pointer from Gazebo.
  gazebo::physics::WorldPtr world_;

  /// ROS node for communication, managed by gazebo_ros.
  gazebo_ros::Node::SharedPtr ros_node_;

  /// \brief ROS service to handle requests for apply link wrench.
  rclcpp::Service<gazebo_msgs::srv::ApplyLinkWrench>::SharedPtr apply_link_wrench_service_;

  /// \brief ROS service to handle requests for clear link wrenches.
  rclcpp::Service<gazebo_msgs::srv::LinkRequest>::SharedPtr clear_link_wrenches_service_;

  /// \brief ROS service to handle requests for apply joint effort.
  rclcpp::Service<gazebo_msgs::srv::ApplyJointEffort>::SharedPtr apply_joint_effort_service_;

  /// \brief ROS service to handle requests for clear joint forces.
  rclcpp::Service<gazebo_msgs::srv::JointRequest>::SharedPtr clear_joint_efforts_service_;

  /// To be notified once the world is created.
  gazebo::event::ConnectionPtr world_created_connection_;

  /// Connection to event called at every world iteration.
  gazebo::event::ConnectionPtr update_connection_;

  /// Pointer to apply wrench tasks
  std::vector<std::shared_ptr<LinkWrenchTask>> link_wrench_tasks_;

  /// Pointer to joint effort tasks
  std::vector<std::shared_ptr<JointEffortTask>> joint_effort_tasks_;

  /// Lock for executing tasks
  std::mutex lock_;
};

GazeboRosForceSystem::GazeboRosForceSystem()
: impl_(std::make_unique<GazeboRosForceSystemPrivate>())
{
}

GazeboRosForceSystem::~GazeboRosForceSystem()
{
}

void GazeboRosForceSystem::Load(int /* argc */, char ** /* argv */)
{
  impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&GazeboRosForceSystemPrivate::TaskExecutor, impl_.get(), std::placeholders::_1));

  impl_->world_created_connection_ = gazebo::event::Events::ConnectWorldCreated(
    std::bind(&GazeboRosForceSystemPrivate::OnWorldCreated, impl_.get(), std::placeholders::_1));
}

void GazeboRosForceSystemPrivate::OnWorldCreated(const std::string & _world_name)
{
  // Only support one world
  world_created_connection_.reset();

  world_ = gazebo::physics::get_world(_world_name);

  // ROS transport
  ros_node_ = gazebo_ros::Node::Get();

  apply_link_wrench_service_ = ros_node_->create_service<gazebo_msgs::srv::ApplyLinkWrench>(
    "apply_link_wrench", std::bind(
      &GazeboRosForceSystemPrivate::ApplyLinkWrench, this,
      std::placeholders::_1, std::placeholders::_2));

  clear_link_wrenches_service_ = ros_node_->create_service<gazebo_msgs::srv::LinkRequest>(
    "clear_link_wrenches", std::bind(
      &GazeboRosForceSystemPrivate::ClearLinkWrenches, this,
      std::placeholders::_1, std::placeholders::_2));

  apply_joint_effort_service_ = ros_node_->create_service<gazebo_msgs::srv::ApplyJointEffort>(
    "apply_joint_effort", std::bind(
      &GazeboRosForceSystemPrivate::ApplyJointEffort, this,
      std::placeholders::_1, std::placeholders::_2));

  clear_joint_efforts_service_ = ros_node_->create_service<gazebo_msgs::srv::JointRequest>(
    "clear_joint_efforts", std::bind(
      &GazeboRosForceSystemPrivate::ClearJointEfforts, this,
      std::placeholders::_1, std::placeholders::_2));
}

void GazeboRosForceSystemPrivate::TaskExecutor(const gazebo::common::UpdateInfo & _info)
{
  // Store sim time
  rclcpp::Time sim_time = Convert<builtin_interfaces::msg::Time>(_info.simTime);

  std::lock_guard<std::mutex> scoped_lock(lock_);

  for (auto link_wrench_task = link_wrench_tasks_.begin();
    link_wrench_task != link_wrench_tasks_.end(); ++link_wrench_task)
  {
    auto start = (*link_wrench_task)->start_time;
    auto duration = (*link_wrench_task)->duration;
    auto end = start + duration;

    if (!(*link_wrench_task)->link) {
      link_wrench_tasks_.erase(link_wrench_task--);
      RCLCPP_ERROR(
        ros_node_->get_logger(),
        "Link [%s] does not exist. Deleting task", (*link_wrench_task)->link->GetName().c_str());
    }

    if (sim_time < start) {
      continue;
    }

    if (sim_time <= end || duration.seconds() < 0.0) {
      (*link_wrench_task)->link->SetForce((*link_wrench_task)->force);
      (*link_wrench_task)->link->SetTorque((*link_wrench_task)->torque);
    } else {
      // remove from queue once expires
      link_wrench_tasks_.erase(link_wrench_task--);
    }
  }

  for (auto joint_effort_task = joint_effort_tasks_.begin();
    joint_effort_task != joint_effort_tasks_.end(); ++joint_effort_task)
  {
    auto start = (*joint_effort_task)->start_time;
    auto duration = (*joint_effort_task)->duration;
    auto end = start + duration;

    if (!(*joint_effort_task)->joint) {
      joint_effort_tasks_.erase(joint_effort_task--);
      RCLCPP_ERROR(
        ros_node_->get_logger(),
        "Joint [%s] does not exist. Deleting task", (*joint_effort_task)->joint->GetName().c_str());
    }

    if (sim_time < start) {
      continue;
    }

    if (sim_time <= end || duration.seconds() < 0.0) {
      (*joint_effort_task)->joint->SetForce(0, (*joint_effort_task)->force);
    } else {
      // remove from queue once expires
      joint_effort_tasks_.erase(joint_effort_task--);
    }
  }
}

void GazeboRosForceSystemPrivate::ApplyLinkWrench(
  gazebo_msgs::srv::ApplyLinkWrench::Request::SharedPtr _req,
  gazebo_msgs::srv::ApplyLinkWrench::Response::SharedPtr _res)
{
  auto link = boost::dynamic_pointer_cast<gazebo::physics::Link>(
    world_->EntityByName(_req->link_name));
  auto frame = world_->EntityByName(_req->reference_frame);

  if (!link) {
    RCLCPP_ERROR(ros_node_->get_logger(), "Link [%s] does not exist", _req->link_name.c_str());
    _res->success = false;
    _res->status_message = "ApplyLinkWrench: link does not exist";
    return;
  }

  // target wrench
  auto ref_frame = _req->reference_frame;
  ignition::math::Vector3d ref_force(
    _req->wrench.force.x, _req->wrench.force.y, _req->wrench.force.z);
  ignition::math::Vector3d ref_torque(
    _req->wrench.torque.x, _req->wrench.torque.y, _req->wrench.torque.z);
  ignition::math::Vector3d ref_point(
    _req->reference_point.x, _req->reference_point.y, _req->reference_point.z);

  ignition::math::Vector3d target_force;
  ignition::math::Vector3d target_torque;

  ref_torque = ref_torque + ref_point.Cross(ref_force);

  if (frame) {
    ignition::math::Pose3d framePose = frame->WorldPose();
    ignition::math::Pose3d linkPose = link->WorldPose();
    ignition::math::Pose3d target_to_ref = framePose - linkPose;

    auto link_pose = linkPose.Pos();
    auto link_rot = linkPose.Rot().Euler();
    auto frame_pose = framePose.Pos();
    auto frame_rot = framePose.Rot().Euler();
    auto target_pose = target_to_ref.Pos();
    auto target_rot = target_to_ref.Rot().Euler();

    RCLCPP_DEBUG(
      ros_node_->get_logger(),
      "Reference frame for applied wrench:"
      "[%f %f %f, %f %f %f] - [%f %f %f, %f %f %f] = [%f %f %f, %f %f %f]",
      link_pose.X(), link_pose.Y(), link_pose.Z(),
      link_rot.X(), link_rot.Y(), link_rot.Z(),
      frame_pose.X(), frame_pose.Y(), frame_pose.Z(),
      frame_rot.X(), frame_rot.Y(), frame_rot.Z(),
      target_pose.X(), target_pose.Y(), target_pose.Z(),
      target_rot.X(), target_rot.Y(), target_rot.Z()
    );

    TransformWrench(target_force, target_torque, ref_force, ref_torque, target_to_ref);

    RCLCPP_INFO(
      ros_node_->get_logger(),
      "Wrench defined as [%s]:[%f %f %f, %f %f %f] -> Applied as [%s]:[%f %f %f, %f %f %f]",
      frame->GetName().c_str(),
      ref_force.X(), ref_force.Y(), ref_force.Z(),
      ref_torque.X(), ref_torque.Y(), ref_torque.Z(),
      link->GetName().c_str(),
      target_force.X(), target_force.Y(), target_force.Z(),
      target_torque.X(), target_torque.Y(), target_torque.Z());

  } else if (ref_frame.empty() || ref_frame == "world" || ref_frame == "map") {
    ignition::math::Pose3d target_to_reference = link->WorldPose();
    target_force = ref_force;
    target_torque = ref_torque;
    RCLCPP_INFO(
      ros_node_->get_logger(), "Reference_frame is empty/world/map,"
      "using inertial frame, transferring from link relative to inertial frame");

  } else {
    RCLCPP_ERROR(ros_node_->get_logger(), "Reference_frame is not a valid entity name");
    _res->success = false;
    _res->status_message = "Reference_frame not found";
    return;
  }

  auto link_wrench_task = std::make_shared<LinkWrenchTask>();
  link_wrench_task->link = link;
  link_wrench_task->force = target_force;
  link_wrench_task->torque = target_torque;
  link_wrench_task->start_time = _req->start_time;
  link_wrench_task->duration = _req->duration;

  std::lock_guard<std::mutex> scoped_lock(lock_);
  link_wrench_tasks_.push_back(link_wrench_task);
  _res->success = true;
}

void GazeboRosForceSystemPrivate::TransformWrench(
  ignition::math::Vector3d & target_force, ignition::math::Vector3d & target_torque,
  const ignition::math::Vector3d & reference_force,
  const ignition::math::Vector3d & reference_torque,
  const ignition::math::Pose3d & target_to_reference)
{
  // rotate force into target frame
  target_force = target_to_reference.Rot().RotateVector(reference_force);
  // rotate torque into target frame
  target_torque = target_to_reference.Rot().RotateVector(reference_torque);

  // target force is the refence force rotated by the target->reference transform
  target_torque = target_torque + target_to_reference.Pos().Cross(target_force);
}

void GazeboRosForceSystemPrivate::ClearLinkWrenches(
  gazebo_msgs::srv::LinkRequest::Request::SharedPtr _req,
  gazebo_msgs::srv::LinkRequest::Response::SharedPtr /*_res*/)
{
  std::lock_guard<std::mutex> scoped_lock(lock_);
  auto prev_end = link_wrench_tasks_.end();
  link_wrench_tasks_.erase(
    std::remove_if(
      link_wrench_tasks_.begin(), link_wrench_tasks_.end(),
      [_req, this](auto & link_wrench_task) {
        if (link_wrench_task->link->GetScopedName() == _req->link_name) {
          RCLCPP_INFO(ros_node_->get_logger(), "Deleted wrench on [%s]", _req->link_name.c_str());
          return true;
        }
        return false;
      }),
    link_wrench_tasks_.end());
  if (prev_end == link_wrench_tasks_.end()) {
    RCLCPP_WARN(ros_node_->get_logger(), "No applied wrenches on [%s]", _req->link_name.c_str());
  }
}

void GazeboRosForceSystemPrivate::ApplyJointEffort(
  gazebo_msgs::srv::ApplyJointEffort::Request::SharedPtr _req,
  gazebo_msgs::srv::ApplyJointEffort::Response::SharedPtr _res)
{
  gazebo::physics::JointPtr joint;

  for (unsigned int i = 0; i < world_->ModelCount(); ++i) {
    joint = world_->ModelByIndex(i)->GetJoint(_req->joint_name);
    if (joint) {
      auto joint_effort_task = std::make_shared<JointEffortTask>();
      joint_effort_task->joint = joint;
      joint_effort_task->force = _req->effort;
      joint_effort_task->start_time = _req->start_time;
      joint_effort_task->duration = _req->duration;

      std::lock_guard<std::mutex> lock(lock_);
      joint_effort_tasks_.push_back(joint_effort_task);

      _res->success = true;
      _res->status_message = "Effort on joint [" + _req->joint_name + "] set";
      return;
    }
  }
  _res->success = false;
  _res->status_message = "Joint not found";
}

void GazeboRosForceSystemPrivate::ClearJointEfforts(
  gazebo_msgs::srv::JointRequest::Request::SharedPtr _req,
  gazebo_msgs::srv::JointRequest::Response::SharedPtr /*_res*/)
{
  std::lock_guard<std::mutex> scoped_lock(lock_);
  auto prev_end = joint_effort_tasks_.end();
  joint_effort_tasks_.erase(
    std::remove_if(
      joint_effort_tasks_.begin(), joint_effort_tasks_.end(),
      [_req, this](auto & joint_effort_task) {
        if (joint_effort_task->joint->GetName() == _req->joint_name) {
          RCLCPP_INFO(ros_node_->get_logger(), "Deleted effort on [%s]", _req->joint_name.c_str());
          return true;
        }
        return false;
      }),
    joint_effort_tasks_.end());
  if (prev_end == joint_effort_tasks_.end()) {
    RCLCPP_WARN(ros_node_->get_logger(), "No applied efforts on [%s]", _req->joint_name.c_str());
  }
}

GZ_REGISTER_SYSTEM_PLUGIN(GazeboRosForceSystem)

}  // namespace gazebo_ros
