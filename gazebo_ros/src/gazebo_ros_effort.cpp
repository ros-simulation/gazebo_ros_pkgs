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
#include <gazebo_msgs/srv/apply_body_wrench.hpp>
#include <gazebo_msgs/srv/body_request.hpp>
#include <gazebo_msgs/srv/apply_joint_effort.hpp>
#include <gazebo_msgs/srv/joint_request.hpp>
#include <gazebo_ros/node.hpp>

#include <memory>
#include <string>
#include <vector>

#include "gazebo_ros/conversions/builtin_interfaces.hpp"
#include "gazebo_ros/gazebo_ros_effort.hpp"

namespace gazebo_ros
{
class GazeboRosEffortPrivate
{
public:
  /// Apply body wrench task
  struct BodyWrenchTask
  {
    gazebo::physics::LinkPtr link;
    ignition::math::Vector3d force;
    ignition::math::Vector3d torque;
    rclcpp::Time start_time;
    rclcpp::Duration duration;

    BodyWrenchTask()
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

  /// \brief Callback for apply body wrench service.
  /// \param[in] req Request
  /// \param[out] res Response
  void ApplyBodyWrench(
    gazebo_msgs::srv::ApplyBodyWrench::Request::SharedPtr _req,
    gazebo_msgs::srv::ApplyBodyWrench::Response::SharedPtr _res);

  /// \brief Callback for clear body wrenches service.
  /// \param[in] req Request
  /// \param[out] res Response
  void ClearBodyWrenches(
    gazebo_msgs::srv::BodyRequest::Request::SharedPtr _req,
    gazebo_msgs::srv::BodyRequest::Response::SharedPtr _res);

  /// \brief Callback for clear joint forces service.
  /// \param[in] req Request
  /// \param[out] res Response
  void ClearJointForces(
    gazebo_msgs::srv::JointRequest::Request::SharedPtr _req,
    gazebo_msgs::srv::JointRequest::Response::SharedPtr _res);

  /// \brief Callback for apply joint effort service.
  /// \param[in] req Request
  /// \param[out] res Response
  void ApplyJointEffort(
    gazebo_msgs::srv::ApplyJointEffort::Request::SharedPtr _req,
    gazebo_msgs::srv::ApplyJointEffort::Response::SharedPtr _res);

  /// \bried Helper to shift wrench from reference frame to target frame
  /// \param[in] reference_force Reference Frame Force
  /// \param[in] reference_torque Reference Frame Torque
  /// \param[in] target_to_reference Reference for transform
  /// \param[out] target_force Shifted Force
  /// \param[out] target_torque Shifted Torque
  void transformWrench(
    ignition::math::Vector3d & target_force, ignition::math::Vector3d & target_torque,
    const ignition::math::Vector3d & reference_force,
    const ignition::math::Vector3d & reference_torque,
    const ignition::math::Pose3d & target_to_reference
  );

  /// Execute body wrench tasks.
  /// \param[in] _info World update information.
  void BodyWrenchTaskExecutor(const gazebo::common::UpdateInfo & _info);

  /// Execute joint effort tasks.
  /// \param[in] _info World update information.
  void JointEffortTaskExecutor(const gazebo::common::UpdateInfo & _info);

  /// \brief World pointer from Gazebo.
  gazebo::physics::WorldPtr world_;

  /// ROS node for communication, managed by gazebo_ros.
  gazebo_ros::Node::SharedPtr ros_node_;

  /// Reusable factory SDF for efficiency.
  sdf::SDFPtr factory_sdf_ = std::make_shared<sdf::SDF>();

  /// \brief ROS service to handle requests for apply body wrench.
  rclcpp::Service<gazebo_msgs::srv::ApplyBodyWrench>::SharedPtr apply_body_wrench_service_;

  /// \brief ROS service to handle requests for clear body wrenches.
  rclcpp::Service<gazebo_msgs::srv::BodyRequest>::SharedPtr clear_body_wrenches_service_;

  /// \brief ROS service to handle requests for apply joint effort.
  rclcpp::Service<gazebo_msgs::srv::ApplyJointEffort>::SharedPtr apply_joint_effort_service_;

  /// \brief ROS service to handle requests for clear joint forces.
  rclcpp::Service<gazebo_msgs::srv::JointRequest>::SharedPtr clear_joint_forces_service_;

  /// To be notified once the world is created.
  gazebo::event::ConnectionPtr world_created_connection_;

  /// Connection to body wrench update event, called at every iteration
  gazebo::event::ConnectionPtr body_wrench_update_event_;

  /// Connection to joint effort update event, called at every iteration
  gazebo::event::ConnectionPtr joint_effort_update_event_;

  /// Pointer to apply wrench tasks
  std::vector<std::shared_ptr<BodyWrenchTask>> body_wrench_tasks_;

  /// Pointer to joint effort tasks
  std::vector<std::shared_ptr<JointEffortTask>> joint_effort_tasks_;

  /// Lock for body wrench tasks
  std::mutex wrench_lock_;

  /// Lock for joint effort tasks
  std::mutex effort_lock_;
};

GazeboRosEffort::GazeboRosEffort()
: impl_(std::make_unique<GazeboRosEffortPrivate>())
{
}

GazeboRosEffort::~GazeboRosEffort()
{
}

void GazeboRosEffort::Load(int /* argc */, char ** /* argv */)
{
  // Keep this in the constructor for performance.
  // sdf::initFile causes disk access.
  sdf::initFile("root.sdf", impl_->factory_sdf_);

  impl_->body_wrench_update_event_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&GazeboRosEffortPrivate::BodyWrenchTaskExecutor, impl_.get(), std::placeholders::_1));

  impl_->joint_effort_update_event_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&GazeboRosEffortPrivate::JointEffortTaskExecutor, impl_.get(),
    std::placeholders::_1));

  impl_->world_created_connection_ = gazebo::event::Events::ConnectWorldCreated(
    std::bind(&GazeboRosEffortPrivate::OnWorldCreated, impl_.get(), std::placeholders::_1));
}

void GazeboRosEffortPrivate::OnWorldCreated(const std::string & _world_name)
{
  // Only support one world
  world_created_connection_.reset();

  world_ = gazebo::physics::get_world(_world_name);

  // ROS transport
  ros_node_ = gazebo_ros::Node::Get();

  apply_body_wrench_service_ = ros_node_->create_service<gazebo_msgs::srv::ApplyBodyWrench>(
    "apply_body_wrench", std::bind(&GazeboRosEffortPrivate::ApplyBodyWrench, this,
    std::placeholders::_1, std::placeholders::_2));

  clear_body_wrenches_service_ = ros_node_->create_service<gazebo_msgs::srv::BodyRequest>(
    "clear_body_wrenches", std::bind(&GazeboRosEffortPrivate::ClearBodyWrenches, this,
    std::placeholders::_1, std::placeholders::_2));

  apply_joint_effort_service_ = ros_node_->create_service<gazebo_msgs::srv::ApplyJointEffort>(
    "apply_joint_effort", std::bind(&GazeboRosEffortPrivate::ApplyJointEffort, this,
    std::placeholders::_1, std::placeholders::_2));

  clear_joint_forces_service_ = ros_node_->create_service<gazebo_msgs::srv::JointRequest>(
    "clear_joint_forces", std::bind(&GazeboRosEffortPrivate::ClearJointForces, this,
    std::placeholders::_1, std::placeholders::_2));
}

void GazeboRosEffortPrivate::BodyWrenchTaskExecutor(const gazebo::common::UpdateInfo & _info)
{
  std::lock_guard<std::mutex> scoped_lock(wrench_lock_);

  for (auto body_wrench_task = body_wrench_tasks_.begin();
    body_wrench_task != body_wrench_tasks_.end(); ++body_wrench_task)
  {
    // check times and apply wrench if necessary
    rclcpp::Time sim_time = Convert<builtin_interfaces::msg::Time>(_info.simTime);

    auto start = (*body_wrench_task)->start_time;
    auto duration = (*body_wrench_task)->duration;
    auto end = start + duration;

    if (!(*body_wrench_task)->link) {
      body_wrench_tasks_.erase(body_wrench_task--);
      RCLCPP_ERROR(ros_node_->get_logger(),
        "Link [%s] does not exist. Deleting task", (*body_wrench_task)->link->GetName().c_str());
    }

    if (sim_time < start) {
      continue;
    }

    if (sim_time <= end || duration.seconds() < 0.0) {
      (*body_wrench_task)->link->SetForce((*body_wrench_task)->force);
      (*body_wrench_task)->link->SetTorque((*body_wrench_task)->torque);
    } else {
      // remove from queue once expires
      body_wrench_tasks_.erase(body_wrench_task--);
    }
  }
}

void GazeboRosEffortPrivate::JointEffortTaskExecutor(const gazebo::common::UpdateInfo & _info)
{
  std::lock_guard<std::mutex> scoped_lock(effort_lock_);

  for (auto joint_effort_task = joint_effort_tasks_.begin();
    joint_effort_task != joint_effort_tasks_.end(); ++joint_effort_task)
  {
    // check times and apply force if necessary
    rclcpp::Time sim_time = Convert<builtin_interfaces::msg::Time>(_info.simTime);

    auto start = (*joint_effort_task)->start_time;
    auto duration = (*joint_effort_task)->duration;
    auto end = start + duration;

    if (!(*joint_effort_task)->joint) {
      joint_effort_tasks_.erase(joint_effort_task--);
      RCLCPP_ERROR(ros_node_->get_logger(),
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

void GazeboRosEffortPrivate::ApplyBodyWrench(
  gazebo_msgs::srv::ApplyBodyWrench::Request::SharedPtr _req,
  gazebo_msgs::srv::ApplyBodyWrench::Response::SharedPtr _res)
{
  auto body = boost::dynamic_pointer_cast<gazebo::physics::Link>(
    world_->EntityByName(_req->body_name));
  auto frame = world_->EntityByName(_req->reference_frame);

  if (!body) {
    RCLCPP_ERROR(ros_node_->get_logger(), "Link [%s] does not exist", _req->body_name.c_str());
    _res->success = false;
    _res->status_message = "ApplyBodyWrench: link does not exist";
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
    ignition::math::Pose3d bodyPose = body->WorldPose();
    ignition::math::Pose3d target_to_ref = framePose - bodyPose;

    auto body_pose = bodyPose.Pos();
    auto body_rot = bodyPose.Rot().Euler();
    auto frame_pose = framePose.Pos();
    auto frame_rot = framePose.Rot().Euler();
    auto target_pose = target_to_ref.Pos();
    auto target_rot = target_to_ref.Rot().Euler();

    RCLCPP_DEBUG(ros_node_->get_logger(),
      "Reference frame for applied wrench:"
      "[%f %f %f, %f %f %f] - [%f %f %f, %f %f %f] = [%f %f %f, %f %f %f]",
      body_pose.X(), body_pose.Y(), body_pose.Z(),
      body_rot.X(), body_rot.Y(), body_rot.Z(),
      frame_pose.X(), frame_pose.Y(), frame_pose.Z(),
      frame_rot.X(), frame_rot.Y(), frame_rot.Z(),
      target_pose.X(), target_pose.Y(), target_pose.Z(),
      target_rot.X(), target_rot.Y(), target_rot.Z()
    );

    transformWrench(target_force, target_torque, ref_force, ref_torque, target_to_ref);

    RCLCPP_INFO(ros_node_->get_logger(),
      "Wrench defined as [%s]:[%f %f %f, %f %f %f] -> Applied as [%s]:[%f %f %f, %f %f %f]",
      frame->GetName().c_str(),
      ref_force.X(), ref_force.Y(), ref_force.Z(),
      ref_torque.X(), ref_torque.Y(), ref_torque.Z(),
      body->GetName().c_str(),
      target_force.X(), target_force.Y(), target_force.Z(),
      target_torque.X(), target_torque.Y(), target_torque.Z());

  } else if (ref_frame.empty() || ref_frame == "world" || ref_frame == "map") {
    ignition::math::Pose3d target_to_reference = body->WorldPose();
    target_force = ref_force;
    target_torque = ref_torque;
    RCLCPP_INFO(ros_node_->get_logger(), "Reference_frame is empty/world/map,"
      "using inertial frame, transferring from body relative to inertial frame");

  } else {
    RCLCPP_ERROR(ros_node_->get_logger(), "Reference_frame is not a valid entity name");
    _res->success = false;
    _res->status_message = "Reference_frame not found";
    return;
  }

  auto body_wrench_task = std::make_shared<BodyWrenchTask>();
  body_wrench_task->link = body;
  body_wrench_task->force = target_force;
  body_wrench_task->torque = target_torque;
  body_wrench_task->start_time = _req->start_time;
  body_wrench_task->duration = _req->duration;

  std::lock_guard<std::mutex> scoped_lock(wrench_lock_);
  body_wrench_tasks_.push_back(body_wrench_task);
  _res->success = true;
}

void GazeboRosEffortPrivate::transformWrench(
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

void GazeboRosEffortPrivate::ClearBodyWrenches(
  gazebo_msgs::srv::BodyRequest::Request::SharedPtr _req,
  gazebo_msgs::srv::BodyRequest::Response::SharedPtr /*_res*/)
{
  bool found = false;
  std::lock_guard<std::mutex> scoped_lock(wrench_lock_);
  for (auto body_wrench_task = body_wrench_tasks_.begin();
    body_wrench_task != body_wrench_tasks_.end(); ++body_wrench_task)
  {
    if ((*body_wrench_task)->link->GetScopedName() == _req->body_name) {
      body_wrench_tasks_.erase(body_wrench_task--);
      RCLCPP_INFO(ros_node_->get_logger(), "Deleted wrench on [%s]", _req->body_name.c_str());
      found = true;
    }
  }
  if (!found) {
    RCLCPP_WARN(ros_node_->get_logger(), "No applied wrenches on [%s]", _req->body_name.c_str());
  }
}

void GazeboRosEffortPrivate::ApplyJointEffort(
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

      std::lock_guard<std::mutex> lock(effort_lock_);
      joint_effort_tasks_.push_back(joint_effort_task);

      _res->success = true;
      _res->status_message = "Effort on joint [" + _req->joint_name + "] set";
      return;
    }
  }
  _res->success = false;
  _res->status_message = "Joint not found";
}

void GazeboRosEffortPrivate::ClearJointForces(
  gazebo_msgs::srv::JointRequest::Request::SharedPtr _req,
  gazebo_msgs::srv::JointRequest::Response::SharedPtr /*_res*/)
{
  bool found = false;
  std::lock_guard<std::mutex> scoped_lock(effort_lock_);
  for (auto joint_effort_task = joint_effort_tasks_.begin();
    joint_effort_task != joint_effort_tasks_.end(); ++joint_effort_task)
  {
    if ((*joint_effort_task)->joint->GetName() == _req->joint_name) {
      joint_effort_tasks_.erase(joint_effort_task--);
      RCLCPP_INFO(ros_node_->get_logger(), "Deleted force on [%s]", _req->joint_name.c_str());
      found = true;
    }
  }
  if (!found) {
    RCLCPP_WARN(ros_node_->get_logger(), "No applied forces on [%s]", _req->joint_name.c_str());
  }
}

GZ_REGISTER_SYSTEM_PLUGIN(GazeboRosEffort)

}  // namespace gazebo_ros
