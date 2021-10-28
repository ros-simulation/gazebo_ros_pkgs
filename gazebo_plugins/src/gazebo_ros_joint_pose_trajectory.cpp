// Copyright 2019 Open Source Robotics Foundation
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

/*
 * \brief Set the trajectory of points to be followed by joints in simulation.
 *
 * \author  Sachin Chitta and John Hsu
 *
 * \date  1 June 2008
 */

#include <gazebo/common/Events.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo_plugins/gazebo_ros_joint_pose_trajectory.hpp>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/node.hpp>
#ifdef IGN_PROFILER_ENABLE
#include <ignition/common/Profiler.hh>
#endif
#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <memory>
#include <string>
#include <vector>

namespace gazebo_plugins
{
class GazeboRosJointPoseTrajectoryPrivate
{
public:
  /// Callback to be called at every simulation iteration.
  /// \param[in] info Updated simulation info.
  void OnUpdate(const gazebo::common::UpdateInfo & info);

  /// \brief Callback for set joint trajectory topic.
  /// \param[in] msg Trajectory msg
  void SetJointTrajectory(trajectory_msgs::msg::JointTrajectory::SharedPtr _msg);

  /// A pointer to the GazeboROS node.
  gazebo_ros::Node::SharedPtr ros_node_;

  /// Subscriber to Trajectory messages.
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr sub_;

  /// Pointer to model.
  gazebo::physics::ModelPtr model_;

  /// Pointer to world.
  gazebo::physics::WorldPtr world_;

  /// Pointer to link wrt which the trajectory is set.
  gazebo::physics::LinkPtr reference_link_;

  /// Joints for setting the trajectory.
  std::vector<gazebo::physics::JointPtr> joints_;

  /// Command trajectory points
  std::vector<trajectory_msgs::msg::JointTrajectoryPoint> points_;

  /// Period in seconds
  double update_period_;

  /// Keep last time an update was published
  gazebo::common::Time last_update_time_;

  /// Desired trajectory start time
  gazebo::common::Time trajectory_start_time_;

  /// Protect variables accessed on callbacks.
  std::mutex lock_;

  /// Index number of trajectory to be executed
  unsigned int trajectory_index_;

  /// True if trajectory is available
  bool has_trajectory_;

  /// Pointer to the update event connection.
  gazebo::event::ConnectionPtr update_connection_;
};

GazeboRosJointPoseTrajectory::GazeboRosJointPoseTrajectory()
: impl_(std::make_unique<GazeboRosJointPoseTrajectoryPrivate>())
{
}

GazeboRosJointPoseTrajectory::~GazeboRosJointPoseTrajectory()
{
}

void GazeboRosJointPoseTrajectory::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
  impl_->model_ = model;

  impl_->world_ = model->GetWorld();

  // Initialize ROS node
  impl_->ros_node_ = gazebo_ros::Node::Get(sdf, model);

  // Get QoS profiles
  const gazebo_ros::QoS & qos = impl_->ros_node_->get_qos();

  // Update rate
  auto update_rate = sdf->Get<double>("update_rate", 100.0).first;
  if (update_rate > 0.0) {
    impl_->update_period_ = 1.0 / update_rate;
  } else {
    impl_->update_period_ = 0.0;
  }
  impl_->last_update_time_ = impl_->world_->SimTime();

  // Set Joint Trajectory Callback
  impl_->sub_ = impl_->ros_node_->create_subscription<trajectory_msgs::msg::JointTrajectory>(
    "set_joint_trajectory", qos.get_subscription_qos("set_joint_trajectory", rclcpp::QoS(1)),
    std::bind(
      &GazeboRosJointPoseTrajectoryPrivate::SetJointTrajectory,
      impl_.get(), std::placeholders::_1));

  // Callback on every iteration
  impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&GazeboRosJointPoseTrajectoryPrivate::OnUpdate, impl_.get(), std::placeholders::_1));
}

void GazeboRosJointPoseTrajectoryPrivate::OnUpdate(const gazebo::common::UpdateInfo & info)
{
  gazebo::common::Time current_time = info.simTime;

  // If the world is reset, for example
  if (current_time < last_update_time_) {
    RCLCPP_INFO(ros_node_->get_logger(), "Negative sim time difference detected.");
    last_update_time_ = current_time;
  }

  // Check period
  double seconds_since_last_update = (current_time - last_update_time_).Double();

  if (seconds_since_last_update < update_period_) {
    return;
  }
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE("GazeboRosJointPoseTrajectoryPrivate::OnUpdate");
  IGN_PROFILE_BEGIN("update");
#endif
  std::lock_guard<std::mutex> scoped_lock(lock_);
  if (has_trajectory_ && current_time >= trajectory_start_time_) {
    if (trajectory_index_ < points_.size()) {
      RCLCPP_INFO(
        ros_node_->get_logger(), "time [%f] updating configuration [%d/%lu]",
        current_time.Double(), trajectory_index_ + 1, points_.size());

      // get reference link pose before updates
      auto reference_pose = model_->WorldPose();

      if (reference_link_) {
        reference_pose = reference_link_->WorldPose();
      }

      // trajectory roll-out based on time:
      // set model configuration from trajectory message
      auto chain_size = static_cast<unsigned int>(joints_.size());
      if (chain_size == points_[trajectory_index_].positions.size()) {
        for (unsigned int i = 0; i < chain_size; ++i) {
          // this is not the most efficient way to set things
          if (joints_[i]) {
            joints_[i]->SetPosition(0, points_[trajectory_index_].positions[i], true);
          }
        }
        // set model pose
        if (reference_link_) {
          model_->SetLinkWorldPose(reference_pose, reference_link_);
        } else {
          model_->SetWorldPose(reference_pose);
        }
      } else {
        RCLCPP_ERROR(
          ros_node_->get_logger(),
          "point[%u] has different number of joint names[%u] and positions[%lu].",
          trajectory_index_ + 1, chain_size, points_[trajectory_index_].positions.size());
      }

      auto duration =
        gazebo_ros::Convert<gazebo::common::Time>(points_[trajectory_index_].time_from_start);

      // reset start time for next trajectory point
      trajectory_start_time_ += duration;
      trajectory_index_++;  // increment to next trajectory point

      // Update time
      last_update_time_ = current_time;
    } else {
      // trajectory finished
      reference_link_.reset();
      // No more trajectory points
      has_trajectory_ = false;
    }
  }
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_END();
#endif
}

void GazeboRosJointPoseTrajectoryPrivate::SetJointTrajectory(
  trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
{
  std::lock_guard<std::mutex> scoped_lock(lock_);

  std::string reference_link_name = msg->header.frame_id;
  // do this every time a new joint trajectory is supplied,
  // use header.frame_id as the reference_link_name_
  if (!(reference_link_name == "world" || reference_link_name == "map")) {
    gazebo::physics::EntityPtr entity = world_->EntityByName(reference_link_name);
    if (entity) {
      reference_link_ = boost::dynamic_pointer_cast<gazebo::physics::Link>(entity);
    }
    if (!reference_link_) {
      RCLCPP_ERROR(
        ros_node_->get_logger(),
        "Plugin needs a reference link [%s] as frame_id, aborting.", reference_link_name.c_str());
      return;
    }
    model_ = reference_link_->GetParentModel();
    RCLCPP_DEBUG(
      ros_node_->get_logger(),
      "Update model pose by keeping link [%s] stationary inertially",
      reference_link_->GetName().c_str());
  }

  // copy joint configuration into a map
  auto chain_size = static_cast<unsigned int>(msg->joint_names.size());
  joints_.resize(chain_size);
  for (unsigned int i = 0; i < chain_size; ++i) {
    joints_[i] = model_->GetJoint(msg->joint_names[i]);
    if (!joints_[i]) {
      RCLCPP_ERROR(
        ros_node_->get_logger(), "Joint [%s] not found. Trajectory not set.",
        msg->joint_names[i].c_str());
      return;
    }
  }

  auto points_size = static_cast<unsigned int>(msg->points.size());
  points_.resize(points_size);
  for (unsigned int i = 0; i < points_size; ++i) {
    points_[i].positions.resize(chain_size);
    points_[i].time_from_start = msg->points[i].time_from_start;
    for (unsigned int j = 0; j < chain_size; ++j) {
      points_[i].positions[j] = msg->points[i].positions[j];
    }
  }

  // trajectory start time
  trajectory_start_time_ = gazebo_ros::Convert<gazebo::common::Time>(msg->header.stamp);

  gazebo::common::Time cur_time = world_->SimTime();
  if (trajectory_start_time_ < cur_time) {
    trajectory_start_time_ = cur_time;
  }
  // update the joint trajectory to play
  has_trajectory_ = true;
  // reset trajectory_index to beginning of new trajectory
  trajectory_index_ = 0;
}

GZ_REGISTER_MODEL_PLUGIN(GazeboRosJointPoseTrajectory)
}  // namespace gazebo_plugins
