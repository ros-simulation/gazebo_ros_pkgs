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

/*
 * \brief Vacuum Gripper plugin for attracting entities around the model like vacuum
 *
 * \author  Kentaro Wada
 *
 * \date 7 Dec 2015
 */

#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo_plugins/gazebo_ros_vacuum_gripper.hpp>
#include <gazebo_ros/node.hpp>
#ifdef IGN_PROFILER_ENABLE
#include <ignition/common/Profiler.hh>
#endif
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <sdf/sdf.hh>

#include <algorithm>
#include <memory>
#include <string>
#include <unordered_set>

namespace gazebo_plugins
{
class GazeboRosVacuumGripperPrivate
{
public:
  /// Callback to be called at every simulation iteration.
  void OnUpdate();

  /// \brief Function to switch the gripper on/off.
  /// \param[in] req Request
  /// \param[out] res Response
  void OnSwitch(
    std_srvs::srv::SetBool::Request::SharedPtr req,
    std_srvs::srv::SetBool::Response::SharedPtr res);

  /// A pointer to the GazeboROS node.
  gazebo_ros::Node::SharedPtr ros_node_;

  /// Publisher for gripper action status
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_;

  /// Service for gripper switch
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;

  /// Connection to event called at every world iteration.
  gazebo::event::ConnectionPtr update_connection_;

  /// Pointer to world.
  gazebo::physics::WorldPtr world_;

  /// Pointer to link.
  gazebo::physics::LinkPtr link_;

  /// Protect variables accessed on callbacks.
  std::mutex lock_;

  /// True if gripper is on.
  bool status_;

  /// Entities not affected by gripper.
  std::unordered_set<std::string> fixed_;

  /// Max distance to apply force.
  double max_distance_;
};

GazeboRosVacuumGripper::GazeboRosVacuumGripper()
: impl_(std::make_unique<GazeboRosVacuumGripperPrivate>())
{
}

GazeboRosVacuumGripper::~GazeboRosVacuumGripper()
{
}

void GazeboRosVacuumGripper::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  impl_->world_ = _model->GetWorld();

  // Initialize ROS node
  impl_->ros_node_ = gazebo_ros::Node::Get(_sdf, _model);

  // Get QoS profiles
  const gazebo_ros::QoS & qos = impl_->ros_node_->get_qos();

  if (_sdf->HasElement("link_name")) {
    auto link = _sdf->Get<std::string>("link_name");
    impl_->link_ = _model->GetLink(link);
    if (!impl_->link_) {
      RCLCPP_ERROR(impl_->ros_node_->get_logger(), "Link [%s] not found. Aborting", link.c_str());
      impl_->ros_node_.reset();
      return;
    }
  } else {
    RCLCPP_ERROR(impl_->ros_node_->get_logger(), "Please specify <link_name>. Aborting.");
  }

  impl_->max_distance_ = _sdf->Get<double>("max_distance", 0.05).first;

  if (_sdf->HasElement("fixed")) {
    for (auto fixed = _sdf->GetElement("fixed"); fixed != nullptr;
      fixed = fixed->GetNextElement("fixed"))
    {
      auto name = fixed->Get<std::string>();
      impl_->fixed_.insert(name);
      RCLCPP_INFO(
        impl_->ros_node_->get_logger(),
        "Model/Link [%s] exempted from gripper force", name.c_str());
    }
  }
  impl_->fixed_.insert(_model->GetName());
  impl_->fixed_.insert(impl_->link_->GetName());

  // Initialize publisher
  impl_->pub_ = impl_->ros_node_->create_publisher<std_msgs::msg::Bool>(
    "grasping", qos.get_publisher_qos("grasping", rclcpp::QoS(1)));

  RCLCPP_INFO(
    impl_->ros_node_->get_logger(),
    "Advertise gripper status on [%s]", impl_->pub_->get_topic_name());

  // Initialize service
  impl_->service_ = impl_->ros_node_->create_service<std_srvs::srv::SetBool>(
    "switch",
    std::bind(
      &GazeboRosVacuumGripperPrivate::OnSwitch, impl_.get(),
      std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(
    impl_->ros_node_->get_logger(),
    "Advertise gripper switch service on [%s]", impl_->service_->get_service_name());

  // Listen to the update event (broadcast every simulation iteration)
  impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&GazeboRosVacuumGripperPrivate::OnUpdate, impl_.get()));
}

void GazeboRosVacuumGripperPrivate::OnUpdate()
{
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE("GazeboRosVacuumGripper::OnUpdate");
#endif
  std_msgs::msg::Bool grasping_msg;
  if (!status_) {
#ifdef IGN_PROFILER_ENABLE
    IGN_PROFILE_BEGIN("publish");
#endif
    pub_->publish(grasping_msg);
#ifdef IGN_PROFILER_ENABLE
    IGN_PROFILE_END();
#endif
    return;
  }

  std::lock_guard<std::mutex> lock(lock_);

  ignition::math::Pose3d parent_pose = link_->WorldPose();
  gazebo::physics::Model_V models = world_->Models();

  for (auto & model : models) {
    if (fixed_.find(model->GetName()) != fixed_.end()) {
      continue;
    }
    gazebo::physics::Link_V links = model->GetLinks();
    for (auto & link : links) {
      ignition::math::Pose3d link_pose = link->WorldPose();
      ignition::math::Pose3d diff = parent_pose - link_pose;
      if (diff.Pos().Length() > max_distance_) {
        continue;
      }
      link->AddForce(link_pose.Rot().RotateVector((parent_pose - link_pose).Pos()).Normalize());
      grasping_msg.data = true;
    }
  }
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_BEGIN("publish grasping_msg");
#endif
  pub_->publish(grasping_msg);
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_END();
#endif
}

void GazeboRosVacuumGripperPrivate::OnSwitch(
  std_srvs::srv::SetBool::Request::SharedPtr req,
  std_srvs::srv::SetBool::Response::SharedPtr res)
{
  res->success = false;
  if (req->data) {
    if (!status_) {
      status_ = true;
      res->success = true;
    } else {
      RCLCPP_WARN(ros_node_->get_logger(), "Gripper is already on");
    }
  } else {
    if (status_) {
      status_ = false;
      res->success = true;
    } else {
      RCLCPP_WARN(ros_node_->get_logger(), "Gripper is already off");
    }
  }
}

GZ_REGISTER_MODEL_PLUGIN(GazeboRosVacuumGripper)
}  // namespace gazebo_plugins
