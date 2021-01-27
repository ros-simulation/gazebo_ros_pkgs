// Copyright 2013 Open Source Robotics Foundation, Inc.
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

#include <gazebo/common/Events.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo_plugins/gazebo_ros_simple_blade.hpp>
#include <gazebo_ros/conversions/geometry_msgs.hpp>
#include <gazebo_ros/node.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <rclcpp/rclcpp.hpp>
#include <gazebo_plugins/bool_array.hpp>

#include <memory>
#include <string>

namespace gazebo_plugins
{
class GazeboRosSimpleBladePrivate
{
public:
  /// A pointer to the Link, where force is applied
  gazebo::physics::LinkPtr link_;

  /// A pointer to the GazeboROS node.
  gazebo_ros::Node::SharedPtr ros_node_;

  /// Wrench subscriber
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

  /// Container for the wrench force that this plugin exerts on the body.
  sensor_msgs::msg::JointState joint_state_msg_;
  std::vector<geometry_msgs::msg::Wrench> wrench_msg_vec_;
  std::vector<gazebo::physics::LinkPtr> links_;
  std::vector<ignition::math::v4::Vector3d> offset_vec_;

  // Pointer to the update event connection
  gazebo::event::ConnectionPtr update_connection_;

  /// Indicates that the rotation direction applied at the motor. 1: Clockwise, -1: Anti-clockwise
  bool rotation_direction;

  /// Drag constant(all axis)
  double km_gain;
  /// Thrust (z axis) gains
  double kf_gain;
  /// Set rotation direction
  bool_array clockwise_vec;

  bool empty_msg;

};

GazeboRosSimpleBlade::GazeboRosSimpleBlade()
: impl_(std::make_unique<GazeboRosSimpleBladePrivate>())
{
}

GazeboRosSimpleBlade::~GazeboRosSimpleBlade()
{
}

void GazeboRosSimpleBlade::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
  auto logger = rclcpp::get_logger("gazebo_ros_force");

  // Target link
  if (!sdf->HasElement("link_name")) {
    RCLCPP_ERROR(logger, "Simple blade plugin missing <link_name>, cannot proceed");
    return;
  }

  sdf::ElementPtr link_elem = sdf->GetElement("link_name");
  while (link_elem) {
    auto link_name = link_elem->Get<std::string>();
    auto link = model->GetLink(link_name);
    if (!link) {
      RCLCPP_ERROR(impl_->ros_node_->get_logger(), "Link %s does not exist!", link_name.c_str());
    } else {
      impl_->links_.push_back(link);
    }
    link_elem = link_elem->GetNextElement("link_name");
  }

  if (impl_->links_.empty()) {
    RCLCPP_ERROR(impl_->ros_node_->get_logger(), "No links found.");
    impl_->ros_node_.reset();
    return;
  }

  if (!sdf->HasElement("blade_offset")) {
    RCLCPP_ERROR(logger, "Simple blade plugin missing <blade_offset>, cannot proceed");
    return;
  }
  sdf::ElementPtr offset_elem = sdf->GetElement("blade_offset");
  while (offset_elem) {
    auto blade_offset = offset_elem->Get<ignition::math::v4::Vector3d>();
    impl_->offset_vec_.push_back(blade_offset);
    offset_elem = offset_elem->GetNextElement("blade_offset");
  }

  if (!sdf->HasElement("kf_gain")) {
    RCLCPP_ERROR(logger, "Simple blade plugin missing <kf_gain>, cannot proceed");
    return;
  }

  impl_->kf_gain = sdf->GetElement("kf_gain")->Get<double>();

  if (!sdf->HasElement("km_gain")) {
    RCLCPP_ERROR(logger, "Simple blade plugin missing <km_gain>, cannot proceed");
    return;
  }

  impl_->km_gain = sdf->GetElement("km_gain")->Get<double>();

  if (!sdf->HasElement("clockwise")) {
    RCLCPP_ERROR(logger, "Simple blade plugin missing <clockwise>, cannot proceed");
    return;
  }

  impl_->clockwise_vec = sdf->GetElement("clockwise")->Get<bool_array>();

  impl_->ros_node_ = gazebo_ros::Node::Get(sdf);
  // Subscribe to JointState msg
  impl_->joint_state_sub_ = impl_->ros_node_->create_subscription<sensor_msgs::msg::JointState>(
    "gazebo_ros_simple_blade", rclcpp::SystemDefaultsQoS(),
    std::bind(&GazeboRosSimpleBlade::OnRosJointStateMsg, this, std::placeholders::_1));

  // Callback on every iteration
  impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&GazeboRosSimpleBlade::OnUpdate, this));

  impl_->empty_msg = true;
  impl_->wrench_msg_vec_.resize(impl_->links_.size());
}

void GazeboRosSimpleBlade::OnRosJointStateMsg(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  if (msg->name.empty()) {
    RCLCPP_ERROR(
      impl_->ros_node_->get_logger(),
      "No joint names in msg");
    return;
  }

  if (msg->name.size() != impl_->links_.size()) {
    RCLCPP_ERROR(
      impl_->ros_node_->get_logger(),
      "All joints from model must be addressed in msg");
    return;
  }

  impl_->joint_state_msg_ = *msg;
  impl_->empty_msg = false;

}

void GazeboRosSimpleBlade::OnUpdate()
{
  std::vector<bool> direction = impl_->clockwise_vec.get();
  // REMOVE LATER!
  if(impl_->empty_msg)
    return;

  for (size_t i = 0; i < impl_->links_.size(); ++i)
  {
    auto omega = impl_->joint_state_msg_.velocity[i];

    // Drag torque
    impl_->wrench_msg_vec_[i].torque.z = impl_->km_gain * omega * omega * (-0.5 + direction[i]) * 2;

    // Thrust
    impl_->wrench_msg_vec_[i].force.z = impl_->kf_gain * omega * omega;

    impl_->links_[i]->AddRelativeTorque(gazebo_ros::Convert<ignition::math::Vector3d>(impl_->wrench_msg_vec_[i].torque));
    impl_->links_[i]->AddLinkForce(gazebo_ros::Convert<ignition::math::Vector3d>(impl_->wrench_msg_vec_[i].force), impl_->offset_vec_[i]);
    std::cout << impl_->offset_vec_[i] << std::endl;

  }
}

GZ_REGISTER_MODEL_PLUGIN(GazeboRosSimpleBlade)
}  // namespace gazebo_plugins
