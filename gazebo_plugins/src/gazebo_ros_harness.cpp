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

#include <gazebo/physics/Model.hh>
#include <gazebo_plugins/gazebo_ros_harness.hpp>
#include <gazebo_ros/node.hpp>
#include <sdf/sdf.hh>
#include <std_msgs/msg/float32.hpp>

#include <memory>
#include <string>

namespace gazebo_plugins
{
class GazeboRosHarnessPrivate
{
public:
  /// A pointer to the GazeboROS node.
  gazebo_ros::Node::SharedPtr ros_node_;

  /// Subscriber to velocity messages
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr vel_sub_;

  /// Subscriber to detach messages
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr detach_sub_;

  /// Model name
  std::string model_;

  /// Detach status
  bool detached_;
};

GazeboRosHarness::GazeboRosHarness()
: impl_(std::make_unique<GazeboRosHarnessPrivate>())
{
}

GazeboRosHarness::~GazeboRosHarness()
{
  impl_->ros_node_.reset();
}

void GazeboRosHarness::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  gazebo::HarnessPlugin::Load(_model, _sdf);

  // Initialize ROS node
  impl_->ros_node_ = gazebo_ros::Node::Get(_sdf, _model);

  // Get QoS profiles
  const gazebo_ros::QoS & qos = impl_->ros_node_->get_qos();

  impl_->model_ = _model->GetName();

  const std::string velocity_topic = impl_->model_ + "/harness/velocity";
  impl_->vel_sub_ = impl_->ros_node_->create_subscription<std_msgs::msg::Float32>(
    velocity_topic, qos.get_subscription_qos(velocity_topic, rclcpp::QoS(1)),
    [this](const std_msgs::msg::Float32::ConstSharedPtr msg) {
      SetWinchVelocity(msg->data);
    });

  RCLCPP_INFO(
    impl_->ros_node_->get_logger(), "Subscribed to [%s]", impl_->vel_sub_->get_topic_name());

  const std::string detach_topic = impl_->model_ + "/harness/detach";
  impl_->detach_sub_ = impl_->ros_node_->create_subscription<std_msgs::msg::Empty>(
    detach_topic, qos.get_subscription_qos(detach_topic, rclcpp::QoS(1)),
    std::bind(&GazeboRosHarness::OnDetach, this, std::placeholders::_1));

  RCLCPP_INFO(
    impl_->ros_node_->get_logger(), "Subscribed to [%s]", impl_->detach_sub_->get_topic_name());

  if (_sdf->HasElement("init_vel")) {
    auto init_vel = _sdf->Get<float>("init_vel");
    SetWinchVelocity(init_vel);
    RCLCPP_INFO(
      impl_->ros_node_->get_logger(), "Setting initial harness velocity to [%.2f]m/s", init_vel);
  }
}

void GazeboRosHarness::OnDetach(const std_msgs::msg::Empty::ConstSharedPtr /*msg*/)
{
  if (impl_->detached_) {
    RCLCPP_WARN(
      impl_->ros_node_->get_logger(),
      "[%s] is already detached from harness", impl_->model_.c_str());
    return;
  }
  Detach();
  RCLCPP_INFO(impl_->ros_node_->get_logger(), "[%s] detached from harness", impl_->model_.c_str());
  impl_->detached_ = true;
}

GZ_REGISTER_MODEL_PLUGIN(GazeboRosHarness)
}  // namespace gazebo_plugins
