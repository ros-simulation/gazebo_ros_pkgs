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
 * \brief Switch for Gazebo Texture Projector
 *
 * \author  Jared Duke
 *
 * \date 17 Jun 2010
 */

#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>

#include <gazebo/transport/transport.hh>
#include <gazebo_plugins/gazebo_ros_projector.hpp>
#include <gazebo_ros/node.hpp>
#ifdef IGN_PROFILER_ENABLE
#include <ignition/common/Profiler.hh>
#endif
#include <std_msgs/msg/bool.hpp>

#include <memory>
#include <string>

namespace gazebo_plugins
{
class GazeboRosProjectorPrivate
{
public:
  /// Callback for switching the projector on/off
  /// \param[in] msg Bool switch message
  void ToggleProjector(std_msgs::msg::Bool::SharedPtr msg);

  /// A pointer to the GazeboROS node.
  gazebo_ros::Node::SharedPtr ros_node_;

  /// Gazebo node used to talk to projector
  gazebo::transport::NodePtr gazebo_node_;

  /// Publisher for gazebo projector
  gazebo::transport::PublisherPtr projector_pub_;

  /// Subscriber to projector switch
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr toggle_sub_;
};

GazeboRosProjector::GazeboRosProjector()
: impl_(std::make_unique<GazeboRosProjectorPrivate>())
{
}

GazeboRosProjector::~GazeboRosProjector()
{
}

void GazeboRosProjector::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  // Initialize ROS node
  impl_->ros_node_ = gazebo_ros::Node::Get(_sdf, _model);

  // Get QoS profiles
  const gazebo_ros::QoS & qos = impl_->ros_node_->get_qos();

  // Create gazebo transport node
  impl_->gazebo_node_ = boost::make_shared<gazebo::transport::Node>();
  impl_->gazebo_node_->Init(_model->GetWorld()->Name());

  auto link_name = _sdf->Get<std::string>("projector_link", "projector_link").first;
  auto projector_name = _sdf->Get<std::string>("projector_name", "projector").first;

  // Setting projector topic
  auto name = "~/" + _model->GetName() + "/" + link_name + "/" + projector_name;

  // Create a Gazebo publisher to switch the projector
  impl_->projector_pub_ = impl_->gazebo_node_->Advertise<gazebo::msgs::Projector>(name);

  RCLCPP_INFO(
    impl_->ros_node_->get_logger(),
    "Controlling projector at [%s]", impl_->projector_pub_->GetTopic().c_str());

  impl_->toggle_sub_ = impl_->ros_node_->create_subscription<std_msgs::msg::Bool>(
    "switch", qos.get_subscription_qos("switch", rclcpp::QoS(1)),
    std::bind(&GazeboRosProjectorPrivate::ToggleProjector, impl_.get(), std::placeholders::_1));

  RCLCPP_INFO(
    impl_->ros_node_->get_logger(), "Subscribed to [%s]", impl_->toggle_sub_->get_topic_name());
}

void GazeboRosProjectorPrivate::ToggleProjector(const std_msgs::msg::Bool::SharedPtr switch_msg)
{
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE("GazeboRosProjectorPrivate::ToggleProjector");
  IGN_PROFILE_BEGIN("fill ROS message");
#endif
  if (switch_msg->data) {
    RCLCPP_INFO(ros_node_->get_logger(), "Switching on projector");
  } else {
    RCLCPP_INFO(ros_node_->get_logger(), "Switching off projector");
  }

  gazebo::msgs::Projector msg;
  msg.set_name("texture_projector");
  msg.set_enabled(switch_msg->data);
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_END();
  IGN_PROFILE_BEGIN("publish");
#endif
  projector_pub_->Publish(msg);
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_END();
#endif
}

GZ_REGISTER_MODEL_PLUGIN(GazeboRosProjector)
}  // namespace gazebo_plugins
