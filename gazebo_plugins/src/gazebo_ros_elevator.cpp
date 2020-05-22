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

#include <gazebo_plugins/gazebo_ros_elevator.hpp>
#include <gazebo_ros/node.hpp>
#include <std_msgs/msg/string.hpp>

#include <limits>
#include <memory>

namespace gazebo_plugins
{
class GazeboRosElevatorPrivate
{
public:
  /// A pointer to the GazeboROS node.
  gazebo_ros::Node::SharedPtr ros_node_;

  /// Subscriber to elevator commands
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;

  /// Lower most floor
  int bottom_;

  /// Top most floor
  int top_;
};

GazeboRosElevator::GazeboRosElevator()
: impl_(std::make_unique<GazeboRosElevatorPrivate>())
{
}

GazeboRosElevator::~GazeboRosElevator()
{
}

void GazeboRosElevator::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  ElevatorPlugin::Load(_model, _sdf);

  // Initialize ROS node
  impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);

  // Get QoS profiles
  const gazebo_ros::QoS & qos = impl_->ros_node_->get_qos();

  impl_->sub_ = impl_->ros_node_->create_subscription<std_msgs::msg::String>(
    "elevator", qos.get_subscription_qos("elevator", rclcpp::QoS(1)),
    std::bind(&GazeboRosElevator::OnElevator, this, std::placeholders::_1));

  RCLCPP_INFO(impl_->ros_node_->get_logger(), "Subscribed to [%s]", impl_->sub_->get_topic_name());

  impl_->bottom_ = _sdf->Get<int>("bottom_floor", std::numeric_limits<int>::min()).first;

  impl_->top_ = _sdf->Get<int>("top_floor", std::numeric_limits<int>::max()).first;
}

void GazeboRosElevator::OnElevator(const std_msgs::msg::String::ConstSharedPtr msg)
{
  try {
    int target_floor = std::stoi(msg->data);
    if (target_floor < impl_->bottom_) {
      RCLCPP_ERROR(impl_->ros_node_->get_logger(), "Target floor number below lowermost floor");
      return;
    }
    if (target_floor > impl_->top_) {
      RCLCPP_ERROR(impl_->ros_node_->get_logger(), "Target floor number above topmost floor");
      return;
    }
    MoveToFloor(target_floor);
  } catch (const std::exception & /*e*/) {
    RCLCPP_ERROR(impl_->ros_node_->get_logger(), "Invalid floor number for elevator");
  }
}

GZ_REGISTER_MODEL_PLUGIN(GazeboRosElevator)
}  // namespace gazebo_plugins
