// Copyright 2020 Open Source Robotics Foundation, Inc.
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


#include <gazebo_plugins/gazebo_ros_logical_camera.hpp>
#include <gazebo/sensors/LogicalCameraSensor.hh>
#include <gazebo_msgs/msg/logical_camera_image.hpp>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/conversions/geometry_msgs.hpp>
#include <gazebo_ros/node.hpp>
#include <gazebo_ros/utils.hpp>

#include <memory>

namespace gazebo
{
namespace sensors
{

class GazeboRosLogicalCameraPrivate
{
public:
  /// Node for ros communication
  gazebo_ros::Node::SharedPtr ros_node_;
  /// Publish for logical camera message
  rclcpp::Publisher<gazebo_msgs::msg::LogicalCameraImage>::SharedPtr pub_;
  /// LogicalCameraImage message modified each update
  gazebo_msgs::msg::LogicalCameraImage::SharedPtr msg_;
  /// LogicalCamera sensor this plugin is attached to
  gazebo::sensors::LogicalCameraSensorPtr sensor_;
  /// Event triggered when sensor updates
  gazebo::event::ConnectionPtr sensor_update_event_;
  /// Publish latest logical camera data to ROS
  void OnUpdate();
};

GazeboRosLogicalCamera::GazeboRosLogicalCamera()
: impl_(std::make_unique<GazeboRosLogicalCameraPrivate>())
{
}

GazeboRosLogicalCamera::~GazeboRosLogicalCamera()
{
}

void GazeboRosLogicalCamera::Load(gazebo::sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
  this->impl_->sensor_ = std::dynamic_pointer_cast<gazebo::sensors::LogicalCameraSensor>(_sensor);
  this->impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);

  auto topic_name = this->impl_->sensor_->Topic();

  impl_->pub_ =
    impl_->ros_node_->create_publisher<gazebo_msgs::msg::LogicalCameraImage>(
    topic_name, rclcpp::SensorDataQoS());

  RCLCPP_INFO(impl_->ros_node_->get_logger(), "Created [%s] topic", impl_->pub_->get_topic_name());

  // Create message to be reused
  impl_->msg_ = std::make_shared<gazebo_msgs::msg::LogicalCameraImage>();

  impl_->sensor_update_event_ = impl_->sensor_->ConnectUpdated(
    std::bind(&GazeboRosLogicalCameraPrivate::OnUpdate, impl_.get()));
}

void GazeboRosLogicalCameraPrivate::OnUpdate()
{
  const auto & image = this->sensor_->Image();

  msg_->camera_pose = gazebo_ros::Convert<geometry_msgs::msg::Pose>(
    gazebo::msgs::ConvertIgn(image.pose()));

  msg_->model_name.clear();
  msg_->model_pose.clear();

  for (int i = 0; i < image.model_size(); i++) {
    const auto & model = image.model(i);
    const auto model_name = model.name();
    const auto model_pose = gazebo_ros::Convert<geometry_msgs::msg::Pose>(
      gazebo::msgs::ConvertIgn(model.pose()));
    msg_->model_name.push_back(model_name);
    msg_->model_pose.push_back(model_pose);
  }

  this->pub_->publish(*msg_);
}

GZ_REGISTER_SENSOR_PLUGIN(GazeboRosLogicalCamera)

}  // namespace sensors
}  // namespace gazebo
