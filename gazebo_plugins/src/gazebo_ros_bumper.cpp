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
 * \desc Bumper controller
 * \author Nate Koenig
 * \date 09 Sept. 2008
 */

#include <gazebo/plugins/ContactPlugin.hh>

#include <gazebo_msgs/msg/contact_state.hpp>
#include <gazebo_msgs/msg/contacts_state.hpp>
#include <gazebo_plugins/gazebo_ros_bumper.hpp>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/conversions/gazebo_msgs.hpp>
#include <gazebo_ros/node.hpp>
#include <gazebo_ros/utils.hpp>
#include <geometry_msgs/msg/wrench.hpp>

#ifdef IGN_PROFILER_ENABLE
#include <ignition/common/Profiler.hh>
#endif

#include <memory>
#include <string>

namespace gazebo_plugins
{
class GazeboRosBumperPrivate
{
public:
  /// Callback to be called when sensor updates.
  void OnUpdate();

  /// A pointer to the GazeboROS node.
  gazebo_ros::Node::SharedPtr ros_node_{nullptr};

  /// Contact mesage publisher.
  rclcpp::Publisher<gazebo_msgs::msg::ContactsState>::SharedPtr pub_{nullptr};

  /// Pointer to sensor
  gazebo::sensors::ContactSensorPtr parent_sensor_;

  /// Frame name, to be used by TF.
  std::string frame_name_;

  /// Connects to pre-render events.
  gazebo::event::ConnectionPtr update_connection_;
};

GazeboRosBumper::GazeboRosBumper()
: impl_(std::make_unique<GazeboRosBumperPrivate>())
{
}

GazeboRosBumper::~GazeboRosBumper()
{
  impl_->ros_node_.reset();
}

void GazeboRosBumper::Load(gazebo::sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
  // Initialize ROS node
  impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);

  impl_->parent_sensor_ = std::dynamic_pointer_cast<gazebo::sensors::ContactSensor>(_sensor);
  if (!impl_->parent_sensor_) {
    RCLCPP_ERROR(
      impl_->ros_node_->get_logger(),
      "Contact sensor parent is not of type ContactSensor. Aborting");
    impl_->ros_node_.reset();
    return;
  }

  // Get QoS profiles
  const gazebo_ros::QoS & qos = impl_->ros_node_->get_qos();

  // Contact state publisher
  impl_->pub_ = impl_->ros_node_->create_publisher<gazebo_msgs::msg::ContactsState>(
    "bumper_states", qos.get_publisher_qos("bumper_states", rclcpp::SensorDataQoS().reliable()));

  RCLCPP_INFO(
    impl_->ros_node_->get_logger(), "Publishing contact states to [%s]",
    impl_->pub_->get_topic_name());

  // Get tf frame for output
  impl_->frame_name_ = _sdf->Get<std::string>("frame_name", "world").first;

  impl_->update_connection_ = impl_->parent_sensor_->ConnectUpdated(
    std::bind(&GazeboRosBumperPrivate::OnUpdate, impl_.get()));

  impl_->parent_sensor_->SetActive(true);
}

void GazeboRosBumperPrivate::OnUpdate()
{
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE("GazeboRosBumperPrivate::OnUpdate");
  IGN_PROFILE_BEGIN("fill message");
#endif
  gazebo::msgs::Contacts contacts;
  contacts = parent_sensor_->Contacts();

  auto contact_state_msg = gazebo_ros::Convert<gazebo_msgs::msg::ContactsState>(contacts);
  contact_state_msg.header.frame_id = frame_name_;
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_END();

  IGN_PROFILE_BEGIN("publish");
#endif

  pub_->publish(contact_state_msg);
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_END();
#endif
}

GZ_REGISTER_SENSOR_PLUGIN(GazeboRosBumper)
}  // namespace gazebo_plugins
