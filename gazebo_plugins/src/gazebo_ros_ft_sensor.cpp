// Copyright 2013 Open Source Robotics Foundation
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
 * \file gazebo_ros_ft_sensor.cpp
 *
 * \Author Sachin Chitta and John Hsu
 * \Author Francisco Suarez-Ruiz
 *
 */

#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Events.hh>
#include <gazebo_plugins/gazebo_ros_ft_sensor.hpp>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/node.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#ifdef IGN_PROFILER_ENABLE
#include <ignition/common/Profiler.hh>
#endif

#include <string>
#include <memory>

namespace gazebo_plugins
{
class GazeboRosFTSensorPrivate
{
public:
  /// Callback to be called at every simulation iteration.
  /// \param[in] _info Updated simulation info.
  void OnUpdate(const gazebo::common::UpdateInfo & _info);

  /// A pointer to Gazebo Joint.
  gazebo::physics::JointPtr joint_;

  /// A pointer to the Gazebo Link.
  gazebo::physics::LinkPtr link_;

  /// A pointer to the GazeboROS node.
  gazebo_ros::Node::SharedPtr rosnode_;

  /// WrenchedStamped message publisher.
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr pub_;

  /// Keep latest wrench_stamped message.
  geometry_msgs::msg::WrenchStamped wrench_msg_;

  /// Protect variables accessed on callbacks.
  std::mutex lock_;

  /// Connection to event called at every world iteration.
  gazebo::event::ConnectionPtr update_connection_;

  /// Wrench message frame_id
  std::string frame_id_;

  /// Keep track of the last update time.
  gazebo::common::Time last_time_;

  /// Publish rate in Hz.
  double update_rate_{0.0};

  /// Gaussian noise
  double gaussian_noise_;
};

GazeboRosFTSensor::GazeboRosFTSensor()
: impl_(std::make_unique<GazeboRosFTSensorPrivate>())
{
}

GazeboRosFTSensor::~GazeboRosFTSensor()
{
  impl_->update_connection_.reset();
}

void GazeboRosFTSensor::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  // Initialize ROS node
  impl_->rosnode_ = gazebo_ros::Node::Get(_sdf);

  // Get QoS profiles
  const gazebo_ros::QoS & qos = impl_->rosnode_->get_qos();

  if (!_sdf->HasElement("update_rate")) {
    RCLCPP_DEBUG(
      impl_->rosnode_->get_logger(),
      "ft_sensor plugin missing <update_rate>, defaults to 0.0 (as fast as possible)");
  }
  impl_->update_rate_ = _sdf->Get<double>("update_rate", 0.0).first;

  if (!_sdf->HasElement("body_name") && !_sdf->HasElement("joint_name")) {
    RCLCPP_ERROR(
      impl_->rosnode_->get_logger(),
      "ft_sensor plugin missing <body_name> and <joint_name>, cannot proceed");
    impl_->rosnode_.reset();
    return;
  }

  if (_sdf->HasElement("body_name")) {
    auto link_name = _sdf->Get<std::string>("body_name");

    impl_->link_ = _model->GetLink(link_name);

    if (!impl_->link_) {
      RCLCPP_ERROR(
        impl_->rosnode_->get_logger(),
        "Link [%s] does not exist. Aborting", link_name.c_str());
      impl_->rosnode_.reset();
      return;
    }

    impl_->frame_id_ = _sdf->Get<std::string>("frame_name", "world").first;
  } else {
    auto joint_name = _sdf->Get<std::string>("joint_name");

    impl_->joint_ = _model->GetJoint(joint_name);

    if (!impl_->joint_) {
      RCLCPP_ERROR(
        impl_->rosnode_->get_logger(),
        "Joint [%s] does not exist. Aborting", joint_name.c_str());
      impl_->rosnode_.reset();
      return;
    }

    if (_sdf->HasElement("frame_name")) {
      RCLCPP_WARN(
        impl_->rosnode_->get_logger(),
        "<frame_name> can be set only for ft_sensor on links.");
    }
    impl_->frame_id_ = impl_->joint_->GetChild()->GetName();
  }

  RCLCPP_INFO(
    impl_->rosnode_->get_logger(),
    "ft_sensor plugin reporting wrench values to the frame [%s]", impl_->frame_id_.c_str());

  if (!_sdf->HasElement("gaussian_noise")) {
    RCLCPP_DEBUG(impl_->rosnode_->get_logger(), "Missing <gassian_noise>, defaults to 0.0");
  }
  impl_->gaussian_noise_ = _sdf->Get<double>("gaussian_noise", 0.0).first;

  impl_->pub_ = impl_->rosnode_->create_publisher<geometry_msgs::msg::WrenchStamped>(
    "wrench", qos.get_publisher_qos("wrench", rclcpp::QoS(1)));

  RCLCPP_INFO(
    impl_->rosnode_->get_logger(),
    "Publishing wrenches on topic [%s]", impl_->pub_->get_topic_name());

  impl_->last_time_ = _model->GetWorld()->SimTime();

  impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&GazeboRosFTSensorPrivate::OnUpdate, impl_.get(), std::placeholders::_1));
}

void GazeboRosFTSensorPrivate::OnUpdate(const gazebo::common::UpdateInfo & _info)
{
  gazebo::common::Time current_time = _info.simTime;

  if (current_time < last_time_) {
    RCLCPP_WARN(rosnode_->get_logger(), "Negative update time difference detected.");
    last_time_ = current_time;
  }

  // Rate control
  if (update_rate_ > 0 && (current_time - last_time_).Double() < (1.0 / update_rate_)) {
    return;
  }
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE("GazeboRosFTSensorPrivate::OnUpdate");
  IGN_PROFILE_BEGIN("fill ROS message");
#endif
  ignition::math::Vector3d torque;
  ignition::math::Vector3d force;

  // get force and torque on body
  if (link_) {
    force = link_->WorldForce();
    torque = link_->WorldTorque();
  } else {
    auto wrench = joint_->GetForceTorque(0);
    force = wrench.body2Force;
    torque = wrench.body2Torque;
  }

  std::lock_guard<std::mutex> scoped_lock(lock_);

  // copy data into wrench message
  wrench_msg_.header.frame_id = frame_id_;
  wrench_msg_.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(_info.simTime);

  wrench_msg_.wrench.force.x = force.X() + ignition::math::Rand::DblNormal(0, gaussian_noise_);
  wrench_msg_.wrench.force.y = force.Y() + ignition::math::Rand::DblNormal(0, gaussian_noise_);
  wrench_msg_.wrench.force.z = force.Z() + ignition::math::Rand::DblNormal(0, gaussian_noise_);
  wrench_msg_.wrench.torque.x = torque.X() + ignition::math::Rand::DblNormal(0, gaussian_noise_);
  wrench_msg_.wrench.torque.y = torque.Y() + ignition::math::Rand::DblNormal(0, gaussian_noise_);
  wrench_msg_.wrench.torque.z = torque.Z() + ignition::math::Rand::DblNormal(0, gaussian_noise_);
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_END();
  // Publish to ROS
  IGN_PROFILE_BEGIN("publish");
#endif
  pub_->publish(wrench_msg_);
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_END();
#endif
  // Save last time stamp
  last_time_ = current_time;
}
GZ_REGISTER_MODEL_PLUGIN(GazeboRosFTSensor)
}  // namespace gazebo_plugins
