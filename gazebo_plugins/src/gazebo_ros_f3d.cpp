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
 * \file gazebo_ros_diff_drive.cpp
 *
 * \brief This is a controller that simulates a 6 dof force sensor.
 * (Force Feed Back Ground Truth)
 *
 * \Author Sachin Chitta and John Hsu
 *
 * \Date 1 June 2008
 */

#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Events.hh>
#include <gazebo_plugins/gazebo_ros_f3d.hpp>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/node.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>

#include <string>
#include <memory>

namespace gazebo_plugins
{
class GazeboRosF3DPrivate
{
public:
  /// Callback to be called at every simulation iteration.
  /// \param[in] _info Updated simulation info.
  void OnUpdate(const gazebo::common::UpdateInfo & _info);

  /// A pointer to the Gazebo Body.
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
};

GazeboRosF3D::GazeboRosF3D()
: impl_(std::make_unique<GazeboRosF3DPrivate>())
{
}

GazeboRosF3D::~GazeboRosF3D()
{
  impl_->update_connection_.reset();
}

void GazeboRosF3D::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  // Initialize ROS node
  impl_->rosnode_ = gazebo_ros::Node::Get(_sdf);

  if (!_sdf->HasElement("body_name")) {
    RCLCPP_WARN(impl_->rosnode_->get_logger(), "F3D plugin missing <body_name>, cannot proceed");
    return;
  }

  auto link_name = _sdf->Get<std::string>("body_name", "link").first;

  impl_->link_ = _model->GetLink(link_name);

  if (!impl_->link_) {
    RCLCPP_WARN(impl_->rosnode_->get_logger(),
      "Link [%s] does not exist\n. Aborting", link_name.c_str());
    return;
  }

  impl_->frame_id_ = _sdf->Get<std::string>("frame_name", "world").first;

  impl_->pub_ = impl_->rosnode_->create_publisher<geometry_msgs::msg::WrenchStamped>("wrench");

  impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&GazeboRosF3DPrivate::OnUpdate, impl_.get(), std::placeholders::_1));
}

void GazeboRosF3DPrivate::OnUpdate(const gazebo::common::UpdateInfo & _info)
{
  ignition::math::Vector3d torque;
  ignition::math::Vector3d force;

  // get force and torque on body
  force = link_->WorldForce();
  torque = link_->WorldTorque();

  std::lock_guard<std::mutex> scoped_lock(lock_);

  // copy data into wrench message
  wrench_msg_.header.frame_id = frame_id_;
  wrench_msg_.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(_info.simTime);

  wrench_msg_.wrench.force.x = force.X();
  wrench_msg_.wrench.force.y = force.Y();
  wrench_msg_.wrench.force.z = force.Z();
  wrench_msg_.wrench.torque.x = torque.X();
  wrench_msg_.wrench.torque.y = torque.Y();
  wrench_msg_.wrench.torque.z = torque.Z();

  pub_->publish(wrench_msg_);
}
GZ_REGISTER_MODEL_PLUGIN(GazeboRosF3D)
}  // namespace gazebo_plugins
