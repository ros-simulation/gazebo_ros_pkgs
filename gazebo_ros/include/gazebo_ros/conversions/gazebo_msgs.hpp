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

#ifndef GAZEBO_ROS__CONVERSIONS__GAZEBO_MSGS_HPP_
#define GAZEBO_ROS__CONVERSIONS__GAZEBO_MSGS_HPP_

#include <gazebo/msgs/contacts.pb.h>
#include <gazebo_msgs/msg/contact_state.hpp>
#include <gazebo_msgs/msg/contacts_state.hpp>

#include <sstream>

#include "gazebo_ros/conversions/builtin_interfaces.hpp"
#include "gazebo_ros/conversions/generic.hpp"
#include "gazebo_ros/conversions/geometry_msgs.hpp"

namespace gazebo_ros
{
/// Generic conversion from an Gazebo Contact message to another type.
/// \param[in] in Input message;
/// \return Conversion result
/// \tparam T Output type
template<class T>
inline
T Convert(const gazebo::msgs::Contacts &)
{
  T::ConversionNotImplemented;
}

/// \brief Specialized conversion from an Gazebo message to a ROS Contacts State.
/// \param[in] in Input message;
/// \return A ROS Contacts state message with the same data as the input message
template<>
inline
gazebo_msgs::msg::ContactsState Convert(const gazebo::msgs::Contacts & in)
{
  gazebo_msgs::msg::ContactsState contact_state_msg;
  contact_state_msg.header.stamp = Convert<builtin_interfaces::msg::Time>(in.time());

  ignition::math::Quaterniond frame_rot;
  ignition::math::Vector3d frame_pos;

  int contacts_packet_size = in.contact_size();
  for (int i = 0; i < contacts_packet_size; ++i) {
    // For each collision contact
    // Create a ContactState
    gazebo_msgs::msg::ContactState state;
    const gazebo::msgs::Contact & contact = in.contact(i);

    state.collision1_name = contact.collision1();
    state.collision2_name = contact.collision2();
    std::ostringstream stream;
    stream << "Debug:  i: (" << i + 1 << "/" << contacts_packet_size <<
      ")     my_geom: [" << state.collision1_name <<
      "]  other_geom: [" << state.collision2_name <<
      "]        time: [" << contact.time().sec() << "." << contact.time().nsec() << "]\n";

    state.info = stream.str();

    // sum up all wrenches for each DOF
    geometry_msgs::msg::Wrench total_wrench;

    int contact_group_size = contact.position_size();
    for (int j = 0; j < contact_group_size; ++j) {
      // loop through individual contacts between collision1 and collision2

      // Get force, torque and rotate into user specified frame.
      // frame_rot is identity if world is used (default for now)
      ignition::math::Vector3d force = frame_rot.RotateVectorReverse(
        ignition::math::Vector3d(
          contact.wrench(j).body_1_wrench().force().x(),
          contact.wrench(j).body_1_wrench().force().y(),
          contact.wrench(j).body_1_wrench().force().z()));
      ignition::math::Vector3d torque = frame_rot.RotateVectorReverse(
        ignition::math::Vector3d(
          contact.wrench(j).body_1_wrench().torque().x(),
          contact.wrench(j).body_1_wrench().torque().y(),
          contact.wrench(j).body_1_wrench().torque().z()));

      // set wrenches
      geometry_msgs::msg::Wrench wrench;
      wrench.force = Convert<geometry_msgs::msg::Vector3>(force);
      wrench.torque = Convert<geometry_msgs::msg::Vector3>(torque);
      state.wrenches.push_back(wrench);

      total_wrench.force.x += wrench.force.x;
      total_wrench.force.y += wrench.force.y;
      total_wrench.force.z += wrench.force.z;
      total_wrench.torque.x += wrench.torque.x;
      total_wrench.torque.y += wrench.torque.y;
      total_wrench.torque.z += wrench.torque.z;

      // transform contact positions into relative frame
      // set contact positions
      ignition::math::Vector3d position = frame_rot.RotateVectorReverse(
        ignition::math::Vector3d(
          contact.position(j).x(), contact.position(j).y(), contact.position(j).z()) - frame_pos);

      auto contact_position = Convert<geometry_msgs::msg::Vector3>(position);

      state.contact_positions.push_back(contact_position);

      // rotate normal into user specified frame.
      // frame_rot is identity if world is used.
      ignition::math::Vector3d normal = frame_rot.RotateVectorReverse(
        ignition::math::Vector3d(
          contact.normal(j).x(), contact.normal(j).y(), contact.normal(j).z()));
      // set contact normals
      auto contact_normal = Convert<geometry_msgs::msg::Vector3>(normal);
      state.contact_normals.push_back(contact_normal);

      // set contact depth, interpenetration
      state.depths.push_back(contact.depth(j));
    }

    state.total_wrench = total_wrench;
    contact_state_msg.states.push_back(state);
  }

  return contact_state_msg;
}

}  // namespace gazebo_ros
#endif  // GAZEBO_ROS__CONVERSIONS__GAZEBO_MSGS_HPP_
