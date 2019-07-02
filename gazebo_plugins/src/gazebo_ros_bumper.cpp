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
#include <gazebo_ros/node.hpp>
#include <gazebo_ros/utils.hpp>
#include <geometry_msgs/msg/wrench.hpp>

#include <memory>
#include <string>

namespace gazebo_plugins
{
class GazeboRosBumperPrivate
{
public:
  /// Callback to be called on contact.
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
    RCLCPP_ERROR(impl_->ros_node_->get_logger(),
      "Contact sensor parent is not of type ContactSensor. Aborting");
    impl_->ros_node_.reset();
    return;
  }

  // Contact state publisher
  impl_->pub_ = impl_->ros_node_->create_publisher<gazebo_msgs::msg::ContactsState>(
    "bumper_states", rclcpp::SensorDataQoS());

  RCLCPP_INFO(impl_->ros_node_->get_logger(), "Publishing contact states to [%s]",
    impl_->pub_->get_topic_name());

  // Get tf frame for output
  impl_->frame_name_ = _sdf->Get<std::string>("frame_name", "world").first;

  impl_->update_connection_ = impl_->parent_sensor_->ConnectUpdated(
    std::bind(&GazeboRosBumperPrivate::OnUpdate, impl_.get()));

  impl_->parent_sensor_->SetActive(true);
}

void GazeboRosBumperPrivate::OnUpdate()
{
  gazebo::msgs::Contacts contacts;
  contacts = parent_sensor_->Contacts();

  gazebo_msgs::msg::ContactsState contact_state_msg;
  contact_state_msg.header.frame_id = frame_name_;
  contact_state_msg.header.stamp =
    rclcpp::Time(contacts.time().sec(), static_cast<uint32_t>(contacts.time().nsec()));

  ignition::math::Pose3d pose, frame_pose;
  ignition::math::Quaterniond rot, frame_rot;
  ignition::math::Vector3d pos, frame_pos;

  frame_pos = ignition::math::Vector3d(0, 0, 0);
  frame_rot = ignition::math::Quaterniond(1, 0, 0, 0);  // gazebo u,x,y,z == identity
  frame_pose = ignition::math::Pose3d(frame_pos, frame_rot);

  // set contact states size
  contact_state_msg.states.clear();

  // GetContacts returns all contacts on the collision body
  int contactsPacketSize = contacts.contact_size();
  for (int i = 0; i < contactsPacketSize; ++i) {
    // For each collision contact
    // Create a ContactState
    gazebo_msgs::msg::ContactState state;
    const gazebo::msgs::Contact & contact = contacts.contact(i);

    state.collision1_name = contact.collision1();
    state.collision2_name = contact.collision2();
    std::ostringstream stream;
    stream << "Debug:  i:(" << i << "/" << contactsPacketSize <<
      ")     my geom:" << state.collision1_name <<
      "   other geom:" << state.collision2_name <<
      "         time:" << contact.time().sec() << "." << contact.time().nsec() << std::endl;

    state.info = stream.str();

    state.wrenches.clear();
    state.contact_positions.clear();
    state.contact_normals.clear();
    state.depths.clear();

    // sum up all wrenches for each DOF
    geometry_msgs::msg::Wrench total_wrench;
    total_wrench.force.x = 0;
    total_wrench.force.y = 0;
    total_wrench.force.z = 0;
    total_wrench.torque.x = 0;
    total_wrench.torque.y = 0;
    total_wrench.torque.z = 0;

    int contactGroupSize = contact.position_size();
    for (int j = 0; j < contactGroupSize; ++j) {
      // loop through individual contacts between collision1 and collision2
      // gzerr << j << "  Position:"
      //       << contact.position(j).x() << " "
      //       << contact.position(j).y() << " "
      //       << contact.position(j).z() << "\n";
      // gzerr << "   Normal:"
      //       << contact.normal(j).x() << " "
      //       << contact.normal(j).y() << " "
      //       << contact.normal(j).z() << "\n";
      // gzerr << "   Depth:" << contact.depth(j) << "\n";

      // Get force, torque and rotate into user specified frame.
      // frame_rot is identity if world is used (default for now)
      ignition::math::Vector3d force = frame_rot.RotateVectorReverse(ignition::math::Vector3d(
            contact.wrench(j).body_1_wrench().force().x(),
            contact.wrench(j).body_1_wrench().force().y(),
            contact.wrench(j).body_1_wrench().force().z()));
      ignition::math::Vector3d torque = frame_rot.RotateVectorReverse(ignition::math::Vector3d(
            contact.wrench(j).body_1_wrench().torque().x(),
            contact.wrench(j).body_1_wrench().torque().y(),
            contact.wrench(j).body_1_wrench().torque().z()));

      // set wrenches
      geometry_msgs::msg::Wrench wrench;
      wrench.force.x = force.X();
      wrench.force.y = force.Y();
      wrench.force.z = force.Z();
      wrench.torque.x = torque.X();
      wrench.torque.y = torque.Y();
      wrench.torque.z = torque.Z();
      state.wrenches.push_back(wrench);

      total_wrench.force.x += wrench.force.x;
      total_wrench.force.y += wrench.force.y;
      total_wrench.force.z += wrench.force.z;
      total_wrench.torque.x += wrench.torque.x;
      total_wrench.torque.y += wrench.torque.y;
      total_wrench.torque.z += wrench.torque.z;

      // transform contact positions into relative frame
      // set contact positions
      ignition::math::Vector3d position =
        frame_rot.RotateVectorReverse(ignition::math::Vector3d(
            contact.position(j).x(), contact.position(j).y(), contact.position(j).z()) - frame_pos);
      geometry_msgs::msg::Vector3 contact_position;
      contact_position.x = position.X();
      contact_position.y = position.Y();
      contact_position.z = position.Z();
      state.contact_positions.push_back(contact_position);

      // rotate normal into user specified frame.
      // frame_rot is identity if world is used.
      ignition::math::Vector3d normal =
        frame_rot.RotateVectorReverse(ignition::math::Vector3d(
            contact.normal(j).x(), contact.normal(j).y(), contact.normal(j).z()));
      // set contact normals
      geometry_msgs::msg::Vector3 contact_normal;
      contact_normal.x = normal.X();
      contact_normal.y = normal.Y();
      contact_normal.z = normal.Z();
      state.contact_normals.push_back(contact_normal);

      // set contact depth, interpenetration
      state.depths.push_back(contact.depth(j));
    }

    state.total_wrench = total_wrench;
    contact_state_msg.states.push_back(state);
  }

  pub_->publish(contact_state_msg);
}

GZ_REGISTER_SENSOR_PLUGIN(GazeboRosBumper)
}  // namespace gazebo_plugins
