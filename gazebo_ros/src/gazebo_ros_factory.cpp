// Copyright 2018 Open Source Robotics Foundation, Inc.
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

#include "gazebo_ros/gazebo_ros_factory.hpp"

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Entity.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/PhysicsIface.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/transport/Node.hh>
#include <gazebo_msgs/srv/get_model_list.hpp>
#include <gazebo_msgs/srv/delete_entity.hpp>
#include <gazebo_msgs/srv/spawn_entity.hpp>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/conversions/geometry_msgs.hpp>
#include <gazebo_ros/node.hpp>
#include <gazebo_ros/utils.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

#include <tinyxml.h>
#include <algorithm>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace gazebo_ros
{

class GazeboRosFactoryPrivate
{
public:
  /// Callback when a world is created.
  /// \param[in] _world_name The world's name
  void OnWorldCreated(const std::string & _world_name);

  /// \brief Function for receiving the model list from a gazebo world.
  /// \param[in] Request
  /// \param[out] res Response
  void GetModelList(
    gazebo_msgs::srv::GetModelList::Request::SharedPtr,
    gazebo_msgs::srv::GetModelList::Response::SharedPtr res);

  /// \brief Function for inserting an entity into Gazebo from ROS Service Call.
  /// Supported formats are SDF and URDF and works with both models and lights.
  /// \param[in] req Request
  /// \param[out] res Response
  void SpawnEntity(
    gazebo_msgs::srv::SpawnEntity::Request::SharedPtr req,
    gazebo_msgs::srv::SpawnEntity::Response::SharedPtr res);

  /// \brief Function for removing an entity from Gazebo from ROS Service Call.
  /// \param[in] req Request
  /// \param[out] res Response
  void DeleteEntity(
    gazebo_msgs::srv::DeleteEntity::Request::SharedPtr req,
    gazebo_msgs::srv::DeleteEntity::Response::SharedPtr res);

  /// Iterate over all child elements and add <ros><namespace> tags to the <plugin> tags.
  /// \param[out] _elem SDF element.
  /// \param[in] _robot_namespace Namespace to be added.
  void AddNamespace(
    const sdf::ElementPtr & _elem,
    const std::string & _robot_namespace);

  /// \brief World pointer from Gazebo.
  gazebo::physics::WorldPtr world_;

  /// Reusable factory SDF for efficiency.
  sdf::SDFPtr factory_sdf_ = std::make_shared<sdf::SDF>();

  /// ROS node for communication, managed by gazebo_ros.
  gazebo_ros::Node::SharedPtr ros_node_;

  /// \brief ROS service to handle requests/responses to get model list.
  rclcpp::Service<gazebo_msgs::srv::GetModelList>::SharedPtr model_list_service_;

  /// \brief ROS service to handle requests/responses to spawn entities.
  rclcpp::Service<gazebo_msgs::srv::SpawnEntity>::SharedPtr spawn_service_;

  /// \brief ROS service to handle requests/responses to delete entities.
  rclcpp::Service<gazebo_msgs::srv::DeleteEntity>::SharedPtr delete_service_;

  /// Gazebo node for communication.
  gazebo::transport::NodePtr gz_node_;

  /// Publishes light factory messages.
  gazebo::transport::PublisherPtr gz_factory_light_pub_;

  /// Publishes request messages.
  gazebo::transport::PublisherPtr gz_request_pub_;

  /// To be notified once the world is created.
  gazebo::event::ConnectionPtr world_created_connection_;
};

GazeboRosFactory::GazeboRosFactory()
: impl_(std::make_unique<GazeboRosFactoryPrivate>())
{
}

GazeboRosFactory::~GazeboRosFactory()
{
}

void GazeboRosFactory::Load(int /* argc */, char ** /* argv */)
{
  // Keep this in the constructor for performance.
  // sdf::initFile causes disk access.
  sdf::initFile("root.sdf", impl_->factory_sdf_);

  impl_->world_created_connection_ = gazebo::event::Events::ConnectWorldCreated(
    std::bind(&GazeboRosFactoryPrivate::OnWorldCreated, impl_.get(), std::placeholders::_1));
}

void GazeboRosFactoryPrivate::OnWorldCreated(const std::string & _world_name)
{
  // Only support one world
  world_created_connection_.reset();

  world_ = gazebo::physics::get_world();

  // ROS transport
  ros_node_ = gazebo_ros::Node::Get();

  model_list_service_ = ros_node_->create_service<gazebo_msgs::srv::GetModelList>(
    "get_model_list",
    std::bind(
      &GazeboRosFactoryPrivate::GetModelList, this,
      std::placeholders::_1, std::placeholders::_2));

  spawn_service_ = ros_node_->create_service<gazebo_msgs::srv::SpawnEntity>(
    "spawn_entity",
    std::bind(
      &GazeboRosFactoryPrivate::SpawnEntity, this,
      std::placeholders::_1, std::placeholders::_2));

  delete_service_ = ros_node_->create_service<gazebo_msgs::srv::DeleteEntity>(
    "delete_entity",
    std::bind(
      &GazeboRosFactoryPrivate::DeleteEntity, this,
      std::placeholders::_1, std::placeholders::_2));

  // Gazebo transport
  gz_node_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
  gz_node_->Init(_world_name);
  gz_factory_light_pub_ = gz_node_->Advertise<gazebo::msgs::Light>("~/factory/light");
  gz_request_pub_ = gz_node_->Advertise<gazebo::msgs::Request>("~/request");
}

void GazeboRosFactoryPrivate::GetModelList(
  gazebo_msgs::srv::GetModelList::Request::SharedPtr,
  gazebo_msgs::srv::GetModelList::Response::SharedPtr res)
{
  res->header.stamp = Convert<builtin_interfaces::msg::Time>(world_->SimTime());
  res->model_names.clear();
  for (unsigned int i = 0; i < world_->ModelCount(); i++) {
    res->model_names.push_back(world_->ModelByIndex(i)->GetName());
  }
  res->success = true;
}

void GazeboRosFactoryPrivate::SpawnEntity(
  gazebo_msgs::srv::SpawnEntity::Request::SharedPtr req,
  gazebo_msgs::srv::SpawnEntity::Response::SharedPtr res)
{
  // Parse XML string into SDF
  factory_sdf_->Root()->ClearElements();
  if (!sdf::readString(req->xml, factory_sdf_)) {
    res->status_message = "Failed to parse XML string: \n" + req->xml;
    res->success = false;
    return;
  }

  // If the XML has a world, use it as root
  auto root = factory_sdf_->Root();
  if (root->HasElement("world")) {
    root = root->GetElement("world");
  }

  // Get entity SDF
  bool isLight{false};
  sdf::ElementPtr entity_elem{nullptr};
  if (root->HasElement("model")) {
    entity_elem = root->GetElement("model");
  } else if (root->HasElement("light")) {
    entity_elem = root->GetElement("light");
    isLight = true;
  }

  if (!entity_elem) {
    res->status_message = "Failed to find a model or light on SDF: \n" + root->ToString("");
    res->success = false;
    return;
  }

  // Update name if set, otherwise keep name from XML
  if (!req->name.empty()) {
    entity_elem->GetAttribute("name")->SetFromString(req->name);

    // When name is given and the entity exists, we assume it's an error and don't spawn
    // When name is not given, spawn anyway, and Gazebo will append a number to the name as needed
    if ((isLight && world_->LightByName(req->name) != nullptr) ||
      world_->ModelByName(req->name) != nullptr)
    {
      res->success = false;
      res->status_message = "Entity [" + req->name + "] already exists.";
      return;
    }
  }

  // Get desired initial pose
  auto initial_xyz = Convert<ignition::math::Vector3d>(req->initial_pose.position);
  auto initial_q = Convert<ignition::math::Quaterniond>(req->initial_pose.orientation);

  // reference frame for initial pose definition, modify initial pose if defined
  auto frame = world_->EntityByName(req->reference_frame);
  if (frame) {
    // convert to relative pose
    ignition::math::Pose3d frame_pose = frame->WorldPose();
    initial_xyz = frame_pose.Pos() + frame_pose.Rot().RotateVector(initial_xyz);
    initial_q = frame_pose.Rot() * initial_q;
  } else {
    if (req->reference_frame == "" || req->reference_frame == "world" ||
      req->reference_frame == "map" || req->reference_frame == "/map")
    {
      RCLCPP_DEBUG(
        ros_node_->get_logger(),
        "SpawnEntity: reference_frame is empty/world/map, using inertial frame");
    } else {
      res->success = false;
      res->status_message =
        "Reference frame [" + req->reference_frame + "] not found, did you forget to scope "
        "the link by model name?";
      return;
    }
  }

  // Add pose from service to pose from XML
  auto xml_pose = entity_elem->Get<ignition::math::Pose3d>("pose");
  auto new_pose = xml_pose + ignition::math::Pose3d(initial_xyz, initial_q);

  entity_elem->GetElement("pose")->Set(new_pose);

  // Walk recursively through the entire file, and add <ros><namespace> tags to all plugins
  auto robot_namespace = req->robot_namespace;
  if (!robot_namespace.empty()) {
    AddNamespace(entity_elem, robot_namespace);
  }

  // Publish factory message
  if (isLight) {
    auto msg = gazebo::msgs::LightFromSDF(entity_elem);
    gz_factory_light_pub_->Publish(msg);
  } else {
    std::ostringstream sdfStr;
    sdfStr << "<sdf version='" << SDF_VERSION << "'>" << entity_elem->ToString("") << "</sdf>";
    world_->InsertModelString(sdfStr.str());
  }

  // Only verify spawning if name is set, to keep ROS 1 functionality, but it would be more
  // complicated to check in case Gazebo appended a number to the model name.
  // TODO(louise) Revisit this once Gazebo provides a factory service that returns the final name.
  if (req->name.empty()) {
    res->success = true;
    res->status_message = "Successfully sent factory message to Gazebo.";
    return;
  }

  rclcpp::Time timeout = ros_node_->now() + rclcpp::Duration(10, 0);

  while (rclcpp::ok()) {
    if (ros_node_->now() > timeout) {
      res->success = false;
      res->status_message = "Entity pushed to spawn queue, but spawn service timed out"
        "waiting for entity to appear in simulation under the name [" + req->name + "]";
      return;
    }

    if ((isLight && world_->LightByName(req->name) != nullptr) ||
      (world_->ModelByName(req->name) != nullptr))
    {
      break;
    }
    usleep(2000);
  }

  // set result
  res->success = true;
  res->status_message = "SpawnEntity: Successfully spawned entity [" + req->name + "]";
}

void GazeboRosFactoryPrivate::DeleteEntity(
  gazebo_msgs::srv::DeleteEntity::Request::SharedPtr req,
  gazebo_msgs::srv::DeleteEntity::Response::SharedPtr res)
{
  auto entity = world_->EntityByName(req->name);
  if (!entity) {
    res->success = false;
    res->status_message = "Entity [" + req->name + "] does not exist";
    return;
  }

  // send delete request
  auto msg = gazebo::msgs::CreateRequest("entity_delete", req->name);
  gz_request_pub_->Publish(*msg, true);

  // Wait and verify that entity is deleted
  rclcpp::Time timeout = ros_node_->now() + rclcpp::Duration(10, 0);
  while (rclcpp::ok()) {
    if (ros_node_->now() > timeout) {
      res->success = false;
      res->status_message =
        "Delete service timed out waiting for entity to disappear from simulation";
      return;
    }

    if (!world_->EntityByName(req->name)) {
      break;
    }
    usleep(1000);
  }

  res->success = true;
  res->status_message = "Successfully deleted entity [" + req->name + "]";
}

void GazeboRosFactoryPrivate::AddNamespace(
  const sdf::ElementPtr & _elem,
  const std::string & _robot_namespace)
{
  for (sdf::ElementPtr child_elem = _elem->GetFirstElement(); child_elem;
    child_elem = child_elem->GetNextElement())
  {
    // <plugin>
    if (child_elem->GetName() == "plugin") {
      // Get / create <ros>
      sdf::ElementPtr ros_elem;
      if (child_elem->HasElement("ros")) {
        ros_elem = child_elem->GetElement("ros");
      } else {
        // Tell SDF it's ok for <plugin> to have <ros> element
        auto ros_desc = std::make_shared<sdf::Element>();
        ros_desc->SetName("ros");
        child_elem->AddElementDescription(ros_desc);

        // Then we can add the element
        ros_elem = child_elem->AddElement("ros");
      }

      // Get namespace element
      sdf::ElementPtr ns_elem;
      if (ros_elem->HasElement("namespace")) {
        ns_elem = ros_elem->GetElement("namespace");
      } else {
        // Tell SDF it's ok for <ros> to have <namespace> element with a string value
        auto namespace_desc = std::make_shared<sdf::Element>();
        namespace_desc->SetName("namespace");
        namespace_desc->AddValue("string", "default", true, "ROS namespace");
        ros_elem->AddElementDescription(namespace_desc);

        // Then we can add the element
        ns_elem = ros_elem->AddElement("namespace");
      }

      // Set namespace
      ns_elem->Set<std::string>(_robot_namespace);
    }
    AddNamespace(child_elem, _robot_namespace);
  }
}

GZ_REGISTER_SYSTEM_PLUGIN(GazeboRosFactory)

}  // namespace gazebo_ros
