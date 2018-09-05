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
#include <gazebo/physics/PhysicsIface.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/transport/Node.hh>
#include <gazebo_msgs/srv/spawn_entity.hpp>
#include <gazebo_ros/conversions/geometry_msgs.hpp>
#include <gazebo_ros/node.hpp>
#include <gazebo_ros/utils.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

#include <tinyxml.h>
#include <algorithm>
#include <memory>
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

  /// \brief Function for inserting a model into Gazebo from ROS Service Call.
  /// Supported formats are SDF and URDF
  /// \param[in] req Request
  /// \param[our] res Response
  void SpawnEntity(
    gazebo_msgs::srv::SpawnEntity::Request::SharedPtr req,
    gazebo_msgs::srv::SpawnEntity::Response::SharedPtr res);

  /// \brief utility for checking if string is in URDF format
  /// \param[in] xml_doc Model description XML document
  /// \return True if URDF
  bool IsURDF(const TiXmlDocument & xml_doc);

  /// \brief utility for checking if string is in SDF format
  /// \param[in] xml_doc Model description XML document
  /// \return True if SDF
  bool IsSDF(const TiXmlDocument & xml_doc);

  /// \brief replace or update initial pose and model name
  ///
  /// This function can handle both regular SDF files and <include> SDFs that are used with the
  /// Gazebo Model Database
  void updateSDFAttributes(
    TiXmlDocument & xml_doc,
    const std::string & name,
    const ignition::math::Vector3d & initial_xyz,
    const ignition::math::Quaterniond & initial_q);

  void updateURDFModelPose(
    TiXmlDocument & xml_doc,
    const ignition::math::Vector3d & initial_xyz,
    const ignition::math::Quaterniond & initial_q);

  void updateURDFName(
    TiXmlDocument & xml_doc,
    const std::string & name);

  /// Iterate over all child nodes and add <ros><namespace> tags to the <plugin> tags
  /// \param[out] xml XML node.
  /// \param[in] robot_namespace Namespace to be added
  void WalkChildAddRobotNamespace(TiXmlNode * xml, const std::string & robot_namespace);

  /// \brief convert xml to Pose
  ignition::math::Pose3d parsePose(const std::string & str);

  /// \brief convert xml to Pose
  ignition::math::Vector3d parseVector3(const std::string & str);

  std::vector<double> convertValuesFromStringToDouble(const std::string & str);

  void spawnAndConform(
    TiXmlDocument & xml_doc,
    const std::string & name,
    gazebo_msgs::srv::SpawnEntity::Response::SharedPtr res);

  GazeboRosFactoryPrivate();
  gazebo_ros::Node::SharedPtr ros_node_;

  gazebo::transport::NodePtr gazebo_node_;
  gazebo::transport::PublisherPtr gz_request_pub_;
  gazebo::transport::PublisherPtr gz_factory_pub_;
  gazebo::transport::PublisherPtr gz_factory_light_pub_;

  /// \brief world pointer from Gazebo
  gazebo::physics::WorldPtr world_;

  /// \brief service to handle requests/responses to spawn models
  rclcpp::Service<gazebo_msgs::srv::SpawnEntity>::SharedPtr spawn_service_;
  // TODO(jrivero): implement the deprecated services with a message and redirection
  rclcpp::Service<gazebo_msgs::srv::SpawnEntity>::SharedPtr
    spawn_deprecated_urdf_;
  rclcpp::Service<gazebo_msgs::srv::SpawnEntity>::SharedPtr
    spawn_deprecated_sdf_;

  gazebo::event::ConnectionPtr world_created_connection_;
};

GazeboRosFactory::GazeboRosFactory()
: impl_(std::make_unique<GazeboRosFactoryPrivate>())
{
}

GazeboRosFactory::~GazeboRosFactory()
{
}

GazeboRosFactoryPrivate::GazeboRosFactoryPrivate()
{
}

void GazeboRosFactory::Load(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

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

  spawn_service_ = ros_node_->create_service<gazebo_msgs::srv::SpawnEntity>("spawn_entity",
      std::bind(&GazeboRosFactoryPrivate::SpawnEntity, this,
      std::placeholders::_1, std::placeholders::_2));

  // Gazebo transport
  gazebo_node_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
  gazebo_node_->Init(_world_name);
  gz_request_pub_ = gazebo_node_->Advertise<gazebo::msgs::Request>("~/request");
  gz_factory_pub_ = gazebo_node_->Advertise<gazebo::msgs::Factory>("~/factory");
  gz_factory_light_pub_ = gazebo_node_->Advertise<gazebo::msgs::Light>("~/factory/light");
}

void GazeboRosFactoryPrivate::SpawnEntity(
  gazebo_msgs::srv::SpawnEntity::Request::SharedPtr req,
  gazebo_msgs::srv::SpawnEntity::Response::SharedPtr res)
{
  // If entity name is given
  std::string name = req->name;

  // get initial pose of model
  auto initial_xyz = Convert<ignition::math::Vector3d>(req->initial_pose.position);

  // get initial roll pitch yaw (fixed frame transform)
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
      RCLCPP_DEBUG(ros_node_->get_logger(),
        "SpawnEntity: reference_frame is empty/world/map, using inertial frame");
    } else {
      res->success = false;
      res->status_message =
        "Reference frame [" + req->reference_frame + "] not found, did you forget to scope "
        "the link by model name?";
      return;
    }
  }

  // incoming robot model string
  std::string xml = req->xml;

  // put string in TiXmlDocument for manipulation
  TiXmlDocument xml_doc;
  xml_doc.Parse(xml.c_str());

  // optional model manipulations: update initial pose && replace model name
  // TODO(louise) Use libsdformat to convert URDF to SDF then handle only the SDF
  TiXmlNode * root_xml;
  if (IsSDF(xml_doc)) {
    updateSDFAttributes(xml_doc, name, initial_xyz, initial_q);

    root_xml = xml_doc.FirstChild("sdf");
  } else if (IsURDF(xml_doc)) {
    updateURDFModelPose(xml_doc, initial_xyz, initial_q);
    updateURDFName(xml_doc, name);

    root_xml = xml_doc.FirstChild("robot");
  } else {
    RCLCPP_ERROR(ros_node_->get_logger(), "SpawnEntity Failure: input xml format not recognized");
    res->success = false;
    res->status_message =
      "SpawnEntity Failure: input xml not SDF or URDF, or cannot be converted to Gazebo "
      "compatible format.";
    return;
  }

  // Walk recursively through the entire file, and add <ros><namespace> tags to all plugins
  auto robot_namespace = req->robot_namespace;
  if (!robot_namespace.empty()) {
    if (root_xml) {
      WalkChildAddRobotNamespace(root_xml, robot_namespace);
    } else {
      RCLCPP_WARN(ros_node_->get_logger(), "Unable to add robot namespace to xml");
    }
  }

  // do spawning check if spawn worked, return response
  return spawnAndConform(xml_doc, name, res);
}

void GazeboRosFactoryPrivate::updateSDFAttributes(
  TiXmlDocument & xml_doc,
  const std::string & name,
  const ignition::math::Vector3d & initial_xyz,
  const ignition::math::Quaterniond & initial_q)
{
  // This is used by both reguar and database SDFs
  TiXmlElement * pose_element;

  // Check SDF for requires SDF element
  TiXmlElement * gazebo_tixml = xml_doc.FirstChildElement("sdf");
  if (!gazebo_tixml) {
    RCLCPP_WARN(ros_node_->get_logger(),
      "Could not find <sdf> element in sdf, so name and initial position cannot be applied");
    return;
  }

  // Check SDF for optional model element. May not have one
  TiXmlElement * model_tixml = gazebo_tixml->FirstChildElement("model");
  if (model_tixml) {
    // Update entity name
    if (model_tixml->Attribute("name") != nullptr) {
      // removing old entity name
      model_tixml->RemoveAttribute("name");
    }
    // replace with user specified name
    model_tixml->SetAttribute("name", name);
  } else {
    // Check SDF for world element
    TiXmlElement * world_tixml = gazebo_tixml->FirstChildElement("world");
    if (!world_tixml) {
      RCLCPP_WARN(
        ros_node_->get_logger(),
        "Could not find <model> or <world> element in sdf, so name and initial position "
        "cannot be applied");
      return;
    }
    // If not <model> element, check SDF for required include element
    model_tixml = world_tixml->FirstChildElement("include");
    if (!model_tixml) {
      RCLCPP_WARN(ros_node_->get_logger(),
        "Could not find <include> element in sdf, so name and initial position "
        "cannot be applied");
      return;
    }

    // Check for name element
    TiXmlElement * name_tixml = model_tixml->FirstChildElement("name");
    if (!name_tixml) {
      // Create the name element
      name_tixml = new TiXmlElement("name");
      model_tixml->LinkEndChild(name_tixml);
    }

    // Set the text within the name element
    TiXmlText * text = new TiXmlText(name);
    name_tixml->LinkEndChild(text);
  }

  // Check for the pose element
  pose_element = model_tixml->FirstChildElement("pose");
  ignition::math::Pose3d model_pose;

  // Create the pose element if it doesn't exist
  // Remove it if it exists, since we are inserting a new one
  if (pose_element) {
    // save pose_element in ignition::math::Pose3d and remove child
    model_pose = parsePose(pose_element->GetText());
    model_tixml->RemoveChild(pose_element);
  }

  // Set and link the pose element after adding initial pose
  {
    // add pose_element Pose to initial pose
    ignition::math::Pose3d new_model_pose = model_pose + ignition::math::Pose3d(initial_xyz,
        initial_q);

    // Create the string of 6 numbers
    std::ostringstream pose_stream;
    ignition::math::Vector3d model_rpy = new_model_pose.Rot().Euler();
    pose_stream << new_model_pose.Pos().X() << " " <<
      new_model_pose.Pos().Y() << " " <<
      new_model_pose.Pos().Z() << " " <<
      model_rpy.X() << " " <<
      model_rpy.Y() << " " <<
      model_rpy.Z();

    // Add value to pose element
    TiXmlText * text = new TiXmlText(pose_stream.str());
    TiXmlElement * new_pose_element = new TiXmlElement("pose");
    new_pose_element->LinkEndChild(text);
    model_tixml->LinkEndChild(new_pose_element);
  }
}

void GazeboRosFactoryPrivate::WalkChildAddRobotNamespace(TiXmlNode * xml,
    const std::string & robot_namespace)
{
  TiXmlNode * child = nullptr;
  while ((child = xml->IterateChildren(child))) {
    // <plugin>
    if (child->Type() == TiXmlNode::TINYXML_ELEMENT &&
      child->ValueStr().compare(std::string("plugin")) == 0)
    {
      // <ros>
      auto ros_elem = child->ToElement()->FirstChildElement("ros");
      if (!ros_elem) {
        ros_elem = new TiXmlElement("ros");
        child->ToElement()->LinkEndChild(ros_elem);
      }

      // <namespace>
      auto ns_elem = ros_elem->FirstChildElement("namespace");
      while (ns_elem) {
        ros_elem->RemoveChild(ns_elem);
        ns_elem = ros_elem->FirstChildElement("namespace");
      }

      ns_elem = new TiXmlElement("namespace");
      auto ns_val = new TiXmlText(robot_namespace);
      ns_elem->LinkEndChild(ns_val);

      ros_elem->ToElement()->LinkEndChild(ns_elem);
    }
    WalkChildAddRobotNamespace(child, robot_namespace);
  }
}

bool GazeboRosFactoryPrivate::IsURDF(const TiXmlDocument & xml_doc)
{
  // FIXME: very crude check
  return xml_doc.FirstChild("robot");
}

bool GazeboRosFactoryPrivate::IsSDF(const TiXmlDocument & xml_doc)
{
  // FIXME: very crude check
  return xml_doc.FirstChild("gazebo") || xml_doc.FirstChild("sdf");
}

void GazeboRosFactoryPrivate::updateURDFModelPose(
  TiXmlDocument & xml_doc,
  const ignition::math::Vector3d & initial_xyz,
  const ignition::math::Quaterniond & initial_q)
{
  TiXmlElement * model_tixml = (xml_doc.FirstChildElement("robot"));

  if (!model_tixml) {
    RCLCPP_WARN(ros_node_->get_logger(),
      "Could not find <robot> element in URDF, so initial position is not applied");
    return;
  }

  // replace initial pose of model
  // find first instance of xyz and rpy, replace with initial pose
  TiXmlElement * origin_key = model_tixml->FirstChildElement("origin");

  if (!origin_key) {
    origin_key = new TiXmlElement("origin");
    model_tixml->LinkEndChild(origin_key);
  }

  ignition::math::Vector3d xyz;
  ignition::math::Vector3d rpy;
  if (origin_key->Attribute("xyz")) {
    xyz = this->parseVector3(origin_key->Attribute("xyz"));
    origin_key->RemoveAttribute("xyz");
  }
  if (origin_key->Attribute("rpy")) {
    rpy = this->parseVector3(origin_key->Attribute("rpy"));
    origin_key->RemoveAttribute("rpy");
  }

  // add xyz, rpy to initial pose
  ignition::math::Pose3d model_pose =
    ignition::math::Pose3d(xyz, ignition::math::Quaterniond(rpy)) +
    ignition::math::Pose3d(initial_xyz, initial_q);

  std::ostringstream xyz_stream;
  xyz_stream << model_pose.Pos().X() << " " << model_pose.Pos().Y() << " " <<
    model_pose.Pos().Z();

  std::ostringstream rpy_stream;
  // convert to Euler angles for Gazebo XML
  ignition::math::Vector3d model_rpy = model_pose.Rot().Euler();
  rpy_stream << model_rpy.X() << " " << model_rpy.Y() << " " << model_rpy.Z();

  origin_key->SetAttribute("xyz", xyz_stream.str());
  origin_key->SetAttribute("rpy", rpy_stream.str());
}

void GazeboRosFactoryPrivate::updateURDFName(
  TiXmlDocument & xml_doc,
  const std::string & name)
{
  TiXmlElement * model_tixml = xml_doc.FirstChildElement("robot");
  // replace model name if one is specified by the user
  if (model_tixml) {
    if (model_tixml->Attribute("name") != nullptr) {
      // removing old model name
      model_tixml->RemoveAttribute("name");
    }
    // replace with user specified name
    model_tixml->SetAttribute("name", name);
  } else {
    RCLCPP_WARN(ros_node_->get_logger(),
      "Could not find <robot> element in URDF, name not replaced");
  }
}

void GazeboRosFactoryPrivate::spawnAndConform(
  TiXmlDocument & xml_doc,
  const std::string & name,
  gazebo_msgs::srv::SpawnEntity::Response::SharedPtr res)
{
  std::string entity_type = xml_doc.RootElement()->FirstChild()->Value();

  // Convert the entity type to lower case
  std::transform(entity_type.begin(), entity_type.end(), entity_type.begin(), ::tolower);

  bool isLight = (entity_type == "light");

  // FIXME: should use entity_info or add lock to World::receiveMutex
  // looking for Model to see if it exists already
  auto entity_info_msg = gazebo::msgs::CreateRequest("entity_info", name);
  gz_request_pub_->Publish(*entity_info_msg, true);

  // TODO(hsu) should wait for response response_sub_, check to see that
  // if _msg->response == "nonexistant"
  auto model = world_->ModelByName(name);
  auto light = world_->LightByName(name);
  if ((isLight && light != nullptr) || (model != nullptr)) {
    RCLCPP_ERROR(ros_node_->get_logger(),
      "SpawnEntity: Failure - model name %s already exist.", name.c_str());
    res->success = false;
    res->status_message = "SpawnEntity: Failure - entity already exists.";
    return;
  }

  // push to factory iface
  std::ostringstream stream;
  stream << xml_doc;
  std::string xml_doc_string = stream.str();

  RCLCPP_DEBUG(ros_node_->get_logger(), "Gazebo Model XML\n\n%s\n\n ", xml_doc_string.c_str());

  // for Gazebo 7 and up, use a different method to spawn lights
  if (isLight) {
    // Publish the light message to spawn the light (Gazebo 7 and up)
    sdf::SDF sdf_light;
    sdf_light.SetFromString(xml_doc_string);
    gazebo::msgs::Light msg = gazebo::msgs::LightFromSDF(sdf_light.Root()->GetElement("light"));
    msg.set_name(name);
    gz_factory_light_pub_->Publish(msg);
  } else {
    // Publish the factory message
    gazebo::msgs::Factory msg;
    gazebo::msgs::Init(msg, "spawn_model");
    msg.set_sdf(xml_doc_string);
    gz_factory_pub_->Publish(msg);
  }
  /// FIXME: should change publish to direct invocation World::LoadModel() and/or
  ///        change the poll for Model existence to common::Events based check.

  /// \brief poll and wait, verify that the model is spawned within Hardcoded 10 seconds
  rclcpp::Duration model_spawn_timeout(10.0);
  rclcpp::Time timeout = ros_node_->now() + model_spawn_timeout;

  while (rclcpp::ok()) {
    if (ros_node_->now() > timeout) {
      res->success = false;
      res->status_message = "SpawnEntity: Entity pushed to spawn queue, but spawn service timed out"
        "waiting for entity to appear in simulation under the name " + name;
      return;
    }

    {
      // boost::recursive_mutex::scoped_lock lock(*world->GetMRMutex());
      if ((isLight && world_->LightByName(name) != nullptr) ||
        (world_->ModelByName(name) != nullptr))
      {
        break;
      }
    }

    auto wait_time = timeout - ros_node_->now();
    double wait_time_d =
      std::chrono::duration<double>(std::chrono::nanoseconds(wait_time.nanoseconds())).count();

    RCLCPP_DEBUG(ros_node_->get_logger(),
      "Waiting for %s for entity %s to spawn", std::to_string(wait_time_d).c_str(), name);
    usleep(2000);
  }

  // set result
  res->success = true;
  res->status_message = "SpawnEntity: Successfully spawned entity";
}

std::vector<double> GazeboRosFactoryPrivate::convertValuesFromStringToDouble(
  const std::string & str)
{
  std::string buf;
  std::vector<std::string> char_values;
  std::vector<double> vals;

  // split text into words
  std::stringstream ss(str);
  while (ss >> buf) {
    char_values.push_back(buf);
  }

  for (auto v : char_values) {
    if (v.empty()) {
      // TODO(jrivero): assert
      RCLCPP_FATAL(ros_node_->get_logger(),
        "Empty char value while parsing pose. This should never happen.");
    }
    try {
      vals.push_back(std::stod(v));
    } catch (std::exception & e) {
      RCLCPP_ERROR(ros_node_->get_logger(),
        "value: '%s' can not be converted to double. Part of the string '%s'",
        std::string(v).c_str(), str.c_str());
      return std::vector<double>();
    }
  }

  return vals;
}

ignition::math::Pose3d GazeboRosFactoryPrivate::parsePose(const std::string & str)
{
  std::vector<double> vals = convertValuesFromStringToDouble(str);

  if (vals.size() != 6) {
    RCLCPP_ERROR(ros_node_->get_logger(),
      "Beware: failed to parse string [%s] as ignition::math::Pose3d, returning zeros.",
      str.c_str());
    return ignition::math::Pose3d();
  }

  return ignition::math::Pose3d(vals[0], vals[1], vals[2], vals[3], vals[4], vals[5]);
}

ignition::math::Vector3d GazeboRosFactoryPrivate::parseVector3(const std::string & str)
{
  std::vector<double> vals = convertValuesFromStringToDouble(str);

  if (vals.size() != 3) {
    RCLCPP_ERROR(
      ros_node_->get_logger(),
      "Beware: failed to parse string [%s] as ignition::math::Vector3d, returning zeros.",
      str.c_str());
    return ignition::math::Vector3d();
  }

  return ignition::math::Vector3d(vals[0], vals[1], vals[2]);
}

GZ_REGISTER_SYSTEM_PLUGIN(GazeboRosFactory)

}  // namespace gazebo_ros
