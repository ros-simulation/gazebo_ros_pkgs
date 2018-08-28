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

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo_msgs/srv/spawn_model.hpp>
#include <gazebo_ros/conversions.hpp>
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
  GazeboRosFactoryPrivate();
  gazebo_ros::Node::SharedPtr ros_node_;
  std::string robot_namespace_;

  gazebo::transport::NodePtr gazebo_node_;
  gazebo::transport::PublisherPtr gz_request_pub_;
  gazebo::transport::PublisherPtr gz_factory_pub_;
  gazebo::transport::PublisherPtr gz_factory_light_pub_;

  /// \brief world pointer from Gazebo
  gazebo::physics::WorldPtr world_;

  /// \brief service to handle requests/responses to spawn models
  rclcpp::Service<gazebo_msgs::srv::SpawnModel>::SharedPtr spawn_service_;
  // TODO(jrivero): implement the deprecated services with a message and redirection
  rclcpp::Service<gazebo_msgs::srv::SpawnModel>::SharedPtr
    spawn_deprecated_urdf_;
  rclcpp::Service<gazebo_msgs::srv::SpawnModel>::SharedPtr
    spawn_deprecated_sdf_;

  /// \brief Function for inserting a model into Gazebo from ROS Service Call.
  /// Supported formats are SDF and URDF
  void spawnModel(
    gazebo_msgs::srv::SpawnModel::Request::SharedPtr req,
    gazebo_msgs::srv::SpawnModel::Response::SharedPtr res);

private:
  /// \brief utility for checking if string is in URDF format
  bool isURDF(std::string model_xml);

  /// \brief utility for checking if string is in SDF format
  bool isSDF(std::string model_xml);

  /// \brief replace or update initial pose and model name
  ///
  /// This function can handle both regular SDF files and <include> SDFs that are used with the
  /// Gazebo Model Database
  void updateSDFAttributes(
    TiXmlDocument & gazebo_model_xml,
    const std::string & model_name,
    const ignition::math::Vector3d & initial_xyz,
    const ignition::math::Quaterniond & initial_q);

  void updateURDFModelPose(
    TiXmlDocument & gazebo_model_xml,
    const ignition::math::Vector3d & initial_xyz,
    const ignition::math::Quaterniond & initial_q);

  void updateURDFName(
    TiXmlDocument & gazebo_model_xml,
    const std::string & model_name);

  void walkChildAddRobotNamespace(TiXmlNode * model_xml);

  /// \brief remove the XML declaration
  void stripXmlDeclaration(std::string & model_xml);

  /// \brief convert xml to Pose
  ignition::math::Pose3d parsePose(const std::string & str);

  /// \brief convert xml to Pose
  ignition::math::Vector3d parseVector3(const std::string & str);

  std::vector<double> convertValuesFromStringToDouble(const std::string & str);

  void spawnAndConform(
    TiXmlDocument & gazebo_model_xml,
    const std::string & model_name,
    gazebo_msgs::srv::SpawnModel::Response::SharedPtr res);
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
  // Factoryialize ROS with arguments
  rclcpp::init(argc, argv);
  impl_->ros_node_ = gazebo_ros::Node::Create("gazebo_ros_factory");
  impl_->spawn_service_ = impl_->ros_node_->create_service<gazebo_msgs::srv::SpawnModel>(
    "spawn_model",
    std::bind(&GazeboRosFactoryPrivate::spawnModel,
    impl_.get(),
    std::placeholders::_1, std::placeholders::_2));

  // Initialize gazebo mechanisms
  impl_->world_ = gazebo::physics::get_world();
  impl_->gazebo_node_->Init(impl_->world_->Name());
  impl_->gz_request_pub_ = impl_->gazebo_node_->Advertise<gazebo::msgs::Request>("~/request");
  impl_->gz_factory_pub_ = impl_->gazebo_node_->Advertise<gazebo::msgs::Factory>("~/factory");
  impl_->gz_factory_light_pub_ = impl_->gazebo_node_->Advertise<gazebo::msgs::Light>(
    "~/factory/light");
}

void GazeboRosFactoryPrivate::spawnModel(
  gazebo_msgs::srv::SpawnModel::Request::SharedPtr req,
  gazebo_msgs::srv::SpawnModel::Response::SharedPtr res)
{
  // incoming entity name
  std::string model_name = req->model_name;

  // get namespace for the corresponding model plugins
  robot_namespace_ = req->robot_namespace;

  // get initial pose of model
  ignition::math::Vector3d initial_xyz(req->initial_pose.position.x,
    req->initial_pose.position.y,
    req->initial_pose.position.z);

  // get initial roll pitch yaw (fixed frame transform)
  ignition::math::Quaterniond initial_q(req->initial_pose.orientation.w,
    req->initial_pose.orientation.x,
    req->initial_pose.orientation.y,
    req->initial_pose.orientation.z);

  // reference frame for initial pose definition, modify initial pose if defined
  gazebo::physics::EntityPtr frame = world_->EntityByName(req->reference_frame);
  if (frame) {
    // convert to relative pose
    ignition::math::Pose3d frame_pose = frame->WorldPose();
    initial_xyz = frame_pose.Pos() + frame_pose.Rot().RotateVector(initial_xyz);
    initial_q = frame_pose.Rot() * initial_q;
    /// @todo: map is really wrong, need to use tf here somehow
  } else {
    if (req->reference_frame == "" || req->reference_frame == "world" ||
      req->reference_frame == "map" || req->reference_frame == "/map")
    {
      RCLCPP_DEBUG(ros_node_->get_logger(),
        "SpawnModel: reference_frame is empty/world/map, using inertial frame");
    } else {
      res->success = false;
      res->status_message =
        "SpawnModel: reference reference_frame not found, did you forget to scope "
        "the link by model name?";
      return;
    }
  }

  // incoming robot model string
  std::string model_xml = req->model_xml;
  // store resulting Gazebo Model XML to be sent to spawn queue
  // get incoming string containg either an URDF or a Gazebo Model XML
  // grab from parameter server if necessary convert to SDF if necessary
  stripXmlDeclaration(model_xml);
  // put string in TiXmlDocument for manipulation
  TiXmlDocument gazebo_model_xml;
  gazebo_model_xml.Parse(model_xml.c_str());

  // optional model manipulations: update initial pose && replace model name
  if (isSDF(model_xml)) {
    updateSDFAttributes(gazebo_model_xml, model_name, initial_xyz, initial_q);

    // Walk recursively through the entire SDF, locate plugin tags and
    // add robotNamespace as a child with the correct namespace
    if (!robot_namespace_.empty()) {
      // Get root element for SDF
      // TODO(hsu): implement the spawning also with <light></light> and <model></model>
      TiXmlNode * model_tixml = gazebo_model_xml.FirstChild("sdf");
      model_tixml = (!model_tixml) ?
        gazebo_model_xml.FirstChild("gazebo") : model_tixml;
      if (model_tixml) {
        walkChildAddRobotNamespace(model_tixml);
      } else {
        RCLCPP_WARN(ros_node_->get_logger(),
          "Unable to add robot namespace to xml");
      }
    }
  } else if (isURDF(model_xml)) {
    updateURDFModelPose(gazebo_model_xml, initial_xyz, initial_q);
    updateURDFName(gazebo_model_xml, model_name);

    // Walk recursively through the entire URDF, locate plugin tags and
    // add robotNamespace as a child with the correct namespace
    if (!this->robot_namespace_.empty()) {
      // Get root element for URDF
      TiXmlNode * model_tixml = gazebo_model_xml.FirstChild("robot");
      if (model_tixml) {
        walkChildAddRobotNamespace(model_tixml);
      } else {
        RCLCPP_WARN(ros_node_->get_logger(), "Unable to add robot namespace to xml");
      }
    }
  } else {
    RCLCPP_ERROR(ros_node_->get_logger(), "SpawnModel Failure: input xml format not recognized");
    res->success = false;
    res->status_message =
      "SpawnModel Failure: input model_xml not SDF or URDF, or cannot be converted to Gazebo "
      "compatible format.";
    return;
  }

  // do spawning check if spawn worked, return response
  return spawnAndConform(gazebo_model_xml, model_name, res);
}

void GazeboRosFactoryPrivate::updateSDFAttributes(
  TiXmlDocument & gazebo_model_xml,
  const std::string & model_name,
  const ignition::math::Vector3d & initial_xyz,
  const ignition::math::Quaterniond & initial_q)
{
  TiXmlElement * pose_element;  // This is used by both reguar and database SDFs

  // Check SDF for requires SDF element
  TiXmlElement * gazebo_tixml = gazebo_model_xml.FirstChildElement("sdf");
  if (!gazebo_tixml) {
    RCLCPP_WARN(ros_node_->get_logger(),
      "Could not find <sdf> element in sdf, so name and initial position cannot be applied");
    return;
  }

  // Check SDF for optional model element. May not have one
  TiXmlElement * model_tixml = gazebo_tixml->FirstChildElement("model");
  if (model_tixml) {
    // Update entity name
    if (model_tixml->Attribute("name") != NULL) {
      // removing old entity name
      model_tixml->RemoveAttribute("name");
    }
    // replace with user specified name
    model_tixml->SetAttribute("name", model_name);
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
    TiXmlText * text = new TiXmlText(model_name);
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

void GazeboRosFactoryPrivate::walkChildAddRobotNamespace(TiXmlNode * model_xml)
{
  TiXmlNode * child = 0;
  child = model_xml->IterateChildren(child);
  while (child != NULL) {
    if (child->Type() == TiXmlNode::TINYXML_ELEMENT &&
      child->ValueStr().compare(std::string("plugin")) == 0)
    {
      if (child->FirstChildElement("robotNamespace") == NULL) {
        TiXmlElement * child_elem = child->ToElement()->FirstChildElement("robotNamespace");
        while (child_elem) {
          child->ToElement()->RemoveChild(child_elem);
          child_elem = child->ToElement()->FirstChildElement("robotNamespace");
        }
        TiXmlElement * key = new TiXmlElement("robotNamespace");
        TiXmlText * val = new TiXmlText(robot_namespace_);
        key->LinkEndChild(val);
        child->ToElement()->LinkEndChild(key);
      }
    }
    walkChildAddRobotNamespace(child);
    child = model_xml->IterateChildren(child);
  }
}

bool GazeboRosFactoryPrivate::isURDF(std::string model_xml)
{
  TiXmlDocument doc_in;
  doc_in.Parse(model_xml.c_str());
  if (doc_in.FirstChild("robot")) {
    return true;
  }

  return false;
}

bool GazeboRosFactoryPrivate::isSDF(std::string model_xml)
{
  // FIXME: very crude check
  TiXmlDocument doc_in;
  doc_in.Parse(model_xml.c_str());
  if (doc_in.FirstChild("gazebo") ||
    doc_in.FirstChild("sdf"))
  {
    return true;
  }

  return false;
}

void GazeboRosFactoryPrivate::stripXmlDeclaration(std::string & model_xml)
{
  // incoming robot model string is a string containing a Gazebo Model XML
  /// STRIP DECLARATION <? ... xml version="1.0" ... ?> from model_xml
  /// @todo: does tinyxml have functionality for this?
  /// @todo: should gazebo take care of the declaration?
  std::string open_bracket("<?");
  std::string close_bracket("?>");
  size_t pos1 = model_xml.find(open_bracket, 0);
  size_t pos2 = model_xml.find(close_bracket, 0);
  if (pos1 != std::string::npos && pos2 != std::string::npos) {
    model_xml.replace(pos1, pos2 - pos1 + 2, std::string(""));
  }
}

void GazeboRosFactoryPrivate::updateURDFModelPose(
  TiXmlDocument & gazebo_model_xml,
  const ignition::math::Vector3d & initial_xyz,
  const ignition::math::Quaterniond & initial_q)
{
  TiXmlElement * model_tixml = (gazebo_model_xml.FirstChildElement("robot"));
  if (model_tixml) {
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
  } else {
    RCLCPP_WARN(ros_node_->get_logger(),
      "Could not find <model> element in sdf, so name and initial position is not applied");
  }
}

void GazeboRosFactoryPrivate::updateURDFName(
  TiXmlDocument & gazebo_model_xml,
  const std::string & model_name)
{
  TiXmlElement * model_tixml = gazebo_model_xml.FirstChildElement("robot");
  // replace model name if one is specified by the user
  if (model_tixml) {
    if (model_tixml->Attribute("name") != NULL) {
      // removing old model name
      model_tixml->RemoveAttribute("name");
    }
    // replace with user specified name
    model_tixml->SetAttribute("name", model_name);
  } else {
    RCLCPP_WARN(ros_node_->get_logger(),
      "Could not find <robot> element in URDF, name not replaced");
  }
}

void GazeboRosFactoryPrivate::spawnAndConform(
  TiXmlDocument & gazebo_model_xml,
  const std::string & model_name,
  gazebo_msgs::srv::SpawnModel::Response::SharedPtr res)
{
  std::string entity_type = gazebo_model_xml.RootElement()->FirstChild()->Value();
  // Convert the entity type to lower case
  std::transform(entity_type.begin(), entity_type.end(), entity_type.begin(), ::tolower);

  bool isLight = (entity_type == "light");

  // push to factory iface
  std::ostringstream stream;
  stream << gazebo_model_xml;
  std::string gazebo_model_xml_string = stream.str();

  RCLCPP_DEBUG(ros_node_->get_logger(),
    "Gazebo Model XML\n\n%s\n\n ", gazebo_model_xml_string.c_str());

  // publish to factory topic
  gazebo::msgs::Factory msg;
  gazebo::msgs::Init(msg, "spawn_model");
  msg.set_sdf(gazebo_model_xml_string);

  // FIXME: should use entity_info or add lock to World::receiveMutex
  // looking for Model to see if it exists already
  gazebo::msgs::Request * entity_info_msg =
    gazebo::msgs::CreateRequest("entity_info", model_name);
  gz_request_pub_->Publish(*entity_info_msg, true);
  // TODO(hsu) should wait for response response_sub_, check to see that
  // if _msg->response == "nonexistant"
  gazebo::physics::ModelPtr model = world_->ModelByName(model_name);
  gazebo::physics::LightPtr light = world_->LightByName(model_name);
  if ((isLight && light != NULL) || (model != NULL)) {
    RCLCPP_ERROR(ros_node_->get_logger(),
      "SpawnModel: Failure - model name %s already exist.", model_name.c_str());
    res->success = false;
    res->status_message = "SpawnModel: Failure - entity already exists.";
    return;
  }

  // for Gazebo 7 and up, use a different method to spawn lights
  if (isLight) {
    // Publish the light message to spawn the light (Gazebo 7 and up)
    sdf::SDF sdf_light;
    sdf_light.SetFromString(gazebo_model_xml_string);
    gazebo::msgs::Light msg = gazebo::msgs::LightFromSDF(sdf_light.Root()->GetElement("light"));
    msg.set_name(model_name);
    gz_factory_light_pub_->Publish(msg);
  } else {
    // Publish the factory message
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
      res->status_message = "SpawnModel: Entity pushed to spawn queue, but spawn service timed out"
        "waiting for entity to appear in simulation under the name " + model_name;
      return;
    }

    {
      // boost::recursive_mutex::scoped_lock lock(*world->GetMRMutex());
      if ((isLight && world_->LightByName(model_name) != NULL) ||
        (world_->ModelByName(model_name) != NULL))
      {
        break;
      }
    }

    auto wait_time = timeout - ros_node_->now();
    double wait_time_d =
      std::chrono::duration<double>(std::chrono::nanoseconds(wait_time.nanoseconds())).count();

    RCLCPP_DEBUG(ros_node_->get_logger(),
      "Waiting for %s for entity %s to spawn", std::to_string(wait_time_d).c_str(), model_name);
    usleep(2000);
  }

  // set result
  res->success = true;
  res->status_message = "SpawnModel: Successfully spawned entity";
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
