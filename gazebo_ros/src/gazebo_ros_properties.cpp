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

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Entity.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Light.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Collision.hh>
#include <gazebo/transport/Node.hh>
#include <gazebo_msgs/srv/get_joint_properties.hpp>
#include <gazebo_msgs/srv/get_light_properties.hpp>
#include <gazebo_msgs/srv/get_link_properties.hpp>
#include <gazebo_msgs/srv/get_model_properties.hpp>
#include <gazebo_msgs/srv/get_physics_properties.hpp>
#include <gazebo_msgs/srv/get_world_properties.hpp>
#include <gazebo_msgs/srv/set_joint_properties.hpp>
#include <gazebo_msgs/srv/set_light_properties.hpp>
#include <gazebo_msgs/srv/set_link_properties.hpp>
#include <gazebo_msgs/srv/set_physics_properties.hpp>
#include <gazebo_ros/conversions/geometry_msgs.hpp>
#include <gazebo_ros/node.hpp>

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "gazebo_ros/gazebo_ros_properties.hpp"

namespace gazebo_ros
{

class GazeboRosPropertiesPrivate
{
public:
  /// \brief Callback for get model properties service.
  /// \param[in] req Request
  /// \param[out] res Response
  void GetModelProperties(
    gazebo_msgs::srv::GetModelProperties::Request::SharedPtr _req,
    gazebo_msgs::srv::GetModelProperties::Response::SharedPtr _res);

  /// \brief Callback for get joint properties service.
  /// \param[in] req Request
  /// \param[out] res Response
  void GetJointProperties(
    gazebo_msgs::srv::GetJointProperties::Request::SharedPtr _req,
    gazebo_msgs::srv::GetJointProperties::Response::SharedPtr _res);

  /// \brief Callback for get link properties service.
  /// \param[in] req Request
  /// \param[out] res Response
  void GetLinkProperties(
    gazebo_msgs::srv::GetLinkProperties::Request::SharedPtr _req,
    gazebo_msgs::srv::GetLinkProperties::Response::SharedPtr _res);

  /// \brief Callback for get light properties service.
  /// \param[in] req Request
  /// \param[out] res Response
  void GetLightProperties(
    gazebo_msgs::srv::GetLightProperties::Request::SharedPtr _req,
    gazebo_msgs::srv::GetLightProperties::Response::SharedPtr _res);


  /// \brief Callback for get physics properties service.
  /// \param[in] req Request
  /// \param[out] res Response
  void GetPhysicsProperties(
    gazebo_msgs::srv::GetPhysicsProperties::Request::SharedPtr _req,
    gazebo_msgs::srv::GetPhysicsProperties::Response::SharedPtr _res);

  /// \brief Callback for set joint properties service.
  /// \param[in] req Request
  /// \param[out] res Response
  void SetJointProperties(
    gazebo_msgs::srv::SetJointProperties::Request::SharedPtr _req,
    gazebo_msgs::srv::SetJointProperties::Response::SharedPtr _res);

  /// \brief Callback for set link properties service.
  /// \param[in] req Request
  /// \param[out] res Response
  void SetLinkProperties(
    gazebo_msgs::srv::SetLinkProperties::Request::SharedPtr _req,
    gazebo_msgs::srv::SetLinkProperties::Response::SharedPtr _res);

  /// \brief Callback for set light properties service.
  /// \param[in] req Request
  /// \param[out] res Response
  void SetLightProperties(
    gazebo_msgs::srv::SetLightProperties::Request::SharedPtr _req,
    gazebo_msgs::srv::SetLightProperties::Response::SharedPtr _res);

  /// \brief Callback for set physics properties service.
  /// \param[in] req Request
  /// \param[out] res Response
  void SetPhysicsProperties(
    gazebo_msgs::srv::SetPhysicsProperties::Request::SharedPtr _req,
    gazebo_msgs::srv::SetPhysicsProperties::Response::SharedPtr _res);

  /// \brief Helper to declare floating point parameters
  /// \param[in] name Parameter name
  /// \param[in] descriptor Parameter description
  /// \param[in] default_value Parameter default value
  /// \param[in] min Parameter minimum value
  /// \param[in] max Parameter maximum value
  void DeclareParameter(
    const std::string & name, std::string descriptor, float default_value, float min, float max);

  /// \brief Helper to declare integer parameters
  /// \param[in] name Parameter name
  /// \param[in] descriptor Parameter description
  /// \param[in] default_value Parameter default value
  /// \param[in] min Parameter minimum value
  /// \param[in] max Parameter maximum value
  void DeclareParameter(
    const std::string & name, std::string descriptor, int default_value, int min, int max);

  /// \brief Helper to declare boolean parameters
  /// \param[in] name Parameter name
  /// \param[in] descriptor Parameter description
  /// \param[in] default_value Parameter default value
  void DeclareParameter(
    const std::string & name, std::string descriptor, bool default_value);

  /// \brief World pointer from Gazebo.
  gazebo::physics::WorldPtr world_;

  /// ROS node for communication, managed by gazebo_ros.
  gazebo_ros::Node::SharedPtr ros_node_;

  /// \brief ROS service to handle requests for model properties.
  rclcpp::Service<gazebo_msgs::srv::GetModelProperties>::SharedPtr get_model_properties_service_;

  /// \brief ROS service to handle requests for joint properties.
  rclcpp::Service<gazebo_msgs::srv::GetJointProperties>::SharedPtr get_joint_properties_service_;

  /// \brief ROS service to handle requests for link properties.
  rclcpp::Service<gazebo_msgs::srv::GetLinkProperties>::SharedPtr get_link_properties_service_;

  /// \brief ROS service to handle requests for light properties.
  rclcpp::Service<gazebo_msgs::srv::GetLightProperties>::SharedPtr get_light_properties_service_;

  /// \brief ROS service to handle requests for physics properties.
  rclcpp::Service<gazebo_msgs::srv::GetPhysicsProperties>::SharedPtr
    get_physics_properties_service_;

  /// \brief ROS service to handle requests to set joint properties.
  rclcpp::Service<gazebo_msgs::srv::SetJointProperties>::SharedPtr set_joint_properties_service_;

  /// \brief ROS service to handle requests to set link properties.
  rclcpp::Service<gazebo_msgs::srv::SetLinkProperties>::SharedPtr set_link_properties_service_;

  /// \brief ROS service to handle requests to set light properties.
  rclcpp::Service<gazebo_msgs::srv::SetLightProperties>::SharedPtr set_light_properties_service_;

  /// \brief ROS service to handle requests to set physics properties.
  rclcpp::Service<gazebo_msgs::srv::SetPhysicsProperties>::SharedPtr
    set_physics_properties_service_;

  /// Gazebo node for communication.
  gazebo::transport::NodePtr gz_node_;

  /// Publishes light modify messages.
  gazebo::transport::PublisherPtr gz_properties_light_pub_;

  /// Store Gazebo Gravity
  ignition::math::Vector3d gravity_;
};

GazeboRosProperties::GazeboRosProperties()
: impl_(std::make_unique<GazeboRosPropertiesPrivate>())
{
}

GazeboRosProperties::~GazeboRosProperties()
{
}

void GazeboRosProperties::Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
  impl_->world_ = _world;

  impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);

  impl_->get_model_properties_service_ =
    impl_->ros_node_->create_service<gazebo_msgs::srv::GetModelProperties>(
    "get_model_properties", std::bind(&GazeboRosPropertiesPrivate::GetModelProperties, impl_.get(),
    std::placeholders::_1, std::placeholders::_2));

  impl_->get_joint_properties_service_ =
    impl_->ros_node_->create_service<gazebo_msgs::srv::GetJointProperties>(
    "get_joint_properties", std::bind(&GazeboRosPropertiesPrivate::GetJointProperties, impl_.get(),
    std::placeholders::_1, std::placeholders::_2));

  impl_->get_link_properties_service_ =
    impl_->ros_node_->create_service<gazebo_msgs::srv::GetLinkProperties>(
    "get_link_properties", std::bind(&GazeboRosPropertiesPrivate::GetLinkProperties, impl_.get(),
    std::placeholders::_1, std::placeholders::_2));

  impl_->get_light_properties_service_ =
    impl_->ros_node_->create_service<gazebo_msgs::srv::GetLightProperties>(
    "get_light_properties", std::bind(&GazeboRosPropertiesPrivate::GetLightProperties, impl_.get(),
    std::placeholders::_1, std::placeholders::_2));

  impl_->get_physics_properties_service_ =
    impl_->ros_node_->create_service<gazebo_msgs::srv::GetPhysicsProperties>(
    "get_physics_properties",
    std::bind(&GazeboRosPropertiesPrivate::GetPhysicsProperties, impl_.get(),
    std::placeholders::_1, std::placeholders::_2));

  impl_->set_joint_properties_service_ =
    impl_->ros_node_->create_service<gazebo_msgs::srv::SetJointProperties>(
    "set_joint_properties", std::bind(&GazeboRosPropertiesPrivate::SetJointProperties, impl_.get(),
    std::placeholders::_1, std::placeholders::_2));

  impl_->set_link_properties_service_ =
    impl_->ros_node_->create_service<gazebo_msgs::srv::SetLinkProperties>(
    "set_link_properties", std::bind(&GazeboRosPropertiesPrivate::SetLinkProperties, impl_.get(),
    std::placeholders::_1, std::placeholders::_2));

  impl_->set_light_properties_service_ =
    impl_->ros_node_->create_service<gazebo_msgs::srv::SetLightProperties>(
    "set_light_properties", std::bind(&GazeboRosPropertiesPrivate::SetLightProperties, impl_.get(),
    std::placeholders::_1, std::placeholders::_2));

  impl_->set_physics_properties_service_ =
    impl_->ros_node_->create_service<gazebo_msgs::srv::SetPhysicsProperties>(
    "set_physics_properties",
    std::bind(&GazeboRosPropertiesPrivate::SetPhysicsProperties, impl_.get(),
    std::placeholders::_1, std::placeholders::_2));

  // Gazebo transport
  impl_->gz_node_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
  impl_->gz_node_->Init(_world->Name());
  impl_->gz_properties_light_pub_ =
    impl_->gz_node_->Advertise<gazebo::msgs::Light>("~/light/modify");
  impl_->gravity_ = impl_->world_->Gravity();

  std::string description;
  description = "Simulation world time step size in seconds,"
    "smaller time steps produces lower, more stable simulation.";
  impl_->DeclareParameter("time_step", description, 0.001f, 0.0f, 10.0f);

  description = "Simulator max update rate, -1 unlimited, 1 restricts to real-time if possible.";
  impl_->DeclareParameter("max_update_rate", description, 60.0f, 0.0f, 1000.0f);

  description = "Simulated gravity in the x direction.";
  impl_->DeclareParameter("gravity_x", description, 0.0f, -100.0f, 100.0f);

  description = "Simulated gravity in the y direction.";
  impl_->DeclareParameter("gravity_y", description, 0.0f, -100.0f, 100.0f);

  description = "Simulated gravity in the z direction.";
  impl_->DeclareParameter("gravity_z", description, -9.8f, -100.0f, 100.0f);

  description = "Auto disable of links in simulation if link is not moving.";
  impl_->DeclareParameter("auto_disable_links", description, false);

  description = "Number of preconditioning iterations for SOR PGS LCP as implemented in quickstep.";
  impl_->DeclareParameter("sor_pgs_precon_iters", description, 0, 0, 10000);

  description = "Number of iterations for SOR PGS LCP as implemented in quickstep.";
  impl_->DeclareParameter("sor_pgs_iters", description, 20, 0, 10000);

  description =
    "Relaxation parameter for SOR PGS LCP, usually set to 1.3, but reduce to stabilize simulation.";
  impl_->DeclareParameter("sor_pgs_w", description, 1.3f, 0.0f, 5.0f);

  description = "The number of scans to skip between each measured scan";
  impl_->DeclareParameter("sor_pgs_rms_error_tol", description, -1, -1, 10000);

  description = "Constraint Force Mixing per ODE's users manual.";
  impl_->DeclareParameter("cfm", description, 0.0f, 0.0f, 10.0f);

  description = "Error Reduction Parameter per ODE's users manual.";
  impl_->DeclareParameter("erp", description, 0.2f, 0.0f, 10.0f);

  description = "Margin for penetration for which restorative forces are not applied.";
  impl_->DeclareParameter("contact_surface_layer", description, 0.001f, 0.0f, 10.0f);

  description = "Maximum contact penetration correction velocity.";
  impl_->DeclareParameter("contact_max_correcting_vel", description, 100.0f, 0.0f, 10000000.0f);

  description = "Maximum number of contacts between any 2 links.";
  impl_->DeclareParameter("max_contacts", description, 100.0f, 0.0f, 10000000.0f);

  auto existing_callback = impl_->ros_node_->set_on_parameters_set_callback(nullptr);
  auto param_change_callback =
    [this, existing_callback](std::vector<rclcpp::Parameter> parameters) {
      auto result = rcl_interfaces::msg::SetParametersResult();
      if (nullptr != existing_callback) {
        result = existing_callback(parameters);
        if (!result.successful) {
          return result;
        }
      }
      bool is_paused = impl_->world_->IsPaused();
      impl_->world_->SetPaused(true);

      gazebo::physics::PhysicsEnginePtr pe = impl_->world_->Physics();
      for (auto & parameter : parameters) {
        const auto & name = parameter.get_name();
        if (name == "time_step") {
          pe->SetMaxStepSize(parameter.as_double());
        } else if (name == "max_update_rate") {
          pe->SetRealTimeUpdateRate(parameter.as_double());
        }
        if (pe->GetType() == "ode") {
          // stuff only works in ODE right now
          if (name == "auto_disable_bodies") {
            pe->SetAutoDisableFlag(parameter.as_bool());
          } else if (name == "sor_pgs_precon_iters") {
            pe->SetParam("precon_iters", parameter.as_int());
          } else if (name == "sor_pgs_iters") {
            pe->SetParam("iters", parameter.as_int());
          } else if (name == "sor_pgs_w") {
            pe->SetParam("sor", parameter.as_double());
          } else if (name == "cfm") {
            pe->SetParam("cfm", parameter.as_double());
          } else if (name == "erp") {
            pe->SetParam("erp", parameter.as_double());
          } else if (name == "contact_surface_layer") {
            pe->SetParam("contact_surface_layer", parameter.as_double());
          } else if (name == "contact_max_correcting_vel") {
            pe->SetParam("contact_max_correcting_vel", parameter.as_double());
          } else if (name == "max_contacts") {
            pe->SetParam("max_contacts", parameter.as_int());
          } else if (name == "gravity_x" || name == "gravity_y" || name == "gravity_z") {
            if (name == "gravity_x") {
              impl_->gravity_.X() = parameter.as_double();
            } else if (name == "gravity_y") {
              impl_->gravity_.Y() = parameter.as_double();
            } else {
              impl_->gravity_.Z() = parameter.as_double();
            }
            pe->SetGravity(impl_->gravity_);
          }
          impl_->world_->SetPaused(is_paused);
          result.successful = true;
        } else {
          /// \TODO: add support for simbody, dart and bullet physics properties.
          RCLCPP_ERROR(impl_->ros_node_->get_logger(),
            "Setting physics properties is not supported for physics engine [%s].",
            pe->GetType().c_str());
          result.successful = false;
        }
      }
      return result;
    };

  impl_->ros_node_->set_on_parameters_set_callback(param_change_callback);
}

void GazeboRosPropertiesPrivate::GetModelProperties(
  gazebo_msgs::srv::GetModelProperties::Request::SharedPtr _req,
  gazebo_msgs::srv::GetModelProperties::Response::SharedPtr _res)
{
  gazebo::physics::ModelPtr model = world_->ModelByName(_req->model_name);
  if (!model) {
    RCLCPP_ERROR(
      ros_node_->get_logger(), "GetModelProperties: model [%s] does not exist",
      _req->model_name.c_str());
    _res->success = false;
    _res->status_message = "GetModelProperties: model does not exist";
    return;
  }
  // get model parent name
  gazebo::physics::ModelPtr parent_model =
    boost::dynamic_pointer_cast<gazebo::physics::Model>(model->GetParent());
  if (parent_model) {_res->parent_model_name = parent_model->GetName();}

  // get list of child links, collisions
  _res->body_names.clear();
  _res->geom_names.clear();
  for (unsigned int i = 0; i < model->GetChildCount(); ++i) {
    gazebo::physics::LinkPtr link =
      boost::dynamic_pointer_cast<gazebo::physics::Link>(model->GetChild(i));
    if (link) {
      if (link->IsCanonicalLink()) {
        _res->canonical_body_name = link->GetScopedName();
      }
      _res->body_names.push_back(link->GetScopedName());
      // get list of collisions
      for (unsigned int j = 0; j < link->GetChildCount(); j++) {
        gazebo::physics::CollisionPtr collision =
          boost::dynamic_pointer_cast<gazebo::physics::Collision>(link->GetChild(j));
        if (collision) {
          _res->geom_names.push_back(collision->GetName());
        }
      }
    }
  }

  // get list of joints
  _res->joint_names.clear();

  gazebo::physics::Joint_V joints = model->GetJoints();
  for (auto & joint : joints) {
    _res->joint_names.push_back(joint->GetName());
  }

  // get children model names
  _res->child_model_names.clear();
  for (unsigned int j = 0; j < model->GetChildCount(); j++) {
    gazebo::physics::ModelPtr child_model =
      boost::dynamic_pointer_cast<gazebo::physics::Model>(model->GetChild(j));
    if (child_model) {
      _res->child_model_names.push_back(child_model->GetName() );
    }
  }

  // is model static
  _res->is_static = model->IsStatic();

  _res->success = true;
  _res->status_message = "GetModelProperties: got properties";
}

void GazeboRosPropertiesPrivate::GetJointProperties(
  gazebo_msgs::srv::GetJointProperties::Request::SharedPtr _req,
  gazebo_msgs::srv::GetJointProperties::Response::SharedPtr _res)
{
  gazebo::physics::JointPtr joint;
  for (unsigned int i = 0; i < world_->ModelCount(); ++i) {
    joint = world_->ModelByIndex(i)->GetJoint(_req->joint_name);
    if (joint) {
      break;
    }
  }

  if (!joint) {
    _res->success = false;
    _res->status_message = "GetJointProperties: joint not found";
    return;
  }

  auto type = joint->GetMsgType();
  if (type == gazebo::msgs::Joint::REVOLUTE) {
    _res->type = _res->REVOLUTE;
  } else if (type == gazebo::msgs::Joint::PRISMATIC) {
    _res->type = _res->PRISMATIC;
  } else if (type == gazebo::msgs::Joint::UNIVERSAL) {
    _res->type = _res->UNIVERSAL;
  } else if (type == gazebo::msgs::Joint::BALL) {
    _res->type = _res->BALL;
  } else if (type == gazebo::msgs::Joint::FIXED) {
    _res->type = _res->FIXED;
  } else {
    RCLCPP_WARN(ros_node_->get_logger(), "Partial support for joint type [%i]", type);
  }

  _res->damping.clear();
  _res->position.clear();
  _res->rate.clear();
  for (auto i = 0u; i < joint->DOF(); ++i) {
    _res->damping.push_back(joint->GetDamping(i));
    _res->position.push_back(joint->Position(i));
    _res->rate.push_back(joint->GetVelocity(i));
  }

  _res->success = true;
  _res->status_message = "GetJointProperties: got properties";
}

void GazeboRosPropertiesPrivate::GetLinkProperties(
  gazebo_msgs::srv::GetLinkProperties::Request::SharedPtr _req,
  gazebo_msgs::srv::GetLinkProperties::Response::SharedPtr _res)
{
  gazebo::physics::LinkPtr link =
    boost::dynamic_pointer_cast<gazebo::physics::Link>(world_->EntityByName(_req->link_name));
  if (!link) {
    _res->success = false;
    _res->status_message =
      "GetLinkProperties: link not found, did you forget to scope the link by model name?";
  } else {
    _res->gravity_mode = link->GetGravityMode();

    gazebo::physics::InertialPtr inertia = link->GetInertial();

    _res->mass = link->GetInertial()->Mass();

    _res->ixx = inertia->IXX();
    _res->iyy = inertia->IYY();
    _res->izz = inertia->IZZ();
    _res->ixy = inertia->IXY();
    _res->ixz = inertia->IXZ();
    _res->iyz = inertia->IYZ();

    auto com = link->GetInertial()->Pose();

    _res->com.position = gazebo_ros::Convert<geometry_msgs::msg::Point>(com.Pos());
    _res->com.orientation = gazebo_ros::Convert<geometry_msgs::msg::Quaternion>(com.Rot());

    _res->success = true;
    _res->status_message = "GetLinkProperties: got properties";
  }
}

void GazeboRosPropertiesPrivate::GetLightProperties(
  gazebo_msgs::srv::GetLightProperties::Request::SharedPtr _req,
  gazebo_msgs::srv::GetLightProperties::Response::SharedPtr _res)
{
  gazebo::physics::LightPtr phy_light = world_->LightByName(_req->light_name);
  if (phy_light == nullptr) {
    _res->success = false;
    _res->status_message = "getLightProperties: Requested light " + _req->light_name +
      " not found!";
  } else {
    gazebo::msgs::Light light;
    phy_light->FillMsg(light);

    _res->diffuse.r = light.diffuse().r();
    _res->diffuse.g = light.diffuse().g();
    _res->diffuse.b = light.diffuse().b();
    _res->diffuse.a = light.diffuse().a();

    _res->attenuation_constant = light.attenuation_constant();
    _res->attenuation_linear = light.attenuation_linear();
    _res->attenuation_quadratic = light.attenuation_quadratic();

    _res->success = true;
  }
}

void GazeboRosPropertiesPrivate::GetPhysicsProperties(
  gazebo_msgs::srv::GetPhysicsProperties::Request::SharedPtr /*_req*/,
  gazebo_msgs::srv::GetPhysicsProperties::Response::SharedPtr _res)
{
  gazebo::physics::PhysicsEnginePtr pe = (world_->Physics());
  _res->time_step = pe->GetMaxStepSize();
  _res->pause = world_->IsPaused();
  _res->max_update_rate = pe->GetRealTimeUpdateRate();
  _res->gravity = gazebo_ros::Convert<geometry_msgs::msg::Vector3>(world_->Gravity());

  // Only ODE supported
  if (pe->GetType() == "ode") {
    _res->ode_config.auto_disable_bodies = pe->GetAutoDisableFlag();
    _res->ode_config.sor_pgs_precon_iters = boost::any_cast<int>(pe->GetParam("precon_iters"));
    _res->ode_config.sor_pgs_iters = boost::any_cast<int>(pe->GetParam("iters"));
    _res->ode_config.sor_pgs_w = boost::any_cast<double>(pe->GetParam("sor"));
    _res->ode_config.contact_surface_layer =
      boost::any_cast<double>(pe->GetParam("contact_surface_layer"));
    _res->ode_config.contact_max_correcting_vel =
      boost::any_cast<double>(pe->GetParam("contact_max_correcting_vel"));
    _res->ode_config.cfm = boost::any_cast<double>(pe->GetParam("cfm"));
    _res->ode_config.erp = boost::any_cast<double>(pe->GetParam("erp"));
    _res->ode_config.max_contacts = boost::any_cast<int>(pe->GetParam("max_contacts"));
    _res->success = true;
    _res->status_message = "GetPhysicsProperties: got properties";
    return;
  }

  RCLCPP_ERROR(ros_node_->get_logger(),
    "get_physics_properties service call does not yet support physics engine [%s].",
    pe->GetType().c_str());
  _res->success = false;
  _res->status_message = "Physics engine [" + pe->GetType() + "]: not supported.";
}

void GazeboRosPropertiesPrivate::SetJointProperties(
  gazebo_msgs::srv::SetJointProperties::Request::SharedPtr _req,
  gazebo_msgs::srv::SetJointProperties::Response::SharedPtr _res)
{
  /// @todo: current settings only allows for setting of 1DOF joints
  /// (e.g. HingeJoint and SliderJoint) correctly.
  gazebo::physics::JointPtr joint;
  for (unsigned int i = 0; i < world_->ModelCount(); ++i) {
    joint = world_->ModelByIndex(i)->GetJoint(_req->joint_name);
    if (joint) {break;}
  }

  if (!joint) {
    _res->success = false;
    _res->status_message = "SetJointProperties: joint not found";
  } else {
    for (unsigned int i = 0; i < _req->ode_joint_config.damping.size(); ++i) {
      joint->SetDamping(i, _req->ode_joint_config.damping[i]);
    }
    for (unsigned int i = 0; i < _req->ode_joint_config.hi_stop.size(); ++i) {
      joint->SetParam("hi_stop", i, _req->ode_joint_config.hi_stop[i]);
    }
    for (unsigned int i = 0; i < _req->ode_joint_config.lo_stop.size(); ++i) {
      joint->SetParam("lo_stop", i, _req->ode_joint_config.lo_stop[i]);
    }
    for (unsigned int i = 0; i < _req->ode_joint_config.erp.size(); ++i) {
      joint->SetParam("erp", i, _req->ode_joint_config.erp[i]);
    }
    for (unsigned int i = 0; i < _req->ode_joint_config.cfm.size(); ++i) {
      joint->SetParam("cfm", i, _req->ode_joint_config.cfm[i]);
    }
    for (unsigned int i = 0; i < _req->ode_joint_config.stop_erp.size(); ++i) {
      joint->SetParam("stop_erp", i, _req->ode_joint_config.stop_erp[i]);
    }
    for (unsigned int i = 0; i < _req->ode_joint_config.stop_cfm.size(); ++i) {
      joint->SetParam("stop_cfm", i, _req->ode_joint_config.stop_cfm[i]);
    }
    for (unsigned int i = 0; i < _req->ode_joint_config.fudge_factor.size(); ++i) {
      joint->SetParam("fudge_factor", i, _req->ode_joint_config.fudge_factor[i]);
    }
    for (unsigned int i = 0; i < _req->ode_joint_config.fmax.size(); ++i) {
      joint->SetParam("fmax", i, _req->ode_joint_config.fmax[i]);
    }
    for (unsigned int i = 0; i < _req->ode_joint_config.vel.size(); ++i) {
      joint->SetParam("vel", i, _req->ode_joint_config.vel[i]);
    }

    _res->success = true;
    _res->status_message = "SetJointProperties: properties set";
  }
}

void GazeboRosPropertiesPrivate::SetLinkProperties(
  gazebo_msgs::srv::SetLinkProperties::Request::SharedPtr _req,
  gazebo_msgs::srv::SetLinkProperties::Response::SharedPtr _res)
{
  gazebo::physics::LinkPtr link =
    boost::dynamic_pointer_cast<gazebo::physics::Link>(world_->EntityByName(_req->link_name));
  if (!link) {
    _res->success = false;
    _res->status_message =
      "SetLinkProperties: link not found, did you forget to scope the link by model name?";
  } else {
    gazebo::physics::InertialPtr mass = link->GetInertial();
    mass->SetCoG(gazebo_ros::Convert<ignition::math::Pose3d>(_req->com));
    mass->SetInertiaMatrix(_req->ixx, _req->iyy, _req->izz, _req->ixy, _req->ixz, _req->iyz);
    mass->SetMass(_req->mass);
    link->SetGravityMode(_req->gravity_mode);

    _res->success = true;
    _res->status_message = "SetLinkProperties: properties set";
  }
}

void GazeboRosPropertiesPrivate::SetLightProperties(
  gazebo_msgs::srv::SetLightProperties::Request::SharedPtr _req,
  gazebo_msgs::srv::SetLightProperties::Response::SharedPtr _res)
{
  gazebo::physics::LightPtr phy_light = world_->LightByName(_req->light_name);
  if (phy_light == nullptr) {
    _res->success = false;
    _res->status_message = "setLightProperties: Requested light " + _req->light_name +
      " not found!";
  } else {
    gazebo::msgs::Light light;

    phy_light->FillMsg(light);

    light.mutable_diffuse()->set_r(_req->diffuse.r);
    light.mutable_diffuse()->set_g(_req->diffuse.g);
    light.mutable_diffuse()->set_b(_req->diffuse.b);
    light.mutable_diffuse()->set_a(_req->diffuse.a);

    light.set_attenuation_constant(_req->attenuation_constant);
    light.set_attenuation_linear(_req->attenuation_linear);
    light.set_attenuation_quadratic(_req->attenuation_quadratic);

    gz_properties_light_pub_->Publish(light);

    _res->success = true;
  }
}

void GazeboRosPropertiesPrivate::SetPhysicsProperties(
  gazebo_msgs::srv::SetPhysicsProperties::Request::SharedPtr _req,
  gazebo_msgs::srv::SetPhysicsProperties::Response::SharedPtr _res)
{
  // pause simulation if requested
  bool is_paused = world_->IsPaused();
  world_->SetPaused(true);
  world_->SetGravity(gazebo_ros::Convert<ignition::math::Vector3d>(_req->gravity));

  gazebo::physics::PhysicsEnginePtr pe = (world_->Physics());
  pe->SetMaxStepSize(_req->time_step);
  pe->SetRealTimeUpdateRate(_req->max_update_rate);

  if (pe->GetType() == "ode") {
    // Only ODE supported
    pe->SetAutoDisableFlag(_req->ode_config.auto_disable_bodies);
    pe->SetParam("precon_iters", static_cast<int>(_req->ode_config.sor_pgs_precon_iters));
    pe->SetParam("iters", static_cast<int>(_req->ode_config.sor_pgs_iters));
    pe->SetParam("sor", _req->ode_config.sor_pgs_w);
    pe->SetParam("cfm", _req->ode_config.cfm);
    pe->SetParam("erp", _req->ode_config.erp);
    pe->SetParam("contact_surface_layer", _req->ode_config.contact_surface_layer);
    pe->SetParam("contact_max_correcting_vel", _req->ode_config.contact_max_correcting_vel);
    pe->SetParam("max_contacts", static_cast<int>(_req->ode_config.max_contacts));

    world_->SetPaused(is_paused);

    _res->success = true;
    _res->status_message = "physics engine updated";
    return;
  }
  RCLCPP_ERROR(ros_node_->get_logger(),
    "set_physics_properties service call does not yet support physics engine [%s].",
    pe->GetType().c_str());
  _res->success = false;
  _res->status_message = "Physics engine [" + pe->GetType() + "] not supported.";
}

void GazeboRosPropertiesPrivate::DeclareParameter(
  const std::string & name, std::string descriptor, float default_value, float min, float max)
{
  rcl_interfaces::msg::ParameterDescriptor parameter_descriptor;
  parameter_descriptor.description = std::move(descriptor);
  parameter_descriptor.floating_point_range.resize(1);
  auto & floating_point_range = parameter_descriptor.floating_point_range.at(0);
  floating_point_range.from_value = min;
  floating_point_range.to_value = max;
  ros_node_->declare_parameter(name, default_value, parameter_descriptor);
}

void GazeboRosPropertiesPrivate::DeclareParameter(
  const std::string & name, std::string descriptor, int default_value, int min, int max)
{
  rcl_interfaces::msg::ParameterDescriptor parameter_descriptor;
  parameter_descriptor.description = std::move(descriptor);
  parameter_descriptor.integer_range.resize(1);
  auto & integer_range = parameter_descriptor.integer_range.at(0);
  integer_range.from_value = min;
  integer_range.to_value = max;
  ros_node_->declare_parameter(name, default_value, parameter_descriptor);
}


void GazeboRosPropertiesPrivate::DeclareParameter(
  const std::string & name, std::string descriptor, bool default_value)
{
  rcl_interfaces::msg::ParameterDescriptor parameter_descriptor;
  parameter_descriptor.description = std::move(descriptor);
  ros_node_->declare_parameter(name, default_value, parameter_descriptor);
}

GZ_REGISTER_WORLD_PLUGIN(GazeboRosProperties)

}  // namespace gazebo_ros
