/*
 * Copyright 2013 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

/* Desc: External interfaces for Gazebo
 * Author: Nate Koenig, John Hsu, Dave Coleman
 * Date: Jun 10 2013
 */

#include <gazebo/common/Events.hh>
#include <gazebo/gazebo_config.h>
#include <gazebo_ros/gazebo_ros_api_plugin.h>

namespace gazebo
{

GazeboRosApiPlugin::GazeboRosApiPlugin() :
  physics_reconfigure_initialized_(false),
  world_created_(false),
  stop_(false),
  plugin_loaded_(false),
{
}

GazeboRosApiPlugin::~GazeboRosApiPlugin()
{
  ROS_DEBUG_STREAM_NAMED("api_plugin","GazeboRosApiPlugin Deconstructor start");

  // Unload the sigint event
  sigint_event_.reset();
  ROS_DEBUG_STREAM_NAMED("api_plugin","After sigint_event unload");

  // Don't attempt to unload this plugin if it was never loaded in the Load() function
  if(!plugin_loaded_)
  {
    ROS_DEBUG_STREAM_NAMED("api_plugin","Deconstructor skipped because never loaded");
    return;
  }

  // Disconnect slots
  load_gazebo_ros_api_plugin_event_.reset();
  time_update_event_.reset();
  ROS_DEBUG_STREAM_NAMED("api_plugin","Slots disconnected");

  // Stop the multi threaded ROS spinner
  async_ros_spin_->stop();
  ROS_DEBUG_STREAM_NAMED("api_plugin","Async ROS Spin Stopped");

  // Shutdown the ROS node
  nh_->shutdown();
  ROS_DEBUG_STREAM_NAMED("api_plugin","Node Handle Shutdown");

  // Shutdown ROS queue
  gazebo_callback_queue_thread_->join();
  ROS_DEBUG_STREAM_NAMED("api_plugin","Callback Queue Joined");

  // Physics Dynamic Reconfigure
  physics_reconfigure_thread_->join();
  ROS_DEBUG_STREAM_NAMED("api_plugin","Physics reconfigure joined");

  ROS_DEBUG_STREAM_NAMED("api_plugin","Unloaded");
}

void GazeboRosApiPlugin::shutdownSignal()
{
  ROS_DEBUG_STREAM_NAMED("api_plugin","shutdownSignal() recieved");
  stop_ = true;
}

void GazeboRosApiPlugin::Load(int argc, char** argv)
{
  ROS_DEBUG_STREAM_NAMED("api_plugin","Load");

  // connect to sigint event
  sigint_event_ = gazebo::event::Events::ConnectSigInt(boost::bind(&GazeboRosApiPlugin::shutdownSignal,this));

  // setup ros related
  if (!ros::isInitialized())
    ros::init(argc,argv,"gazebo",ros::init_options::NoSigintHandler);
  else
    ROS_ERROR_NAMED("api_plugin", "Something other than this gazebo_ros_api plugin started ros::init(...), command line arguments may not be parsed properly.");

  // check if the ros master is available - required
  while(!ros::master::check())
  {
    ROS_WARN_STREAM_NAMED("api_plugin","No ROS master - start roscore to continue...");
    // wait 0.5 second
    usleep(500*1000); // can't use ROS Time here b/c node handle is not yet initialized

    if(stop_)
    {
      ROS_WARN_STREAM_NAMED("api_plugin","Canceled loading Gazebo ROS API plugin by sigint event");
      return;
    }
  }

  nh_.reset(new ros::NodeHandle("~")); // advertise topics and services in this node's namespace

  // Built-in multi-threaded ROS spinning
  async_ros_spin_.reset(new ros::AsyncSpinner(0)); // will use a thread for each CPU core
  async_ros_spin_->start();

  /// \brief setup custom callback queue
  gazebo_callback_queue_thread_.reset(new boost::thread( &GazeboRosApiPlugin::gazeboQueueThread, this) );

  /// \brief start a thread for the physics dynamic reconfigure node
  physics_reconfigure_thread_.reset(new boost::thread(boost::bind(&GazeboRosApiPlugin::physicsReconfigureThread, this)));

  // below needs the world to be created first
  load_gazebo_ros_api_plugin_event_ = gazebo::event::Events::ConnectWorldCreated(boost::bind(&GazeboRosApiPlugin::loadGazeboRosApiPlugin,this,_1));

  plugin_loaded_ = true;
  ROS_INFO_NAMED("api_plugin", "Finished loading Gazebo ROS API Plugin.");
}

void GazeboRosApiPlugin::loadGazeboRosApiPlugin(std::string world_name)
{
  // make sure things are only called once
  lock_.lock();
  if (world_created_)
  {
    lock_.unlock();
    return;
  }

  // set flag to true and load this plugin
  world_created_ = true;
  lock_.unlock();

  world_ = gazebo::physics::get_world(world_name);
  if (!world_)
  {
    //ROS_ERROR_NAMED("api_plugin", "world name: [%s]",world->Name().c_str());
    // connect helper function to signal for scheduling torque/forces, etc
    ROS_FATAL_NAMED("api_plugin", "cannot load gazebo ros api server plugin, physics::get_world() fails to return world");
    return;
  }

  gazebonode_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
  gazebonode_->Init(world_name);
}

void GazeboRosApiPlugin::gazeboQueueThread()
{
  static const double timeout = 0.001;
  while (nh_->ok())
  {
    gazebo_queue_.callAvailable(ros::WallDuration(timeout));
  }
}

void GazeboRosApiPlugin::advertiseServices()
{
  // Advertise more services on the custom queue
  std::string get_physics_properties_service_name("get_physics_properties");
  ros::AdvertiseServiceOptions get_physics_properties_aso =
    ros::AdvertiseServiceOptions::create<gazebo_msgs::GetPhysicsProperties>(
                                                                            get_physics_properties_service_name,
                                                                            boost::bind(&GazeboRosApiPlugin::getPhysicsProperties,this,_1,_2),
                                                                            ros::VoidPtr(), &gazebo_queue_);
  get_physics_properties_service_ = nh_->advertiseService(get_physics_properties_aso);

  // Advertise more services on the custom queue
  std::string set_physics_properties_service_name("set_physics_properties");
  ros::AdvertiseServiceOptions set_physics_properties_aso =
    ros::AdvertiseServiceOptions::create<gazebo_msgs::SetPhysicsProperties>(
                                                                            set_physics_properties_service_name,
                                                                            boost::bind(&GazeboRosApiPlugin::setPhysicsProperties,this,_1,_2),
                                                                            ros::VoidPtr(), &gazebo_queue_);
  set_physics_properties_service_ = nh_->advertiseService(set_physics_properties_aso);

  // Advertise more services on the custom queue
  std::string set_model_configuration_service_name("set_model_configuration");
  ros::AdvertiseServiceOptions set_model_configuration_aso =
    ros::AdvertiseServiceOptions::create<gazebo_msgs::SetModelConfiguration>(
                                                                             set_model_configuration_service_name,
                                                                             boost::bind(&GazeboRosApiPlugin::setModelConfiguration,this,_1,_2),
                                                                             ros::VoidPtr(), &gazebo_queue_);
  set_model_configuration_service_ = nh_->advertiseService(set_model_configuration_aso);
}

bool GazeboRosApiPlugin::setPhysicsProperties(gazebo_msgs::SetPhysicsProperties::Request &req,
                                              gazebo_msgs::SetPhysicsProperties::Response &res)
{
  // pause simulation if requested
  bool is_paused = world_->IsPaused();
  world_->SetPaused(true);
  world_->SetGravity(ignition::math::Vector3d(req.gravity.x,req.gravity.y,req.gravity.z));

  // supported updates
#if GAZEBO_MAJOR_VERSION >= 8
  gazebo::physics::PhysicsEnginePtr pe = (world_->Physics());
#else
  gazebo::physics::PhysicsEnginePtr pe = (world_->GetPhysicsEngine());
#endif
  pe->SetMaxStepSize(req.time_step);
  pe->SetRealTimeUpdateRate(req.max_update_rate);

  if (pe->GetType() == "ode")
  {
    // stuff only works in ODE right now
    pe->SetAutoDisableFlag(req.ode_config.auto_disable_bodies);
    pe->SetParam("precon_iters", int(req.ode_config.sor_pgs_precon_iters));
    pe->SetParam("iters", int(req.ode_config.sor_pgs_iters));
    pe->SetParam("sor", req.ode_config.sor_pgs_w);
    pe->SetParam("cfm", req.ode_config.cfm);
    pe->SetParam("erp", req.ode_config.erp);
    pe->SetParam("contact_surface_layer",
        req.ode_config.contact_surface_layer);
    pe->SetParam("contact_max_correcting_vel",
        req.ode_config.contact_max_correcting_vel);
    pe->SetParam("max_contacts", int(req.ode_config.max_contacts));

    world_->SetPaused(is_paused);

    res.success = true;
    res.status_message = "physics engine updated";
  }
  else
  {
    /// \TODO: add support for simbody, dart and bullet physics properties.
    ROS_ERROR_NAMED("api_plugin", "ROS set_physics_properties service call does not yet support physics engine [%s].", pe->GetType().c_str());
    res.success = false;
    res.status_message = "Physics engine [" + pe->GetType() + "]: set_physics_properties not supported.";
  }
  return res.success;
}

bool GazeboRosApiPlugin::getPhysicsProperties(gazebo_msgs::GetPhysicsProperties::Request &req,
                                              gazebo_msgs::GetPhysicsProperties::Response &res)
{
  // supported updates
#if GAZEBO_MAJOR_VERSION >= 8
  gazebo::physics::PhysicsEnginePtr pe = (world_->Physics());
#else
  gazebo::physics::PhysicsEnginePtr pe = (world_->GetPhysicsEngine());
#endif
  res.time_step = pe->GetMaxStepSize();
  res.pause = world_->IsPaused();
  res.max_update_rate = pe->GetRealTimeUpdateRate();
  ignition::math::Vector3d gravity = world_->Gravity();
  res.gravity.x = gravity.X();
  res.gravity.y = gravity.Y();
  res.gravity.z = gravity.Z();

  // stuff only works in ODE right now
  if (pe->GetType() == "ode")
  {
    res.ode_config.auto_disable_bodies =
      pe->GetAutoDisableFlag();
    res.ode_config.sor_pgs_precon_iters = boost::any_cast<int>(
      pe->GetParam("precon_iters"));
    res.ode_config.sor_pgs_iters = boost::any_cast<int>(
        pe->GetParam("iters"));
    res.ode_config.sor_pgs_w = boost::any_cast<double>(
        pe->GetParam("sor"));
    res.ode_config.contact_surface_layer = boost::any_cast<double>(
      pe->GetParam("contact_surface_layer"));
    res.ode_config.contact_max_correcting_vel = boost::any_cast<double>(
      pe->GetParam("contact_max_correcting_vel"));
    res.ode_config.cfm = boost::any_cast<double>(
        pe->GetParam("cfm"));
    res.ode_config.erp = boost::any_cast<double>(
        pe->GetParam("erp"));
    res.ode_config.max_contacts = boost::any_cast<int>(
      pe->GetParam("max_contacts"));

    res.success = true;
    res.status_message = "GetPhysicsProperties: got properties";
  }
  else
  {
    /// \TODO: add support for simbody, dart and bullet physics properties.
    ROS_ERROR_NAMED("api_plugin", "ROS get_physics_properties service call does not yet support physics engine [%s].", pe->GetType().c_str());
    res.success = false;
    res.status_message = "Physics engine [" + pe->GetType() + "]: get_physics_properties not supported.";
  }
  return res.success;
}

bool GazeboRosApiPlugin::setModelConfiguration(gazebo_msgs::SetModelConfiguration::Request &req,
                                               gazebo_msgs::SetModelConfiguration::Response &res)
{
  std::string gazebo_model_name = req.model_name;

  // search for model with name
#if GAZEBO_MAJOR_VERSION >= 8
  gazebo::physics::ModelPtr gazebo_model = world_->ModelByName(req.model_name);
#else
  gazebo::physics::ModelPtr gazebo_model = world_->GetModel(req.model_name);
#endif
  if (!gazebo_model)
  {
    ROS_ERROR_NAMED("api_plugin", "SetModelConfiguration: model [%s] does not exist",gazebo_model_name.c_str());
    res.success = false;
    res.status_message = "SetModelConfiguration: model does not exist";
    return true;
  }

  if (req.joint_names.size() == req.joint_positions.size())
  {
    std::map<std::string, double> joint_position_map;
    for (unsigned int i = 0; i < req.joint_names.size(); i++)
    {
      joint_position_map[req.joint_names[i]] = req.joint_positions[i];
    }

    // make the service call to pause gazebo
    bool is_paused = world_->IsPaused();
    if (!is_paused) world_->SetPaused(true);

    gazebo_model->SetJointPositions(joint_position_map);

    // resume paused state before this call
    world_->SetPaused(is_paused);

    res.success = true;
    res.status_message = "SetModelConfiguration: success";
    return true;
  }
  else
  {
    res.success = false;
    res.status_message = "SetModelConfiguration: joint name and position list have different lengths";
    return true;
  }
}

void GazeboRosApiPlugin::physicsReconfigureCallback(gazebo_ros::PhysicsConfig &config, uint32_t level)
{
  if (!physics_reconfigure_initialized_)
  {
    gazebo_msgs::GetPhysicsProperties srv;
    physics_reconfigure_get_client_.call(srv);

    config.time_step                   = srv.response.time_step;
    config.max_update_rate             = srv.response.max_update_rate;
    config.gravity_x                   = srv.response.gravity.x;
    config.gravity_y                   = srv.response.gravity.y;
    config.gravity_z                   = srv.response.gravity.z;
    config.auto_disable_bodies         = srv.response.ode_config.auto_disable_bodies;
    config.sor_pgs_precon_iters        = srv.response.ode_config.sor_pgs_precon_iters;
    config.sor_pgs_iters               = srv.response.ode_config.sor_pgs_iters;
    config.sor_pgs_rms_error_tol       = srv.response.ode_config.sor_pgs_rms_error_tol;
    config.sor_pgs_w                   = srv.response.ode_config.sor_pgs_w;
    config.contact_surface_layer       = srv.response.ode_config.contact_surface_layer;
    config.contact_max_correcting_vel  = srv.response.ode_config.contact_max_correcting_vel;
    config.cfm                         = srv.response.ode_config.cfm;
    config.erp                         = srv.response.ode_config.erp;
    config.max_contacts                = srv.response.ode_config.max_contacts;
    physics_reconfigure_initialized_ = true;
  }
  else
  {
    bool changed = false;
    gazebo_msgs::GetPhysicsProperties srv;
    physics_reconfigure_get_client_.call(srv);

    // check for changes
    if (config.time_step                      != srv.response.time_step)                                 changed = true;
    if (config.max_update_rate                != srv.response.max_update_rate)                           changed = true;
    if (config.gravity_x                      != srv.response.gravity.x)                                 changed = true;
    if (config.gravity_y                      != srv.response.gravity.y)                                 changed = true;
    if (config.gravity_z                      != srv.response.gravity.z)                                 changed = true;
    if (config.auto_disable_bodies            != srv.response.ode_config.auto_disable_bodies)            changed = true;
    if ((uint32_t)config.sor_pgs_precon_iters != srv.response.ode_config.sor_pgs_precon_iters)           changed = true;
    if ((uint32_t)config.sor_pgs_iters        != srv.response.ode_config.sor_pgs_iters)                  changed = true;
    if (config.sor_pgs_rms_error_tol          != srv.response.ode_config.sor_pgs_rms_error_tol)          changed = true;
    if (config.sor_pgs_w                      != srv.response.ode_config.sor_pgs_w)                      changed = true;
    if (config.contact_surface_layer          != srv.response.ode_config.contact_surface_layer)          changed = true;
    if (config.contact_max_correcting_vel     != srv.response.ode_config.contact_max_correcting_vel)     changed = true;
    if (config.cfm                            != srv.response.ode_config.cfm)                            changed = true;
    if (config.erp                            != srv.response.ode_config.erp)                            changed = true;
    if ((uint32_t)config.max_contacts         != srv.response.ode_config.max_contacts)                   changed = true;

    if (changed)
    {
      // pause simulation if requested
      gazebo_msgs::SetPhysicsProperties srv;
      srv.request.time_step                             = config.time_step                   ;
      srv.request.max_update_rate                       = config.max_update_rate             ;
      srv.request.gravity.x                             = config.gravity_x                   ;
      srv.request.gravity.y                             = config.gravity_y                   ;
      srv.request.gravity.z                             = config.gravity_z                   ;
      srv.request.ode_config.auto_disable_bodies        = config.auto_disable_bodies         ;
      srv.request.ode_config.sor_pgs_precon_iters       = config.sor_pgs_precon_iters        ;
      srv.request.ode_config.sor_pgs_iters              = config.sor_pgs_iters               ;
      srv.request.ode_config.sor_pgs_rms_error_tol      = config.sor_pgs_rms_error_tol       ;
      srv.request.ode_config.sor_pgs_w                  = config.sor_pgs_w                   ;
      srv.request.ode_config.contact_surface_layer      = config.contact_surface_layer       ;
      srv.request.ode_config.contact_max_correcting_vel = config.contact_max_correcting_vel  ;
      srv.request.ode_config.cfm                        = config.cfm                         ;
      srv.request.ode_config.erp                        = config.erp                         ;
      srv.request.ode_config.max_contacts               = config.max_contacts                ;
      physics_reconfigure_set_client_.call(srv);
      ROS_INFO_NAMED("api_plugin", "physics dynamics reconfigure update complete");
    }
    ROS_INFO_NAMED("api_plugin", "physics dynamics reconfigure complete");
  }
}

void GazeboRosApiPlugin::physicsReconfigureThread()
{
  physics_reconfigure_set_client_ = nh_->serviceClient<gazebo_msgs::SetPhysicsProperties>("set_physics_properties");
  physics_reconfigure_get_client_ = nh_->serviceClient<gazebo_msgs::GetPhysicsProperties>("get_physics_properties");

  // Wait until the rest of this plugin is loaded and the services are being offered
  physics_reconfigure_set_client_.waitForExistence();
  physics_reconfigure_get_client_.waitForExistence();

  physics_reconfigure_srv_.reset(new dynamic_reconfigure::Server<gazebo_ros::PhysicsConfig>());

  physics_reconfigure_callback_ = boost::bind(&GazeboRosApiPlugin::physicsReconfigureCallback, this, _1, _2);
  physics_reconfigure_srv_->setCallback(physics_reconfigure_callback_);

  ROS_INFO_NAMED("api_plugin", "Physics dynamic reconfigure ready.");
}

// Register this plugin with the simulator
GZ_REGISTER_SYSTEM_PLUGIN(GazeboRosApiPlugin)
}
