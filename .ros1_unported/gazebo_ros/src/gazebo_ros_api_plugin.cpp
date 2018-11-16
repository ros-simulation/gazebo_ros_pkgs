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
  pub_link_states_connection_count_(0),
  pub_model_states_connection_count_(0),
  pub_clock_frequency_(0),
  enable_ros_network_(true)
{
  robot_namespace_.clear();
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
  wrench_update_event_.reset();
  force_update_event_.reset();
  time_update_event_.reset();
  ROS_DEBUG_STREAM_NAMED("api_plugin","Slots disconnected");

  if (pub_link_states_connection_count_ > 0) // disconnect if there are subscribers on exit
    pub_link_states_event_.reset();
  if (pub_model_states_connection_count_ > 0) // disconnect if there are subscribers on exit
    pub_model_states_event_.reset();
  ROS_DEBUG_STREAM_NAMED("api_plugin","Disconnected World Updates");

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

  // Delete Force and Wrench Jobs
  lock_.lock();
  for (std::vector<GazeboRosApiPlugin::ForceJointJob*>::iterator iter=force_joint_jobs_.begin();iter!=force_joint_jobs_.end();)
  {
    delete (*iter);
    iter = force_joint_jobs_.erase(iter);
  }
  force_joint_jobs_.clear();
  ROS_DEBUG_STREAM_NAMED("api_plugin","ForceJointJobs deleted");
  for (std::vector<GazeboRosApiPlugin::WrenchBodyJob*>::iterator iter=wrench_body_jobs_.begin();iter!=wrench_body_jobs_.end();)
  {
    delete (*iter);
    iter = wrench_body_jobs_.erase(iter);
  }
  wrench_body_jobs_.clear();
  lock_.unlock();
  ROS_DEBUG_STREAM_NAMED("api_plugin","WrenchBodyJobs deleted");

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

  nh_->getParam("enable_ros_network", enable_ros_network_);

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
  //stat_sub_ = gazebonode_->Subscribe("~/world_stats", &GazeboRosApiPlugin::publishSimTime, this); // TODO: does not work in server plugin?
  light_modify_pub_ = gazebonode_->Advertise<gazebo::msgs::Light>("~/light/modify");

  // reset topic connection counts
  pub_link_states_connection_count_ = 0;
  pub_model_states_connection_count_ = 0;

  /// \brief advertise all services
  if (enable_ros_network_)
    advertiseServices();

  // Manage clock for simulated ros time
  pub_clock_ = nh_->advertise<rosgraph_msgs::Clock>("/clock",10);
  // set param for use_sim_time if not set by user already
  if(!(nh_->hasParam("/use_sim_time")))
    nh_->setParam("/use_sim_time", true);

  nh_->getParam("pub_clock_frequency", pub_clock_frequency_);
#if GAZEBO_MAJOR_VERSION >= 8
  last_pub_clock_time_ = world_->SimTime();
#else
  last_pub_clock_time_ = world_->GetSimTime();
#endif

  // hooks for applying forces, publishing simtime on /clock
  wrench_update_event_ = gazebo::event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboRosApiPlugin::wrenchBodySchedulerSlot,this));
  force_update_event_  = gazebo::event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboRosApiPlugin::forceJointSchedulerSlot,this));
  time_update_event_   = gazebo::event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboRosApiPlugin::publishSimTime,this));
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
  if (! enable_ros_network_)
  {
    ROS_INFO_NAMED("api_plugin", "ROS gazebo topics/services are disabled");
    return;
  }

  pub_clock_ = nh_->advertise<rosgraph_msgs::Clock>("/clock",10);

  // Advertise more services on the custom queue
  std::string get_model_properties_service_name("get_model_properties");
  ros::AdvertiseServiceOptions get_model_properties_aso =
    ros::AdvertiseServiceOptions::create<gazebo_msgs::GetModelProperties>(
                                                                          get_model_properties_service_name,
                                                                          boost::bind(&GazeboRosApiPlugin::getModelProperties,this,_1,_2),
                                                                          ros::VoidPtr(), &gazebo_queue_);
  get_model_properties_service_ = nh_->advertiseService(get_model_properties_aso);

  // Advertise more services on the custom queue
  std::string get_world_properties_service_name("get_world_properties");
  ros::AdvertiseServiceOptions get_world_properties_aso =
    ros::AdvertiseServiceOptions::create<gazebo_msgs::GetWorldProperties>(
                                                                          get_world_properties_service_name,
                                                                          boost::bind(&GazeboRosApiPlugin::getWorldProperties,this,_1,_2),
                                                                          ros::VoidPtr(), &gazebo_queue_);
  get_world_properties_service_ = nh_->advertiseService(get_world_properties_aso);

  // Advertise more services on the custom queue
  std::string get_joint_properties_service_name("get_joint_properties");
  ros::AdvertiseServiceOptions get_joint_properties_aso =
    ros::AdvertiseServiceOptions::create<gazebo_msgs::GetJointProperties>(
                                                                          get_joint_properties_service_name,
                                                                          boost::bind(&GazeboRosApiPlugin::getJointProperties,this,_1,_2),
                                                                          ros::VoidPtr(), &gazebo_queue_);
  get_joint_properties_service_ = nh_->advertiseService(get_joint_properties_aso);

  // Advertise more services on the custom queue
  std::string get_link_properties_service_name("get_link_properties");
  ros::AdvertiseServiceOptions get_link_properties_aso =
    ros::AdvertiseServiceOptions::create<gazebo_msgs::GetLinkProperties>(
                                                                         get_link_properties_service_name,
                                                                         boost::bind(&GazeboRosApiPlugin::getLinkProperties,this,_1,_2),
                                                                         ros::VoidPtr(), &gazebo_queue_);
  get_link_properties_service_ = nh_->advertiseService(get_link_properties_aso);

  // Advertise more services on the custom queue
  std::string get_light_properties_service_name("get_light_properties");
  ros::AdvertiseServiceOptions get_light_properties_aso =
    ros::AdvertiseServiceOptions::create<gazebo_msgs::GetLightProperties>(
                                                                          get_light_properties_service_name,
                                                                          boost::bind(&GazeboRosApiPlugin::getLightProperties,this,_1,_2),
                                                                          ros::VoidPtr(), &gazebo_queue_);
  get_light_properties_service_ = nh_->advertiseService(get_light_properties_aso);

  // Advertise more services on the custom queue
  std::string set_light_properties_service_name("set_light_properties");
  ros::AdvertiseServiceOptions set_light_properties_aso =
    ros::AdvertiseServiceOptions::create<gazebo_msgs::SetLightProperties>(
                                                                          set_light_properties_service_name,
                                                                          boost::bind(&GazeboRosApiPlugin::setLightProperties,this,_1,_2),
                                                                          ros::VoidPtr(), &gazebo_queue_);
  set_light_properties_service_ = nh_->advertiseService(set_light_properties_aso);

  // Advertise more services on the custom queue
  std::string get_physics_properties_service_name("get_physics_properties");
  ros::AdvertiseServiceOptions get_physics_properties_aso =
    ros::AdvertiseServiceOptions::create<gazebo_msgs::GetPhysicsProperties>(
                                                                            get_physics_properties_service_name,
                                                                            boost::bind(&GazeboRosApiPlugin::getPhysicsProperties,this,_1,_2),
                                                                            ros::VoidPtr(), &gazebo_queue_);
  get_physics_properties_service_ = nh_->advertiseService(get_physics_properties_aso);

  // publish complete link states in world frame
  ros::AdvertiseOptions pub_link_states_ao =
    ros::AdvertiseOptions::create<gazebo_msgs::LinkStates>(
                                                           "link_states",10,
                                                           boost::bind(&GazeboRosApiPlugin::onLinkStatesConnect,this),
                                                           boost::bind(&GazeboRosApiPlugin::onLinkStatesDisconnect,this),
                                                           ros::VoidPtr(), &gazebo_queue_);
  pub_link_states_ = nh_->advertise(pub_link_states_ao);

  // publish complete model states in world frame
  ros::AdvertiseOptions pub_model_states_ao =
    ros::AdvertiseOptions::create<gazebo_msgs::ModelStates>(
                                                            "model_states",10,
                                                            boost::bind(&GazeboRosApiPlugin::onModelStatesConnect,this),
                                                            boost::bind(&GazeboRosApiPlugin::onModelStatesDisconnect,this),
                                                            ros::VoidPtr(), &gazebo_queue_);
  pub_model_states_ = nh_->advertise(pub_model_states_ao);

  // Advertise more services on the custom queue
  std::string set_link_properties_service_name("set_link_properties");
  ros::AdvertiseServiceOptions set_link_properties_aso =
    ros::AdvertiseServiceOptions::create<gazebo_msgs::SetLinkProperties>(
                                                                         set_link_properties_service_name,
                                                                         boost::bind(&GazeboRosApiPlugin::setLinkProperties,this,_1,_2),
                                                                         ros::VoidPtr(), &gazebo_queue_);
  set_link_properties_service_ = nh_->advertiseService(set_link_properties_aso);

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

  // Advertise more services on the custom queue
  std::string set_joint_properties_service_name("set_joint_properties");
  ros::AdvertiseServiceOptions set_joint_properties_aso =
    ros::AdvertiseServiceOptions::create<gazebo_msgs::SetJointProperties>(
                                                                          set_joint_properties_service_name,
                                                                          boost::bind(&GazeboRosApiPlugin::setJointProperties,this,_1,_2),
                                                                          ros::VoidPtr(), &gazebo_queue_);
  set_joint_properties_service_ = nh_->advertiseService(set_joint_properties_aso);

  // Advertise more services on the custom queue
  std::string pause_physics_service_name("pause_physics");
  ros::AdvertiseServiceOptions pause_physics_aso =
    ros::AdvertiseServiceOptions::create<std_srvs::Empty>(
                                                          pause_physics_service_name,
                                                          boost::bind(&GazeboRosApiPlugin::pausePhysics,this,_1,_2),
                                                          ros::VoidPtr(), &gazebo_queue_);
  pause_physics_service_ = nh_->advertiseService(pause_physics_aso);

  // Advertise more services on the custom queue
  std::string unpause_physics_service_name("unpause_physics");
  ros::AdvertiseServiceOptions unpause_physics_aso =
    ros::AdvertiseServiceOptions::create<std_srvs::Empty>(
                                                          unpause_physics_service_name,
                                                          boost::bind(&GazeboRosApiPlugin::unpausePhysics,this,_1,_2),
                                                          ros::VoidPtr(), &gazebo_queue_);
  unpause_physics_service_ = nh_->advertiseService(unpause_physics_aso);

  // Advertise more services on the custom queue
  std::string apply_body_wrench_service_name("apply_body_wrench");
  ros::AdvertiseServiceOptions apply_body_wrench_aso =
    ros::AdvertiseServiceOptions::create<gazebo_msgs::ApplyBodyWrench>(
                                                                       apply_body_wrench_service_name,
                                                                       boost::bind(&GazeboRosApiPlugin::applyBodyWrench,this,_1,_2),
                                                                       ros::VoidPtr(), &gazebo_queue_);
  apply_body_wrench_service_ = nh_->advertiseService(apply_body_wrench_aso);

  // Advertise more services on the custom queue
  std::string apply_joint_effort_service_name("apply_joint_effort");
  ros::AdvertiseServiceOptions apply_joint_effort_aso =
    ros::AdvertiseServiceOptions::create<gazebo_msgs::ApplyJointEffort>(
                                                                        apply_joint_effort_service_name,
                                                                        boost::bind(&GazeboRosApiPlugin::applyJointEffort,this,_1,_2),
                                                                        ros::VoidPtr(), &gazebo_queue_);
  apply_joint_effort_service_ = nh_->advertiseService(apply_joint_effort_aso);

  // Advertise more services on the custom queue
  std::string clear_joint_forces_service_name("clear_joint_forces");
  ros::AdvertiseServiceOptions clear_joint_forces_aso =
    ros::AdvertiseServiceOptions::create<gazebo_msgs::JointRequest>(
                                                                    clear_joint_forces_service_name,
                                                                    boost::bind(&GazeboRosApiPlugin::clearJointForces,this,_1,_2),
                                                                    ros::VoidPtr(), &gazebo_queue_);
  clear_joint_forces_service_ = nh_->advertiseService(clear_joint_forces_aso);

  // Advertise more services on the custom queue
  std::string clear_body_wrenches_service_name("clear_body_wrenches");
  ros::AdvertiseServiceOptions clear_body_wrenches_aso =
    ros::AdvertiseServiceOptions::create<gazebo_msgs::BodyRequest>(
                                                                   clear_body_wrenches_service_name,
                                                                   boost::bind(&GazeboRosApiPlugin::clearBodyWrenches,this,_1,_2),
                                                                   ros::VoidPtr(), &gazebo_queue_);
  clear_body_wrenches_service_ = nh_->advertiseService(clear_body_wrenches_aso);

  // Advertise more services on the custom queue
  std::string reset_simulation_service_name("reset_simulation");
  ros::AdvertiseServiceOptions reset_simulation_aso =
    ros::AdvertiseServiceOptions::create<std_srvs::Empty>(
                                                          reset_simulation_service_name,
                                                          boost::bind(&GazeboRosApiPlugin::resetSimulation,this,_1,_2),
                                                          ros::VoidPtr(), &gazebo_queue_);
  reset_simulation_service_ = nh_->advertiseService(reset_simulation_aso);

  // Advertise more services on the custom queue
  std::string reset_world_service_name("reset_world");
  ros::AdvertiseServiceOptions reset_world_aso =
    ros::AdvertiseServiceOptions::create<std_srvs::Empty>(
                                                          reset_world_service_name,
                                                          boost::bind(&GazeboRosApiPlugin::resetWorld,this,_1,_2),
                                                          ros::VoidPtr(), &gazebo_queue_);
  reset_world_service_ = nh_->advertiseService(reset_world_aso);


  // set param for use_sim_time if not set by user already
  if(!(nh_->hasParam("/use_sim_time")))
    nh_->setParam("/use_sim_time", true);

  // todo: contemplate setting environment variable ROBOT=sim here???
  nh_->getParam("pub_clock_frequency", pub_clock_frequency_);
#if GAZEBO_MAJOR_VERSION >= 8
  last_pub_clock_time_ = world_->SimTime();
#else
  last_pub_clock_time_ = world_->GetSimTime();
#endif
}

void GazeboRosApiPlugin::onLinkStatesConnect()
{
  pub_link_states_connection_count_++;
  if (pub_link_states_connection_count_ == 1) // connect on first subscriber
    pub_link_states_event_   = gazebo::event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboRosApiPlugin::publishLinkStates,this));
}

void GazeboRosApiPlugin::onModelStatesConnect()
{
  pub_model_states_connection_count_++;
  if (pub_model_states_connection_count_ == 1) // connect on first subscriber
    pub_model_states_event_   = gazebo::event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboRosApiPlugin::publishModelStates,this));
}

void GazeboRosApiPlugin::onLinkStatesDisconnect()
{
  pub_link_states_connection_count_--;
  if (pub_link_states_connection_count_ <= 0) // disconnect with no subscribers
  {
    pub_link_states_event_.reset();
    if (pub_link_states_connection_count_ < 0) // should not be possible
      ROS_ERROR_NAMED("api_plugin", "One too mandy disconnect from pub_link_states_ in gazebo_ros.cpp? something weird");
  }
}

void GazeboRosApiPlugin::onModelStatesDisconnect()
{
  pub_model_states_connection_count_--;
  if (pub_model_states_connection_count_ <= 0) // disconnect with no subscribers
  {
    pub_model_states_event_.reset();
    if (pub_model_states_connection_count_ < 0) // should not be possible
      ROS_ERROR_NAMED("api_plugin", "One too mandy disconnect from pub_model_states_ in gazebo_ros.cpp? something weird");
  }
}

bool GazeboRosApiPlugin::getModelProperties(gazebo_msgs::GetModelProperties::Request &req,
                                            gazebo_msgs::GetModelProperties::Response &res)
{
#if GAZEBO_MAJOR_VERSION >= 8
  gazebo::physics::ModelPtr model = world_->ModelByName(req.model_name);
#else
  gazebo::physics::ModelPtr model = world_->GetModel(req.model_name);
#endif
  if (!model)
  {
    ROS_ERROR_NAMED("api_plugin", "GetModelProperties: model [%s] does not exist",req.model_name.c_str());
    res.success = false;
    res.status_message = "GetModelProperties: model does not exist";
    return true;
  }
  else
  {
    // get model parent name
    gazebo::physics::ModelPtr parent_model = boost::dynamic_pointer_cast<gazebo::physics::Model>(model->GetParent());
    if (parent_model) res.parent_model_name = parent_model->GetName();

    // get list of child bodies, geoms
    res.body_names.clear();
    res.geom_names.clear();
    for (unsigned int i = 0 ; i < model->GetChildCount(); i ++)
    {
      gazebo::physics::LinkPtr body = boost::dynamic_pointer_cast<gazebo::physics::Link>(model->GetChild(i));
      if (body)
      {
        res.body_names.push_back(body->GetName());
        // get list of geoms
        for (unsigned int j = 0; j < body->GetChildCount() ; j++)
        {
          gazebo::physics::CollisionPtr geom = boost::dynamic_pointer_cast<gazebo::physics::Collision>(body->GetChild(j));
          if (geom)
            res.geom_names.push_back(geom->GetName());
        }
      }
    }

    // get list of joints
    res.joint_names.clear();

    gazebo::physics::Joint_V joints = model->GetJoints();
    for (unsigned int i=0;i< joints.size(); i++)
      res.joint_names.push_back( joints[i]->GetName() );

    // get children model names
    res.child_model_names.clear();
    for (unsigned int j = 0; j < model->GetChildCount(); j++)
    {
      gazebo::physics::ModelPtr child_model = boost::dynamic_pointer_cast<gazebo::physics::Model>(model->GetChild(j));
      if (child_model)
        res.child_model_names.push_back(child_model->GetName() );
    }

    // is model static
    res.is_static = model->IsStatic();

    res.success = true;
    res.status_message = "GetModelProperties: got properties";
    return true;
  }
  return true;
}

bool GazeboRosApiPlugin::getWorldProperties(gazebo_msgs::GetWorldProperties::Request &req,
                                            gazebo_msgs::GetWorldProperties::Response &res)
{
  res.model_names.clear();
#if GAZEBO_MAJOR_VERSION >= 8
  res.sim_time = world_->SimTime().Double();
  for (unsigned int i = 0; i < world_->ModelCount(); i ++)
    res.model_names.push_back(world_->ModelByIndex(i)->GetName());
#else
  res.sim_time = world_->GetSimTime().Double();
  for (unsigned int i = 0; i < world_->GetModelCount(); i ++)
    res.model_names.push_back(world_->GetModel(i)->GetName());
#endif
  gzerr << "disablign rendering has not been implemented, rendering is always enabled\n";
  res.rendering_enabled = true; //world->GetRenderEngineEnabled();
  res.success = true;
  res.status_message = "GetWorldProperties: got properties";
  return true;
}

bool GazeboRosApiPlugin::getJointProperties(gazebo_msgs::GetJointProperties::Request &req,
                                            gazebo_msgs::GetJointProperties::Response &res)
{
  gazebo::physics::JointPtr joint;
#if GAZEBO_MAJOR_VERSION >= 8
  for (unsigned int i = 0; i < world_->ModelCount(); i ++)
  {
    joint = world_->ModelByIndex(i)->GetJoint(req.joint_name);
#else
  for (unsigned int i = 0; i < world_->GetModelCount(); i ++)
  {
    joint = world_->GetModel(i)->GetJoint(req.joint_name);
#endif
    if (joint) break;
  }

  if (!joint)
  {
    res.success = false;
    res.status_message = "GetJointProperties: joint not found";
    return true;
  }
  else
  {
    /// @todo: FIXME
    res.type = res.REVOLUTE;

    res.damping.clear(); // to be added to gazebo
    //res.damping.push_back(joint->GetDamping(0));

    res.position.clear();
#if GAZEBO_MAJOR_VERSION >= 8
    res.position.push_back(joint->Position(0));
#else
    res.position.push_back(joint->GetAngle(0).Radian());
#endif

    res.rate.clear(); // use GetVelocity(i)
    res.rate.push_back(joint->GetVelocity(0));

    res.success = true;
    res.status_message = "GetJointProperties: got properties";
    return true;
  }
}

bool GazeboRosApiPlugin::getLinkProperties(gazebo_msgs::GetLinkProperties::Request &req,
                                           gazebo_msgs::GetLinkProperties::Response &res)
{
#if GAZEBO_MAJOR_VERSION >= 8
  gazebo::physics::LinkPtr body = boost::dynamic_pointer_cast<gazebo::physics::Link>(world_->EntityByName(req.link_name));
#else
  gazebo::physics::LinkPtr body = boost::dynamic_pointer_cast<gazebo::physics::Link>(world_->GetEntity(req.link_name));
#endif
  if (!body)
  {
    res.success = false;
    res.status_message = "GetLinkProperties: link not found, did you forget to scope the link by model name?";
    return true;
  }
  else
  {
    /// @todo: validate
    res.gravity_mode = body->GetGravityMode();

    gazebo::physics::InertialPtr inertia = body->GetInertial();

#if GAZEBO_MAJOR_VERSION >= 8
    res.mass = body->GetInertial()->Mass();

    res.ixx = inertia->IXX();
    res.iyy = inertia->IYY();
    res.izz = inertia->IZZ();
    res.ixy = inertia->IXY();
    res.ixz = inertia->IXZ();
    res.iyz = inertia->IYZ();

    ignition::math::Vector3d com = body->GetInertial()->CoG();
#else
    res.mass = body->GetInertial()->GetMass();

    res.ixx = inertia->GetIXX();
    res.iyy = inertia->GetIYY();
    res.izz = inertia->GetIZZ();
    res.ixy = inertia->GetIXY();
    res.ixz = inertia->GetIXZ();
    res.iyz = inertia->GetIYZ();

    ignition::math::Vector3d com = body->GetInertial()->GetCoG().Ign();
#endif
    res.com.position.x = com.X();
    res.com.position.y = com.Y();
    res.com.position.z = com.Z();
    res.com.orientation.x = 0; // @todo: gazebo do not support rotated inertia yet
    res.com.orientation.y = 0;
    res.com.orientation.z = 0;
    res.com.orientation.w = 1;

    res.success = true;
    res.status_message = "GetLinkProperties: got properties";
    return true;
  }
}

bool GazeboRosApiPlugin::getLightProperties(gazebo_msgs::GetLightProperties::Request &req,
                                               gazebo_msgs::GetLightProperties::Response &res)
{
#if GAZEBO_MAJOR_VERSION >= 8
  gazebo::physics::LightPtr phy_light = world_->LightByName(req.light_name);
#else
  gazebo::physics::LightPtr phy_light = world_->Light(req.light_name);
#endif

  if (phy_light == NULL)
  {
      res.success = false;
      res.status_message = "getLightProperties: Requested light " + req.light_name + " not found!";
  }
  else
  {
    gazebo::msgs::Light light;
    phy_light->FillMsg(light);

    res.diffuse.r = light.diffuse().r();
    res.diffuse.g = light.diffuse().g();
    res.diffuse.b = light.diffuse().b();
    res.diffuse.a = light.diffuse().a();

    res.attenuation_constant = light.attenuation_constant();
    res.attenuation_linear = light.attenuation_linear();
    res.attenuation_quadratic = light.attenuation_quadratic();

    res.success = true;
  }

  return true;
}

bool GazeboRosApiPlugin::setLightProperties(gazebo_msgs::SetLightProperties::Request &req,
                                               gazebo_msgs::SetLightProperties::Response &res)
{
#if GAZEBO_MAJOR_VERSION >= 8
  gazebo::physics::LightPtr phy_light = world_->LightByName(req.light_name);
#else
  gazebo::physics::LightPtr phy_light = world_->Light(req.light_name);
#endif

  if (phy_light == NULL)
  {
    res.success = false;
    res.status_message = "setLightProperties: Requested light " + req.light_name + " not found!";
  }
  else
  {
    gazebo::msgs::Light light;

    phy_light->FillMsg(light);

    light.mutable_diffuse()->set_r(req.diffuse.r);
    light.mutable_diffuse()->set_g(req.diffuse.g);
    light.mutable_diffuse()->set_b(req.diffuse.b);
    light.mutable_diffuse()->set_a(req.diffuse.a);

    light.set_attenuation_constant(req.attenuation_constant);
    light.set_attenuation_linear(req.attenuation_linear);
    light.set_attenuation_quadratic(req.attenuation_quadratic);

    light_modify_pub_->Publish(light, true);

    res.success = true;
  }

  return true;
}

bool GazeboRosApiPlugin::setLinkProperties(gazebo_msgs::SetLinkProperties::Request &req,
                                           gazebo_msgs::SetLinkProperties::Response &res)
{
#if GAZEBO_MAJOR_VERSION >= 8
  gazebo::physics::LinkPtr body = boost::dynamic_pointer_cast<gazebo::physics::Link>(world_->EntityByName(req.link_name));
#else
  gazebo::physics::LinkPtr body = boost::dynamic_pointer_cast<gazebo::physics::Link>(world_->GetEntity(req.link_name));
#endif
  if (!body)
  {
    res.success = false;
    res.status_message = "SetLinkProperties: link not found, did you forget to scope the link by model name?";
    return true;
  }
  else
  {
    gazebo::physics::InertialPtr mass = body->GetInertial();
    // @todo: FIXME: add inertia matrix rotation to Gazebo
    // mass.SetInertiaRotation(ignition::math::Quaterniondion(req.com.orientation.w,res.com.orientation.x,req.com.orientation.y req.com.orientation.z));
    mass->SetCoG(ignition::math::Vector3d(req.com.position.x,req.com.position.y,req.com.position.z));
    mass->SetInertiaMatrix(req.ixx,req.iyy,req.izz,req.ixy,req.ixz,req.iyz);
    mass->SetMass(req.mass);
    body->SetGravityMode(req.gravity_mode);
    // @todo: mass change unverified
    res.success = true;
    res.status_message = "SetLinkProperties: properties set";
    return true;
  }
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

bool GazeboRosApiPlugin::setJointProperties(gazebo_msgs::SetJointProperties::Request &req,
                                            gazebo_msgs::SetJointProperties::Response &res)
{
  /// @todo: current settings only allows for setting of 1DOF joints (e.g. HingeJoint and SliderJoint) correctly.
  gazebo::physics::JointPtr joint;
#if GAZEBO_MAJOR_VERSION >= 8
  for (unsigned int i = 0; i < world_->ModelCount(); i ++)
  {
    joint = world_->ModelByIndex(i)->GetJoint(req.joint_name);
#else
  for (unsigned int i = 0; i < world_->GetModelCount(); i ++)
  {
    joint = world_->GetModel(i)->GetJoint(req.joint_name);
#endif
    if (joint) break;
  }

  if (!joint)
  {
    res.success = false;
    res.status_message = "SetJointProperties: joint not found";
    return true;
  }
  else
  {
    for(unsigned int i=0;i< req.ode_joint_config.damping.size();i++)
      joint->SetDamping(i,req.ode_joint_config.damping[i]);
    for(unsigned int i=0;i< req.ode_joint_config.hiStop.size();i++)
      joint->SetParam("hi_stop",i,req.ode_joint_config.hiStop[i]);
    for(unsigned int i=0;i< req.ode_joint_config.loStop.size();i++)
      joint->SetParam("lo_stop",i,req.ode_joint_config.loStop[i]);
    for(unsigned int i=0;i< req.ode_joint_config.erp.size();i++)
      joint->SetParam("erp",i,req.ode_joint_config.erp[i]);
    for(unsigned int i=0;i< req.ode_joint_config.cfm.size();i++)
      joint->SetParam("cfm",i,req.ode_joint_config.cfm[i]);
    for(unsigned int i=0;i< req.ode_joint_config.stop_erp.size();i++)
      joint->SetParam("stop_erp",i,req.ode_joint_config.stop_erp[i]);
    for(unsigned int i=0;i< req.ode_joint_config.stop_cfm.size();i++)
      joint->SetParam("stop_cfm",i,req.ode_joint_config.stop_cfm[i]);
    for(unsigned int i=0;i< req.ode_joint_config.fudge_factor.size();i++)
      joint->SetParam("fudge_factor",i,req.ode_joint_config.fudge_factor[i]);
    for(unsigned int i=0;i< req.ode_joint_config.fmax.size();i++)
      joint->SetParam("fmax",i,req.ode_joint_config.fmax[i]);
    for(unsigned int i=0;i< req.ode_joint_config.vel.size();i++)
      joint->SetParam("vel",i,req.ode_joint_config.vel[i]);

    res.success = true;
    res.status_message = "SetJointProperties: properties set";
    return true;
  }
}

bool GazeboRosApiPlugin::applyJointEffort(gazebo_msgs::ApplyJointEffort::Request &req,
                                          gazebo_msgs::ApplyJointEffort::Response &res)
{
  gazebo::physics::JointPtr joint;
#if GAZEBO_MAJOR_VERSION >= 8
  for (unsigned int i = 0; i < world_->ModelCount(); i ++)
  {
    joint = world_->ModelByIndex(i)->GetJoint(req.joint_name);
#else
  for (unsigned int i = 0; i < world_->GetModelCount(); i ++)
  {
    joint = world_->GetModel(i)->GetJoint(req.joint_name);
#endif
    if (joint)
    {
      GazeboRosApiPlugin::ForceJointJob* fjj = new GazeboRosApiPlugin::ForceJointJob;
      fjj->joint = joint;
      fjj->force = req.effort;
      fjj->start_time = req.start_time;
#if GAZEBO_MAJOR_VERSION >= 8
      if (fjj->start_time < ros::Time(world_->SimTime().Double()))
        fjj->start_time = ros::Time(world_->SimTime().Double());
#else
      if (fjj->start_time < ros::Time(world_->GetSimTime().Double()))
        fjj->start_time = ros::Time(world_->GetSimTime().Double());
#endif
      fjj->duration = req.duration;
      lock_.lock();
      force_joint_jobs_.push_back(fjj);
      lock_.unlock();

      res.success = true;
      res.status_message = "ApplyJointEffort: effort set";
      return true;
    }
  }

  res.success = false;
  res.status_message = "ApplyJointEffort: joint not found";
  return true;
}

bool GazeboRosApiPlugin::resetSimulation(std_srvs::Empty::Request &req,std_srvs::Empty::Response &res)
{
  world_->Reset();
  return true;
}

bool GazeboRosApiPlugin::resetWorld(std_srvs::Empty::Request &req,std_srvs::Empty::Response &res)
{
  world_->ResetEntities(gazebo::physics::Base::MODEL);
  return true;
}

bool GazeboRosApiPlugin::pausePhysics(std_srvs::Empty::Request &req,std_srvs::Empty::Response &res)
{
  world_->SetPaused(true);
  return true;
}

bool GazeboRosApiPlugin::unpausePhysics(std_srvs::Empty::Request &req,std_srvs::Empty::Response &res)
{
  world_->SetPaused(false);
  return true;
}

bool GazeboRosApiPlugin::clearJointForces(gazebo_msgs::JointRequest::Request &req,
                                          gazebo_msgs::JointRequest::Response &res)
{
  return clearJointForces(req.joint_name);
}
bool GazeboRosApiPlugin::clearJointForces(std::string joint_name)
{
  bool search = true;
  lock_.lock();
  while(search)
  {
    search = false;
    for (std::vector<GazeboRosApiPlugin::ForceJointJob*>::iterator iter=force_joint_jobs_.begin();iter!=force_joint_jobs_.end();++iter)
    {
      if ((*iter)->joint->GetName() == joint_name)
      {
        // found one, search through again
        search = true;
        delete (*iter);
        force_joint_jobs_.erase(iter);
        break;
      }
    }
  }
  lock_.unlock();
  return true;
}

bool GazeboRosApiPlugin::clearBodyWrenches(gazebo_msgs::BodyRequest::Request &req,
                                           gazebo_msgs::BodyRequest::Response &res)
{
  return clearBodyWrenches(req.body_name);
}
bool GazeboRosApiPlugin::clearBodyWrenches(std::string body_name)
{
  bool search = true;
  lock_.lock();
  while(search)
  {
    search = false;
    for (std::vector<GazeboRosApiPlugin::WrenchBodyJob*>::iterator iter=wrench_body_jobs_.begin();iter!=wrench_body_jobs_.end();++iter)
    {
      //ROS_ERROR_NAMED("api_plugin", "search %s %s",(*iter)->body->GetScopedName().c_str(), body_name.c_str());
      if ((*iter)->body->GetScopedName() == body_name)
      {
        // found one, search through again
        search = true;
        delete (*iter);
        wrench_body_jobs_.erase(iter);
        break;
      }
    }
  }
  lock_.unlock();
  return true;
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

void GazeboRosApiPlugin::transformWrench( ignition::math::Vector3d &target_force, ignition::math::Vector3d &target_torque,
                                          const ignition::math::Vector3d &reference_force,
                                          const ignition::math::Vector3d &reference_torque,
                                          const ignition::math::Pose3d &target_to_reference )
{
  // rotate force into target frame
  target_force = target_to_reference.Rot().RotateVector(reference_force);
  // rotate torque into target frame
  target_torque = target_to_reference.Rot().RotateVector(reference_torque);

  // target force is the refence force rotated by the target->reference transform
  target_torque = target_torque + target_to_reference.Pos().Cross(target_force);
}

bool GazeboRosApiPlugin::applyBodyWrench(gazebo_msgs::ApplyBodyWrench::Request &req,
                                         gazebo_msgs::ApplyBodyWrench::Response &res)
{
#if GAZEBO_MAJOR_VERSION >= 8
  gazebo::physics::LinkPtr body = boost::dynamic_pointer_cast<gazebo::physics::Link>(world_->EntityByName(req.body_name));
  gazebo::physics::EntityPtr frame = world_->EntityByName(req.reference_frame);
#else
  gazebo::physics::LinkPtr body = boost::dynamic_pointer_cast<gazebo::physics::Link>(world_->GetEntity(req.body_name));
  gazebo::physics::EntityPtr frame = world_->GetEntity(req.reference_frame);
#endif
  if (!body)
  {
    ROS_ERROR_NAMED("api_plugin", "ApplyBodyWrench: body [%s] does not exist",req.body_name.c_str());
    res.success = false;
    res.status_message = "ApplyBodyWrench: body does not exist";
    return true;
  }

  // target wrench
  ignition::math::Vector3d reference_force(req.wrench.force.x,req.wrench.force.y,req.wrench.force.z);
  ignition::math::Vector3d reference_torque(req.wrench.torque.x,req.wrench.torque.y,req.wrench.torque.z);
  ignition::math::Vector3d reference_point(req.reference_point.x,req.reference_point.y,req.reference_point.z);

  ignition::math::Vector3d target_force;
  ignition::math::Vector3d target_torque;

  /// shift wrench to body frame if a non-zero reference point is given
  ///   @todo: to be more general, should we make the reference point a reference pose?
  reference_torque = reference_torque + reference_point.Cross(reference_force);

  /// @todo: FIXME map is really wrong, need to use tf here somehow
  if (frame)
  {
    // get reference frame (body/model(body)) pose and
    // transform target pose to absolute world frame
    // @todo: need to modify wrench (target force and torque by frame)
    //        transform wrench from reference_point in reference_frame
    //        into the reference frame of the body
    //        first, translate by reference point to the body frame
#if GAZEBO_MAJOR_VERSION >= 8
    ignition::math::Pose3d framePose = frame->WorldPose();
    ignition::math::Pose3d bodyPose = body->WorldPose();
#else
    ignition::math::Pose3d framePose = frame->GetWorldPose().Ign();
    ignition::math::Pose3d bodyPose = body->GetWorldPose().Ign();
#endif
    ignition::math::Pose3d target_to_reference = framePose - bodyPose;
    ROS_DEBUG_NAMED("api_plugin", "reference frame for applied wrench: [%f %f %f, %f %f %f]-[%f %f %f, %f %f %f]=[%f %f %f, %f %f %f]",
              bodyPose.Pos().X(),
              bodyPose.Pos().Y(),
              bodyPose.Pos().Z(),
              bodyPose.Rot().Euler().X(),
              bodyPose.Rot().Euler().Y(),
              bodyPose.Rot().Euler().Z(),
              framePose.Pos().X(),
              framePose.Pos().Y(),
              framePose.Pos().Z(),
              framePose.Rot().Euler().X(),
              framePose.Rot().Euler().Y(),
              framePose.Rot().Euler().Z(),
              target_to_reference.Pos().X(),
              target_to_reference.Pos().Y(),
              target_to_reference.Pos().Z(),
              target_to_reference.Rot().Euler().X(),
              target_to_reference.Rot().Euler().Y(),
              target_to_reference.Rot().Euler().Z()
              );
    transformWrench(target_force, target_torque, reference_force, reference_torque, target_to_reference);
    ROS_ERROR_NAMED("api_plugin", "wrench defined as [%s]:[%f %f %f, %f %f %f] --> applied as [%s]:[%f %f %f, %f %f %f]",
              frame->GetName().c_str(),
              reference_force.X(),
              reference_force.Y(),
              reference_force.Z(),
              reference_torque.X(),
              reference_torque.Y(),
              reference_torque.Z(),
              body->GetName().c_str(),
              target_force.X(),
              target_force.Y(),
              target_force.Z(),
              target_torque.X(),
              target_torque.Y(),
              target_torque.Z()
              );

  }
  else if (req.reference_frame == "" || req.reference_frame == "world" || req.reference_frame == "map" || req.reference_frame == "/map")
  {
    ROS_INFO_NAMED("api_plugin", "ApplyBodyWrench: reference_frame is empty/world/map, using inertial frame, transferring from body relative to inertial frame");
    // FIXME: transfer to inertial frame
#if GAZEBO_MAJOR_VERSION >= 8
    ignition::math::Pose3d target_to_reference = body->WorldPose();
#else
    ignition::math::Pose3d target_to_reference = body->GetWorldPose().Ign();
#endif
    target_force = reference_force;
    target_torque = reference_torque;

  }
  else
  {
    ROS_ERROR_NAMED("api_plugin", "ApplyBodyWrench: reference_frame is not a valid entity name");
    res.success = false;
    res.status_message = "ApplyBodyWrench: reference_frame not found";
    return true;
  }

  // apply wrench
  // schedule a job to do below at appropriate times:
  // body->SetForce(force)
  // body->SetTorque(torque)
  GazeboRosApiPlugin::WrenchBodyJob* wej = new GazeboRosApiPlugin::WrenchBodyJob;
  wej->body = body;
  wej->force = target_force;
  wej->torque = target_torque;
  wej->start_time = req.start_time;
#if GAZEBO_MAJOR_VERSION >= 8
  if (wej->start_time < ros::Time(world_->SimTime().Double()))
    wej->start_time = ros::Time(world_->SimTime().Double());
#else
  if (wej->start_time < ros::Time(world_->GetSimTime().Double()))
    wej->start_time = ros::Time(world_->GetSimTime().Double());
#endif
  wej->duration = req.duration;
  lock_.lock();
  wrench_body_jobs_.push_back(wej);
  lock_.unlock();

  res.success = true;
  res.status_message = "";
  return true;
}

void GazeboRosApiPlugin::wrenchBodySchedulerSlot()
{
  // MDMutex locks in case model is getting deleted, don't have to do this if we delete jobs first
  // boost::recursive_mutex::scoped_lock lock(*world->GetMDMutex());
  lock_.lock();
  for (std::vector<GazeboRosApiPlugin::WrenchBodyJob*>::iterator iter=wrench_body_jobs_.begin();iter!=wrench_body_jobs_.end();)
  {
    // check times and apply wrench if necessary
#if GAZEBO_MAJOR_VERSION >= 8
    ros::Time simTime = ros::Time(world_->SimTime().Double());
#else
    ros::Time simTime = ros::Time(world_->GetSimTime().Double());
#endif
    if (simTime >= (*iter)->start_time)
      if (simTime <= (*iter)->start_time+(*iter)->duration ||
          (*iter)->duration.toSec() < 0.0)
      {
        if ((*iter)->body) // if body exists
        {
          (*iter)->body->SetForce((*iter)->force);
          (*iter)->body->SetTorque((*iter)->torque);
        }
        else
          (*iter)->duration.fromSec(0.0); // mark for delete
      }

    if (simTime > (*iter)->start_time+(*iter)->duration &&
        (*iter)->duration.toSec() >= 0.0)
    {
      // remove from queue once expires
      delete (*iter);
      iter = wrench_body_jobs_.erase(iter);
    }
    else
      ++iter;
  }
  lock_.unlock();
}

void GazeboRosApiPlugin::forceJointSchedulerSlot()
{
  // MDMutex locks in case model is getting deleted, don't have to do this if we delete jobs first
  // boost::recursive_mutex::scoped_lock lock(*world->GetMDMutex());
  lock_.lock();
  for (std::vector<GazeboRosApiPlugin::ForceJointJob*>::iterator iter=force_joint_jobs_.begin();iter!=force_joint_jobs_.end();)
  {
    // check times and apply force if necessary
#if GAZEBO_MAJOR_VERSION >= 8
    ros::Time simTime = ros::Time(world_->SimTime().Double());
#else
    ros::Time simTime = ros::Time(world_->GetSimTime().Double());
#endif
    if (simTime >= (*iter)->start_time)
      if (simTime <= (*iter)->start_time+(*iter)->duration ||
          (*iter)->duration.toSec() < 0.0)
      {
        if ((*iter)->joint) // if joint exists
          (*iter)->joint->SetForce(0,(*iter)->force);
        else
          (*iter)->duration.fromSec(0.0); // mark for delete
      }

    if (simTime > (*iter)->start_time+(*iter)->duration &&
        (*iter)->duration.toSec() >= 0.0)
    {
      // remove from queue once expires
      iter = force_joint_jobs_.erase(iter);
    }
    else
      ++iter;
  }
  lock_.unlock();
}

void GazeboRosApiPlugin::publishSimTime(const boost::shared_ptr<gazebo::msgs::WorldStatistics const> &msg)
{
  ROS_ERROR_NAMED("api_plugin", "CLOCK2");
#if GAZEBO_MAJOR_VERSION >= 8
  gazebo::common::Time sim_time = world_->SimTime();
#else
  gazebo::common::Time sim_time = world_->GetSimTime();
#endif
  if (pub_clock_frequency_ > 0 && (sim_time - last_pub_clock_time_).Double() < 1.0/pub_clock_frequency_)
    return;

  gazebo::common::Time currentTime = gazebo::msgs::Convert( msg->sim_time() );
  rosgraph_msgs::Clock ros_time_;
  ros_time_.clock.fromSec(currentTime.Double());
  //  publish time to ros
  last_pub_clock_time_ = sim_time;
  pub_clock_.publish(ros_time_);
}
void GazeboRosApiPlugin::publishSimTime()
{
#if GAZEBO_MAJOR_VERSION >= 8
  gazebo::common::Time sim_time = world_->SimTime();
#else
  gazebo::common::Time sim_time = world_->GetSimTime();
#endif
  if (pub_clock_frequency_ > 0 && (sim_time - last_pub_clock_time_).Double() < 1.0/pub_clock_frequency_)
    return;

#if GAZEBO_MAJOR_VERSION >= 8
  gazebo::common::Time currentTime = world_->SimTime();
#else
  gazebo::common::Time currentTime = world_->GetSimTime();
#endif
  rosgraph_msgs::Clock ros_time_;
  ros_time_.clock.fromSec(currentTime.Double());
  //  publish time to ros
  last_pub_clock_time_ = sim_time;
  pub_clock_.publish(ros_time_);
}

void GazeboRosApiPlugin::publishLinkStates()
{
  gazebo_msgs::LinkStates link_states;

  // fill link_states
#if GAZEBO_MAJOR_VERSION >= 8
  for (unsigned int i = 0; i < world_->ModelCount(); i ++)
  {
    gazebo::physics::ModelPtr model = world_->ModelByIndex(i);
#else
  for (unsigned int i = 0; i < world_->GetModelCount(); i ++)
  {
    gazebo::physics::ModelPtr model = world_->GetModel(i);
#endif

    for (unsigned int j = 0 ; j < model->GetChildCount(); j ++)
    {
      gazebo::physics::LinkPtr body = boost::dynamic_pointer_cast<gazebo::physics::Link>(model->GetChild(j));

      if (body)
      {
        link_states.name.push_back(body->GetScopedName());
        geometry_msgs::Pose pose;
#if GAZEBO_MAJOR_VERSION >= 8
        ignition::math::Pose3d  body_pose = body->WorldPose(); // - myBody->GetCoMPose();
        ignition::math::Vector3d linear_vel  = body->WorldLinearVel();
        ignition::math::Vector3d angular_vel = body->WorldAngularVel();
#else
        ignition::math::Pose3d  body_pose = body->GetWorldPose().Ign(); // - myBody->GetCoMPose();
        ignition::math::Vector3d linear_vel  = body->GetWorldLinearVel().Ign();
        ignition::math::Vector3d angular_vel = body->GetWorldAngularVel().Ign();
#endif
        ignition::math::Vector3d pos = body_pose.Pos();
        ignition::math::Quaterniond rot = body_pose.Rot();
        pose.position.x = pos.X();
        pose.position.y = pos.Y();
        pose.position.z = pos.Z();
        pose.orientation.w = rot.W();
        pose.orientation.x = rot.X();
        pose.orientation.y = rot.Y();
        pose.orientation.z = rot.Z();
        link_states.pose.push_back(pose);
        geometry_msgs::Twist twist;
        twist.linear.x = linear_vel.X();
        twist.linear.y = linear_vel.Y();
        twist.linear.z = linear_vel.Z();
        twist.angular.x = angular_vel.X();
        twist.angular.y = angular_vel.Y();
        twist.angular.z = angular_vel.Z();
        link_states.twist.push_back(twist);
      }
    }
  }

  pub_link_states_.publish(link_states);
}

void GazeboRosApiPlugin::publishModelStates()
{
  gazebo_msgs::ModelStates model_states;

  // fill model_states
#if GAZEBO_MAJOR_VERSION >= 8
  for (unsigned int i = 0; i < world_->ModelCount(); i ++)
  {
    gazebo::physics::ModelPtr model = world_->ModelByIndex(i);
    ignition::math::Pose3d  model_pose = model->WorldPose(); // - myBody->GetCoMPose();
    ignition::math::Vector3d linear_vel  = model->WorldLinearVel();
    ignition::math::Vector3d angular_vel = model->WorldAngularVel();
#else
  for (unsigned int i = 0; i < world_->GetModelCount(); i ++)
  {
    gazebo::physics::ModelPtr model = world_->GetModel(i);
    ignition::math::Pose3d  model_pose = model->GetWorldPose().Ign(); // - myBody->GetCoMPose();
    ignition::math::Vector3d linear_vel  = model->GetWorldLinearVel().Ign();
    ignition::math::Vector3d angular_vel = model->GetWorldAngularVel().Ign();
#endif
    ignition::math::Vector3d pos = model_pose.Pos();
    ignition::math::Quaterniond rot = model_pose.Rot();
    geometry_msgs::Pose pose;
    pose.position.x = pos.X();
    pose.position.y = pos.Y();
    pose.position.z = pos.Z();
    pose.orientation.w = rot.W();
    pose.orientation.x = rot.X();
    pose.orientation.y = rot.Y();
    pose.orientation.z = rot.Z();
    model_states.pose.push_back(pose);
    model_states.name.push_back(model->GetName());
    geometry_msgs::Twist twist;
    twist.linear.x = linear_vel.X();
    twist.linear.y = linear_vel.Y();
    twist.linear.z = linear_vel.Z();
    twist.angular.x = angular_vel.X();
    twist.angular.y = angular_vel.Y();
    twist.angular.z = angular_vel.Z();
    model_states.twist.push_back(twist);
  }
  pub_model_states_.publish(model_states);
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
