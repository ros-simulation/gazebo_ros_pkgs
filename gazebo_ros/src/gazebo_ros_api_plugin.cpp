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
#include "gazebo_ros_api_plugin.h"

namespace gazebo
{

GazeboRosApiPlugin::GazeboRosApiPlugin() :
  physics_reconfigure_initialized_(false),
  world_created_(false),
  stop_(false),
  plugin_loaded_(false),
  pub_link_states_connection_count_(0),
  pub_model_states_connection_count_(0)
{
  robot_namespace_.clear();
}

GazeboRosApiPlugin::~GazeboRosApiPlugin()
{
  ROS_DEBUG_STREAM_NAMED("api_plugin","GazeboRosApiPlugin Deconstructor start");

  // Unload the sigint event
  gazebo::event::Events::DisconnectSigInt(sigint_event_);
  ROS_DEBUG_STREAM_NAMED("api_plugin","After sigint_event unload");

  // Don't attempt to unload this plugin if it was never loaded in the Load() function
  if(!plugin_loaded_)
  {
    ROS_DEBUG_STREAM_NAMED("api_plugin","Deconstructor skipped because never loaded");
    return;
  }

  // Disconnect slots
  gazebo::event::Events::DisconnectWorldUpdateBegin(wrench_update_event_);
  gazebo::event::Events::DisconnectWorldUpdateBegin(force_update_event_);
  gazebo::event::Events::DisconnectWorldUpdateBegin(time_update_event_);
  ROS_DEBUG_STREAM_NAMED("api_plugin","Slots disconnected");

  if (pub_link_states_connection_count_ > 0) // disconnect if there are subscribers on exit
    gazebo::event::Events::DisconnectWorldUpdateBegin(pub_link_states_event_);
  if (pub_model_states_connection_count_ > 0) // disconnect if there are subscribers on exit
    gazebo::event::Events::DisconnectWorldUpdateBegin(pub_model_states_event_);
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
    ROS_ERROR("Something other than this gazebo_ros_api plugin started ros::init(...), command line arguments may not be parsed properly.");

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
  ROS_INFO("Finished loading Gazebo ROS API Plugin.");
}

void GazeboRosApiPlugin::loadGazeboRosApiPlugin(std::string world_name)
{
  // make sure things are only called once
  gazebo::event::Events::DisconnectWorldCreated(load_gazebo_ros_api_plugin_event_);
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
    //ROS_ERROR("world name: [%s]",world->GetName().c_str());
    // connect helper function to signal for scheduling torque/forces, etc
    ROS_FATAL("cannot load gazebo ros api server plugin, physics::get_world() fails to return world");
    return;
  }

  gazebonode_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
  gazebonode_->Init(world_name);
  //stat_sub_ = gazebonode_->Subscribe("~/world_stats", &GazeboRosApiPlugin::publishSimTime, this); // TODO: does not work in server plugin?
  factory_pub_ = gazebonode_->Advertise<gazebo::msgs::Factory>("~/factory");
  request_pub_ = gazebonode_->Advertise<gazebo::msgs::Request>("~/request");
  response_sub_ = gazebonode_->Subscribe("~/response",&GazeboRosApiPlugin::onResponse, this);

  // reset topic connection counts
  pub_link_states_connection_count_ = 0;
  pub_model_states_connection_count_ = 0;

  /// \brief advertise all services
  advertiseServices();

  // hooks for applying forces, publishing simtime on /clock
  wrench_update_event_ = gazebo::event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboRosApiPlugin::wrenchBodySchedulerSlot,this));
  force_update_event_  = gazebo::event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboRosApiPlugin::forceJointSchedulerSlot,this));
  time_update_event_   = gazebo::event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboRosApiPlugin::publishSimTime,this));
}

void GazeboRosApiPlugin::onResponse(ConstResponsePtr &response)
{

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
  // publish clock for simulated ros time
  pub_clock_ = nh_->advertise<rosgraph_msgs::Clock>("/clock",10);

  // Advertise spawn services on the custom queue - DEPRECATED IN HYDRO
  std::string spawn_gazebo_model_service_name("spawn_gazebo_model");
  ros::AdvertiseServiceOptions spawn_gazebo_model_aso =
    ros::AdvertiseServiceOptions::create<gazebo_msgs::SpawnModel>(
                                                                  spawn_gazebo_model_service_name,
                                                                  boost::bind(&GazeboRosApiPlugin::spawnGazeboModel,this,_1,_2),
                                                                  ros::VoidPtr(), &gazebo_queue_);
  spawn_gazebo_model_service_ = nh_->advertiseService(spawn_gazebo_model_aso);

  // Advertise spawn services on the custom queue
  std::string spawn_sdf_model_service_name("spawn_sdf_model");
  ros::AdvertiseServiceOptions spawn_sdf_model_aso =
    ros::AdvertiseServiceOptions::create<gazebo_msgs::SpawnModel>(
                                                                  spawn_sdf_model_service_name,
                                                                  boost::bind(&GazeboRosApiPlugin::spawnSDFModel,this,_1,_2),
                                                                  ros::VoidPtr(), &gazebo_queue_);
  spawn_sdf_model_service_ = nh_->advertiseService(spawn_sdf_model_aso);

  // Advertise spawn services on the custom queue
  std::string spawn_urdf_model_service_name("spawn_urdf_model");
  ros::AdvertiseServiceOptions spawn_urdf_model_aso =
    ros::AdvertiseServiceOptions::create<gazebo_msgs::SpawnModel>(
                                                                  spawn_urdf_model_service_name,
                                                                  boost::bind(&GazeboRosApiPlugin::spawnURDFModel,this,_1,_2),
                                                                  ros::VoidPtr(), &gazebo_queue_);
  spawn_urdf_model_service_ = nh_->advertiseService(spawn_urdf_model_aso);

  // Advertise delete services on the custom queue
  std::string delete_model_service_name("delete_model");
  ros::AdvertiseServiceOptions delete_aso =
    ros::AdvertiseServiceOptions::create<gazebo_msgs::DeleteModel>(
                                                                   delete_model_service_name,
                                                                   boost::bind(&GazeboRosApiPlugin::deleteModel,this,_1,_2),
                                                                   ros::VoidPtr(), &gazebo_queue_);
  delete_model_service_ = nh_->advertiseService(delete_aso);

  // Advertise more services on the custom queue
  std::string get_model_properties_service_name("get_model_properties");
  ros::AdvertiseServiceOptions get_model_properties_aso =
    ros::AdvertiseServiceOptions::create<gazebo_msgs::GetModelProperties>(
                                                                          get_model_properties_service_name,
                                                                          boost::bind(&GazeboRosApiPlugin::getModelProperties,this,_1,_2),
                                                                          ros::VoidPtr(), &gazebo_queue_);
  get_model_properties_service_ = nh_->advertiseService(get_model_properties_aso);

  // Advertise more services on the custom queue
  std::string get_model_state_service_name("get_model_state");
  ros::AdvertiseServiceOptions get_model_state_aso =
    ros::AdvertiseServiceOptions::create<gazebo_msgs::GetModelState>(
                                                                     get_model_state_service_name,
                                                                     boost::bind(&GazeboRosApiPlugin::getModelState,this,_1,_2),
                                                                     ros::VoidPtr(), &gazebo_queue_);
  get_model_state_service_ = nh_->advertiseService(get_model_state_aso);

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
  std::string get_link_state_service_name("get_link_state");
  ros::AdvertiseServiceOptions get_link_state_aso =
    ros::AdvertiseServiceOptions::create<gazebo_msgs::GetLinkState>(
                                                                    get_link_state_service_name,
                                                                    boost::bind(&GazeboRosApiPlugin::getLinkState,this,_1,_2),
                                                                    ros::VoidPtr(), &gazebo_queue_);
  get_link_state_service_ = nh_->advertiseService(get_link_state_aso);

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
  std::string set_model_state_service_name("set_model_state");
  ros::AdvertiseServiceOptions set_model_state_aso =
    ros::AdvertiseServiceOptions::create<gazebo_msgs::SetModelState>(
                                                                     set_model_state_service_name,
                                                                     boost::bind(&GazeboRosApiPlugin::setModelState,this,_1,_2),
                                                                     ros::VoidPtr(), &gazebo_queue_);
  set_model_state_service_ = nh_->advertiseService(set_model_state_aso);

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
  std::string set_link_state_service_name("set_link_state");
  ros::AdvertiseServiceOptions set_link_state_aso =
    ros::AdvertiseServiceOptions::create<gazebo_msgs::SetLinkState>(
                                                                    set_link_state_service_name,
                                                                    boost::bind(&GazeboRosApiPlugin::setLinkState,this,_1,_2),
                                                                    ros::VoidPtr(), &gazebo_queue_);
  set_link_state_service_ = nh_->advertiseService(set_link_state_aso);

  // Advertise topic on custom queue
  // topic callback version for set_link_state
  ros::SubscribeOptions link_state_so =
    ros::SubscribeOptions::create<gazebo_msgs::LinkState>(
                                                          "set_link_state",10,
                                                          boost::bind( &GazeboRosApiPlugin::updateLinkState,this,_1),
                                                          ros::VoidPtr(), &gazebo_queue_);
  set_link_state_topic_ = nh_->subscribe(link_state_so);

  // topic callback version for set_model_state
  ros::SubscribeOptions model_state_so =
    ros::SubscribeOptions::create<gazebo_msgs::ModelState>(
                                                           "set_model_state",10,
                                                           boost::bind( &GazeboRosApiPlugin::updateModelState,this,_1),
                                                           ros::VoidPtr(), &gazebo_queue_);
  set_model_state_topic_ = nh_->subscribe(model_state_so);

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
  nh_->setParam("/use_sim_time", true);

  // todo: contemplate setting environment variable ROBOT=sim here???

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
    gazebo::event::Events::DisconnectWorldUpdateBegin(pub_link_states_event_);
    if (pub_link_states_connection_count_ < 0) // should not be possible
      ROS_ERROR("one too mandy disconnect from pub_link_states_ in gazebo_ros.cpp? something weird");
  }
}

void GazeboRosApiPlugin::onModelStatesDisconnect()
{
  pub_model_states_connection_count_--;
  if (pub_model_states_connection_count_ <= 0) // disconnect with no subscribers
  {
    gazebo::event::Events::DisconnectWorldUpdateBegin(pub_model_states_event_);
    if (pub_model_states_connection_count_ < 0) // should not be possible
      ROS_ERROR("one too mandy disconnect from pub_model_states_ in gazebo_ros.cpp? something weird");
  }
}

bool GazeboRosApiPlugin::spawnURDFModel(gazebo_msgs::SpawnModel::Request &req,
                                        gazebo_msgs::SpawnModel::Response &res)
{
  // get name space for the corresponding model plugins
  robot_namespace_ = req.robot_namespace;

  // incoming robot model string
  std::string model_xml = req.model_xml;

  if (!isURDF(model_xml))
  {
    ROS_ERROR("SpawnModel: Failure - model format is not URDF.");
    res.success = false;
    res.status_message = "SpawnModel: Failure - model format is not URDF.";
    return false;
  }

  /// STRIP DECLARATION <? ... xml version="1.0" ... ?> from model_xml
  /// @todo: does tinyxml have functionality for this?
  /// @todo: should gazebo take care of the declaration?
  {
    std::string open_bracket("<?");
    std::string close_bracket("?>");
    size_t pos1 = model_xml.find(open_bracket,0);
    size_t pos2 = model_xml.find(close_bracket,0);
    if (pos1 != std::string::npos && pos2 != std::string::npos)
      model_xml.replace(pos1,pos2-pos1+2,std::string(""));
  }

  // Now, replace package://xxx with the full path to the package
  {
    std::string package_prefix("package://");
    size_t pos1 = model_xml.find(package_prefix,0);
    while (pos1 != std::string::npos)
    {
      size_t pos2 = model_xml.find("/", pos1+10);
      //ROS_DEBUG(" pos %d %d",(int)pos1, (int)pos2);
      if (pos2 == std::string::npos || pos1 >= pos2)
      {
        ROS_ERROR("malformed package name?");
        break;
      }

      std::string package_name = model_xml.substr(pos1+10,pos2-pos1-10);
      //ROS_DEBUG("package name [%s]", package_name.c_str());
      std::string package_path = ros::package::getPath(package_name);
      if (package_path.empty())
      {
        ROS_FATAL("Package[%s] does not have a path",package_name.c_str());
        res.success = false;
        res.status_message = std::string("urdf reference package name does not exist: ")+package_name;
        return false;
      }
      ROS_DEBUG_ONCE("Package name [%s] has path [%s]", package_name.c_str(), package_path.c_str());

      model_xml.replace(pos1,(pos2-pos1),package_path);
      pos1 = model_xml.find(package_prefix,0);
    }
  }
  // ROS_DEBUG("Model XML\n\n%s\n\n ",model_xml.c_str());

  req.model_xml = model_xml;

  // Model is now considered convert to SDF
  return spawnSDFModel(req,res);
}

// DEPRECATED IN HYDRO
bool GazeboRosApiPlugin::spawnGazeboModel(gazebo_msgs::SpawnModel::Request &req,
                                          gazebo_msgs::SpawnModel::Response &res)
{
  ROS_WARN_STREAM_NAMED("api_plugin","/gazebo/spawn_gazebo_model is deprecated, use /gazebo/spawn_sdf_model instead");
  spawnSDFModel(req, res);
}

bool GazeboRosApiPlugin::spawnSDFModel(gazebo_msgs::SpawnModel::Request &req,
                                       gazebo_msgs::SpawnModel::Response &res)
{
  // incoming robot name
  std::string model_name = req.model_name;

  // get name space for the corresponding model plugins
  robot_namespace_ = req.robot_namespace;

  // get initial pose of model
  gazebo::math::Vector3 initial_xyz(req.initial_pose.position.x,req.initial_pose.position.y,req.initial_pose.position.z);
  // get initial roll pitch yaw (fixed frame transform)
  gazebo::math::Quaternion initial_q(req.initial_pose.orientation.w,req.initial_pose.orientation.x,req.initial_pose.orientation.y,req.initial_pose.orientation.z);

  // refernce frame for initial pose definition, modify initial pose if defined
  gazebo::physics::LinkPtr frame = boost::dynamic_pointer_cast<gazebo::physics::Link>(world_->GetEntity(req.reference_frame));
  if (frame)
  {
    // convert to relative pose
    gazebo::math::Pose frame_pose = frame->GetWorldPose();
    initial_xyz = frame_pose.rot.RotateVector(initial_xyz);
    initial_xyz += frame_pose.pos;
    initial_q *= frame_pose.rot;
  }

  /// @todo: map is really wrong, need to use tf here somehow
  else if (req.reference_frame == "" || req.reference_frame == "world" || req.reference_frame == "map" || req.reference_frame == "/map")
  {
    ROS_DEBUG("SpawnModel: reference_frame is empty/world/map, using inertial frame");
  }
  else
  {
    res.success = false;
    res.status_message = "SpawnModel: reference reference_frame not found, did you forget to scope the link by model name?";
    return true;
  }

  // incoming robot model string
  std::string model_xml = req.model_xml;

  // store resulting Gazebo Model XML to be sent to spawn queue
  // get incoming string containg either an URDF or a Gazebo Model XML
  // grab from parameter server if necessary convert to SDF if necessary
  stripXmlDeclaration(model_xml);

  // put string in TiXmlDocument for manipulation
  TiXmlDocument gazebo_model_xml;
  gazebo_model_xml.Parse(model_xml.c_str());

  // optional model manipulations: update initial pose && replace model name
  if (isSDF(model_xml))
  {
    updateSDFAttributes(gazebo_model_xml, model_name, initial_xyz, initial_q);

    // Walk recursively through the entire SDF, locate plugin tags and
    // add robotNamespace as a child with the correct namespace
    if (!this->robot_namespace_.empty())
    {
      // Get root element for SDF
      TiXmlNode* model_tixml = gazebo_model_xml.FirstChild("sdf");
      model_tixml = (!model_tixml) ?
          gazebo_model_xml.FirstChild("gazebo") : model_tixml;
      if (model_tixml)
      {
        walkChildAddRobotNamespace(model_tixml);
      }
      else
      {
        ROS_WARN("Unable to add robot namespace to xml");
      }
    }
  }
  else if (isURDF(model_xml))
  {
    updateURDFModelPose(gazebo_model_xml, initial_xyz, initial_q);
    updateURDFName(gazebo_model_xml, model_name);

    // Walk recursively through the entire URDF, locate plugin tags and
    // add robotNamespace as a child with the correct namespace
    if (!this->robot_namespace_.empty())
    {
      // Get root element for URDF
      TiXmlNode* model_tixml = gazebo_model_xml.FirstChild("robot");
      if (model_tixml)
      {
        walkChildAddRobotNamespace(model_tixml);
      }
      else
      {
        ROS_WARN("Unable to add robot namespace to xml");
      }
    }
  }
  else
  {
    ROS_ERROR("GazeboRosApiPlugin SpawnModel Failure: input xml format not recognized");
    res.success = false;
    res.status_message = std::string("GazeboRosApiPlugin SpawnModel Failure: input model_xml not SDF or URDF, or cannot be converted to Gazebo compatible format.");
    return true;
  }

  // do spawning check if spawn worked, return response
  return spawnAndConform(gazebo_model_xml, model_name, res);
}

bool GazeboRosApiPlugin::deleteModel(gazebo_msgs::DeleteModel::Request &req,
                                     gazebo_msgs::DeleteModel::Response &res)
{
  // clear forces, etc for the body in question
  gazebo::physics::ModelPtr model = world_->GetModel(req.model_name);
  if (!model)
  {
    ROS_ERROR("DeleteModel: model [%s] does not exist",req.model_name.c_str());
    res.success = false;
    res.status_message = "DeleteModel: model does not exist";
    return true;
  }

  // delete wrench jobs on bodies
  for (unsigned int i = 0 ; i < model->GetChildCount(); i ++)
  {
    gazebo::physics::LinkPtr body = boost::dynamic_pointer_cast<gazebo::physics::Link>(model->GetChild(i));
    if (body)
    {
      // look for it in jobs, delete body wrench jobs
      clearBodyWrenches(body->GetScopedName());
    }
  }

  // delete force jobs on joints
  gazebo::physics::Joint_V joints = model->GetJoints();
  for (unsigned int i=0;i< joints.size(); i++)
  {
    // look for it in jobs, delete joint force jobs
    clearJointForces(joints[i]->GetName());
  }

  // send delete model request
  gazebo::msgs::Request *msg = gazebo::msgs::CreateRequest("entity_delete",req.model_name);
  request_pub_->Publish(*msg,true);

  ros::Duration model_spawn_timeout(60.0);
  ros::Time timeout = ros::Time::now() + model_spawn_timeout;
  // wait and verify that model is deleted
  while (true)
  {
    if (ros::Time::now() > timeout)
    {
      res.success = false;
      res.status_message = std::string("DeleteModel: Model pushed to delete queue, but delete service timed out waiting for model to disappear from simulation");
      return true;
    }
    {
      //boost::recursive_mutex::scoped_lock lock(*world->GetMRMutex());
      if (!world_->GetModel(req.model_name)) break;
    }
    ROS_DEBUG("Waiting for model deletion (%s)",req.model_name.c_str());
    usleep(1000);
  }

  // set result
  res.success = true;
  res.status_message = std::string("DeleteModel: successfully deleted model");
  return true;
}

bool GazeboRosApiPlugin::getModelState(gazebo_msgs::GetModelState::Request &req,
                                       gazebo_msgs::GetModelState::Response &res)
{
  gazebo::physics::ModelPtr model = world_->GetModel(req.model_name);
  gazebo::physics::LinkPtr frame = boost::dynamic_pointer_cast<gazebo::physics::Link>(world_->GetEntity(req.relative_entity_name));
  if (!model)
  {
    ROS_ERROR("GetModelState: model [%s] does not exist",req.model_name.c_str());
    res.success = false;
    res.status_message = "GetModelState: model does not exist";
    return true;
  }
  else
  {
    // get model pose
    gazebo::math::Pose       model_pose = model->GetWorldPose();
    gazebo::math::Vector3    model_pos = model_pose.pos;
    gazebo::math::Quaternion model_rot = model_pose.rot;

    // get model twist
    gazebo::math::Vector3 model_linear_vel  = model->GetWorldLinearVel();
    gazebo::math::Vector3 model_angular_vel = model->GetWorldAngularVel();


    if (frame)
    {
      // convert to relative pose
      gazebo::math::Pose frame_pose = frame->GetWorldPose();
      model_pos = model_pos - frame_pose.pos;
      model_pos = frame_pose.rot.RotateVectorReverse(model_pos);
      model_rot *= frame_pose.rot.GetInverse();

      // convert to relative rates
      gazebo::math::Vector3 frame_vpos = frame->GetWorldLinearVel(); // get velocity in gazebo frame
      gazebo::math::Vector3 frame_veul = frame->GetWorldAngularVel(); // get velocity in gazebo frame
      model_linear_vel = frame_pose.rot.RotateVector(model_linear_vel - frame_vpos);
      model_angular_vel = frame_pose.rot.RotateVector(model_angular_vel - frame_veul);
    }
    /// @todo: FIXME map is really wrong, need to use tf here somehow
    else if (req.relative_entity_name == "" || req.relative_entity_name == "world" || req.relative_entity_name == "map" || req.relative_entity_name == "/map")
    {
      ROS_DEBUG("GetModelState: relative_entity_name is empty/world/map, using inertial frame");
    }
    else
    {
      res.success = false;
      res.status_message = "GetModelState: reference relative_entity_name not found, did you forget to scope the body by model name?";
      return true;
    }

    // fill in response
    res.pose.position.x = model_pos.x;
    res.pose.position.y = model_pos.y;
    res.pose.position.z = model_pos.z;
    res.pose.orientation.w = model_rot.w;
    res.pose.orientation.x = model_rot.x;
    res.pose.orientation.y = model_rot.y;
    res.pose.orientation.z = model_rot.z;

    res.twist.linear.x = model_linear_vel.x;
    res.twist.linear.y = model_linear_vel.y;
    res.twist.linear.z = model_linear_vel.z;
    res.twist.angular.x = model_angular_vel.x;
    res.twist.angular.y = model_angular_vel.y;
    res.twist.angular.z = model_angular_vel.z;

    res.success = true;
    res.status_message = "GetModelState: got properties";
    return true;
  }
  return true;
}

bool GazeboRosApiPlugin::getModelProperties(gazebo_msgs::GetModelProperties::Request &req,
                                            gazebo_msgs::GetModelProperties::Response &res)
{
  gazebo::physics::ModelPtr model = world_->GetModel(req.model_name);
  if (!model)
  {
    ROS_ERROR("GetModelProperties: model [%s] does not exist",req.model_name.c_str());
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
  res.sim_time = world_->GetSimTime().Double();
  res.model_names.clear();
  for (unsigned int i = 0; i < world_->GetModelCount(); i ++)
    res.model_names.push_back(world_->GetModel(i)->GetName());
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
  for (unsigned int i = 0; i < world_->GetModelCount(); i ++)
  {
    joint = world_->GetModel(i)->GetJoint(req.joint_name);
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

    res.position.clear(); // use GetAngle(i)
    res.position.push_back(joint->GetAngle(0).Radian());

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
  gazebo::physics::LinkPtr body = boost::dynamic_pointer_cast<gazebo::physics::Link>(world_->GetEntity(req.link_name));
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

    res.mass = body->GetInertial()->GetMass();

    gazebo::physics::InertialPtr inertia = body->GetInertial();
    res.ixx = inertia->GetIXX();
    res.iyy = inertia->GetIYY();
    res.izz = inertia->GetIZZ();
    res.ixy = inertia->GetIXY();
    res.ixz = inertia->GetIXZ();
    res.iyz = inertia->GetIYZ();

    gazebo::math::Vector3 com = body->GetInertial()->GetCoG();
    res.com.position.x = com.x;
    res.com.position.y = com.y;
    res.com.position.z = com.z;
    res.com.orientation.x = 0; // @todo: gazebo do not support rotated inertia yet
    res.com.orientation.y = 0;
    res.com.orientation.z = 0;
    res.com.orientation.w = 1;

    res.success = true;
    res.status_message = "GetLinkProperties: got properties";
    return true;
  }
}

bool GazeboRosApiPlugin::getLinkState(gazebo_msgs::GetLinkState::Request &req,
                                      gazebo_msgs::GetLinkState::Response &res)
{
  gazebo::physics::LinkPtr body = boost::dynamic_pointer_cast<gazebo::physics::Link>(world_->GetEntity(req.link_name));
  gazebo::physics::LinkPtr frame = boost::dynamic_pointer_cast<gazebo::physics::Link>(world_->GetEntity(req.reference_frame));

  if (!body)
  {
    res.success = false;
    res.status_message = "GetLinkState: link not found, did you forget to scope the link by model name?";
    return true;
  }

  // get body pose
  gazebo::math::Pose body_pose = body->GetWorldPose();
  // Get inertial rates
  gazebo::math::Vector3 body_vpos = body->GetWorldLinearVel(); // get velocity in gazebo frame
  gazebo::math::Vector3 body_veul = body->GetWorldAngularVel(); // get velocity in gazebo frame

  if (frame)
  {
    // convert to relative pose
    gazebo::math::Pose frame_pose = frame->GetWorldPose();
    body_pose.pos = body_pose.pos - frame_pose.pos;
    body_pose.pos = frame_pose.rot.RotateVectorReverse(body_pose.pos);
    body_pose.rot *= frame_pose.rot.GetInverse();

    // convert to relative rates
    gazebo::math::Vector3 frame_vpos = frame->GetWorldLinearVel(); // get velocity in gazebo frame
    gazebo::math::Vector3 frame_veul = frame->GetWorldAngularVel(); // get velocity in gazebo frame
    body_vpos = frame_pose.rot.RotateVector(body_vpos - frame_vpos);
    body_veul = frame_pose.rot.RotateVector(body_veul - frame_veul);
  }
  /// @todo: FIXME map is really wrong, need to use tf here somehow
  else if (req.reference_frame == "" || req.reference_frame == "world" || req.reference_frame == "map" || req.reference_frame == "/map")
  {
    ROS_DEBUG("GetLinkState: reference_frame is empty/world/map, using inertial frame");
  }
  else
  {
    res.success = false;
    res.status_message = "GetLinkState: reference reference_frame not found, did you forget to scope the link by model name?";
    return true;
  }

  res.link_state.link_name = req.link_name;
  res.link_state.pose.position.x = body_pose.pos.x;
  res.link_state.pose.position.y = body_pose.pos.y;
  res.link_state.pose.position.z = body_pose.pos.z;
  res.link_state.pose.orientation.x = body_pose.rot.x;
  res.link_state.pose.orientation.y = body_pose.rot.y;
  res.link_state.pose.orientation.z = body_pose.rot.z;
  res.link_state.pose.orientation.w = body_pose.rot.w;
  res.link_state.twist.linear.x = body_vpos.x;
  res.link_state.twist.linear.y = body_vpos.y;
  res.link_state.twist.linear.z = body_vpos.z;
  res.link_state.twist.angular.x = body_veul.x;
  res.link_state.twist.angular.y = body_veul.y;
  res.link_state.twist.angular.z = body_veul.x;
  res.link_state.reference_frame = req.reference_frame;

  res.success = true;
  res.status_message = "GetLinkState: got state";
  return true;
}

bool GazeboRosApiPlugin::setLinkProperties(gazebo_msgs::SetLinkProperties::Request &req,
                                           gazebo_msgs::SetLinkProperties::Response &res)
{
  gazebo::physics::LinkPtr body = boost::dynamic_pointer_cast<gazebo::physics::Link>(world_->GetEntity(req.link_name));
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
    // mass.SetInertiaRotation(gazebo::math::Quaternionion(req.com.orientation.w,res.com.orientation.x,req.com.orientation.y req.com.orientation.z));
    mass->SetCoG(gazebo::math::Vector3(req.com.position.x,req.com.position.y,req.com.position.z));
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

  // supported updates
  gazebo::physics::PhysicsEnginePtr pe = (world_->GetPhysicsEngine());
  pe->SetMaxStepSize(req.time_step);
  pe->SetRealTimeUpdateRate(req.max_update_rate);
  pe->SetGravity(gazebo::math::Vector3(req.gravity.x,req.gravity.y,req.gravity.z));

  if (world_->GetPhysicsEngine()->GetType() == "ode")
  {
    // stuff only works in ODE right now
    pe->SetAutoDisableFlag(req.ode_config.auto_disable_bodies);
#if GAZEBO_MAJOR_VERSION >= 3
    pe->SetParam("precon_iters", req.ode_config.sor_pgs_precon_iters);
    pe->SetParam("iters", req.ode_config.sor_pgs_iters);
    pe->SetParam("sor", req.ode_config.sor_pgs_w);
    pe->SetParam("cfm", req.ode_config.cfm);
    pe->SetParam("erp", req.ode_config.erp);
    pe->SetParam("contact_surface_layer",
        req.ode_config.contact_surface_layer);
    pe->SetParam("contact_max_correcting_vel",
        req.ode_config.contact_max_correcting_vel);
    pe->SetParam("max_contacts", req.ode_config.max_contacts);
#else
    pe->SetSORPGSPreconIters(req.ode_config.sor_pgs_precon_iters);
    pe->SetSORPGSIters(req.ode_config.sor_pgs_iters);
    pe->SetSORPGSW(req.ode_config.sor_pgs_w);
    pe->SetWorldCFM(req.ode_config.cfm);
    pe->SetWorldERP(req.ode_config.erp);
    pe->SetContactSurfaceLayer(req.ode_config.contact_surface_layer);
    pe->SetContactMaxCorrectingVel(req.ode_config.contact_max_correcting_vel);
    pe->SetMaxContacts(req.ode_config.max_contacts);
#endif

    world_->SetPaused(is_paused);

    res.success = true;
    res.status_message = "physics engine updated";
  }
  else
  {
    /// \TODO: add support for simbody, dart and bullet physics properties.
    ROS_ERROR("ROS set_physics_properties service call does not yet support physics engine [%s].", world_->GetPhysicsEngine()->GetType().c_str());
    res.success = false;
    res.status_message = "Physics engine [" + world_->GetPhysicsEngine()->GetType() + "]: set_physics_properties not supported.";
  }
  return res.success;
}

bool GazeboRosApiPlugin::getPhysicsProperties(gazebo_msgs::GetPhysicsProperties::Request &req,
                                              gazebo_msgs::GetPhysicsProperties::Response &res)
{
  // supported updates
  res.time_step = world_->GetPhysicsEngine()->GetMaxStepSize();
  res.pause = world_->IsPaused();
  res.max_update_rate = world_->GetPhysicsEngine()->GetRealTimeUpdateRate();
  gazebo::math::Vector3 gravity = world_->GetPhysicsEngine()->GetGravity();
  res.gravity.x = gravity.x;
  res.gravity.y = gravity.y;
  res.gravity.z = gravity.z;

  // stuff only works in ODE right now
  if (world_->GetPhysicsEngine()->GetType() == "ode")
  {
    res.ode_config.auto_disable_bodies =
      world_->GetPhysicsEngine()->GetAutoDisableFlag();
#if GAZEBO_MAJOR_VERSION >= 3
    res.ode_config.sor_pgs_precon_iters = boost::any_cast<int>(
      world_->GetPhysicsEngine()->GetParam("precon_iters"));
    res.ode_config.sor_pgs_iters = boost::any_cast<int>(
        world_->GetPhysicsEngine()->GetParam("iters"));
    res.ode_config.sor_pgs_w = boost::any_cast<double>(
        world_->GetPhysicsEngine()->GetParam("sor"));
    res.ode_config.contact_surface_layer = boost::any_cast<double>(
      world_->GetPhysicsEngine()->GetParam("contact_surface_layer"));
    res.ode_config.contact_max_correcting_vel = boost::any_cast<double>(
      world_->GetPhysicsEngine()->GetParam("contact_max_correcting_vel"));
    res.ode_config.cfm = boost::any_cast<double>(
        world_->GetPhysicsEngine()->GetParam("cfm"));
    res.ode_config.erp = boost::any_cast<double>(
        world_->GetPhysicsEngine()->GetParam("erp"));
    res.ode_config.max_contacts = boost::any_cast<int>(
      world_->GetPhysicsEngine()->GetParam("max_contacts"));
#else
    res.ode_config.sor_pgs_precon_iters = world_->GetPhysicsEngine()->GetSORPGSPreconIters();
    res.ode_config.sor_pgs_iters = world_->GetPhysicsEngine()->GetSORPGSIters();
    res.ode_config.sor_pgs_w = world_->GetPhysicsEngine()->GetSORPGSW();
    res.ode_config.contact_surface_layer = world_->GetPhysicsEngine()->GetContactSurfaceLayer();
    res.ode_config.contact_max_correcting_vel = world_->GetPhysicsEngine()->GetContactMaxCorrectingVel();
    res.ode_config.cfm = world_->GetPhysicsEngine()->GetWorldCFM();
    res.ode_config.erp = world_->GetPhysicsEngine()->GetWorldERP();
    res.ode_config.max_contacts = world_->GetPhysicsEngine()->GetMaxContacts();
#endif

    res.success = true;
    res.status_message = "GetPhysicsProperties: got properties";
  }
  else
  {
    /// \TODO: add support for simbody, dart and bullet physics properties.
    ROS_ERROR("ROS get_physics_properties service call does not yet support physics engine [%s].", world_->GetPhysicsEngine()->GetType().c_str());
    res.success = false;
    res.status_message = "Physics engine [" + world_->GetPhysicsEngine()->GetType() + "]: get_physics_properties not supported.";
  }
  return res.success;
}

bool GazeboRosApiPlugin::setJointProperties(gazebo_msgs::SetJointProperties::Request &req,
                                            gazebo_msgs::SetJointProperties::Response &res)
{
  /// @todo: current settings only allows for setting of 1DOF joints (e.g. HingeJoint and SliderJoint) correctly.
  gazebo::physics::JointPtr joint;
  for (unsigned int i = 0; i < world_->GetModelCount(); i ++)
  {
    joint = world_->GetModel(i)->GetJoint(req.joint_name);
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
#if GAZEBO_MAJOR_VERSION >= 4
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
#else
    for(unsigned int i=0;i< req.ode_joint_config.hiStop.size();i++)
      joint->SetAttribute("hi_stop",i,req.ode_joint_config.hiStop[i]);
    for(unsigned int i=0;i< req.ode_joint_config.loStop.size();i++)
      joint->SetAttribute("lo_stop",i,req.ode_joint_config.loStop[i]);
    for(unsigned int i=0;i< req.ode_joint_config.erp.size();i++)
      joint->SetAttribute("erp",i,req.ode_joint_config.erp[i]);
    for(unsigned int i=0;i< req.ode_joint_config.cfm.size();i++)
      joint->SetAttribute("cfm",i,req.ode_joint_config.cfm[i]);
    for(unsigned int i=0;i< req.ode_joint_config.stop_erp.size();i++)
      joint->SetAttribute("stop_erp",i,req.ode_joint_config.stop_erp[i]);
    for(unsigned int i=0;i< req.ode_joint_config.stop_cfm.size();i++)
      joint->SetAttribute("stop_cfm",i,req.ode_joint_config.stop_cfm[i]);
    for(unsigned int i=0;i< req.ode_joint_config.fudge_factor.size();i++)
      joint->SetAttribute("fudge_factor",i,req.ode_joint_config.fudge_factor[i]);
    for(unsigned int i=0;i< req.ode_joint_config.fmax.size();i++)
      joint->SetAttribute("fmax",i,req.ode_joint_config.fmax[i]);
    for(unsigned int i=0;i< req.ode_joint_config.vel.size();i++)
      joint->SetAttribute("vel",i,req.ode_joint_config.vel[i]);
#endif

    res.success = true;
    res.status_message = "SetJointProperties: properties set";
    return true;
  }
}

bool GazeboRosApiPlugin::setModelState(gazebo_msgs::SetModelState::Request &req,
                                       gazebo_msgs::SetModelState::Response &res)
{
  gazebo::math::Vector3 target_pos(req.model_state.pose.position.x,req.model_state.pose.position.y,req.model_state.pose.position.z);
  gazebo::math::Quaternion target_rot(req.model_state.pose.orientation.w,req.model_state.pose.orientation.x,req.model_state.pose.orientation.y,req.model_state.pose.orientation.z);
  target_rot.Normalize(); // eliminates invalid rotation (0, 0, 0, 0)
  gazebo::math::Pose target_pose(target_pos,target_rot);
  gazebo::math::Vector3 target_pos_dot(req.model_state.twist.linear.x,req.model_state.twist.linear.y,req.model_state.twist.linear.z);
  gazebo::math::Vector3 target_rot_dot(req.model_state.twist.angular.x,req.model_state.twist.angular.y,req.model_state.twist.angular.z);

  gazebo::physics::ModelPtr model = world_->GetModel(req.model_state.model_name);
  if (!model)
  {
    ROS_ERROR("Updating ModelState: model [%s] does not exist",req.model_state.model_name.c_str());
    res.success = false;
    res.status_message = "SetModelState: model does not exist";
    return true;
  }
  else
  {
    gazebo::physics::LinkPtr relative_entity = boost::dynamic_pointer_cast<gazebo::physics::Link>(world_->GetEntity(req.model_state.reference_frame));
    if (relative_entity)
    {
      gazebo::math::Pose  frame_pose = relative_entity->GetWorldPose(); // - myBody->GetCoMPose();
      gazebo::math::Vector3 frame_pos = frame_pose.pos;
      gazebo::math::Quaternion frame_rot = frame_pose.rot;

      //std::cout << " debug : " << relative_entity->GetName() << " : " << frame_pose << " : " << target_pose << std::endl;
      //target_pose = frame_pose + target_pose; // seems buggy, use my own
      target_pose.pos = model->GetWorldPose().pos + frame_rot.RotateVector(target_pos);
      target_pose.rot = frame_rot * target_pose.rot;

      // Velocities should be commanded in the requested reference
      // frame, so we need to translate them to the world frame
      target_pos_dot = frame_rot.RotateVector(target_pos_dot);
      target_rot_dot = frame_rot.RotateVector(target_rot_dot);
    }
    /// @todo: FIXME map is really wrong, need to use tf here somehow
    else if (req.model_state.reference_frame == "" || req.model_state.reference_frame == "world" || req.model_state.reference_frame == "map" || req.model_state.reference_frame == "/map" )
    {
      ROS_DEBUG("Updating ModelState: reference frame is empty/world/map, usig inertial frame");
    }
    else
    {
      ROS_ERROR("Updating ModelState: for model[%s], specified reference frame entity [%s] does not exist",
                req.model_state.model_name.c_str(),req.model_state.reference_frame.c_str());
      res.success = false;
      res.status_message = "SetModelState: specified reference frame entity does not exist";
      return true;
    }

    //ROS_ERROR("target state: %f %f %f",target_pose.pos.x,target_pose.pos.y,target_pose.pos.z);
    bool is_paused = world_->IsPaused();
    world_->SetPaused(true);
    model->SetWorldPose(target_pose);
    world_->SetPaused(is_paused);
    //gazebo::math::Pose p3d = model->GetWorldPose();
    //ROS_ERROR("model updated state: %f %f %f",p3d.pos.x,p3d.pos.y,p3d.pos.z);

    // set model velocity
    model->SetLinearVel(target_pos_dot);
    model->SetAngularVel(target_rot_dot);

    res.success = true;
    res.status_message = "SetModelState: set model state done";
    return true;
  }
}

void GazeboRosApiPlugin::updateModelState(const gazebo_msgs::ModelState::ConstPtr& model_state)
{
  gazebo_msgs::SetModelState::Response res;
  gazebo_msgs::SetModelState::Request req;
  req.model_state = *model_state;
  /*bool success =*/ setModelState(req,res);
}

bool GazeboRosApiPlugin::applyJointEffort(gazebo_msgs::ApplyJointEffort::Request &req,
                                          gazebo_msgs::ApplyJointEffort::Response &res)
{
  gazebo::physics::JointPtr joint;
  for (unsigned int i = 0; i < world_->GetModelCount(); i ++)
  {
    joint = world_->GetModel(i)->GetJoint(req.joint_name);
    if (joint)
    {
      GazeboRosApiPlugin::ForceJointJob* fjj = new GazeboRosApiPlugin::ForceJointJob;
      fjj->joint = joint;
      fjj->force = req.effort;
      fjj->start_time = req.start_time;
      if (fjj->start_time < ros::Time(world_->GetSimTime().Double()))
        fjj->start_time = ros::Time(world_->GetSimTime().Double());
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
      //ROS_ERROR("search %s %s",(*iter)->body->GetScopedName().c_str(), body_name.c_str());
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
  gazebo::physics::ModelPtr gazebo_model = world_->GetModel(req.model_name);
  if (!gazebo_model)
  {
    ROS_ERROR("SetModelConfiguration: model [%s] does not exist",gazebo_model_name.c_str());
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

bool GazeboRosApiPlugin::setLinkState(gazebo_msgs::SetLinkState::Request &req,
                                      gazebo_msgs::SetLinkState::Response &res)
{
  gazebo::physics::LinkPtr body = boost::dynamic_pointer_cast<gazebo::physics::Link>(world_->GetEntity(req.link_state.link_name));
  gazebo::physics::LinkPtr frame = boost::dynamic_pointer_cast<gazebo::physics::Link>(world_->GetEntity(req.link_state.reference_frame));
  if (!body)
  {
    ROS_ERROR("Updating LinkState: link [%s] does not exist",req.link_state.link_name.c_str());
    res.success = false;
    res.status_message = "SetLinkState: link does not exist";
    return true;
  }

  /// @todo: FIXME map is really wrong, unless using tf here somehow
  // get reference frame (body/model(link)) pose and
  // transform target pose to absolute world frame
  gazebo::math::Vector3 target_pos(req.link_state.pose.position.x,req.link_state.pose.position.y,req.link_state.pose.position.z);
  gazebo::math::Quaternion target_rot(req.link_state.pose.orientation.w,req.link_state.pose.orientation.x,req.link_state.pose.orientation.y,req.link_state.pose.orientation.z);
  gazebo::math::Pose target_pose(target_pos,target_rot);
  gazebo::math::Vector3 target_linear_vel(req.link_state.twist.linear.x,req.link_state.twist.linear.y,req.link_state.twist.linear.z);
  gazebo::math::Vector3 target_angular_vel(req.link_state.twist.angular.x,req.link_state.twist.angular.y,req.link_state.twist.angular.z);

  if (frame)
  {
    gazebo::math::Pose  frame_pose = frame->GetWorldPose(); // - myBody->GetCoMPose();
    gazebo::math::Vector3 frame_pos = frame_pose.pos;
    gazebo::math::Quaternion frame_rot = frame_pose.rot;

    //std::cout << " debug : " << frame->GetName() << " : " << frame_pose << " : " << target_pose << std::endl;
    //target_pose = frame_pose + target_pose; // seems buggy, use my own
    target_pose.pos = frame_pos + frame_rot.RotateVector(target_pos);
    target_pose.rot = frame_rot * target_pose.rot;

    gazebo::math::Vector3 frame_linear_vel = frame->GetWorldLinearVel();
    gazebo::math::Vector3 frame_angular_vel = frame->GetWorldAngularVel();
    target_linear_vel -= frame_linear_vel;
    target_angular_vel -= frame_angular_vel;
  }
  else if (req.link_state.reference_frame == "" || req.link_state.reference_frame == "world" || req.link_state.reference_frame == "map" || req.link_state.reference_frame == "/map")
  {
    ROS_INFO("Updating LinkState: reference_frame is empty/world/map, using inertial frame");
  }
  else
  {
    ROS_ERROR("Updating LinkState: reference_frame is not a valid link name");
    res.success = false;
    res.status_message = "SetLinkState: failed";
    return true;
  }

  //std::cout << " debug : " << target_pose << std::endl;
  //boost::recursive_mutex::scoped_lock lock(*world->GetMRMutex());

  bool is_paused = world_->IsPaused();
  if (!is_paused) world_->SetPaused(true);
  body->SetWorldPose(target_pose);
  world_->SetPaused(is_paused);

  // set body velocity to desired twist
  body->SetLinearVel(target_linear_vel);
  body->SetAngularVel(target_angular_vel);

  res.success = true;
  res.status_message = "SetLinkState: success";
  return true;
}

void GazeboRosApiPlugin::updateLinkState(const gazebo_msgs::LinkState::ConstPtr& link_state)
{
  gazebo_msgs::SetLinkState::Request req;
  gazebo_msgs::SetLinkState::Response res;
  req.link_state = *link_state;
  /*bool success = */ setLinkState(req,res);
}

void GazeboRosApiPlugin::transformWrench( gazebo::math::Vector3 &target_force, gazebo::math::Vector3 &target_torque,
                                          gazebo::math::Vector3 reference_force, gazebo::math::Vector3 reference_torque,
                                          gazebo::math::Pose target_to_reference )
{
  // rotate force into target frame
  target_force = target_to_reference.rot.RotateVector(reference_force);
  // rotate torque into target frame
  target_torque = target_to_reference.rot.RotateVector(reference_torque);

  // target force is the refence force rotated by the target->reference transform
  target_torque = target_torque + target_to_reference.pos.Cross(target_force);
}

bool GazeboRosApiPlugin::applyBodyWrench(gazebo_msgs::ApplyBodyWrench::Request &req,
                                         gazebo_msgs::ApplyBodyWrench::Response &res)
{
  gazebo::physics::LinkPtr body = boost::dynamic_pointer_cast<gazebo::physics::Link>(world_->GetEntity(req.body_name));
  gazebo::physics::LinkPtr frame = boost::dynamic_pointer_cast<gazebo::physics::Link>(world_->GetEntity(req.reference_frame));
  if (!body)
  {
    ROS_ERROR("ApplyBodyWrench: body [%s] does not exist",req.body_name.c_str());
    res.success = false;
    res.status_message = "ApplyBodyWrench: body does not exist";
    return true;
  }

  // target wrench
  gazebo::math::Vector3 reference_force(req.wrench.force.x,req.wrench.force.y,req.wrench.force.z);
  gazebo::math::Vector3 reference_torque(req.wrench.torque.x,req.wrench.torque.y,req.wrench.torque.z);
  gazebo::math::Vector3 reference_point(req.reference_point.x,req.reference_point.y,req.reference_point.z);

  gazebo::math::Vector3 target_force;
  gazebo::math::Vector3 target_torque;

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
    gazebo::math::Pose target_to_reference = frame->GetWorldPose() - body->GetWorldPose();
    ROS_DEBUG("reference frame for applied wrench: [%f %f %f, %f %f %f]-[%f %f %f, %f %f %f]=[%f %f %f, %f %f %f]",
              body->GetWorldPose().pos.x,
              body->GetWorldPose().pos.y,
              body->GetWorldPose().pos.z,
              body->GetWorldPose().rot.GetAsEuler().x,
              body->GetWorldPose().rot.GetAsEuler().y,
              body->GetWorldPose().rot.GetAsEuler().z,
              frame->GetWorldPose().pos.x,
              frame->GetWorldPose().pos.y,
              frame->GetWorldPose().pos.z,
              frame->GetWorldPose().rot.GetAsEuler().x,
              frame->GetWorldPose().rot.GetAsEuler().y,
              frame->GetWorldPose().rot.GetAsEuler().z,
              target_to_reference.pos.x,
              target_to_reference.pos.y,
              target_to_reference.pos.z,
              target_to_reference.rot.GetAsEuler().x,
              target_to_reference.rot.GetAsEuler().y,
              target_to_reference.rot.GetAsEuler().z
              );
    transformWrench(target_force, target_torque, reference_force, reference_torque, target_to_reference);
    ROS_ERROR("wrench defined as [%s]:[%f %f %f, %f %f %f] --> applied as [%s]:[%f %f %f, %f %f %f]",
              frame->GetName().c_str(),
              reference_force.x,
              reference_force.y,
              reference_force.z,
              reference_torque.x,
              reference_torque.y,
              reference_torque.z,
              body->GetName().c_str(),
              target_force.x,
              target_force.y,
              target_force.z,
              target_torque.x,
              target_torque.y,
              target_torque.z
              );

  }
  else if (req.reference_frame == "" || req.reference_frame == "world" || req.reference_frame == "map" || req.reference_frame == "/map")
  {
    ROS_INFO("ApplyBodyWrench: reference_frame is empty/world/map, using inertial frame, transferring from body relative to inertial frame");
    // FIXME: transfer to inertial frame
    gazebo::math::Pose target_to_reference = body->GetWorldPose();
    target_force = reference_force;
    target_torque = reference_torque;

  }
  else
  {
    ROS_ERROR("ApplyBodyWrench: reference_frame is not a valid link name");
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
  if (wej->start_time < ros::Time(world_->GetSimTime().Double()))
    wej->start_time = ros::Time(world_->GetSimTime().Double());
  wej->duration = req.duration;
  lock_.lock();
  wrench_body_jobs_.push_back(wej);
  lock_.unlock();

  res.success = true;
  res.status_message = "";
  return true;
}

bool GazeboRosApiPlugin::isURDF(std::string model_xml)
{
  TiXmlDocument doc_in;
  doc_in.Parse(model_xml.c_str());
  if (doc_in.FirstChild("robot"))
    return true;
  else
    return false;
}

bool GazeboRosApiPlugin::isSDF(std::string model_xml)
{
  // FIXME: very crude check
  TiXmlDocument doc_in;
  doc_in.Parse(model_xml.c_str());
  if (doc_in.FirstChild("gazebo") ||
      doc_in.FirstChild("sdf")) // sdf
    return true;
  else
    return false;
}

void GazeboRosApiPlugin::wrenchBodySchedulerSlot()
{
  // MDMutex locks in case model is getting deleted, don't have to do this if we delete jobs first
  // boost::recursive_mutex::scoped_lock lock(*world->GetMDMutex());
  lock_.lock();
  for (std::vector<GazeboRosApiPlugin::WrenchBodyJob*>::iterator iter=wrench_body_jobs_.begin();iter!=wrench_body_jobs_.end();)
  {
    // check times and apply wrench if necessary
    if (ros::Time(world_->GetSimTime().Double()) >= (*iter)->start_time)
      if (ros::Time(world_->GetSimTime().Double()) <= (*iter)->start_time+(*iter)->duration ||
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

    if (ros::Time(world_->GetSimTime().Double()) > (*iter)->start_time+(*iter)->duration &&
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
    if (ros::Time(world_->GetSimTime().Double()) >= (*iter)->start_time)
      if (ros::Time(world_->GetSimTime().Double()) <= (*iter)->start_time+(*iter)->duration ||
          (*iter)->duration.toSec() < 0.0)
      {
        if ((*iter)->joint) // if joint exists
          (*iter)->joint->SetForce(0,(*iter)->force);
        else
          (*iter)->duration.fromSec(0.0); // mark for delete
      }

    if (ros::Time(world_->GetSimTime().Double()) > (*iter)->start_time+(*iter)->duration &&
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
  ROS_ERROR("CLOCK2");
  gazebo::common::Time currentTime = gazebo::msgs::Convert( msg->sim_time() );
  rosgraph_msgs::Clock ros_time_;
  ros_time_.clock.fromSec(currentTime.Double());
  //  publish time to ros
  pub_clock_.publish(ros_time_);
}
void GazeboRosApiPlugin::publishSimTime()
{
  gazebo::common::Time currentTime = world_->GetSimTime();
  rosgraph_msgs::Clock ros_time_;
  ros_time_.clock.fromSec(currentTime.Double());
  //  publish time to ros
  pub_clock_.publish(ros_time_);
}

void GazeboRosApiPlugin::publishLinkStates()
{
  gazebo_msgs::LinkStates link_states;

  // fill link_states
  for (unsigned int i = 0; i < world_->GetModelCount(); i ++)
  {
    gazebo::physics::ModelPtr model = world_->GetModel(i);

    for (unsigned int j = 0 ; j < model->GetChildCount(); j ++)
    {
      gazebo::physics::LinkPtr body = boost::dynamic_pointer_cast<gazebo::physics::Link>(model->GetChild(j));

      if (body)
      {
        link_states.name.push_back(body->GetScopedName());
        geometry_msgs::Pose pose;
        gazebo::math::Pose  body_pose = body->GetWorldPose(); // - myBody->GetCoMPose();
        gazebo::math::Vector3 pos = body_pose.pos;
        gazebo::math::Quaternion rot = body_pose.rot;
        pose.position.x = pos.x;
        pose.position.y = pos.y;
        pose.position.z = pos.z;
        pose.orientation.w = rot.w;
        pose.orientation.x = rot.x;
        pose.orientation.y = rot.y;
        pose.orientation.z = rot.z;
        link_states.pose.push_back(pose);
        gazebo::math::Vector3 linear_vel  = body->GetWorldLinearVel();
        gazebo::math::Vector3 angular_vel = body->GetWorldAngularVel();
        geometry_msgs::Twist twist;
        twist.linear.x = linear_vel.x;
        twist.linear.y = linear_vel.y;
        twist.linear.z = linear_vel.z;
        twist.angular.x = angular_vel.x;
        twist.angular.y = angular_vel.y;
        twist.angular.z = angular_vel.z;
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
  for (unsigned int i = 0; i < world_->GetModelCount(); i ++)
  {
    gazebo::physics::ModelPtr model = world_->GetModel(i);
    model_states.name.push_back(model->GetName());
    geometry_msgs::Pose pose;
    gazebo::math::Pose  model_pose = model->GetWorldPose(); // - myBody->GetCoMPose();
    gazebo::math::Vector3 pos = model_pose.pos;
    gazebo::math::Quaternion rot = model_pose.rot;
    pose.position.x = pos.x;
    pose.position.y = pos.y;
    pose.position.z = pos.z;
    pose.orientation.w = rot.w;
    pose.orientation.x = rot.x;
    pose.orientation.y = rot.y;
    pose.orientation.z = rot.z;
    model_states.pose.push_back(pose);
    gazebo::math::Vector3 linear_vel  = model->GetWorldLinearVel();
    gazebo::math::Vector3 angular_vel = model->GetWorldAngularVel();
    geometry_msgs::Twist twist;
    twist.linear.x = linear_vel.x;
    twist.linear.y = linear_vel.y;
    twist.linear.z = linear_vel.z;
    twist.angular.x = angular_vel.x;
    twist.angular.y = angular_vel.y;
    twist.angular.z = angular_vel.z;
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
      ROS_INFO("physics dynamics reconfigure update complete");
    }
    ROS_INFO("physics dynamics reconfigure complete");
  }
}

void GazeboRosApiPlugin::physicsReconfigureThread()
{
  physics_reconfigure_set_client_ = nh_->serviceClient<gazebo_msgs::SetPhysicsProperties>("/gazebo/set_physics_properties");
  physics_reconfigure_get_client_ = nh_->serviceClient<gazebo_msgs::GetPhysicsProperties>("/gazebo/get_physics_properties");

  // Wait until the rest of this plugin is loaded and the services are being offered
  physics_reconfigure_set_client_.waitForExistence();
  physics_reconfigure_get_client_.waitForExistence();

  physics_reconfigure_srv_.reset(new dynamic_reconfigure::Server<gazebo_ros::PhysicsConfig>());

  physics_reconfigure_callback_ = boost::bind(&GazeboRosApiPlugin::physicsReconfigureCallback, this, _1, _2);
  physics_reconfigure_srv_->setCallback(physics_reconfigure_callback_);

  ROS_INFO("Physics dynamic reconfigure ready.");
}

void GazeboRosApiPlugin::stripXmlDeclaration(std::string &model_xml)
{
  // incoming robot model string is a string containing a Gazebo Model XML
  /// STRIP DECLARATION <? ... xml version="1.0" ... ?> from model_xml
  /// @todo: does tinyxml have functionality for this?
  /// @todo: should gazebo take care of the declaration?
  std::string open_bracket("<?");
  std::string close_bracket("?>");
  size_t pos1 = model_xml.find(open_bracket,0);
  size_t pos2 = model_xml.find(close_bracket,0);
  if (pos1 != std::string::npos && pos2 != std::string::npos)
    model_xml.replace(pos1,pos2-pos1+2,std::string(""));
}

void GazeboRosApiPlugin::updateSDFAttributes(TiXmlDocument &gazebo_model_xml, std::string model_name,
                                             gazebo::math::Vector3 initial_xyz, gazebo::math::Quaternion initial_q)
{
  // This function can handle both regular SDF files and <include> SDFs that are used with the
  // Gazebo Model Database

  TiXmlElement* pose_element; // This is used by both reguar and database SDFs

  // Check SDF for requires SDF element
  TiXmlElement* gazebo_tixml = gazebo_model_xml.FirstChildElement("sdf");
  if (!gazebo_tixml)
  {
    ROS_WARN("Could not find <sdf> element in sdf, so name and initial position cannot be applied");
    return;
  }

  // Check SDF for optional model element. May not have one
  TiXmlElement* model_tixml = gazebo_tixml->FirstChildElement("model");
  if (model_tixml)
  {
    // Update model name
    if (model_tixml->Attribute("name") != NULL)
    {
      // removing old model name
      model_tixml->RemoveAttribute("name");
    }
    // replace with user specified name
    model_tixml->SetAttribute("name",model_name);
  }
  else
  {
    // Check SDF for world element
    TiXmlElement* world_tixml = gazebo_tixml->FirstChildElement("world");
    if (!world_tixml)
    {
      ROS_WARN("Could not find <model> or <world> element in sdf, so name and initial position cannot be applied");
      return;
    }
    // If not <model> element, check SDF for required include element
    model_tixml = world_tixml->FirstChildElement("include");
    if (!model_tixml)
    {
      ROS_WARN("Could not find <include> element in sdf, so name and initial position cannot be applied");
      return;
    }

    // Check for name element
    TiXmlElement* name_tixml = model_tixml->FirstChildElement("name");
    if (!name_tixml)
    {
      // Create the name element
      name_tixml = new TiXmlElement("name");
      model_tixml->LinkEndChild(name_tixml);
    }

    // Set the text within the name element
    TiXmlText* text = new TiXmlText(model_name);
    name_tixml->LinkEndChild( text );
  }


  // Check for the pose element
  pose_element = model_tixml->FirstChildElement("pose");
  gazebo::math::Pose model_pose;

  // Create the pose element if it doesn't exist
  // Remove it if it exists, since we are inserting a new one
  if (pose_element)
  {
    // save pose_element in math::Pose and remove child
    model_pose = this->parsePose(pose_element->GetText());
    model_tixml->RemoveChild(pose_element);
  }

  // Set and link the pose element after adding initial pose
  {
    // add pose_element Pose to initial pose
    gazebo::math::Pose new_model_pose = model_pose + gazebo::math::Pose(initial_xyz, initial_q);

    // Create the string of 6 numbers
    std::ostringstream pose_stream;
    gazebo::math::Vector3 model_rpy = new_model_pose.rot.GetAsEuler(); // convert to Euler angles for Gazebo XML
    pose_stream << new_model_pose.pos.x << " " << new_model_pose.pos.y << " " << new_model_pose.pos.z << " "
                << model_rpy.x << " " << model_rpy.y << " " << model_rpy.z;

    // Add value to pose element
    TiXmlText* text = new TiXmlText(pose_stream.str());
    TiXmlElement* new_pose_element = new TiXmlElement("pose");
    new_pose_element->LinkEndChild(text);
    model_tixml->LinkEndChild(new_pose_element);
  }
}

gazebo::math::Pose GazeboRosApiPlugin::parsePose(const std::string &str)
{
  std::vector<std::string> pieces;
  std::vector<double> vals;

  boost::split(pieces, str, boost::is_any_of(" "));
  for (unsigned int i = 0; i < pieces.size(); ++i)
  {
    if (pieces[i] != "")
    {
      try
      {
        vals.push_back(boost::lexical_cast<double>(pieces[i].c_str()));
      }
      catch(boost::bad_lexical_cast &e)
      {
        sdferr << "xml key [" << str
          << "][" << i << "] value [" << pieces[i]
          << "] is not a valid double from a 3-tuple\n";
        return gazebo::math::Pose();
      }
    }
  }

  if (vals.size() == 6)
    return gazebo::math::Pose(vals[0], vals[1], vals[2], vals[3], vals[4], vals[5]);
  else
  {
    ROS_ERROR("Beware: failed to parse string [%s] as gazebo::math::Pose, returning zeros.", str.c_str());
    return gazebo::math::Pose();
  }
}

gazebo::math::Vector3 GazeboRosApiPlugin::parseVector3(const std::string &str)
{
  std::vector<std::string> pieces;
  std::vector<double> vals;

  boost::split(pieces, str, boost::is_any_of(" "));
  for (unsigned int i = 0; i < pieces.size(); ++i)
  {
    if (pieces[i] != "")
    {
      try
      {
        vals.push_back(boost::lexical_cast<double>(pieces[i].c_str()));
      }
      catch(boost::bad_lexical_cast &e)
      {
        sdferr << "xml key [" << str
          << "][" << i << "] value [" << pieces[i]
          << "] is not a valid double from a 3-tuple\n";
        return gazebo::math::Vector3();
      }
    }
  }

  if (vals.size() == 3)
    return gazebo::math::Vector3(vals[0], vals[1], vals[2]);
  else
  {
    ROS_ERROR("Beware: failed to parse string [%s] as gazebo::math::Vector3, returning zeros.", str.c_str());
    return gazebo::math::Vector3();
  }
}

void GazeboRosApiPlugin::updateURDFModelPose(TiXmlDocument &gazebo_model_xml, gazebo::math::Vector3 initial_xyz, gazebo::math::Quaternion initial_q)
{
  TiXmlElement* model_tixml = (gazebo_model_xml.FirstChildElement("robot"));
  if (model_tixml)
  {
    // replace initial pose of robot
    // find first instance of xyz and rpy, replace with initial pose
    TiXmlElement* origin_key = model_tixml->FirstChildElement("origin");

    if (!origin_key)
    {
      origin_key = new TiXmlElement("origin");
      model_tixml->LinkEndChild(origin_key);
    }

    gazebo::math::Vector3 xyz;
    gazebo::math::Vector3 rpy;
    if (origin_key->Attribute("xyz"))
    {
      xyz = this->parseVector3(origin_key->Attribute("xyz"));
      origin_key->RemoveAttribute("xyz");
    }
    if (origin_key->Attribute("rpy"))
    {
      rpy = this->parseVector3(origin_key->Attribute("rpy"));
      origin_key->RemoveAttribute("rpy");
    }

    // add xyz, rpy to initial pose
    gazebo::math::Pose model_pose = gazebo::math::Pose(xyz, rpy) + gazebo::math::Pose(initial_xyz, initial_q);

    std::ostringstream xyz_stream;
    xyz_stream << model_pose.pos.x << " " << model_pose.pos.y << " " << model_pose.pos.z;

    std::ostringstream rpy_stream;
    gazebo::math::Vector3 model_rpy = model_pose.rot.GetAsEuler(); // convert to Euler angles for Gazebo XML
    rpy_stream << model_rpy.x << " " << model_rpy.y << " " << model_rpy.z;

    origin_key->SetAttribute("xyz",xyz_stream.str());
    origin_key->SetAttribute("rpy",rpy_stream.str());
  }
  else
    ROS_WARN("could not find <model> element in sdf, so name and initial position is not applied");
}

void GazeboRosApiPlugin::updateURDFName(TiXmlDocument &gazebo_model_xml, std::string model_name)
{
  TiXmlElement* model_tixml = gazebo_model_xml.FirstChildElement("robot");
  // replace model name if one is specified by the user
  if (model_tixml)
  {
    if (model_tixml->Attribute("name") != NULL)
    {
      // removing old model name
      model_tixml->RemoveAttribute("name");
    }
    // replace with user specified name
    model_tixml->SetAttribute("name",model_name);
  }
  else
    ROS_WARN("could not find <robot> element in URDF, name not replaced");
}

void GazeboRosApiPlugin::walkChildAddRobotNamespace(TiXmlNode* robot_xml)
{
  TiXmlNode* child = 0;
  child = robot_xml->IterateChildren(child);
  while (child != NULL)
  {
    if (child->ValueStr().find(std::string("plugin")) == 0)
    {
      if (child->FirstChildElement("robotNamespace") == NULL)
      {
        TiXmlElement* child_elem = child->ToElement()->FirstChildElement("robotNamespace");
        while (child_elem)
        {
          child->ToElement()->RemoveChild(child_elem);
          child_elem = child->ToElement()->FirstChildElement("robotNamespace");
        }
        TiXmlElement* key = new TiXmlElement("robotNamespace");
        TiXmlText* val = new TiXmlText(robot_namespace_);
        key->LinkEndChild(val);
        child->ToElement()->LinkEndChild(key);
      }
    }
    walkChildAddRobotNamespace(child);
    child = robot_xml->IterateChildren(child);
  }
}

bool GazeboRosApiPlugin::spawnAndConform(TiXmlDocument &gazebo_model_xml, std::string model_name,
                                         gazebo_msgs::SpawnModel::Response &res)
{
  // push to factory iface
  std::ostringstream stream;
  stream << gazebo_model_xml;
  std::string gazebo_model_xml_string = stream.str();
  ROS_DEBUG("Gazebo Model XML\n\n%s\n\n ",gazebo_model_xml_string.c_str());

  // publish to factory topic
  gazebo::msgs::Factory msg;
  gazebo::msgs::Init(msg, "spawn_model");
  msg.set_sdf( gazebo_model_xml_string );

  //ROS_ERROR("attempting to spawn model name [%s] [%s]", model_name.c_str(),gazebo_model_xml_string.c_str());

  // FIXME: should use entity_info or add lock to World::receiveMutex
  // looking for Model to see if it exists already
  gazebo::msgs::Request *entity_info_msg = gazebo::msgs::CreateRequest("entity_info", model_name);
  request_pub_->Publish(*entity_info_msg,true);
  // todo: should wait for response response_sub_, check to see that if _msg->response == "nonexistant"

  gazebo::physics::ModelPtr model = world_->GetModel(model_name);
  if (model)
  {
    ROS_ERROR("SpawnModel: Failure - model name %s already exist.",model_name.c_str());
    res.success = false;
    res.status_message = "SpawnModel: Failure - model already exists.";
    return true;
  }

  // Publish the factory message
  factory_pub_->Publish(msg);
  /// FIXME: should change publish to direct invocation World::LoadModel() and/or
  ///        change the poll for Model existence to common::Events based check.

  /// \brief poll and wait, verify that the model is spawned within Hardcoded 10 seconds
  ros::Duration model_spawn_timeout(10.0);
  ros::Time timeout = ros::Time::now() + model_spawn_timeout;

  while (ros::ok())
  {
    if (ros::Time::now() > timeout)
    {
      res.success = false;
      res.status_message = std::string("SpawnModel: Model pushed to spawn queue, but spawn service")
        + std::string(" timed out waiting for model to appear in simulation under the name ")
        + model_name;
      return true;
    }

    {
      //boost::recursive_mutex::scoped_lock lock(*world->GetMRMutex());
      if (world_->GetModel(model_name))
        break;
    }

    ROS_DEBUG_STREAM_ONCE_NAMED("api_plugin","Waiting for " << timeout - ros::Time::now()
      << " for model " << model_name << " to spawn");

    usleep(2000);
  }

  // set result
  res.success = true;
  res.status_message = std::string("SpawnModel: Successfully spawned model");
  return true;
}

// Register this plugin with the simulator
GZ_REGISTER_SYSTEM_PLUGIN(GazeboRosApiPlugin)
}

