/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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
/*
 * Desc: External interfaces for Gazebo
 * Author: Nate Koenig, John Hsu, Dave Coleman
 * Date: 25 Apr 2010
 */

#ifndef __GAZEBO_ROS_API_PLUGIN_HH__
#define __GAZEBO_ROS_API_PLUGIN_HH__

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <errno.h>
#include <iostream>

#include <tinyxml.h>

#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/transport/transport.hh>

// ROS
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <ros/package.h>
#include <rosgraph_msgs/Clock.h>

// Services
#include "std_srvs/Empty.h"

#include "gazebo_msgs/SetPhysicsProperties.h"
#include "gazebo_msgs/GetPhysicsProperties.h"

// Topics
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"

// For model pose transform to set custom joint angles
#include <ros/ros.h>
#include <gazebo_msgs/SetModelConfiguration.h>
#include <boost/shared_ptr.hpp>

// For physics dynamics reconfigure
#include <dynamic_reconfigure/server.h>
#include <gazebo_ros/PhysicsConfig.h>
#include "gazebo_msgs/SetPhysicsProperties.h"
#include "gazebo_msgs/GetPhysicsProperties.h"

#include <boost/algorithm/string.hpp>

namespace gazebo
{

/// \brief A plugin loaded within the gzserver on startup.
class GazeboRosApiPlugin : public SystemPlugin
{
public:
  /// \brief Constructor
  GazeboRosApiPlugin();

  /// \brief Destructor
  ~GazeboRosApiPlugin();

  /// \bried Detect if sig-int shutdown signal is recieved
  void shutdownSignal();

  /// \brief Gazebo-inherited load function
  ///
  /// Called before Gazebo is loaded. Must not block.
  /// Capitalized per Gazebo cpp style guidelines
  /// \param _argc Number of command line arguments.
  /// \param _argv Array of command line arguments.
  void Load(int argc, char** argv);

  /// \brief ros queue thread for this node
  void gazeboQueueThread();

  /// \brief advertise services
  void advertiseServices();

private:

  /// \brief Used for the dynamic reconfigure callback function template
  void physicsReconfigureCallback(gazebo_ros::PhysicsConfig &config, uint32_t level);

  /// \brief waits for the rest of Gazebo to be ready before initializing the dynamic reconfigure services
  void physicsReconfigureThread();

  /// \brief Connect to Gazebo via its plugin interface, get a pointer to the world, start events
  void loadGazeboRosApiPlugin(std::string world_name);

  // track if the desconstructor event needs to occur
  bool plugin_loaded_;

  // detect if sigint event occurs
  bool stop_;
  gazebo::event::ConnectionPtr sigint_event_;

  gazebo::transport::NodePtr gazebonode_;

  boost::shared_ptr<ros::NodeHandle> nh_;
  ros::CallbackQueue gazebo_queue_;
  boost::shared_ptr<boost::thread> gazebo_callback_queue_thread_;

  gazebo::physics::WorldPtr world_;
  gazebo::event::ConnectionPtr time_update_event_;
  gazebo::event::ConnectionPtr load_gazebo_ros_api_plugin_event_;

  ros::ServiceServer set_physics_properties_service_;
  ros::ServiceServer get_physics_properties_service_;
  ros::ServiceServer set_model_configuration_service_;

  // ROS comm
  boost::shared_ptr<ros::AsyncSpinner> async_ros_spin_;

  // physics dynamic reconfigure
  boost::shared_ptr<boost::thread> physics_reconfigure_thread_;
  bool physics_reconfigure_initialized_;
  ros::ServiceClient physics_reconfigure_set_client_;
  ros::ServiceClient physics_reconfigure_get_client_;
  boost::shared_ptr< dynamic_reconfigure::Server<gazebo_ros::PhysicsConfig> > physics_reconfigure_srv_;
  dynamic_reconfigure::Server<gazebo_ros::PhysicsConfig>::CallbackType physics_reconfigure_callback_;

  bool world_created_;
};
}
#endif
