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
 * Desc: A plugin which publishes the gazebo world state as a MoveIt! planning scene
 * Author: Jonathan Bohren
 * Date: 15 May 2014
 */
#ifndef GAZEBO_ROS_MOVEIT_PLANNING_SCENE_H
#define GAZEBO_ROS_MOVEIT_PLANNING_SCENE_H

#include <string>

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <geometry_msgs/Pose.h>

#include <ros/ros.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>

#include <std_srvs/Empty.h>
#include <moveit_msgs/PlanningScene.h>

namespace gazebo
{
/// @addtogroup gazebo_dynamic_plugins Gazebo ROS Dynamic Plugins
/// @{
/** \defgroup GazeboRosMoveItPlanningScene Plugin XML Reference and Example

  \brief Ros MoveIt Planning Scene Plugin.
  
  This is a model plugin which broadcasts MoveIt PlanningScene messages so
  that the planning scene stays up-to-date with the world simulation.  This is
  useful if you want to "fake" perfect perception of the environment.

  Example Usage:

  \verbatim
      <gazebo>
        <plugin filename="libgazebo_ros_moveit_planning_scene.so" name="gazebo_ros_moveit_planning_scene">
          <topicName>/planning_scene</topicName>
          <sceneName>laboratory</sceneName>
          <robotName>my_robot</robotName>
          <updatePeriod>0.5</updatePeriod>
        </plugin>
      </gazebo>
  \endverbatim

  Design:

  Note that while this is a _model_ plugin, its primary purpose is to advertize
  information about the gazebo world. It's designed as a model plugin, however,
  since that information is relevant to the context of a single robot's planner.
  As such, this plugin should be loaded in the model representing the robot
  performing the planning.

  At some period specified by <updatePeriod>, the plugin will publish the
  complete world state. If this period is 0, then periodic messages will not be
  published.

  You can manually re-synch the world state by calling the publish_planning_scene
  service.
\{
*/

/**
           .
 
*/

class GazeboRosMoveItPlanningScene : public ModelPlugin
{
  /// \brief Constructor
  public: GazeboRosMoveItPlanningScene();

  /// \brief Destructor
  public: virtual ~GazeboRosMoveItPlanningScene();

  // Documentation inherited
  protected: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

  // Documentation inherited
  protected: virtual void UpdateCB();

  protected: void subscriber_connected();

  /// \brief publish the complete planning scene
  private: bool PublishPlanningSceneCB(
               std_srvs::Empty::Request& req,
               std_srvs::Empty::Response& resp);

  /// \brief The custom callback queue thread function.
  private: void QueueThread();

  /// \brief A pointer to the gazebo world.
  private: physics::WorldPtr world_;

  /// \brief A pointer to the Model of the robot doing the planning
  private: physics::ModelPtr model_;

  /// \brief A pointer to the ROS node.  A node will be instantiated if it does not exist.
  private: boost::scoped_ptr<ros::NodeHandle> rosnode_;
  private: ros::Publisher planning_scene_pub_;
           ros::ServiceServer publish_planning_scene_service_;

  /// \brief A mutex to lock access to fields that are used in ROS message callbacks
  private: boost::mutex mutex_;

  /// \brief ROS topic name inputs
  private: std::string topic_name_;
  /// \brief The MoveIt scene name
  private: std::string scene_name_;
  private: std::string robot_name_;
  private: std::string model_name_;

  /// \brief for setting ROS name space
  private: std::string robot_namespace_;

  // Custom Callback Queue
  private: ros::CallbackQueue queue_;
  /// \brief Thead object for the running callback Thread.
  private: boost::thread callback_queue_thread_;
  /// \brief Container for the planning scene.
  private: moveit_msgs::PlanningScene planning_scene_msg_;
           std::map<std::string, moveit_msgs::CollisionObject> collision_object_map_;

  // Pointer to the update event connection
  private: event::ConnectionPtr update_connection_;
  private: event::ConnectionPtr add_connection_;
  private: event::ConnectionPtr delete_connection_;

  private:
           bool publish_full_scene_;

           ros::Duration publish_period_;
           ros::Time last_publish_time_;
};
/** \} */
/// @}
}
#endif
