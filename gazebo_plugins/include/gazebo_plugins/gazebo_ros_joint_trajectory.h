/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003  
 *     Nate Koenig & Andrew Howard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/*
 * Desc: 3D position interface.
 * Author: Sachin Chitta and John Hsu
 * Date: 10 June 2008
 * SVN: $Id$
 */
#ifndef GAZEBO_ROS_JOINT_TRAJECTORY_PLUGIN_HH
#define GAZEBO_ROS_JOINT_TRAJECTORY_PLUGIN_HH

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include <ros/subscribe_options.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/Pose.h>

#include <gazebo_msgs/SetJointTrajectory.h>

#include "physics/physics.hh"
#include "transport/TransportTypes.hh"
#include "common/Time.hh"
#include "common/Plugin.hh"
#include "common/Events.hh"

#include <boost/thread.hpp>
#include "boost/thread/mutex.hpp"

namespace gazebo
{

   class GazeboRosJointTrajectory : public WorldPlugin
   {
      /// \brief Constructor
      public: GazeboRosJointTrajectory();

      /// \brief Destructor
      public: virtual ~GazeboRosJointTrajectory();

      /// \brief Load the controller
      public: void Load( physics::WorldPtr _world, sdf::ElementPtr _sdf );

      /// \brief Update the controller
      private: void SetTrajectory(const trajectory_msgs::JointTrajectory::ConstPtr& trajectory);
      private: bool SetTrajectory(const gazebo_msgs::SetJointTrajectory::Request& req,
                                  const gazebo_msgs::SetJointTrajectory::Response& res);
      private: void UpdateStates();

      private: physics::WorldPtr world_;
      private: physics::ModelPtr model_;

      /// \brief pose should be set relative to this link (default to "world")
      private: physics::LinkPtr reference_link_;
      private: std::string reference_link_name_;
      /// \brief frame transform name, should match link name
      //private: std::string tf_frame_name_;

      /// \brief pointer to ros node
      private: ros::NodeHandle* rosnode_;
      private: ros::Subscriber sub_;
      private: ros::ServiceServer srv_;
      private: bool has_trajectory_;

      /// \brief ros message
      private: trajectory_msgs::JointTrajectory trajectory_msg_;
      private: bool set_model_pose_;
      private: geometry_msgs::Pose model_pose_;

      /// \brief topic name
      private: std::string topic_name_;
      private: std::string service_name_;

      /// \brief A mutex to lock access to fields that are used in message callbacks
      private: boost::mutex update_mutex;

      /// \brief save last_time
      private: common::Time last_time_;

      // trajectory time control
      private: common::Time trajectory_start;
      private: unsigned int trajectory_index;

      // rate control
      private: double update_rate_;
      private: bool disable_physics_updates_;
      private: bool physics_engine_enabled_;

      /// \brief for setting ROS name space
      private: std::string robot_namespace_;

      private: ros::CallbackQueue queue_;
      private: void QueueThread();
      private: boost::thread callback_queue_thread_;
      
      // Pointer to the update event connection
      private: event::ConnectionPtr update_connection_;

      private: trajectory_msgs::JointTrajectory joint_trajectory_;

      void FixLink(physics::LinkPtr link);
      void UnfixLink();
      private: physics::JointPtr joint_;
   };

/** \} */
/// @}


}

#endif
