/*
 *  Copyright (C) 2014
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
 * Desc: Force Torque Sensor Plugin
 * Author: Francisco Suarez-Ruiz
 * Date: 5 June 2014
 */
 
#ifndef GAZEBO_ROS_FT_HH
#define GAZEBO_ROS_FT_HH

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>

#include <ros/ros.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <geometry_msgs/WrenchStamped.h>

namespace gazebo
{

/// \brief GazeboRosFT controller
/// This is a controller that simulates a 6 dof force sensor
class GazeboRosFT : public ModelPlugin
{
  /// \brief Constructor
  /// \param parent The parent entity must be a Model
  public: GazeboRosFT();

  /// \brief Destructor
  public: virtual ~GazeboRosFT();

  /// \brief Load the controller
  public: void Load( physics::ModelPtr _parent, sdf::ElementPtr _sdf );

  /// \brief Update the controller
  protected: virtual void UpdateChild();

  /// \brief A pointer to the Gazebo joint
  private: physics::JointPtr joint_;
  
  /// \brief A pointer to the Gazebo parent link
  private: physics::LinkPtr parent_link_;
  
  /// \brief A pointer to the Gazebo child link
  private: physics::LinkPtr child_link_;
  
  /// \brief A pointer to the Gazebo model
  private: physics::ModelPtr model_;
  
  /// \brief A pointer to the Gazebo world
  private: physics::WorldPtr world_;

  /// \brief A pointer to the ROS node.  A node will be instantiated if it does not exist.
  private: ros::NodeHandle* rosnode_;
  private: ros::Publisher pub_;

  /// \brief ROS WrenchStamped message
  private: geometry_msgs::WrenchStamped wrench_msg_;

  /// \brief store bodyname
  private: std::string joint_name_;

  /// \brief ROS WrenchStamped topic name
  private: std::string topic_name_;

  /// \brief ROS frame transform name to use in the image message header.
  /// FIXME: extract link name directly?
  private: std::string frame_name_;

  /// \brief for setting ROS name space
  private: std::string robot_namespace_;

  /// \brief A mutex to lock access to fields that are used in message callbacks
  private: boost::mutex lock_;
  
  /// \brief save last_time
  private: common::Time last_time_;
  
  // rate control
  private: double update_rate_;

  /// \brief: keep track of number of connections
  private: int ft_connect_count_;
  private: void FTConnect();
  private: void FTDisconnect();

  // Custom Callback Queue
  private: ros::CallbackQueue queue_;
  private: void QueueThread();
  private: boost::thread callback_queue_thread_;

  // Pointer to the update event connection
  private: event::ConnectionPtr update_connection_;

};

/** \} */
/// @}


}

#endif
