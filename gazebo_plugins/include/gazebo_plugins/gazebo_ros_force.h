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
 * Desc: A dynamic controller plugin that performs generic force interface.
 * Author: John Hsu
 * Date: 24 Sept 2008
 * SVN: $Id$
 */
#ifndef GAZEBO_ROS_FORCE_HH
#define GAZEBO_ROS_FORCE_HH

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>

#include <ros/ros.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include "physics/physics.hh"
#include "transport/TransportTypes.hh"
#include "common/Plugin.hh"
#include "common/Events.hh"

#include <geometry_msgs/Wrench.h>

namespace gazebo
{

/// @addtogroup gazebo_dynamic_plugins Gazebo ROS Dynamic Plugins
/// @{
/** \defgroup GazeboRosForce Plugin XML Reference and Example

  \brief Ros Force Controller.
  
  This is a controller that collects data from a ROS topic and applies wrench to a body accordingly.

  Example Usage:
  \verbatim
  <model:physical name="box_model">
    <body:empty name="box_body">
     ...
    </body:empty>
    <controller:gazebo_ros_force name="box_force_controller" plugin="libgazebo_ros_force.so">
        <alwaysOn>true</alwaysOn>
        <topicName>box_force</topicName>
        <bodyName>box_body</bodyName>
    </controller:gazebo_ros_force>
  </model:phyiscal>
  \endverbatim
 
\{
*/

/**
           .
 
*/

class GazeboRosForce : public ModelPlugin
{
  /// \brief Constructor
  /// \param parent The parent entity, must be a Model or a Sensor
  public: GazeboRosForce();

  /// \brief Destructor
  public: virtual ~GazeboRosForce();

  /// \brief Load the controller
  /// \param node XML config node
  protected: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

  /// \brief Update the controller
  protected: virtual void UpdateChild();

  /// \brief call back when a Wrench message is published
  private: void UpdateObjectForce(const geometry_msgs::Wrench::ConstPtr& _msg);

  private: physics::WorldPtr world_;

  /// \brief A pointer to the Link, where force is applied
  private: physics::LinkPtr link_;

  /// \brief A pointer to the ROS node.  A node will be instantiated if it does not exist.
  private: ros::NodeHandle* rosnode_;
  private: ros::Subscriber sub_;

  /// \brief A mutex to lock access to fields that are used in ROS message callbacks
  private: boost::mutex lock_;

  /// \brief ROS Wrench topic name
  /// \brief inputs
  private: std::string topic_name_;
  private: std::string link_name_;

  /// \brief for setting ROS name space
  private: std::string robot_namespace_;

  // Custom Callback Queue
  private: ros::CallbackQueue queue_;
  private: void QueueThread();
  private: boost::thread callback_queue_thread_;
  private: geometry_msgs::Wrench wrench_msg_;

  // Pointer to the update event connection
  private: event::ConnectionPtr update_connection_;
};

/** \} */
/// @}

}
#endif

