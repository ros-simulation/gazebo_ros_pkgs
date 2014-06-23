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
#ifndef GAZEBO_ROS_TEMPLATE_HH
#define GAZEBO_ROS_TEMPLATE_HH

#include <ros/ros.h>

#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

namespace gazebo
{

  class GazeboRosHandOfGod : public ModelPlugin
  {
  /// \brief Constructor
  public: GazeboRosHandOfGod();

  /// \brief Destructor
  public: virtual ~GazeboRosHandOfGod();

  /// \brief Load the controller
  public: void Load( physics::ModelPtr _parent, sdf::ElementPtr _sdf );

  /// \brief Update the controller
  protected: virtual void GazeboUpdate();

  /// Pointer to the update event connection
  private: event::ConnectionPtr update_connection_;
           boost::shared_ptr<tf2_ros::Buffer> tf_buffer_;
           boost::shared_ptr<tf2_ros::TransformListener> tf_listener_;
           boost::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
           physics::ModelPtr model_;
           physics::LinkPtr floating_link_;
           std::string link_name_;
           std::string robot_namespace_;
           std::string frame_id_;
           double kl_, ka_;
           double cl_, ca_;
  };

}

#endif

