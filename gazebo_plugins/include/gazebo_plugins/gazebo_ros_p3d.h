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
#ifndef GAZEBO_ROS_P3D_HH
#define GAZEBO_ROS_P3D_HH

#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

#include "physics/physics.hh"
#include "transport/TransportTypes.hh"
#include "common/Time.hh"
#include "common/Plugin.hh"
#include "common/Events.hh"

#include <ros/ros.h>
#include <boost/thread.hpp>
#include "boost/thread/mutex.hpp"
#include <nav_msgs/Odometry.h>

namespace gazebo
{

   class GazeboRosP3D : public ModelPlugin
   {
      /// \brief Constructor
      public: GazeboRosP3D();

      /// \brief Destructor
      public: virtual ~GazeboRosP3D();

      /// \brief Load the controller
      public: void Load( physics::ModelPtr _parent, sdf::ElementPtr _sdf );

      /// \brief Update the controller
      protected: virtual void UpdateChild();

      private: physics::WorldPtr world_;
      private: physics::ModelPtr model_;

      /// \brief The parent Model
      private: physics::LinkPtr link_; //Gazebo Link

      /// \brief The body of the frame to display pose, twist
      private: physics::LinkPtr reference_link_;


      /// \brief pointer to ros node
      private: ros::NodeHandle* rosnode_;
      private: ros::Publisher pub_;

      /// \brief ros message
      private: nav_msgs::Odometry pose_msg_;

      /// \brief store bodyname
      private: std::string link_name_;

      /// \brief topic name
      private: std::string topic_name_;

      /// \brief frame transform name, should match link name
      /// FIXME: extract link name directly?
      private: std::string frame_name_;
      private: std::string tf_frame_name_;

      /// \brief allow specifying constant xyz and rpy offsets
      private: math::Pose offset_;

      /// \brief A mutex to lock access to fields that are used in message callbacks
      private: boost::mutex lock;

      /// \brief save last_time
      private: common::Time last_time_;
      private: math::Vector3 last_vpos_;
      private: math::Vector3 last_veul_;
      private: math::Vector3 apos_;
      private: math::Vector3 aeul_;
      private: math::Vector3 last_frame_vpos_;
      private: math::Vector3 last_frame_veul_;
      private: math::Vector3 frame_apos_;
      private: math::Vector3 frame_aeul_;

      // rate control
      private: double update_rate_;

      /// \brief Gaussian noise
      private: double gaussian_noise_;

      /// \brief Gaussian noise generator
      private: double GaussianKernel(double mu,double sigma);

      /// \brief for setting ROS name space
      private: std::string robot_namespace_;

      /// \brief Keep track of number of connctions
      private: int p3d_connect_count_;
      private: void P3DConnect();
      private: void P3DDisconnect();

      private: ros::CallbackQueue p3d_queue_;
      private: void P3DQueueThread();
      private: boost::thread callback_queue_thread_;
      
      // Pointer to the update event connection
      private: event::ConnectionPtr update_connection_;

   };

/** \} */
/// @}


}

#endif

