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
#ifndef GAZEBO_ROS_IMU_HH
#define GAZEBO_ROS_IMU_HH

#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

#include "physics/physics.hh"
#include "transport/TransportTypes.hh"
#include "msgs/MessageTypes.hh"
#include "common/Time.hh"
#include "common/Plugin.hh"
#include "common/Events.hh"

#include <ros/ros.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/bind.hpp>
#include <sensor_msgs/Imu.h>
#include <std_srvs/Empty.h>


namespace gazebo
{
   class GazeboRosIMU : public ModelPlugin
   {
      /// \brief Constructor
      public: GazeboRosIMU();

      /// \brief Destructor
      public: virtual ~GazeboRosIMU();

      /// \brief Load the controller
      /// \param node XML config node
      public: void Load( physics::ModelPtr _parent, sdf::ElementPtr _sdf );

      /// \brief Update the controller
      protected: virtual void UpdateChild();

      /// \brief The parent World
      private: physics::WorldPtr world_;

      /// \brief The link referred to by this plugin
      private: physics::LinkPtr link;

      /// \brief pointer to ros node
      private: ros::NodeHandle* rosnode_;
      private: ros::Publisher pub_;
      private: ros::Publisher deprecated_pub_;

      /// \brief ros message
      private: sensor_msgs::Imu imu_msg_;

      /// \brief store link name
      private: std::string link_name_;

      /// \brief topic name
      private: std::string topic_name_;

      /// \brief allow specifying constant xyz and rpy offsets
      private: math::Pose offset_;

      /// \brief A mutex to lock access to fields that are used in message callbacks
      private: boost::mutex lock_;

      /// \brief save last_time
      private: common::Time last_time_;
      private: math::Vector3 last_vpos_;
      private: math::Vector3 last_veul_;
      private: math::Vector3 apos_;
      private: math::Vector3 aeul_;

      /// \brief: keep initial pose to offset orientation in imu message
      private: math::Pose initial_pose_;

      /// \brief Gaussian noise
      private: double gaussian_noise_;

      /// \brief Gaussian noise generator
      private: double GaussianKernel(double mu,double sigma);

      /// \brief for setting ROS name space
      private: std::string robot_namespace_;

      /// \brief Keep track of number of connctions
      private: int imu_connect_count_;
      private: void IMUConnect();
      private: void IMUDisconnect();

      /// \brief call back when using service
      private: bool ServiceCallback(std_srvs::Empty::Request &req,
                                    std_srvs::Empty::Response &res);
      private: ros::ServiceServer srv_;
      private: std::string service_name_;

      private: ros::CallbackQueue imu_queue_;
      private: void IMUQueueThread();
      private: boost::thread callback_queue_thread_;

      // Pointer to the update event connection
      private: event::ConnectionPtr update_connection_;
   };

/** \} */
/// @}


}

#endif

