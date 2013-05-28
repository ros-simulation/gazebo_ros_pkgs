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
 * Desc: ROS laser controller.
 * Author: John Hsu
 * Date: 24 Sept 2007
 * SVN: $Id$
 */

#ifndef GAZEBO_ROS_LASER_HH
#define GAZEBO_ROS_LASER_HH

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

#include "sdf/interface/Param.hh"
#include "physics/physics.hh"
#include "transport/TransportTypes.hh"
#include "msgs/MessageTypes.hh"
#include "common/Time.hh"
#include "common/Plugin.hh"
#include "common/Events.hh"
#include "sensors/SensorTypes.hh"
#include "plugins/RayPlugin.hh"

#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <sensor_msgs/LaserScan.h>

namespace gazebo
{
  class GazeboRosLaser : public RayPlugin
  {
    /// \brief Constructor
    public: GazeboRosLaser();

    /// \brief Destructor
    public: ~GazeboRosLaser();

    /// \brief Load the plugin
    /// \param take in SDF root element
    public: void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Update the controller
    protected: virtual void OnNewLaserScans();

    /// \brief Put laser data to the ROS topic
    private: void PutLaserData(common::Time &_updateTime);

    /// \brief Keep track of number of connctions
    private: int laser_connect_count_;
    private: void LaserConnect();
    private: void LaserDisconnect();

    // Pointer to the model
    private: physics::WorldPtr world_;
    /// \brief The parent sensor
    private: sensors::SensorPtr parent_sensor_;
    private: sensors::RaySensorPtr parent_ray_sensor_;

    /// \brief pointer to ros node
    private: ros::NodeHandle* rosnode_;
    private: ros::Publisher pub_;

    /// \brief ros message
    private: sensor_msgs::LaserScan laser_msg_;
   
    /// \brief topic name
    private: std::string topic_name_;
    //private: ParamT<std::string> *topicNameP;

    /// \brief frame transform name, should match link name
    private: std::string frame_name_;

    /// \brief Gaussian noise
    private: double gaussian_noise_;
    //private: ParamT<double> *gaussianNoiseP;

    /// \brief Gaussian noise generator
    private: double GaussianKernel(double mu,double sigma);

    /// \brief A mutex to lock access to fields that are used in message callbacks
    private: boost::mutex lock_;

    /// \brief hack to mimic hokuyo intensity cutoff of 100
    //private: ParamT<double> *hokuyoMinIntensityP;
    private: double hokuyo_min_intensity_;

    /// update rate of this sensor
    private: double update_rate_;
    private: double update_period_;
    private: common::Time last_update_time_;

    /// \brief for setting ROS name space
    private: std::string robot_namespace_;

    private: ros::CallbackQueue laser_queue_;
    private: void LaserQueueThread();
    private: boost::thread callback_queue_thread_;
  };

}

#endif

