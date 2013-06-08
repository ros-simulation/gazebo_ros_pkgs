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
 * Desc: A dynamic controller plugin that publishes a ROS point cloud or laser scan topic for generic ray sensor.
 * Author: Mihai Emanuel Dolha
 * Date: 29 March 2012
 * SVN: $Id$
 */
#ifndef GAZEBO_ROS_GPU_LASER_HH
#define GAZEBO_ROS_GPU_LASER_HH

// ros stuff
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

// ros messages stuff
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>

#include <sensor_msgs/Image.h>
#include "image_transport/image_transport.h"

// gazebo stuff
#include "sdf/interface/Param.hh"
#include "physics/physics.hh"
#include "transport/TransportTypes.hh"
#include "msgs/MessageTypes.hh"
#include "common/Time.hh"
#include "sensors/SensorTypes.hh"
#include "plugins/GpuRayPlugin.hh"

// dynamic reconfigure stuff
#include <dynamic_reconfigure/server.h>

// boost stuff
#include "boost/thread/mutex.hpp"

namespace gazebo
{
  class GazeboRosGpuLaser : public GpuRayPlugin
  {
    /// \brief Constructor
    /// \param parent The parent entity, must be a Model or a Sensor
    public: GazeboRosGpuLaser();

    /// \brief Destructor
    public: ~GazeboRosGpuLaser();

    /// \brief Load the plugin
    /// \param take in SDF root element
    public: void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

    public: void Init();

    /// \brief Update the controller
    protected: virtual void OnNewLaserFrame(const float *_image,
                   unsigned int _width, unsigned int _height,
                   unsigned int _depth, const std::string &_format);

    /// \brief Update the controller
    //protected: virtual void OnNewImageFrame(const unsigned char *_image,
    //               unsigned int _width, unsigned int _height,
    //               unsigned int _depth, unsigned int cam);
    //protected: void PutCameraData(const unsigned char *_src, unsigned int w, unsigned int h, unsigned int d, image_transport::Publisher *pub_);
    //
    /////// \brief ROS image message
    //protected: image_transport::Publisher image_pub_;
    //protected: image_transport::Publisher image2_pub_;
    //protected: image_transport::Publisher image3_pub_;
    //protected: image_transport::Publisher image4_pub_;
    //private: image_transport::ImageTransport* itnode_;
    ///// \brief Keep track of number of connctions
    //protected: int imageConnectCount;
    //private: void ImageConnect();
    //private: void ImageDisconnect();

    protected: void PublishLaserScan(const float *_scan, unsigned int _width);

    protected: void PublishPointCloud(const float *_scan, unsigned int _width, unsigned int _height);

    /// \brief Gaussian noise
    private: double gaussian_noise_;

    /// \brief Gaussian noise generator
    private: double GaussianKernel(double mu,double sigma);

    /// \brief hack to mimic hokuyo intensity cutoff of 100
    //private: ParamT<double> *hokuyoMinIntensityP;
    private: double hokuyo_min_intensity_;

    /// \brief Keep track of number of connctions for point clouds
    private: int laser_connect_count_;
    private: void LaserConnect();
    private: void LaserDisconnect();

    /// \brief A pointer to the ROS node.  A node will be instantiated if it does not exist.
    private: ros::Publisher laser_scan_pub_;

    /// \brief PCL point cloud message
    private: pcl::PointCloud<pcl::PointXYZI> point_cloud_msg_;

    private: sensor_msgs::LaserScan laser_scan_msg_;

    private: double point_cloud_cutoff_;

    /// \brief ROS image topic name
    private: std::string laser_topic_name_;

    // overload with our own
    private: common::Time sensor_update_time_;

    protected: ros::NodeHandle* rosnode_;
    private: std::string robot_namespace_;

    protected: ros::CallbackQueue queue_;
    protected: void QueueThread();
    protected: boost::thread callback_queue_thread_;

    protected: ros::WallTime last_publish_;
    // protected: std::ofstream timelog_;
    // protected: unsigned int logCount_;

    protected: std::string frame_name_;
    protected: double update_rate_;
    protected: double update_period_;
  };

}
#endif

