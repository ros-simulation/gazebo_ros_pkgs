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
 * Desc: A dynamic controller plugin that publishes ROS image_raw camera_info topic for generic camera sensor.
 * Author: John Hsu
 * Date: 24 Sept 2008
 * SVN: $Id$
 */
#ifndef GAZEBO_ROS_CAMERA_UTILS_HH
#define GAZEBO_ROS_CAMERA_UTILS_HH

// ros stuff
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

// ros messages stuff
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/Float64.h>
#include "image_transport/image_transport.h"

// gazebo stuff
#include "sdf/interface/Param.hh"
#include "physics/physics.hh"
#include "transport/TransportTypes.hh"
#include "msgs/MessageTypes.hh"
#include "common/Time.hh"
#include "sensors/SensorTypes.hh"
#include "plugins/CameraPlugin.hh"


// dynamic reconfigure stuff
#include <gazebo_plugins/GazeboRosCameraConfig.h>
#include <dynamic_reconfigure/server.h>

// boost stuff
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>


namespace gazebo
{

  class GazeboRosCameraUtils //: public CameraPlugin
  {
    /// \brief Constructor
    /// \param parent The parent entity, must be a Model or a Sensor
    public: GazeboRosCameraUtils();

    /// \brief Destructor
    public: ~GazeboRosCameraUtils();

    /// \brief Load the plugin
    /// \param take in SDF root element
    public: virtual void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

    private: void Init();

    /// \brief Put camera data to the ROS topic
    protected: void PutCameraData(const unsigned char *_src);
    protected: void PutCameraData(const unsigned char *_src, common::Time &last_update_time);

    /// \brief Keep track of number of connctions
    protected: int image_connect_count_;
    protected: void ImageConnect();
    protected: void ImageDisconnect();

    /// \brief: Camera modification functions
    private: void SetHFOV(const std_msgs::Float64::ConstPtr& hfov);
    private: void SetUpdateRate(const std_msgs::Float64::ConstPtr& update_rate);

    /// \brief A pointer to the ROS node.  A node will be instantiated if it does not exist.
    protected: ros::NodeHandle* rosnode_;
    protected: image_transport::Publisher image_pub_;
    private: image_transport::ImageTransport* itnode_;

    /// \brief ROS image message
    protected: sensor_msgs::Image image_msg_;

    /// \brief for setting ROS name space
    private: std::string robot_namespace_;

    /// \brief ROS camera name
    private: std::string camera_name_;

    /// \brief ROS image topic name
    protected: std::string image_topic_name_;

    /// \brief Publish CameraInfo to the ROS topic
    protected: virtual void PublishCameraInfo(ros::Publisher &camera_info_publisher);
    protected: virtual void PublishCameraInfo(common::Time &last_update_time);
    protected: virtual void PublishCameraInfo();
    /// \brief Keep track of number of connctions for CameraInfo
    protected: int info_connect_count_;
    private: void InfoConnect();
    private: void InfoDisconnect();
    /// \brief camera info
    protected: ros::Publisher camera_info_pub_;
    protected: std::string camera_info_topic_name_;
    protected: common::Time last_info_update_time_;

    /// \brief ROS frame transform name to use in the image message header.
    ///        This should typically match the link name the sensor is attached.
    protected: std::string frame_name_;
    /// update rate of this sensor
    protected: double update_rate_;
    protected: double update_period_;
    protected: common::Time last_update_time_;

    protected: double cx_prime_;
    protected: double cx_;
    protected: double cy_;
    protected: double focal_length_;
    protected: double hack_baseline_;
    protected: double distortion_k1_;
    protected: double distortion_k2_;
    protected: double distortion_k3_;
    protected: double distortion_t1_;
    protected: double distortion_t2_;

    /// \brief A mutex to lock access to fields that are used in ROS message callbacks
    protected: boost::mutex lock_;

    /// \brief size of image buffer
    protected: std::string type_;
    protected: int skip_;

    private: ros::Subscriber cameraHFOVSubscriber_;
    private: ros::Subscriber cameraUpdateRateSubscriber_;

    // Time last published, refrain from publish unless new image has been rendered
    // Allow dynamic reconfiguration of camera params
    dynamic_reconfigure::Server<gazebo_plugins::GazeboRosCameraConfig> *dyn_srv_;

    void configCallback(gazebo_plugins::GazeboRosCameraConfig &config, uint32_t level);

    protected: ros::CallbackQueue camera_queue_;
    protected: void CameraQueueThread();
    protected: boost::thread callback_queue_thread_;


    // copied from CameraPlugin
    protected: unsigned int width_, height_, depth_;
    protected: std::string format_;

    protected: sensors::SensorPtr parentSensor_;
    protected: rendering::CameraPtr camera_;

    // Pointer to the world
    protected: physics::WorldPtr world_;

    private: event::ConnectionPtr newFrameConnection_;

    protected: common::Time sensor_update_time_;

    // maintain for one more release for backwards compatibility with pr2_gazebo_plugins
    protected: int imageConnectCount;
    protected: int infoConnectCount;
    protected: physics::WorldPtr world;
  };

}
#endif

