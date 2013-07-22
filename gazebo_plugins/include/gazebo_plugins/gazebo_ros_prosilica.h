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
 * Desc: A dynamic controller plugin that publishes ROS image topic for generic camera sensor.
 * Author: John Hsu
 * Date: 24 Sept 2008
 * SVN: $Id$
 */
#ifndef GAZEBO_ROS_PROSILICA_CAMERA_HH
#define GAZEBO_ROS_PROSILICA_CAMERA_HH

// library for processing camera data for gazebo / ros conversions
#include <gazebo_plugins/gazebo_ros_camera_utils.h>
#include <gazebo/plugins/DepthCameraPlugin.hh>

#include <ros/callback_queue.h>
#include "boost/thread/mutex.hpp"

// image components
#include "cv_bridge/cv_bridge.h"
// used by polled_camera
#include "sensor_msgs/RegionOfInterest.h"

// prosilica components
// Stuff in image_common
#include <image_transport/image_transport.h>
#include <polled_camera/publication_server.h>
#include <polled_camera/GetPolledImage.h>

namespace gazebo
{

class GazeboRosProsilica : public DepthCameraPlugin, GazeboRosCameraUtils
{
  /// \brief Constructor
  /// \param parent The parent entity, must be a Model or a Sensor
  public: GazeboRosProsilica();

  /// \brief Destructor
  public: virtual ~GazeboRosProsilica();

  /// \brief Load the controller
  /// \param node XML config node
  public: void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

  /// \brief does nothing for now
  private: static void mouse_cb(int event, int x, int y, int flags, void* param) { };

  /// \brief image_transport
  private: polled_camera::PublicationServer poll_srv_;      // Handles requests in polled mode

  private: std::string mode_;

  private: std::string mode_param_name;
/*
  /// \brief Service call to publish images, cam info
  private: bool camInfoService(prosilica_camera::CameraInfo::Request &req,
                               prosilica_camera::CameraInfo::Response &res);
  private: bool triggeredGrab(prosilica_camera::PolledImage::Request &req,
                              prosilica_camera::PolledImage::Response &res);
*/

  private: void pollCallback(polled_camera::GetPolledImage::Request& req,
                             polled_camera::GetPolledImage::Response& rsp,
                             sensor_msgs::Image& image, sensor_msgs::CameraInfo& info);

  /// \brief ros message
  /// \brief construct raw stereo message
  private: sensor_msgs::Image *roiImageMsg;
  private: sensor_msgs::CameraInfo *roiCameraInfoMsg;

  /// \brief ROS image topic name
  private: std::string pollServiceName;

  // subscribe to world stats
  //private: transport::NodePtr node;
  //private: transport::SubscriberPtr statsSub;
  //private: common::Time simTime;
  //public: void OnStats( const boost::shared_ptr<msgs::WorldStatistics const> &_msg);

  /// \brief Update the controller does nothing for depth
  protected: virtual void OnNewDepthFrame(const float *_image, 
                 unsigned int _width, unsigned int _height, 
                 unsigned int _depth, const std::string &_format) {};


  /// \brief Update the controller
  protected: virtual void OnNewImageFrame(const unsigned char *_image, 
                 unsigned int _width, unsigned int _height, 
                 unsigned int _depth, const std::string &_format);

};

}
#endif

