/*
 * Copyright 2018 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#ifndef GAZEBO_ROS_RAY_SENSOR_HH
#define GAZEBO_ROS_RAY_SENSOR_HH

#include <string>

#include <boost/bind.hpp>

#include <ros/ros.h>
#include <ros/advertise_options.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>

#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/msgs/MessageTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <gazebo/sensors/GpuRaySensor.hh>
#include <gazebo_plugins/gazebo_ros_utils.h>
#include <gazebo_plugins/PubQueue.h>

namespace gazebo
{
  class GazeboRosRaySensor : public SensorPlugin
  {
    /// \brief Constructor
    public: GazeboRosRaySensor();

    /// \brief Destructor
    public: ~GazeboRosRaySensor();

    /// \brief Load the plugin
    /// \param take in SDF root element
    public: void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Keep track of number of connctions
    private: int laser_connect_count_;
    private: void LaserConnect();
    private: void LaserDisconnect();

    /// \brief The parent sensor
    private: sensors::SensorPtr parent_ray_sensor_;

    /// \brief pointer to ros node
    private: ros::NodeHandlePtr rosnode_;
    private: ros::Publisher pub_;

    /// \brief topic name
    private: std::string topic_name_;

    /// \brief frame transform name, should match link name
    private: std::string frame_name_;

    /// \brief Enum for type of message to output
    private: enum OutputType
    {
      LASERSCAN,
      POINTCLOUD,
      POINTCLOUD2
    };
    /// \brief
    private: OutputType output_type_;

    private: template<typename T> void AdvertiseOutput();
    private: template<typename T> void SetParams(T sensor);
    private: void SubscribeGazeboLaserScan();

    /// \brief Resolve the tf frame from the SDF specified frame, robot namespace, and node handle.
    /// \detail DEPRECATED. In the future, the frame will simply by the frame specified in the SDF.
    ///         Overriden by other plugins to provide backwards compatibility to the inconsistent ways
    ///         of resolving the frame.
    protected: virtual std::string resolveTF(const std::string& _frame, const std::string& _robot_namespace, ros::NodeHandle& _nh);

    /// \brief for setting ROS name space
    private: std::string robot_namespace_;

    private: double min_intensity_;

    private: gazebo::transport::NodePtr gazebo_node_;
    private: gazebo::transport::SubscriberPtr laser_scan_sub_;
    private: void PublishLaserScan(ConstLaserScanStampedPtr &_msg);
    private: void PublishPointCloud(ConstLaserScanStampedPtr &_msg);
    private: void PublishPointCloud2(ConstLaserScanStampedPtr &_msg);

    private:
      int rayCount;
      int rangeCount;
      int verticalRayCount;
      int verticalRangeCount;
      double minAngle;
      double maxAngle;
      double verticalMinAngle;
      double verticalMaxAngle;
      double yDiff;
      double pDiff;
  };

  template<typename T>
  void GazeboRosRaySensor::AdvertiseOutput()
  {
    ros::AdvertiseOptions ao =
      ros::AdvertiseOptions::create<T>(this->topic_name_, 1,
        boost::bind(&GazeboRosRaySensor::LaserConnect, this),
        boost::bind(&GazeboRosRaySensor::LaserDisconnect, this),
        ros::VoidPtr(), nullptr);
    this->pub_ = this->rosnode_->advertise(ao);
  }

  template<typename T>
  void GazeboRosRaySensor::SetParams(T sensor)
  {
    minAngle = sensor->AngleMin().Radian();
    maxAngle = sensor->AngleMax().Radian();
    verticalMaxAngle = sensor->VerticalAngleMax().Radian();
    verticalMinAngle = sensor->VerticalAngleMin().Radian();
    rayCount = sensor->RayCount();
    rangeCount = sensor->RangeCount();
    verticalRayCount = sensor->VerticalRayCount();
    verticalRangeCount = sensor->VerticalRangeCount();
    yDiff = maxAngle - minAngle;
    pDiff = verticalMaxAngle - verticalMinAngle;
  }
}

#endif
