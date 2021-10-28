// Copyright 2018 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <boost/make_shared.hpp>
#include <boost/variant.hpp>
#include <gazebo/transport/transport.hh>
#include <gazebo_plugins/gazebo_ros_ray_sensor.hpp>
#include <gazebo_ros/conversions/sensor_msgs.hpp>
#include <gazebo_ros/node.hpp>
#include <gazebo_ros/utils.hpp>
#include <rclcpp/rclcpp.hpp>

#include <string>
#include <algorithm>
#include <limits>
#include <memory>

namespace gazebo_plugins
{

class GazeboRosRaySensorPrivate
{
public:
  /// Node for ROS communication.
  gazebo_ros::Node::SharedPtr ros_node_;

  // Aliases
  using LaserScan = sensor_msgs::msg::LaserScan;
  using PointCloud = sensor_msgs::msg::PointCloud;
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using Range = sensor_msgs::msg::Range;
  using LaserScanPub = rclcpp::Publisher<LaserScan>::SharedPtr;
  using PointCloudPub = rclcpp::Publisher<PointCloud>::SharedPtr;
  using PointCloud2Pub = rclcpp::Publisher<PointCloud2>::SharedPtr;
  using RangePub = rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr;

  /// Publisher of output
  /// \todo use std::variant one c++17 is supported in ROS2
  boost::variant<LaserScanPub, PointCloudPub, PointCloud2Pub, RangePub> pub_;

  /// TF frame output is published in
  std::string frame_name_;

  /// Subscribe to gazebo's laserscan, calling the appropriate callback based on output type
  void SubscribeGazeboLaserScan();

  /// Publish a sensor_msgs/LaserScan message from a gazebo laser scan
  void PublishLaserScan(ConstLaserScanStampedPtr & _msg);

  /// Publish a sensor_msgs/PointCloud message from a gazebo laser scan
  void PublishPointCloud(ConstLaserScanStampedPtr & _msg);

  /// Publish a sensor_msgs/PointCloud2 message from a gazebo laser scan
  void PublishPointCloud2(ConstLaserScanStampedPtr & _msg);

  /// Publish a sensor_msgs/Range message from a gazebo laser scan
  void PublishRange(ConstLaserScanStampedPtr & _msg);

  /// Gazebo transport topic to subscribe to for laser scan
  std::string sensor_topic_;

  /// Minimum intensity value to publish for laser scan / pointcloud messages
  double min_intensity_{0.0};

  /// brief Radiation type to report when output type is range
  uint8_t range_radiation_type_;

  /// Gazebo node used to subscribe to laser scan
  gazebo::transport::NodePtr gazebo_node_;

  /// Gazebo subscribe to parent sensor's laser scan
  gazebo::transport::SubscriberPtr laser_scan_sub_;
};

GazeboRosRaySensor::GazeboRosRaySensor()
: impl_(std::make_unique<GazeboRosRaySensorPrivate>())
{
}

GazeboRosRaySensor::~GazeboRosRaySensor()
{
  // Must release subscriber and then call fini on node to remove it from topic manager.
  impl_->laser_scan_sub_.reset();
  if (impl_->gazebo_node_) {
    impl_->gazebo_node_->Fini();
  }
  impl_->gazebo_node_.reset();
}

void GazeboRosRaySensor::Load(gazebo::sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
  // Create ros_node configured from sdf
  impl_->ros_node_ = gazebo_ros::Node::Get(_sdf, _sensor);

  // Get QoS profiles
  const gazebo_ros::QoS & qos = impl_->ros_node_->get_qos();

  // Get QoS profile for the publisher
  rclcpp::QoS pub_qos = qos.get_publisher_qos("~/out", rclcpp::SensorDataQoS().reliable());

  // Get tf frame for output
  impl_->frame_name_ = gazebo_ros::SensorFrameID(*_sensor, *_sdf);

  // Get output type from sdf if provided
  if (!_sdf->HasElement("output_type")) {
    RCLCPP_WARN(
      impl_->ros_node_->get_logger(), "missing <output_type>, defaults to sensor_msgs/PointCloud2");
    impl_->pub_ = impl_->ros_node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      "~/out", pub_qos);
  } else {
    std::string output_type_string = _sdf->Get<std::string>("output_type");
    if (output_type_string == "sensor_msgs/LaserScan") {
      impl_->pub_ = impl_->ros_node_->create_publisher<sensor_msgs::msg::LaserScan>(
        "~/out", pub_qos);
    } else if (output_type_string == "sensor_msgs/PointCloud") {
      impl_->pub_ = impl_->ros_node_->create_publisher<sensor_msgs::msg::PointCloud>(
        "~/out", pub_qos);
    } else if (output_type_string == "sensor_msgs/PointCloud2") {
      impl_->pub_ = impl_->ros_node_->create_publisher<sensor_msgs::msg::PointCloud2>(
        "~/out", pub_qos);
    } else if (output_type_string == "sensor_msgs/Range") {
      impl_->pub_ = impl_->ros_node_->create_publisher<sensor_msgs::msg::Range>(
        "~/out", pub_qos);
    } else {
      RCLCPP_ERROR(
        impl_->ros_node_->get_logger(), "Invalid <output_type> [%s]", output_type_string.c_str());
      return;
    }
  }

  // Get parameters specific to Range output from sdf
  if (impl_->pub_.type() == typeid(GazeboRosRaySensorPrivate::RangePub)) {
    if (!_sdf->HasElement("radiation_type")) {
      RCLCPP_INFO(
        impl_->ros_node_->get_logger(), "missing <radiation_type>, defaulting to infrared");
      impl_->range_radiation_type_ = sensor_msgs::msg::Range::INFRARED;
    } else if ("ultrasound" == _sdf->Get<std::string>("radiation_type")) {
      impl_->range_radiation_type_ = sensor_msgs::msg::Range::ULTRASOUND;
    } else if ("infrared" == _sdf->Get<std::string>("radiation_type")) {
      impl_->range_radiation_type_ = sensor_msgs::msg::Range::INFRARED;
    } else {
      RCLCPP_ERROR(
        impl_->ros_node_->get_logger(),
        "Invalid <radiation_type> [%s]. Can be ultrasound or infrared",
        _sdf->Get<std::string>("radiation_type").c_str());
      return;
    }
  }

  if (!_sdf->HasElement("min_intensity")) {
    RCLCPP_DEBUG(
      impl_->ros_node_->get_logger(), "missing <min_intensity>, defaults to %f",
      impl_->min_intensity_);
  } else {
    impl_->min_intensity_ = _sdf->Get<double>("min_intensity");
  }

  // Create gazebo transport node and subscribe to sensor's laser scan
  impl_->gazebo_node_ = boost::make_shared<gazebo::transport::Node>();
  impl_->gazebo_node_->Init(_sensor->WorldName());

  // TODO(ironmig): use lazy publisher to only process laser data when output has a subscriber
  impl_->sensor_topic_ = _sensor->Topic();
  impl_->SubscribeGazeboLaserScan();
}

void GazeboRosRaySensorPrivate::SubscribeGazeboLaserScan()
{
  if (pub_.type() == typeid(LaserScanPub)) {
    laser_scan_sub_ = gazebo_node_->Subscribe(
      sensor_topic_, &GazeboRosRaySensorPrivate::PublishLaserScan, this);
  } else if (pub_.type() == typeid(PointCloudPub)) {
    laser_scan_sub_ = gazebo_node_->Subscribe(
      sensor_topic_, &GazeboRosRaySensorPrivate::PublishPointCloud, this);
  } else if (pub_.type() == typeid(PointCloud2Pub)) {
    laser_scan_sub_ = gazebo_node_->Subscribe(
      sensor_topic_, &GazeboRosRaySensorPrivate::PublishPointCloud2, this);
  } else if (pub_.type() == typeid(RangePub)) {
    laser_scan_sub_ = gazebo_node_->Subscribe(
      sensor_topic_, &GazeboRosRaySensorPrivate::PublishRange, this);
  } else {
    RCLCPP_ERROR(ros_node_->get_logger(), "Publisher is an invalid type. This is an internal bug.");
  }
}

void GazeboRosRaySensorPrivate::PublishLaserScan(ConstLaserScanStampedPtr & _msg)
{
  // Convert Laser scan to ROS LaserScan
  auto ls = gazebo_ros::Convert<sensor_msgs::msg::LaserScan>(*_msg);
  // Set tf frame
  ls.header.frame_id = frame_name_;
  // Publish output
  boost::get<LaserScanPub>(pub_)->publish(ls);
}

void GazeboRosRaySensorPrivate::PublishPointCloud(ConstLaserScanStampedPtr & _msg)
{
  // Convert Laser scan to PointCloud
  auto pc = gazebo_ros::Convert<sensor_msgs::msg::PointCloud>(*_msg, min_intensity_);
  // Set tf frame
  pc.header.frame_id = frame_name_;
  // Publish output
  boost::get<PointCloudPub>(pub_)->publish(pc);
}

void GazeboRosRaySensorPrivate::PublishPointCloud2(ConstLaserScanStampedPtr & _msg)
{
  // Convert Laser scan to PointCloud2
  auto pc2 = gazebo_ros::Convert<sensor_msgs::msg::PointCloud2>(*_msg, min_intensity_);
  // Set tf frame
  pc2.header.frame_id = frame_name_;
  // Publish output
  boost::get<PointCloud2Pub>(pub_)->publish(pc2);
}

void GazeboRosRaySensorPrivate::PublishRange(ConstLaserScanStampedPtr & _msg)
{
  // Convert Laser scan to range
  auto range_msg = gazebo_ros::Convert<sensor_msgs::msg::Range>(*_msg);
  // Set tf frame
  range_msg.header.frame_id = frame_name_;
  // Set radiation type from sdf
  range_msg.radiation_type = range_radiation_type_;
  // Publish output
  boost::get<RangePub>(pub_)->publish(range_msg);
}

// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboRosRaySensor)

}  // namespace gazebo_plugins
