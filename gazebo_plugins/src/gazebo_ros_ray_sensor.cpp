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

#include <gazebo_plugins/gazebo_ros_ray_sensor.hpp>
#include <gazebo/transport/transport.hh>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <gazebo_ros/utils.hpp>

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
  gazebo_ros::Node::SharedPtr rosnode_;

  /// Sensor this plugin is attached to
  boost::variant<gazebo::sensors::RaySensorPtr, gazebo::sensors::GpuRaySensorPtr> sensor_;

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

  /// Store sensor constants like min/max angle, templated for either RaySensor or GpuRaySensor
  template<typename T>
  void SetParams(T sensor);

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
  double min_intensity_;

  /// brief Radiation type to report when output type is range
  uint8_t range_radiation_type_;

  /// Gazebo node used to subscribe to laser scan
  gazebo::transport::NodePtr gazebo_node_;
  /// Gazebo subscribe to parent sensor's laser scan
  gazebo::transport::SubscriberPtr laser_scan_sub_;

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
  double rangeMin;
  double rangeMax;
};

GazeboRosRaySensor::GazeboRosRaySensor()
: impl_(std::make_unique<GazeboRosRaySensorPrivate>())
{
}

GazeboRosRaySensor::~GazeboRosRaySensor()
{
}

void GazeboRosRaySensor::Load(gazebo::sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{
  using gazebo::sensors::RaySensor;
  using gazebo::sensors::RaySensorPtr;
  using gazebo::sensors::GpuRaySensor;
  using gazebo::sensors::GpuRaySensorPtr;

  // Create rosnode configured from sdf
  impl_->rosnode_ = gazebo_ros::Node::Create("gazebo_ray_sensor", _sdf);

  // Set constants from either RaySensor or GpuRaySensor
  if (_parent->Type() == "ray") {
    auto ray_sensor = std::dynamic_pointer_cast<RaySensor>(_parent);
    if (!ray_sensor) {
      RCLCPP_ERROR(impl_->rosnode_->get_logger(), "Could not cast to ray sensor. Exiting.");
      return;
    }
    impl_->SetParams<RaySensorPtr>(ray_sensor);
    impl_->sensor_ = ray_sensor;
  } else if (_parent->Type() == "gpu_ray") {
    auto gpu_sensor = std::dynamic_pointer_cast<GpuRaySensor>(_parent);
    if (!gpu_sensor) {
      RCLCPP_ERROR(impl_->rosnode_->get_logger(), "Could not cast to gpu_ray sensor. Exiting.");
      return;
    }
    impl_->SetParams<GpuRaySensorPtr>(gpu_sensor);
    impl_->sensor_ = gpu_sensor;
  } else {
    RCLCPP_ERROR(
      impl_->rosnode_->get_logger(),
      "Plugin attached to a sensor that is neither ray or gpu_ray. Exiting.");
    return;
  }

  // Get tf frame form sdf if provided
  if (!_sdf->HasElement("frameName")) {
    // Frame defaults to link of parent
    impl_->frame_name_ = gazebo_ros::ScopedNameBase(_parent->ParentName());
  } else {
    impl_->frame_name_ = _sdf->Get<std::string>("frameName");
  }

  // Get output type from sdf if provided
  if (!_sdf->HasElement("outputType")) {
    RCLCPP_WARN(
      impl_->rosnode_->get_logger(), "missing <outputType>, defaults to sensor_msgs/PointCloud2");
    impl_->pub_ = impl_->rosnode_->create_publisher<sensor_msgs::msg::PointCloud2>("~/out");
  } else {
    std::string output_type_string = _sdf->Get<std::string>("outputType");
    if (output_type_string == "sensor_msgs/LaserScan") {
      impl_->pub_ = impl_->rosnode_->create_publisher<sensor_msgs::msg::LaserScan>("~/out");
    } else if (output_type_string == "sensor_msgs/PointCloud") {
      impl_->pub_ = impl_->rosnode_->create_publisher<sensor_msgs::msg::PointCloud>("~/out");
    } else if (output_type_string == "sensor_msgs/PointCloud2") {
      impl_->pub_ = impl_->rosnode_->create_publisher<sensor_msgs::msg::PointCloud2>("~/out");
    } else if (output_type_string == "sensor_msgs/Range") {
      impl_->pub_ = impl_->rosnode_->create_publisher<sensor_msgs::msg::Range>("~/out");
    } else {
      RCLCPP_ERROR(impl_->rosnode_->get_logger(), "Invalid <outputType> [%s]", output_type_string);
      return;
    }
  }

  // Get parameters specific to Range output from sdf
  if (impl_->pub_.type() == typeid(GazeboRosRaySensorPrivate::RangePub)) {
    if (!_sdf->HasElement("radiationType")) {
      RCLCPP_INFO(impl_->rosnode_->get_logger(), "missing <radiationType>, defaulting to infared");
      impl_->range_radiation_type_ = sensor_msgs::msg::Range::INFRARED;
    } else if ("ultrasound" == _sdf->Get<std::string>("radiationType")) {
      impl_->range_radiation_type_ = sensor_msgs::msg::Range::ULTRASOUND;
    } else if ("infared" == _sdf->Get<std::string>("radiationType")) {
      impl_->range_radiation_type_ = sensor_msgs::msg::Range::INFRARED;
    } else {
      RCLCPP_ERROR(
        impl_->rosnode_->get_logger(), "Invalid <radiationType> [%s]. Can be ultrasound or infared",
        _sdf->Get<std::string>("radiationType").c_str());
      return;
    }
  }

  if (!_sdf->HasElement("minIntensity")) {
    impl_->min_intensity_ = 0.0;
    RCLCPP_DEBUG(impl_->rosnode_->get_logger(), "missing <minIntensity>, defaults to %f",
      impl_->min_intensity_);
  } else {
    impl_->min_intensity_ = _sdf->Get<double>("minIntensity");
  }


  // Create gazebo tranport node and subscribe to sensor's laser scan
  impl_->gazebo_node_ = boost::make_shared<gazebo::transport::Node>();
  impl_->gazebo_node_->Init(_parent->WorldName());
  // TODO(ironmig): use lazy publisher to only process laser data when output has a subscriber
  impl_->SubscribeGazeboLaserScan();
}

template<typename T>
void GazeboRosRaySensorPrivate::SetParams(T sensor)
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
  rangeMin = sensor->RangeMin();
  rangeMax = sensor->RangeMax();
  sensor_topic_ = sensor->Topic();
}

void GazeboRosRaySensorPrivate::SubscribeGazeboLaserScan()
{
  if (pub_.type() == typeid(LaserScanPub)) {
    laser_scan_sub_ = gazebo_node_->Subscribe(sensor_topic_,
        &GazeboRosRaySensorPrivate::PublishLaserScan, this);
  } else if (pub_.type() == typeid(PointCloudPub)) {
    laser_scan_sub_ = gazebo_node_->Subscribe(sensor_topic_,
        &GazeboRosRaySensorPrivate::PublishPointCloud, this);
  } else if (pub_.type() == typeid(PointCloud2Pub)) {
    laser_scan_sub_ = gazebo_node_->Subscribe(sensor_topic_,
        &GazeboRosRaySensorPrivate::PublishPointCloud2, this);
  } else if (pub_.type() == typeid(RangePub)) {
    laser_scan_sub_ = gazebo_node_->Subscribe(sensor_topic_,
        &GazeboRosRaySensorPrivate::PublishRange, this);
  } else {
    RCLCPP_ERROR(rosnode_->get_logger(), "Publisher is an invalid type. This is an internal bug.");
  }
}

void GazeboRosRaySensorPrivate::PublishLaserScan(ConstLaserScanStampedPtr & _msg)
{
  sensor_msgs::msg::LaserScan ls;
  ls.header.frame_id = frame_name_;
  ls.header.stamp.sec = _msg->time().sec();
  ls.header.stamp.nanosec = _msg->time().nsec();
  ls.angle_min = minAngle;
  ls.angle_max = maxAngle;
  ls.angle_increment = _msg->scan().angle_step();
  ls.time_increment = 0;
  ls.scan_time = 0;
  ls.range_min = _msg->scan().range_min();
  ls.range_max = _msg->scan().range_max();

  // If there are multiple vertical beams, use the one in the middle
  size_t start = (verticalRangeCount / 2) * rangeCount;

  // Copy ranges and intensities over
  ls.ranges.resize(rangeCount);
  ls.intensities.resize(rangeCount);
  std::copy(_msg->scan().ranges().begin() + start,
    _msg->scan().ranges().begin() + start + rangeCount,
    ls.ranges.begin());
  std::copy(_msg->scan().intensities().begin() + start,
    _msg->scan().intensities().begin() + start + rangeCount,
    ls.intensities.begin());

  boost::get<LaserScanPub>(pub_)->publish(ls);
}

void GazeboRosRaySensorPrivate::PublishPointCloud(ConstLaserScanStampedPtr & _msg)
{
  // Create message to send
  sensor_msgs::msg::PointCloud pc;

  // Fill header
  pc.header.frame_id = frame_name_;
  pc.header.stamp.sec = _msg->time().sec();
  pc.header.stamp.nanosec = _msg->time().nsec();

  // Setup point cloud fields
  pc.points.reserve(verticalRangeCount * rangeCount);
  pc.channels.push_back(sensor_msgs::msg::ChannelFloat32());
  pc.channels[0].values.reserve(verticalRangeCount * rangeCount);
  pc.channels[0].name = "intensity";

  // Fill pointcloud with laser scan
  for (int i = 0; i < rangeCount; i++) {
    // Calculate azimuth (horizontal angle)
    double azimuth;
    if (rangeCount > 1) {
      azimuth = i * yDiff / (rangeCount - 1) + minAngle;
    } else {
      azimuth = minAngle;
    }
    double c_azimuth = cos(azimuth);
    double s_azimuth = sin(azimuth);

    for (int j = 0; j < verticalRangeCount; ++j) {
      double inclination;
      // Calculate inclination (vertical angle)
      if (verticalRayCount > 1) {
        inclination = j * pDiff / (verticalRangeCount - 1) + verticalMinAngle;
      } else {
        inclination = verticalMinAngle;
      }
      double c_inclination = cos(inclination);
      double s_inclination = sin(inclination);

      // Get range and intensity at this scan
      size_t index = i + j * rangeCount;
      double r = _msg->scan().ranges(index);
      double intensity = _msg->scan().intensities(index);
      if (intensity < this->min_intensity_) {
        intensity = this->min_intensity_;
      }

      // Convert spherical coordinates to cartesian for pointcloud
      // See https://en.wikipedia.org/wiki/Spherical_coordinate_system
      geometry_msgs::msg::Point32 point;
      point.x = r * c_inclination * c_azimuth;
      point.y = r * c_inclination * s_azimuth;
      point.z = r * s_inclination;
      pc.points.push_back(point);
      pc.channels[0].values.push_back(intensity);
    }
  }

  // Publish output
  boost::get<PointCloudPub>(pub_)->publish(pc);
}

void GazeboRosRaySensorPrivate::PublishPointCloud2(ConstLaserScanStampedPtr & _msg)
{
  // Create message to send
  sensor_msgs::msg::PointCloud2 pc;

  // Fill header
  pc.header.frame_id = frame_name_;
  pc.header.stamp.sec = _msg->time().sec();
  pc.header.stamp.nanosec = _msg->time().nsec();

  // Create fields in pointcloud
  sensor_msgs::PointCloud2Modifier pcd_modifier(pc);
  pcd_modifier.setPointCloud2Fields(4,
    "x", 1, sensor_msgs::msg::PointField::FLOAT32,
    "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32,
    "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
  pcd_modifier.resize(verticalRangeCount * rangeCount);
  pc.is_dense = true;
  sensor_msgs::PointCloud2Iterator<float> iter_x(pc, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(pc, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(pc, "z");
  sensor_msgs::PointCloud2Iterator<float> iter_intensity(pc, "intensity");

  // Fill pointcloud with laser scan
  for (int i = 0; i < rangeCount; i++) {
    // Calculate azimuth (horizontal angle)
    double azimuth;
    if (rangeCount > 1) {
      azimuth = i * yDiff / (rangeCount - 1) + minAngle;
    } else {
      azimuth = minAngle;
    }
    double c_azimuth = cos(azimuth);
    double s_azimuth = sin(azimuth);

    for (int j = 0; j < verticalRangeCount;
      ++j, ++iter_x, ++iter_y, ++iter_z, ++iter_intensity)
    {
      double inclination;
      // Calculate inclination (vertical angle)
      if (verticalRayCount > 1) {
        inclination = j * pDiff / (verticalRangeCount - 1) + verticalMinAngle;
      } else {
        inclination = verticalMinAngle;
      }
      double c_inclination = cos(inclination);
      double s_inclination = sin(inclination);

      // Get range and intensity at this scan
      size_t index = i + j * rangeCount;
      double r = _msg->scan().ranges(index);
      double intensity = _msg->scan().intensities(index);
      if (intensity < min_intensity_) {
        intensity = min_intensity_;
      }

      // Convert spherical coordinates to cartesian for pointcloud
      // See https://en.wikipedia.org/wiki/Spherical_coordinate_system
      *iter_x = r * c_inclination * c_azimuth;
      *iter_y = r * c_inclination * s_azimuth;
      *iter_z = r * s_inclination;
      *iter_intensity = intensity;
    }
  }

  // Publish output
  boost::get<PointCloud2Pub>(pub_)->publish(pc);
}

void GazeboRosRaySensorPrivate::PublishRange(ConstLaserScanStampedPtr & _msg)
{
  sensor_msgs::msg::Range range_msg;
  range_msg.header.frame_id = frame_name_;
  range_msg.header.stamp.sec = _msg->time().sec();
  range_msg.header.stamp.nanosec = _msg->time().nsec();
  range_msg.field_of_view = std::max(pDiff, yDiff);
  range_msg.min_range = rangeMin;
  range_msg.max_range = rangeMax;
  range_msg.radiation_type = range_radiation_type_;

  // Set range to the minimum of the ray ranges
  // For single rays, this will just be the range of the ray
  range_msg.range = std::numeric_limits<sensor_msgs::msg::Range::_range_type>::max();
  for (double range : _msg->scan().ranges()) {
    if (range < range_msg.range) {
      range_msg.range = range;
    }
  }

  boost::get<RangePub>(pub_)->publish(range_msg);
}

// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboRosRaySensor)

}  // namespace gazebo_plugins
