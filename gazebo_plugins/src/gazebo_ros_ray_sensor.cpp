/*
 * Copyright 2013 Open Source Robotics Foundation
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

#include <gazebo/transport/transport.hh>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <gazebo_plugins/gazebo_ros_ray_sensor.h>

namespace gazebo
{

// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboRosRaySensor)


////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosRaySensor::GazeboRosRaySensor()
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosRaySensor::~GazeboRosRaySensor()
{
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosRaySensor::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{
  GAZEBO_SENSORS_USING_DYNAMIC_POINTER_CAST;
  if (_parent->Type() == "ray")
  {
    sensors::RaySensorPtr ray_sensor = dynamic_pointer_cast<sensors::RaySensor>(_parent);
    if (!ray_sensor)
    {
      ROS_ERROR_NAMED("laser", "Could not cast to ray sensor. Exiting.");
      return;
    }
    SetParams<sensors::RaySensorPtr>(ray_sensor);
  }
  else if (_parent->Type() == "gpu_ray")
  {
    sensors::GpuRaySensorPtr gpu_ray_sensor = dynamic_pointer_cast<sensors::GpuRaySensor>(_parent);
    if (!gpu_ray_sensor)
    {
      ROS_ERROR_NAMED("laser", "Could not cast to gpu_ray sensor. Exiting.");
      return;
    }
    SetParams<sensors::GpuRaySensorPtr>(gpu_ray_sensor);
  }
  else
  {
    ROS_ERROR_NAMED("laser", "Plugin attached to a sensor that is neither ray or gpu_ray. Exiting.");
    return;
  }

  this->parent_ray_sensor_ = _parent;

  this->robot_namespace_ =  GetRobotNamespace(_parent, _sdf, "Laser");

  if (!_sdf->HasElement("frameName"))
  {
    ROS_INFO_NAMED("laser", "Laser plugin missing <frameName>, defaults to /world");
    this->frame_name_ = "/world";
  }
  else
    this->frame_name_ = _sdf->Get<std::string>("frameName");

  if (!_sdf->HasElement("topicName"))
  {
    ROS_INFO_NAMED("laser", "Laser plugin missing <topicName>, defaults to /world");
    this->topic_name_ = "/world";
  }
  else
    this->topic_name_ = _sdf->Get<std::string>("topicName");

  if (this->topic_name_.empty())
  {
    ROS_FATAL_STREAM_NAMED("laser", "<topicName> is empty. Exiting.");
    return;
  }

  if(!_sdf->HasElement("outputType"))
  {
    ROS_WARN_NAMED("laser", "missing <outputType>, defaults to %s",
        ros::message_traits::DataType<sensor_msgs::PointCloud2>::value());
    this->output_type_ = POINTCLOUD2;
  }
  else
  {
    std::string output_type_string = _sdf->Get<std::string>("outputType");
    if(output_type_string == ros::message_traits::DataType<sensor_msgs::LaserScan>::value())
      this->output_type_ = LASERSCAN;
    else if (output_type_string == ros::message_traits::DataType<sensor_msgs::PointCloud>::value())
      this->output_type_ = POINTCLOUD;
    else if (output_type_string == ros::message_traits::DataType<sensor_msgs::PointCloud2>::value())
      this->output_type_ = POINTCLOUD2;
    else if (output_type_string == ros::message_traits::DataType<sensor_msgs::Range>::value())
      this->output_type_ = RANGE;
    else
    {
      ROS_WARN_NAMED("laser", "Invalid value of output <outputType>, defaults to %s",
          ros::message_traits::DataType<sensor_msgs::PointCloud2>::value());
      this->output_type_ = POINTCLOUD2;
    }
  }

  if (RANGE == this->output_type_)
  {
    if(!_sdf->HasElement("radiationType"))
    {
      ROS_INFO_NAMED("laser", "missing <radiationType>, defaulting to infared");
      this->range_radiation_type_ = sensor_msgs::Range::INFRARED;
    } else if ("ultrasound" == _sdf->Get<std::string>("radiationType")) {
      this->range_radiation_type_ == sensor_msgs::Range::ULTRASOUND;
    } else if ("infared" ==  _sdf->Get<std::string>("radiationType")) {
      this->range_radiation_type_ == sensor_msgs::Range::INFRARED;
    } else {
      ROS_ERROR_NAMED("laser", "Invalid <radiationType> [%s]. Must be ultrasound or infared",
        _sdf->Get<std::string>("radiationType").c_str());
      return;
    }
  }

  if(!_sdf->HasElement("minIntensity"))
  {
    this->min_intensity_ = 0.0;
    ROS_INFO_NAMED("laser", "missing <minIntensity>, defaults to %f",
        this->min_intensity_);
  }
  else
    this->min_intensity_ = _sdf->Get<double>("minIntensity");

  this->laser_connect_count_ = 0;

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM_NAMED("laser", "A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  this->gazebo_node_ = boost::make_shared<gazebo::transport::Node>();
  this->gazebo_node_->Init(_parent->WorldName());

  this->rosnode_ = boost::make_shared<ros::NodeHandle>(this->robot_namespace_);

  // Resolve TF frame for backwards compability, TODO: remove
  this->frame_name_ = resolveTF(this->frame_name_, this->robot_namespace_, *this->rosnode_);

  switch (this->output_type_)
  {
    case LASERSCAN:
      this->AdvertiseOutput<sensor_msgs::LaserScan>();
      break;
    case POINTCLOUD:
      this->AdvertiseOutput<sensor_msgs::PointCloud>();
      break;
    case POINTCLOUD2:
      this->AdvertiseOutput<sensor_msgs::PointCloud2>();
      break;
    case RANGE:
      this->AdvertiseOutput<sensor_msgs::Range>();
      break;
  }
}

std::string GazeboRosRaySensor::resolveTF(const std::string& _frame, const std::string& _robot_namespace, ros::NodeHandle& _nh)
{
  return _frame;
}

void GazeboRosRaySensor::SubscribeGazeboLaserScan()
{
  switch (this->output_type_)
  {
    case LASERSCAN:
      this->laser_scan_sub_ =
        this->gazebo_node_->Subscribe(this->parent_ray_sensor_->Topic(),
                                      &GazeboRosRaySensor::PublishLaserScan, this);
      break;
    case POINTCLOUD:
      this->laser_scan_sub_ =
        this->gazebo_node_->Subscribe(this->parent_ray_sensor_->Topic(),
                                      &GazeboRosRaySensor::PublishPointCloud, this);
      break;
    case POINTCLOUD2:
      this->laser_scan_sub_ =
        this->gazebo_node_->Subscribe(this->parent_ray_sensor_->Topic(),
                                      &GazeboRosRaySensor::PublishPointCloud2, this);
      break;
    case RANGE:
      this->laser_scan_sub_ =
        this->gazebo_node_->Subscribe(this->parent_ray_sensor_->Topic(),
                                     &GazeboRosRaySensor::PublishRange, this);
      break;
  }
}

////////////////////////////////////////////////////////////////////////////////
// Increment count
void GazeboRosRaySensor::LaserConnect()
{
  this->laser_connect_count_++;
  if (this->laser_connect_count_ == 1)
    this->SubscribeGazeboLaserScan();
}

////////////////////////////////////////////////////////////////////////////////
// Decrement count
void GazeboRosRaySensor::LaserDisconnect()
{
  this->laser_connect_count_--;
  if (this->laser_connect_count_ == 0)
    this->laser_scan_sub_.reset();
}

////////////////////////////////////////////////////////////////////////////////
// Convert new Gazebo message to ROS message and publish it
void GazeboRosRaySensor::PublishLaserScan(ConstLaserScanStampedPtr &_msg)
{
  // We got a new message from the Gazebo sensor.  Stuff a
  // corresponding ROS message and publish it.
  sensor_msgs::LaserScan ls;
  ls.header.frame_id = this->frame_name_;
  ls.header.stamp.sec = _msg->time().sec();
  ls.header.stamp.nsec = _msg->time().nsec();
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
  this->pub_.publish(ls);
}

void GazeboRosRaySensor::PublishPointCloud(ConstLaserScanStampedPtr &_msg)
{
  // Create message to send
  sensor_msgs::PointCloud pc;

  // Fill header
  pc.header.frame_id = frame_name_;
  pc.header.stamp.sec = _msg->time().sec();
  pc.header.stamp.nsec = _msg->time().nsec();

  // Setup point cloud fields
  pc.points.reserve(verticalRangeCount * rangeCount);
  pc.channels.push_back(sensor_msgs::ChannelFloat32());
  pc.channels[0].values.reserve(verticalRangeCount * rangeCount);
  pc.channels[0].name = "intensity";

  // Fill pointcloud with laser scan
  for (int i = 0; i < rangeCount; i++) {
    // Calculate azimuth (horizontal angle)
    double azimuth;
    if (rangeCount > 1) {
      azimuth = i * yDiff / (rangeCount -1) + minAngle;
    } else {
      azimuth = minAngle;
    }
    double c_azimuth = cos(azimuth);
    double s_azimuth = sin(azimuth);

    for (int j = 0; j < verticalRangeCount; ++j) {
      double inclination;
      // Calculate inclination (vertical angle)
      if (verticalRayCount > 1) {
        inclination = j * pDiff / (verticalRangeCount -1) + verticalMinAngle;
      } else {
        inclination = verticalMinAngle;
      }
      double c_inclination = cos(inclination);
      double s_inclination = sin(inclination);

      // Get range and intensity at this scan
      size_t index = i + j * rangeCount;
      double r = _msg->scan().ranges(index);
      double intensity = _msg->scan().intensities(index);
      if (intensity < this->min_intensity_)
        intensity = this->min_intensity_;

      // Convert spherical coordinates to cartesian for pointcloud
      // See https://en.wikipedia.org/wiki/Spherical_coordinate_system
      geometry_msgs::Point32 point;
      point.x = r * c_inclination * c_azimuth;
      point.y = r * c_inclination * s_azimuth;
      point.z = r * s_inclination;
      pc.points.push_back(point);
      pc.channels[0].values.push_back(intensity);
    }
  }

  // Publish output
  pub_.publish(pc);
}

void GazeboRosRaySensor::PublishPointCloud2(ConstLaserScanStampedPtr &_msg)
{
  // Create message to send
  sensor_msgs::PointCloud2 pc;

  // Fill header
  pc.header.frame_id = frame_name_;
  pc.header.stamp.sec = _msg->time().sec();
  pc.header.stamp.nsec = _msg->time().nsec();

  // Create fields in pointcloud
  sensor_msgs::PointCloud2Modifier pcd_modifier(pc);
  pcd_modifier.setPointCloud2Fields(4,
      "x", 1, sensor_msgs::PointField::FLOAT32,
      "y", 1, sensor_msgs::PointField::FLOAT32,
      "z", 1, sensor_msgs::PointField::FLOAT32,
      "intensity", 1, sensor_msgs::PointField::FLOAT32);
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
      azimuth = i * yDiff / (rangeCount -1) + minAngle;
    } else {
      azimuth = minAngle;
    }
    double c_azimuth = cos(azimuth);
    double s_azimuth = sin(azimuth);

    for (int j = 0; j < verticalRangeCount; ++j, ++iter_x, ++iter_y, ++iter_z, ++iter_intensity) {
      double inclination;
      // Calculate inclination (vertical angle)
      if (verticalRayCount > 1) {
        inclination = j * pDiff / (verticalRangeCount -1) + verticalMinAngle;
      } else {
        inclination = verticalMinAngle;
      }
      double c_inclination = cos(inclination);
      double s_inclination = sin(inclination);

      // Get range and intensity at this scan
      size_t index = i + j * rangeCount;
      double r = _msg->scan().ranges(index);
      double intensity = _msg->scan().intensities(index);
      if (intensity < this->min_intensity_)
        intensity = this->min_intensity_;

      // Convert spherical coordinates to cartesian for pointcloud
      // See https://en.wikipedia.org/wiki/Spherical_coordinate_system
      *iter_x = r * c_inclination * c_azimuth;
      *iter_y = r * c_inclination * s_azimuth;
      *iter_z = r * s_inclination;
      *iter_intensity = intensity;
    }
  }

  // Publish output
  pub_.publish(pc);
}

void GazeboRosRaySensor::PublishRange(ConstLaserScanStampedPtr &_msg)
{
  sensor_msgs::Range range_msg;
  range_msg.header.frame_id = this->frame_name_;
  range_msg.header.stamp.sec = _msg->time().sec();
  range_msg.header.stamp.nsec = _msg->time().nsec();
  range_msg.field_of_view = std::max(pDiff, yDiff);
  range_msg.min_range = rangeMin;
  range_msg.max_range = rangeMax;

  // Set range to the minimum of the ray ranges
  // For single rays, this will just be the range of the ray
  range_msg.range = std::numeric_limits<sensor_msgs::Range::_range_type>::max();
  for(double range : _msg->scan().ranges())
  {
      if (range < range_msg.range)
          range_msg.range = range;
  }

  pub_.publish(range_msg);
}

}
