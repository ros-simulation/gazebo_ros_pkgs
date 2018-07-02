/*
 * Copyright 2013-2018 Open Source Robotics Foundation
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

#include <algorithm>
#include <assert.h>
#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>

#include <gazebo_plugins/gazebo_ros_depth_camera.h>

#include <gazebo/sensors/Sensor.hh>
#include <sdf/sdf.hh>
#include <gazebo/sensors/SensorTypes.hh>

#include <sensor_msgs/point_cloud2_iterator.h>

#include <tf/tf.h>

namespace gazebo
{
// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboRosDepthCamera)

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosDepthCamera::GazeboRosDepthCamera()
{
  this->point_cloud_connect_count_ = 0;
  this->depth_image_connect_count_ = 0;
  this->depth_info_connect_count_ = 0;
  this->last_depth_image_camera_info_update_time_ = common::Time(0);
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosDepthCamera::~GazeboRosDepthCamera()
{
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosDepthCamera::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{
  DepthCameraPlugin::Load(_parent, _sdf);

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM_NAMED("depth_camera", "A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  // copying from DepthCameraPlugin into GazeboRosCameraUtils
  this->parentSensor_ = this->parentSensor;
  this->width_ = this->width;
  this->height_ = this->height;
  this->depth_ = this->depth;
  this->format_ = this->format;
  this->camera_ = this->depthCamera;

  // using a different default
  if (!_sdf->HasElement("imageTopicName"))
    this->image_topic_name_ = "ir/image_raw";
  if (!_sdf->HasElement("cameraInfoTopicName"))
    this->camera_info_topic_name_ = "ir/camera_info";

  // point cloud stuff
  if (!_sdf->HasElement("pointCloudTopicName"))
    this->point_cloud_topic_name_ = "points";
  else
    this->point_cloud_topic_name_ = _sdf->GetElement("pointCloudTopicName")->Get<std::string>();

  // depth image stuff
  if (!_sdf->HasElement("depthImageTopicName"))
    this->depth_image_topic_name_ = "depth/image_raw";
  else
    this->depth_image_topic_name_ = _sdf->GetElement("depthImageTopicName")->Get<std::string>();

  if (!_sdf->HasElement("depthImageCameraInfoTopicName"))
    this->depth_image_camera_info_topic_name_ = "depth/camera_info";
  else
    this->depth_image_camera_info_topic_name_ = _sdf->GetElement("depthImageCameraInfoTopicName")->Get<std::string>();

  if (!_sdf->HasElement("pointCloudCutoff"))
  {
    this->point_cloud_cutoff_ = 0.4;
    ROS_INFO_NAMED("depth_camera", "Tag <pointCloudCutoff> not set, defaulting to %f", this->point_cloud_cutoff_);
  }
  else
    this->point_cloud_cutoff_ = _sdf->GetElement("pointCloudCutoff")->Get<double>();
  if (!_sdf->HasElement("pointCloudCutoffMax"))
  {
    this->point_cloud_cutoff_max_ = -1;
    ROS_INFO_NAMED("depth_camera", "Tag <pointCloudCutoffMax> not set, defaulting to no max");
  }
  else
    this->point_cloud_cutoff_max_ = _sdf->GetElement("pointCloudCutoffMax")->Get<double>();

  load_connection_ = GazeboRosCameraUtils::OnLoad(boost::bind(&GazeboRosDepthCamera::Advertise, this));
  GazeboRosCameraUtils::Load(_parent, _sdf);
}

void GazeboRosDepthCamera::Advertise()
{
  ros::AdvertiseOptions point_cloud_ao =
    ros::AdvertiseOptions::create<sensor_msgs::PointCloud2 >(
      this->point_cloud_topic_name_,1,
      boost::bind( &GazeboRosDepthCamera::PointCloudConnect,this),
      boost::bind( &GazeboRosDepthCamera::PointCloudDisconnect,this),
      ros::VoidPtr(), &this->camera_queue_);
  this->point_cloud_pub_ = this->rosnode_->advertise(point_cloud_ao);

  ros::AdvertiseOptions depth_image_ao =
    ros::AdvertiseOptions::create< sensor_msgs::Image >(
      this->depth_image_topic_name_,1,
      boost::bind( &GazeboRosDepthCamera::DepthImageConnect,this),
      boost::bind( &GazeboRosDepthCamera::DepthImageDisconnect,this),
      ros::VoidPtr(), &this->camera_queue_);
  this->depth_image_pub_ = this->rosnode_->advertise(depth_image_ao);

  ros::AdvertiseOptions depth_image_camera_info_ao =
    ros::AdvertiseOptions::create<sensor_msgs::CameraInfo>(
        this->depth_image_camera_info_topic_name_,1,
        boost::bind( &GazeboRosDepthCamera::DepthInfoConnect,this),
        boost::bind( &GazeboRosDepthCamera::DepthInfoDisconnect,this),
        ros::VoidPtr(), &this->camera_queue_);
  this->depth_image_camera_info_pub_ = this->rosnode_->advertise(depth_image_camera_info_ao);
}


////////////////////////////////////////////////////////////////////////////////
// Increment count
void GazeboRosDepthCamera::PointCloudConnect()
{
  this->point_cloud_connect_count_++;
  (*this->image_connect_count_)++;
  this->parentSensor->SetActive(true);
}
////////////////////////////////////////////////////////////////////////////////
// Decrement count
void GazeboRosDepthCamera::PointCloudDisconnect()
{
  this->point_cloud_connect_count_--;
  (*this->image_connect_count_)--;
  if (this->point_cloud_connect_count_ <= 0)
    this->parentSensor->SetActive(false);
}

////////////////////////////////////////////////////////////////////////////////
// Increment count
void GazeboRosDepthCamera::DepthImageConnect()
{
  this->depth_image_connect_count_++;
  this->parentSensor->SetActive(true);
}
////////////////////////////////////////////////////////////////////////////////
// Decrement count
void GazeboRosDepthCamera::DepthImageDisconnect()
{
  this->depth_image_connect_count_--;
}

////////////////////////////////////////////////////////////////////////////////
// Increment count
void GazeboRosDepthCamera::DepthInfoConnect()
{
  this->depth_info_connect_count_++;
}
////////////////////////////////////////////////////////////////////////////////
// Decrement count
void GazeboRosDepthCamera::DepthInfoDisconnect()
{
  this->depth_info_connect_count_--;
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosDepthCamera::OnNewDepthFrame(const float *_image,
    unsigned int _width, unsigned int _height, unsigned int _depth,
    const std::string &_format)
{
  if (!this->initialized_ || this->height_ <=0 || this->width_ <=0)
    return;

# if GAZEBO_MAJOR_VERSION >= 7
  this->depth_sensor_update_time_ = this->parentSensor->LastMeasurementTime();
# else
  this->depth_sensor_update_time_ = this->parentSensor->GetLastMeasurementTime();
# endif

  if (this->parentSensor->IsActive())
  {
    if (this->point_cloud_connect_count_ <= 0 &&
        this->depth_image_connect_count_ <= 0 &&
        (*this->image_connect_count_) <= 0)
    {
      this->parentSensor->SetActive(false);
    }
    else
    {
      if (this->point_cloud_connect_count_ > 0)
        this->FillPointdCloud(_image);

      if (this->depth_image_connect_count_ > 0)
        this->FillDepthImage(_image);
    }
  }
  else
  {
    if (this->point_cloud_connect_count_ > 0 ||
        this->depth_image_connect_count_ <= 0)
      // do this first so there's chance for sensor to run 1 frame after activate
      this->parentSensor->SetActive(true);
  }

  PublishCameraInfo();
}

///////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosDepthCamera::OnNewRGBPointCloud(const float *_pcd,
    unsigned int _width, unsigned int _height, unsigned int _depth,
    const std::string &_format)
{
  if (!this->initialized_ || this->height_ <=0 || this->width_ <=0)
    return;

# if GAZEBO_MAJOR_VERSION >= 7
  this->depth_sensor_update_time_ = this->parentSensor->LastMeasurementTime();
# else
  this->depth_sensor_update_time_ = this->parentSensor->GetLastMeasurementTime();
# endif

  if (!this->parentSensor->IsActive())
  {
    if (this->point_cloud_connect_count_ > 0)
      // do this first so there's chance for sensor to run 1 frame after activate
      this->parentSensor->SetActive(true);
  }
  else
  {
    if (this->point_cloud_connect_count_ > 0)
    {
      this->lock_.lock();
      this->point_cloud_msg_.header.frame_id = this->frame_name_;
      this->point_cloud_msg_.header.stamp.sec = this->depth_sensor_update_time_.sec;
      this->point_cloud_msg_.header.stamp.nsec = this->depth_sensor_update_time_.nsec;
      this->point_cloud_msg_.width = this->width;
      this->point_cloud_msg_.height = this->height;
      this->point_cloud_msg_.row_step = this->point_cloud_msg_.point_step * this->width;

      sensor_msgs::PointCloud2Modifier pcd_modifier(point_cloud_msg_);
      pcd_modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
      pcd_modifier.resize(_width*_height);

      point_cloud_msg_.is_dense = true;

      sensor_msgs::PointCloud2Iterator<float> iter_x(point_cloud_msg_, "x");
      sensor_msgs::PointCloud2Iterator<float> iter_y(point_cloud_msg_, "y");
      sensor_msgs::PointCloud2Iterator<float> iter_z(point_cloud_msg_, "z");
      sensor_msgs::PointCloud2Iterator<float> iter_rgb(point_cloud_msg_, "rgb");

      for (unsigned int i = 0; i < _width; i++)
      {
        for (unsigned int j = 0; j < _height; j++, ++iter_x, ++iter_y, ++iter_z, ++iter_rgb)
        {
          unsigned int index = (j * _width) + i;
          *iter_x = _pcd[4 * index];
          *iter_y = _pcd[4 * index + 1];
          *iter_z = _pcd[4 * index + 2];
          *iter_rgb = _pcd[4 * index + 3];
          if (i == _width /2 && j == _height / 2)
          {
            uint32_t rgb = *reinterpret_cast<int*>(&(*iter_rgb));
            uint8_t r = (rgb >> 16) & 0x0000ff;
            uint8_t g = (rgb >> 8)  & 0x0000ff;
            uint8_t b = (rgb)       & 0x0000ff;
            std::cerr << (int)r << " " << (int)g << " " << (int)b << "\n";
          }
        }
      }

      this->point_cloud_pub_.publish(this->point_cloud_msg_);
      this->lock_.unlock();
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosDepthCamera::OnNewImageFrame(const unsigned char *_image,
    unsigned int _width, unsigned int _height, unsigned int _depth,
    const std::string &_format)
{
  if (!this->initialized_ || this->height_ <=0 || this->width_ <=0)
    return;

# if GAZEBO_MAJOR_VERSION >= 7
  this->sensor_update_time_ = this->parentSensor->LastMeasurementTime();
# else
  this->sensor_update_time_ = this->parentSensor->GetLastMeasurementTime();
# endif

  if (this->parentSensor->IsActive())
  {
    if (this->point_cloud_connect_count_ <= 0 &&
        this->depth_image_connect_count_ <= 0 &&
        (*this->image_connect_count_) <= 0)
    {
      this->parentSensor->SetActive(false);
    }
    else
    {
      if ((*this->image_connect_count_) > 0)
        this->PutCameraData(_image);
    }
  }
  else
  {
    if ((*this->image_connect_count_) > 0)
      // do this first so there's chance for sensor to run 1 frame after activate
      this->parentSensor->SetActive(true);
  }
}

////////////////////////////////////////////////////////////////////////////////
// Put camera data to the interface
void GazeboRosDepthCamera::FillPointdCloud(const float *_src)
{
  // TODO: use scoped lock gaurds
  this->lock_.lock();

  this->point_cloud_msg_.header.frame_id = this->frame_name_;
  this->point_cloud_msg_.header.stamp.sec = this->depth_sensor_update_time_.sec;
  this->point_cloud_msg_.header.stamp.nsec = this->depth_sensor_update_time_.nsec;
  this->point_cloud_msg_.width = this->width;
  this->point_cloud_msg_.height = this->height;
  this->point_cloud_msg_.row_step = this->point_cloud_msg_.point_step * this->width;

  ///copy from depth to point cloud message
  bool success = FillPointCloudHelper(this->point_cloud_msg_,
                                      this->height,
                                      this->width,
                                      this->skip_,
                                      (void*)_src );

  // If filling point cloud failed, don't publish it (user has already been warned)
  if (success)
    this->point_cloud_pub_.publish(this->point_cloud_msg_);

  this->lock_.unlock();
}

////////////////////////////////////////////////////////////////////////////////
// Put depth image data to the interface
void GazeboRosDepthCamera::FillDepthImage(const float *_src)
{
  this->lock_.lock();
  // copy data into image
  this->depth_image_msg_.header.frame_id = this->frame_name_;
  this->depth_image_msg_.header.stamp.sec = this->depth_sensor_update_time_.sec;
  this->depth_image_msg_.header.stamp.nsec = this->depth_sensor_update_time_.nsec;

  ///copy from depth to depth image message
  FillDepthImageHelper(this->depth_image_msg_,
                 this->height,
                 this->width,
                 this->skip_,
                 (void*)_src );

  this->depth_image_pub_.publish(this->depth_image_msg_);

  this->lock_.unlock();
}


// Fill depth information
bool GazeboRosDepthCamera::FillPointCloudHelper(
    sensor_msgs::PointCloud2 &point_cloud_msg,
    uint32_t rows_arg, uint32_t cols_arg,
    uint32_t step_arg, void* data_arg)
{
  // Skip if color image has not been filled yet
  if (this->image_msg_.data.empty())
  {
    ROS_DEBUG_STREAM_NAMED("depth_camera", "Image is empty, skipping point cloud generation. "
                           << "This should only happen once at the beginning");
    return false;
  }

  // Determine if/how to set color/intensity fields based on camera encoding
  enum {RGB, BGR, MONO, NO_COLOR} color;
  std::string encoding = this->image_msg_.encoding;
  if (encoding == sensor_msgs::image_encodings::RGB8)
    color = RGB;
  else if (encoding == sensor_msgs::image_encodings::BGR8)
    color = BGR;
  else if (encoding == sensor_msgs::image_encodings::MONO8)
    color = MONO;
  else
  {
    ROS_WARN_ONCE_NAMED("depth_camera", "Unsupported image encoding %s. Pointcloud will not contain color/intensity.",
                        encoding.c_str());
    color = NO_COLOR;
  }

  // Ensure image is correct size
  if ((color == RGB || color == BGR) && this->image_msg_.data.size() != rows_arg*cols_arg*3)
  {
    ROS_WARN_ONCE_NAMED("depth_camera", "Image is wrong size, point cloud will not contain color");
    color = NO_COLOR;
  }
  else if (color == MONO && this->image_msg_.data.size() != rows_arg*cols_arg)
  {
    ROS_WARN_ONCE_NAMED("depth_camera", "Image is wrong size, point cloud will not contain intensity");
    color = NO_COLOR;
  }

  sensor_msgs::PointCloud2Modifier pcd_modifier(point_cloud_msg);
  // Set fields of pointcloud based on camera encoding
  if (color == RGB || color == BGR)
  {
    pcd_modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
  }
  else if (color == MONO)
  {
    pcd_modifier.setPointCloud2Fields(4, "x", 1, sensor_msgs::PointField::FLOAT32,
                                     "y", 1, sensor_msgs::PointField::FLOAT32,
                                     "z", 1, sensor_msgs::PointField::FLOAT32,
                                     "intensity", 1, sensor_msgs::PointField::UINT8);
  }
  else
    pcd_modifier.setPointCloud2FieldsByString(1, "xyz");

  // Preallocate points for each pixel
  pcd_modifier.resize(rows_arg*cols_arg);

  // Create iterator for point xyz position
  sensor_msgs::PointCloud2Iterator<float> iter_x(point_cloud_msg_, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(point_cloud_msg_, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(point_cloud_msg_, "z");

  // Create iterator for rgb or intensity based on camera encoding
  typedef sensor_msgs::PointCloud2Iterator<uint8_t> color_iter_t;
  std::unique_ptr<color_iter_t> iter_color = nullptr;
  if (color == BGR || color == RGB)
    iter_color.reset(new color_iter_t(point_cloud_msg_, "rgb"));
  else if (color == MONO)
    iter_color.reset(new color_iter_t(point_cloud_msg_, "intensity"));

  point_cloud_msg.is_dense = true;

  float* toCopyFrom = (float*)data_arg;
  int index = 0;

  double hfov = this->parentSensor->DepthCamera()->HFOV().Radian();
  double fl = ((double)this->width) / (2.0 *tan(hfov/2.0));

  // convert depth to point cloud
  for (uint32_t j=0; j<rows_arg; j++)
  {
    double pAngle;
    if (rows_arg>1) pAngle = atan2( (double)j - 0.5*(double)(rows_arg-1), fl);
    else            pAngle = 0.0;

    for (uint32_t i=0; i < cols_arg; i++, ++iter_x, ++iter_y, ++iter_z)
    {
      double yAngle;
      if (cols_arg>1) yAngle = atan2( (double)i - 0.5*(double)(cols_arg-1), fl);
      else            yAngle = 0.0;

      double depth = toCopyFrom[index++];

      if (depth > this->point_cloud_cutoff_ &&
         (this->point_cloud_cutoff_max_ < 0 || depth < this->point_cloud_cutoff_max_))
      {
        *iter_x      = depth * tan(yAngle);
        *iter_y      = depth * tan(pAngle);
        *iter_z    = depth;
      }
      else //point in the unseeable range
      {
        *iter_x = *iter_y = *iter_z = std::numeric_limits<float>::quiet_NaN ();
        point_cloud_msg.is_dense = false;
      }

      // put image color data for each point
      uint8_t*  image_src = (uint8_t*)(&(this->image_msg_.data[0]));

      // Set rgb or intensity fields based on encoding
      if (color == RGB)
      {
        // color
        (*iter_color)[0] = image_src[i*3+j*cols_arg*3+2];
        (*iter_color)[1] = image_src[i*3+j*cols_arg*3+1];
        (*iter_color)[2] = image_src[i*3+j*cols_arg*3+0];
      }
      else if (color == BGR)
      {
        // color
        (*iter_color)[0] = image_src[i*3+j*cols_arg*3+0];
        (*iter_color)[1] = image_src[i*3+j*cols_arg*3+1];
        (*iter_color)[2] = image_src[i*3+j*cols_arg*3+2];

      }
      else if (color == MONO)
      {
        *(*iter_color) = image_src[i+j*cols_arg];
      }

      // Increment color iterator if it is being used
      if(iter_color) ++(*iter_color);
    }
  }

  // reconvert to original height and width after the flat reshape
  point_cloud_msg.height = rows_arg;
  point_cloud_msg.width = cols_arg;
  point_cloud_msg.row_step = point_cloud_msg.point_step * point_cloud_msg.width;

  return true;
}

// Fill depth information
bool GazeboRosDepthCamera::FillDepthImageHelper(
    sensor_msgs::Image& image_msg,
    uint32_t rows_arg, uint32_t cols_arg,
    uint32_t step_arg, void* data_arg)
{
  image_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  image_msg.height = rows_arg;
  image_msg.width = cols_arg;
  image_msg.step = sizeof(float) * cols_arg;
  image_msg.data.resize(rows_arg * cols_arg * sizeof(float));
  image_msg.is_bigendian = 0;

  const float bad_point = std::numeric_limits<float>::quiet_NaN();

  float* dest = (float*)(&(image_msg.data[0]));
  float* toCopyFrom = (float*)data_arg;
  int index = 0;

  // convert depth to point cloud
  for (uint32_t j = 0; j < rows_arg; j++)
  {
    for (uint32_t i = 0; i < cols_arg; i++)
    {
      float depth = toCopyFrom[index++];

      if (depth > this->point_cloud_cutoff_ &&
         (this->point_cloud_cutoff_max_ < 0 || depth < this->point_cloud_cutoff_max_))
      {
        dest[i + j * cols_arg] = depth;
      }
      else //point in the unseeable range
      {
        dest[i + j * cols_arg] = bad_point;
      }
    }
  }
  return true;
}

void GazeboRosDepthCamera::PublishCameraInfo()
{
  ROS_DEBUG_NAMED("depth_camera", "publishing default camera info, then depth camera info");
  GazeboRosCameraUtils::PublishCameraInfo();

  if (this->depth_info_connect_count_ > 0)
  {
# if GAZEBO_MAJOR_VERSION >= 7
    this->sensor_update_time_ =  this->parentSensor_->LastMeasurementTime();
# else
    this->sensor_update_time_ =  this->parentSensor_->GetLastMeasurementTime();
# endif
    if (this->sensor_update_time_ - this->last_depth_image_camera_info_update_time_ >= this->update_period_)
    {
      this->PublishCameraInfo(this->depth_image_camera_info_pub_);
      this->last_depth_image_camera_info_update_time_ = this->sensor_update_time_;
    }
  }
}
}
