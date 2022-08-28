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

/*
   Desc: GazeboRosOpenniKinect plugin for simulating cameras in Gazebo
   Author: John Hsu
   Date: 24 Sept 2008
 */

#include <algorithm>
#include <assert.h>
#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>

#include <gazebo_plugins/gazebo_ros_openni_kinect.h>

#include <gazebo/sensors/Sensor.hh>
#include <sdf/sdf.hh>
#include <gazebo/sensors/SensorTypes.hh>

#include <sensor_msgs/point_cloud2_iterator.h>

#include <tf/tf.h>

namespace gazebo
{
// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboRosOpenniKinect)

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosOpenniKinect::GazeboRosOpenniKinect()
{
  this->point_cloud_connect_count_ = 0;
  this->depth_info_connect_count_ = 0;
  this->depth_image_connect_count_ = 0;
  this->last_depth_image_camera_info_update_time_ = common::Time(0);
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosOpenniKinect::~GazeboRosOpenniKinect()
{
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosOpenniKinect::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{
  DepthCameraPlugin::Load(_parent, _sdf);

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
    this->point_cloud_cutoff_ = 0.4;
  else
    this->point_cloud_cutoff_ = _sdf->GetElement("pointCloudCutoff")->Get<double>();
  if (!_sdf->HasElement("pointCloudCutoffMax"))
    this->point_cloud_cutoff_max_ = 5.0;
  else
    this->point_cloud_cutoff_max_ = _sdf->GetElement("pointCloudCutoffMax")->Get<double>();

  // noise
  if (!_sdf->HasElement("gaussianNoise")) {
    this->gaussian_noise_ = 0;
  } else {
    this->gaussian_noise_ = _sdf->GetElement("gaussianNoise")->Get<double>();
  }
  
  // outlier magnitude
  if (!_sdf->HasElement("outlierMagnitude")) {
    this->outlier_magnitude_ = 0;
  } else {
    this->outlier_magnitude_ = _sdf->GetElement("outlierMagnitude")->Get<double>();
  }

  // outlier index
  if (!_sdf->HasElement("outlierIndex")) {
    this->outlier_index_ = 0;
  } else {
    this->outlier_index_ = _sdf->GetElement("outlierIndex")->Get<double>();
  }
  int max_index = this->width * this->height;
  if (outlier_index_ == 0) {
  // generate a uniformly distributed random number    
  std::uniform_int_distribution<> uniform_distribution(1, max_index);
  std::mt19937 generator( std::random_device{}() );
  this->outlier_index_ = uniform_distribution(generator);
  }

  load_connection_ = GazeboRosCameraUtils::OnLoad(boost::bind(&GazeboRosOpenniKinect::Advertise, this));
  GazeboRosCameraUtils::Load(_parent, _sdf);
}

void GazeboRosOpenniKinect::Advertise()
{
  ros::AdvertiseOptions point_cloud_ao =
    ros::AdvertiseOptions::create<sensor_msgs::PointCloud2 >(
      this->point_cloud_topic_name_,1,
      boost::bind( &GazeboRosOpenniKinect::PointCloudConnect,this),
      boost::bind( &GazeboRosOpenniKinect::PointCloudDisconnect,this),
      ros::VoidPtr(), &this->camera_queue_);
  this->point_cloud_pub_ = this->rosnode_->advertise(point_cloud_ao);

  ros::AdvertiseOptions depth_image_ao =
    ros::AdvertiseOptions::create< sensor_msgs::Image >(
      this->depth_image_topic_name_,1,
      boost::bind( &GazeboRosOpenniKinect::DepthImageConnect,this),
      boost::bind( &GazeboRosOpenniKinect::DepthImageDisconnect,this),
      ros::VoidPtr(), &this->camera_queue_);
  this->depth_image_pub_ = this->rosnode_->advertise(depth_image_ao);

  ros::AdvertiseOptions depth_image_camera_info_ao =
    ros::AdvertiseOptions::create<sensor_msgs::CameraInfo>(
        this->depth_image_camera_info_topic_name_,1,
        boost::bind( &GazeboRosOpenniKinect::DepthInfoConnect,this),
        boost::bind( &GazeboRosOpenniKinect::DepthInfoDisconnect,this),
        ros::VoidPtr(), &this->camera_queue_);
  this->depth_image_camera_info_pub_ = this->rosnode_->advertise(depth_image_camera_info_ao);
}

////////////////////////////////////////////////////////////////////////////////
// Increment count
void GazeboRosOpenniKinect::PointCloudConnect()
{
  this->point_cloud_connect_count_++;
  (*this->image_connect_count_)++;
  this->parentSensor->SetActive(true);
}
////////////////////////////////////////////////////////////////////////////////
// Decrement count
void GazeboRosOpenniKinect::PointCloudDisconnect()
{
  this->point_cloud_connect_count_--;
  (*this->image_connect_count_)--;
  if (this->point_cloud_connect_count_ <= 0)
    this->parentSensor->SetActive(false);
}

////////////////////////////////////////////////////////////////////////////////
// Increment count
void GazeboRosOpenniKinect::DepthImageConnect()
{
  this->depth_image_connect_count_++;
  this->parentSensor->SetActive(true);
}
////////////////////////////////////////////////////////////////////////////////
// Decrement count
void GazeboRosOpenniKinect::DepthImageDisconnect()
{
  this->depth_image_connect_count_--;
}

////////////////////////////////////////////////////////////////////////////////
// Increment count
void GazeboRosOpenniKinect::DepthInfoConnect()
{
  this->depth_info_connect_count_++;
}
////////////////////////////////////////////////////////////////////////////////
// Decrement count
void GazeboRosOpenniKinect::DepthInfoDisconnect()
{
  this->depth_info_connect_count_--;
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosOpenniKinect::OnNewDepthFrame(const float *_image,
    unsigned int _width, unsigned int _height, unsigned int _depth,
    const std::string &_format)
{
  if (!this->initialized_ || this->height_ <=0 || this->width_ <=0)
    return;

  this->depth_sensor_update_time_ = this->parentSensor->LastMeasurementTime();
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

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosOpenniKinect::OnNewImageFrame(const unsigned char *_image,
    unsigned int _width, unsigned int _height, unsigned int _depth,
    const std::string &_format)
{
  if (!this->initialized_ || this->height_ <=0 || this->width_ <=0)
    return;

  //ROS_ERROR_NAMED("openni_kinect", "camera_ new frame %s %s",this->parentSensor_->Name().c_str(),this->frame_name_.c_str());
  this->sensor_update_time_ = this->parentSensor_->LastMeasurementTime();

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
// Put point cloud data to the interface
void GazeboRosOpenniKinect::FillPointdCloud(const float *_src)
{
  this->lock_.lock();

  this->point_cloud_msg_.header.frame_id = this->frame_name_;
  this->point_cloud_msg_.header.stamp.sec = this->depth_sensor_update_time_.sec;
  this->point_cloud_msg_.header.stamp.nsec = this->depth_sensor_update_time_.nsec;
  this->point_cloud_msg_.width = this->width;
  this->point_cloud_msg_.height = this->height;
  this->point_cloud_msg_.row_step = this->point_cloud_msg_.point_step * this->width;

  ///copy from depth to point cloud message
  FillPointCloudHelper(this->point_cloud_msg_,
                 this->height,
                 this->width,
                 this->skip_,
                 (void*)_src );

  this->point_cloud_pub_.publish(this->point_cloud_msg_);

  this->lock_.unlock();
}

////////////////////////////////////////////////////////////////////////////////
// Put depth image data to the interface
void GazeboRosOpenniKinect::FillDepthImage(const float *_src)
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
bool GazeboRosOpenniKinect::FillPointCloudHelper(
    sensor_msgs::PointCloud2 &point_cloud_msg,
    uint32_t rows_arg, uint32_t cols_arg,
    uint32_t step_arg, void* data_arg)
{
  sensor_msgs::PointCloud2Modifier pcd_modifier(point_cloud_msg);
  pcd_modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
  // convert to flat array shape, we need to reconvert later
  pcd_modifier.resize(rows_arg*cols_arg);
  point_cloud_msg.is_dense = true;

  sensor_msgs::PointCloud2Iterator<float> iter_x(point_cloud_msg_, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(point_cloud_msg_, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(point_cloud_msg_, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_rgb(point_cloud_msg_, "rgb");

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

    for (uint32_t i=0; i<cols_arg; i++, ++iter_x, ++iter_y, ++iter_z, ++iter_rgb)
    {
      double yAngle;
      if (cols_arg>1) yAngle = atan2( (double)i - 0.5*(double)(cols_arg-1), fl);
      else            yAngle = 0.0;

      double depth = toCopyFrom[index++]; // + 0.0*this->myParent->GetNearClip();

      // add noise
      if (gaussian_noise_ != 0) {
        depth *= ( 1 + gaussianKernel(0, this->gaussian_noise_) );
      }

      // add outlier
      if (outlier_magnitude_ != 0.0 && abs(index - outlier_index_) < 2 ) {
        depth *= ( 1 - outlier_magnitude_);
      }

      if(depth > this->point_cloud_cutoff_ &&
         depth < this->point_cloud_cutoff_max_)
      {
        // in optical frame
        // hardcoded rotation rpy(-M_PI/2, 0, -M_PI/2) is built-in
        // to urdf, where the *_optical_frame should have above relative
        // rotation from the physical camera *_frame
        *iter_x = depth * tan(yAngle);
        *iter_y = depth * tan(pAngle);
        *iter_z = depth;
      }
      else //point in the unseeable range
      {
        *iter_x = *iter_y = *iter_z = std::numeric_limits<float>::quiet_NaN ();
        point_cloud_msg.is_dense = false;
      }

      // put image color data for each point
      uint8_t*  image_src = (uint8_t*)(&(this->image_msg_.data[0]));
      if (this->image_msg_.data.size() == rows_arg*cols_arg*3)
      {
        // color
        iter_rgb[0] = image_src[i*3+j*cols_arg*3+0];
        iter_rgb[1] = image_src[i*3+j*cols_arg*3+1];
        iter_rgb[2] = image_src[i*3+j*cols_arg*3+2];
      }
      else if (this->image_msg_.data.size() == rows_arg*cols_arg)
      {
        // mono (or bayer?  @todo; fix for bayer)
        iter_rgb[0] = image_src[i+j*cols_arg];
        iter_rgb[1] = image_src[i+j*cols_arg];
        iter_rgb[2] = image_src[i+j*cols_arg];
      }
      else
      {
        // no image
        iter_rgb[0] = 0;
        iter_rgb[1] = 0;
        iter_rgb[2] = 0;
      }
    }
  }

  // reconvert to original height and width after the flat reshape
  point_cloud_msg.height = rows_arg;
  point_cloud_msg.width = cols_arg;
  point_cloud_msg.row_step = point_cloud_msg.point_step * point_cloud_msg.width;

  return true;
}

// Fill depth information
bool GazeboRosOpenniKinect::FillDepthImageHelper(
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
          depth < this->point_cloud_cutoff_max_)
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

void GazeboRosOpenniKinect::PublishCameraInfo()
{
  ROS_DEBUG_NAMED("openni_kinect", "publishing default camera info, then openni kinect camera info");
  GazeboRosCameraUtils::PublishCameraInfo();

  if (this->depth_info_connect_count_ > 0)
  {
    this->sensor_update_time_ = this->parentSensor_->LastMeasurementTime();
#if GAZEBO_MAJOR_VERSION >= 8
    common::Time cur_time = this->world_->SimTime();
#else
    common::Time cur_time = this->world_->GetSimTime();
#endif
    if (this->sensor_update_time_ - this->last_depth_image_camera_info_update_time_ >= this->update_period_)
    {
      this->PublishCameraInfo(this->depth_image_camera_info_pub_);
      this->last_depth_image_camera_info_update_time_ = this->sensor_update_time_;
    }
  }
}

//@todo: publish disparity similar to openni_camera_deprecated/src/nodelets/openni_nodelet.cpp.
/*
#include <stereo_msgs/DisparityImage.h>
pub_disparity_ = comm_nh.advertise<stereo_msgs::DisparityImage > ("depth/disparity", 5, subscriberChanged2, subscriberChanged2);

void GazeboRosDepthCamera::PublishDisparityImage(const DepthImage& depth, ros::Time time)
{
  stereo_msgs::DisparityImagePtr disp_msg = boost::make_shared<stereo_msgs::DisparityImage > ();
  disp_msg->header.stamp                  = time;
  disp_msg->header.frame_id               = device_->isDepthRegistered () ? rgb_frame_id_ : depth_frame_id_;
  disp_msg->image.header                  = disp_msg->header;
  disp_msg->image.encoding                = sensor_msgs::image_encodings::TYPE_32FC1;
  disp_msg->image.height                  = depth_height_;
  disp_msg->image.width                   = depth_width_;
  disp_msg->image.step                    = disp_msg->image.width * sizeof (float);
  disp_msg->image.data.resize (disp_msg->image.height * disp_msg->image.step);
  disp_msg->T = depth.getBaseline ();
  disp_msg->f = depth.getFocalLength () * depth_width_ / depth.getWidth ();

  /// @todo Compute these values from DepthGenerator::GetDeviceMaxDepth() and the like
  disp_msg->min_disparity = 0.0;
  disp_msg->max_disparity = disp_msg->T * disp_msg->f / 0.3;
  disp_msg->delta_d = 0.125;

  depth.fillDisparityImage (depth_width_, depth_height_, reinterpret_cast<float*>(&disp_msg->image.data[0]), disp_msg->image.step);

  pub_disparity_.publish (disp_msg);
}
*/

}
