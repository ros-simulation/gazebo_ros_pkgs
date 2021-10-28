// Copyright 2013 Open Source Robotics Foundation, Inc.
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


/*
 * \file  gazebo_ros_video.cpp
 *
 * \brief Video plugin for displaying ROS image topics on Ogre textures
 *
 * \author Piyush Khandelwal
 *
 */

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.h>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo_plugins/gazebo_ros_video.hpp>
#include <gazebo_ros/node.hpp>
#ifdef IGN_PROFILER_ENABLE
#include <ignition/common/Profiler.hh>
#endif

#include <memory>
#include <string>

namespace gazebo_plugins
{
/**
 * Helper class for GazeboRosVideo
 * The class deals with the image conversions required for display on Gazebo
 * Ogre texture
 **/
class VideoVisual : public gazebo::rendering::Visual
{
public:
  /// Constructor
  VideoVisual(
    const std::string & name, gazebo::rendering::VisualPtr parent,
    int height, int width);

  /// Destructor
  virtual ~VideoVisual();

  /// Resize image and copy image data into pixel buffer.
  /// \param[in] image Subscribed image
  void render(const cv::Mat & image);

private:
  /// Instance of Ogre Texture manager
  Ogre::TexturePtr texture_;

  /// Height of image to be displayed
  int height_;

  /// Width of image to be displayed
  int width_;
};

VideoVisual::VideoVisual(
  const std::string & name,
  gazebo::rendering::VisualPtr parent, int height,
  int width)
: gazebo::rendering::Visual(name, parent), height_(height), width_(width)
{
  texture_ = Ogre::TextureManager::getSingleton().createManual(
    name + "__VideoTexture__",
    Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
    Ogre::TEX_TYPE_2D, width_, height_, 0, Ogre::PF_BYTE_BGRA,
    Ogre::TU_DYNAMIC_WRITE_ONLY_DISCARDABLE);

  Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().create(
    name + "__VideoMaterial__", "General");
  material->getTechnique(0)->getPass(0)->createTextureUnitState(
    name + "__VideoTexture__");
  material->setReceiveShadows(false);

  double factor = 1.0;

  Ogre::ManualObject mo(name + "__VideoObject__");
  mo.begin(name + "__VideoMaterial__", Ogre::RenderOperation::OT_TRIANGLE_LIST);

  mo.position(-factor / 2, factor / 2, 0.51);
  mo.textureCoord(0, 0);

  mo.position(factor / 2, factor / 2, 0.51);
  mo.textureCoord(1, 0);

  mo.position(factor / 2, -factor / 2, 0.51);
  mo.textureCoord(1, 1);

  mo.position(-factor / 2, -factor / 2, 0.51);
  mo.textureCoord(0, 1);

  mo.triangle(0, 3, 2);
  mo.triangle(2, 1, 0);
  mo.end();

  mo.convertToMesh(name + "__VideoMesh__");

  Ogre::MovableObject * obj =
    (Ogre::MovableObject *) GetSceneNode()->getCreator()->createEntity(
    name + "__VideoEntity__", name + "__VideoMesh__");
  obj->setCastShadows(false);
  AttachObject(obj);
}

VideoVisual::~VideoVisual() {}
void VideoVisual::render(const cv::Mat & image)
{
  // Fix image size if necessary
  const cv::Mat * image_ptr = &image;
  cv::Mat converted_image;
  if (image_ptr->rows != height_ || image_ptr->cols != width_) {
    cv::resize(*image_ptr, converted_image, cv::Size(width_, height_));
    image_ptr = &converted_image;
  }

  // Get the pixel buffer
  Ogre::HardwarePixelBufferSharedPtr pixelBuffer = texture_->getBuffer();

  // Lock the pixel buffer and get a pixel box
  pixelBuffer->lock(Ogre::HardwareBuffer::HBL_DISCARD);
  const Ogre::PixelBox & pixelBox = pixelBuffer->getCurrentLock();
  auto * pDest = static_cast<uint8_t *>(pixelBox.data);

  memcpy(pDest, image_ptr->data, height_ * width_ * 4);

  // Unlock the pixel buffer
  pixelBuffer->unlock();
}

class GazeboRosVideoPrivate
{
public:
  /// Callback when a image is received.
  /// \param[in] _msg Image command message.
  void processImage(const sensor_msgs::msg::Image::ConstSharedPtr msg);

  /// Callback to be called at every simulation iteration.
  void onUpdate();

  /// Connection to event called at every world iteration.
  gazebo::event::ConnectionPtr update_connection_;

  /// A shared_ptr to instance of Video Visual
  std::shared_ptr<VideoVisual> video_visual_;

  /// Pointer to OpenCV-compatible CvImage created by converting the subscribed
  /// images
  cv_bridge::CvImagePtr image_;

  /// Protect variables accessed on callbacks.
  std::mutex m_image_;

  /// Flag variable set on arrival of a new image
  bool new_image_available_;

  /// A pointer to the GazeboROS node.
  gazebo_ros::Node::SharedPtr rosnode_;

  /// Subscriber to images
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_subscriber_;
};

// Constructor
GazeboRosVideo::GazeboRosVideo()
: impl_(std::make_unique<GazeboRosVideoPrivate>())
{
}

// Destructor
GazeboRosVideo::~GazeboRosVideo()
{
  impl_->update_connection_.reset();
  impl_->rosnode_.reset();
}

void GazeboRosVideo::Load(
  gazebo::rendering::VisualPtr _parent,
  sdf::ElementPtr _sdf)
{
  impl_->rosnode_ = gazebo_ros::Node::Get(_sdf, _parent);

  // Get QoS profiles
  const gazebo_ros::QoS & qos = impl_->rosnode_->get_qos();

  int height = _sdf->Get<int>("height", 240).first;

  int width = _sdf->Get<int>("width", 320).first;

  impl_->video_visual_ = std::make_shared<VideoVisual>(
    _parent->Name() + "::video_visual::" + _sdf->Get<std::string>("name"), _parent, height, width);
  _parent->GetScene()->AddVisual(impl_->video_visual_);

  // Subscribe to the image topic
  impl_->camera_subscriber_ =
    impl_->rosnode_->create_subscription<sensor_msgs::msg::Image>(
    "~/image_raw", qos.get_subscription_qos("~/image_raw", rclcpp::QoS(1)),
    std::bind(&GazeboRosVideoPrivate::processImage, impl_.get(), std::placeholders::_1));

  impl_->new_image_available_ = false;

  impl_->update_connection_ = gazebo::event::Events::ConnectPreRender(
    std::bind(&GazeboRosVideoPrivate::onUpdate, impl_.get()));

  RCLCPP_INFO(
    impl_->rosnode_->get_logger(),
    "GazeboRosVideo has started. Subscribed to [%s]",
    impl_->camera_subscriber_->get_topic_name());
}

void GazeboRosVideoPrivate::onUpdate()
{
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE("GazeboRosVideoPrivate::onUpdate");
#endif
  std::lock_guard<std::mutex> scoped_lock(m_image_);
  if (new_image_available_) {
#ifdef IGN_PROFILER_ENABLE
    IGN_PROFILE_BEGIN("render");
#endif
    video_visual_->render(image_->image);
#ifdef IGN_PROFILER_ENABLE
    IGN_PROFILE_END();
#endif
  }
  new_image_available_ = false;
}

void GazeboRosVideoPrivate::processImage(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  // Get a reference to the image from the image message pointer
  std::lock_guard<std::mutex> scoped_lock(m_image_);
  // We get image with alpha channel as it allows memcpy onto ogre texture
  image_ = cv_bridge::toCvCopy(msg, "bgra8");
  new_image_available_ = true;
}
GZ_REGISTER_VISUAL_PLUGIN(GazeboRosVideo)
}  // namespace gazebo_plugins
