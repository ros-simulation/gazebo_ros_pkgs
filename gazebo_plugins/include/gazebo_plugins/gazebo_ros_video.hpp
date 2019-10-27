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
 * Desc: Video plugin for displaying ROS image topics on Ogre textures
 * Author: Piyush Khandelwal
 * Date: 26 July 2013
 */

#ifndef GAZEBO_PLUGINS__GAZEBO_ROS_VIDEO_HPP_
#define GAZEBO_PLUGINS__GAZEBO_ROS_VIDEO_HPP_

#include <gazebo/common/Plugin.hh>

#include <memory>

namespace gazebo_plugins
{
class GazeboRosVideoPrivate;

/// Video plugin for displaying ROS image topics on Ogre textures
/*
 *
 * \author Piyush Khandelwal (piyushk@gmail.com)
 *
 */
/**
  Example Usage:
  \code{.xml}
    <plugin name="gazebo_ros_video" filename="libgazebo_ros_video.so">
      <!-- Images are subscribed from: /custom_ns/custom_img -->
      <ros>
        <!-- Add a namespace -->
        <namespace>/custom_ns</namespace>
        <!-- remap image subscribe topic -->
        <remapping>~/image_raw:=custom_img</remapping>
      </ros>

      <!-- Dimensions of image -->
      <height>240</height>
      <weight>320</weight>

    </plugin>
  \endcode
*/

class GazeboRosVideo : public gazebo::VisualPlugin
{
public:
  /// Constructor
  GazeboRosVideo();

  /// Destructor
  virtual ~GazeboRosVideo();

protected:
  // Documentation inherited
  void Load(gazebo::rendering::VisualPtr _parent, sdf::ElementPtr _sdf) override;

private:
  /// Private data pointer
  std::unique_ptr<GazeboRosVideoPrivate> impl_;
};
}  // namespace gazebo_plugins

#endif  // GAZEBO_PLUGINS__GAZEBO_ROS_VIDEO_HPP_
