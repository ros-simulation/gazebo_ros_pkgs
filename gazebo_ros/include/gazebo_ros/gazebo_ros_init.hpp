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

#ifndef GAZEBO_ROS__GAZEBO_ROS_INIT_HPP_
#define GAZEBO_ROS__GAZEBO_ROS_INIT_HPP_

#include <gazebo/common/Plugin.hh>

#include <memory>

namespace gazebo_ros
{

class GazeboRosInitPrivate;

/// Initializes ROS with the system arguments passed to Gazebo (i.e. calls rclcpp::init).
/// Also publishes the latest simtime to /clock
class GazeboRosInit : public gazebo::SystemPlugin
{
public:
  /// Constructor
  GazeboRosInit();

  /// Destructor
  virtual ~GazeboRosInit();

  // Documentation inherited
  void Load(int argc, char ** argv) override;

private:
  std::unique_ptr<GazeboRosInitPrivate> impl_;
};

}  // namespace gazebo_ros
#endif  // GAZEBO_ROS__GAZEBO_ROS_INIT_HPP_
