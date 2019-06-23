// Copyright 2019 Open Source Robotics Foundation, Inc.
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

#ifndef GAZEBO_ROS__GAZEBO_ROS_EFFORT_HPP_
#define GAZEBO_ROS__GAZEBO_ROS_EFFORT_HPP_

#include <gazebo/common/Plugin.hh>

#include <memory>

namespace gazebo_ros
{

class GazeboRosEffortPrivate;

/// Provides services and topics to
/// Apply and clear wrenches on gazebo bodies.
/// Apply and clear forces on gazebo joints
class GazeboRosEffort : public gazebo::SystemPlugin
{
public:
  /// Constructor
  GazeboRosEffort();

  /// Destructor
  virtual ~GazeboRosEffort();

  // Documentation inherited
  void Load(int argc, char ** argv) override;

private:
  std::unique_ptr<GazeboRosEffortPrivate> impl_;
};

}  // namespace gazebo_ros
#endif  // GAZEBO_ROS__GAZEBO_ROS_EFFORT_HPP_
