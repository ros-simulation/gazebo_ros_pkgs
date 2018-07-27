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

#ifndef GAZEBO_PLUGINS__GAZEBO_ROS_TEMPLATE_HPP_
#define GAZEBO_PLUGINS__GAZEBO_ROS_TEMPLATE_HPP_

#include <gazebo/common/Plugin.hh>

// For std::unique_ptr, could be removed
#include <memory>

namespace gazebo
{
// Forward declaration of private data class.
class GazeboRosTemplatePrivate;

class GazeboRosTemplate : public ModelPlugin
{
public:
  /// Constructor
  GazeboRosTemplate();

  /// Destructor
  virtual ~GazeboRosTemplate();

  /// Gazebo calls this when the plugin is loaded.
  /// \param[in] model Pointer to parent model.
  /// \param[in] sdf SDF element containing user-defined parameters.
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf);

protected:
  /// Optional callback to be called at every simulation iteration.
  virtual void OnUpdate();

private:
  /// Recommended PIMPL pattern. This variable should hold all private
  /// data members.
  std::unique_ptr<GazeboRosTemplatePrivate> d_ptr_;
};
}  // namespace gazebo

#endif  // GAZEBO_PLUGINS__GAZEBO_ROS_TEMPLATE_HPP_
