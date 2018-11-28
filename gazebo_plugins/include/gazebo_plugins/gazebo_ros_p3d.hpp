/*
 * Copyright 2012 Open Source Robotics Foundation
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
 * Desc: 3D position interface for ground truth.
 * Author: Sachin Chitta and John Hsu
 * Date: 1 June 2008
 */

#ifndef GAZEBO_PLUGINS__GAZEBO_ROS_P3D_HPP_
#define GAZEBO_PLUGINS__GAZEBO_ROS_P3D_HPP_

#include <memory>
#include "gazebo/physics/physics.hh"
#include "gazebo/common/Plugin.hh"

namespace gazebo_plugins
{

class GazeboRosP3DPrivate;

class GazeboRosP3D : public gazebo::ModelPlugin
{
public:
  /// \brief Constructor
  GazeboRosP3D();

  /// \brief Destructor
  virtual ~GazeboRosP3D();

  /// \brief Load the controller
  void Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf);

protected:
  /// \brief Update the controller
  virtual void UpdateChild();

  /// \brief Gaussian noise generator
  double GaussianKernel(double mu, double sigma);

private:
  /// Private data pointer
  std::unique_ptr<GazeboRosP3DPrivate> impl_;
};

}  // namespace gazebo_plugins

#endif  // GAZEBO_PLUGINS__GAZEBO_ROS_P3D_HPP_
