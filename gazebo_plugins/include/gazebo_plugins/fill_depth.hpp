// Copyright 2019 Open Source Robotics Foundation
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

#ifndef GAZEBO_PLUGINS__FILL_DEPTH_HPP_
#define GAZEBO_PLUGINS__FILL_DEPTH_HPP_

#include <cuda.h>

class FillDepthPrivate;

class FillDepth
{
public:
  /// Constructor
  FillDepth();

  /// Destructor
  ~FillDepth();

  /// Initializes parameters and allocates GPU memory
  /*
  * \param[in] height Image height
  * \param[in] width Image width
  * \param[in] fl Camera focal length
  * \param[in] min_depth Minimum depth in image
  * \param[in] max_depth Maximum depth in image
  */
  void initialize(
    unsigned int _height, unsigned int _width,
    double _fl,
    double _min_depth, double _max_depth);

  /// Computes depth image and pointcloud
  /*
  * \param[in] image_depth Depth image
  * \param[in] image_rgb RGB image
  * \param[out] depth Depth image
  * \param[out] cloud PointCloud
  */
  void compute(const float * image_depth, uint8_t * image_rgb, float * depth, float * cloud);

private:
  /// Private data pointer
  FillDepthPrivate * impl_;
};

#endif  // GAZEBO_PLUGINS__FILL_DEPTH_HPP_
