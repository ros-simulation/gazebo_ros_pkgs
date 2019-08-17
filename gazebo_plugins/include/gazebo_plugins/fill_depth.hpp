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

class FillDepth
{
public:
  struct parameters
  {
    int height;
    int width;
    int size;
    double fl;
    double min_depth;
    double max_depth;
    float infinity;
  } params;

  FillDepth();

  ~FillDepth();

  void initialize(
    unsigned int height, unsigned int width, double fl, double min_depth, double max_depth);

  void compute(const float * image_depth, uint8_t * image_rgb, float * depth, float * cloud);

private:
  parameters * gpu_params;
  float * gpu_image_depth;
  uint8_t * gpu_image_rgb;
  float * gpu_cloud;
  float * gpu_depth;
  int threads, blocks;
  int size;
};

#endif  // GAZEBO_PLUGINS__FILL_DEPTH_HPP_
