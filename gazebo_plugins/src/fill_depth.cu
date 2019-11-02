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

#include <gazebo_plugins/fill_depth.hpp>

#include <iostream>
#include <limits>

class FillDepthPrivate
{
public:
  struct parameters
  {
    /// Image height
    int height;

    /// Image width
    int width;

    /// Size of single image channel
    int size;

    /// Camera focal length
    double fl;

    /// Minimum depth value in image
    double min_depth;

    /// Maximum depth value in image
    double max_depth;

    /// Value of infinity
    float infinity;
  } params;

  /// Pointer to struct parameters
  parameters * gpu_params;

  /// Pointer to depth information stored in GPU
  float * gpu_image_depth;

  /// Pointer to RGB image stored in GPU
  uint8_t * gpu_image_rgb;

  /// Pointer to pointcloud computed in GPU
  float * gpu_cloud;

  /// Pointer to depth image computed in GPU
  float * gpu_depth;

  /// Number of GPU threads to use
  int threads;

  /// Number of GPU blocks to use
  int blocks;

  /// Size of single image channel
  int size;
};

extern "C" __global__ void fill(
  FillDepthPrivate::parameters * params,
  float * image_depth, uint8_t * image_rgb, float * depth, float * cloud)
{
  double pAngle = 0;
  double yAngle = 0;

  typedef union {
    uint8_t rgb_int[3];
    float rgb_float;
  } rgb_data;
  rgb_data data;

  for (uint32_t index = blockIdx.x * blockDim.x + threadIdx.x;
    index < params->size; index += blockDim.x * gridDim.x)
  {
    uint32_t i = index % params->width;
    uint32_t j = index / params->width;

    if (params->height > 1) {
      pAngle = atan2(
        static_cast<double>(j) - 0.5 * static_cast<double>(params->height - 1), params->fl);
    }

    if (params->width > 1) {
      yAngle = atan2(
        static_cast<double>(i) - 0.5 * static_cast<double>(params->width - 1), params->fl);
    }
    // in optical frame
    // hardcoded rotation rpy(-M_PI/2, 0, -M_PI/2) is built-in
    // to urdf, where the *_optical_frame should have above relative
    // rotation from the physical camera *_frame

    float distance = image_depth[index];

    if (distance > params->min_depth && distance < params->max_depth) {
      depth[index] = distance;

      auto x = static_cast<float>(distance * tan(yAngle));
      auto y = static_cast<float>(distance * tan(pAngle));

      cloud[index * 8] = x;
      cloud[index * 8 + 1] = y;
      cloud[index * 8 + 2] = distance;
    } else if (distance <= params->min_depth) {
      depth[index] = -params->infinity;

      cloud[index * 8] = -params->infinity;
      cloud[index * 8 + 1] = -params->infinity;
      cloud[index * 8 + 2] = -params->infinity;
    } else {
      depth[index] = params->infinity;

      cloud[index * 8] = params->infinity;
      cloud[index * 8 + 1] = params->infinity;
      cloud[index * 8 + 2] = params->infinity;
    }

    data.rgb_int[0] = image_rgb[index * 3];
    data.rgb_int[1] = image_rgb[index * 3 + 1];
    data.rgb_int[2] = image_rgb[index * 3 + 2];

    cloud[index * 8 + 4] = data.rgb_float;
  }
}

FillDepth::FillDepth()
{
  impl_ = (FillDepthPrivate *) std::malloc(sizeof(FillDepthPrivate));
  cudaMalloc(reinterpret_cast<void **>(&(impl_->gpu_params)), sizeof(impl_->params));
}

FillDepth::~FillDepth()
{
  cudaFree(impl_->gpu_params);
  cudaFree(impl_->gpu_image_depth);
  cudaFree(impl_->gpu_image_rgb);
  cudaFree(impl_->gpu_cloud);
  free(impl_);
}

void FillDepth::initialize(
  unsigned int height, unsigned int width, double fl, double min_depth, double max_depth)
{
  impl_->params.height = height;
  impl_->params.width = width;
  impl_->params.fl = fl;
  impl_->params.min_depth = min_depth;
  impl_->params.max_depth = max_depth;
  impl_->size = impl_->params.width * impl_->params.height;
  impl_->params.size = impl_->size;
  impl_->params.infinity = std::numeric_limits<float>::infinity();

  cudaMemcpy(impl_->gpu_params, &impl_->params, sizeof(impl_->params), cudaMemcpyHostToDevice);
  cudaMalloc(reinterpret_cast<void **>(&(impl_->gpu_image_depth)), impl_->size * sizeof(float));
  cudaMalloc(reinterpret_cast<void **>(&(impl_->gpu_image_rgb)), 3 * impl_->size * sizeof(uint8_t));
  cudaMalloc(reinterpret_cast<void **>(&(impl_->gpu_depth)), impl_->size * sizeof(float));
  cudaMalloc(reinterpret_cast<void **>(&(impl_->gpu_cloud)), 8 * impl_->size * sizeof(float));

  impl_->threads = 512;
  impl_->blocks = (impl_->size + impl_->threads - 1) / impl_->threads;
}

void FillDepth::compute(
  const float * image_depth, uint8_t * image_rgb, float * depth, float * cloud)
{
  cudaMemcpy(
    impl_->gpu_image_depth, image_depth, impl_->size * sizeof(float), cudaMemcpyHostToDevice);
  cudaMemcpy(
    impl_->gpu_image_rgb, image_rgb, 3 * impl_->size * sizeof(uint8_t), cudaMemcpyHostToDevice);

  fill <<< impl_->blocks, impl_->threads >>> (
    impl_->gpu_params, impl_->gpu_image_depth, impl_->gpu_image_rgb,
    impl_->gpu_depth, impl_->gpu_cloud);

  cudaMemcpy(depth, impl_->gpu_depth, impl_->size * sizeof(float), cudaMemcpyDeviceToHost);
  cudaMemcpy(cloud, impl_->gpu_cloud, 8 * impl_->size * sizeof(float), cudaMemcpyDeviceToHost);
}
