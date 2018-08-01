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

#include <gazebo_ros/utils.hpp>
#include <gazebo/sensors/GaussianNoiseModel.hh>
#include <gtest/gtest.h>

#include <memory>

TEST(TestUtils, NoiseVariance)
{
  // Test no noise has 0 variance
  {
    auto noise = gazebo::sensors::Noise(gazebo::sensors::Noise::NoiseType::NONE);
    EXPECT_EQ(0.0, gazebo_ros::NoiseVariance(noise));
  }

  // Test gaussian noise type's variance is square of stddev
  {
    // SDF for gaussian noise model
    auto stddev_description = std::make_shared<sdf::Element>();
    stddev_description->SetName("stddev");
    stddev_description->AddValue("double", "5.0", true);
    auto noise_element = std::make_shared<sdf::Element>();
    noise_element->SetName("noise");
    noise_element->AddElementDescription(stddev_description);

    // Create noise with sdf
    auto noise = gazebo::sensors::GaussianNoiseModel();
    noise.Load(noise_element);

    EXPECT_EQ(25.0, gazebo_ros::NoiseVariance(noise));
  }
}

TEST(TestUtils, ScopedNameBase)
{
  EXPECT_EQ(gazebo_ros::ScopedNameBase("afgfgf::vnjkkfds::my"), "my");
  EXPECT_EQ(gazebo_ros::ScopedNameBase("base"), "base");
  EXPECT_EQ(gazebo_ros::ScopedNameBase(""), "");
  EXPECT_EQ(gazebo_ros::ScopedNameBase("fdfd::"), "fdfd::");
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
