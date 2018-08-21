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
#include <gazebo/sensors/sensors.hh>
#include <gtest/gtest.h>

#include <memory>
#include <string>

TEST(TestUtils, NoiseVariance)
{
  // Test no noise has 0 variance
  {
    auto noise = std::make_shared<gazebo::sensors::Noise>(gazebo::sensors::Noise::NoiseType::NONE);
    EXPECT_EQ(0., gazebo_ros::NoiseVariance(noise));
    EXPECT_EQ(0., gazebo_ros::NoiseVariance(*noise));
  }

  // Test custom noise errors with -1
  {
    auto noise =
      std::make_shared<gazebo::sensors::Noise>(gazebo::sensors::Noise::NoiseType::CUSTOM);
    EXPECT_EQ(-1., gazebo_ros::NoiseVariance(noise));
    EXPECT_EQ(-1., gazebo_ros::NoiseVariance(*noise));
  }

  // Test Gaussian noise type's variance is square of stddev
  {
    // SDF for Gaussian noise model
    auto stddev_description = std::make_shared<sdf::Element>();
    stddev_description->SetName("stddev");
    stddev_description->AddValue("double", "5.0", true);
    auto noise_element = std::make_shared<sdf::Element>();
    noise_element->SetName("noise");
    noise_element->AddElementDescription(stddev_description);

    // Create noise with sdf
    auto noise = std::make_shared<gazebo::sensors::GaussianNoiseModel>();
    noise->Load(noise_element);

    EXPECT_EQ(25.0, gazebo_ros::NoiseVariance(noise));
    EXPECT_EQ(25.0, gazebo_ros::NoiseVariance(*noise));
  }

  {
    // Test nullptr noise
    gazebo::sensors::NoisePtr noise = nullptr;
    EXPECT_EQ(0., gazebo_ros::NoiseVariance(noise));
  }
}

TEST(TestUtils, ScopedNameBase)
{
  EXPECT_EQ(gazebo_ros::ScopedNameBase("afgfgf::vnjkkfds::my"), "my");
  EXPECT_EQ(gazebo_ros::ScopedNameBase("base"), "base");
  EXPECT_EQ(gazebo_ros::ScopedNameBase(""), "");
  EXPECT_EQ(gazebo_ros::ScopedNameBase("fdfd::"), "fdfd::");
}

TEST(TestUtils, SensorFrameID)
{
  {
    auto stddev_description = std::make_shared<sdf::Element>();
    stddev_description->SetName("frame_name");
    stddev_description->AddValue("string", "", false);
    auto noise_element = std::make_shared<sdf::Element>();
    noise_element->SetName("noise");
    noise_element->AddElementDescription(stddev_description);
    noise_element->GetElement("frame_name")->Set<std::string>("foo");
    gazebo::sensors::CameraSensor s;
    EXPECT_EQ(gazebo_ros::SensorFrameID(s, *noise_element), "foo");
  }

  {
    sdf::Element sdf;
    gazebo::sensors::CameraSensor s;
    s.SetParent("world::my_robot::link", 0);
    EXPECT_EQ(gazebo_ros::SensorFrameID(s, sdf), "link");
  }
}

TEST(TestUtils, Throttler)
{
  using Time = gazebo::common::Time;
  using Throttler = gazebo_ros::Throttler;
  {
    // Test negative period (always ready)
    Throttler t(-1);
    EXPECT_TRUE(t.IsReady(Time()));
    EXPECT_TRUE(t.IsReady(Time(50, 0)));
    EXPECT_TRUE(t.IsReady(Time(20, 0)));
    EXPECT_TRUE(t.IsReady(Time(1E3, 0)));
  }
  {
    // Test frequency
    Throttler t(2.0);
    EXPECT_TRUE(t.IsReady(Time(0, 6E8)));
    EXPECT_FALSE(t.IsReady(Time(0, 7E8)));
    EXPECT_FALSE(t.IsReady(Time(0, 8E8)));
    EXPECT_TRUE(t.IsReady(Time(1E4, 0)));
    EXPECT_TRUE(t.IsReady(Time(1E4, 6E8)));
    EXPECT_FALSE(t.IsReady(Time(1E4, 9E8)));
  }
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
