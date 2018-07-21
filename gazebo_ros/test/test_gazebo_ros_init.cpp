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

#include <unistd.h>
#include <stdlib.h>

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <thread>

using namespace std::chrono_literals;

class GazeboRosInitTest : public ::testing::Test
{
public:
  void TearDown();
  int pid_{-1};
};

void GazeboRosInitTest::TearDown()
{
  // If fork failed, don't need to do anything
  if (pid_ < 0) {
    return;
  }

  // Kill gazebo (simulating ^C command)
  if (kill(pid_, SIGINT)) {
    throw std::runtime_error("could not kill");
  }

  // Wait for gazebo to terminate
  wait(nullptr);
}

TEST_F(GazeboRosInitTest, load)
{
  // Fork process so gazebo can be run as child
  pid_ = fork();
  if (pid_ < 0) {
    throw std::runtime_error("fork failed");
  }

  // Child process
  if (0 == pid_) {

    // Check that Gazebo is loaded without any issues
    ASSERT_TRUE(execlp("gzserver", "/usr/bin/gzserver", "--verbose", "-s", "libgazebo_ros_init.so",
        NULL));

    exit(1);
  }

  // Wait a bit so Gazebo is fully loaded
  for (unsigned int i = 0; i < 100; ++i) {
    std::this_thread::sleep_for(30ms);
  }
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

