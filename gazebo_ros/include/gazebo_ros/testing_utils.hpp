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

#ifndef GAZEBO_ROS__TESTING_UTILS_HPP_
#define GAZEBO_ROS__TESTING_UTILS_HPP_

#include <unistd.h>
#include <stdlib.h>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <utility>
#include <vector>
#include <string>

namespace gazebo_ros
{


/// Helper class to run gzserver in a separate process and later terminate that process
class GazeboProcess
{
public:
  /// Start gzserver with a list of arguments
  /// \note The path and --verbose are automatically added
  explicit GazeboProcess(const std::vector<const char *> & args);

  /// Destructor
  ~GazeboProcess();

  /// Start gzserver with the arguments passed to constructor
  /// \return The result of fork(), either the pid of gazebo process or error if < 0
  int Run();

  /// Terminate the child gzserver process
  /// \return -1 if run() failed or has not been called,
  int Terminate();

private:
  /// Arguments to run gzserver with
  std::vector<const char *> arguments;

  // pid of gzserver
  int pid_ = -1;
};

GazeboProcess::GazeboProcess(const std::vector<const char *> & args)
{
  arguments = {"/usr/bin/gzserver", "--verbose"};
  arguments.insert(arguments.end(), args.begin(), args.end());
  arguments.push_back(nullptr);
}

GazeboProcess::~GazeboProcess()
{
  Terminate();
}

int GazeboProcess::Run()
{
  // Fork process so gazebo can be run as child
  pid_ = fork();

  // Child process
  if (0 == pid_) {
    // Run gazebo with arguments
    if (execvp("gzserver", const_cast<char **>(arguments.data()))) {
      // Exec failed, cannot return (in separate process), so just print errno
      printf("gzserver failed with errno=%d", errno);
      exit(1);
    }
  }

  if (pid_ < 0) {
    return errno;
  }

  // Parent process, return pid of child (or error produced by fork())
  return pid_;
}

int GazeboProcess::Terminate()
{
  // Return -1
  if (pid_ < 0) {
    return ECHILD;
  }

  // Kill gazebo (simulating ^C command)
  if (kill(pid_, SIGINT)) {
    return errno;
  }

  // Wait for gazebo to terminate
  if (waitpid(pid_, nullptr, 0) < 0) {
    return errno;
  }

  return 0;
}


/// Helper function to get the next message on a ROS topic with a timeout
/// \param node Pointer to a node to use to subscribe to the topic
/// \param topic Topic to subscribe to for message
/// \param timeout Maximum time to wait for message
/// \tparam T Message type to get
/// \return Shared pointer to new message, or nullptr if none received before timeout
template<typename T>
typename T::SharedPtr
get_message_or_timeout(
  rclcpp::Node::SharedPtr node, const std::string & topic,
  rclcpp::Duration timeout = rclcpp::Duration(10, 0))
{
  rclcpp::Clock clock;
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::atomic_bool msg_received(false);
  typename T::SharedPtr msg = nullptr;

  auto sub = node->create_subscription<T>(
    topic, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local(),
    [&msg_received, &msg](typename T::SharedPtr _msg) {
      // If this is the first message from this topic, increment the counter
      if (!msg_received.exchange(true)) {
        msg = _msg;
      }
    });

  // Wait for topic to be available
  using namespace std::literals::chrono_literals; // NOLINT
  auto timeout_absolute = clock.now() + timeout * 0.5;
  while (node->count_publishers(topic) == 0) {
    executor.spin_once(200ms);
  }
  EXPECT_GT(node->count_publishers(topic), 0u);

  // Wait until message is received or timeout occurs
  timeout_absolute = clock.now() + timeout * 0.5;

  while (false == msg_received && clock.now() < timeout_absolute) {
    executor.spin_once(200ms);
  }

  return msg;
}


}  // namespace gazebo_ros

#endif  // GAZEBO_ROS__TESTING_UTILS_HPP_
