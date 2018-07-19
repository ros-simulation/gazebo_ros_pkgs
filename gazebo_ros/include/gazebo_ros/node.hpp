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

#ifndef GAZEBO_ROS__NODE_HPP_
#define GAZEBO_ROS__NODE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <gazebo_ros/executor.hpp>

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <utility>

namespace gazebo_ros
{
/// ROS Node for gazebo plugins
/**
 * \class Node node.hpp <gazebo_ros/node.hpp>
 * Wrapper around an rclcpp::Node which ensures all instances share an executor.
 * \include gazebo_ros/node.hpp
 */
class Node : public rclcpp::Node
{
public:
  /// Shared pointer to a #gazebo_ros::Node
  typedef std::shared_ptr<Node> SharedPtr;

  /// Destructor
  virtual ~Node();

  /// Create a #gazebo_ros::Node and add it to the global #gazebo_ros::Executor.
  /**
   * \param[in] node_name Name for the new node to create
   * \return A shared pointer to a new #gazebo_ros::Node, or nullptr if failed to create.
   */
  static SharedPtr Create(const std::string & node_name);

  /// Create a #gazebo_ros::Node and add it to the global #gazebo_ros::Executor.
  /**
   * \details Forwards arguments to the constructor for rclcpp::Node
   * \param[in] args List of arguments to pass to <a href="http://docs.ros2.org/latest/api/rclcpp/classrclcpp_1_1_node.html">rclcpp::Node</a>
   * \return A shared pointer to a new #gazebo_ros::Node, or nullptr if failed to create.
   */
  template<typename ... Args>
  static SharedPtr Create(Args && ... args);

private:
  /// Inherit constructor
  using rclcpp::Node::Node;

  /// Points to #static_executor_, so that when all #gazebo_ros::Node instances are destroyed, the
  /// executor thread is too
  std::shared_ptr<Executor> executor_;

  /// Locks #initialized_ and #executor_
  static std::mutex lock_;

  /// Points to an #gazebo_ros::Executor shared between all #gazebo_ros::Node instances
  static std::weak_ptr<Executor> static_executor_;
};

template<typename ... Args>
Node::SharedPtr Node::Create(Args && ... args)
{
  // Contruct Node by forwarding arguments
  // TODO(chapulina): use rclcpp::isInitialized() once that's available, see
  // https://github.com/ros2/rclcpp/issues/518
  Node::SharedPtr node;
  try
  {
    node = std::make_shared<Node>(std::forward<Args>(args) ...);
  }
  catch(rclcpp::exceptions::RCLError e)
  {
    RCLCPP_WARN(rclcpp::get_logger("gazebo_ros_node"),
      "Called Node::Create before rclcpp::init, node not created.");
    return nullptr;
  }

  std::lock_guard<std::mutex> l(lock_);

  // Store shared pointer to static executor in this object
  node->executor_ = static_executor_.lock();

  // If executor has not been contructed yet, do so now
  if (!node->executor_) {
    node->executor_ = std::make_shared<Executor>();
    static_executor_ = node->executor_;
  }

  // Add new node to the executor so its callbacks are called
  node->executor_->add_node(node);

  return node;
}
}  // namespace gazebo_ros
#endif  // GAZEBO_ROS__NODE_HPP_
