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

#include "gazebo_ros/gazebo_ros_init.hpp"

#include <gazebo/common/Plugin.hh>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/node.hpp>
#include <gazebo_ros/utils.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

#include <memory>

namespace gazebo_ros
{

class GazeboRosInitPrivate
{
public:
  GazeboRosInitPrivate();
  gazebo_ros::Node::SharedPtr ros_node_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
  gazebo::event::ConnectionPtr world_update_event_;
  gazebo_ros::Throttler throttler_;
  static constexpr double DEFAULT_PUBLISH_FREQUENCY = 10.;

  void PublishSimTime(const gazebo::common::UpdateInfo & _info);
};

GazeboRosInit::GazeboRosInit()
: impl_(std::make_unique<GazeboRosInitPrivate>())
{
}

GazeboRosInit::~GazeboRosInit()
{
}

void GazeboRosInit::Load(int argc, char ** argv)
{
  // Initialize ROS with arguments
  if (!rclcpp::is_initialized()) {
    rclcpp::init(argc, argv);
    impl_->ros_node_ = gazebo_ros::Node::Get();
  } else {
    impl_->ros_node_ = gazebo_ros::Node::Get();
    RCLCPP_WARN(impl_->ros_node_->get_logger(),
      "gazebo_ros_init didn't initialize ROS "
      "because it's already initialized with other arguments");
  }

  // Offer transient local durability on the clock topic so that if publishing is infrequent (e.g.
  // the simulation is paused), late subscribers can receive the previously published message(s).
  impl_->clock_pub_ = impl_->ros_node_->create_publisher<rosgraph_msgs::msg::Clock>(
    "/clock",
    rclcpp::QoS(rclcpp::KeepLast(10)).transient_local());

  // Publish rate parameter
  auto rate_param = impl_->ros_node_->declare_parameter(
    "publish_rate",
    rclcpp::ParameterValue(GazeboRosInitPrivate::DEFAULT_PUBLISH_FREQUENCY));
  impl_->throttler_ = Throttler(rate_param.get<double>());

  impl_->world_update_event_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&GazeboRosInitPrivate::PublishSimTime, impl_.get(), std::placeholders::_1));
}

GazeboRosInitPrivate::GazeboRosInitPrivate()
: throttler_(DEFAULT_PUBLISH_FREQUENCY)
{
}

void GazeboRosInitPrivate::PublishSimTime(const gazebo::common::UpdateInfo & _info)
{
  if (!throttler_.IsReady(_info.realTime)) {return;}

  rosgraph_msgs::msg::Clock clock;
  clock.clock = gazebo_ros::Convert<builtin_interfaces::msg::Time>(_info.simTime);
  clock_pub_->publish(clock);
}

GZ_REGISTER_SYSTEM_PLUGIN(GazeboRosInit)

}  // namespace gazebo_ros
