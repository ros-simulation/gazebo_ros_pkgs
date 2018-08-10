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

#include <gazebo_ros/node.hpp>
#include <gazebo_ros/utils.hpp>
#include <gazebo_ros/conversions.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <gazebo/common/Plugin.hh>

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
  rclcpp::init(argc, argv);

  impl_->ros_node_ = gazebo_ros::Node::Create("gazebo_ros_clock");
  impl_->clock_pub_ = impl_->ros_node_->create_publisher<rosgraph_msgs::msg::Clock>("/clock");

  // Get publish rate from parameter if set
  rclcpp::Parameter rate_param;
  if (impl_->ros_node_->get_parameter("publish_rate", rate_param)) {
    if (rclcpp::ParameterType::PARAMETER_DOUBLE == rate_param.get_type()) {
      impl_->throttler_ = Throttler(rate_param.as_double());
    } else if (rclcpp::ParameterType::PARAMETER_INTEGER == rate_param.get_type()) {
      impl_->throttler_ = Throttler(rate_param.as_int());
    } else {
      RCLCPP_WARN(impl_->ros_node_->get_logger(),
        "Could not read value of param publish_rate [%s] as double/int, using default %ghz.",
        rate_param.value_to_string().c_str(),
        GazeboRosInitPrivate::DEFAULT_PUBLISH_FREQUENCY);
    }
  }

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
