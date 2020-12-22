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
#include <gazebo/msgs/MessageTypes.hh>
#include <gazebo/physics/PhysicsIface.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/transport/Node.hh>

#include <gazebo_msgs/msg/performance_metrics.hpp>
#include <gazebo_msgs/msg/sensor_performance_metric.hpp>

#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/node.hpp>
#include <gazebo_ros/utils.hpp>

#include <rosgraph_msgs/msg/clock.hpp>
#include <std_srvs/srv/empty.hpp>

#include <memory>
#include <string>

#ifndef GAZEBO_ROS_HAS_PERFORMANCE_METRICS
#if \
  (GAZEBO_MAJOR_VERSION == 11 && GAZEBO_MINOR_VERSION > 1) || \
  (GAZEBO_MAJOR_VERSION == 9 && GAZEBO_MINOR_VERSION > 14)
#define GAZEBO_ROS_HAS_PERFORMANCE_METRICS
#endif
#endif  // ifndef GAZEBO_ROS_HAS_PERFORMANCE_METRICS

namespace gazebo_ros
{

class GazeboRosInitPrivate
{
public:
  /// Constructor
  GazeboRosInitPrivate();

  /// Callback when a world is created.
  /// \param[in] _world_name The world's name
  void OnWorldCreated(const std::string & _world_name);

  /// Publish simulation time.
  /// \param[in] _info World update information.
  void PublishSimTime(const gazebo::common::UpdateInfo & _info);

  /// Callback from ROS service to reset simulation.
  /// \param[in] req Empty request
  /// \param[out] res Empty response
  void OnResetSimulation(
    std_srvs::srv::Empty::Request::SharedPtr req,
    std_srvs::srv::Empty::Response::SharedPtr res);

  /// Callback from ROS service to reset world.
  /// \param[in] req Empty request
  /// \param[out] res Empty response
  void OnResetWorld(
    std_srvs::srv::Empty::Request::SharedPtr req,
    std_srvs::srv::Empty::Response::SharedPtr res);

  /// Callback from ROS service to pause physics.
  /// \param[in] req Empty request
  /// \param[out] res Empty response
  void OnPause(
    std_srvs::srv::Empty::Request::SharedPtr req,
    std_srvs::srv::Empty::Response::SharedPtr res);

  /// Callback from ROS service to unpause (play) physics.
  /// \param[in] req Empty request
  /// \param[out] res Empty response
  void OnUnpause(
    std_srvs::srv::Empty::Request::SharedPtr req,
    std_srvs::srv::Empty::Response::SharedPtr res);

#ifdef GAZEBO_ROS_HAS_PERFORMANCE_METRICS
  /// \brief Subscriber callback for performance metrics. This will be send in the ROS network
  /// \param[in] msg Received PerformanceMetrics message
  void onPerformanceMetrics(ConstPerformanceMetricsPtr & msg);
#endif

  /// \brief Keep a pointer to the world.
  gazebo::physics::WorldPtr world_;

  /// Gazebo-ROS node
  gazebo_ros::Node::SharedPtr ros_node_;

  /// Publishes simulation time
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;

  /// ROS service to handle requests to reset simulation.
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_simulation_service_;

  /// ROS service to handle requests to reset world.
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_world_service_;

  /// ROS service to handle requests to pause physics.
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr pause_service_;

  /// ROS service to handle requests to unpause physics.
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr unpause_service_;

  /// \brief ROS publisher to publish performance metrics.
  rclcpp::Publisher<gazebo_msgs::msg::PerformanceMetrics>::SharedPtr performance_metrics_pub_;

  /// Connection to world update event, called at every iteration
  gazebo::event::ConnectionPtr world_update_event_;

  /// To be notified once the world is created.
  gazebo::event::ConnectionPtr world_created_event_;

  /// Throttler for clock publisher.
  gazebo_ros::Throttler throttler_;

  /// Gazebo subscriber to receive the performance metrics message.
  gazebo::transport::SubscriberPtr performance_metric_sub_;

  /// Gazebo node for communication.
  gazebo::transport::NodePtr gz_node_;

  /// Default frequency for clock publisher.
  static constexpr double DEFAULT_PUBLISH_FREQUENCY = 10.;
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
  if (!rclcpp::ok()) {
    rclcpp::init(argc, argv);
    impl_->ros_node_ = gazebo_ros::Node::Get();
  } else {
    impl_->ros_node_ = gazebo_ros::Node::Get();
    RCLCPP_WARN(
      impl_->ros_node_->get_logger(),
      "gazebo_ros_init didn't initialize ROS "
      "because it's already initialized with other arguments");
  }

  // Offer transient local durability on the clock topic so that if publishing is infrequent (e.g.
  // the simulation is paused), late subscribers can receive the previously published message(s).
  impl_->clock_pub_ = impl_->ros_node_->create_publisher<rosgraph_msgs::msg::Clock>(
    "/clock",
    rclcpp::QoS(rclcpp::KeepLast(10)).transient_local());

#ifdef GAZEBO_ROS_HAS_PERFORMANCE_METRICS
  impl_->performance_metrics_pub_ =
    impl_->ros_node_->create_publisher<gazebo_msgs::msg::PerformanceMetrics>(
    "performance_metrics", 10);
#endif

  // Publish rate parameter
  auto rate_param = impl_->ros_node_->declare_parameter(
    "publish_rate",
    rclcpp::ParameterValue(GazeboRosInitPrivate::DEFAULT_PUBLISH_FREQUENCY));
  impl_->throttler_ = Throttler(rate_param.get<double>());

  impl_->world_update_event_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&GazeboRosInitPrivate::PublishSimTime, impl_.get(), std::placeholders::_1));

  // Get a callback when a world is created
  impl_->world_created_event_ = gazebo::event::Events::ConnectWorldCreated(
    std::bind(&GazeboRosInitPrivate::OnWorldCreated, impl_.get(), std::placeholders::_1));
}

#ifdef GAZEBO_ROS_HAS_PERFORMANCE_METRICS
void GazeboRosInitPrivate::onPerformanceMetrics(
  ConstPerformanceMetricsPtr & msg)
{
  gazebo_msgs::msg::PerformanceMetrics msg_ros;
  msg_ros.header.stamp = Convert<builtin_interfaces::msg::Time>(world_->SimTime());
  msg_ros.real_time_factor = msg->real_time_factor();
  for (auto sensor : msg->sensor()) {
    gazebo_msgs::msg::SensorPerformanceMetric sensor_msgs;
    sensor_msgs.sim_update_rate = sensor.sim_update_rate();
    sensor_msgs.real_update_rate = sensor.real_update_rate();
    sensor_msgs.name = sensor.name();

    if (sensor.has_fps()) {
      sensor_msgs.fps = sensor.fps();
    } else {
      sensor_msgs.fps = -1;
    }

    msg_ros.sensors.push_back(sensor_msgs);
  }

  performance_metrics_pub_->publish(msg_ros);
}
#endif

void GazeboRosInitPrivate::OnWorldCreated(const std::string & _world_name)
{
  // Only support one world
  world_created_event_.reset();

  world_ = gazebo::physics::get_world(_world_name);

  // Reset services
  reset_simulation_service_ = ros_node_->create_service<std_srvs::srv::Empty>(
    "reset_simulation",
    std::bind(
      &GazeboRosInitPrivate::OnResetSimulation, this,
      std::placeholders::_1, std::placeholders::_2));

  reset_world_service_ = ros_node_->create_service<std_srvs::srv::Empty>(
    "reset_world",
    std::bind(
      &GazeboRosInitPrivate::OnResetWorld, this,
      std::placeholders::_1, std::placeholders::_2));

  // Pause services
  pause_service_ = ros_node_->create_service<std_srvs::srv::Empty>(
    "pause_physics",
    std::bind(
      &GazeboRosInitPrivate::OnPause, this,
      std::placeholders::_1, std::placeholders::_2));

  unpause_service_ = ros_node_->create_service<std_srvs::srv::Empty>(
    "unpause_physics",
    std::bind(
      &GazeboRosInitPrivate::OnUnpause, this,
      std::placeholders::_1, std::placeholders::_2));

#ifdef GAZEBO_ROS_HAS_PERFORMANCE_METRICS
  // Initialize gazebo transport node
  gz_node_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
  gz_node_->Init(world_->Name());
#endif
}

GazeboRosInitPrivate::GazeboRosInitPrivate()
: throttler_(DEFAULT_PUBLISH_FREQUENCY)
{
}

void GazeboRosInitPrivate::PublishSimTime(const gazebo::common::UpdateInfo & _info)
{
  if (!throttler_.IsReady(_info.realTime)) {
    return;
  }

  rosgraph_msgs::msg::Clock clock;
  clock.clock = gazebo_ros::Convert<builtin_interfaces::msg::Time>(_info.simTime);
  clock_pub_->publish(clock);

#ifdef GAZEBO_ROS_HAS_PERFORMANCE_METRICS
  if (!performance_metric_sub_ && performance_metrics_pub_->get_subscription_count() > 0) {
    // Subscribe to gazebo performance_metrics topic if there are ros subscribers
    performance_metric_sub_ = gz_node_->Subscribe(
      "/gazebo/performance_metrics",
      &GazeboRosInitPrivate::onPerformanceMetrics, this);
  } else if (performance_metric_sub_ && performance_metrics_pub_->get_subscription_count() == 0) {
    // Unsubscribe from gazebo performance_metrics topic if there are no more ros subscribers
    performance_metric_sub_.reset();
  }
#endif
}

void GazeboRosInitPrivate::OnResetSimulation(
  std_srvs::srv::Empty::Request::SharedPtr,
  std_srvs::srv::Empty::Response::SharedPtr)
{
  world_->Reset();
}

void GazeboRosInitPrivate::OnResetWorld(
  std_srvs::srv::Empty::Request::SharedPtr,
  std_srvs::srv::Empty::Response::SharedPtr)
{
  world_->ResetEntities(gazebo::physics::Base::MODEL);
}

void GazeboRosInitPrivate::OnPause(
  std_srvs::srv::Empty::Request::SharedPtr,
  std_srvs::srv::Empty::Response::SharedPtr)
{
  world_->SetPaused(true);
}

void GazeboRosInitPrivate::OnUnpause(
  std_srvs::srv::Empty::Request::SharedPtr,
  std_srvs::srv::Empty::Response::SharedPtr)
{
  world_->SetPaused(false);
}

GZ_REGISTER_SYSTEM_PLUGIN(GazeboRosInit)

}  // namespace gazebo_ros
