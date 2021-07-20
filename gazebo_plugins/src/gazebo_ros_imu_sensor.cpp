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


#include <builtin_interfaces/msg/time.hpp>
#include <gazebo_plugins/gazebo_ros_imu_sensor.hpp>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/conversions/geometry_msgs.hpp>
#include <gazebo_ros/node.hpp>
#include <gazebo_ros/utils.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#ifdef IGN_PROFILER_ENABLE
#include <ignition/common/Profiler.hh>
#endif
#include <sensor_msgs/msg/imu.hpp>

#include <iostream>
#include <memory>
#include <string>

namespace gazebo_plugins
{

class GazeboRosImuSensorPrivate
{
public:
  /// Node for ros communication
  gazebo_ros::Node::SharedPtr ros_node_;
  /// Publish for imu message
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
  /// IMU message modified each update
  sensor_msgs::msg::Imu::SharedPtr msg_;
  /// IMU sensor this plugin is attached to
  gazebo::sensors::ImuSensorPtr sensor_;
  /// Event triggered when sensor updates
  gazebo::event::ConnectionPtr sensor_update_event_;

  /// Publish latest imu data to ROS
  void OnUpdate();
};

GazeboRosImuSensor::GazeboRosImuSensor()
: impl_(std::make_unique<GazeboRosImuSensorPrivate>())
{
}

GazeboRosImuSensor::~GazeboRosImuSensor()
{
}

void GazeboRosImuSensor::Load(gazebo::sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
  impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);

  // Get QoS profiles
  const gazebo_ros::QoS & qos = impl_->ros_node_->get_qos();

  impl_->sensor_ = std::dynamic_pointer_cast<gazebo::sensors::ImuSensor>(_sensor);
  if (!impl_->sensor_) {
    RCLCPP_ERROR(impl_->ros_node_->get_logger(), "Parent is not an imu sensor. Exiting.");
    return;
  }

  bool initial_orientation_as_reference = false;
  if (!_sdf->HasElement("initial_orientation_as_reference")) {
    RCLCPP_INFO_STREAM(
      impl_->ros_node_->get_logger(),
      "<initial_orientation_as_reference> is unset, using default value of false "
      "to comply with REP 145 (world as orientation reference)");
  } else {
    initial_orientation_as_reference = _sdf->Get<bool>("initial_orientation_as_reference");
  }

  if (initial_orientation_as_reference) {
    RCLCPP_WARN_STREAM(
      impl_->ros_node_->get_logger(),
      "<initial_orientation_as_reference> set to true, this behavior is deprecated "
      "as it does not comply with REP 145.");
  } else {
    // This complies with REP 145
    impl_->sensor_->SetWorldToReferenceOrientation(ignition::math::Quaterniond::Identity);
  }

  impl_->pub_ = impl_->ros_node_->create_publisher<sensor_msgs::msg::Imu>(
    "~/out", qos.get_publisher_qos("~/out", rclcpp::SensorDataQoS().reliable()));

  // Create message to be reused
  auto msg = std::make_shared<sensor_msgs::msg::Imu>();

  // Get frame for message
  msg->header.frame_id = gazebo_ros::SensorFrameID(*_sensor, *_sdf);

  // Fill covariances
  // TODO(anyone): covariance for IMU's orientation once this is added to gazebo
  using SNT = gazebo::sensors::SensorNoiseType;
  msg->angular_velocity_covariance[0] =
    gazebo_ros::NoiseVariance(impl_->sensor_->Noise(SNT::IMU_ANGVEL_X_NOISE_RADIANS_PER_S));
  msg->angular_velocity_covariance[4] =
    gazebo_ros::NoiseVariance(impl_->sensor_->Noise(SNT::IMU_ANGVEL_Y_NOISE_RADIANS_PER_S));
  msg->angular_velocity_covariance[8] =
    gazebo_ros::NoiseVariance(impl_->sensor_->Noise(SNT::IMU_ANGVEL_Z_NOISE_RADIANS_PER_S));
  msg->linear_acceleration_covariance[0] =
    gazebo_ros::NoiseVariance(impl_->sensor_->Noise(SNT::IMU_LINACC_X_NOISE_METERS_PER_S_SQR));
  msg->linear_acceleration_covariance[4] =
    gazebo_ros::NoiseVariance(impl_->sensor_->Noise(SNT::IMU_LINACC_Y_NOISE_METERS_PER_S_SQR));
  msg->linear_acceleration_covariance[8] =
    gazebo_ros::NoiseVariance(impl_->sensor_->Noise(SNT::IMU_LINACC_Z_NOISE_METERS_PER_S_SQR));

  impl_->msg_ = msg;

  impl_->sensor_update_event_ = impl_->sensor_->ConnectUpdated(
    std::bind(&GazeboRosImuSensorPrivate::OnUpdate, impl_.get()));
}

void GazeboRosImuSensorPrivate::OnUpdate()
{
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE("GazeboRosImuSensorPrivate::OnUpdate");
  IGN_PROFILE_BEGIN("fill ROS message");
#endif
  // Fill message with latest sensor data
  msg_->header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(
    sensor_->LastUpdateTime());
  msg_->orientation =
    gazebo_ros::Convert<geometry_msgs::msg::Quaternion>(sensor_->Orientation());
  msg_->angular_velocity = gazebo_ros::Convert<geometry_msgs::msg::Vector3>(
    sensor_->AngularVelocity());
  msg_->linear_acceleration = gazebo_ros::Convert<geometry_msgs::msg::Vector3>(
    sensor_->LinearAcceleration());
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_END();
  IGN_PROFILE_BEGIN("Publish");
#endif
  // Publish message
  pub_->publish(*msg_);
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_END();
#endif
}

GZ_REGISTER_SENSOR_PLUGIN(GazeboRosImuSensor)

}  // namespace gazebo_plugins
