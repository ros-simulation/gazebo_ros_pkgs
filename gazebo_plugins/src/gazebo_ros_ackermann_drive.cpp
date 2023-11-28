// Copyright 2019 Open Source Robotics Foundation, Inc.
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

#include <gazebo/common/Time.hh>
#include <gazebo/common/PID.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_plugins/gazebo_ros_ackermann_drive.hpp>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/conversions/geometry_msgs.hpp>
#include <gazebo_ros/node.hpp>
#include <geometry_msgs/msg/twist.hpp>
#ifdef IGN_PROFILER_ENABLE
#include <ignition/common/Profiler.hh>
#endif
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sdf/sdf.hh>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <string>
#include <vector>

namespace gazebo_plugins
{
class GazeboRosAckermannDrivePrivate
{
public:
  /// Indicates which joint
  enum
  {
    /// Front right wheel
    FRONT_RIGHT,

    /// Front left wheel
    FRONT_LEFT,

    /// Rear right wheel
    REAR_RIGHT,

    /// Rear left wheel
    REAR_LEFT,

    /// Right steering
    STEER_RIGHT,

    /// Left steering
    STEER_LEFT,

    /// Steering wheel
    STEER_WHEEL
  };

  /// Callback to be called at every simulation iteration.
  /// \param[in] _info Updated simulation info.
  void OnUpdate(const gazebo::common::UpdateInfo & _info);

  /// Callback when a velocity command is received.
  /// \param[in] _msg Twist command message.
  void OnCmdVel(geometry_msgs::msg::Twist::SharedPtr _msg);

  /// Extracts radius of a cylinder or sphere collision shape
  /// \param[in] _coll Pointer to collision
  /// \return If the collision shape is valid, return radius
  /// \return If the collision shape is invalid, return 0
  double CollisionRadius(const gazebo::physics::CollisionPtr & _coll);

  /// Update odometry according to world
  void UpdateOdometryWorld();

  /// Publish odometry transforms
  /// \param[in] _current_time Current simulation time
  void PublishOdometryTf(const gazebo::common::Time & _current_time);

  /// Publish trasforms for the wheels
  /// \param[in] _current_time Current simulation time
  void PublishWheelsTf(const gazebo::common::Time & _current_time);

  /// Publish odometry messages
  /// \param[in] _current_time Current simulation time
  void PublishOdometryMsg(const gazebo::common::Time & _current_time);

  /// A pointer to the GazeboROS node.
  gazebo_ros::Node::SharedPtr ros_node_;

  /// Subscriber to command velocities
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

  /// Odometry publisher
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;

  /// Distance publisher
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr distance_pub_;

  /// Steerangle publisher
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr steerangle_pub_;

  /// Connection to event called at every world iteration.
  gazebo::event::ConnectionPtr update_connection_;

  /// Pointers to wheel joints.
  std::vector<gazebo::physics::JointPtr> joints_;

  /// Pointer to model.
  gazebo::physics::ModelPtr model_;

  /// Distance between the wheels, in meters.
  double wheel_separation_;

  /// Distance between front and rear axles, in meters.
  double wheel_base_;

  /// Radius of rear wheels, in meters.
  double wheel_radius_;

  /// Angle ratio between the steering wheel and the front wheels
  double steering_ratio_ = 0;

  // Max steering angle
  double max_speed_ = 0;

  // Max steering angle of tyre
  double max_steer_ = 0;

  /// To broadcast TFs
  std::shared_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;

  /// Protect variables accessed on callbacks.
  std::mutex lock_;

  /// Linear velocity in X received on command (m/s).
  double target_linear_{0.0};

  /// Angular velocity in Z received on command (rad/s).
  double target_rot_{0.0};

  /// Update period in seconds.
  double update_period_;

  /// Last update time.
  gazebo::common::Time last_update_time_;

  /// Odometry frame ID
  std::string odometry_frame_;

  /// Keep latest odometry message
  nav_msgs::msg::Odometry odom_;

  /// Keep latest distance message
  std_msgs::msg::Float32 distance_;

  /// Keep latest steerangle message
  std_msgs::msg::Float32 steerangle_;

  /// Robot base frame ID
  std::string robot_base_frame_;

  /// True to publish odometry messages.
  bool publish_odom_;

  /// True to publish distance travelled
  bool publish_distance_;

  /// True to publish steering angle
  bool publish_steerangle_;

  /// True to publish wheel-to-base transforms.
  bool publish_wheel_tf_;

  /// True to publish odom-to-world transforms.
  bool publish_odom_tf_;

  /// Covariance in odometry
  double covariance_[3];

  /// PID control for left steering control
  gazebo::common::PID pid_left_steering_;

  /// PID control for right steering control
  gazebo::common::PID pid_right_steering_;

  /// PID control for linear velocity control
  gazebo::common::PID pid_linear_vel_;
};

GazeboRosAckermannDrive::GazeboRosAckermannDrive()
: impl_(std::make_unique<GazeboRosAckermannDrivePrivate>())
{
}

GazeboRosAckermannDrive::~GazeboRosAckermannDrive()
{
}

void GazeboRosAckermannDrive::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  impl_->model_ = _model;

  auto world = impl_->model_->GetWorld();
  auto physicsEngine = world->Physics();
  physicsEngine->SetParam("friction_model", std::string("cone_model"));

  // Initialize ROS node
  impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);

  // Get QoS profiles
  const gazebo_ros::QoS & qos = impl_->ros_node_->get_qos();

  impl_->joints_.resize(7);

  auto steering_wheel_joint =
    _sdf->Get<std::string>("steering_wheel_joint", "steering_wheel_joint").first;
  impl_->joints_[GazeboRosAckermannDrivePrivate::STEER_WHEEL] =
    _model->GetJoint(steering_wheel_joint);
  if (!impl_->joints_[GazeboRosAckermannDrivePrivate::STEER_WHEEL]) {
    RCLCPP_WARN(
      impl_->ros_node_->get_logger(),
      "Steering wheel joint [%s] not found.", steering_wheel_joint.c_str());
    impl_->joints_.resize(6);
  }

  auto front_right_joint = _sdf->Get<std::string>("front_right_joint", "front_right_joint").first;
  impl_->joints_[GazeboRosAckermannDrivePrivate::FRONT_RIGHT] = _model->GetJoint(front_right_joint);
  if (!impl_->joints_[GazeboRosAckermannDrivePrivate::FRONT_RIGHT]) {
    RCLCPP_ERROR(
      impl_->ros_node_->get_logger(),
      "Front right wheel joint [%s] not found, plugin will not work.", front_right_joint.c_str());
    impl_->ros_node_.reset();
    return;
  }

  auto front_left_joint = _sdf->Get<std::string>("front_left_joint", "front_left_joint").first;
  impl_->joints_[GazeboRosAckermannDrivePrivate::FRONT_LEFT] = _model->GetJoint(front_left_joint);
  if (!impl_->joints_[GazeboRosAckermannDrivePrivate::FRONT_LEFT]) {
    RCLCPP_ERROR(
      impl_->ros_node_->get_logger(),
      "Front left wheel joint [%s] not found, plugin will not work.", front_left_joint.c_str());
    impl_->ros_node_.reset();
    return;
  }

  auto rear_right_joint = _sdf->Get<std::string>("rear_right_joint", "rear_right_joint").first;
  impl_->joints_[GazeboRosAckermannDrivePrivate::REAR_RIGHT] = _model->GetJoint(rear_right_joint);
  if (!impl_->joints_[GazeboRosAckermannDrivePrivate::REAR_RIGHT]) {
    RCLCPP_ERROR(
      impl_->ros_node_->get_logger(),
      "Rear right wheel joint [%s] not found, plugin will not work.", rear_right_joint.c_str());
    impl_->ros_node_.reset();
    return;
  }

  auto rear_left_joint = _sdf->Get<std::string>("rear_left_joint", "rear_left_joint").first;
  impl_->joints_[GazeboRosAckermannDrivePrivate::REAR_LEFT] = _model->GetJoint(rear_left_joint);
  if (!impl_->joints_[GazeboRosAckermannDrivePrivate::REAR_LEFT]) {
    RCLCPP_ERROR(
      impl_->ros_node_->get_logger(),
      "Rear left wheel joint [%s] not found, plugin will not work.", rear_left_joint.c_str());
    impl_->ros_node_.reset();
    return;
  }

  auto right_steering_joint =
    _sdf->Get<std::string>("right_steering_joint", "right_steering_joint").first;
  impl_->joints_[GazeboRosAckermannDrivePrivate::STEER_RIGHT] =
    _model->GetJoint(right_steering_joint);
  if (!impl_->joints_[GazeboRosAckermannDrivePrivate::STEER_RIGHT]) {
    RCLCPP_ERROR(
      impl_->ros_node_->get_logger(),
      "Right wheel steering joint [%s] not found, plugin will not work.",
      right_steering_joint.c_str());
    impl_->ros_node_.reset();
    return;
  }

  auto left_steering_joint =
    _sdf->Get<std::string>("left_steering_joint", "left_steering_joint").first;
  impl_->joints_[GazeboRosAckermannDrivePrivate::STEER_LEFT] =
    _model->GetJoint(left_steering_joint);
  if (!impl_->joints_[GazeboRosAckermannDrivePrivate::STEER_LEFT]) {
    RCLCPP_ERROR(
      impl_->ros_node_->get_logger(),
      "Left wheel steering joint [%s] not found, plugin will not work.",
      left_steering_joint.c_str());
    impl_->ros_node_.reset();
    return;
  }

  impl_->max_speed_ = _sdf->Get<double>("max_speed", 20.0).first;
  impl_->max_steer_ = _sdf->Get<double>("max_steer", 0.6).first;

  // Max the steering wheel can rotate
  auto max_steering_angle = _sdf->Get<double>("max_steering_angle", 7.85).first;

  // Compute the angle ratio between the steering wheel and the tires
  impl_->steering_ratio_ = impl_->max_steer_ / max_steering_angle;

  auto pid = _sdf->Get<ignition::math::Vector3d>(
    "right_steering_pid_gain", ignition::math::Vector3d::Zero).first;
  auto i_range = _sdf->Get<ignition::math::Vector2d>(
    "right_steering_i_range", ignition::math::Vector2d::Zero).first;
  impl_->pid_right_steering_.Init(pid.X(), pid.Y(), pid.Z(), i_range.Y(), i_range.X());

  pid = _sdf->Get<ignition::math::Vector3d>(
    "left_steering_pid_gain", ignition::math::Vector3d::Zero).first;
  i_range = _sdf->Get<ignition::math::Vector2d>(
    "left_steering_i_range", ignition::math::Vector2d::Zero).first;
  impl_->pid_left_steering_.Init(pid.X(), pid.Y(), pid.Z(), i_range.Y(), i_range.X());

  pid = _sdf->Get<ignition::math::Vector3d>(
    "linear_velocity_pid_gain", ignition::math::Vector3d::Zero).first;
  i_range = _sdf->Get<ignition::math::Vector2d>(
    "linear_velocity_i_range", ignition::math::Vector2d::Zero).first;
  impl_->pid_linear_vel_.Init(pid.X(), pid.Y(), pid.Z(), i_range.Y(), i_range.X());

  // Update wheel radiu for wheel from SDF collision objects
  // assumes that wheel link is child of joint (and not parent of joint)
  // assumes that wheel link has only one collision
  // assumes all wheel of both rear wheels of same radii
  unsigned int id = 0;
  impl_->wheel_radius_ = impl_->CollisionRadius(
    impl_->joints_[GazeboRosAckermannDrivePrivate::REAR_RIGHT]->GetChild()->GetCollision(id));

  // Compute wheel_base, front wheel separation, and rear wheel separation
  // first compute the positions of the 4 wheel centers
  // again assumes wheel link is child of joint and has only one collision
  auto front_right_center_pos = impl_->joints_[GazeboRosAckermannDrivePrivate::FRONT_RIGHT]->
    GetChild()->GetCollision(id)->WorldPose().Pos();
  auto front_left_center_pos = impl_->joints_[GazeboRosAckermannDrivePrivate::FRONT_LEFT]->
    GetChild()->GetCollision(id)->WorldPose().Pos();
  auto rear_right_center_pos = impl_->joints_[GazeboRosAckermannDrivePrivate::REAR_RIGHT]->
    GetChild()->GetCollision(id)->WorldPose().Pos();
  auto rear_left_center_pos = impl_->joints_[GazeboRosAckermannDrivePrivate::REAR_LEFT]->
    GetChild()->GetCollision(id)->WorldPose().Pos();

  auto distance = front_left_center_pos - front_right_center_pos;
  impl_->wheel_separation_ = distance.Length();

  // to compute wheelbase, first position of axle centers are computed
  auto front_axle_pos = (front_left_center_pos + front_right_center_pos) / 2;
  auto rear_axle_pos = (rear_left_center_pos + rear_right_center_pos) / 2;
  // then the wheelbase is the distance between the axle centers
  distance = front_axle_pos - rear_axle_pos;
  impl_->wheel_base_ = distance.Length();

  // Update rate
  auto update_rate = _sdf->Get<double>("update_rate", 100.0).first;
  if (update_rate > 0.0) {
    impl_->update_period_ = 1.0 / update_rate;
  } else {
    impl_->update_period_ = 0.0;
  }
  impl_->last_update_time_ = _model->GetWorld()->SimTime();

  impl_->cmd_vel_sub_ = impl_->ros_node_->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", qos.get_subscription_qos("cmd_vel", rclcpp::QoS(1)),
    std::bind(&GazeboRosAckermannDrivePrivate::OnCmdVel, impl_.get(), std::placeholders::_1));

  RCLCPP_INFO(
    impl_->ros_node_->get_logger(), "Subscribed to [%s]", impl_->cmd_vel_sub_->get_topic_name());

  // Odometry
  impl_->odometry_frame_ = _sdf->Get<std::string>("odometry_frame", "odom").first;
  impl_->robot_base_frame_ = _sdf->Get<std::string>("robot_base_frame", "base_footprint").first;

  // Advertise odometry topic
  impl_->publish_odom_ = _sdf->Get<bool>("publish_odom", false).first;
  if (impl_->publish_odom_) {
    impl_->odometry_pub_ = impl_->ros_node_->create_publisher<nav_msgs::msg::Odometry>(
      "odom", qos.get_publisher_qos("odom", rclcpp::QoS(1)));

    RCLCPP_INFO(
      impl_->ros_node_->get_logger(), "Advertise odometry on [%s]",
      impl_->odometry_pub_->get_topic_name());
  }

  // Advertise distance travelled
  impl_->publish_distance_ = _sdf->Get<bool>("publish_distance", false).first;
  if (impl_->publish_distance_) {
    impl_->distance_pub_ = impl_->ros_node_->create_publisher<std_msgs::msg::Float32>(
      "distance", qos.get_publisher_qos("distance", rclcpp::QoS(1)));

    RCLCPP_INFO(
      impl_->ros_node_->get_logger(), "Advertise distance on [%s]",
      impl_->distance_pub_->get_topic_name());
  }

  // Advertise steering angle
  impl_->publish_steerangle_ = _sdf->Get<bool>("publish_steerangle", false).first;
  if (impl_->publish_steerangle_) {
    impl_->steerangle_pub_ = impl_->ros_node_->create_publisher<std_msgs::msg::Float32>(
      "steerangle", qos.get_publisher_qos("steerangle", rclcpp::QoS(1)));

    RCLCPP_INFO(
      impl_->ros_node_->get_logger(), "Advertise steerangle on [%s]",
      impl_->steerangle_pub_->get_topic_name());
  }

  // Create TF broadcaster if needed
  impl_->publish_wheel_tf_ = _sdf->Get<bool>("publish_wheel_tf", false).first;
  impl_->publish_odom_tf_ = _sdf->Get<bool>("publish_odom_tf", false).first;
  if (impl_->publish_wheel_tf_ || impl_->publish_odom_tf_) {
    impl_->transform_broadcaster_ =
      std::make_shared<tf2_ros::TransformBroadcaster>(impl_->ros_node_);

    if (impl_->publish_odom_tf_) {
      RCLCPP_INFO(
        impl_->ros_node_->get_logger(),
        "Publishing odom transforms between [%s] and [%s]", impl_->odometry_frame_.c_str(),
        impl_->robot_base_frame_.c_str());
    }

    if (impl_->publish_wheel_tf_) {
      for (auto & joint : impl_->joints_) {
        RCLCPP_INFO(
          impl_->ros_node_->get_logger(),
          "Publishing wheel transforms between [%s], [%s] and [%s]",
          impl_->robot_base_frame_.c_str(), joint->GetName().c_str(), joint->GetName().c_str());
      }
    }
  }

  auto pose = impl_->model_->WorldPose();
  impl_->odom_.pose.pose.position = gazebo_ros::Convert<geometry_msgs::msg::Point>(pose.Pos());
  impl_->odom_.pose.pose.orientation = gazebo_ros::Convert<geometry_msgs::msg::Quaternion>(
    pose.Rot());

  impl_->covariance_[0] = _sdf->Get<double>("covariance_x", 0.00001).first;
  impl_->covariance_[1] = _sdf->Get<double>("covariance_y", 0.00001).first;
  impl_->covariance_[2] = _sdf->Get<double>("covariance_yaw", 0.001).first;

  // Listen to the update event (broadcast every simulation iteration)
  impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&GazeboRosAckermannDrivePrivate::OnUpdate, impl_.get(), std::placeholders::_1));
}

void GazeboRosAckermannDrive::Reset()
{
  impl_->last_update_time_ = impl_->model_->GetWorld()->SimTime();

  impl_->target_linear_ = 0;
  impl_->target_rot_ = 0;
  impl_->distance_.data = 0;
  impl_->steerangle_.data = 0;
}

void GazeboRosAckermannDrivePrivate::OnUpdate(const gazebo::common::UpdateInfo & _info)
{
  #ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE("GazeboRosAckermannDrivePrivate::OnUpdate");
  #endif
  std::lock_guard<std::mutex> lock(lock_);

  double seconds_since_last_update = (_info.simTime - last_update_time_).Double();

#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_BEGIN("UpdateOdometryWorld");
#endif
  // Update odom
  UpdateOdometryWorld();
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_END();
#endif
  if (seconds_since_last_update < update_period_) {
    return;
  }

  if (publish_distance_) {
#ifdef IGN_PROFILER_ENABLE
    IGN_PROFILE_BEGIN("publish distance");
#endif
    distance_pub_->publish(distance_);
#ifdef IGN_PROFILER_ENABLE
    IGN_PROFILE_END();
#endif
  }

  if (publish_odom_) {
#ifdef IGN_PROFILER_ENABLE
    IGN_PROFILE_BEGIN("PublishOdometryMsg");
#endif
    PublishOdometryMsg(_info.simTime);
#ifdef IGN_PROFILER_ENABLE
    IGN_PROFILE_END();
#endif
  }

  if (publish_wheel_tf_) {
#ifdef IGN_PROFILER_ENABLE
    IGN_PROFILE_BEGIN("PublishWheelsTf");
#endif
    PublishWheelsTf(_info.simTime);
#ifdef IGN_PROFILER_ENABLE
    IGN_PROFILE_END();
#endif
  }

  if (publish_odom_tf_) {
#ifdef IGN_PROFILER_ENABLE
    IGN_PROFILE_BEGIN("PublishOdometryTf");
#endif
    PublishOdometryTf(_info.simTime);
#ifdef IGN_PROFILER_ENABLE
    IGN_PROFILE_END();
#endif
  }

#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_BEGIN("update");
#endif
  // Current speed assuming equal for left rear and right rear
  auto linear_vel = joints_[REAR_RIGHT]->GetVelocity(0);
  auto target_linear = ignition::math::clamp(target_linear_, -max_speed_, max_speed_);
  double linear_diff = linear_vel - target_linear / wheel_radius_;
  double linear_cmd = pid_linear_vel_.Update(linear_diff, seconds_since_last_update);

  auto target_rot = target_rot_ * copysign(1.0, target_linear_);
  target_rot = ignition::math::clamp(target_rot, -max_steer_, max_steer_);

  double tanSteer = tan(target_rot);

  auto target_left_steering =
    atan2(tanSteer, 1.0 - wheel_separation_ / 2.0 / wheel_base_ * tanSteer);
  auto target_right_steering =
    atan2(tanSteer, 1.0 + wheel_separation_ / 2.0 / wheel_base_ * tanSteer);

  auto left_steering_angle = joints_[STEER_LEFT]->Position(0);
  auto right_steering_angle = joints_[STEER_RIGHT]->Position(0);

  double left_steering_diff = left_steering_angle - target_left_steering;
  double left_steering_cmd =
    pid_left_steering_.Update(left_steering_diff, seconds_since_last_update);

  double right_steering_diff = right_steering_angle - target_right_steering;
  double right_steering_cmd =
    pid_right_steering_.Update(right_steering_diff, seconds_since_last_update);

  auto steer_wheel_angle = (left_steering_angle + right_steering_angle) * 0.5 / steering_ratio_;
  steerangle_.data = (left_steering_angle + right_steering_angle) * 0.5;

  if (publish_steerangle_) {
#ifdef IGN_PROFILER_ENABLE
    IGN_PROFILE_BEGIN("publish steerangle");
#endif
    steerangle_pub_->publish(steerangle_);
#ifdef IGN_PROFILER_ENABLE
    IGN_PROFILE_END();
#endif
  }

  joints_[STEER_LEFT]->SetForce(0, left_steering_cmd);
  joints_[STEER_RIGHT]->SetForce(0, right_steering_cmd);
  joints_[REAR_RIGHT]->SetForce(0, linear_cmd);
  joints_[REAR_LEFT]->SetForce(0, linear_cmd);

  if (joints_.size() == 7) {
    joints_[STEER_WHEEL]->SetPosition(0, steer_wheel_angle);
  }

  last_update_time_ = _info.simTime;
  #ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_END();
  #endif
}

void GazeboRosAckermannDrivePrivate::OnCmdVel(const geometry_msgs::msg::Twist::SharedPtr _msg)
{
  std::lock_guard<std::mutex> scoped_lock(lock_);
  target_linear_ = _msg->linear.x;
  target_rot_ = _msg->angular.z;
}

double GazeboRosAckermannDrivePrivate::CollisionRadius(const gazebo::physics::CollisionPtr & _coll)
{
  if (!_coll || !(_coll->GetShape())) {
    return 0;
  }
  if (_coll->GetShape()->HasType(gazebo::physics::Base::CYLINDER_SHAPE)) {
    gazebo::physics::CylinderShape * cyl =
      dynamic_cast<gazebo::physics::CylinderShape *>(_coll->GetShape().get());
    return cyl->GetRadius();
  } else if (_coll->GetShape()->HasType(gazebo::physics::Base::SPHERE_SHAPE)) {
    gazebo::physics::SphereShape * sph =
      dynamic_cast<gazebo::physics::SphereShape *>(_coll->GetShape().get());
    return sph->GetRadius();
  }
  return 0;
}

void GazeboRosAckermannDrivePrivate::UpdateOdometryWorld()
{
  auto prev_x = odom_.pose.pose.position.x;
  auto prev_y = odom_.pose.pose.position.y;

  auto pose = model_->WorldPose();
  odom_.pose.pose.position = gazebo_ros::Convert<geometry_msgs::msg::Point>(pose.Pos());
  odom_.pose.pose.orientation = gazebo_ros::Convert<geometry_msgs::msg::Quaternion>(pose.Rot());

  distance_.data += hypot(prev_x - odom_.pose.pose.position.x, prev_y - odom_.pose.pose.position.y);

  // Get velocity in odom frame
  auto linear = model_->WorldLinearVel();
  odom_.twist.twist.angular.z = model_->WorldAngularVel().Z();

  // Convert velocity to child_frame_id(aka base_footprint)
  auto yaw = static_cast<float>(pose.Rot().Yaw());
  odom_.twist.twist.linear.x = cosf(yaw) * linear.X() + sinf(yaw) * linear.Y();
  odom_.twist.twist.linear.y = cosf(yaw) * linear.Y() - sinf(yaw) * linear.X();
}

void GazeboRosAckermannDrivePrivate::PublishOdometryTf(const gazebo::common::Time & _current_time)
{
  geometry_msgs::msg::TransformStamped msg;
  msg.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(_current_time);
  msg.header.frame_id = odometry_frame_;
  msg.child_frame_id = robot_base_frame_;
  msg.transform.translation =
    gazebo_ros::Convert<geometry_msgs::msg::Vector3>(odom_.pose.pose.position);
  msg.transform.rotation = odom_.pose.pose.orientation;

  transform_broadcaster_->sendTransform(msg);
}

void GazeboRosAckermannDrivePrivate::PublishWheelsTf(const gazebo::common::Time & _current_time)
{
  for (const auto & joint : joints_) {
    auto pose = joint->GetChild()->WorldPose() - model_->WorldPose();

    geometry_msgs::msg::TransformStamped msg;
    msg.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(_current_time);
    msg.header.frame_id = robot_base_frame_;
    msg.child_frame_id = joint->GetChild()->GetName();
    msg.transform.translation = gazebo_ros::Convert<geometry_msgs::msg::Vector3>(pose.Pos());
    msg.transform.rotation = gazebo_ros::Convert<geometry_msgs::msg::Quaternion>(pose.Rot());

    transform_broadcaster_->sendTransform(msg);
  }
}

void GazeboRosAckermannDrivePrivate::PublishOdometryMsg(const gazebo::common::Time & _current_time)
{
  // Set covariance
  odom_.pose.covariance[0] = covariance_[0];
  odom_.pose.covariance[7] = covariance_[1];
  odom_.pose.covariance[14] = 1000000000000.0;
  odom_.pose.covariance[21] = 1000000000000.0;
  odom_.pose.covariance[28] = 1000000000000.0;
  odom_.pose.covariance[35] = covariance_[2];

  odom_.twist.covariance[0] = covariance_[0];
  odom_.twist.covariance[7] = covariance_[1];
  odom_.twist.covariance[14] = 1000000000000.0;
  odom_.twist.covariance[21] = 1000000000000.0;
  odom_.twist.covariance[28] = 1000000000000.0;
  odom_.twist.covariance[35] = covariance_[2];

  // Set header
  odom_.header.frame_id = odometry_frame_;
  odom_.child_frame_id = robot_base_frame_;
  odom_.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(_current_time);

  // Publish
  odometry_pub_->publish(odom_);
}
GZ_REGISTER_MODEL_PLUGIN(GazeboRosAckermannDrive)
}  // namespace gazebo_plugins
