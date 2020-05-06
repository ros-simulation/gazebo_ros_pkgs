// Copyright 2020 Open Source Robotics Foundation
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

#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>

#include <gazebo/transport/transport.hh>
#include <gazebo_plugins/gazebo_ros_wheel_slip.hpp>
#include <gazebo_ros/node.hpp>
#include <std_msgs/msg/bool.hpp>

#include <memory>
#include <string>

namespace gazebo_plugins
{
class GazeboRosWheelSlipPrivate
{
public:
  /// A pointer to the GazeboROS node.
  gazebo_ros::Node::SharedPtr ros_node_;
};

GazeboRosWheelSlip::GazeboRosWheelSlip()
: impl_(std::make_unique<GazeboRosWheelSlipPrivate>())
{
}

GazeboRosWheelSlip::~GazeboRosWheelSlip()
{
}

void GazeboRosWheelSlip::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  // Initialize ROS node
  impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);

  impl_->ros_node_->declare_parameter("slip_compliance_unitless_lateral", 0.0);
  impl_->ros_node_->declare_parameter("slip_compliance_unitless_longitudinal", 0.0);

  auto existing_callback = impl_->ros_node_->set_on_parameters_set_callback(nullptr);
  auto param_change_callback =
    [this, existing_callback](std::vector<rclcpp::Parameter> parameters) {
      auto result = rcl_interfaces::msg::SetParametersResult();
      if (nullptr != existing_callback) {
        result = existing_callback(parameters);
        if (!result.successful) {
          return result;
        }
      }

      result.successful = true;
      for (const auto & parameter : parameters) {
        std::string param_name = parameter.get_name();
        if (param_name == "slip_compliance_unitless_lateral") {
          RCLCPP_INFO(impl_->ros_node_->get_logger(),
            "New lateral slip compliance: %.3e", parameter.as_double());
          this->SetSlipComplianceLateral(parameter.as_double());
        } else if (param_name == "slip_compliance_unitless_longitudinal") {
          RCLCPP_INFO(impl_->ros_node_->get_logger(),
            "New longitudinal slip compliance: %.3e", parameter.as_double());
          this->SetSlipComplianceLongitudinal(parameter.as_double());
        } else {
          RCLCPP_WARN(impl_->ros_node_->get_logger(),
            "Unrecognized parameter name[%s]", param_name);
          result.successful = false;
        }
      }
      return result;
    };

  impl_->ros_node_->set_on_parameters_set_callback(param_change_callback);

  // Initialize the WheelSlipPlugin after the ros node so it doesn't overwrite
  // the sdf parameters.
  WheelSlipPlugin::Load(_model, _sdf);
}

GZ_REGISTER_MODEL_PLUGIN(GazeboRosWheelSlip)
}  // namespace gazebo_plugins
