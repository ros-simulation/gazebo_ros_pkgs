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

#include <algorithm>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace gazebo_plugins
{
class GazeboRosWheelSlipPrivate
{
public:
  /// A pointer to the GazeboROS node.
  gazebo_ros::Node::SharedPtr ros_node_;

  /// Handle to parameters callback
  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_handle_;

  // Containers to hold default values of slip parameters
  std::map<std::string, double> mapSlipLateralDefault;
  double default_slip_lateral;
  std::map<std::string, double> mapSlipLongitudinalDefault;
  double default_slip_longitudinal;
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
  // Initialize the WheelSlipPlugin first so its values are preferred unless the ros
  // parameters are overridden by a launch file.
  WheelSlipPlugin::Load(_model, _sdf);

  // Read slip values from the sdf file and set them as default
  if (_sdf->HasElement("wheel")) {
    for (auto wheel_element = _sdf->GetElement("wheel"); wheel_element;
      wheel_element = wheel_element->GetNextElement("wheel"))
    {
      auto wheel_name = wheel_element->Get<std::string>("link_name");
      double slip_lateral = wheel_element->Get<double>("slip_compliance_lateral");
      double default_slip_lateral = std::max(slip_lateral, 0.);
      this->impl_->mapSlipLateralDefault[wheel_name] = default_slip_lateral;
      this->impl_->default_slip_lateral = default_slip_lateral;

      double slip_longitudinal = wheel_element->Get<double>("slip_compliance_longitudinal");
      double default_slip_longitudinal = std::max(slip_longitudinal, 0.);
      this->impl_->mapSlipLongitudinalDefault[wheel_name] = default_slip_longitudinal;
      this->impl_->default_slip_longitudinal = default_slip_longitudinal;
    }
  }

  // Initialize ROS node
  impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);

  auto param_change_callback =
    [this](std::vector<rclcpp::Parameter> parameters) {
      auto result = rcl_interfaces::msg::SetParametersResult();
      result.successful = true;

      for (const auto & parameter : parameters) {
        std::string param_name = parameter.get_name();

        if (param_name.find("slip_compliance_unitless_lateral/") != std::string::npos) {
          auto wheel_name = param_name.substr(param_name.find("/") + 1);
          if (impl_->mapSlipLateralDefault.count(wheel_name) == 1) {
            // The parameter has a valid name
            double slip = parameter.as_double();
            if (slip >= 0.) {
              RCLCPP_INFO(
                impl_->ros_node_->get_logger(),
                "New lateral slip compliance for %s: %.3e", wheel_name.c_str(), slip);
              this->SetSlipComplianceLateral(wheel_name, slip);
            } else {
              result.successful = false;
              result.reason = "Slip compliance values cannot be negative";
            }
          }
        }

        if (param_name.find("slip_compliance_unitless_longitudinal/") != std::string::npos) {
          auto wheel_name = param_name.substr(param_name.find("/") + 1);
          if (impl_->mapSlipLongitudinalDefault.count(wheel_name) == 1) {
            // The parameter has a valid name
            double slip = parameter.as_double();
            if (slip >= 0.) {
              RCLCPP_INFO(
                impl_->ros_node_->get_logger(),
                "New longitudinal slip compliance for %s: %.3e", wheel_name.c_str(), slip);
              this->SetSlipComplianceLongitudinal(wheel_name, slip);
            } else {
              result.successful = false;
              result.reason = "Slip compliance values cannot be negative";
            }
          }
        }

        if (param_name == "slip_compliance_unitless_lateral") {
          double slip = parameter.as_double();
          if (slip >= 0.) {
            RCLCPP_INFO(
              impl_->ros_node_->get_logger(),
              "New lateral slip compliance for all wheels: %.3e", slip);
            this->SetSlipComplianceLateral(slip);
          } else {
            result.successful = false;
            result.reason = "Slip compliance values cannot be negative";
          }
        }

        if (param_name == "slip_compliance_unitless_longitudinal") {
          double slip = parameter.as_double();
          if (slip >= 0.) {
            RCLCPP_INFO(
              impl_->ros_node_->get_logger(),
              "New longitudinal slip compliance for all wheels: %.3e", slip);
            this->SetSlipComplianceLongitudinal(slip);
          } else {
            result.successful = false;
            result.reason = "Slip compliance values cannot be negative";
          }
        }
      }
      return result;
    };

  impl_->on_set_parameters_callback_handle_ = impl_->ros_node_->add_on_set_parameters_callback(
    param_change_callback);

  // Declare parameters after adding callback so that callback will trigger immediately.
  // Set negative values by default, which are ignored by the callback.
  // This approach allows values specified in a launch file to override the SDF/URDF values.
  impl_->ros_node_->declare_parameter(
    "slip_compliance_unitless_lateral",
    impl_->default_slip_lateral);
  impl_->ros_node_->declare_parameter(
    "slip_compliance_unitless_longitudinal",
    impl_->default_slip_longitudinal);

  for (auto & wheel_parameter : impl_->mapSlipLateralDefault) {
    impl_->ros_node_->declare_parameter(
      "slip_compliance_unitless_lateral/" + wheel_parameter.first,
      wheel_parameter.second
    );
  }

  for (auto & wheel_parameter : impl_->mapSlipLongitudinalDefault) {
    impl_->ros_node_->declare_parameter(
      "slip_compliance_unitless_longitudinal/" + wheel_parameter.first,
      wheel_parameter.second
    );
  }
}

GZ_REGISTER_MODEL_PLUGIN(GazeboRosWheelSlip)
}  // namespace gazebo_plugins
