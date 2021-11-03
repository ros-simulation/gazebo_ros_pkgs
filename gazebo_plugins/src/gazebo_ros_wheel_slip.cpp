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

#include <unordered_map>
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
  std::unordered_map<std::string, double> map_slip_lateral_default_;
  double default_slip_lateral_;
  std::unordered_map<std::string, double> map_slip_longitudinal_default_;
  double default_slip_longitudinal_;

  // Event handler to set slip compliance values for individual wheels based
  std::shared_ptr<rclcpp::ParameterEventHandler> parameter_event_handler_;
  rclcpp::ParameterEventCallbackHandle::SharedPtr parameter_set_event_callback_;
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
  // Initialize ROS node
  impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);

  // Read slip values from the sdf file and set them as default
  if (_sdf->HasElement("wheel")) {
    for (auto wheel_element = _sdf->GetElement("wheel"); wheel_element;
      wheel_element = wheel_element->GetNextElement("wheel"))
    {
      auto wheel_name = wheel_element->Get<std::string>("link_name");
      double slip_lateral = wheel_element->Get<double>("slip_compliance_lateral");

      if (slip_lateral >= 0.) {
        this->impl_->default_slip_lateral_ = slip_lateral;
        this->impl_->map_slip_lateral_default_[wheel_name] = slip_lateral;
      } else {
        this->impl_->default_slip_lateral_ = 0.0;
        this->impl_->map_slip_lateral_default_[wheel_name] = 0.0;
        auto msg = "Negative slip lateral compliance value found in sdf for " + wheel_name +
          " will be set to 0.0";
        RCLCPP_INFO(
          this->impl_->ros_node_->get_logger(),
          msg.c_str()
        );
      }

      double slip_longitudinal = wheel_element->Get<double>("slip_compliance_longitudinal");

      if (slip_longitudinal >= 0.) {
        this->impl_->default_slip_longitudinal_ = slip_longitudinal;
        this->impl_->map_slip_longitudinal_default_[wheel_name] = slip_longitudinal;
      } else {
        this->impl_->default_slip_longitudinal_ = 0.0;
        this->impl_->map_slip_longitudinal_default_[wheel_name] = 0.0;
        auto msg = "Negative slip longitudinal compliance value found in sdf for " + wheel_name +
          " will be set to 0.0";
        RCLCPP_INFO(
          this->impl_->ros_node_->get_logger(),
          msg.c_str()
        );
      }
    }
  }

  auto param_validation_callback =
    [this](std::vector<rclcpp::Parameter> parameters) {
      auto result = rcl_interfaces::msg::SetParametersResult();
      result.successful = true;

      for (const auto & parameter : parameters) {
        std::string param_name = parameter.get_name();

        if (param_name.find("slip_compliance") != std::string::npos) {
          double slip = parameter.as_double();
          if (slip < 0.) {
            result.successful = false;
            result.reason = "Slip compliance values cannot be negative";
          }
        }
      }
      return result;
    };

  impl_->on_set_parameters_callback_handle_ = impl_->ros_node_->add_on_set_parameters_callback(
    param_validation_callback);

  // Add hook from global slip compliance parameter to the individual wheels
  auto parameter_set_callback_function = [this](const rcl_interfaces::msg::ParameterEvent & event) {
      // Only care about this node
      std::string node_namespace = this->impl_->ros_node_->get_namespace();
      std::string node_name = this->impl_->ros_node_->get_name();
      auto complete_node_name = node_namespace + "/" + node_name;
      if (event.node == complete_node_name) {
        // Combine the new and changed parameter vectors
        auto param_list = event.new_parameters;
        param_list.insert(
          param_list.end(), event.changed_parameters.begin(),
          event.changed_parameters.end());

        // Iterate over parameters
        for (auto & parameter : param_list) {
          auto temp_param = this->impl_->ros_node_->get_parameter(parameter.name);

          // Propagate global slip compliance lateral parameter to all wheels
          // This does not set the actual parameter in gazebo
          if (parameter.name == "slip_compliance_unitless_lateral") {
            double slip = temp_param.as_double();
            if (slip >= 0.) {
              RCLCPP_INFO(
                this->impl_->ros_node_->get_logger(),
                "Setting lateral slip compliance for all wheels: %.3e", slip);
              // Iterate over all wheels
              for (auto & wheel_parameter : this->impl_->map_slip_lateral_default_) {
                auto param_name = parameter.name + "/" + wheel_parameter.first;
                this->impl_->ros_node_->set_parameter(rclcpp::Parameter(param_name, slip));
              }
            }
          }

          // Propagate global slip compliance longitudinal parameter to all wheels
          // This does not set the actual parameter in gazebo
          if (parameter.name == "slip_compliance_unitless_longitudinal") {
            double slip = temp_param.as_double();
            if (slip >= 0.) {
              RCLCPP_INFO(
                this->impl_->ros_node_->get_logger(),
                "Setting longitudinal slip compliance for all wheels: %.3e", slip);
              // Iterate over all wheels
              for (auto & wheel_parameter : this->impl_->map_slip_longitudinal_default_) {
                auto param_name = parameter.name + "/" + wheel_parameter.first;
                this->impl_->ros_node_->set_parameter(rclcpp::Parameter(param_name, slip));
              }
            }
          }

          // Set lateral slip compliance for individual wheel
          // Must actually set the parameter in gazebo
          if (parameter.name.find("slip_compliance_unitless_lateral/") != std::string::npos) {
            auto wheel_name = parameter.name.substr(parameter.name.find("/") + 1);
            if (this->impl_->map_slip_lateral_default_.count(wheel_name) == 1) {
              // The parameter has a valid name
              double slip = temp_param.as_double();
              if (slip >= 0.) {
                RCLCPP_INFO(
                  this->impl_->ros_node_->get_logger(),
                  "New lateral slip compliance for %s: %.3e", wheel_name.c_str(), slip);
                this->SetSlipComplianceLateral(wheel_name, slip);
              }
            }
          }

          // Set longitudinal slip compliance for individual wheel
          // Must actually set the parameter in gazebo
          if (parameter.name.find("slip_compliance_unitless_longitudinal/") != std::string::npos) {
            auto wheel_name = parameter.name.substr(parameter.name.find("/") + 1);
            if (this->impl_->map_slip_longitudinal_default_.count(wheel_name) == 1) {
              // The parameter has a valid name
              double slip = temp_param.as_double();
              if (slip >= 0.) {
                RCLCPP_INFO(
                  this->impl_->ros_node_->get_logger(),
                  "New longitudinal slip compliance for %s: %.3e", wheel_name.c_str(), slip);
                this->SetSlipComplianceLongitudinal(wheel_name, slip);
              }
            }
          }
        }
        // Iteration over parameters done
      }
    };

  // Declare parameters after adding callback so that callback will trigger immediately.
  // Default values are taken from the sdf.
  impl_->ros_node_->declare_parameter(
    "slip_compliance_unitless_lateral",
    impl_->default_slip_lateral_);
  impl_->ros_node_->declare_parameter(
    "slip_compliance_unitless_longitudinal",
    impl_->default_slip_longitudinal_);


  // Global slip_parameters are declared before this callback is set, as we don't want the
  // individual wheel slip params to be set which haven't been declared yet
  impl_->parameter_event_handler_ =
    std::make_shared<rclcpp::ParameterEventHandler>(impl_->ros_node_);
  impl_->parameter_set_event_callback_ = impl_->parameter_event_handler_->
    add_parameter_event_callback(parameter_set_callback_function);

  for (auto & wheel_parameter : impl_->map_slip_lateral_default_) {
    impl_->ros_node_->declare_parameter(
      "slip_compliance_unitless_lateral/" + wheel_parameter.first,
      wheel_parameter.second
    );
  }

  for (auto & wheel_parameter : impl_->map_slip_longitudinal_default_) {
    impl_->ros_node_->declare_parameter(
      "slip_compliance_unitless_longitudinal/" + wheel_parameter.first,
      wheel_parameter.second
    );
  }

  // If the world file contains global wheelslip ros parameters, these should be set now
  if (_sdf->HasElement("ros")) {
    auto ros_element = _sdf->GetElement("ros");
    if (ros_element->HasElement("parameter")) {
      // Iterate over parameter tags
      for (auto param_element = ros_element->GetElement("parameter"); param_element;
        param_element = param_element->GetNextElement("parameter"))
      {
        auto param_name = param_element->Get<std::string>("name");
        if (param_name == "slip_compliance_unitless_lateral" ||
          param_name == "slip_compliance_unitless_longitudinal")
        {
          // Global wheelslip ros parameters were declared
          double value = param_element->Get<double>();
          this->impl_->ros_node_->set_parameter(rclcpp::Parameter(param_name, value));
          RCLCPP_INFO(
            this->impl_->ros_node_->get_logger(),
            "Global wheelslip parameter found in world file, overriding sdf parameters..");
        }
      }
    }
  }
}

GZ_REGISTER_MODEL_PLUGIN(GazeboRosWheelSlip)
}  // namespace gazebo_plugins
