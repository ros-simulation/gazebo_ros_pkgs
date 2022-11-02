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
#include <gazebo_msgs/msg/wheel_slip.hpp>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#if (GAZEBO_MAJOR_VERSION == 11 && GAZEBO_MINOR_VERSION >= 11)
#define GAZEBO_WHEELSLIP_HAS_FRICTION
#endif

using namespace std::chrono_literals;

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
  std::unordered_map<std::string, double> map_slip_longitudinal_default_;

#ifdef GAZEBO_WHEELSLIP_HAS_FRICTION
  // Containers to hold default values of friction coefficients
  std::unordered_map<std::string, double> map_friction_primary_default_;
  std::unordered_map<std::string, double> map_friction_secondary_default_;
#endif

  // Message to publish. Store as member variable to avoid problems during
  // teardown.
  gazebo_msgs::msg::WheelSlip slip_msg;

  // Event handler to set slip compliance values for individual wheels based
  std::shared_ptr<rclcpp::ParameterEventHandler> parameter_event_handler_;
  rclcpp::ParameterEventCallbackHandle::SharedPtr parameter_set_event_callback_;
  // Publish wheel slip
  rclcpp::Publisher<gazebo_msgs::msg::WheelSlip>::SharedPtr slip_publisher_;

  /// Period in seconds
  double publisher_update_period_;

  /// Keep last time an update was published
  gazebo::common::Time publisher_last_update_time_;

  /// Pointer to the update event connection.
  gazebo::event::ConnectionPtr publisher_update_connection_;

  /// Validate wheel slip value.
  double validate_wheel_slip_value(
    const double & value, std::string slip_type, std::string wheel_name) const
  {
    if (value < 0.0) {
      RCLCPP_INFO(
        ros_node_->get_logger(),
        "Negative slip %s compliance value found in sdf for %s will be set to 0.0",
        slip_type.c_str(),
        wheel_name.c_str()
      );
      return 0.0;
    }
    return value;
  }
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

      const double slip_lateral = this->impl_->validate_wheel_slip_value(
        wheel_element->Get<double>("slip_compliance_lateral"),
        "lateral",
        wheel_name);
      this->impl_->map_slip_lateral_default_[wheel_name] = slip_lateral;

      const double slip_longitudinal = this->impl_->validate_wheel_slip_value(
        wheel_element->Get<double>("slip_compliance_longitudinal"),
        "longitudinal",
        wheel_name);
      this->impl_->map_slip_longitudinal_default_[wheel_name] = slip_longitudinal;
    }
  }

  // Check for "global" slip parameters and use them for individual wheels if provided
  impl_->ros_node_->declare_parameter(
    "slip_compliance_unitless_lateral", rclcpp::PARAMETER_DOUBLE);
  impl_->ros_node_->declare_parameter(
    "slip_compliance_unitless_longitudinal", rclcpp::PARAMETER_DOUBLE);

  double param_value;
  if (impl_->ros_node_->get_parameter("slip_compliance_unitless_lateral", param_value)) {
    for (auto & wheel_parameter : impl_->map_slip_lateral_default_) {
      wheel_parameter.second = param_value;
    }
  }
  if (impl_->ros_node_->get_parameter("slip_compliance_unitless_longitudinal", param_value)) {
    for (auto & wheel_parameter : impl_->map_slip_longitudinal_default_) {
      wheel_parameter.second = param_value;
    }
  }

  for (auto & wheel_parameter : impl_->map_slip_lateral_default_) {
    double value = impl_->ros_node_->declare_parameter(
      "slip_compliance_unitless_lateral/" + wheel_parameter.first,
      wheel_parameter.second
    );
    this->SetSlipComplianceLateral(wheel_parameter.first, value);
  }

  for (auto & wheel_parameter : impl_->map_slip_longitudinal_default_) {
    double value = impl_->ros_node_->declare_parameter(
      "slip_compliance_unitless_longitudinal/" + wheel_parameter.first,
      wheel_parameter.second
    );
    this->SetSlipComplianceLongitudinal(wheel_parameter.first, value);
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

#ifdef GAZEBO_WHEELSLIP_HAS_FRICTION
        if (param_name.find("friction_coefficient") != std::string::npos) {
          double friction = parameter.as_double();
          if (friction < 0.) {
            result.successful = false;
            result.reason = "Friction coefficient values cannot be negative";
          }
        }
#endif
      }

      return result;
    };

#ifdef GAZEBO_WHEELSLIP_HAS_FRICTION
  // Read friction coefficient values from model and set friction parameters
  auto frictionCoeffs = WheelSlipPlugin::GetFrictionCoefficients();
  for (const auto & friction_coef : frictionCoeffs) {
    impl_->map_friction_primary_default_[friction_coef.first] = friction_coef.second.X();
    impl_->ros_node_->declare_parameter(
      "friction_coefficient_primary/" + friction_coef.first,
      impl_->map_friction_primary_default_[friction_coef.first]
    );
    impl_->map_friction_secondary_default_[friction_coef.first] = friction_coef.second.Y();
    impl_->ros_node_->declare_parameter(
      "friction_coefficient_secondary/" + friction_coef.first,
      impl_->map_friction_secondary_default_[friction_coef.first]
    );
  }
#endif

  impl_->on_set_parameters_callback_handle_ = impl_->ros_node_->add_on_set_parameters_callback(
    param_validation_callback);

  // Add hook from global slip compliance parameter to the individual wheels
  auto parameter_set_callback_function = [this](const rcl_interfaces::msg::ParameterEvent & event) {
      // Only care about this node
      std::string node_namespace = this->impl_->ros_node_->get_namespace();
      std::string node_name = this->impl_->ros_node_->get_name();
      auto complete_node_name = node_namespace + "/" + node_name;
      if (event.node == complete_node_name) {
        // Ignore new parameters since they're handled before this callback is registered
        const auto & param_list = event.changed_parameters;

        // Iterate over parameters
        for (auto & parameter : param_list) {
          auto temp_param = this->impl_->ros_node_->get_parameter(parameter.name);

          // Propagate global slip compliance lateral parameter to all wheels
          // This does not set the actual parameter in gazebo
          if (parameter.name == "slip_compliance_unitless_lateral") {
            double slip = temp_param.as_double();
            RCLCPP_INFO(
              this->impl_->ros_node_->get_logger(),
              "Setting lateral slip compliance for all wheels: %.3e", slip);
            // Iterate over all wheels
            for (auto & wheel_parameter : this->impl_->map_slip_lateral_default_) {
              auto param_name = parameter.name + "/" + wheel_parameter.first;
              this->impl_->ros_node_->set_parameter(rclcpp::Parameter(param_name, slip));
            }
          }

          // Propagate global slip compliance longitudinal parameter to all wheels
          // This does not set the actual parameter in gazebo
          if (parameter.name == "slip_compliance_unitless_longitudinal") {
            double slip = temp_param.as_double();
            RCLCPP_INFO(
              this->impl_->ros_node_->get_logger(),
              "Setting longitudinal slip compliance for all wheels: %.3e", slip);
            // Iterate over all wheels
            for (auto & wheel_parameter : this->impl_->map_slip_longitudinal_default_) {
              auto param_name = parameter.name + "/" + wheel_parameter.first;
              this->impl_->ros_node_->set_parameter(rclcpp::Parameter(param_name, slip));
            }
          }

          // Set lateral slip compliance for individual wheel
          // Must actually set the parameter in gazebo
          if (parameter.name.find("slip_compliance_unitless_lateral/") != std::string::npos) {
            auto wheel_name = parameter.name.substr(parameter.name.find("/") + 1);
            if (this->impl_->map_slip_lateral_default_.count(wheel_name) == 1) {
              // The parameter has a valid name
              double slip = temp_param.as_double();
              RCLCPP_INFO(
                this->impl_->ros_node_->get_logger(),
                "New lateral slip compliance for %s: %.3e", wheel_name.c_str(), slip);
              this->SetSlipComplianceLateral(wheel_name, slip);
            }
          }

          // Set longitudinal slip compliance for individual wheel
          // Must actually set the parameter in gazebo
          if (parameter.name.find("slip_compliance_unitless_longitudinal/") != std::string::npos) {
            auto wheel_name = parameter.name.substr(parameter.name.find("/") + 1);
            if (this->impl_->map_slip_longitudinal_default_.count(wheel_name) == 1) {
              // The parameter has a valid name
              double slip = temp_param.as_double();
              RCLCPP_INFO(
                this->impl_->ros_node_->get_logger(),
                "New longitudinal slip compliance for %s: %.3e", wheel_name.c_str(), slip);
              this->SetSlipComplianceLongitudinal(wheel_name, slip);
            }
          }

#ifdef GAZEBO_WHEELSLIP_HAS_FRICTION
          // Set the friction coefficient in the primary direction for an individual wheel
          if (parameter.name.find("friction_coefficient_primary/") != std::string::npos) {
            auto wheel_name = parameter.name.substr(parameter.name.find("/") + 1);
            if (this->impl_->map_friction_primary_default_.count(wheel_name) == 1) {
              double friction = temp_param.as_double();
              if (this->SetMuPrimary(wheel_name, friction)) {
                RCLCPP_INFO(
                  this->impl_->ros_node_->get_logger(),
                  "New friction coefficient in primary direction for %s: %.3e",
                  wheel_name.c_str(), friction);
              } else {
                RCLCPP_ERROR(
                  this->impl_->ros_node_->get_logger(),
                  "Unable to set friction coefficient in primary direction for %s",
                  wheel_name.c_str());
              }
            }
          }

          // Set the friction coefficient in the secondary direction for an individual wheel
          if (parameter.name.find("friction_coefficient_secondary/") != std::string::npos) {
            auto wheel_name = parameter.name.substr(parameter.name.find("/") + 1);
            if (this->impl_->map_friction_secondary_default_.count(wheel_name) == 1) {
              double friction = temp_param.as_double();
              if (this->SetMuSecondary(wheel_name, friction)) {
                RCLCPP_INFO(
                  this->impl_->ros_node_->get_logger(),
                  "New friction coefficient in secondary direction for %s: %.3e",
                  wheel_name.c_str(), friction);
              } else {
                RCLCPP_ERROR(
                  this->impl_->ros_node_->get_logger(),
                  "Unable to set friction coefficient in secondary direction for %s",
                  wheel_name.c_str());
              }
            }
          }
#endif
        }
        // Iteration over parameters done
      }
    };

  impl_->parameter_event_handler_ =
    std::make_shared<rclcpp::ParameterEventHandler>(impl_->ros_node_);
  impl_->parameter_set_event_callback_ = impl_->parameter_event_handler_->
    add_parameter_event_callback(parameter_set_callback_function);

  double wheel_spin_tolerance = 1.0e-3;
  if (!_sdf->HasElement("wheel_spin_tolerance")) {
    RCLCPP_INFO(
      impl_->ros_node_->get_logger(), "Missing <wheel_spin_tolerance>, defaults to %f",
      wheel_spin_tolerance);
  } else {
    wheel_spin_tolerance = _sdf->GetElement("wheel_spin_tolerance")->Get<double>();
  }

  double publisher_update_rate = 100.0;
  if (!_sdf->HasElement("publisher_update_rate")) {
    RCLCPP_INFO(
      impl_->ros_node_->get_logger(),
      "Missing <publisher_update_rate>, defaults to %f", publisher_update_rate);
  } else {
    publisher_update_rate = _sdf->GetElement("publisher_update_rate")->Get<double>();
  }

  if (publisher_update_rate > 0.0) {
    impl_->publisher_update_period_ = 1.0 / publisher_update_rate;
  } else {
    impl_->publisher_update_period_ = 0.0;
  }

  impl_->publisher_last_update_time_ = _model->GetWorld()->SimTime();

  impl_->slip_publisher_ = impl_->ros_node_->create_publisher<gazebo_msgs::msg::WheelSlip>(
    "wheel_slip", 10);

  auto on_update_callback = [this, wheel_spin_tolerance](
    const gazebo::common::UpdateInfo & info) {

    #ifdef IGN_PROFILER_ENABLE
      IGN_PROFILE("GazeboRosWheelSlip publisher");
    #endif

      gazebo::common::Time current_time = info.simTime;

      // If the world is reset, for example
      if (current_time < impl_->publisher_last_update_time_) {
        RCLCPP_INFO(impl_->ros_node_->get_logger(), "Negative sim time difference detected.");
        impl_->publisher_last_update_time_ = current_time;
      }

      // Check period
      double seconds_since_last_update = (
        current_time - impl_->publisher_last_update_time_).Double();

      if (seconds_since_last_update < impl_->publisher_update_period_) {
        return;
      }

    #ifdef IGN_PROFILER_ENABLE
      IGN_PROFILE_BEGIN("fill ROS message");
    #endif

      // Reset message
      impl_->slip_msg = gazebo_msgs::msg::WheelSlip();

      std::map<std::string, ignition::math::Vector3d> slips;
      this->GetSlips(slips);
      for (const auto & wheel : slips) {
        auto name = wheel.first;
        auto slip = wheel.second;
        auto spin_speed = slip.Z();
        auto long_vel = slip.X() + slip.Z();
        auto lat_vel = slip.Y();
        double long_slip;
        double lat_slip;
        if (fabs(spin_speed) < wheel_spin_tolerance) {
          long_slip = 0;
          lat_slip = 0;
        } else {
          long_slip = (spin_speed - long_vel) / spin_speed;
          lat_slip = atan2(lat_vel, long_vel);
        }
        impl_->slip_msg.name.push_back(name);
        impl_->slip_msg.lateral_slip.push_back(lat_slip);
        impl_->slip_msg.longitudinal_slip.push_back(long_slip);
      }

    #ifdef IGN_PROFILER_ENABLE
      IGN_PROFILE_END();
      IGN_PROFILE_BEGIN("publish");
    #endif

      // Publish
      impl_->slip_publisher_->publish(impl_->slip_msg);

    #ifdef IGN_PROFILER_ENABLE
      IGN_PROFILE_END();
    #endif

      // Update time
      impl_->publisher_last_update_time_ = current_time;
    };

  // Callback on every iteration
  impl_->publisher_update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    on_update_callback);
}

GZ_REGISTER_MODEL_PLUGIN(GazeboRosWheelSlip)
}  // namespace gazebo_plugins
