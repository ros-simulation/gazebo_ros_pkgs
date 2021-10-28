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
  impl_->ros_node_ = gazebo_ros::Node::Get(_sdf, _model);

  auto param_change_callback =
    [this](std::vector<rclcpp::Parameter> parameters) {
      auto result = rcl_interfaces::msg::SetParametersResult();
      result.successful = true;
      for (const auto & parameter : parameters) {
        std::string param_name = parameter.get_name();
        if (param_name == "slip_compliance_unitless_lateral") {
          double slip = parameter.as_double();
          if (slip >= 0.) {
            RCLCPP_INFO(
              impl_->ros_node_->get_logger(),
              "New lateral slip compliance: %.3e", slip);
            this->SetSlipComplianceLateral(slip);
          }
        } else if (param_name == "slip_compliance_unitless_longitudinal") {
          double slip = parameter.as_double();
          if (slip >= 0.) {
            RCLCPP_INFO(
              impl_->ros_node_->get_logger(),
              "New longitudinal slip compliance: %.3e", slip);
            this->SetSlipComplianceLongitudinal(slip);
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
  impl_->ros_node_->declare_parameter("slip_compliance_unitless_lateral", -1.);
  impl_->ros_node_->declare_parameter("slip_compliance_unitless_longitudinal", -1.);
}

GZ_REGISTER_MODEL_PLUGIN(GazeboRosWheelSlip)
}  // namespace gazebo_plugins
