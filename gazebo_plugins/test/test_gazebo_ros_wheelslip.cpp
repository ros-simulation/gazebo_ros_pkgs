// Copyright 2021 Open Source Robotics Foundation, Inc.
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

#include <gazebo/test/ServerFixture.hh>
#include <gazebo_ros/node.hpp>
#include <gazebo_ros/testing_utils.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <vector>

using namespace std::chrono_literals;

/// Tests the gazebo_ros_wheelslip plugin
class GazeboRosWheelSlipTest : public gazebo::ServerFixture
{
};

TEST_F(GazeboRosWheelSlipTest, SetSlipCompliance)
{
  // Load test world and start paused
  this->Load("worlds/gazebo_ros_wheelslip.world", true);

  // World
  auto world = gazebo::physics::get_world();
  ASSERT_NE(nullptr, world);

  // Create node
  auto node = std::make_shared<rclcpp::Node>("test_gazebo_ros_wheelslip");
  ASSERT_NE(nullptr, node);

  auto parameters_client_rear = std::make_shared<rclcpp::SyncParametersClient>(
    node,
    "trisphere_cycle_slip0/wheel_slip_rear");

  std::map<std::string, double> parameters_rear_pairs = {
    {"slip_compliance_unitless_lateral", 5},
    {"slip_compliance_unitless_lateral/wheel_rear_left", 0.0},
    {"slip_compliance_unitless_lateral/wheel_rear_right", 5},
    {"slip_compliance_unitless_longitudinal", 6},
    {"slip_compliance_unitless_longitudinal/wheel_rear_left", 4},
    {"slip_compliance_unitless_longitudinal/wheel_rear_right", 6}};

  int counter = 0;
  while (!parameters_client_rear->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      rclcpp::shutdown();
    }
    std::cout << "service not available, waiting again..." << std::endl;
    counter++;
    if (counter > 5) {FAIL();}
  }

  std::vector<std::string> parameters_rear_names;
  for (auto & element : parameters_rear_pairs) {
    parameters_rear_names.push_back(element.first);
  }

  // Verify parameters were set as per the SDF, negative values should be replaced by 0
  auto parameters_rear = parameters_client_rear->get_parameters(parameters_rear_names);
  for (auto & parameter : parameters_rear) {
    ASSERT_EQ(parameters_rear_pairs[parameter.get_name()], parameter.as_double());
  }

  // Set slip compliance for one wheel, verify others remain unchanged
  parameters_rear_pairs["slip_compliance_unitless_lateral/wheel_rear_left"] = 3.0;
  std::vector<rclcpp::Parameter> change_param = {
    rclcpp::Parameter("slip_compliance_unitless_lateral/wheel_rear_left", 3.0)};
  auto result = parameters_client_rear->set_parameters(change_param);
  ASSERT_TRUE(result[0].successful);

  parameters_rear = parameters_client_rear->get_parameters(parameters_rear_names);
  for (auto & parameter : parameters_rear) {
    ASSERT_EQ(parameters_rear_pairs[parameter.get_name()], parameter.as_double());
  }

  // Set global slip compliance parameters, which should override the ones for wheels
  std::vector<rclcpp::Parameter> change_params_global = {
    rclcpp::Parameter("slip_compliance_unitless_lateral", 10.0),
    rclcpp::Parameter("slip_compliance_unitless_longitudinal", 11.0)
  };
  result = parameters_client_rear->set_parameters(change_params_global);
  ASSERT_TRUE(result[0].successful);
  ASSERT_TRUE(result[1].successful);

  std::this_thread::sleep_for(2s);
  parameters_rear = parameters_client_rear->get_parameters(parameters_rear_names);
  for (auto & parameter : parameters_rear) {
    if (parameter.get_name().find("slip_compliance_unitless_lateral") != std::string::npos) {
      ASSERT_EQ(parameter.as_double(), 10.0);
    } else {
      ASSERT_EQ(parameter.as_double(), 11.0);
    }
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
