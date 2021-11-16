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
public:
  void TestParameters(
    std::string world_file,
    std::string node_name,
    std::map<std::string, double> parameter_pairs)
  {
    // Load the world
    this->Load(world_file, true);
    auto world = gazebo::physics::get_world();
    ASSERT_NE(nullptr, world);
    auto node = std::make_shared<rclcpp::Node>("test_gazebo_ros_wheelslip");
    ASSERT_NE(nullptr, node);

    // Create parameters client
    auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(
      node, node_name);

    // Wait for serive to show up
    int counter = 0;
    while (!parameters_client->wait_for_service(1s)) {
      std::cout << "service not available, waiting again..." << std::endl;
      counter++;
      if (counter > 5) {FAIL();}
    }

    // Verify the parameters
    std::vector<std::string> parameter_names;
    for (auto & element : parameter_pairs) {
      parameter_names.push_back(element.first);
    }
    std::this_thread::sleep_for(1s);
    auto parameters_received = parameters_client->get_parameters(parameter_names);
    for (auto & parameter : parameters_received) {
      ASSERT_EQ(parameter_pairs[parameter.get_name()], parameter.as_double());
    }
  }
};


TEST_F(GazeboRosWheelSlipTest, TestWorldFile_2)
{
  // We have global ROS parameters and sdf parameters in the world file
  std::map<std::string, double> parameter_pairs = {
    {"slip_compliance_unitless_lateral", 10.0},
    {"slip_compliance_unitless_lateral/wheel_front", 10.0},
    {"slip_compliance_unitless_longitudinal", 11.0},
    {"slip_compliance_unitless_longitudinal/wheel_front", 11.0}};

  this->TestParameters(
    "worlds/wheelslip_worlds/gazebo_ros_wheelslip_2.world",
    "trisphere_cycle_slip0/wheel_slip_front",
    parameter_pairs
  );
}

TEST_F(GazeboRosWheelSlipTest, TestWorldFile_3)
{
  // We have only global ROS parameters in the world file
  std::map<std::string, double> parameter_pairs = {
    {"slip_compliance_unitless_lateral", 10.0},
    {"slip_compliance_unitless_lateral/wheel_front", 10.0},
    {"slip_compliance_unitless_longitudinal", 11.0},
    {"slip_compliance_unitless_longitudinal/wheel_front", 11.0}};

  this->TestParameters(
    "worlds/wheelslip_worlds/gazebo_ros_wheelslip_3.world",
    "trisphere_cycle_slip0/wheel_slip_front",
    parameter_pairs
  );
}


TEST_F(GazeboRosWheelSlipTest, TestWorldFile_4)
{
  // We have local ROS parameters and sdf parameters in the world file
  std::map<std::string, double> parameter_pairs = {
    {"slip_compliance_unitless_lateral", 1.0},
    {"slip_compliance_unitless_lateral/wheel_front", 10.0},
    {"slip_compliance_unitless_longitudinal", 2.0},
    {"slip_compliance_unitless_longitudinal/wheel_front", 11.0}};

  this->TestParameters(
    "worlds/wheelslip_worlds/gazebo_ros_wheelslip_4.world",
    "trisphere_cycle_slip0/wheel_slip_front",
    parameter_pairs
  );
}

TEST_F(GazeboRosWheelSlipTest, TestWorldFile_5)
{
  // We have global, local ROS params as well as sdf params in the world file
  std::map<std::string, double> parameter_pairs = {
    {"slip_compliance_unitless_lateral", 100.5},
    {"slip_compliance_unitless_lateral/wheel_rear_left", 100.5},
    {"slip_compliance_unitless_lateral/wheel_rear_right", 100.5},
    {"slip_compliance_unitless_longitudinal", 200.67},
    {"slip_compliance_unitless_longitudinal/wheel_rear_left", 200.67},
    {"slip_compliance_unitless_longitudinal/wheel_rear_right", 200.67}};

  this->TestParameters(
    "worlds/wheelslip_worlds/gazebo_ros_wheelslip_5.world",
    "trisphere_cycle_slip0/wheel_slip_rear",
    parameter_pairs
  );
}


TEST_F(GazeboRosWheelSlipTest, TestWorldFile_1)
{
  this->Load("worlds/wheelslip_worlds/gazebo_ros_wheelslip_1.world", true);
  auto world = gazebo::physics::get_world();
  ASSERT_NE(nullptr, world);
  auto node = std::make_shared<rclcpp::Node>("test_gazebo_ros_wheelslip");
  ASSERT_NE(nullptr, node);
  auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(
    node,
    "trisphere_cycle_slip0/wheel_slip_rear");

  std::map<std::string, double> parameter_pairs = {
    {"slip_compliance_unitless_lateral", 5},
    {"slip_compliance_unitless_lateral/wheel_rear_left", 0.0},
    {"slip_compliance_unitless_lateral/wheel_rear_right", 5},
    {"slip_compliance_unitless_longitudinal", 6},
    {"slip_compliance_unitless_longitudinal/wheel_rear_left", 4},
    {"slip_compliance_unitless_longitudinal/wheel_rear_right", 6}};

  // TEST 1 : Verify parameters were set as per the SDF, negative values should be replaced by 0
  int counter = 0;
  while (!parameters_client->wait_for_service(1s)) {
    std::cout << "service not available, waiting again..." << std::endl;
    counter++;
    if (counter > 5) {FAIL();}
  }
  std::vector<std::string> parameter_names;
  for (auto & element : parameter_pairs) {
    parameter_names.push_back(element.first);
  }
  std::this_thread::sleep_for(1s);
  auto parameters_received = parameters_client->get_parameters(parameter_names);
  for (auto & parameter : parameters_received) {
    ASSERT_EQ(parameter_pairs[parameter.get_name()], parameter.as_double());
  }

  // TEST 2 : Set slip compliance for one wheel, verify others remain unchanged
  parameter_pairs["slip_compliance_unitless_lateral/wheel_rear_left"] = 3.0;
  std::vector<rclcpp::Parameter> change_param = {
    rclcpp::Parameter("slip_compliance_unitless_lateral/wheel_rear_left", 3.0)};
  auto result = parameters_client->set_parameters(change_param);
  ASSERT_TRUE(result[0].successful);

  parameters_received = parameters_client->get_parameters(parameter_names);
  for (auto & parameter : parameters_received) {
    ASSERT_EQ(parameter_pairs[parameter.get_name()], parameter.as_double());
  }

  // TEST 3 : Set global slip compliance parameters, which should override the ones for wheels
  std::vector<rclcpp::Parameter> change_params_global = {
    rclcpp::Parameter("slip_compliance_unitless_lateral", 10.0),
    rclcpp::Parameter("slip_compliance_unitless_longitudinal", 11.0)
  };
  result = parameters_client->set_parameters(change_params_global);
  ASSERT_TRUE(result[0].successful);
  ASSERT_TRUE(result[1].successful);

  // slip compliance parameters for individual wheels are set after the global ones
  // This takes time, and the sleep_for() below allows them to be set.
  std::this_thread::sleep_for(2s);
  parameters_received = parameters_client->get_parameters(parameter_names);
  for (auto & parameter : parameters_received) {
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
