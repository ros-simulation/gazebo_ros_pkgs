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

#include <gazebo/common/Time.hh>
#include <gazebo/test/ServerFixture.hh>
#include <gazebo_msgs/msg/wheel_slip.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <vector>

using namespace std::chrono_literals;

/// Tests the gazebo_ros_wheelslip plugin
class GazeboRosWheelSlipTest : public gazebo::ServerFixture
{
public:
  void test_parameters(
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

    // Wait for service to show up
    ASSERT_TRUE(parameters_client->wait_for_service(5s));

    // Verify the parameters
    std::vector<std::string> parameter_names;
    for (auto & element : parameter_pairs) {
      parameter_names.push_back(element.first);
    }

    /* Query for the parameters and check if they match with the expected ones.
       Plugin takes a while to set the parameters, so we make multiple attempts to
       check the parameters */
    int max_attempts = 30;
    bool flag_do_parameters_match;
    std::vector<rclcpp::Parameter> parameters_received;
    for (int i = 0; i < max_attempts; i++) {
      // Assume all expected and received parameters match
      flag_do_parameters_match = true;

      // Query for parameters
      parameters_received = parameters_client->get_parameters(parameter_names);
      for (auto & parameter : parameters_received) {
        try {
          if (fabs(parameter_pairs[parameter.get_name()] - parameter.as_double()) < 1e-3) {
            // Parameters don't match
            flag_do_parameters_match = false;
            break;
          }
        } catch (std::runtime_error &) {
          // Parameter may not be set yet
          flag_do_parameters_match = false;
          break;
        }
      }

      // Check the flag_do_parameters_match, if it was true, no need to query again
      if (flag_do_parameters_match) {break;}
      std::this_thread::sleep_for(0.2s);
    }
    if (!flag_do_parameters_match) {
      for (auto & parameter : parameters_received) {
        EXPECT_NEAR(parameter_pairs[parameter.get_name()], parameter.as_double(), 1e-3)
          << "unexpected value for param '" << parameter.get_name() << "'";
      }
    }
  }
};


TEST_F(GazeboRosWheelSlipTest, TestRosGlobalParamsOverrideSdf)
{
  // World file contains:
  // - ROS parameters (for all wheels) - ('slip_compliance_unitless_lateral',
  //                  'slip_compliance_unitless_longitudinal')
  // - SDF tags for wheels : <slip_compliance_lateral>, <slip_compliance_longitudinal>
  // ----------------------------------------------------------------------------------------------
  // World file does NOT contain:
  // - ROS Parameters (for each wheel) - ('slip_compliance_unitless_lateral/wheel_*',
  //                  'slip_compliance_unitless_longitudinal/wheel_*')
  // ----------------------------------------------------------------------------------------------
  // Expected result: ROS parameters (for all wheels) override the ones in SDF
  std::map<std::string, double> parameter_pairs = {
    {"slip_compliance_unitless_lateral", 0.3},
    {"slip_compliance_unitless_lateral/wheel_front", 0.3},
    {"slip_compliance_unitless_longitudinal", 0.4},
    {"slip_compliance_unitless_longitudinal/wheel_front", 0.4}};

  this->test_parameters(
    "worlds/wheelslip_worlds/gazebo_ros_wheelslip_2.world",
    "trisphere_cycle_slip0/wheel_slip_front",
    parameter_pairs
  );
}

TEST_F(GazeboRosWheelSlipTest, TestRosGlobalParamsApply)
{
  // World file contains:
  // - ROS parameters ('slip_compliance_unitless_lateral',
  //                  'slip_compliance_unitless_longitudinal')
  // ----------------------------------------------------------------------------------------------
  // World file does NOT contain:
  // - SDF tags for wheels : <slip_compliance_lateral>, <slip_compliance_longitudinal>
  // - ROS Parameters ('slip_compliance_unitless_lateral/wheel_*',
  //                  'slip_compliance_unitless_longitudinal/wheel_*')
  // ----------------------------------------------------------------------------------------------
  // Expected result: ROS parameters (for all wheels) should apply
  std::map<std::string, double> parameter_pairs = {
    {"slip_compliance_unitless_lateral", 0.3},
    {"slip_compliance_unitless_lateral/wheel_front", 0.3},
    {"slip_compliance_unitless_longitudinal", 0.4},
    {"slip_compliance_unitless_longitudinal/wheel_front", 0.4}};

  this->test_parameters(
    "worlds/wheelslip_worlds/gazebo_ros_wheelslip_3.world",
    "trisphere_cycle_slip0/wheel_slip_front",
    parameter_pairs
  );
}


TEST_F(GazeboRosWheelSlipTest, RosLocalParamsOverrideSdf)
{
  // World file contains:
  // - SDF tags for wheels : <slip_compliance_lateral>, <slip_compliance_longitudinal>
  // - ROS Parameters (for each wheel) - ('slip_compliance_unitless_lateral/wheel_*',
  //                  'slip_compliance_unitless_longitudinal/wheel_*')
  // ----------------------------------------------------------------------------------------------
  // World file does NOT contain:
  // - ROS parameters (for all wheels) - ('slip_compliance_unitless_lateral',
  //                  'slip_compliance_unitless_longitudinal')
  // ----------------------------------------------------------------------------------------------
  // Expected result: ROS parameters (for each wheel) should override the SDF ones
  std::map<std::string, double> parameter_pairs = {
    {"slip_compliance_unitless_lateral/wheel_front", 0.3},
    {"slip_compliance_unitless_longitudinal/wheel_front", 0.4}};

  this->test_parameters(
    "worlds/wheelslip_worlds/gazebo_ros_wheelslip_4.world",
    "trisphere_cycle_slip0/wheel_slip_front",
    parameter_pairs
  );
}

TEST_F(GazeboRosWheelSlipTest, TestRosGlobalParamsOverrideAll)
{
  // World file contains:
  // - SDF tags for wheels : <slip_compliance_lateral>, <slip_compliance_longitudinal>
  // - ROS Parameters (for each wheel) - ('slip_compliance_unitless_lateral/wheel_*',
  //                  'slip_compliance_unitless_longitudinal/wheel_*')
  // - ROS parameters (for all wheels) - ('slip_compliance_unitless_lateral',
  //                  'slip_compliance_unitless_longitudinal')
  // ----------------------------------------------------------------------------------------------
  // Expected result: ROS parameters for all wheels should override SDF values
  // and ROS parameters for each wheel should override parameters for all wheels.
  std::map<std::string, double> parameter_pairs = {
    {"slip_compliance_unitless_lateral", 0.1},
    {"slip_compliance_unitless_lateral/wheel_rear_left", 0.6},
    {"slip_compliance_unitless_lateral/wheel_rear_right", 0.1},
    {"slip_compliance_unitless_longitudinal", 0.2},
    {"slip_compliance_unitless_longitudinal/wheel_rear_left", 0.7},
    {"slip_compliance_unitless_longitudinal/wheel_rear_right", 0.2}};

  this->test_parameters(
    "worlds/wheelslip_worlds/gazebo_ros_wheelslip_5.world",
    "trisphere_cycle_slip0/wheel_slip_rear",
    parameter_pairs
  );
}


TEST_F(GazeboRosWheelSlipTest, TestSetParameters)
{
  // World file contains:
  // - SDF tags for wheels : <slip_compliance_lateral>, <slip_compliance_longitudinal>
  // ----------------------------------------------------------------------------------------------
  // World file does NOT contain:
  // - ROS Parameters (for each wheel) - ('slip_compliance_unitless_lateral/wheel_*',
  //                  'slip_compliance_unitless_longitudinal/wheel_*')
  // - ROS parameters (for all wheels) - ('slip_compliance_unitless_lateral',
  //                  'slip_compliance_unitless_longitudinal')
  // ----------------------------------------------------------------------------------------------
  // Expected result: SDF parameters should apply to wheels
  this->Load("worlds/wheelslip_worlds/gazebo_ros_wheelslip_1.world", true);
  auto world = gazebo::physics::get_world();
  ASSERT_NE(nullptr, world);
  auto node = std::make_shared<rclcpp::Node>("test_gazebo_ros_wheelslip");
  ASSERT_NE(nullptr, node);
  auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(
    node,
    "trisphere_cycle_slip0/wheel_slip_rear");

  std::map<std::string, double> parameter_pairs = {
    {"slip_compliance_unitless_lateral/wheel_rear_left", 0.0},
    {"slip_compliance_unitless_lateral/wheel_rear_right", 0.5},
    {"slip_compliance_unitless_longitudinal/wheel_rear_left", 0.4},
    {"slip_compliance_unitless_longitudinal/wheel_rear_right", 0.6}};

  // TEST 1 : Verify parameters were set as per the SDF, negative values should be replaced by 0
  ASSERT_TRUE(parameters_client->wait_for_service(5s));
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
  parameter_pairs["slip_compliance_unitless_lateral/wheel_rear_left"] = 0.3;
  std::vector<rclcpp::Parameter> change_param = {
    rclcpp::Parameter("slip_compliance_unitless_lateral/wheel_rear_left", 0.3)};
  auto result = parameters_client->set_parameters(change_param);
  ASSERT_TRUE(result[0].successful);

  parameters_received = parameters_client->get_parameters(parameter_names);
  for (auto & parameter : parameters_received) {
    ASSERT_EQ(parameter_pairs[parameter.get_name()], parameter.as_double());
  }

  // TEST 3 : Set global slip compliance parameters, which should override the ones for wheels
  std::vector<rclcpp::Parameter> change_params_global = {
    rclcpp::Parameter("slip_compliance_unitless_lateral", 0.1),
    rclcpp::Parameter("slip_compliance_unitless_longitudinal", 0.2)
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
      ASSERT_EQ(parameter.as_double(), 0.1);
    } else {
      ASSERT_EQ(parameter.as_double(), 0.2);
    }
  }
}

#if (GAZEBO_MAJOR_VERSION == 11 && GAZEBO_MINOR_VERSION >= 11)
TEST_F(GazeboRosWheelSlipTest, TestFrictionParameters)
{
  // World file contains:
  // - SDF wheels link_names: wheel_rear_left, wheel_rear_right
  // ----------------------------------------------------------------------------------------------
  // World file does NOT contain:
  // - ROS Parameters (for each wheel) - ('friction_coefficient_primary/wheel_*',
  //                  'friction_coefficient_secondary/wheel_*')
  // ----------------------------------------------------------------------------------------------
  // Expected result: Default friction ROS Parameters are 1.0 and updating new friction should apply
  this->Load("worlds/wheelslip_worlds/gazebo_ros_wheelslip_1.world", true);
  auto world = gazebo::physics::get_world();
  ASSERT_NE(nullptr, world);
  auto node = std::make_shared<rclcpp::Node>("test_gazebo_ros_wheelslip");
  ASSERT_NE(nullptr, node);
  auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(
    node,
    "trisphere_cycle_slip0/wheel_slip_rear");

  std::map<std::string, double> parameter_pairs = {
    {"friction_coefficient_primary/wheel_rear_left", 1.0},
    {"friction_coefficient_primary/wheel_rear_right", 1.0},
    {"friction_coefficient_secondary/wheel_rear_left", 1.0},
    {"friction_coefficient_secondary/wheel_rear_right", 1.0}};

  // TEST 1 : Verify parameters default friction parameters are 1.0
  // Since the trisphere model does not set SDF tags //friction/ode/mu or //friction/ode/mu2,
  // we expect both to be 1.0 by default
  ASSERT_TRUE(parameters_client->wait_for_service(5s));
  std::vector<std::string> parameter_names;
  for (auto & element : parameter_pairs) {
    parameter_names.push_back(element.first);
  }
  std::this_thread::sleep_for(1s);
  auto parameters_received = parameters_client->get_parameters(parameter_names);
  EXPECT_EQ(parameters_received.size(), parameter_pairs.size());
  for (auto & parameter : parameters_received) {
    ASSERT_EQ(parameter_pairs[parameter.get_name()], parameter.as_double());
  }

  // TEST 2 : Set friction coefficient for one wheel, verify others remain unchanged
  parameter_pairs["friction_coefficient_primary/wheel_rear_left"] = 0.5;
  std::vector<rclcpp::Parameter> change_param = {
    rclcpp::Parameter("friction_coefficient_primary/wheel_rear_left", 0.5)};
  auto result = parameters_client->set_parameters(change_param);
  ASSERT_TRUE(result[0].successful);

  parameters_received = parameters_client->get_parameters(parameter_names);
  EXPECT_EQ(parameters_received.size(), parameter_pairs.size());
  for (auto & parameter : parameters_received) {
    ASSERT_EQ(parameter_pairs[parameter.get_name()], parameter.as_double());
  }
}
#endif

class GazeboRosWheelSlipPublisherTest : public gazebo::ServerFixture
{
public:
  void test_publishing(
    std::string world_file,
    bool at_rest)
  {
    // Load test world and start paused
    this->Load(world_file, true);

    // World
    auto world = gazebo::physics::get_world();
    ASSERT_NE(nullptr, world);

    // Create node and executor
    auto node = std::make_shared<rclcpp::Node>("gazebo_ros_joint_state_publisher_test");
    ASSERT_NE(nullptr, node);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    // Create subscriber
    gazebo_msgs::msg::WheelSlip::SharedPtr latestMsg;
    auto sub = node->create_subscription<gazebo_msgs::msg::WheelSlip>(
      "trisphere_cycle_slip/wheel_slip", rclcpp::QoS(1),
      [&latestMsg](const gazebo_msgs::msg::WheelSlip::SharedPtr msg) {
        latestMsg = msg;
      });

    // Spin until we get a message or timeout
    auto startTime = std::chrono::steady_clock::now();
    while (latestMsg == nullptr &&
      (std::chrono::steady_clock::now() - startTime) < std::chrono::seconds(2))
    {
      world->Step(1000);
      executor.spin_once(100ms);
      gazebo::common::Time::MSleep(100);
    }

    // Check that we receive the latest joint state
    ASSERT_NE(nullptr, latestMsg);

    EXPECT_EQ(3u, latestMsg->name.size());
    EXPECT_EQ(latestMsg->name.size(), latestMsg->lateral_slip.size());
    EXPECT_EQ(latestMsg->name.size(), latestMsg->longitudinal_slip.size());

    for (unsigned int i = 0; i < latestMsg->name.size(); ++i) {
      if (at_rest) {
        EXPECT_DOUBLE_EQ(0.0, latestMsg->lateral_slip[i]);
        EXPECT_DOUBLE_EQ(0.0, latestMsg->longitudinal_slip[i]);
      } else {
        EXPECT_NEAR(0.0, latestMsg->lateral_slip[i], 1e-4);
        EXPECT_NEAR(0.3, latestMsg->longitudinal_slip[i], 0.05);
      }
    }
  }
};

TEST_F(GazeboRosWheelSlipPublisherTest, Publishing)
{
  this->test_publishing(
    "worlds/gazebo_ros_wheel_slip.world",
    false
  );
}

TEST_F(GazeboRosWheelSlipPublisherTest, PublishingAtRest)
{
  this->test_publishing(
    "worlds/gazebo_ros_wheel_slip_at_rest.world",
    true
  );
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
