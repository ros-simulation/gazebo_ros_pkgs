# Copyright 2021 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import unittest

import launch
import launch.actions

import launch_testing
import launch_testing.asserts
import pytest
import os

import rclpy
from gazebo_msgs.msg import PerformanceMetrics
from rclpy.node import Node
from rcl_interfaces.msg import Parameter, ParameterValue
from ros2param.api import call_set_parameters
import time
from threading import Thread


@pytest.mark.launch_test
def generate_test_description():
    os.chdir(os.path.dirname(__file__))
    return launch.LaunchDescription([
        launch.actions.ExecuteProcess(
            cmd=['gzserver', '-s', 'libgazebo_ros_init.so'],
            output='screen',
            sigterm_timeout='20',
            sigkill_timeout='20'
        ),
        launch_testing.actions.ReadyToTest()
    ])


class MyTestingClass(unittest.TestCase):
    def test_routine(self):
        rclpy.init()
        node = MakeTestNode()
        node_found = node.wait_for_gazebo_node()

        if node_found:
            response = node.set_parameter(False)
            if response.successful:
                print("enable_performance_metrics successfully set to false !")
                node.start_subscriber()
                time.sleep(2.5)

                if node.msg_count > 0:
                    print("Received {} messages, test failed".format(node.msg_count))
                    rclpy.shutdown()
                    assert False
                else:
                    print("No messages were published")
                    rclpy.shutdown()
                    assert True
            else:
                print("Parameter could not be set, exiting")
                rclpy.shutdown()
                assert False
        else:
            print("Could not find Gazebo node, exiting")
            rclpy.shutdown()
            assert False


class MakeTestNode(Node):
    def __init__(self):
        super().__init__('testing_node')
        self.flag_gazebo_node = False
        self.msg_count = 0

    def wait_for_gazebo_node(self):
        start = time.time()
        print("Waiting to find Gazebo Node...")
        while time.time() - start < 5 and not self.flag_gazebo_node:
            active_nodes = self.get_node_names()
            if 'gazebo' in active_nodes:
                self.flag_gazebo_node = True
                print("Found the gazebo node !")
                time.sleep(0.1)

        if not self.flag_gazebo_node:
            print("Gazebo node was not found !")
        return self.flag_gazebo_node

    # Set enable_performance_metrics parameter to false
    def set_parameter(self, state=False):
        parameter = Parameter()
        parameter.name = 'enable_performance_metrics'

        temp_value = ParameterValue()
        temp_value.bool_value = state
        temp_value.type = 1
        parameter.value = temp_value

        return call_set_parameters(node=self, node_name="gazebo",
                                    parameters=[parameter]).results[0]

    # Start listening on messages
    def start_subscriber(self):
        self.subscription = self.create_subscription(
            PerformanceMetrics,
            'performance_metrics',
            self.subscriber_callback,
            10)
        self.subscription

        # Add a spin thread
        self.ros_spin_thread = Thread(target=lambda node: rclpy.spin(node), args=(self,))
        self.ros_spin_thread.start()

    def subscriber_callback(self, data):
        self.msg_count += 1
