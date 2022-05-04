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

from threading import Event, Thread
import time
import unittest

from gazebo_msgs.msg import PerformanceMetrics

import launch
import launch.actions

import launch_testing
import launch_testing.asserts

import pytest

from rcl_interfaces.msg import Parameter, ParameterType
from rcl_interfaces.srv import SetParameters
import rclpy
from rclpy.node import Node


@pytest.mark.launch_test
def generate_test_description():
    """Launch a gzserver node."""
    return launch.LaunchDescription([
        launch.actions.ExecuteProcess(
            cmd=['gzserver', '-s', 'libgazebo_ros_init.so'],
            output='screen',
            sigterm_timeout='20',
            sigkill_timeout='20'
        ),
        launch_testing.actions.ReadyToTest()
    ])


class TestPerformanceMetricsParam(unittest.TestCase):
    """Toggle the parameter and check for messages."""

    def test_parameter_enable(self):
        """
        Test function to set parameter to true.

        Checks if /performance_metrics topic starts publishing if
        enable_performance_metrics is set to True.
        """
        rclpy.init()
        node = MakeTestNode('performance_metrics_enable_testing_node')

        assert node.wait_for_gazebo_node(timeout_sec=5.0),\
            'Gazebo node not found, parameter could not be set'

        # Set enable_performance_metrics parameter to True and listen for messages
        response = node.set_parameter(state=True)
        assert response.successful, 'Parameter could not be set to True'

        node.start_subscriber()
        msgs_received_flag = node.msg_event_object.wait(timeout=5.0)
        assert msgs_received_flag, 'Did not receive any messages after\
            setting enable_performance_metrics to True, test failed'
        rclpy.shutdown()

    def test_parameter_disable(self):
        """
        Test function to set parameter to false.

        Checks if /performance_metrics topic stops publishing if
        enable_performance_metrics parameter is set to False.
        """
        rclpy.init()
        node = MakeTestNode('performance_metrics_disable_testing_node')

        assert node.wait_for_gazebo_node(timeout_sec=5.0),\
            'Gazebo node not found, parameter could not be set'

        # Set enable_performance_metrics parameter to False and listen for messages
        response = node.set_parameter(state=False)
        assert response.successful, 'Parameter could not be set to False'

        node.start_subscriber()
        msgs_received_flag = node.msg_event_object.wait(timeout=5.0)
        assert not msgs_received_flag, 'Received messages after\
            setting enable_performance_metrics parameter to False, test failed'
        rclpy.shutdown()


class MakeTestNode(Node):

    def __init__(self, name):
        """Initialize node and counters."""
        super().__init__(name)
        self.flag_gazebo_node = False
        self.msg_event_object = Event()

    def wait_for_gazebo_node(self, timeout_sec=5.0):
        """Wait for 'timeout_sec' seconds to detect a running gazebo node."""
        start = time.time()
        print('Waiting to find Gazebo Node...')
        while time.time() - start < timeout_sec and not self.flag_gazebo_node:
            self.flag_gazebo_node = 'gazebo' in self.get_node_names()
            time.sleep(0.1)

        if not self.flag_gazebo_node:
            print('Gazebo node was not found !')
        return self.flag_gazebo_node

    def set_parameter(self, state=False, timeout_sec=5.0):
        """Set enable_performance_metrics parameter to boolean 'state'."""
        parameter = Parameter()
        parameter.name = 'enable_performance_metrics'
        parameter.value.bool_value = state
        parameter.value.type = ParameterType.PARAMETER_BOOL

        client = self.create_client(SetParameters, 'gazebo/set_parameters')
        ready = client.wait_for_service(timeout_sec)
        if not ready:
            raise RuntimeError('Wait for service timed out')

        request = SetParameters.Request()
        request.parameters = [parameter]
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        response = future.result()
        if response is None:
            raise RuntimeError('Exception while setting parameter: {future.exception()}')

        return response.results[0]

    def start_subscriber(self):
        """
        Create a subscriber.

        Start listening for messages on /performance_metrics topic
        and run rclpy.spin() on a separate thread.
        """
        self.subscription = self.create_subscription(
            PerformanceMetrics,
            'performance_metrics',
            self.subscriber_callback,
            10)

        # Add a spin thread
        self.ros_spin_thread = Thread(target=lambda node: rclpy.spin(node), args=(self,))
        self.ros_spin_thread.start()

    def subscriber_callback(self, data):
        """Set the event object when the a message is received."""
        self.msg_event_object.set()
