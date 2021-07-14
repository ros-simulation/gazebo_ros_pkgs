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


@pytest.mark.launch_test
def generate_test_description():
    os.chdir(os.path.dirname(__file__))
    return launch.LaunchDescription([
        launch.actions.ExecuteProcess(
            cmd=['gzserver', '-s', 'libgazebo_ros_factory.so', '-s', 'libgazebo_ros_init.so'],
            output='screen',
            sigterm_timeout='20',
            sigkill_timeout='20'
        ),
        launch.actions.TimerAction(
            period=2.0,
            actions=[
                launch.actions.ExecuteProcess(
                    cmd=['ros2', 'param', 'set', '/gazebo', 'enable_performance_metrics', 'false'],
                    output='screen'
                )
            ]
        ),
        launch.actions.TimerAction(
            period=7.0,
            actions=[
                launch.actions.ExecuteProcess(
                    cmd=['python3', 'perf_metrics_subscriber.py'],
                    additional_env={'PYTHONUNBUFFERED': '1'},
                    output='screen',
                    sigterm_timeout='1'
                )
            ]
        ),
        launch_testing.actions.ReadyToTest()
    ])


class TestGoodProcess(unittest.TestCase):
    def test_disable_performance_metrics(self, proc_output):
        """ Verify that the parameter can be set  """
        proc_output.assertWaitFor('Set parameter successful', timeout=10, stream='stdout')

    def test_topic_does_not_publish(self, proc_output):
        """ Verify that /performance_metrics does not publish anything"""
        proc_output.assertWaitFor('No messages were published', timeout=10, stream='stdout')
