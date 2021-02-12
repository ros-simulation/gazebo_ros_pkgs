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
import launch_testing.markers
import launch_testing.tools
import pytest


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'mock_gazebo_ros_factory',
            description='Path to mock gazebo ros factory executable',
        ),
        launch.actions.DeclareLaunchArgument(
            'mock_robot_state_publisher',
            description='Path to mock robot state publisher executable',
        ),
        launch.actions.ExecuteProcess(
            cmd=[launch.substitutions.LaunchConfiguration('mock_robot_state_publisher')],
            output='screen',
        ),
        launch.actions.ExecuteProcess(
            cmd=[launch.substitutions.LaunchConfiguration('mock_gazebo_ros_factory')],
            output='screen',
        ),
        launch_testing.actions.ReadyToTest()
    ])


class TestTerminatingProc(unittest.TestCase):

    def test_spawn_entity(self, launch_service, proc_info, proc_output):
        """Test terminating_proc without command line arguments."""
        print('Running spawn entity test on /mock_robot_description topic')
        entity_spawner_action = launch.actions.ExecuteProcess(
            cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py', '-entity',
                 'mock_robot_state_entity', '-topic', '/mock_robot_description'],
            output='screen'
        )
        with launch_testing.tools.launch_process(
              launch_service, entity_spawner_action, proc_info, proc_output) as command:
            assert command.wait_for_shutdown(timeout=10)
        assert command.exit_code == launch_testing.asserts.EXIT_OK
