# Copyright 2020 Open Source Robotics Foundation, Inc.
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
            'mock_robot_state_publisher',
            description='Path to mock robot state publisher executable',
        ),
        launch.actions.ExecuteProcess(
            cmd=[launch.substitutions.LaunchConfiguration('mock_robot_state_publisher')],
            output='screen',
        ),
        launch.actions.ExecuteProcess(
            cmd=['gzserver', '-s', 'libgazebo_ros_factory.so'],
            output='screen',
            sigterm_timeout='30',
            sigkill_timeout='30'
        ),
        launch_testing.actions.ReadyToTest()
    ])


class TestTerminatingProc(unittest.TestCase):

    def test_spawn_entity(self, launch_service, proc_info, proc_output):
        """Test spawn entity from topic."""
        entity_spawner_action = launch.actions.ExecuteProcess(
            cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                 '-entity', 'mock_entity_test_spawn_entity', '-topic', '/mock_robot_description'],
            output='screen'
        )
        with launch_testing.tools.launch_process(
              launch_service, entity_spawner_action, proc_info, proc_output) as command:
            assert command.wait_for_shutdown(timeout=10)
        assert command.exit_code == launch_testing.asserts.EXIT_OK

    def test_spawn_entity_timeout(self, launch_service, proc_info, proc_output):
        """Test spawn entity from topic and output with -timeout option."""
        entity_spawner_action = launch.actions.ExecuteProcess(
            cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                 '-entity', 'mock_entity_test_spawn_entity_timeout',
                 '-topic', '/mock_robot_description',
                 '-timeout', '10'],
            output='screen'
        )
        with launch_testing.tools.launch_process(
              launch_service, entity_spawner_action, proc_info, proc_output) as command:
            assert command.wait_for_shutdown(timeout=10)
        assert command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=['Waiting for service /spawn_entity, timeout = 10'],
            text=command.output,
            strict=False
        )

    def test_spawn_entity_deprecated_timeout(self, launch_service, proc_info, proc_output):
        """Test spawn entity from topic and deprecation with -spawn_service_timeout option."""
        entity_spawner_action = launch.actions.ExecuteProcess(
            cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                 '-entity', 'mock_entity_test_spawn_entity_deprecated_timeout',
                 '-topic', '/mock_robot_description',
                 '-spawn_service_timeout', '10'],
            output='screen'
        )
        with launch_testing.tools.launch_process(
              launch_service, entity_spawner_action, proc_info, proc_output) as command:
            assert command.wait_for_shutdown(timeout=10)
        assert command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=[
                "'-spawn_service_timeout' is deprecated, use '-timeout' instead",
                'Waiting for service /spawn_entity, timeout = 10',
            ],
            text=command.output,
            strict=False
        )
