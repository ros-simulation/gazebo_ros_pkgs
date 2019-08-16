# Copyright 2019 Open Source Robotics Foundation, Inc.
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

"""Launch a Gazebo client with command line arguments."""

from os import environ

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression

from scripts import GazeboRosPaths


def generate_launch_description():
    model, plugin, media = GazeboRosPaths.get_paths()

    if 'GAZEBO_MODEL_PATH' in environ:
        model += ':'+environ['GAZEBO_MODEL_PATH']
    if 'GAZEBO_PLUGIN_PATH' in environ:
        plugin += ':'+environ['GAZEBO_PLUGIN_PATH']
    if 'GAZEBO_RESOURCE_PATH' in environ:
        media += ':'+environ['GAZEBO_RESOURCE_PATH']

    env = {'GAZEBO_MODEL_PATH': model,
           'GAZEBO_PLUGIN_PATH': plugin,
           'GAZEBO_RESOURCE_PATH': media}

    return LaunchDescription([
        DeclareLaunchArgument('version', default_value='false',
                              description='Set "true" to output version information'),
        DeclareLaunchArgument('verbose', default_value='false',
                              description='Set "true" to increase messages written to terminal'),
        DeclareLaunchArgument('help', default_value='false',
                              description='Set "true" to produce gzclient help message'),
        DeclareLaunchArgument('extra_gazebo_args', default_value='',
                              description='Extra arguments to be passed to Gazebo'),

        # Specific to gazebo_ros
        DeclareLaunchArgument('gdb', default_value='false',
                              description='Set "true" to run gzserver with gdb'),
        DeclareLaunchArgument('valgrind', default_value='false',
                              description='Set "true" to run gzserver with valgrind'),

        # Execute
        ExecuteProcess(
            cmd=[['gzclient',
                 _boolean_command('version'), ' ',
                 _boolean_command('verbose'), ' ',
                 _boolean_command('help'), ' ',
                 LaunchConfiguration('extra_gazebo_args'),
                  ]],
            output='screen',
            additional_env=env,
            shell=True,
            prefix=PythonExpression(['"gdb -ex run --args" if "true" == "',
                                     LaunchConfiguration('gdb'),
                                     '"else "valgrind" if "true" == "',
                                     LaunchConfiguration('valgrind'),
                                     '"else ""']),
        )
    ])


# Add boolean commands if true
def _boolean_command(arg):
    cmd = ['"--', arg, '" if "true" == "', LaunchConfiguration(arg), '" else ""']
    py_cmd = PythonExpression(cmd)
    return py_cmd
