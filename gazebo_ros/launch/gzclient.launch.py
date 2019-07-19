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

from scripts import GazeboRosPaths

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression


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
        DeclareLaunchArgument('Timer', default_value='false',
                              description='Set "true" to start gzclient with Timer GUI plugin'),
        DeclareLaunchArgument('Cessna', default_value='false',
                              description='Set "true" to start gzclient with Cessna GUI plugin'),
        DeclareLaunchArgument('Keyboard', default_value='false',
                              description='Set "true" to start gzclient with Keyboard GUI plugin'),

        ExecuteProcess(
            cmd=['gzclient',
                 _command('version'),
                 _command('verbose'),
                 _command('help'),
                 _plugin_command('Timer'), _plugin_value('Timer'),
                 _plugin_command('Cessna'), _plugin_value('Cessna'),
                 _plugin_command('Keyboard'), _plugin_value('Keyboard'),
                 '-s', 'libgazebo_ros_init.so',
                 '-s', 'libgazebo_ros_factory.so',
                 # Wait for (https://github.com/ros-simulation/gazebo_ros_pkgs/pull/941)
                 # '-s', 'libgazebo_ros_effort.so',
                 ],
            output='screen',
            additional_env=env,
        )
    ])


def _command(arg):
    cmd = ['"--', arg, '" if "true" == "', LaunchConfiguration(arg), '" else ""']
    py_cmd = PythonExpression(cmd)
    return py_cmd


def _plugin_command(arg):
    cmd = ['"--gui-client-plugin" if "true" == "', LaunchConfiguration(arg), '" else ""']
    py_cmd = PythonExpression(cmd)
    return py_cmd


def _plugin_value(arg):
    cmd = ['"lib', arg, 'GUIPlugin.so" if "true" == "', LaunchConfiguration(arg), '" else ""']
    py_cmd = PythonExpression(cmd)
    return py_cmd
