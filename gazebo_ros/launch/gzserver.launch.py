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

"""Launch a Gazebo server with command line arguments."""

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
        DeclareLaunchArgument('world', default_value='worlds/empty.world',
                              description='Specify world file name'),
        DeclareLaunchArgument('version', default_value='false',
                              description='Set "true" to output version information.'),
        DeclareLaunchArgument('verbose', default_value='false',
                              description='Set "true" to increase messages written to terminal.'),
        DeclareLaunchArgument('help', default_value='false',
                              description='Set "true" to produce gzserver help message.'),
        DeclareLaunchArgument('pause', default_value='false',
                              description='Set "true" to start the server in a paused state.'),
        DeclareLaunchArgument('physics', default_value='',
                              description='Specify a physics engine (ode|bullet|dart|simbody).'),
        DeclareLaunchArgument('play', default_value='',
                              description='Play the specified log file.'),
        DeclareLaunchArgument('record', default_value='false',
                              description='Set "true" to record state data.'),
        DeclareLaunchArgument('record_encoding', default_value='',
                              description='Specify compression encoding format for log data '
                                          '(zlib|bz2|txt).'),
        DeclareLaunchArgument('record_path', default_value='',
                              description='Absolute path in which to store state data.'),
        DeclareLaunchArgument('record_period', default_value='',
                              description='Specify recording period (seconds).'),
        DeclareLaunchArgument('record_filter', default_value='',
                              description='Specify recording filter '
                                          '(supports wildcard and regular expression).'),
        DeclareLaunchArgument('seed', default_value='',
                              description='Set "true" to start with a random number seed.'),
        DeclareLaunchArgument('iters', default_value='',
                              description='Specify number of iterations to simulate.'),
        DeclareLaunchArgument('minimal_comms', default_value='false',
                              description='Set "true" to reduce TCP/IP traffic output.'),
        DeclareLaunchArgument('profile', default_value='',
                              description='Specify physics preset profile name from the options '
                                          'in the world file.'),
        DeclareLaunchArgument('gdb', default_value='false',
                              description='Set "true" to run gzserver with gdb'),
        DeclareLaunchArgument('valgrind', default_value='false',
                              description='Set "true" to run gzserver with valgrind'),

        ExecuteProcess(
            cmd=['gzserver',
                 LaunchConfiguration('world'),
                 _command('version'),
                 _command('verbose'),
                 _command('help'),
                 _command('pause'),
                 _arg_command('physics'), LaunchConfiguration('physics'),
                 _arg_command('play'), LaunchConfiguration('play'),
                 _command('record'),
                 _arg_command('record_encoding'), LaunchConfiguration('record_encoding'),
                 _arg_command('record_path'), LaunchConfiguration('record_path'),
                 _arg_command('record_period'), LaunchConfiguration('record_period'),
                 _arg_command('record_filter'), LaunchConfiguration('record_filter'),
                 _arg_command('seed'), LaunchConfiguration('seed'),
                 _arg_command('iters'), LaunchConfiguration('iters'),
                 _command('minimal_comms'),
                 '-s', 'libgazebo_ros_init.so',
                 '-s', 'libgazebo_ros_factory.so',
                 # Wait for (https://github.com/ros-simulation/gazebo_ros_pkgs/pull/941)
                 # '-s', 'libgazebo_ros_effort.so',
                 _arg_command('profile'), LaunchConfiguration('profile'),
                 ],
            output='screen',
            additional_env=env,
            prefix=PythonExpression(['"gdb -ex run --args" if "true" == "',
                                     LaunchConfiguration('gdb'),
                                     '" else "valgrind" if "true" == "',
                                     LaunchConfiguration('valgrind'),
                                     '" else ""']),
        )
    ])


def _command(arg):
    cmd = ['"--', arg, '" if "true" == "', LaunchConfiguration(arg), '" else ""']
    py_cmd = PythonExpression(cmd)
    return py_cmd


def _arg_command(arg):
    cmd = ['"--', arg, '" if "" != "', LaunchConfiguration(arg), '" else ""']
    py_cmd = PythonExpression(cmd)
    return py_cmd
