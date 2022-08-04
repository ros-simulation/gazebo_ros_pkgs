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
from os import pathsep

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import OpaqueFunction
from launch.actions import Shutdown
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch.utilities import normalize_to_list_of_substitutions

from scripts import GazeboRosPaths


def generate_launch_description():
    model, plugin, media = GazeboRosPaths.get_paths()

    if 'GAZEBO_MODEL_PATH' in environ:
        model += pathsep+environ['GAZEBO_MODEL_PATH']
    if 'GAZEBO_PLUGIN_PATH' in environ:
        plugin += pathsep+environ['GAZEBO_PLUGIN_PATH']
    if 'GAZEBO_RESOURCE_PATH' in environ:
        media += pathsep+environ['GAZEBO_RESOURCE_PATH']

    env = {
        'GAZEBO_MODEL_PATH': model,
        'GAZEBO_PLUGIN_PATH': plugin,
        'GAZEBO_RESOURCE_PATH': media
    }

    prefix = PythonExpression([
        '"gdb -ex run --args" if "true" == "',
        LaunchConfiguration('gdb'),
        '" else "valgrind" if "true" == "',
        LaunchConfiguration('valgrind'),
        '" else ""'
    ])

    # Function that returns the ExecuteProcess command for gzserver
    def execute_command(context, command):
        # Workaround for https://github.com/ros2/launch/issues/638
        # Filter out any empty strings from the argument list
        normalized_command = normalize_to_list_of_substitutions(command)
        command_post_substitution = [
            context.perform_substitution(command_part)
            for command_part in normalized_command
        ]

        # Check if we should shutdown if server_required is true and gzserver stops
        # Could be simplified with https://github.com/ros2/launch/issues/290
        server_required = LaunchConfiguration('server_required').perform(context)
        on_exit_action = Shutdown() if server_required.lower() == 'true' else None

        return [
            ExecuteProcess(
                cmd=list(filter(None, command_post_substitution)),
                output='screen',
                additional_env=env,
                shell=False,
                prefix=prefix,
                on_exit=on_exit_action,
            )
        ]

    gzserver_command = [
        'gzserver',
        # Pass through arguments to gzserver
        LaunchConfiguration('world'),
        _boolean_command('version'),
        _boolean_command('verbose'),
        _boolean_command('lockstep'),
        _boolean_command('help'),
        _boolean_command('pause'),
        _arg_command('physics'), LaunchConfiguration('physics'),
        _arg_command('play'), LaunchConfiguration('play'),
        _boolean_command('record'),
        _arg_command('record_encoding'), LaunchConfiguration('record_encoding'),
        _arg_command('record_path'), LaunchConfiguration('record_path'),
        _arg_command('record_period'), LaunchConfiguration('record_period'),
        _arg_command('record_filter'), LaunchConfiguration('record_filter'),
        _arg_command('seed'), LaunchConfiguration('seed'),
        _arg_command('iters'), LaunchConfiguration('iters'),
        _boolean_command('minimal_comms'),
        _plugin_command('init'),
        _plugin_command('factory'),
        _plugin_command('force_system'),
        # Wait for (https://github.com/ros-simulation/gazebo_ros_pkgs/pull/941)
        # _plugin_command('force_system'), ' ',
        _arg_command('profile'), LaunchConfiguration('profile'),
        # convenience parameter for params file
        _arg_command('ros-args', condition=LaunchConfiguration('params_file')),
        _arg_command('params-file', condition=LaunchConfiguration('params_file')),
        LaunchConfiguration('params_file'),
        _arg_command('', condition=LaunchConfiguration('params_file')),
        LaunchConfiguration('extra_gazebo_args'),
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'world', default_value='',
            description='Specify world file name'
        ),
        DeclareLaunchArgument(
            'version', default_value='false',
            description='Set "true" to output version information.'
        ),
        DeclareLaunchArgument(
            'verbose', default_value='false',
            description='Set "true" to increase messages written to terminal.'
        ),
        DeclareLaunchArgument(
            'lockstep', default_value='false',
            description='Set "true" to respect update rates'
        ),
        DeclareLaunchArgument(
            'help', default_value='false',
            description='Set "true" to produce gzserver help message.'
        ),
        DeclareLaunchArgument(
            'pause', default_value='false',
            description='Set "true" to start the server in a paused state.'
        ),
        DeclareLaunchArgument(
            'physics', default_value='',
            description='Specify a physics engine (ode|bullet|dart|simbody).'
        ),
        DeclareLaunchArgument(
            'play', default_value='',
            description='Play the specified log file.'
        ),
        DeclareLaunchArgument(
            'record', default_value='false',
            description='Set "true" to record state data.'
        ),
        DeclareLaunchArgument(
            'record_encoding', default_value='',
            description='Specify compression encoding format for log data (zlib|bz2|txt).'
        ),
        DeclareLaunchArgument(
            'record_path', default_value='',
            description='Absolute path in which to store state data.'
        ),
        DeclareLaunchArgument(
            'record_period', default_value='',
            description='Specify recording period (seconds).'
        ),
        DeclareLaunchArgument(
            'record_filter', default_value='',
            description='Specify recording filter (supports wildcard and regular expression).'
        ),
        DeclareLaunchArgument(
            'seed', default_value='', description='Start with a given a random number seed.'
        ),
        DeclareLaunchArgument(
            'iters', default_value='', description='Specify number of iterations to simulate.'
        ),
        DeclareLaunchArgument(
            'minimal_comms', default_value='false',
            description='Set "true" to reduce TCP/IP traffic output.'
        ),
        DeclareLaunchArgument(
            'profile', default_value='',
            description='Specify physics preset profile name from the options in the world file.'
        ),
        DeclareLaunchArgument(
            'extra_gazebo_args', default_value='',
            description='Extra arguments to be passed to Gazebo'
        ),
        DeclareLaunchArgument(
            'params_file', default_value='',
            description='Path to ROS 2 yaml parameter file'
        ),

        # Specific to gazebo_ros
        DeclareLaunchArgument(
            'gdb', default_value='false',
            description='Set "true" to run gzserver with gdb'
        ),
        DeclareLaunchArgument(
            'valgrind', default_value='false',
            description='Set "true" to run gzserver with valgrind'
        ),
        DeclareLaunchArgument(
            'init', default_value='true',
            description='Set "false" not to load "libgazebo_ros_init.so"'
        ),
        DeclareLaunchArgument(
            'factory', default_value='true',
            description='Set "false" not to load "libgazebo_ros_factory.so"'
        ),
        DeclareLaunchArgument(
            'force_system', default_value='true',
            description='Set "false" not to load "libgazebo_ros_force_system.so"'
        ),
        DeclareLaunchArgument(
            'server_required', default_value='false',
            description='Set "true" to shut down launch script when server is terminated'
        ),
        # Start gzserver
        OpaqueFunction(function=execute_command, args=[gzserver_command])

    ])


# Add boolean commands if true
def _boolean_command(arg):
    cmd = ['"--', arg, '" if "true" == "', LaunchConfiguration(arg), '" else ""']
    py_cmd = PythonExpression(cmd)
    return py_cmd


# Add string commands if not empty
def _arg_command(arg, condition=None):
    if condition:
        cmd = ['"--', arg, '" if "" != "', condition, '" else ""']
    else:
        cmd = ['"--', arg, '" if "" != "', LaunchConfiguration(arg), '" else ""']
    py_cmd = PythonExpression(cmd)
    return py_cmd


# Add gazebo_ros plugins if true
def _plugin_command(arg):
    cmd = ['"-s', 'libgazebo_ros_', arg, '.so" if "true" == "',
           LaunchConfiguration(arg), '" else ""']
    py_cmd = PythonExpression(cmd)
    return py_cmd
