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

"""Launch Gazebo server and client with command line arguments."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir

from ament_index_python.packages import get_package_share_directory

import em
import os


def gzserver_with_params(context, *args, **kwargs):
    rate = LaunchConfiguration('publish_rate').perform(context)
    params_yaml_in = os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'params.yaml.in')
    interpreter = em.Interpreter(output=open('/tmp/gazebo_params.yaml', 'w'))
    interpreter.string("@{publish_rate = %f}" % float(rate))
    interpreter.file(open(params_yaml_in))
    interpreter.shutdown()
    return [
        IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/gzserver.launch.py']),
        condition=IfCondition(LaunchConfiguration('server')),
        launch_arguments={
            'extra_gazebo_args': '--ros-args --params-file /tmp/gazebo_params.yaml'
        }.items()
    )]


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true',
                              description='Set to "false" to run headless.'),

        DeclareLaunchArgument('server', default_value='true',
                              description='Set to "false" not to run gzserver.'),

        DeclareLaunchArgument('publish_rate', default_value='10',
                              description='Rate (in Hz) at which the clock publishes.'),

        OpaqueFunction(function=gzserver_with_params),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/gzclient.launch.py']),
            condition=IfCondition(LaunchConfiguration('gui'))
        ),
    ])
