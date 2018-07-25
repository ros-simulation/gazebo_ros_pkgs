# Copyright 2018 Open Source Robotics Foundation, Inc.
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

"""Launch a Gazebo server with an empty world and initialize ROS with command line arguments."""

import launch


def generate_launch_description():
    # TODO(anyone): Forward command line arguments once that's supported, see
    # https://github.com/ros2/launch/issues/107
    gzserver = launch.actions.ExecuteProcess(
        cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_init.so'],
        output='screen'
    )

    return launch.LaunchDescription([
        gzserver
    ])
