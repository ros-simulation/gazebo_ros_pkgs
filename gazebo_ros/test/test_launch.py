# Copyright 2022 Open Source Robotics Foundation, Inc.
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

import os

from ament_index_python.packages import get_package_share_path
import launch
from launch.actions.include_launch_description import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_pytest
import psutil
import pytest


@pytest.fixture
def gzserver_proc():
    return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_path('gazebo_ros'), 'launch', 'gzserver.launch.py')
            ))


@pytest.fixture
def gzclient_proc():
    return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_path('gazebo_ros'), 'launch', 'gzclient.launch.py')
            ))


@pytest.fixture
def gazebo_ros_proc():
    return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_path('gazebo_ros'), 'launch', 'gazebo.launch.py')
            ))


@launch_pytest.fixture()
def launch_gzserver(gzserver_proc):
    return launch.LaunchDescription([
        gzserver_proc,
        launch_pytest.actions.ReadyToTest()
    ])


@launch_pytest.fixture()
def launch_gzclient(gzclient_proc):
    return launch.LaunchDescription([
        gzclient_proc,
        launch_pytest.actions.ReadyToTest()
    ])


@launch_pytest.fixture()
def launch_gazebo(gazebo_ros_proc):
    return launch.LaunchDescription([
        gazebo_ros_proc,
        launch_pytest.actions.ReadyToTest()
    ])


@pytest.mark.launch(fixture=launch_gzserver)
def test_launch_gzserver(gzserver_proc, launch_context):
    current_process = psutil.Process()
    children = current_process.children(recursive=True)
    assert 'gzserver' in [i.name() for i in children], 'Error; gzserver not launched'


@pytest.mark.launch(fixture=launch_gzclient)
def test_launch_gzclient(gzclient_proc, launch_context):
    current_process = psutil.Process()
    children = current_process.children(recursive=True)
    assert 'gzclient' in [i.name() for i in children], 'Error; gzclient not launched'


@pytest.mark.launch(fixture=launch_gazebo)
def test_launch_gazebo_ros(gzclient_proc, launch_context):
    current_process = psutil.Process()
    children = current_process.children(recursive=True)
    names = [i.name() for i in children]
    assert 'gzclient' in names and 'gzserver' in names, 'Error; gazebo not launched'
