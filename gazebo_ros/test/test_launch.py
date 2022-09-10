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
from threading import Event

from ament_index_python.packages import get_package_share_path
import launch
from launch.actions import IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.actions import SetLaunchConfiguration
from launch.event_handlers import OnProcessIO
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


@pytest.fixture()
def launch_capture_io():

    class LaunchCaptureIO:

        def __init__(self):
            self._event = Event()
            self.stderr = []
            self.action = RegisterEventHandler(OnProcessIO(on_stderr=self._on_stderr))

        def _on_stderr(self, io):
            self.stderr.append(io.text.decode('utf-8'))
            self._event.set()

        def wait_for_stderr(self, *, timeout) -> bool:
            result = self._event.wait(timeout)
            self._event.clear()
            return result

    return LaunchCaptureIO()


@launch_pytest.fixture()
def launch_gzserver_with_args(gzserver_proc, launch_capture_io):
    return launch.LaunchDescription([
        launch_capture_io.action,
        SetLaunchConfiguration('physics', 'ode'),
        SetLaunchConfiguration('verbose', 'true'),
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


@pytest.mark.launch(fixture=launch_gzserver_with_args)
def test_launch_gzserver_with_args(gzserver_proc, launch_capture_io, launch_context):
    current_process = psutil.Process()
    children = current_process.children(recursive=True)
    assert 'gzserver' in [i.name() for i in children], 'Error; gzserver not launched'

    assert launch_capture_io.wait_for_stderr(timeout=5.0), \
        'Expected output on stderr, received none'
    assert 'Gazebo multi-robot simulator, version 11.11.0\nCopyright (C) 2012 Open Source ' \
           'Robotics Foundation.\nReleased under the Apache 2 License.\nhttp://gazebosim.org\n\n' \
           in launch_capture_io.stderr
    assert not launch_capture_io.wait_for_stderr(timeout=1.0), \
        f'Expected no more output on stderr, got {launch_capture_io.stderr}'


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
