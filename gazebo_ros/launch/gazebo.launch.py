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

"""
Launch Gazebo with an empty world and initialize ROS with command line arguments.

Search for model, plugin and media paths exported by packages and add them to
Gazebo environment variables.

e.g.  <export>
          <gazebo_ros gazebo_model_path="${prefix}/models"/>
          <gazebo_ros gazebo_media_path="${prefix}/models"/>
      </export>

${prefix} is replaced by package's share directory in install.
Thus the required directory needs to be installed from CMakeLists.txt

e.g.  install(DIRECTORY models
          DESTINATION share/${PROJECT_NAME})
"""

import os

from ament_index_python.packages import get_packages_with_prefixes
from catkin_pkg.package import InvalidPackage, PACKAGE_MANIFEST_FILENAME, parse_package

import launch


def add_gazebo_ros_paths():
    gazebo_model_path = []
    gazebo_plugin_path = []
    gazebo_media_path = []

    for package_name, package_path in get_packages_with_prefixes().items():
        package_share_path = os.path.join(package_path, 'share', package_name)
        package_file_path = os.path.join(package_share_path, PACKAGE_MANIFEST_FILENAME)
        if os.path.isfile(package_file_path):
            # only try to import catkin if a PACKAGE_FILE is found
            try:
                package = parse_package(package_file_path)
            except InvalidPackage:
                continue
            for export in package.exports:
                if export.tagname == 'gazebo_ros':
                    if 'gazebo_model_path' in export.attributes:
                        plugin_xml_path = export.attributes['gazebo_model_path']
                        plugin_xml_path = plugin_xml_path.replace('${prefix}', package_share_path)
                        gazebo_model_path.append(plugin_xml_path)
                    if 'plugin_path' in export.attributes:
                        plugin_xml_path = export.attributes['plugin_path']
                        plugin_xml_path = plugin_xml_path.replace('${prefix}', package_share_path)
                        gazebo_plugin_path.append(plugin_xml_path)
                    if 'gazebo_media_path' in export.attributes:
                        plugin_xml_path = export.attributes['gazebo_media_path']
                        plugin_xml_path = plugin_xml_path.replace('${prefix}', package_share_path)
                        gazebo_media_path.appedn(plugin_xml_path)

    os.environ['GAZEBO_MODEL_PATH'] = ':'.join(gazebo_model_path)
    os.environ['GAZEBO_PLUGIN_PATH'] = ':'.join(gazebo_plugin_path)
    os.environ['GAZEBO_RESOURCE_PATH'] = ':'.join(gazebo_media_path)


def generate_launch_description():
    add_gazebo_ros_paths()

    gazebo = launch.actions.ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so'],
        output='screen'
    )

    return launch.LaunchDescription([
        gazebo
    ])
