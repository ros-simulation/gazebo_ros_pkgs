#!/usr/bin/env python3
#
# Copyright 2019 Open Source Robotics Foundation
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
#
# Desc: helper script for spawning models in gazebo
# Author: John Hsu, Dave Coleman
#
import argparse
import math
import os
import sys
from urllib.parse import SplitResult, urlsplit
from xml.etree import ElementTree

from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import DeleteEntity
from gazebo_msgs.srv import SetModelConfiguration
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose
import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty


class SpawnModelNode(Node):
    # Node to spawn a model in Gazebo using the Gazebo ROS Factory
    MODEL_DATABASE_TEMPLATE = """\
    <sdf version="1.6">
        <world name="default">
            <include>
                <uri>model://{}</uri>
            </include>
        </world>
    </sdf>"""

    def __init__(self, args):
        super().__init__('spawn_model')
        parser = argparse.ArgumentParser(
            description='Spawn a model in gazebo using the Gazebo ROS Factory')
        source = parser.add_mutually_exclusive_group(required=True)
        source.add_argument(
            '-file', type=str, metavar='FILE_NAME', help='Load model xml from file')
        source.add_argument(
            '-param', type=str, metavar='PARAM_NAME', help='Load model xml from ROS parameter')
        source.add_argument('-database', type=str, metavar='MODEL_NAME',
                            help='Load model XML from specified model in Gazebo Model Database')
        source.add_argument('-stdin', action='store_true', help='Load model from stdin')
        parser.add_argument(
            '-model', required=True, type=str, metavar='MODEL_NAME', help='Name of model to spawn')
        parser.add_argument('-reference_frame', type=str, default='',
                            help='Name of the model/body where initial pose is defined.\
                            If left empty or specified as "world", gazebo world frame is used')
        parser.add_argument('-gazebo_namespace', type=str, default='/gazebo',
                            help='ROS namespace of gazebo offered ROS interfaces. \
                            Defaults to /gazebo/')
        parser.add_argument('-robot_namespace', type=str, default=self.get_namespace(),
                            help='change ROS namespace of gazebo-plugins')
        parser.add_argument('-unpause', action='store_true',
                            help='unpause physics after spawning model')
        parser.add_argument('-wait', type=str, metavar='MODEL_NAME',
                            help='Wait for model to exist')
        parser.add_argument('-x', type=float, default=0,
                            help='x component of initial position, meters')
        parser.add_argument('-y', type=float, default=0,
                            help='y component of initial position, meters')
        parser.add_argument('-z', type=float, default=0,
                            help='z component of initial position, meters')
        parser.add_argument('-R', type=float, default=0,
                            help='roll angle of initial orientation, radians')
        parser.add_argument('-P', type=float, default=0,
                            help='pitch angle of initial orientation, radians')
        parser.add_argument('-Y', type=float, default=0,
                            help='yaw angle of initial orientation, radians')

        # TODO(shivesh): Wait for /set_model_configuration
        # (https://github.com/ros-simulation/gazebo_ros_pkgs/issues/779)
        # parser.add_argument('-J', dest='joints', default=[], action='append',
        #     metavar=('JOINT_NAME', 'JOINT_POSITION'), type=str, nargs=2,
        #     help='initialize the specified joint at the specified position')

        parser.add_argument('-package_to_model', action='store_true', help='convert urdf \
                            <mesh filename="package://..." to <mesh filename="model://..."')

        # TODO(shivesh): Wait for https://github.com/ros2/rclpy/issues/244
        # parser.add_argument('-b', dest='bond', action='store_true', help='bond to gazebo \
        #                      and delete the model when this program is interrupted')
        self.args = parser.parse_args(args[1:])

        # TODO(shivesh): Wait for /set_model_configuration
        # (https://github.com/ros-simulation/gazebo_ros_pkgs/issues/779)
        # Convert position of joints to floats
        # for i in range(len(self.args.joints)):
        #     self.args.joints[i][1] = float(self.args.joints[i][1])

    def run(self):
        """
        Run node, spawning model and doing other actions as configured in program arguments.

        Returns exit code, 1 for failure, 0 for success
        """
        # Wait for model to exist if wait flag is enabled
        if self.args.wait:
            self.model_exists = False

            def models_cb(models):
                self.model_exists = self.args.wait in models.name

            self.subscription = self.create_subscription(
                ModelStates, '%s/model_states' % self.args.gazebo_namespace, models_cb, 10)
            self.get_logger().info(
                'Waiting for model {} before proceeding.'.format(self.args.wait))

            while rclpy.ok() and not self.model_exists:
                pass

        # Load model XML from file
        if self.args.file:
            self.get_logger().info('Loading model XML from file %s' % self.args.file)
            if not os.path.exists(self.args.file):
                self.get_logger().error('Error: specified file %s does not exist', self.args.file)
                return 1
            if not os.path.isfile(self.args.file):
                self.get_logger().error('Error: specified file %s is not a file', self.args.file)
                return 1
            # load file
            try:
                f = open(self.args.file, 'r')
                model_xml = f.read()
            except IOError as e:
                self.get_logger().error('Error reading file {}: {}'.format(self.args.file, e))
                return 1
            if model_xml == '':
                self.get_logger().error('Error: file %s is empty', self.args.file)
                return 1
        # Load model XML from ROS param
        elif self.args.param:
            self.get_logger().info('Loading model XML from ros parameter %s' % self.args.param)
            model_xml = self.get_parameter(self.args.param)
            if model_xml == '':
                self.get_logger().error('Error: param does not exist or is empty')
                return 1
        # Generate model XML by putting requested model name into request template
        elif self.args.database:
            self.get_logger().info('Loading model XML from Gazebo Model Database')
            model_xml = self.MODEL_DATABASE_TEMPLATE.format(self.args.database)
        elif self.args.stdin:
            self.get_logger().info('Loading model XML from stdin')
            model_xml = sys.stdin.read()
            if model_xml == '':
                self.get_logger().error('Error: stdin buffer was empty')
                return 1

        # Parse xml to detect invalid xml before sending to gazebo
        try:
            xml_parsed = ElementTree.fromstring(model_xml)
        except ElementTree.ParseError as e:
            self.get_logger().error('Invalid XML: {}'.format(e))
            return 1

        # Replace package:// with model:// for mesh tags if flag is set
        if self.args.package_to_model:
            for element in xml_parsed.iterfind('.//mesh'):
                filename_tag = element.get('filename')
                if filename_tag is None:
                    continue
                url = urlsplit(filename_tag)
                if url.scheme == 'package':
                    url = SplitResult('model', *url[1:])
                    element.set('filename', url.geturl())

        # Encode xml object back into string for service call
        model_xml = ElementTree.tostring(xml_parsed)

        # Form requested Pose from arguments
        initial_pose = Pose()
        initial_pose.position.x = float(self.args.x)
        initial_pose.position.y = float(self.args.y)
        initial_pose.position.z = float(self.args.z)

        q = quaternion_from_euler(self.args.R, self.args.P, self.args.Y)
        initial_pose.orientation.w = q[0]
        initial_pose.orientation.x = q[1]
        initial_pose.orientation.y = q[2]
        initial_pose.orientation.z = q[3]

        # Spawn model using urdf or sdf service based on arguments
        success = self._spawn_entity(model_xml, initial_pose)
        if not success:
            self.get_logger().error('Spawn service failed. Exiting.')
            return 1

        # TODO(shivesh): Wait for /set_model_configuration
        # (https://github.com/ros-simulation/gazebo_ros_pkgs/issues/779)
        # Apply joint positions if any specified
        # if len(self.args.joints) != 0:
        #     joint_names = [joint[0] for joint in self.args.joints]
        #     joint_positions = [joint[1] for joint in self.args.joints]
        #     success = _set_model_configuration(joint_names, joint_positions)
        #     if not success:
        #         self.get_logger().error('SetModelConfiguration service failed. Exiting.')
        #         return 1

        # Unpause physics if user requested
        if self.args.unpause:
            client = self.create_client(Empty, '/unpause_physics')
            client.wait_for_service(timeout_sec=5)
            self.get_logger().info('Calling service /unpause_physics')
            client.call_async(Empty.Request())

        # TODO(shivesh): Wait for https://github.com/ros2/rclpy/issues/244
        # If bond enabled, setup shutdown callback and wait for shutdown
        # if self.args.bond:
        #     rospy.on_shutdown(self._delete_entity)
        #     rospy.loginfo('Waiting for shutdown to delete model {}'.format(self.args.model))
        #     rospy.spin()

        return 0

    def _spawn_entity(self, model_xml, initial_pose):
        self.get_logger().info('Waiting for service %s/spawn_entity' % self.args.gazebo_namespace)
        client = self.create_client(SpawnEntity, 'spawn_entity')
        client.wait_for_service(timeout_sec=5.0)
        req = SpawnEntity.Request()
        req.name = self.args.model
        req.xml = str(model_xml, 'utf-8')
        req.robot_namespace = self.args.robot_namespace
        req.initial_pose = initial_pose
        req.reference_frame = self.args.reference_frame
        self.get_logger().info('Calling service %s/spawn_entity' % self.args.gazebo_namespace)
        srv_call = client.call_async(req)
        while rclpy.ok():
            if srv_call.done():
                self.get_logger().info('Spawn status: %s' % srv_call.result().status_message)
                break
            rclpy.spin_once(self)
        return srv_call.result().success

    def _delete_entity(self):
        # Delete model from gazebo on shutdown if bond flag enabled
        self.get_logger().info('Deleting model {}'.format(self.args.model))
        client = self.create_client(DeleteEntity, '%s/delete_entity' % self.args.gazebo_namespace)
        client.wait_for_service(timeout_sec=5)
        req = DeleteEntity.Request()
        req.name = self.args.model
        self.get_logger().info('Calling service %s/delete_entity' % self.args.gazebo_namespace)
        srv_call = client.call_async(req)
        while rclpy.ok():
            if srv_call.done():
                self.get_logger().info('Deleting status: %s' % srv_call.result().status_message)
                break
            rclpy.spin_once(self)

    def _set_model_configuration(self, joint_names, joint_positions):
        self.get_logger().info(
            'Waiting for service %s/set_model_configuration' % self.args.gazebo_namespace)
        client = self.create_client(SetModelConfiguration, 'set_model_configuration')
        client.wait_for_service(timeout_sec=5)
        req = SetModelConfiguration.Request()
        req.model_name = self.args.model
        req.urdf_param_name = ''
        req.joint_names = joint_names
        req.joint_positions = joint_positions
        self.get_logger().info(
            'Calling service %s/set_model_configuration' % self.args.gazebo_namespace)
        srv_call = client.call_async(req)
        while rclpy.ok():
            if srv_call.done():
                self.get_logger().info(
                    'Set model configuration status: %s' % srv_call.result().status_message)
                break
            rclpy.spin_once(self)
        return srv_call.result().success


def quaternion_from_euler(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = [0] * 4
    q[0] = cy * cp * cr + sy * sp * sr
    q[1] = cy * cp * sr - sy * sp * cr
    q[2] = sy * cp * sr + cy * sp * cr
    q[3] = sy * cp * cr - cy * sp * sr

    return q


def main(args=sys.argv):
    rclpy.init(args=args)
    spawn_model_node = SpawnModelNode(args)
    exit_code = spawn_model_node.run()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
