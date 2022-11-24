#!/usr/bin/env python3
#
# Copyright 2022 Open Source Robotics Foundation
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
# Desc: helper script for publishing drawbar pull wrenches
# Author: Steve Peters
#
import argparse
import math
import rclpy
import sys
from geometry_msgs.msg import Wrench
from rclpy.node import Node

DEFAULT_FORCE_INCREMENT = 10
DEFAULT_MAX_FORCE = 70
DEFAULT_PUBLISH_INTERVAL = 2.0


class WheelSlipDrawbarPullPublisher(Node):

    def __init__(self, args):
        super().__init__('wheel_slip_drawbar_pull_publisher')
        parser = argparse.ArgumentParser(
            description='Publish a wrench representing drawbar pull on the back of a vehicle.')
        parser.add_argument('-f', '--force-increment', type=float, default=DEFAULT_FORCE_INCREMENT,
                            help='The drawbar pull force increment (Newtons)')
        parser.add_argument('-i', '--interval', type=float, default=DEFAULT_PUBLISH_INTERVAL,
                            help='The drawar pull publication interval (seconds)')
        parser.add_argument('-m', '--max-force', type=float, default=DEFAULT_MAX_FORCE,
                            help='The maximum drawbar pull force')
        self.args = parser.parse_args(args[1:])

        self.publisher = self.create_publisher(Wrench, "drawbar_pull", 1)
        self.publish_counter = 1
        self.publish_timer = self.create_timer(self.args.interval, self.update_wrench)

    def update_wrench(self):
        i = self.publish_counter
        self.publish_counter += 1
        amplitude = self.args.force_increment * math.floor(i / 2)
        sign = (-1) ** (i % 2)
        if amplitude > self.args.max_force:
            amplitude = 0

        msg = Wrench()
        msg.force.x = sign * float(amplitude)

        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing drawbar pull force of {msg.force.x} N')


def main(args=sys.argv):
    rclpy.init(args=args)
    args_without_ros = rclpy.utilities.remove_ros_args(args)
    node = WheelSlipDrawbarPullPublisher(args_without_ros)
    node.get_logger().info('Wheel Slip Drawbar Pull Publisher started')

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
