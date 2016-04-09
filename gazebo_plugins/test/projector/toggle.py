#!/usr/bin/env python

import rospy
import unittest

from std_msgs.msg import Int32

class TestProjectorToggle(unittest.TestCase):
    def setUp(self):
        rospy.init_node('toggle')
    def test_projector_toggle(self):
        pub = rospy.Publisher("/projector_plugin/projector", Int32, queue_size=5)
        pub2 = rospy.Publisher("/projector_plugin2/projector", Int32, queue_size=5)
        toggle = True
        start_time = None
        while not rospy.is_shutdown():
            for i in range(3):
                pub.publish(Int32(int(toggle)))
                pub2.publish(Int32(int(not toggle)))
                toggle = not toggle
            rospy.sleep(0.5)
            cur_time = rospy.Time.now().to_sec()
            if start_time is None:
                start_time = cur_time
            if cur_time - start_time > 10:
                break
            # print cur_time - start_time, start_time, cur_time

if __name__ == '__main__':
    import rostest
    rostest.rosrun('gazebo_plugins', 'test_projector_toggle', TestProjectorToggle)
