#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Range

import unittest

class TestRangePlugin(unittest.TestCase):

  def test_max_range(self):
    msg = rospy.wait_for_message('/sonar2', Range)
    self.assertAlmostEqual(msg.range, msg.max_range)

  def test_inside_range(self):
    msg = rospy.wait_for_message('/sonar', Range)
    # Nominal value is 0.225
    # TODO(lucasw) could have this average over 10 samples or so
    min_range = rospy.get_param("~min_range", 0.20)
    max_range = rospy.get_param("~max_range", 0.25)
    self.assertGreater(msg.range, min_range, 'range too small: {} < {}'.format(msg.range, min_range))
    self.assertLess(msg.range, max_range, 'range too big: {} > {}'.format(msg.range, max_range))

if __name__ == '__main__':
  import rostest
  PKG_NAME = 'gazebo_plugins'
  TEST_NAME = PKG_NAME + 'range_test'
  rospy.init_node(TEST_NAME)
  rostest.rosrun(PKG_NAME, TEST_NAME, TestRangePlugin)
