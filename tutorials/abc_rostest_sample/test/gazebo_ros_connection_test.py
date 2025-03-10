#!/usr/bin/env python


import sys
import unittest

import rospy
import rostest
from std_msgs.msg import Int32

PKG = "abc_rostest_sample"
NAME = "abc_gazebo_plugin_test"


class TestABCPlugin(unittest.TestCase):
    """Test class for ABC plugin

    Args:
    ----
        unittest (unittest.TestCase): Base class for unit tests

    """

    def __init__(self, *args):
        """Constructor"""
        super(TestABCPlugin, self).__init__(*args)

    def test_gazebo_plugin(self):
        """Test for abc gazebo plugin"""
        rospy.init_node(NAME, anonymous=True)
        # Wait 3 seconds before starting test
        START_TIME = 3
        TIMEOUT = 5
        rospy.sleep(START_TIME)
        # Set 5 minutes timeout from Gazebo's published time
        msg = rospy.wait_for_message("secs", Int32)
        timeout_t = rospy.Time.from_sec(msg.data) + rospy.Duration(secs=TIMEOUT)
        while not rospy.is_shutdown() and rospy.Time.now() < timeout_t:
            rospy.sleep(0.5)
        msg = rospy.wait_for_message("secs", Int32)
        self.assertGreaterEqual(START_TIME + TIMEOUT, msg.data)


if __name__ == "__main__":
    rostest.rosrun(PKG, NAME, TestABCPlugin, sys.argv)
