#!/usr/bin/env python
'''
Created Sept 2016
@author: Ike Dean 

 odometriclocalizer.py for edbot robot's differential drive base
 this ros node publishs odometry messages so ros navigation stack
 can localize the robot on a static map of the world.

'''

import roslib
import rospy
import tf
import math
from math import sin, cos, pi
import sys

from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class edbotOdom(object):

    def Start(self):
        rospy.logdebug("Starting")

    def Stop(self):
        rospy.logdebug("Stopping")

    def update(self):
        now = rospy.get_rostime()
        rospy.loginfo("Current time %i %i", now.secs, now.nsecs)

    def __init__(self):

        rospy.init_node('edbotodom')
        rospy.loginfo("edbotodom node init")

        # subscriptions and publications
	self._OdometryTransformBroadcaster = tf.TransformBroadcaster()
	self._OdometryPublisher = rospy.Publisher("odom", Odometry, queue_size=10)

if __name__ == '__main__':
    odometry = edbotOdom()
    try:
        odometry.Start()
	r = rospy.Rate(1)
	while not rospy.is_shutdown():
		odometry.update()
		r.sleep()

    except rospy.ROSInterruptException:
        odometry.Stop()
