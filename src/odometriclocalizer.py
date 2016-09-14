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

import edspi53
import odom53

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
        self._t1 = self._t0                   # previous t1 = current
        self._t0 = self._spi.rdEncoders()     # current  t0 = new sample  ---present state--
        self._odom.update(self._t0, self._t1) # calc odometry = f(t0,t1, [X Y Heading V Omega])
        print "OUT: X: %f Y: %f Heading: %f V: %f Omega: %f" % (self._odom.X, self._odom.Y, self._odom.Heading, self._odom.V, self._odom.Omega)

    def __init__(self):

        rospy.init_node('edbotodom')
        rospy.loginfo("edbotodom node init")

        # subscriptions and publications
	self._OdometryTransformBroadcaster = tf.TransformBroadcaster()
	self._OdometryPublisher = rospy.Publisher("odom", Odometry, queue_size=10)

        # setup storage for current(t0) and last(t1) periodic samples
        self._t0 = edspi53.MotorEncoders()    # of time stamped wheel encoder data
        self._t1 = edspi53.MotorEncoders()
        self._spi = edspi53.Spi()   # instance Spi() class to read samples of 
        self._spi.start()           # time stamped wheel encoder data using libspi52.so
        self._odom = odom53.Odom()  # instance Odom() class to compute odometry

        self._t0 = self._spi.rdEncoders()   # equalize t0 & t1 before periodic updates in 
        self._t1 = self._spi.rdEncoders()   # case we r starting w non-zero encoder counts

if __name__ == '__main__':    # this code runs 1st if module is run as $ python module.py
    odometry = edbotOdom()    # & not being imported:                    import module.py
    try:
        odometry.Start()
	r = rospy.Rate(4)
	while not rospy.is_shutdown():
		odometry.update()
		r.sleep()

    except rospy.ROSInterruptException:
        odometry.Stop()
