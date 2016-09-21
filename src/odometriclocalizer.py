#!/usr/bin/env python
'''
Created Sept 2016
@author: Ike Dean 

 odometriclocalizer.py for edbot robot's differential drive base
 this ros node publishs odometry messages so ros navigation stack
 can localize the robot on a static map of the world.

 Thanks to Intel's lame hw design of edison gpio mux controls and 
 the idiotic sw library (libmraa) that controls the gpio mux setting 
 via sysfs:

 a) U have to have root priviliges to start programs that incorporate 
    libmraa to config edison gpio settings.
 b) whenever you use libmraa to initialize specific gpio pins, 
    various other gpio signals will glitch.
 c) the spi bus interface driver that intel puts out is useless for any
    practical purposes...it's quicker to bitbang the spi i/f than use
    intel's spi driver, which i'm having wr/use libspi5.so library
    which uses libmraa's memory-mapped gpio i/f to bit bang the spi bus to read
    wheel encoders interfaced to spi-based quadrature encoder decoders

 (a) + (b) + (c) results in  INTEL EDISON and it's associated software 
  support being a real pain in the rear to work with: 

 libspi52.so uses libmraa mem-mapped gpio to bitbang spi i/f
 libmraa's initialization routines that config spi gpio pins cause
 the serial port gpio pins to glitch which causes kangaroo mc to 
 detect serial connection disconnects (error E6), which requires the 
 Kangaroo MC to be reinitialized otherwise the differential drive 
 system stops responding to drive/turn signals being sent to it
 over the serial port.

 WORKAROUNDS for the above mess:
 1st 
  Start odometriclocalizer.py node... this node will load
  libspi52.so which will use libmraa to initialze spi bus gpio
  pins which will cause the serial port gpio pins to glitch
  - since this node uses libspi52.so which uses libmraa, this
    node has to be launched with root privileges. 
  - launching a rosnode with root privilages instead of a normal
    user IS NOT the norm, so it requires following song and dance:

 iqdean@ubilinux:~$ roscd edbot
 iqdean@ubilinux:~/catkin_ws/src/edbot$ sudo /bin/bash
 root@ubilinux:/home/iqdean/catkin_ws/src/edbot# source src/startOdomAsRoot
 root@ubilinux:/home/iqdean/catkin_ws/src/edbot# rosrun edbot src/odometriclocalizer.py

 2nd 
 - Start diffdrv node AFTER starting odometriclocalizer node
   so libmraa gpio init's don't clobber the serial port connection
   with Dimension Engineering's Kangaroo Motion Contoller over 
   serial port /dev/ttyMFLD2. The cfgttyMFLD2.sh script incorporated into 
   /etc/rc.local (so it start with root privilages) uses sysfs i/f to 
   enable gpio pins for use as serial port... only the cfgttyMFLD2.sh has
   to be run with root priviliges, the diffdrv node can be started as normal
   user since it assumes the serial port gpio pins have already been configd:

   $ rosrun edbot src/diffdrv.py
 
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
        #now = rospy.get_rostime()
        #rospy.loginfo("Current time %i %i", now.secs, now.nsecs)
        self._t1 = self._t0                   # previous t1 = current
        self._t0 = self._spi.rdEncoders()     # current  t0 = new sample  ---present state--
        self._odom.update(self._t0, self._t1) # compute odometry = f(t0,t1, [X Y Heading V Omega])

        #print "OUT: X: %f Y: %f Heading: %f V: %f Omega: %f" % (self._odom.X, self._odom.Y, self._odom.Heading, self._odom.V, self._odom.Omega)
        #self._cnt -= 1
        #if ( (self._cnt == 0) or (self._cnt == 10) ):
        #            print "OUT: X: %f Y: %f Heading: %f V: %f Omega: %f" % (self._odom.X, self._odom.Y, self._odom.Heading, self._odom.V, self._odom.Omega)
        #            if (self._cnt == 0):
        #                self._cnt = 20

        quaternion = Quaternion()
	quaternion.x = 0.0 
	quaternion.y = 0.0
	quaternion.z = sin(self._odom.Heading / 2.0)
	quaternion.w = cos(self._odom.Heading / 2.0)
		
	rosNow = rospy.Time.now()
			
	# First, we'll publish the transform from frame odom to frame base_link over tf
	# Note that sendTransform requires that 'to' is passed in before 'from' while
	# the TransformListener' lookupTransform function expects 'from' first followed by 'to'.
	self._OdometryTransformBroadcaster.sendTransform(
            (self._odom.X, self._odom.Y, 0),
            (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
            rosNow,
            "base_link",
            "odom")

	# next, we'll publish the odometry message over ROS
	odometry = Odometry()
	odometry.header.frame_id = "odom"
	odometry.header.stamp = rosNow
	odometry.pose.pose.position.x = self._odom.X
	odometry.pose.pose.position.y = self._odom.Y
	odometry.pose.pose.position.z = 0
	odometry.pose.pose.orientation = quaternion

	odometry.child_frame_id = "base_link"
	odometry.twist.twist.linear.x = self._odom.V
	odometry.twist.twist.linear.y = 0
	odometry.twist.twist.angular.z = self._odom.Omega

	self._OdometryPublisher.publish(odometry)

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
        self._spi.start()           #   time stamped wheel encoder data using libspi52.so

        # TODO
        # - create launch file to read robot parameters from yaml file and store them to ros param server
        # - add code to read robot parameters stored on ros param server
        wd = .2626
        tw = .4680
        cpr = 4096
        self._odom = odom53.Odom(wd,tw,cpr)  # instance Odom() class to compute odometry

        self._t0 = self._spi.rdEncoders()   # equalize t0 & t1 before periodic updates in 
        self._t1 = self._spi.rdEncoders()   # case we r starting w non-zero encoder counts

        self._cnt = 20;

if __name__ == '__main__':    # this code runs 1st if module is run as $ python module.py
    edbotodom = edbotOdom()    #                   & not being imported:  import module.py
    try:
        edbotodom.Start()
	r = rospy.Rate(20)               # update/publish  odometry at 20Hz
	while not rospy.is_shutdown():
		edbotodom.update()
		r.sleep()

    except rospy.ROSInterruptException:
        edbotodom.Stop()
