#!/usr/bin/env python
'''
Created Aug 2016
@author: Ike A. Dean

  ROS PKG: edbot    ROS NODE: diffdrv.py
  Copyright (c) 2016 Ike A. Dean. All rights reserved.
  Differential Drive Robot Base Controller for edbot robot.
  Inspired by Dr. Rainer Hessmer's ardros/arduino.py code below. Modified for
  use on ubilinux based Intel Edision platform driving a Dimension Engineering
  Kangaroo Motion Controller + SaberTooth 2x12 Motor Driver.
  Copyright (c) 2016 Ike A. Dean. All rights reserved.

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>

Created January, 2011

@author: Dr. Rainer Hessmer

  arduino.py - gateway to Arduino based differential drive base
  Borrowed heavily from Mike Feguson's ArbotiX base_controller.py code.

'''

import roslib
import rospy
#import tf
import math
from math import sin, cos, pi
import sys

from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from SerialDataGateway import SerialDataGateway

class DiffDrv(object):

	def __init__(self, port="/dev/ttyMFD1", baudRate=19200):
		self._Counter = 0

		rospy.init_node('diffdrv')

		rospy.loginfo("DiffDrv _init_: Starting with serial port: " + port + ", baud rate: " + str(baudRate))

		# subscriptions and publications
		rospy.Subscriber("cmd_vel", Twist, self._HandleVelocityCommand)

		self._SerialDataGateway = SerialDataGateway(port, baudRate,  self._HandleReceivedLine)

	def Start(self):
		rospy.loginfo("DiffDrv.Start: Starting")
		self._SerialDataGateway.Start()
		self._KangarooMC_start()

	def Stop(self):
		rospy.loginfo("DiffDrv.Stop: Stopping")
		self._KangarooMC_stop()
		self._SerialDataGateway.Stop()

	def _HandleVelocityCommand(self, twistCommand):
		""" Handle movement requests. """
		v1 = twistCommand.linear.x          # m/s   from ros
		v2 = int(round(v1*1000))            # mm/s  to kanagroo motion controller
		omega1 = twistCommand.angular.z     # rad/s from ros
		omega2 = int(round(omega1*57.2958)) # deg/s to kangroo motion controller
		message = 'd,s' + str(v2) + '\r\n' + 't,s' + str(omega2) + '\r\n'
		self._SerialDataGateway.Write(message)

	def _HandleReceivedLine(self,  line):
		rospy.loginfo("HRL kmc rcv: " + line)

	def _KangarooMC_start(self):
		# start kmc using simplified serial interface
		message = 'd,start\r\nd,start\r\nt,start\r\nt,start\r\nd,s0\r\nt,s0\r\n'
		self._SerialDataGateway.Write(message)
		# set the units for all subseqent drive/turn commands
		# units are based on robot geometry (wheel dia, track width, encoder resolution)
		# REF: https://www.dimensionengineering.com/info/encoders?mode=mixed
		message = 'd,units 825 mm = 1024 lines\r\n'
		self._SerialDataGateway.Write(message)
		message = 't,units 360 degrees = 1825 lines\r\n'
		self._SerialDataGateway.Write(message)

	def _KangarooMC_stop(self):
		# stop kangaroo motion controller
		message = 'd,stop\r\nt,stop\r\n'
		self._SerialDataGateway.Write(message)

if __name__ == '__main__':
	diffdrv = DiffDrv()      #runs __init__ constructor to instance  diffdrv class object
	try:
		diffdrv.Start()  #calls diffdrv.Start() to start serial port listener thread
		rospy.spin()

	except rospy.ROSInterruptException:
		diffdrv.Stop()
