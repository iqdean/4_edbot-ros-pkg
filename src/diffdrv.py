#!/usr/bin/env python
'''
Created Aug 2016
@author: Ike A. Dean
 
  ROS PKG: edbot    ROS NODE: diffdrv.py
  Copyright (c) 2016 Ike A. Dean. All rights reserved.    
  Differential Drive Robot Base Controller for edbot robot. 
  Inspired by Dr. Rainer Hessmer's ardros/arduino.py code below. Modified for 
  use on ubilinux based Intel Edision platform driving a Dimension Engineering 
  Kangaroo Motion Controller + SaberTooth Motor Driver.
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
	
	def __init__(self, port="/dev/ttyMFD1", baudRate=9600):
		self._Counter = 0

		rospy.init_node('diffdrv')
    
		rospy.loginfo("DiffDrv _init_: Starting with serial port: " + port + ", baud rate: " + str(baudRate))

		# subscriptions and publications
		rospy.Subscriber("cmd_vel", Twist, self._HandleVelocityCommand)
		
		self._SerialDataGateway = SerialDataGateway(port, baudRate,  self._HandleReceivedLine)

	def Start(self):
		rospy.loginfo("DiffDrv.Start: Starting")
		self._SerialDataGateway.Start()

	def Stop(self):
		rospy.loginfo("DiffDrv.Stop: Stopping")
		self._SerialDataGateway.Stop()
		
	def _HandleVelocityCommand(self, twistCommand):
		""" Handle movement requests. """
		v = twistCommand.linear.x        # m/s
		omega = twistCommand.angular.z      # rad/s
		rospy.loginfo("DiffDrv.hvc: Input ( /cmd_vel msg from ROS) : " + str(v) + "," + str(omega))

		message = 's %.2f %.2f\r' % (v, omega)
		rospy.loginfo("DiffDrv.hvc: Output > Msg to Kanagroo : " + message)
                '''self._SerialDataGateway.Write(message)'''

	def _HandleReceivedLine(self,  line):
		rospy.loginfo("DiffDrv.hrl: Output Ack/Err < Msgs rcvd from Kanagroo : " + line)


if __name__ == '__main__':
	diffdrv = DiffDrv()      #runs __init__ constructor to instance  diffdrv class object
	try:
		diffdrv.Start()  #calls diffdrv.Start() to start serial port listener thread
		rospy.spin()

	except rospy.ROSInterruptException:
		diffdrv.Stop()

