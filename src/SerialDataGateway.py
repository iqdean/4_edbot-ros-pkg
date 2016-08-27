#!/usr/bin/env python
'''
Updated Aug 2016
@author: Ike A. Dean
Modified for use with edbot robot

Created on November 20, 2010
@author: Dr. Rainer Hessmer
'''
import threading
import serial
from cStringIO import StringIO
import time
import rospy

def _OnLineReceived(line):
	print(line)

class SerialDataGateway(object):
	'''
	Helper class for receiving lines from a serial port
	'''

	def __init__(self, port="/dev/ttyMFD1", baudrate=19200, lineHandler = _OnLineReceived):
		'''
		Initializes the receiver class.
		port: The serial port to listen to.
		receivedLineHandler: The function to call when a line was received.
		'''
		rospy.loginfo("sdg._init_");
		self._Port = port
		self._Baudrate = baudrate
		self.ReceivedLineHandler = lineHandler
		self._KeepRunning = False

 	def Start(self):
		rospy.loginfo("sdg.Start: Starting serial gateway")
		self._Serial = serial.Serial(port = self._Port, baudrate = self._Baudrate, timeout = 1)

		self._KeepRunning = True
		self._ReceiverThread = threading.Thread(target=self._Listen)
		self._ReceiverThread.setDaemon(True)
		self._ReceiverThread.start()

	def Stop(self):
		rospy.loginfo("sdg.Stop: Stopping serial gateway")
		self._KeepRunning = False
		time.sleep(.1)
		self._Serial.close()

	def _Listen(self):
		stringIO = StringIO()
		while self._KeepRunning:
			data = self._Serial.read()
			if data == '\r':
				pass
			if data == '\n':
				self.ReceivedLineHandler(stringIO.getvalue())
				stringIO.close()
				stringIO = StringIO()
			else:
				stringIO.write(data)

	def Write(self, data):
		info = "sdg.Write serial port: %s" %data
		rospy.loginfo(info)
		self._Serial.write(data)
        #
        # ed <-- /dev/ttyMFD1 --> Kanagroo Motion Controller serial link config
        # 1       19200 baud      Kangaroo is setup for 19200 baud, kang <-9600-> sbt
        # 2 edison gpio mux have to be configured to enable use of /dev/ttyMFD1
        #   by running cfgttyMFD1.sh bash script. we can autostart in /etc/rc.local
        #   but for now, make sure to run $ sudo ./cfgttyMFD1.sh BEFORE using this node

	if __name__ == '__main__':
		dataReceiver = SerialDataGateway("/dev/ttyMFD1",  19200)
		dataReceiver.Start()

		raw_input("sdg._main_: Hit <Enter> to end.")
		dataReceiver.Stop()
