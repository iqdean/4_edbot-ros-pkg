'''
Created Sept 2016
@author: Ike Dean 

odom53.py - python class to compute odometry based on wheel encoder samples

>>> import math
>>> math.pi
3.141592653589793 
Measured
Robot Parmeters    wheelCircumfrence    825mm   = Pi*Dia           
>>> wd = .2626         wheelDiameter   .2626 m  = 825mm/3.14
>>> tw = .4680         trackWidth      .4680 m
>>> cpr = 4096         Wheel Encoders   4096 cpr (counts per rev)
>>> dpc = (math.pi * wd)/cpr   distancePerCount  k1
>>> rpc = dpc/tw               radiansPerCount   k2

>>> dpc
0.00020141167744938468
>>> rpc
0.00043036683215680484

                x LEFT    y RIGHT     <- looking forward on the robot
                WHEEL     WHEEL
current	 t0	t0.x_enc  t0.y_enc
previous t1     t1.x_enc  t1.y_enc

ex data set:
            count   ts_sec    ts_ns                 ts_ns range
t0.x :    27AAB 1473554582 473208282    < current   000 000 000
t0.y :    27A01 1473554582 473332905                ms  us  ns
                                                    999 999 999 + 1ns = 1.000 000 000 sec
t1.x :    27676 1473554581 373834059    < last
t1.y :    275CA 1473554581 374106956

Every 1/Rate Hz

	double X;        // x coord in global frame
	double Y;        // y coord in global frame
	double Heading;  // heading (radians) in the global frame. 
                            The value lies in (-PI, PI]
	
	double VLeft;   // left motor speed
	double VRight;  // right motor speed
	double V;       // forward speed
	double Omega;   // angular speed (radians per sec)
'''

import ctypes
import time
import math

'''
class MotorEncoders(ctypes.Structure):
    _fields_ = [("x_enc", ctypes.c_uint),
                ("x_ts_sec", ctypes.c_uint),
                ("x_ts_ns", ctypes.c_uint),
                ("y_enc", ctypes.c_uint),
                ("y_ts_sec", ctypes.c_uint),
                ("y_ts_ns", ctypes.c_uint)]
'''

class Odom(object):

    # class variables    
    # internal methods access like so:  Odom.X Odom.Y, etc
    X = 0.0
    Y = 0.0
    Heading = 0.0
    V = 0.0
    Omega = 0.0

    #k1_dpc = 0.00020141167744938468
    #k2_rpc = 0.00043036683215680484
    PI = 3.141592653589793
    TwoPI = 6.283185307179586
    
    # robot param units:    meters      meters     counts per wheel rev
    #                         |           |          |
    # ex:                 .2626         .4680      4096
    def __init__(self, wheelDiameter, trackWidth, encoderCPR):

	print "Robot Params: wd: %f tw: %f cpr: %d" % (wheelDiameter, trackWidth, encoderCPR)
	# Robot Params: wd: 0.262600 tw: 0.468000 cpr: 4096 

        self.k1_dpc = (math.pi * wheelDiameter)/encoderCPR     # distancePerCount k1_dpc = (PI * wheelDia)/encCPR
        self.k2_rpc = self.k1_dpc / trackWidth                 # radiansPerCount  k2_rpc = k1_dpc / trackWidth

	print "k1_dpc: %1.15f k2_rpc: %1.15f" % (self.k1_dpc, self.k2_rpc)
	# k1_dpc: 0.000201411677449 k2_rpc: 0.000430366832157

    def update(self,t0,t1):
	#print "t0.x : %8X %d %d" % (t0.x_enc, t0.x_ts_sec, t0.x_ts_ns)
        #print "t1.y : %8X %d %d" % (t1.y_enc, t1.y_ts_sec, t1.y_ts_ns)
        #print "IN : X: %f Y: %f Heading: %f V: %f Omega: %f" % (Odom.X, Odom.Y, Odom.Heading, Odom.V, Odom.Omega)

	# encoder counts need to be interpeted as 32bit 2's complement encoded numbers

	if (t0.x_enc > 0x7FFFFFFF):
		t0x = -((~t0.x_enc & 0xFFFFFFFF)+1)
	else:
		t0x = t0.x_enc

        if (t0.y_enc > 0x7FFFFFFF):
                t0y = -((~t0.y_enc & 0xFFFFFFFF)+1)
        else:
                t0y = t0.y_enc

        if (t1.x_enc > 0x7FFFFFFF):
                t1x = -((~t1.x_enc & 0xFFFFFFFF)+1)
        else:
                t1x = t1.x_enc

        if (t1.y_enc > 0x7FFFFFFF):
                t1y = -((~t1.y_enc & 0xFFFFFFFF)+1)
        else:
                t1y = t1.y_enc

        #print "t0x: %d t1x: %d t0y: %d t1y: %d" % (t0x, t1x, t0y, t1y)

        deltaLeft = t0x - t1x    # delta = current - previous
        deltaRight= t0y - t1y    #            t0       t1

        deltaTime_sec = (t0.x_ts_sec - t1.x_ts_sec)                    # dt_sec part
        deltaTime_ns = (t0.x_ts_ns - t1.x_ts_ns)*(1.0/1000000000.0)    # dt_ns part in sec
        deltaTime = deltaTime_sec + deltaTime_ns                       # delta t in sec

        deltaDistance = 0.5 * (deltaLeft + deltaRight)*self.k1_dpc

        deltaX = deltaDistance * math.cos(Odom.Heading);
        deltaY = deltaDistance * math.sin(Odom.Heading);

        deltaHeading = (deltaRight - deltaLeft) * self.k2_rpc

        #print "dLeft: %d dRight: %d dt: %f dDist: %f dX: %f dY: %f dHeading: %f" % (
        #    deltaLeft, deltaRight, deltaTime, deltaDistance, deltaX, deltaY, deltaHeading)

        Odom.X += deltaX
        Odom.Y += deltaY
        Odom.Heading += deltaHeading

	# ROS Robot Coordinate Frames are:                         X fwd, Y left, Z up
	# Polarity of rotations about Z per right hand rule:  - rotate left, + rotate right

	# Limit Heading to +180 to -180 relative to X
	# +180 (pi) -- left -- 0 -- right -- (-pi) (-180)
        #  3.14        turn         turn            -3.14

        if (Odom.Heading > Odom.PI):                # if (heading > 180) then heading = heading - 360
                Odom.Heading -= Odom.TwoPI          #     181                 -179    = 181 - 360
        else:
                if (Odom.Heading <= -Odom.PI):      # if (heading <= -180) then heading = heading + 360
                        Odom.Heading += Odom.TwoPI  #     -181                  +179    = -181 + 360 

        Odom.V = deltaDistance/deltaTime
        Odom.Omega = deltaHeading/deltaTime

        #print "OUT: X: %f Y: %f Heading: %f V: %f Omega: %f" % (Odom.X, Odom.Y, Odom.Heading, Odom.V, Odom.Omega)
 
