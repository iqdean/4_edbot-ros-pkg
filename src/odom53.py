'''
Created Sept 2016
@author: Ike Dean 

odom53.py - python class to compute odometry based on wheel encoder samples

Robot Parameters
wheelDiameter = .2476    # .2476 m = 9.75"
trackWidth = .4572       # .4572 m = 18"
countsPerRev = 1024      # 1024 = wheel encoder resolution

>>> import math
>>> math.pi
3.141592653589793
>>> wd = .2476
>>> tw = .4572
>>> cpr = 1024
>>> dpc = (math.pi * wd)/cpr   distancePerCount  k1
>>> rpc = dpc/tw               radiansPerCount   k2
>>> dpc
0.0007596272861609695
>>> rpc
0.0016614770038516395
>>> 

                x LEFT          y RIGHT     <- looking forward on the robot
                  WHEEL           WHEEL
current	 t0	t0.x_enc	t0.y_enc
previous t1     t1.x_enc        t1.y_enc

ex data set:
          count   ts_sec    ts_ns                      ts_ns range
t0.x :    27AAB 1473554582 473208282    < current      000 000 000
t0.y :    27A01 1473554582 473332905                     ms   us  ns
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

class MotorEncoders(ctypes.Structure):
    _fields_ = [("x_enc", ctypes.c_uint),
                ("x_ts_sec", ctypes.c_uint),
                ("x_ts_ns", ctypes.c_uint),
                ("y_enc", ctypes.c_uint),
                ("y_ts_sec", ctypes.c_uint),
                ("y_ts_ns", ctypes.c_uint)]

class Odom(object):

    # class variables    
    # internal methods access like so:  Odom.X Odom.Y, etc
    X = 0.0
    Y = 0.0
    Heading = 0.0
    V = 0.0
    Omega = 0.0

    k1_dpc = 0.0007596272861609695
    k2_rpc = 0.0016614770038516395
    PI = 3.141592653589793
    TwoPI = 3.141592653589793

    def update(self,t0,t1):
	print "t0.x : %8X %d %d" % (t0.x_enc, t0.x_ts_sec, t0.x_ts_ns)
        print "t0.y : %8X %d %d" % (t0.y_enc, t0.y_ts_sec, t0.y_ts_ns)
        print "IN : X: %d Y: %d Heading: %d V: %d Omega: %d" % (Odom.X, Odom.Y, Odom.Heading, Odom.V, Odom.Omega) 
        deltaLeft = t0.x_enc - t1.x_enc    # delta = current - previous
        deltaRight= t0.y_enc - t1.y_enc    #            t0       t1

        deltaTime = (t0.x_ts_sec - t1.x_ts_sec) + (t0.x_ts_ns - t1.x_ts_ns)   # dt in ns
        deltaTime = deltaTime*(1.0/1000000000.0)                              # dt in sec

        deltaDistance = 0.5 * (deltaLeft + deltaRight)*Odom.k1_dpc

        deltaX = deltaDistance * math.cos(Odom.Heading);
        deltaY = deltaDistance * math.sin(Odom.Heading);

        deltaHeading = (deltaRight - deltaLeft) * Odom.k2_rpc

        Odom.X += deltaX
        Odom.Y += deltaY
        Odom.Heading += deltaHeading

        if (Odom.Heading > Odom.PI):
                Odom.Heading -= Odom.TwoPI
        else:
                if (Odom.Heading <= -Odom.PI):
                        Odom.Heading += Odom.TwoPI

        Odom.V = deltaDistance/deltaTime
        Odom.Omega = deltaHeading/deltaTime

        print "OUT: X: %d Y: %d Heading: %d V: %d Omega: %d" % (Odom.X, Odom.Y, Odom.Heading, Odom.V, Odom.Omega)
 
