'''
test-odom53.py = test-edspi53.py + odometry calculations
'''

import edspi53
import math

'''
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

def computeOdom(t0, t1):

        print "IN : X: %d Y: %d Heading: %d V: %d Omega: %d" % (X, Y, Heading, V, Omega) 
        deltaLeft = t0.x_enc - t1.x_enc    # delta = current - previous
        deltaRight= t0.y_enc - t1.y_enc    #            t0       t1

        deltaTime = (t0_x_ts_sec - t1_x_ts_sec) + (t0_x_ts_ns - t1_x_ts_ns)   # dt in ns
        deltaTime = deltaTime*(1.0/1000000000.0)                              # dt in sec

        deltaDistance = 0.5 * (deltaLeft + deltaRight)*k1_dpc

        deltaX = deltaDistance * math.cos(Heading);
        deltaY = deltaDistance * math.sin(Heading);

        deltaHeading = (deltaRight - deltaLeft) * k2_rpc

        X += deltaX
        Y += deltaY
        Heading += deltaHeading

        if (Heading > PI):
                Heading -= TwoPI
        else:
                if (Heading <= -PI):
                        Heading += TwoPI

        V = deltaDistance/deltaTime
        Omega = deltaHeading/deltaTime

        print "OUT: X: %d Y: %d Heading: %d V: %d Omega: %d" % (X, Y, Heading, V, Omega) 

def dmpEncData(src, str):
	print str+".x : %8X %d %d" % (src.x_enc, src.x_ts_sec, src.x_ts_ns)
	print str+".y : %8X %d %d" % (src.y_enc, src.y_ts_sec, src.y_ts_ns)

# Constants

k1_dpc = 0.0007596272861609695
k2_rpc = 0.0016614770038516395
PI = 3.141592653589793
TwoPI = 3.141592653589793

# VARIABLES operated on by computeOdom(t0, t1)

X = 0.0
Y = 0.0
Heading = 0.0
V = 0.0
Omega = 0.0
        
# setup storage for current (t0) and previous (t1) periodic samples
t0 = edspi53.MotorEncoders()   # module.class() edspi53.MotorEncoders()
t1 = edspi53.MotorEncoders()

# initial spi library to read wheel encoders
spi = edspi53.Spi()
spi.start()

dmpEncData(t0,"t0")
dmpEncData(t1,"t1")

count = 0
while (count < 100):

	t1 = t0                  # previous <- current
	t0 = spi.rdEncoders()    # current <- new sample

	dmpEncData(t0,"t0")      # dump samples
	dmpEncData(t1,"t1")

        computeOdom(t0,t1)       # compute odometry using t0 t1 samples
	raw_input('--> ')        # wait for input
	count = count+1

spi.stop()



