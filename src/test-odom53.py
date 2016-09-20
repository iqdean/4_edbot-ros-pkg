'''
test-odom53.py = test-edspi53.py + odometry calculations
'''

import edspi53     # class MotorEncoder() gets defined in here
import odom53      # & you don't need to define it again in here
import math        # but now odom53 depends on edspi53 being already imported

def dmpEncData(src, str):
	print str+".x : %8X %d %d" % (src.x_enc, src.x_ts_sec, src.x_ts_ns)
	print str+".y : %8X %d %d" % (src.y_enc, src.y_ts_sec, src.y_ts_ns)

      
# setup storage for current (t0) and previous (t1) periodic samples
t0 = edspi53.MotorEncoders()   # module.class() edspi53.MotorEncoders()
t1 = edspi53.MotorEncoders()

# initial spi library to read wheel encoders
spi = edspi53.Spi()
spi.start()

#odom = odom53.Odom()        
wd = .2626
tw = .4680
cpr = 4096
odom = odom53.Odom(wd,tw,cpr)

dmpEncData(t0,"t0")
dmpEncData(t1,"t1")

# on program start, make previous = initial to avoid delta in computed
# values due to previous being 0 and t0 being something other than 0
t0 = spi.rdEncoders()
t1 = spi.rdEncoders()

count = 0
while (count < 1000):

	t1 = t0                  # previous <- current
	t0 = spi.rdEncoders()    # current <- new sample

	dmpEncData(t0,"t0")      # dump samples
	dmpEncData(t1,"t1")

        odom.update(t0,t1)       # compute odometry using t0 t1 samples
	print "OUT: X: %f Y: %f Heading: %f V: %f Omega: %f" % (odom.X, odom.Y, odom.Heading, odom.V, odom.Omega)

	raw_input('--> ')        # wait for input
	count = count+1

spi.stop()



