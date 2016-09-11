'''
test-odom53.py = test-edspi53.py + odometry calculations
'''

import edspi53
import odom53
import math

def dmpEncData(src, str):
	print str+".x : %8X %d %d" % (src.x_enc, src.x_ts_sec, src.x_ts_ns)
	print str+".y : %8X %d %d" % (src.y_enc, src.y_ts_sec, src.y_ts_ns)

      
# setup storage for current (t0) and previous (t1) periodic samples
t0 = edspi53.MotorEncoders()   # module.class() edspi53.MotorEncoders()
t1 = edspi53.MotorEncoders()

# initial spi library to read wheel encoders
spi = edspi53.Spi()
spi.start()
odom = odom53.Odom()

dmpEncData(t0,"t0")
dmpEncData(t1,"t1")

count = 0
while (count < 100):

	t1 = t0                  # previous <- current
	t0 = spi.rdEncoders()    # current <- new sample

	dmpEncData(t0,"t0")      # dump samples
	dmpEncData(t1,"t1")

        odom.update(t0,t1)       # compute odometry using t0 t1 samples

	raw_input('--> ')        # wait for input
	count = count+1

spi.stop()



