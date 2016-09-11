import edspi53

def cpEncoderData(src,dst):
	dst.x_enc = src.x_enc
	dst.x_ts_sec = src.x_ts_sec
	des.x_ts_ns = src.x_ts_ns
        dst.y_enc = src.y_enc
        dst.y_ts_sec = src.y_ts_sec
        des.y_ts_ns = src.y_ts_ns

def dmpEncData(src, str):
	print str+".x : %8X %d %d" % (src.x_enc, src.x_ts_sec, src.x_ts_ns)
	print str+".y : %8X %d %d" % (src.y_enc, src.y_ts_sec, src.y_ts_ns)

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
	t1 = t0
	t0 = spi.rdEncoders()
	dmpEncData(t0,"t0")
	dmpEncData(t1,"t1")
	raw_input('--> ')
	count = count+1

#spi.stop()



