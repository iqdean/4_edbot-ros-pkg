import edspi53

def dmpEncData(data, label):
        print label+".x : %8X %d %d" % (data.x_enc, data.x_ts_sec, data.x_ts_ns)
        print label+".y : %8X %d %d" % (data.y_enc, data.y_ts_sec, data.y_ts_ns)

# setup storage for current (t0) and previous (t1) periodic samples
t0 = edspi53.MotorEncoders()   
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

spi.stop()



