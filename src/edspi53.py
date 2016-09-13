'''
Created Sept 2016
@author: Ike Dean 

edspi53.py - python class to read wheel encoders using libspi52.so library

libspi52.so - uses libmraa's memory mapped fast gpio to bitbang spi bus 
connected to robogia quadrature encoder decoder

'''
import ctypes
import time
import math


'''
NOTES:
libspi52.c returns:

typedef struct {
  uint32_t x_enc_cnt;     // x-axis encoder count
  uint32_t x_ts_sec;      // x-axis timestamp (sec:nsec)
  uint32_t x_ts_ns;
  uint32_t y_enc_cnt;     // y-axis encoder count
  uint32_t y_ts_sec;      // y-axis timestamp (sec:nsec)
  uint32_t y_ts_ns;
} mtrEnc;

Class object names:   Name
Class method names:   name

'''

class MotorEncoders(ctypes.Structure):
    _fields_ = [("x_enc", ctypes.c_uint),
                ("x_ts_sec", ctypes.c_uint),
                ("x_ts_ns", ctypes.c_uint),
                ("y_enc", ctypes.c_uint),
                ("y_ts_sec", ctypes.c_uint),
                ("y_ts_ns", ctypes.c_uint)]

class Spi(object):

    def start(self):
        # initialize the library
        self.test = self.lib.edspi52_init(None)

    def stop(self):
        # deinit the library
        self.test = self.lib.edspi52_deinit(None)

    # this function is called everytime this class is instanced
    def __init__(self):
        # load libedspi52.so library
        self.lib = ctypes.cdll.LoadLibrary("./libedspi52.so")
        # define library function arg types
        self.lib.edspi52_init.argtypes = None
        self.lib.edspi52_init.restype = ctypes.c_uint
        self.lib.edspi52_deinit.argtypes = None
        self.lib.edspi52_deinit.restype = ctypes.c_uint
        # getXYEncCount() returns mtrEnc struct on the stack
        self.lib.getXYEncCount.argtypes = None
        self.lib.getXYEncCount.restype = MotorEncoders

    def rdEncoders(self):

	# read the wheel encoders
        self.ei = self.lib.getXYEncCount(None)
        print "ei.x : %8X %d %d" % (self.ei.x_enc, self.ei.x_ts_sec, self.ei.x_ts_ns)
        print "ei.y : %8X %d %d" % (self.ei.y_enc, self.ei.y_ts_sec, self.ei.y_ts_ns)
        return(self.ei)
