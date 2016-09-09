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

libspi52.c returns:

typedef struct {
  uint32_t x_enc_cnt;     // x-axis encoder count
  uint32_t x_ts_sec;      // x-axis timestamp (sec:nsec)
  uint32_t x_ts_ns;
  uint32_t y_enc_cnt;     // y-axis encoder count
  uint32_t y_ts_sec;      // y-axis timestamp (sec:nsec)
  uint32_t y_ts_ns;
} mtrEnc;
'''

class mtrEnc(ctypes.Structure):
    _fields_ = [("x_enc_cnt", ctypes.c_uint),
                ("x_ts_sec", ctypes.c_uint),
                ("x_ts_ns", ctypes.c_uint),
                ("y_enc_cnt", ctypes.c_uint),
                ("y_ts_sec", ctypes.c_uint),
                ("y_ts_ns", ctypes.c_uint)]

class edspi53(object):

    def Start(self):
        # initialize the library
        self.test = self.mylib.edspi52_init(None)

    def Stop(self):
        # deinit the library
        self.test = self.mylib.edspi52_deinit(None)

    def __init__(self):
        # this function will be called everytime this class is instanced
        # load the library
        self.mylib = ctypes.cdll.LoadLibrary("./libedspi52.so")
        # define library function arg types
        self.mylib.edspi52_init.argtypes = None
        self.mylib.edspi52_init.restype = ctypes.c_uint
        self.mylib.edspi52_deinit.argtypes = None
        self.mylib.edspi52_deinit.restype = ctypes.c_uint
        # getXYEncCount() returns mtrEnc struct on the stack
        self.mylib.getXYEncCount.argtypes = None
        self.mylib.getXYEncCount.restype = mtrEnc

    def rdWheelEncoders(self):
        self.xyenc = mylib.getXYEncCount(None)
        print "x_enc: %8X %d %d" % (self.xyenc.x_enc_cnt, self.xyenc.x_ts_sec, self.xyenc.x_ts_ns)
        print "y_enc: %8X %d %d" % (self.xyenc.y_enc_cnt, self.xyenc.y_ts_sec, self.xyenc.y_ts_ns)
        return(self.xyenc)
