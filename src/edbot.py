import ctypes

class TimeStampedEncoderData(ctypes.Structure):
    _fields_ = [("x_enc", ctypes.c_uint),
                ("x_ts_sec", ctypes.c_uint),
                ("x_ts_ns", ctypes.c_uint),
                ("y_enc", ctypes.c_uint),
                ("y_ts_sec", ctypes.c_uint),
                ("y_ts_ns", ctypes.c_uint)]
