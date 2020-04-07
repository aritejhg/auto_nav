import os
import numpy as np
import numpy.ctypeslib as npct
from ctypes import c_int

scriptDir = os.path.dirname(__file__)

# input type for the cos_doubles function
# must be an int array, with single dimension that is contiguous
array_1d_int = npct.ndpointer(dtype=np.uint8, ndim=1, flags='CONTIGUOUS')

# load the library, using numpy mechanisms
libmapProc = npct.load_library("libnavMapProc", os.path.join(scriptDir, '.'))

# setup the return types and argument types
libmapProc.MapDilateErode.restype = None
libmapProc.MapDilateErode.argtypes = [array_1d_int, c_int, c_int]

def MapDilateErode(mapArray):
    assert mapArray.ndim == 2
    h = mapArray.shape[0]
    w = mapArray.shape[1]
    mapArray = mapArray.ravel()
    libmapProc.MapDilateErode(mapArray, h, w)
    return mapArray.reshape((h, w))