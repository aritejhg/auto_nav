import os
import numpy as np
import numpy.ctypeslib as npct
from ctypes import c_int

scriptDir = os.path.dirname(__file__)

# input type for the cos_doubles function
# must be an int array, with single dimension that is contiguous
array_1d_u8 = npct.ndpointer(dtype=np.uint8, ndim=1, flags='CONTIGUOUS')
array_1d_i32 = npct.ndpointer(dtype=np.int32, ndim=1, flags='CONTIGUOUS')

# load the library, using numpy mechanisms
libmapProc = npct.load_library("libnavMapProc", os.path.join(scriptDir, '.'))

# setup the return types and argument types
libmapProc.MapDilateErode.restype = None
libmapProc.MapDilateErode.argtypes = [array_1d_u8, c_int, c_int]
libmapProc.FindNavGoal.restype = c_int
libmapProc.FindNavGoal.argtypes = [array_1d_u8, c_int, c_int, c_int, c_int, array_1d_i32]

def MapDilateErode(mapArray):
    assert mapArray.ndim == 2
    h = mapArray.shape[0]
    w = mapArray.shape[1]
    mapArray = mapArray.ravel()
    libmapProc.MapDilateErode(mapArray, h, w)
    return mapArray.reshape((h, w))

def FindNavGoal(mapArray, posX, posY):
    NUM_NAVPNT = 10
    assert mapArray.ndim == 2
    h = mapArray.shape[0]
    w = mapArray.shape[1]
    res = np.zeros(1 + 2 * NUM_NAVPNT, np.int32)
    res[0] = NUM_NAVPNT
    posX = np.int32(posX)
    posY = np.int32(posY)
    state = libmapProc.FindNavGoal(mapArray.ravel(), h, w, posY, posX, res)
    navGoals = []
    for i in range(1, res[0] * 2 + 1, 2):
        navGoals.append((res[i], res[i + 1]))
    if state == 1: fini = True
    else: fini = False
    return (fini, navGoals)