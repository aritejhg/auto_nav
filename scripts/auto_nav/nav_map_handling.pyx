import numpy as np
cimport numpy as np

DTYPE = np.uint8
ctypedef np.uint8_t DTYPE_t

def CheckNeighboursOr(np.ndarray arr, int i, int j, DTYPE_t v):
    if arr[i-1, j-1]==v or arr[i-1, j]==v or arr[i-1, j+1]==v or arr[i, j-1]==v or \
        arr[i, j+1]==v or arr[i+1, j-1]==v or arr[i+1, j]==v or arr[i+1, j+1]==v :
        return True
    else: return False

def MapDilateErode(np.ndarray mArray):

    assert mArray.dtype == DTYPE

    cdef int w = mArray.shape[1]
    cdef int h = mArray.shape[0]
    cdef np.ndarray mc = np.zeros([h, w], dtype=DTYPE)
    cdef int i = 1
    cdef int j = 1
    
    while i < h - 1:
        while j < w - 1:
            if mArray[i, j] != 2 and CheckNeighboursOr(mArray, i, j, 2):
                mc[i, j] = 2    # dilate the wall cells
            elif mArray[i, j] == 0 and CheckNeighboursOr(mArray, i, j, 1):
                mc[i, j] = 1    # erode the unmapped cells
            else:
                mc[i, j] = mArray[i, j]
            j += 1
        i += 1

    return mc