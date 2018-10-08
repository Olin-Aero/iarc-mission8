import numpy as np
def rint(x):
    return np.int32(np.round(x))
def anorm(x):
    return (x+np.pi) % (2*np.pi) - np.pi
def adiff(a,b):
    return anorm(a-b)

