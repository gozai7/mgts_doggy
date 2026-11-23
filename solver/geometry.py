import math

import numpy as np


def compose(a, b):
    c, s = math.cos(a[2]), math.sin(a[2])
    return np.array([a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], a[2] + b[2]])


def inverse(p):
    c, s = math.cos(p[2]), math.sin(p[2])
    return [-c * p[0] - s * p[1], s * p[0] - c * p[1], -p[2]]


def csv_save(path, rows):
    np.savetxt(path, rows, delimiter=",", header="t,x,y,yaw", comments="", fmt="%.9f")
