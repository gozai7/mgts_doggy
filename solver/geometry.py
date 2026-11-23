import math

import numpy as np
import yaml


def load_amcl_map(path):
    config = yaml.safe_load(path.read_text())
    if not math.isclose(float(config["origin"][2]), 0.0, abs_tol=1e-9):
        raise ValueError(f"{path}: AMCL requires zero origin yaw; reproject the raster and its origin")
    return config


def compose(a, b):
    c, s = math.cos(a[2]), math.sin(a[2])
    return np.array([a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], a[2] + b[2]])


def inverse(p):
    c, s = math.cos(p[2]), math.sin(p[2])
    return [-c * p[0] - s * p[1], s * p[0] - c * p[1], -p[2]]


def csv_save(path, rows):
    np.savetxt(path, rows, delimiter=",", header="t,x,y,yaw", comments="", fmt="%.9f")
