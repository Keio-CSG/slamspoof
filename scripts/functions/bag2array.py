#!/usr/bin/env python3
import numpy as np
import struct

def binary2float(data):
    float_value = struct.unpack('<f', data)[0]
    return float_value

def array2float(bin_array):
    float_array = np.apply_along_axis(binary2float, axis=1, arr = bin_array)
    return float_array

def distance_filter(x, y, z, min_dist, max_dist):
    dist = (x ** 2 + y ** 2) ** 0.5 
    mask = (dist > min_dist) & (dist < max_dist)
    x_filtered = x[mask]
    y_filtered = y[mask]
    z_filtered = z[mask]
    return x_filtered, y_filtered, z_filtered

def height_filter(x, y, z, min_height):
    mask = (z > min_height)
    x_filtered = x[mask]
    y_filtered = y[mask]
    z_filtered = z[mask]
    return x_filtered, y_filtered, z_filtered
