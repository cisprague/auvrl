# -*- coding: utf-8 -*-
"""
Created on Mon Mar  6 19:13:19 2017

@author: ozer
bunch of utility functions shared across just about everything
"""
from __future__ import print_function
import numpy as np

def scale_range(values, new_min, new_max, org_min=None, org_max=None):
    if org_min is None:
        org_min = np.min(values)
    if org_max is None:
        org_max = np.max(values)
    org_range = org_max - org_min
    new_range = new_max - new_min
    values = np.array(values)
    ret = (((values - org_min) * new_range) / org_range) + new_min
    if ret.shape == (1,):
        return ret[0]
    return ret

def regular_int_grid_coords2(xto, yto, xfrom=0, yfrom=0):
    """
    i do this too much
    """
    for x in range(xfrom,xto):
        for y in range(yfrom, yto):
            yield (x,y)



def make_circle_targets(center=[0,0], count=8, radii=[1]):
    """
    returns count number of points radii away from center.
    """
    pts = []
    for radius in radii:
        for angle in np.linspace(0,2*np.pi, count+1):
            px,py = radius*np.sin(angle), radius*np.cos(angle)
            px += center[0]
            py += center[1]
            pts.append((px,py))
    return pts[1:]


def rand_range(rng):
    """
    return (-rng,rng) random float
    """
    return (np.random.ranf()*2*rng) - rng

def float_format2(value):
    try:
        return  "{0:.7f}".format(value)
    except:
        return '-'

def load_colormap(filename, reverse=False):
        """
        loads a simple matrix file as a colormap into a numpy array
        """
        v = []
        with open(filename) as f:
            for line in f:
                c = [int(part) for part in line.split()]
                v.append(c)
        if reverse:
            v = v[::-1]
        m = np.array(v)/255.
        return m


def clamp(minval,maxval, val):
    """
    Simply clamps the value into min-max
    """
    return max(minval, min(maxval, val))


def to_rad(deg):
    """
    Simple degree to radian conversion
    """
    return np.pi*deg/180.

def to_deg(rad):
    """
    Simple radian to degree conversion
    """
    return 180*rad/np.pi

def sin_deg(deg):
    """
    wrapper for numpy's sin
    """
    return np.sin(to_rad(deg))


def cos_deg(deg):
    """
    wrapper for numpy's cos
    """
    return np.cos(to_rad(deg))
