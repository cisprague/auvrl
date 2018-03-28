# -*- coding: utf-8 -*-
"""
Created on Sun Feb 21 11:44:13 2016

@author: kkalem
"""
from __future__ import print_function
import numpy as np
import math


ZERO = 1e-15

def quat_to_yaw(quat):
    return math.atan2(2.*(quat[0]*quat[1] + quat[3]*quat[2]), quat[3]**2 + quat[0]**2 - quat[1]**2 - quat[2]**2)

def norm(V):
    return np.linalg.norm(np.array(V))

def normalize(V):
    n = norm(V)
    if n == 0:
        return V
    else:
        return np.array(V)/n

def project(a,b):
    """
    project a vector (np array) a onto b
    """
    return (a.dot(b)/b.dot(b))*b

def rotate_vec_vec(v1s, rads):
    """
    v1s is an array of shape (N,2) where each row is a vector
    rotates ALL of them rads radians around the origin
    """
    x1s = v1s[:,0]
    y1s = v1s[:,1]

    x2s = np.cos(rads)*x1s - np.sin(rads)*y1s
    y2s = np.sin(rads)*x1s + np.cos(rads)*y1s

    res = np.zeros_like(v1s)
    res[:,0] = x2s
    res[:,1] = y2s

    return res

def rotate_vec(v1, rads):
    """
    v1 = [x,y], rotates v1 around origin rads radians, returns new vector
    """
    x1,y1 = v1
    x2 = np.cos(rads)*x1 - np.sin(rads)*y1
    y2 = np.sin(rads)*x1 + np.cos(rads)*y1

    if np.abs(x2) <= ZERO:
        x2 = 0
    if np.abs(y2) <= ZERO:
        y2 = 0

    return (x2,y2)

def directed_angle(v1,v2):
    """
    returns angle in a directed fashion, from v1 to v2, v1+angle = v2
    negative value means v2 is closer if v1 rotates cw
    """
    x1,x2 = v1[0],v2[0]
    y1,y2 = v1[1],v2[1]

    dot = x1*x2 + y1*y2      # dot product
    det = x1*y2 - y1*x2      # determinant
    angle_diff = np.arctan2(det, dot)  # atan2(y, x) or atan2(sin, cos)

    return angle_diff

def directed_angle_vec(v1, v2):
    """
    works on a whole list of vectors instead of one
    v1,v2 must have the same shape
    and are numpy arrays
    """
    x1s = v1[:,0]
    x2s = v2[:,0]
    y1s = v1[:,1]
    y2s = v2[:,1]

    dots = x1s*x2s + y1s*y2s
    dets = x1s*y2s - y1s*x2s

    angles = np.arctan2(dets,dots)
    return angles

def vec_angle(v1,v2):
    """
    returns shortest positive angle between v1,v2
    """
    u = np.array(v1)
    v = np.array(v2)

    cost = np.dot(u,v) / (euclid_distance([0,0],u)*euclid_distance([0,0],v))
    return np.arccos(cost)


def perp_vec(V):
    P = np.dot(V,np.array([[0,-1],[1,0]]))
    return P


def euclid_distance(pos1,pos2):
    try:
        return np.sqrt((pos2[0]-pos1[0])**2 + (pos2[1]-pos1[1])**2)
    except Exception as e:
        raise ValueError('Can not calc euclid_distance for these '+str(pos1)+
                         '  '+str(pos2)+'\nOrg error:'+str(e))

def euclid_distance3(pos1,pos2):
    return np.sqrt( (pos2[0]-pos1[0])**2 +
                    (pos2[1]-pos1[1])**2 +
                    (pos2[2]-pos1[2])**2)

def euclid_distance_vec(pos1,pos2):
    """
    pos's are numpy arrays with shape >= (:,2)
    """

    return np.sqrt(np.square(pos2[:,0]-pos1[:,0]) + np.square(pos2[:,1]-pos1[:,1]))

def magnitude3_vec(vecs):
    """
    vecs is a np array of (N,3)
    """
    xsq = vecs[:,0]**2
    ysq = vecs[:,1]**2
    zsq = vecs[:,2]**2
    sumsq = xsq+ysq+zsq
    return np.sqrt(sumsq)



# returns a line in the form y = mx+b, returns the m,b
def lineFrom2pts(A,B):
    m = (A[1]-B[1]) / (A[0]-B[0])
    b = A[1]-A[0]*m
    return m,b

#returns the a,b,c of the line perpendicular to this one, passing through X
#y=mx+b
def perpendicularLine(m1,X):
    m2 = -1/m1
    b2 = X[1]-X[0]*m2
    return m2,b2

def in3d(pt,z=0):
    return [pt[0],pt[1],z]


#returns true if the given point is not in a polygon in the map
def validPoint(polys, pt):
    for p in polys:
        poly = polys[p]
        if len(poly) == 0:
            continue
        if ptInPoly(poly,pt):
            return False
    return True

#http://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect/19550879#19550879
#taken from above
# l1, l2
def find_intersection( p0, p1, p2, p3 ) :

    s10_x = p1[0] - p0[0]
    s10_y = p1[1] - p0[1]
    s32_x = p3[0] - p2[0]
    s32_y = p3[1] - p2[1]

    denom = s10_x * s32_y - s32_x * s10_y

    if denom == 0 : return 'collinear' # collinear

    denom_is_positive = denom > 0

    s02_x = p0[0] - p2[0]
    s02_y = p0[1] - p2[1]

    s_numer = s10_x * s02_y - s10_y * s02_x

    if (s_numer < 0) == denom_is_positive : return None # no collision

    t_numer = s32_x * s02_y - s32_y * s02_x

    if (t_numer < 0) == denom_is_positive : return None # no collision

    if (s_numer > denom) == denom_is_positive or (t_numer > denom) == denom_is_positive : return None # no collision


    # collision detected

    t = t_numer / denom

    intersection_point = [ p0[0] + (t * s10_x), p0[1] + (t * s10_y) ]


    return intersection_point


#https://www.ecse.rpi.edu/Homepages/wrf/Research/Short_Notes/pnpoly.html
#taken from above
# MAGIC !!
def __pnpoly(nvert, vertx, verty, testx, testy):
    i = 0
    j = nvert-1
    c = False
    while i < nvert:
        #body
        if ((verty[i] > testy) != (verty[j]>testy)):
            if (testx < (vertx[j]-vertx[i]) * (testy-verty[i]) / (verty[j]-verty[i]) + vertx[i]):
                c = not c
        #/body
        j = i
        i += 1
    return c

#wrapper for pnpoly
#poly is a list of vectors for edge, pt is a 2D point as a vector
def ptInPoly(poly,pt):
    poly = np.array(poly)
    nvert = len(poly[:,0])
    vertx = poly[:,0]
    verty = poly[:,1]
    testx = pt[0]
    testy = pt[1]
    return __pnpoly(nvert,vertx,verty,testx,testy)

#http://stackoverflow.com/a/2233538
#taken from above
def _dist(x1,y1, x2,y2, x3,y3): # x3,y3 is the point
    px = x2-x1
    py = y2-y1

    something = px*px + py*py

    u =  ((x3 - x1) * px + (y3 - y1) * py) / float(something)

    if u > 1:
        u = 1
    elif u < 0:
        u = 0

    x = x1 + u * px
    y = y1 + u * py

    dx = x - x3
    dy = y - y3

    # Note: If the actual distance does not matter,
    # if you only want to compare what this function
    # returns to other results of this function, you
    # can just return the squared distance instead
    # (i.e. remove the sqrt) to gain a little performance

    dist = np.sqrt(dx*dx + dy*dy)

    return dist

# A,B make the line segment, P is the point
def ptToLineSegment(A,B,P):
    return _dist(A[0],A[1],B[0],B[1],P[0],P[1])

#returns the coordiantes for a point between p1 p2 at percentage away from p1
def tracePoint(p1,p2,percent):
    L = euclid_distance(p1,p2)
    if L < 0.0000001:
        return (p1[0], p1[1])
    l = percent * L
    a = (p2[0]-p1[0]) * (l/L) + p1[0]
    b = (p2[1]-p1[1]) * (l/L) + p1[1]
    return (a,b)

def subdividePath(ideal_path, divs = 1):
    res = [ideal_path[0]]
    for i in range(1,len(ideal_path)):
        mid_pts = []
        for div in range(1,divs+1):
            perc = div/(divs+1.)
            mid_pt = tracePoint(ideal_path[i-1],ideal_path[i],perc)
            mid_pts.append(mid_pt)
        res.extend(mid_pts)
    res.append(ideal_path[-1])
    return res
