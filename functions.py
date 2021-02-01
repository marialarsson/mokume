import numpy as np
import math

def dotprod(v1, v2):
    return sum((a*b) for a, b in zip(v1,v2))

def length(v):
    return math.sqrt(dotprod(v,v))

def angle(v1,v2,radians=False):
    dotp = dotprod(v1, v2)
    lenp = length(v1)*length(v2)
    if round(dotp/lenp,10) in [-1.0,1.0]: ang = 0
    else: ang = math.acos(dotp/lenp)
    if not radians: ang = math.degrees(ang)
    return ang

def normalize(vec):
    return vec/np.linalg.norm(vec)

def get_dist_to_line(p, p0, p1): #p: point, p0: line start point, p1: line end point
    p  = np.array(p)
    p0 = np.array(p0)
    p1 = np.array(p1)
    v0 = p0-p1
    v1 = p-p0
    v2 = p-p1
    a0 = angle(v0,v1)
    a1 = angle(-v0,v2)
    if a0<90 and a1<90: dist = np.linalg.norm(v1)*math.sin(math.radians(a0))
    elif a0>a1: dist = np.linalg.norm(v1)
    else: dist = np.linalg.norm(v2)
    return dist

def get_dist_to_line_2(p, p0, p1): #p: point, p0: line start point, p1: line end point
    p  = np.array(p)
    p0 = np.array(p0)
    p1 = np.array(p1)
    v0 = p1-p0
    v1 = p-p0
    v2 = p-p1


    a = angle(v0,v1)
    if a<90: dist = np.linalg.norm(v1)*math.sin(math.radians(a))
    else: dist = np.linalg.norm(v1)
    return dist

def rotateZ(pts,org,ang,degrees=False):
    if degrees: ang = math.degrees(ang)
    for i in range(len(pts)):
        pts[i] = np.array(pts[i])-np.array(org)
        x0 = pts[i][0]
        y0 = pts[i][1]
        pts[i][0] = x0*math.cos(ang)-y0*math.sin(ang)
        pts[i][1] = x0*math.sin(ang)+y0*math.cos(ang)
        pts[i] = pts[i]+np.array(org)
    return pts
