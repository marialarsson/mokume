import numpy as np
import math

def dotprod(v1, v2):
    return sum((a*b) for a, b in zip(v1,v2))

def length(v):
    return math.sqrt(dotprod(v,v))

def angle(v1,v2):
    dotp = dotprod(v1, v2)
    lenp = length(v1)*length(v2)
    if round(dotp/lenp,5) in [-1.0,1.0]: ang = 0
    else: ang = math.degrees(math.acos(dotp/lenp))
    return ang

def get_dist_to_line(p, p0, p1): #p: point, p0: line start point, p1: line end point
    p  = np.array(p)
    p0 = np.array(p0)
    p1 = np.array(p1)
    v0 = p-p0
    v1 = p-p1
    a = angle(v0,v1)
    b = np.linalg.norm(v0)
    return b*math.sin(a)
