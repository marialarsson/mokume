import math
import numpy as np
import random
import itertools
from functions import *

class Knot:
    def __init__(self, box, para, stem, i): #t0.1
        self.box = box
        self.para = para
        self.stem = stem
        self.pts = para.kpts[i]
        #print(np.array(self.pts).shape)
        self.rads = para.krads[i]
        self.vec = np.array(self.pts[-1])-np.array(self.pts[0])
        self.org = self.pts[0]
        self.d = 0.05
        self.create_3D_distance_array()
        print("Knot", str(i)+"/"+str(len(self.para.kpts)))

    def create_3D_distance_array(self):
        self.dist_array = np.zeros(self.box.res+1, dtype='float64')+999
        for ind in self.box.inds:
            pt = self.box.abs_pts[tuple(ind)]
            self.dist_array[tuple(ind)] = self.distance_function(pt)



    def distance_function(self,pt):
        pt_vec = pt-np.array(self.pts[0])
        ang = angle(self.vec,pt_vec)
        len_ratio = np.linalg.norm(pt_vec)/np.linalg.norm(self.vec)
        max_ang = 25+(1-len_ratio)*50
        if ang>max_ang or len_ratio>1.5: distance=999.0 #behind branch
        else: distance = point_to_points_distance(pt,self.pts,self.rads,self.para.yrs,self.d)
        return distance

#FUNCTIONS
def get_point_in_best_direction(pt, pts, i):
    if i==0: index = 1
    if i==len(pts)-1: index = len(pts)-2
    else:
        pt = np.array(pt)
        pt_i = np.array(pts[i])
        pt_0 = np.array(pts[i-1])
        pt_1 = np.array(pts[i+1])
        v0 = normalize(pt_0-pt_i)
        v1 = normalize(pt_1-pt_i)
        d0 = np.linalg.norm(pt-pt_i-v0)
        d1 = np.linalg.norm(pt-pt_i-v1)
        if d0<d1: index = i-1
        else: index = i+1
    return pts[index]

def point_to_points_distance(pt,pts,rads,yrs,d):
    #find the two closest points
    D = [] #make a list of all distances
    for i in range(len(pts)): D.append(np.linalg.norm(pt-pts[i]))
    closest_i = np.argmin(np.array(D))
    p0 = pts[closest_i]
    p1 = get_point_in_best_direction(pt,pts,closest_i)
    # get distnace to line between the two closest points points
    dist = get_dist_to_line_2(pt, p0, p1)
    dist = dist/d
    return dist
