import math
import numpy as np
import random
import itertools
from functions import *

def point_to_points_distance(pt,pts,rads,yrs):
    #find the two closest points
    D = [] #make a list of all distances
    for i in range(len(pts)): D.append(np.linalg.norm(pt-pts[i]))
    D = np.array(D)
    idx = np.argpartition(D, 2) # get the indices of the two closests points
    # get distnace to line between the two closest points points
    rad = get_dist_to_line(pt, pts[idx[0]], pts[idx[1]])
    # interpolate max radius ... later
    #ratio = np.linalg.norm(np.array(pts[i0])-np.array(pt))/np.linalg.norm(np.array(pts[i0])-np.array(pts[i1]))
    #rad_max = ratio*rads[i0] + (1-ratio)*rads[i1]
    #rad_max = rads[idx[0]] #update later
    #d = rad_max/yrs
    #distance = rad/d
    distance = rad/0.5
    return distance

class Branch:
    def __init__(self, stem, pts, rads): #t0.1
        self.s = stem
        self.pts = pts
        self.rads = rads
        self.vec = np.array(pts[-1])-np.array(pts[0])
        self.org = pts[0]
        self.create_3D_distance_array()


    def create_3D_distance_array(self):
        ind_list = [i for i in range(self.s.dim)]
        point_inds = list(itertools.product(ind_list,repeat=3))
        dist_array=[]
        for ind in point_inds: dist_array.append(self.distance_function(ind))
        self.dist_array = np.reshape(np.array(dist_array),(self.s.dim,self.s.dim,self.s.dim))

    def distance_function(self,pt):
        pt = np.array(pt)/self.s.ppc+self.s.org
        pt_vec = pt-np.array(self.pts[0])
        ang = angle(self.vec,pt_vec)
        if ang>60: distance=999.0 #behind branch
        else: distance = point_to_points_distance(pt,self.pts,self.rads,self.s.yrs)
        return distance
