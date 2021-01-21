import math
import numpy as np
import itertools

class Stem:
    def __init__(self, dwh, dim, ppts, rads, yrs):
        self.dwh = dwh #depth, width, height
        self.dim = dim
        self.ppc = dim/dwh
        self.ppts = ppts
        self.rads = rads
        self.yrs = yrs
        self.org = ppts[0]
        self.create_3D_distance_array()

    def create_3D_distance_array(self):
        ind_list = [i for i in range(self.dim)]
        point_inds = list(itertools.product(ind_list,repeat=3))
        dist_array=[]
        for ind in point_inds: dist_array.append(self.distance_function(np.array(ind),self.rads))
        self.dist_array = np.reshape(np.array(dist_array),(self.dim,self.dim,self.dim))

    def distance_function(self,pt,rads):
        pt = pt/self.ppc
        x,y,z = pt
        radi = int(pt[2])
        ratio = pt[2]-radi
        dist = 999
        if y==0: alfa=0
        else: alfa = math.atan(x/y)
        r_max = (1-ratio)*rads[radi][round(alfa)]+ratio*rads[radi+1][round(alfa)]
        r = math.sqrt(x**2+y**2)
        if r<r_max:
            d = r_max/self.yrs
            dist = r/d
        return dist
