import math
import numpy as np
import itertools

class Stem:
    def __init__(self, box, para):
        self.box = box
        self.para = para
        self.create_3D_distance_array()

    def create_3D_distance_array(self):
        self.dist_array = np.zeros(self.box.res+1, dtype='float64')
        for ind in self.box.inds:
            pt = self.box.abs_pts[tuple(ind)]
            self.dist_array[tuple(ind)] = self.distance_function(pt)

    def distance_function(self,pt):
        zf = pt[2]-self.box.pos[2]
        zi = int(zf)
        ratio = zf-zi
        rel_pt = pt-self.para.ppts[zi]
        x,y,z = rel_pt
        if y==0: omega=0
        else: omega = math.degrees(math.atan(x/y))
        r0 = self.para.rads[zi][round(omega)]
        #r1 = self.para.rads[radi+1][round(omega)]
        #r_max = ratio*r0+(1-ratio)*r1
        #d = r_max/(self.para.yrs-1)
        d = r0/(self.para.yrs-1) #temp
        return math.sqrt(x**2+y**2)/d
