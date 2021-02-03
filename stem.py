import math
import numpy as np
from functions import *

class Stem:
    def __init__(self, box, para):
        self.box = box
        self.para = para
        self.create_3D_distance_array()
        print("---")
        print("Stem")

    def create_3D_distance_array(self):
        self.dist_array = np.zeros(self.box.res+1, dtype='float64')+999
        self.dd = np.zeros(self.box.res+1, dtype='float64')+999
        for ind in self.box.inds:
            pt = self.box.abs_pts[tuple(ind)]
            self.dist_array[tuple(ind)],self.dd[tuple(ind)] = self.distance_function(pt)

    def distance_function(self,pt):
        zf = pt[2]-self.box.pos[2]
        zi = int(zf)
        ratio = zf-zi
        ###
        ppt0 = np.array(self.para.ppts[zi])
        if (zi+1)<len(self.para.ppts): ppt1 = np.array(self.para.ppts[zi+1])
        else: ppt1 = ppt0
        ppt = (1-ratio)*ppt0+ratio*ppt1
        ###
        rel_pt = rotateZ([pt],self.box.org,-self.box.rot)[0]
        #rel_pt = rotateZ([rel_pt],self.box.loc_org,-self.box.loc_rot)[0]
        rel_pt = rel_pt-ppt
        x,y,z = rel_pt
        ###
        if x==0 and y>=0: omega=90
        elif x==0 and y<0: omega=270
        else: omega = math.degrees(math.atan(y/x))
        if x<0: omega+=180
        omega = (omega+360)%360
        if round(omega)==360: omega=0
        r0 = self.para.rads[zi][round(omega)]
        if (zi+1)<len(self.para.rads): r1 = self.para.rads[zi+1][round(omega)]
        else: r1=r0
        r_max = (1-ratio)*r0+ratio*r1
        ###
        d = r_max/(self.para.yrs-1)
        dist_to_pith = math.sqrt(x**2+y**2)
        return dist_to_pith/d,d
