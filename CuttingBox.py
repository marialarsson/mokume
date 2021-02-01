import math
import numpy as np
from functions import *

class CuttingBox:
    def __init__(self, dim, ppc, pos, rot=0, exterior_only=False, org_ctr=True, ctr_xy=True):
        self.dim = np.array(dim)
        self.ppc = ppc
        self.res = self.dim*ppc
        self.exterior_only = exterior_only
        self.pos = pos
        if org_ctr: self.pos -= np.array([0.5*dim[0], 0.5*dim[1], 0]) #centering
        self.z0 = pos[2]
        self.z1 = pos[2]+dim[2]
        self.rot = math.radians(rot)
        #self.loc_rot = math.radians(loc_rot)
        self.ctr_xy = ctr_xy
        self.abs_pts = np.zeros(np.append(self.res+1,[3]),dtype=np.float)
        if self.exterior_only: self.inds = np.pad(np.ones(self.res-1), ((1, 1),(1, 1),(1, 1)), 'constant', constant_values=((0, 0),(0, 0),(0, 0)))
        else: self.inds = np.zeros(self.res+1)
        self.inds = np.argwhere(self.inds==0)

    def update_xy_pos(self,org):
        self.org = org

        if self.ctr_xy:
            self.pos[0]+=org[0]
            self.pos[1]+=org[1]

        #self.loc_org = self.pos #-0.5*self.dim
        #self.loc_org = rotateZ([self.loc_org],self.org,self.rot)[0]

        for ind in self.inds:

            #scale
            pt = np.array(ind)/self.ppc

            ### move
            pt += self.pos

            ### rotate around global z-axis (ppts[0])
            pt = rotateZ([pt],self.org,self.rot)[0]

            ### rotate around local z_axis (center or box)
            #pt = rotateZ([pt],self.loc_org,self.loc_rot)[0]

            ### assign
            self.abs_pts[tuple(ind)] = pt
