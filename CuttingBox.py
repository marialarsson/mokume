import math
import numpy as np
import itertools

class CuttingBox:
    def __init__(self, dim, ppc, pos, rot, loc_rot=0, ctr_xy=False):
        self.dim = np.array(dim)
        self.ppc = ppc
        self.res = self.dim*ppc
        self.pos = pos
        self.z0 = pos[2]
        self.z1 = pos[2]+dim[2]
        self.rot = rot
        self.loc_rot = loc_rot
        self.abs_pts = np.zeros(np.append(self.res+1,[3]),dtype=np.float)
        self.inds = np.zeros(self.res+1)
        self.inds = np.argwhere(self.inds==0)
        for ind in self.inds: self.abs_pts[tuple(ind)] = self.pos+np.array(ind)/self.ppc
