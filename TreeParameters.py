import math
import numpy as np
from functions import *

class TreeParameters:
    def __init__(self, box, name, yrs):
        self.box = box
        self.name = name
        self.yrs = yrs
        zd = box.dim[2]+1
        self.load_pith_points()
        self.box.update_xy_pos(self.ppts[0])
        self.load_outer_shape_points()
        self.load_branch_points()

    def load_pith_points(self):
        path = "param_data\\"+self.name+"pith.txt"
        file = open(path, "r")
        lines = file.readlines()
        z_num = int(len(lines)/3)
        self.ppts = []
        self.ppts_all = []
        for i in range(z_num):
            x = 0.1*float(lines[i].split("\n")[0])          # millimeter to centimeter
            y = -0.1*float(lines[i+z_num].split("\n")[0])   # millimeter to centimeter
            if i==0: x0, y0 = x, y
            pt = [x-x0,y-y0,i]
            if pt[2]>=self.box.z0 and pt[2]<=(self.box.z1): self.ppts.append(pt)
            self.ppts_all.append(pt)
        #x = rad*math.cos(ang)-org[0]
        #y = rad*math.sin(ang)-org[1]
        #z = org[2]

    def load_outer_shape_points(self):
        path = "param_data\\"+self.name+"radii.txt"
        file = open(path, "r")
        lines = file.readlines()
        z_num = int(len(lines)/360)

        #get radii
        self.rads = []
        smooth_v = 2
        smooth_h = 5
        st = self.box.z0-smooth_v
        en = self.box.z1+smooth_v+1
        for i in range(st,en):
            rads = []
            for j in range(360): rads.append(0.1*float(lines[i+j*z_num].split("\n")[0])) #millimeter to centimeter
            self.rads.append(rads)
        self.rads = smoothen_rads(self.rads,smooth_v,smooth_h)
        for i in range(smooth_v):
            self.rads.pop(0)
            self.rads.pop(-1)

        ### #get points
        self.rpts = []
        for i in range(len(self.rads)):
            pts = []
            for j in range(360):
                org = self.ppts[i]
                rad = self.rads[i][j]
                pts.append(point_from_rad(org,rad,j))
            self.rpts.append(pts)

        ## Ends (just for display)
        self.rpts_ends = []
        for i in range(1):
            if i>0: i=len(self.ppts)-1 #start/end
            rads = []
            for j in range(360):
                rads.append(0.1*float(lines[i+j*z_num].split("\n")[0])) #millimeter to centimeter
            pts = []
            for j in range(360): pts.append(point_from_rad(self.ppts[i],rads[j],j))
            self.rpts_ends.append(pts)


    def load_branch_points(self,step_size=0.03):
        path = "param_data\\"+self.name+"branches.txt"
        file = open(path, "r")
        lines = file.read().split("\n")
        k_num = len(lines)
        self.kpts = []
        self.krads = []
        self.kpts_all = []

        for i in range(k_num):
            k_info = lines[i].split("\t")
            A,B,C,D,E,F,G,H,I,J,K =  [float(item) for item in k_info]
            rp_max = 0.1*I #mm to cm
            rp_num = round(rp_max/step_size)+1
            rp_step = rp_max/(rp_num-1)
            pts = []
            rads = []
            x_org,y_org,z_org = self.ppts_all[round(G)]
            z_org = G
            for j in range(rp_num):
                ##point
                rp = rp_step*j
                z = z_org+H*math.sqrt(rp)
                if j==0: om=C
                else: om=C+D*math.log(rp)
                om = math.radians(om)
                x = x_org+rp*math.cos(om)
                y = y_org+rp*math.sin(om)
                pts.append([x,y,z])
                ##crosssection
                rad = 0.5*(A+B*math.pow(rp,0.25))*rp #*scale
                if rad<=0: rad=0.01
                rads.append(rad)
            pts,rads = reduce_number_of_points(pts,rads,0.5)
            if len(pts)<2:
                print("Skipped a very short branch")
                continue
            #if len(pts[0])<3: print(pts)
            extra_z = 2.0 #arbitrary... to catch branches below and about current z-range
            if z_org>=(self.box.z0-2*extra_z) and z_org<=(self.box.z1+extra_z):
                self.kpts.append(pts)
                self.krads.append(rads)
            self.kpts_all.append(pts)
        self.knots_no = len(self.kpts)

### FUNCTIONS ###

def reduce_number_of_points(pts,rads,dd):
    change = True
    #print("start len",len(pts))
    while change:
        new_pts = []
        new_rads = []
        for i in range(len(pts)-2):
            if i%2==1: continue #even only
            p0, p1, p2 = pts[i], pts[i+1], pts[i+2]
            dist = get_dist_to_line(p1,p0,p2)
            new_pts.append(p0)
            new_rads.append(rads[i])
            if dist>dd:
                new_pts.append(p1)
                new_rads.append(rads[i+1])
        new_pts.append(pts[-1])
        new_rads.append(rads[-1])
        if len(new_pts)==len(pts): change=False
        pts = list(new_pts)
        rads = list(new_rads)
    #print("end len",len(pts))
    return pts,rads

def smoothen_rads(R,n0,n1):
    R_new = []
    for i in range(len(R)):
        rads_new = []
        for j in range(360):
            near_rads = []
            for a in range(-n0,n0+1):
                for b in range(-n1,n1+1):
                    ii = (i+a)%len(R)
                    jj = (j+b)%360
                    near_rads.append(R[ii][jj])
            rad = sum(near_rads)/len(near_rads)
            rads_new.append(rad)
        R_new.append(rads_new)
    return R_new

def point_from_rad(org,rad,ang):
    ang = math.radians(ang)
    x = rad*math.cos(ang)+org[0]
    y = rad*math.sin(ang)+org[1]
    z = org[2]
    return [x,y,z]
