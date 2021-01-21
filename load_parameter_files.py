import math
import numpy as np
from functions import *

def get_pts_and_rads_from_parameters(org,lines,i):
    z_num = int(len(lines)/360)
    pts = []
    rads = []
    for j in range(360):
        rad = 0.1*float(lines[i+j*z_num].split("\n")[0]) #millimeter to centimeter
        ang = math.radians(j)
        x = rad*math.cos(ang)+org[0]
        y = rad*math.sin(ang)+org[1]
        z = org[2]
        pts.append([x,y,z])
        rads.append(rad)
    return pts,rads

def load_pith_points(tree_name,z0,zd):
    path = "param_data\\"+tree_name+"pith.txt"
    file = open(path, "r")
    lines = file.readlines()
    z_num = int(len(lines)/3)
    pts_range = []
    pts_all = []
    for i in range(z_num):
        x = 0.1*float(lines[i].split("\n")[0])          # millimeter to centimeter
        y = -0.1*float(lines[i+z_num].split("\n")[0])   # millimeter to centimeter
        if i==0: x0, y0 = x, y
        x=x-x0
        y=y-y0
        z=i
        pt = [x,y,z]
        if z>z0 and z<(z0+zd): pts_range.append(pt)
        pts_all.append(pt)
    return pts_range, pts_all
    x = rad*math.cos(ang)-org[0]
    y = rad*math.sin(ang)-org[1]
    z = org[2]
    return [x,y,z]

def load_outer_shape_points(tree_name, pith_pts, z0, zd, depth_step=1):
    path = "param_data\\"+tree_name+"radii.txt"
    file = open(path, "r")
    lines = file.readlines()

    ## Range
    pts_range = []
    rads_range = []
    for i in range(z0,z0+zd,depth_step):
        pts,rads = get_pts_and_rads_from_parameters(pith_pts[i],lines,i)
        pts_range.append(pts)
        rads_range.append(rads)

    ## Ends
    pts_ends = []
    for i in range(2):
        if i>0: i=len(pith_pts)-1 #start/end
        pts,_ = get_pts_and_rads_from_parameters(pith_pts[i],lines,i)
        pts_ends.append(pts)

    return pts_range, rads_range, pts_ends

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

def load_branch_points(tree_name,pith_pts,z0,zd,step_size=0.1):
    path = "param_data\\"+tree_name+"branches.txt"
    file = open(path, "r")
    lines = file.read().split("\n")
    k_num = len(lines)
    branch_pts_range = []
    branch_rads_range = []
    branch_pts_all = []

    for i in range(k_num):
        k_info = lines[i].split("\t")
        A,B,C,D,E,F,G,H,I,J,K =  [float(item) for item in k_info]
        rp_max = 0.1*I #mm to cm
        rp_num = round(rp_max/step_size)+1
        rp_step = rp_max/(rp_num-1)
        pts = []
        rads = []
        x_org,y_org,z_org = pith_pts[round(G)]
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
        pts,rads = reduce_number_of_points(pts,rads,0.1)
        if len(pts)<2:
            print("Skipped a very short branch")
            continue
        extra_z = 2.0 #arbitrary... to catch branches below and about current z-range
        if z_org>=(z0-2*extra_z) and z_org<=(z0+zd+extra_z):
            branch_pts_range.append(pts)
            branch_rads_range.append(rads)
        branch_pts_all.append(pts)

    #reduce number of points


    return branch_pts_range, branch_rads_range, branch_pts_all
