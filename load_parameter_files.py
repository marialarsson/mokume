import math


def get_pts_from_parameters(org,lines,i,degree_step,z_num):
    pts = []
    for j in range(0,360,degree_step):
        rad = 0.1*float(lines[i+j*z_num].split("\n")[0]) #millimeter to centimeter
        ang = math.radians(j)
        x = rad*math.cos(ang)+org[0]
        y = rad*math.sin(ang)+org[1]
        z = org[2]
        pts.append([x,y,z])
    return pts


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

def load_outer_shape_points(tree_name, pith_pts, z0, zd, depth_step=1, degree_step=3):
    path = "param_data\\"+tree_name+"radii.txt"
    file = open(path, "r")
    lines = file.readlines()
    z_num = int(len(lines)/360)

    ## Range
    pts_range = []
    for i in range(z0,z0+zd,depth_step):
        pts_range.append(get_pts_from_parameters(pith_pts[i],lines,i,degree_step,z_num))

    ## Ends
    pts_ends = []
    for i in range(2):
        if i>0: i=len(pith_pts)-1 #start/end
        pts_ends.append(get_pts_from_parameters(pith_pts[i],lines,i,degree_step,z_num))

    return pts_range, pts_ends

def load_branch_points(tree_name,pith_pts,z0,zd,step_size=2):
    path = "param_data\\"+tree_name+"branches.txt"
    file = open(path, "r")
    lines = file.read().split("\n")
    k_num = len(lines)
    branch_pts_range = []
    branch_pts_all = []

    for i in range(k_num):
        k_info = lines[i].split("\t")
        A,B,C,D,E,F,G,H,I,J,K =  [float(item) for item in k_info]
        rp_max = I
        rp_num = round(rp_max/step_size)
        rp_step = rp_max/(rp_num-1)
        pts = []
        rads = []
        x_org,y_org,z_org = pith_pts[round(G)]
        #z_org = G
        for j in range(rp_num):
            ##point
            rp = rp_step*j
            z = z_org+0.1*(G+H*math.sqrt(rp)) #millimeter to centimeter
            if j==0: om=C
            else: om=C+D*math.log(rp)
            om = math.radians(om)
            x = x_org+0.1*rp*math.cos(om) #millimeter to centimeter
            y = y_org+0.1*rp*math.sin(om) #millimeter to centimeter
            pts.append([x,y,z])
            ## calculate d??
            ##crosssection
            #rad = 0.5*(A+B*math.pow(rp,0.25))*rp*scale
            #if rad<=0: rad=0.01
            #rads.append(rad)
        extra_z = 20.0 #arbitrary... to catch branches below and about current z-range
        print(z_org,z0,zd)
        if z_org>=(z0-extra_z) and z_org<=(z0+zd+extra_z):
            print("yes")
            branch_pts_range.append(pts)
        branch_pts_all.append(pts)

    return branch_pts_range,branch_pts_all
