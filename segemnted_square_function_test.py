import numpy as np
import random
import open3d
import math
import colorsys

class LineSegment:
    def __init__(self, p0, p1):
        dx = p1[0]-p0[0]
        dy = p1[1]-p0[1]
        if dx!=0: self.k = dy/dx
        else: self.k = 0
        self.xdomain = [p0[0],p1[0]]
        self.a = p0[1]-self.k*p0[0]

cube_resolution = 64
a=3

# the real graph GREEN
cont_pts = []
cont_cols = []
for i in range(100):
    cont_pts.append([i,a*math.sqrt(i),0])
    cont_cols.append([0,255,0])

pts = [[0.0001,0,0]]
cols = [[255,0,0]]
#the approximated graph RED

ss = 1.00 #step size
col = cols[0]
for i in range(100):
    x0=pts[-1][0] #last x
    k = a*0.5*x0**(-0.5) #derivative
    dx = math.sqrt(ss**2/(1+k**2)) #difference in x
    x = x0+dx #new x
    y = a*math.sqrt(x) #new y
    ##
    pts.append([x,y,0])
    cols.append(col)

dd = 0.05
for iter in range(10):
    change = False
    new_pts = [pts[0]]
    for i in range(len(pts)-2):
        if i%2==1:
            continue #even only
        p0 = pts[i]
        p1 = pts[i+1]
        p2 = pts[i+2]
        ##
        dx = p2[0]-p0[0]
        dy = p2[1]-p0[1]
        k = dy/dx
        La = p0[1]-k*p0[0]
        ## line function is L(x)=La+k*x
        ## find line closest point for p1
        k2 = -1/k
        L2a = p1[1]-k2*p1[0]
        ix = (L2a-La)/(k-k2) #intersection x
        iy = La+k*ix #intersection y
        pi = [ix,iy,0] # intersection point, i.e. line closest point
        ##
        diff_vec = np.array(p1)-np.array(pi)
        dist = np.linalg.norm(diff_vec)
        ##
        new_pts.append(p0)
        if dist>dd:
            change=True
            new_pts.append(p1)
    new_pts.append(pts[-1])
    pts=list(new_pts)
    if not change: break

segs = []
for i in range(len(pts)-1):
    segs.append(LineSegment(pts[i],pts[i+1]))

# Define point cloud for visualization with open3d
open3d.PointCloud = open3d.geometry.PointCloud
point_cloud = open3d.PointCloud()
point_cloud.points = open3d.utility.Vector3dVector(pts)
point_cloud.colors = open3d.utility.Vector3dVector(cols)


# Define outlines of the cube for visualization with open3d
points = [[0, 0, 0], [1, 0, 0], [0, 1, 0], [1, 1, 0], [0, 0, 1], [1, 0, 1],[0, 1, 1], [1, 1, 1]]
points= cube_resolution*np.array(points)
lines = [[0, 1], [0, 2], [1, 3], [2, 3], [4, 5], [4, 6], [5, 7], [6, 7],[0, 4], [1, 5], [2, 6], [3, 7]]
colors = [[0, 0, 0] for i in range(len(lines))]
line_set = open3d.geometry.LineSet()
line_set.points = open3d.utility.Vector3dVector(points)
line_set.lines = open3d.utility.Vector2iVector(lines)
line_set.colors = open3d.utility.Vector3dVector(colors)


# Draw with open3d

#open3d.visualization.draw_geometries([point_cloud,point_cloud2,line_set])
open3d.visualization.draw_geometries([point_cloud,line_set])
