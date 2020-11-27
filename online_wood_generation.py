import numpy as np
import random
import open3d
import math
import colorsys

def point_to_line_distance(p, a, b):
    d = np.divide(b - a, np.linalg.norm(b - a)) # normalize tangent vector
    s = np.dot(a - p, d) # signed parallel distance components
    t = np.dot(p - b, d)
    h = np.maximum.reduce([s, t, 0]) # clamped parallel distance
    c = np.cross(p - a, d)     # perpendicular distance component
    dist = np.hypot(h, np.linalg.norm(c))
    return dist

class Stem:
    def __init__(self, res, randomize=False, d=0.1, cx=0.5, cy=0.5):
        self.res = res
        if randomize:
            self.d = random.uniform(0.08,0.12)      # distance between growth surface
            self.cx = random.uniform(-0.25,1.25)    # skeleton center point x value
            self.cy = random.uniform(-0.25,1.25)    # skeleton center point y value
        else:
            self.d = d
            self.cx = cx
            self.cy = cy
        self.create_3D_distance_array()

    def create_3D_distance_array(self):
        self.dist_array = np.zeros((self.res,self.res,self.res))
        for i in range(self.res):
            for j in range(self.res):
                for k in range(self.res):
                    self.dist_array[i][j][k] = self.distance_function(i,j,k)

    def distance_function(self,x,y,z):
        x/=self.res
        y/=self.res
        z/=self.res
        distance = math.sqrt( (x-self.cx)**2 + (y-self.cy)**2 )/self.d
        return distance

class Branch:
    def __init__(self, stem, res, d=0.03, t=0.5, alfa=0.0, randomize=False):
        self.stem=stem
        self.res = res
        if randomize:
            self.d = random.uniform(0.02,0.04)      # distance between growth surface
            self.t = random.uniform(0.2,0.6)        # position along main skeleton
            #self.alfa = rand.uniform(0.0,360.0)     # rotation around the skeleton (0.0 is x-axis aligned and so on)
        else:
            self.d = d
            self.t = t
            #self.alfa = alfa
        self.create_3D_distance_array()

    def create_3D_distance_array(self):
        self.dist_array = np.zeros((self.res,self.res,self.res))
        ln_a = np.array([stem.cx,stem.cy,self.t])     # branch line skeleton start point
        ln_b = np.array([stem.cx+1,stem.cy,self.t+0.4])   # branch line skeleton end point
        for i in range(self.res):
            for j in range(self.res):
                for k in range(self.res):
                    self.dist_array[i][j][k] = self.distance_function(i,j,k,ln_a,ln_b)

    def distance_function(self,x,y,z,a,b):
        x/=self.res
        y/=self.res
        z/=self.res
        p = np.array([x,y,z])
        distance = point_to_line_distance(p,a,b)/self.d
        return distance

def smooth_union(d1,d2,k):
    absd = k-np.absolute(d1-d2)
    absd[absd < 0] = 0.0
    subd = 0.25*np.power(absd,2)/k
    d = np.minimum(d1,d2)-subd
    return d

def dist_array_to_point_cloud(dist_array):
    points = np.argwhere(dist_array%1<0.25)
    colors = [] # Colors (rainbow color gradient according to year)
    for ind in points:
        t = (dist_array[tuple(ind)]/10)%10
        (r, g, b) = colorsys.hsv_to_rgb(t, 1.0, 1.0)
        colors.append([r,g,b])
    colors = np.array(colors)
    return points,colors


# Resolution of the 3D image
cube_resolution = 128

# Create an instance of the stem class
stem = Stem(cube_resolution,randomize=True)

# Create and instance of the branch class
branch = Branch(stem,cube_resolution,randomize=True)

# Smoothly join the stem and the branch
dist_array = smooth_union(stem.dist_array, branch.dist_array,0.1*cube_resolution)

# Create point cloud
points,colors = dist_array_to_point_cloud(dist_array)

# Define point cloud for visualization with open3d
open3d.PointCloud = open3d.geometry.PointCloud
point_cloud = open3d.PointCloud()
point_cloud.points = open3d.utility.Vector3dVector(points)
point_cloud.colors = open3d.utility.Vector3dVector(colors)
#open3d.visualization.draw_geometries([point_cloud])

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
open3d.visualization.draw_geometries([point_cloud,line_set])
