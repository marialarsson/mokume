import numpy as np
import random
import open3d
import math
import mcubes
import pymesh
import time

def export_obj(vertices, triangles, filename):
    """
    Exports a mesh in the (.obj) format.
    """

    with open(filename, 'w') as fh:

        for v in vertices:
            rgb = np.array([1.0,0,0])
            v = np.concatenate((v,rgb))
            fh.write("v {} {} {} {} {} {}\n".format(*v))


        for f in triangles:
            fh.write("f {} {} {}\n".format(*(f + 1)))

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
        for i in range(self.res):
            for j in range(self.res):
                for k in range(self.res):
                    self.dist_array[i][j][k] = self.distance_function(i,j,k)

    def distance_function(self,x,y,z):
        self.ln_a = np.array([stem.cx,stem.cy,self.t])     # branch line skeleton start point
        self.ln_b = np.array([stem.cx+1,stem.cy,self.t+0.4])   # branch line skeleton end point
        x/=self.res
        y/=self.res
        z/=self.res
        p = np.array([x,y,z])
        distance = point_to_line_distance(p,self.ln_a,self.ln_b)/self.d
        return distance

def smooth_union(d1,d2,k):
    absd = k-np.absolute(d1-d2)
    absd[absd < 0] = 0.0
    subd = 0.25*np.power(absd,2)/k
    d = np.minimum(d1,d2)-subd
    return d

def smooth_union_one(d1,d2,k):
    absd = k-abs(d1-d2)
    if absd<0: absd = 0.0
    subd = 0.25*math.pow(absd,2)/k
    d = min([d1,d2])-subd
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

start_time = time.time()

# Resolution of the 3D image
cube_resolution = 32

# Create an instance of the stem class
stem = Stem(cube_resolution,randomize=True)

# Create and instance of the branch class
branch = Branch(stem,cube_resolution,randomize=True)

# Smoothly join the stem and the branch
dist_array = smooth_union(stem.dist_array, branch.dist_array,0.1*cube_resolution)

print("Done creating array after", time.time()-start_time)

n_start = int(np.min(dist_array))+1
n_end = int(np.max(dist_array))
print("start and end layers:",n_start,n_end)

for n in range(n_start,n_end):
    loc_dist_array = np.copy(dist_array)-n
    vertices, triangles = mcubes.marching_cubes(loc_dist_array, 0)
    if n==n_start:
        vers = vertices.copy()
        tris = triangles.copy()
    else:
        triangles = triangles+vers.shape[0]
        vers = np.concatenate((vers, vertices), axis=0)
        tris = np.concatenate((tris, triangles), axis=0)
    print("progress:", n)

"""
print("verticis",vers)
print(vers.shape)
print(np.min(vers),"-",np.max(vers))
print("triangles",tris)
print(tris.shape)
print(np.min(tris),"-",np.max(tris))
"""



#export_obj(vers, tris, "wood_3Darray_all.obj")
print("Done exporting array based geometry after", time.time()-start_time)

"""

# Create the volume
def f(x, y, z):
    return x**2 + y**2 + z**2


def myf(x,y,z):
    branch_distance = branch.distance_function(x,y,z)
    stem_distance = stem.distance_function(x,y,z)
    k = 0.1*cube_resolution
    dist = smooth_union_one(branch_distance,stem_distance,k)
    return dist-3

d = myf(50,50,50)


#b0, b1 = -0.1*cube_resolution,0.1*cube_resolution
#s0 = cube_resolution
b0, b1 = -10,10
s0 = 100

# Extract the 16-isosurface
vertices2, triangles2 = mcubes.marching_cubes_func(
        (b0,b0,b0), (b1,b1,b1),  # Bounds
        s0, s0, s0, # Number of samples in each dimension
        myf,                          # Implicit function
        16)                         # Isosurface value

# Export the result to sphere2.dae
mcubes.export_obj(vertices2, triangles2, "wood_function.obj")
print("Done exporting function based geometry after", time.time()-start_time)
"""
