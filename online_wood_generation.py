import numpy as np
import random
import open3d
import math
import colorsys
import sympy
import time
import itertools
import mcubes
import pymesh
from stem import Stem
from branch import Branch
from load_parameter_files import *

def smooth_union(d1,d2,k):
    #Smoothly joining 2 distnace fields
    absd = k - np.absolute(d1-d2)
    absd[absd<0] = 0.0
    subd = 0.25*np.power(absd,2)/k
    d = np.minimum(d1,d2)-subd
    d[d<0] = 0
    return d

def smooth_union_incre(d1,d2,kmin,kmax):
    #maxrunda
    k = 5.0
    absd = k - np.absolute(d1-d2)
    absd[absd<0] = 0.0
    subd = 0.25*np.power(absd,2)/k
    d = np.minimum(d1,d2)-subd
    d[d<0] = 0
    #runda
    k = 1.5 + 0.5*d
    absd = k - np.absolute(d1-d2)
    absd[absd<0] = 0.0
    subd = 0.25*np.power(absd,2)/k
    d = np.minimum(d1,d2)-subd
    d[d<0] = 0
    return d

def dist_array_to_point_cloud(dist_array,org,ppc,yrs):
    #inds = np.argwhere(dist_array%1<0.25)
    #inds = np.argwhere(dist_array>=0)
    #inds = np.argwhere(dist_array<=yrs)
    inds = np.argwhere( (dist_array%1<0.25) & (dist_array<=yrs) )
    points = []
    colors = [] # Colors (rainbow color gradient according to year)
    org = np.array(org)
    for ind in inds:
        points.append(ind/ppc+org) #+org
        t = (dist_array[tuple(ind)]/yrs)%yrs
        (r, g, b) = colorsys.hsv_to_rgb(t, 1.0, 1.0)
        colors.append([r,g,b])
    points = np.array(points)
    points = points.astype('float64')
    colors = np.array(colors)
    return points,colors

start_time = time.time()

# Resolution of the 3D image
cube_side_length = 5 #cm
cube_resolution = 30 # points per side
x0 = y0 = 0
z0 = 167 #cm
yrs = 10

# Load parameter files
zd = cube_side_length+1
ppts, ppts_all =  load_pith_points(        "000111", z0, zd)
rpts, rads, rpts_ends = load_outer_shape_points( "000111", ppts_all, z0, zd)
bpts, brads, bpts_all =  load_branch_points(      "000111", ppts_all, z0, zd)

# Create an instance of the stem class
stem = Stem(cube_side_length, cube_resolution, ppts, rads, yrs)

# Create and instance of the branch class
branches = []
for pts,rads in zip(bpts,brads): branches.append(Branch(stem,pts,rads))

# Smoothly join the stem and the branch
#dist_array = smooth_union_incre(stem.dist_array, branch.dist_array, 0.5, 5)
#dist_array = branches[0].dist_array
dist_array = stem.dist_array
#for branch in branches: dist_array = smooth_union(dist_array, branch.dist_array, 20)

# Create point cloud
points,colors = dist_array_to_point_cloud(dist_array,stem.org,stem.ppc,stem.yrs)

# Define point cloud for visualization with open3d
open3d.PointCloud = open3d.geometry.PointCloud

point_cloud = open3d.PointCloud()
point_cloud.points = open3d.utility.Vector3dVector(points)
point_cloud.colors = open3d.utility.Vector3dVector(colors)


# mesh stuff
meshes = []
n_start = int(np.min(dist_array))+1
n_end = int(np.max(dist_array))
for n in range(n_start,n_end):
    loc_dist_array = np.copy(dist_array)-n
    vers, tris = mcubes.marching_cubes(loc_dist_array, 0)
    vers = vers/stem.ppc + stem.org
    #Define mesh
    mesh = open3d.geometry.TriangleMesh()
    mesh.vertices = open3d.utility.Vector3dVector(vers)
    mesh.triangles = open3d.utility.Vector3iVector(tris)
    (r, g, b) = colorsys.hsv_to_rgb((n/10)%10, 1.0, 1.0)
    mesh.paint_uniform_color(np.array([r,g,b]))
    #mesh.compute_vertex_normals()
    meshes.append(mesh)


# Open3D stuff
# Define outlines of the cube for visualization with open3d
L = cube_side_length
points = np.array(list(itertools.product([0,1],repeat=3)))
points = points*L+stem.org
#[[0, 0, z0], [L, 0, z0], [0, L, z0], [L, L, z0], [0, 0, z1], [L, 0, z1],[0, L, z1], [L, L, z1]]
#points = (cube_resolution-1)*np.array(points)
lines = [[0, 1], [0, 2], [1, 3], [2, 3], [4, 5], [4, 6], [5, 7], [6, 7], [0, 4], [1, 5], [2, 6], [3, 7]]
colors = [[0, 0, 0] for i in range(len(lines))]
line_set = open3d.geometry.LineSet()
line_set.points = open3d.utility.Vector3dVector(points)
line_set.lines = open3d.utility.Vector2iVector(lines)
line_set.colors = open3d.utility.Vector3dVector(colors)

point_cloud_param = open3d.PointCloud()
param_pts = []
param_pts.extend(ppts_all)
param_pts.extend(rpts[0])
param_pts.extend(rpts[-1])
for pts in rpts_ends: param_pts.extend(pts)
for pts in bpts_all: param_pts.extend(pts)
point_cloud_param.points = open3d.utility.Vector3dVector(param_pts)

points = []
lines = []
cnt=0
for pts in bpts_all:
    points.extend(pts)
    for i in range(len(pts)-1): lines.append([cnt+i,cnt+i+1])
    cnt+=len(pts)
line_branches = open3d.geometry.LineSet()
line_branches.points = open3d.utility.Vector3dVector(points)
line_branches.lines = open3d.utility.Vector2iVector(lines)

#print('Resolution:', cube_resolution)
#print('Calculation time:', time.time()-start_time)

geos = []
geos.extend(meshes)
geos.append(line_set)
geos.append(point_cloud_param)
#geos.append(point_cloud)
geos.append(line_branches)


vis = open3d.visualization.Visualizer()
vis.create_window()
for geo in geos: vis.add_geometry(geo)
vis.get_render_option().load_from_json("renderoptions.json")
ctr = vis.get_view_control()
parameters = open3d.io.read_pinhole_camera_parameters("screencamera.json")
ctr.convert_from_pinhole_camera_parameters(parameters)
vis.run()
vis.destroy_window()
