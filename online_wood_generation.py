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

def dist_array_to_point_cloud(dist_array,res):

    #points = np.argwhere(dist_array%1<0.25)
    points = np.argwhere(dist_array>=0)
    #points = np.argwhere( (dist_array%1<0.25) & (dist_array<20.25) )
    colors = [] # Colors (rainbow color gradient according to year)
    for ind in points:
        t = (dist_array[tuple(ind)]/10)%10
        (r, g, b) = colorsys.hsv_to_rgb(t, 1.0, 1.0)
        colors.append([r,g,b])
    colors = np.array(colors)
    points = points.astype('float64')/(res-1)
    return points,colors

start_time = time.time()

# Resolution of the 3D image
cube_resolution = 32
z0 = 177
zd = 15

# Load parameter files
ppts, ppts_all =  load_pith_points(        "000111", z0,zd)
rpts, rpts_ends = load_outer_shape_points( "000111", ppts_all, z0,zd)
bpts, bpts_all =  load_branch_points(      "000111", ppts_all, z0,zd)


"""
# Create an instance of the stem class
stem = Stem(cube_resolution,randomize=False)

# Create and instance of the branch class
bn = 1
branches = []
for i in range(bn): branches.append(Branch(stem,cube_resolution,randomize=True))

# Smoothly join the stem and the branch
#dist_array = smooth_union_incre(stem.dist_array, branch.dist_array,0.5,5)
dist_array = stem.dist_array
for branch in branches: dist_array = smooth_union(dist_array, branch.dist_array,3)

# Create point cloud
points,colors = dist_array_to_point_cloud(dist_array,cube_resolution)

"""
# Define point cloud for visualization with open3d
open3d.PointCloud = open3d.geometry.PointCloud
"""
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
    vers = vers/(cube_resolution-1)
    #Define mesh
    mesh = open3d.geometry.TriangleMesh()
    mesh.vertices = open3d.utility.Vector3dVector(vers)
    mesh.triangles = open3d.utility.Vector3iVector(tris)
    (r, g, b) = colorsys.hsv_to_rgb((n/10)%10, 1.0, 1.0)
    mesh.paint_uniform_color(np.array([r,g,b]))
    #mesh.compute_vertex_normals()
    meshes.append(mesh)
"""
# Open3D stuff
# Define outlines of the cube for visualization with open3d
points = [[0, 0, 0], [1, 0, 0], [0, 1, 0], [1, 1, 0], [0, 0, 1], [1, 0, 1],[0, 1, 1], [1, 1, 1]]
#points = (cube_resolution-1)*np.array(points)
lines = [[0, 1], [0, 2], [1, 3], [2, 3], [4, 5], [4, 6], [5, 7], [6, 7],[0, 4], [1, 5], [2, 6], [3, 7]]
colors = [[0, 0, 0] for i in range(len(lines))]
line_set = open3d.geometry.LineSet()
line_set.points = open3d.utility.Vector3dVector(points)
line_set.lines = open3d.utility.Vector2iVector(lines)
line_set.colors = open3d.utility.Vector3dVector(colors)

"""
## segemnts
points = []
lines = []
for i,seg in enumerate(branch.segs):
    x = seg.xdomain[0]
    z = seg.a+seg.k*x+branch.t
    points.append([x,0,z])
    x = seg.xdomain[1]
    z = seg.a+seg.k*x+branch.t
    points.append([x,0,z])
    lines.append([2*i,2*i+1])
line_segs = open3d.geometry.LineSet()
line_segs.points = open3d.utility.Vector3dVector(points)
line_segs.lines = open3d.utility.Vector2iVector(lines)
point_cloud_seg_pts = open3d.PointCloud()
point_cloud_seg_pts.points = open3d.utility.Vector3dVector(points)
"""

point_cloud_param = open3d.PointCloud()
param_pts = []
param_pts.extend(ppts_all)
for pts in rpts: param_pts.extend(pts)
for pts in rpts_ends: param_pts.extend(pts)
for pts in bpts: param_pts.extend(pts)
point_cloud_param.points = open3d.utility.Vector3dVector(param_pts)

#print('Resolution:', cube_resolution)
#print('Calculation time:', time.time()-start_time)

geos = []
#geos.extend(meshes)
geos.append(line_set)
#geos.append(line_segs)
#geos.append(point_cloud_seg_pts)
geos.append(point_cloud_param)
#geos.append(point_cloud)


vis = open3d.visualization.Visualizer()
vis.create_window()
for geo in geos: vis.add_geometry(geo)
vis.get_render_option().load_from_json("renderoptions.json")
ctr = vis.get_view_control()
parameters = open3d.io.read_pinhole_camera_parameters("screencamera.json")
ctr.convert_from_pinhole_camera_parameters(parameters)
vis.run()
vis.destroy_window()
