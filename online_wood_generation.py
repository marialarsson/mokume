import numpy as np
import random
import open3d
import math
import colorsys
import time
import itertools
import mcubes
from CuttingBox import CuttingBox
from TreeParameters import TreeParameters
from Stem import Stem
from Knot import Knot
from functions import *

def union(d1,d2):
    #Joining 2 distnace fields
    d = np.minimum(d1,d2)
    d[d<0] = 0
    return d


def special_smooth_union(d1,d2,k):
    #Smoothly joining 2 distnace fields
    absd = k - np.absolute(d1-d2)
    absd[absd<0] = 0.0
    subd = 0.25*np.power(absd,2)/k
    d = np.minimum(d1-0.2*subd,d2-0.02*subd)
    d[d<0] = 0
    return d

def smooth_union(d1,d2,k):
    #Smoothly joining 2 distnace fields
    absd = k - np.absolute(d1-d2)
    absd[absd<0] = 0.0
    subd = 0.25*np.power(absd,2)/k
    d = np.minimum(d1,d2)-subd
    d[d<0] = 0
    return d

def smooth_union_incre(d1,d2,kmin,kmax,yrs):
    # max field
    absd = kmax - np.absolute(d1-d2)
    absd[absd<0] = 0.0
    subd = 0.25*np.power(absd,2)/kmax
    d = np.minimum(d1,d2)-subd
    d[d<0] = 0

    # varying k
    kd = (kmax-kmin)/yrs
    k = kmin + kd*d
    absd = k - np.absolute(d1-d2)
    absd[absd<0] = 0.0
    subd = 0.25*np.power(absd,2)/k
    d = np.minimum(d1,d2)-subd
    d[d<0] = 0

    return d

def dist_array_to_point_cloud(dist_array,box,para,color_step=10):
    #inds = np.argwhere(dist_array%1<0.25)
    #inds = np.argwhere(dist_array>=0)
    inds = np.argwhere(dist_array<=para.yrs)
    #inds = np.argwhere( (dist_array%1<0.25) & (dist_array<=para.yrs) )
    points = []
    colors = [] # Colors (rainbow color gradient according to year)
    pos = np.array(box.pos)
    for ind in inds:
        points.append(ind/box.ppc+pos)
        t = (dist_array[tuple(ind)]/color_step)%color_step
        (r, g, b) = colorsys.hsv_to_rgb(t, 1.0, 1.0)
        colors.append([r,g,b])
    points = np.array(points)
    points = points.astype('float64')
    colors = np.array(colors)
    return points,colors

def create_box_outline_and_mesh(box):
    #points/vertices
    inds = np.array(list(itertools.product([0,1],repeat=3)))
    points = []
    a,b,c = np.array(box.abs_pts.shape[:3])-1
    for i,j,k in inds: points.append(box.abs_pts[a*i][b*j][c*k])
    #lines for outlines
    lines = [[0, 1], [0, 2], [1, 3], [2, 3], [4, 5], [4, 6], [5, 7], [6, 7], [0, 4], [1, 5], [2, 6], [3, 7]]
    colors = [[0, 0, 0] for i in range(len(lines))]
    line_set = open3d.geometry.LineSet()
    line_set.points = open3d.utility.Vector3dVector(points)
    line_set.lines = open3d.utility.Vector2iVector(lines)
    line_set.colors = open3d.utility.Vector3dVector(colors)
    #faces for mesh
    tris = [[0,3,1],[0,2,3], [4,6,5],[6,5,7], [0,4,1],[1,5,4], [1,3,7],[1,7,5], [2,3,7],[2,7,6], [0,2,4],[2,4,6]]
    mesh = open3d.geometry.TriangleMesh()
    mesh.vertices = open3d.utility.Vector3dVector(points)
    mesh.triangles = open3d.utility.Vector3iVector(tris)
    mesh.paint_uniform_color(np.array([1.0,1.0,1.0])) #white
    #return
    return line_set, mesh

def create_point_cloud(points, colors=[]):
    point_cloud = open3d.geometry.PointCloud()
    point_cloud.points = open3d.utility.Vector3dVector(points)
    if len(colors)>0: point_cloud.colors = open3d.utility.Vector3dVector(colors)
    return point_cloud

def create_meshes(dist_array, box, para, n_start=None, n_end=None):
    meshes = []
    if n_start==None: n_start = int(np.min(dist_array))+1
    if n_end==None:   n_end =   int(np.max(dist_array))
    if n_end>para.yrs: n_end=para.yrs
    #n_start = 2
    #n_end = 7
    for n in range(n_start,n_end):
        loc_dist_array = np.copy(dist_array)-n
        vers, tris = mcubes.marching_cubes(loc_dist_array, 0)
        vers = vers/box.ppc+box.pos #scale,move
        vers = rotateZ(vers,box.org,box.rot) #rotate
        #vers = rotateZ(vers,box.loc_org,box.loc_rot) #rotate local
        #Define mesh
        mesh = open3d.geometry.TriangleMesh()
        mesh.vertices = open3d.utility.Vector3dVector(vers)
        mesh.triangles = open3d.utility.Vector3iVector(tris)
        (r, g, b) = colorsys.hsv_to_rgb((n/10)%10, 1.0, 1.0)
        mesh.paint_uniform_color(np.array([r,g,b]))
        mesh.compute_vertex_normals()
        meshes.append(mesh)
    print(n, "meshes")
    return meshes

def create_lines(point_lists):
    points = []
    lines = []
    cnt=0
    for pts in point_lists:
        points.extend(pts)
        for i in range(len(pts)-1): lines.append([cnt+i,cnt+i+1])
        cnt+=len(pts)
    line_segments = open3d.geometry.LineSet()
    line_segments.points = open3d.utility.Vector3dVector(points)
    line_segments.lines = open3d.utility.Vector2iVector(lines)
    return line_segments

def show(geometries):
    vis = open3d.visualization.Visualizer()
    vis.create_window()
    for geo in geometries: vis.add_geometry(geo)
    vis.get_render_option().load_from_json("renderoptions.json")
    ctr = vis.get_view_control()
    parameters = open3d.io.read_pinhole_camera_parameters("screencamera.json")
    ctr.convert_from_pinhole_camera_parameters(parameters)
    vis.run()
    vis.destroy_window()

start_time = time.time()

exterior_only=False

# Initiate cutting cube
box = CuttingBox([5,1,5], 8, [3,0,165], rot=-15.5, exterior_only=exterior_only, org_ctr=True)

# Load parameter files
para = TreeParameters(box, "000111", 30)

# Create the tree stem
stem = Stem(box, para)

# Create the knots
knots = []
for i in range(para.knots_no): knots.append(Knot(box, para, stem, i))

# Smoothly join the stem and the branch
dist_array = stem.dist_array
for knot in knots:
    #dist_array = union(dist_array, knot.dist_array)
    #dist_array = smooth_union(dist_array, knot.dist_array, 10)
    dist_array = special_smooth_union(dist_array, knot.dist_array, 50)
    #dist_array = smooth_union_incre(dist_array, knot.dist_array, 0.01, 20, para.yrs)
print("Stem and knots joined.")

# Create point cloud
if exterior_only:
    points,colors = dist_array_to_point_cloud(dist_array,box,para, color_step=3)
    point_cloud = create_point_cloud(points, colors=colors)

param_points = para.ppts_all+para.rpts[0]+para.rpts[-1]
#for pts in para.rpts: param_points.extend(pts)
for pts in para.kpts_all: param_points.extend(pts)
point_cloud_param = create_point_cloud(param_points)

box_outline, box_mesh = create_box_outline_and_mesh(box)

if not exterior_only: meshes = create_meshes(dist_array, box, para)

knot_lines = create_lines(para.kpts_all)

print('---')
print('Sample points:', np.prod(box.res))
print('Calculation time:', time.time()-start_time)
print('Calculation time per sample point:', (time.time()-start_time)/np.prod(box.res))

geos = []
geos.append(box_outline)
if exterior_only:
    geos.append(box_mesh)
    geos.append(point_cloud)
else: geos.extend(meshes)
geos.append(point_cloud_param)
geos.append(knot_lines)
show(geos)
