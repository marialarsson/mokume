import math
import numpy as np
import random
import itertools

class LineSegment:
    def __init__(self, p0, p1):
        dx = p1[0]-p0[0]
        dy = p1[2]-p0[2]
        if dx!=0: self.k = dy/dx
        else: self.k = 0
        self.xdomain = [p0[0],p1[0]]
        self.a = p0[2]-self.k*p0[0]

def point_to_line_distance(p, a, b):
    d = np.divide(b - a, np.linalg.norm(b - a)) # normalize tangent vector
    s = np.dot(a - p, d) # signed parallel distance components
    t = np.dot(p - b, d)
    h = np.maximum.reduce([s, t, 0]) # clamped parallel distance
    c = np.cross(p - a, d)     # perpendicular distance component
    dist = np.hypot(h, np.linalg.norm(c))
    return dist

def point_to_sqrt_graph_distance(p,h,d):
    ## Get x0 (a) of the perpendicular line equation-formula derived from some algebra
    a=p[2]+(2/h)*math.sqrt(p[0])*p[0]
    b=h
    myx = sympy.symbols('myx')
    expr = b*sympy.sqrt(myx)+(2/b)*sympy.sqrt(myx)*myx-a
    sol = sympy.solveset(expr, domain=sympy.S.Reals)
    try:
        cpx = list(sol)[0]
        cpy = 0
        cpz = h*math.sqrt(cpx)
        cp = np.array([cpx,cpy,cpz])
        squared_dist = np.sum((cp-p)**2, axis=0)
        distance = np.sqrt(float(squared_dist))
        distance/=d
    except:
        distance=9999
    return distance

def point_to_seg_distance(p,segs,d):
    distance = 999
    # 1. identify which line segment is relevant for this point based on xrange
    sel_seg = None
    x,y = p[0], p[2]
    for seg in segs:
        if x>seg.xdomain[0] and x<seg.xdomain[1]:
            sel_seg=seg
            break
    # 2. find the closest point on this line segment
    # 2a formula for the perpendicular line that crosses the point and the closest point y=a2-1/kx, solve for a2
    if sel_seg!=None:
        a2 = y+x/sel_seg.k
        # 2c solve for line-line intersection
        cpx = (a2-sel_seg.a)/(1/sel_seg.k+sel_seg.k)
        cpy = sel_seg.a+sel_seg.k*cpx
        cp = np.array([cpx,0,cpy])
        # measure the distance between point and closest point
        distance = np.linalg.norm(p-cp)/d
    return distance

class Branch:
    def __init__(self, stem, res, d=0.025, t=0.25, omega=math.pi/6, h=0.5, randomize=False): #t0.1
        self.stem = stem
        self.res = res
        if randomize:
            self.d = random.uniform(0.020,0.030)        # distance between growth surfaces
            self.t = random.uniform(-0.2,0.6)            # position along main skeleton
            self.omega = random.uniform(0.0,2*math.pi)    # rotation around the skeleton (0.0 is x-axis aligned and so on)
            self.h = random.uniform(0.25,0.75)          # parameter for steepness of curve describing the branch skeleton
        else:
            self.d = d
            self.t = t
            self.omega = omega
            self.h = h
        print("Angle of branch rotation", self.omega)
        self.ln_a = np.array([self.stem.cx,self.stem.cy,self.t])     # branch line skeleton start point
        self.ln_b = np.array([self.stem.cx+1,self.stem.cy,self.t+0.4])   # branch line skeleton end point
        self.get_graph_segments()
        print("Number of segements of approximation", len(self.segs))
        self.create_3D_distance_array()

    def create_3D_distance_array(self):
        ind_list = [i for i in range(self.res)]
        point_inds = list(itertools.product(ind_list,repeat=3))
        dist_array=[]
        for ind in point_inds: dist_array.append(self.distance_function(ind))
        dist_array = np.array(dist_array)
        self.dist_array = np.reshape(dist_array,(self.res,self.res,self.res))

    def distance_function(self,xyz):
        x,y,z = xyz
        x0=x/self.res-self.stem.cx
        y0=y/self.res-self.stem.cy
        ## for omega param
        x = x0*math.cos(self.omega) - y0*math.sin(self.omega)
        y = x0*math.sin(self.omega) + y0*math.cos(self.omega)
        ##
        z=z/self.res-self.t
        p = np.array([x,y,z])

        if x<=0: #behind branch
            distance=999.0
        else:
            # chose type of function
            #distance = point_to_line_distance(p,self.ln_a,self.ln_b)/self.d
            #distance = point_to_sqrt_graph_distance(p,self.h,self.d)
            distance = point_to_seg_distance(p,self.segs,self.d)
        return distance

    def get_graph_segments(self):
        pts = [[0.0001,0,0]]
        ss = 0.0125 #step size
        #all points
        for i in range(100):
            x0=pts[-1][0] #last x
            k = self.h*0.5*x0**(-0.5) #derivative
            dx = math.sqrt(ss**2/(1+k**2)) #difference in x
            x = x0+dx #new x
            y = self.h*math.sqrt(x) #new y
            ##
            pts.append([x,0,y])
        #reduce number of points
        dd = 0.001
        change = True
        while change:
            new_pts = []
            for i in range(len(pts)-2):
                if i%2==1: continue #even only
                p0, p1, p2 = pts[i], pts[i+1], pts[i+2]
                dx, dy = p2[0]-p0[0], p2[2]-p0[2]
                k = dy/dx
                a = p0[2]-k*p0[0] # line function is L(x)=a+k*x
                ## find line closest point on the line a+k*x for p1
                k2 = -1/k
                a2 = p1[2]-k2*p1[0]
                ix = (a2-a)/(k-k2) #intersection x
                iy = a+k*ix #intersection y
                pi = [ix,0,iy] # intersection point, i.e. line closest point
                ## calculate distnace
                diff_vec = np.array(p1)-np.array(pi)
                dist = np.linalg.norm(diff_vec)
                new_pts.append(p0)
                if dist>dd: new_pts.append(p1)
            new_pts.append(pts[-1])
            if len(new_pts)==len(pts): change = False
            pts = list(new_pts)
        #collect segments
        self.segs = []
        for i in range(len(pts)-1):
            self.segs.append(LineSegment(pts[i],pts[i+1]))
