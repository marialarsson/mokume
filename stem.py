import math
import numpy as np

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
        x = x/self.res-self.cx
        y = y/self.res-self.cy
        z = z/self.res
        distance = math.sqrt(x**2+y**2)/self.d
        return distance
