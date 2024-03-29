import numpy as np
import math
import time
from matplotlib import pyplot as plt
from scipy.stats import norm
import pdb
from math import *

import bresenham

from MapReader import MapReader

vis_flag = 0
def visualize_map(occupancy_map):
    fig = plt.figure()
    # plt.switch_backend('TkAgg')
    mng = plt.get_current_fig_manager();  # mng.resize(*mng.window.maxsize())
    plt.ion(); plt.imshow(occupancy_map, cmap='Greys'); plt.axis([0, 800, 0, 800]);


def visualize_raycast(ray):
    x_locs = [x[0] for x in ray]
    y_locs = [x[1] for x in ray]
    scat = plt.scatter(x_locs, y_locs, c='r', marker='o')
    plt.pause(0.1)
    scat.remove()

class SensorModel:

    """
    References: Thrun, Sebastian, Wolfram Burgard, and Dieter Fox. Probabilistic robotics. MIT press, 2005.
    [Chapter 6.3]
    """

    DEG_2_RAD = 0.0174533;
    def __init__(self, occupancy_map):
        self._occupancy_map = occupancy_map;



        self.zHit = 0.8;
        self.zShort = 0.133;
        self.zMax = 0.000325;
        self.zRand = 0.133;
        self.DEG_2_RAD = 0.0174533
        self.sigmaHit = 700;
        self.etaPHit = 1;
        self.lambdaShort = 0.001;
        self.Z_MAX = 8191
        self.Z_MAX_INVERSE = 1.0/self.Z_MAX
        self.sense_noise = 500
        """
        TODO : Initialize Sensor Model parameters here
        """
    def raycast(self,pos,angle):
        theta = pos[2] + angle*self.DEG_2_RAD
        x = int(pos[0]/10)
        y = int(pos[1]/10)

        #accounting for offset
        x = x + 25*cos(pos[2])
        y = y + 25*sin(pos[2])
        x0 = x
        y0 = y

        max_dist = max(self._occupancy_map.shape)
        stride =5

        dist = self.Z_MAX;
        while(self._occupancy_map[y,x] != -1):
            if(self._occupancy_map[y,x] > 0.9):
                dist = min(np.sqrt((x-x0) **2 + (y-y0)**2),max_dist)
                break

            x = x + int(stride* np.cos(theta))
            y = y + int(stride* np.sin(theta))

        vis_flag = 0
        if vis_flag:
            finalRayPoints = list(bresenham.bresenham(int(x0), int(y0), int(x), int(y)))
            visualize_raycast(finalRayPoints)

        return dist

    def raycast_alt(self, pos,angle):
        theta = pos[2]+ (angle-90)*self.DEG_2_RAD
        xStart = pos[0]/10
        yStart = pos[1]/10

        #change the constant 500 here to a smaller value, ideally just zmax.

        xEnd = xStart + 500*math.cos(theta);
        yEnd = yStart + 500*math.sin(theta);

        hit_x =0;
        hit_y =0;
        linePoints = list(bresenham.bresenham(int(xStart),int(yStart),int(xEnd),int(yEnd)))

        for each in linePoints:
          #TODO: change the occupancy condition to generate a random number instead of naively checking for non zero values
            if self._occupancy_map[each[1]][each[0]] <> 0:
                hit_x = each[0]
                hit_y = each[1]
                break;

        if vis_flag:
            finalRayPoints = list(bresenham.bresenham(int(xStart), int(yStart), int(hit_x), int(hit_y)))
            visualize_raycast(finalRayPoints)

        deltax = abs(pos[0] - hit_x*10)
        deltay = abs(pos[1] - hit_y*10)
        range = (deltax**2 + deltay**2)**0.5

        return range

    def calcPHit(self, z_k_star,z_k_t):
        if z_k_t >= self.Z_MAX:
            return 0
        constant = 2*math.pi*self.sigmaHit*self.sigmaHit
        constant = constant**0.5
        constant = 1.0/constant

        exponentTerm = -0.5*(z_k_star - z_k_t)*(z_k_star - z_k_t)/(self.sigmaHit*self.sigmaHit)

        pHit = constant*(math.e**exponentTerm)

        return pHit

    def Gaussian(self, mu, sigma, x):

        # calculates the probability of x for 1-dim Gaussian with mean mu and var. sigma
        return exp(- ((mu - x) ** 2) / (sigma ** 2) / 2.0) / sqrt(2.0 * pi * (sigma ** 2))

    def calcPShort(self,z_k_star,z_k_t):
        denominator = 1.0- math.e**(-self.lambdaShort*z_k_star)
        etaPShort =  1.0/(denominator)
        pShort =1
        if z_k_t < 0 or z_k_t > z_k_star:
            return 0
        pShort = etaPShort * self.lambdaShort*(math.e ** (-self.lambdaShort*z_k_t))

        return pShort

    def calcPMax(self, z_k_t):

        pMax = int(z_k_t == self.Z_MAX)
        return pMax

    def calcPRand(self,z_k_t):
        if z_k_t < 0 or z_k_t >= self.Z_MAX:
            return 0
        pRand = self.Z_MAX_INVERSE
        return pRand

    def beam_range_finder_model(self, z_t1_arr, x_t1):
        """
        param[in] z_t1_arr : laser range readings [array of 180 values] at time t
        param[in] x_t1 : particle state belief [x, y, theta] at time t [world_frame]
        param[out] prob_zt1 : likelihood of a range scan zt1 at time t
        """
        q =1
        i=0
        if vis_flag:
            visualize_map(self._occupancy_map);

        if (self._occupancy_map[x_t1[1] / 10, x_t1[0] / 10] == -1):
            q = 0
            return 0

        #TODO: siddhant: write cleaner code. remove constant 180
        for i in range(0,180,10):
            z_k_t = z_t1_arr[i]
            z_k_star =  self.raycast(x_t1,i)

            p_hit = self.calcPHit(z_k_star,z_k_t)
            p_short = self.calcPShort(z_k_star,z_k_t)
            p_max = self.calcPMax(z_k_t)
            p_rand = self.calcPRand(z_k_t)

            p = self.zHit*p_hit + self.zShort*p_short + self.zMax*p_max + self.zRand*p_rand
            q += log(p,10)

        return q    
 
if __name__=='__main__':
    pass