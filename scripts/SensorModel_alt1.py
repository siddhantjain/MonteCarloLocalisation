import numpy as np
import math
import time
from matplotlib import pyplot as plt
from scipy.stats import norm
import pdb

import bresenham

from MapReader import MapReader

vis_flag = 1
def visualize_map(occupancy_map):
    fig = plt.figure()
    # plt.switch_backend('TkAgg')
    mng = plt.get_current_fig_manager();  # mng.resize(*mng.window.maxsize())
    plt.ion(); plt.imshow(occupancy_map, cmap='Greys'); plt.axis([0, 800, 0, 800]);


def visualize_raycast(ray):
    x_locs = [x[0] for x in ray]
    y_locs = [x[1] for x in ray]
    scat = plt.scatter(x_locs, y_locs, c='r', marker='o')
    plt.pause(0.00001)
    scat.remove()

class SensorModel:

    """
    References: Thrun, Sebastian, Wolfram Burgard, and Dieter Fox. Probabilistic robotics. MIT press, 2005.
    [Chapter 6.3]
    """

    DEG_2_RAD = 0.0174533;
    def __init__(self, occupancy_map):
        self._occupancy_map = occupancy_map;
        self.zHit = 1;
        self.zShort = 1;
        self.zMax = 1;
        self.zRand = 1;
        self.DEG_2_RAD = 0.0174533
        self.sigmaHit = 1;
        self.etaPHit = 1;
        self.etaPshort = 1;
        """
        TODO : Initialize Sensor Model parameters here
        """
    def raycast(self, pos,angle):
        theta = pos[2]+angle*self.DEG_2_RAD
        xStart = pos[0]/10
        yStart = pos[1]/10

        #change the constant 500 here to a smaller value, ideally just zmax.

        xEnd = xStart + 500*math.cos(theta);
        yEnd = yStart + 500*math.sin(theta);

        hit_x =0;
        hit_y =0;
        linePoints = list(bresenham.bresenham(int(xStart),int(yStart),int(xEnd),int(yEnd)))

        if vis_flag:
            visualize_raycast(linePoints)

        for each in linePoints:
          #TODO: change the occupancy condition to generate a random number instead of naively checking for non zero values
            if self._occupancy_map[each[1]][each[0]] <> 0:
                hit_x = each[0]
                hit_y = each[1]
                break;

        deltax = abs(pos[0] - hit_x*10)
        deltay = abs(pos[1] - hit_y*10)
        range = (deltax**2 + deltay**2)**0.5

        return range
    def calcPHit(self, z_k_star,z_k_t):
        constant = 2*math.pi*self.sigmaHit*self.sigmaHit
        constant = constant**0.5
        constant = 1.0/constant

        exponentTerm = -0.5*(z_k_star - z_k_t)*(z_k_star - z_k_t)/(self.sigmaHit*self.sigmaHit)

        pHit = constant*(math.e**exponentTerm)

        return pHit

    def calcPShort(self):
        pShort =1
        return pShort

    def calcPMax(self):
        pMax = 1
        return pMax

    def calcPRand(self):
        pRand = 1
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
        #TODO: siddhant: write cleaner code. remove constant 180
        for i in range(180):
            if i%5 ==0:
                z_k_star =  self.raycast(x_t1,i)
                p_hit = self.calcPHit()
                p_short = self.calcPShort()
                p_max = self.calcPMax()
                p_rand = self.calcPRand()

                p = self.zHit*p_hit + self.zShort*p_short + self.zMax*p_max + self.zRand*p_rand
                q = q*p
        return q    
 
if __name__=='__main__':
    pass