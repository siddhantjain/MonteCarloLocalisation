import numpy as np
import math
import time
from matplotlib import pyplot as plt
from scipy.stats import norm
import pdb

from MapReader import MapReader

class SensorModel:

    """
    References: Thrun, Sebastian, Wolfram Burgard, and Dieter Fox. Probabilistic robotics. MIT press, 2005.
    [Chapter 6.3]
    """

    def __init__(self, occupancy_map):
        self._occupancy_map = occupancy_map;
        self.zHit = 1;
        self.zShort = 1;
        self.zMax = 1;
        self.zRand = 1;
        """
        TODO : Initialize Sensor Model parameters here
        """
    def raycast(self, pos,angle):
        range = 0;

        return range
    def calcPHit(self):
        pHit = 1

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

        """
        TODO : Add your code here
        """
        q =1
        i=0
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