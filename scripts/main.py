import numpy as np
import sys
import pdb
import math

from MapReader import MapReader
from MotionModel import MotionModel
from SensorModel import SensorModel
from Resampling import Resampling

from matplotlib import pyplot as plt
from matplotlib import figure as fig
import time

import random

def visualiseSensorModel(sensor_model):
    return
    zHit = sensor_model.zHit;
    zShort = sensor_model.zShort;
    zMax = sensor_model.zMax;
    zRand = sensor_model.zRand;
    z_k_t = np.arange(0, 8192)
    # p = sensor_model.calcPHit(1000, z_k_t)
    p = []
    for i in range(len(z_k_t)):
        # p.append(sensor_model.calcPMax(z_k_t[i]));
        # p.append(sensor_model.calcPRand(z_k_t[i]));
        # zHit zShort zMax zRand
        p.append(zHit * sensor_model.calcPHit(4000, z_k_t[i]) +
                 zShort * sensor_model.calcPShort(4000, z_k_t[i]) +
                 zMax * sensor_model.calcPMax(z_k_t[i]) +
                 zRand * sensor_model.calcPRand(z_k_t[i]))
        # p.append(sensor_model.calcPShort(4000, z_k_t[i]));
    plt.plot(z_k_t, p)
    plt.show()

def visualize_map(occupancy_map):
    fig = plt.figure()
    # plt.switch_backend('TkAgg')
    mng = plt.get_current_fig_manager();  # mng.resize(*mng.window.maxsize())
    plt.ion(); plt.imshow(occupancy_map, cmap='Greys'); plt.axis([0, 800, 0, 800]);


def visualize_timestep(X_bar, tstep):
    x_locs = X_bar[:,0]/10.0
    y_locs = X_bar[:,1]/10.0
    scat = plt.scatter(x_locs, y_locs, c='r', marker='o')
    plt.pause(0.00002)
    scat.remove()




def init_particles_random(num_particles, occupancy_map):

    # initialize [x, y, theta] positions in world_frame for all particles
    # (randomly across the map) 
    y0_vals = np.random.uniform( 0, 7000, (num_particles, 1) )
    x0_vals = np.random.uniform( 3000, 7000, (num_particles, 1) )
    theta0_vals = np.random.uniform( -3.14, 3.14, (num_particles, 1) )

    # initialize weights for all particles
    w0_vals = np.ones( (num_particles,1), dtype=np.float64)
    w0_vals = w0_vals / num_particles

    X_bar_init = np.hstack((x0_vals,y0_vals,theta0_vals,w0_vals))
    
    return X_bar_init

def init_particles_freespace(num_particles, occupancy_map):

    # initialize [x, y, theta] positions in world_frame for all particles
    # (in free space areas of the map)

    #Siddhant Find out free space areas of the map
    """

    TODO : (siddhant) Vectorise this code here
    is greater than 0.5
    """
    unoccupiedSpacesList = []
    particleWeightInit = 1.0/num_particles
    [numCols, numRows] = occupancy_map.shape
    for eachX in range(numCols):
        for eachY in range(numRows):
            if(occupancy_map[eachY,eachX]==0):
                theta_init = np.random.uniform(-math.pi,math.pi)
                x_init = np.random.uniform(eachX*10,eachX*10 + 10.0)
                y_init = np.random.uniform(eachY*10, eachY*10 + 10.0)

                unoccupiedSpacesList.append([x_init,y_init,theta_init,particleWeightInit])


    initialParticlesList = random.sample(unoccupiedSpacesList,num_particles)
    X_bar_init = np.array(initialParticlesList)
    return X_bar_init

def main():

    """
    Description of variables used
    u_t0 : particle state odometry reading [x, y, theta] at time (t-1) [odometry_frame]   
    u_t1 : particle state odometry reading [x, y, theta] at time t [odometry_frame]
    x_t0 : particle state belief [x, y, theta] at time (t-1) [world_frame]
    x_t1 : particle state belief [x, y, theta] at time t [world_frame]
    X_bar : [num_particles x 4] sized array containing [x, y, theta, wt] values for all particles
    z_t : array of 180 range measurements for each laser scan
    """

    """
    Initialize Parameters
    """
    src_path_map = '../data/map/wean.dat'
    src_path_log = '../data/log/robotdata1.log'

    map_obj = MapReader(src_path_map)
    occupancy_map = map_obj.get_map() 
    logfile = open(src_path_log, 'r')

    motion_model = MotionModel()
    sensor_model = SensorModel(occupancy_map)
    resampler = Resampling()

    vis_flag_sensor_model = 1

    if vis_flag_sensor_model:
        visualiseSensorModel(sensor_model)

    num_particles = 500
    #X_bar = init_particles_random(num_particles, occupancy_map)
    X_bar = init_particles_freespace(num_particles, occupancy_map)

    vis_flag = 1

    """
    Monte Carlo Localization Algorithm : Main Loop
    """
    if vis_flag:
        visualize_map(occupancy_map)

    first_time_idx = True
    for time_idx, line in enumerate(logfile):

        # Read a single 'line' from the log file (can be either odometry or laser measurement)
        meas_type = line[0] # L : laser scan measurement, O : odometry measurement
        meas_vals = np.fromstring(line[2:], dtype=np.float64, sep=' ') # convert measurement values from string to double

        odometry_robot = meas_vals[0:3] # odometry reading [x, y, theta] in odometry frame
        time_stamp = meas_vals[-1]

        # if ((time_stamp <= 0.0) | (meas_type == "O")): # ignore pure odometry measurements for now (faster debugging) 
            # continue

        if (meas_type == "L"):
             odometry_laser = meas_vals[3:6] # [x, y, theta] coordinates of laser in odometry frame
             ranges = meas_vals[6:-1] # 180 range measurement values from single laser scan
        else:
            continue #skipping for saving time
        print "Processing time step " + str(time_idx) + " at time " + str(time_stamp) + "s"

        if (first_time_idx):
            u_t0 = odometry_robot
            first_time_idx = False
            continue

        X_bar_new = np.zeros( (num_particles,4), dtype=np.float64)
        u_t1 = odometry_robot
        for m in range(0, num_particles):

            """
            MOTION MODEL
            """
            x_t0 = X_bar[m, 0:3]
            x_t1 = motion_model.update(u_t0, u_t1, x_t0)

            """
            SENSOR MODEL
            """
            if (meas_type == "L"):
                z_t = ranges
                w_t = sensor_model.beam_range_finder_model(z_t, x_t1)
                # w_t = 1/num_particles
                X_bar_new[m,:] = np.hstack((x_t1, w_t))
            else:
                X_bar_new[m,:] = np.hstack((x_t1, X_bar[m,3]))

        if (meas_type == "L"):
            X_bar_new[:, 3] = X_bar_new[:, 3]/sum(X_bar_new[:, 3])

        X_bar = X_bar_new
        u_t0 = u_t1


        """
        RESAMPLING
        """
        X_bar = resampler.low_variance_sampler(X_bar)

        if vis_flag:
            visualize_timestep(X_bar, time_idx)

if __name__=="__main__":
    main()
