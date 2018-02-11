import sys
import numpy as np
import math
import random
from MapReader import MapReader

class MotionModel:

    """
    References: Thrun, Sebastian, Wolfram Burgard, and Dieter Fox. Probabilistic robotics. MIT press, 2005.
    [Chapter 5]
    """

    def __init__(self):

        """
        TODO : Initialize Motion Model parameters here
        """
        #self.angle_noise = 0.1
        #self.distance_noise = 5.0
        #self.sigma_trans = 0.0
        #self.sigma_theta = 0

        self.alpha1 = 0.01
        self.alpha2 = 0.01
        self.alpha3 = 0.1
        self.alpha4 = 0.1


    def update(self, u_t0, u_t1, x_t0):
        """
        param[in] u_t0 : particle state odometry reading [x, y, theta] at time (t-1) [odometry_frame]
        param[in] u_t1 : particle state odometry reading [x, y, theta] at time t [odometry_frame]
        param[in] x_t0 : particle state belief [x, y, theta] at time (t-1) [world_frame]
        param[out] x_t1 : particle state belief [x, y, theta] at time t [world_frame]
        """

        """
        TODO : Add your code here
        """

        x_bar_prime = u_t1[0] # x coordinates for odometry control info at time t
        y_bar_prime = u_t1[1] # y coordinates for odometry control info at time t
        theta_bar_prime = u_t1[2] # theta for odometry control info at time t
        x_bar = u_t0[0] # x coordinates for odometry control info at time t-1
        y_bar = u_t0[1] # y coordinates for odometry control info at time t-1
        theta_bar = u_t0[2] # theta for odometry control info at time t-1

        # the difference between odometry information at time t-1 and t
        delta_rot1 = np.arctan2(y_bar_prime - y_bar, x_bar_prime - x_bar) - theta_bar
        delta_trans = np.sqrt((y_bar_prime - y_bar)**2 + (x_bar_prime - x_bar)**2)
        delta_rot2 = theta_bar_prime - theta_bar - delta_rot1

        # the predicted difference between the robot coordinates at time t-1 and t
        delta_rot1_hat = delta_rot1 - np.random.normal(0.0, math.sqrt(self.alpha1*(delta_rot1**2) + self.alpha2*(delta_trans**2)))
        delta_trans_hat = delta_trans - np.random.normal(0.0, math.sqrt(self.alpha3*(delta_trans**2) + self.alpha4*((delta_rot1**2) + (delta_rot2**2))))
        delta_rot2_hat = delta_rot2 - np.random.normal(0.0, math.sqrt(self.alpha1*(delta_rot2**2) + self.alpha2*(delta_trans**2)))

        # the predicted particle coordinates
        x_t1 = np.zeros(3)
        x_t1[0] = x_t0[0] + delta_trans_hat * np.cos(x_t0[2] + delta_rot1_hat)
        x_t1[1] = x_t0[1] + delta_trans_hat * np.sin(x_t0[2] + delta_rot1_hat)
        x_t1[2] = x_t0[2] + delta_rot1_hat + delta_rot2_hat
        return x_t1

        """
        # change from odometry measurement
        delta_ux = u_t1[0] - u_t0[0]
        delta_uy = u_t1[1] - u_t0[1]
        delta_utheta = u_t1[2] - u_t0[2]

        cos_theta = np.cos(u_t1[2])
        sin_theta = np.sin(u_t1[2])
        if cos_theta > 0.5:
            trans = delta_ux / cos_theta
        else:
            trans = delta_uy / sin_theta

        # calculate the new theta for robot
        theta_error = delta_utheta * self.sigma_theta * np.random.normal(0.0, 0.0)
        theta = x_t0[2] + delta_utheta + theta_error
        theta = (theta - np.pi) % (2 * np.pi)

        #add translational error
        trans_error = trans * self.sigma_trans * np.random.normal()
        trans += trans_error
        # x_t0
        x = x_t0[0] + trans * np.cos(theta)
        y = x_t0[1] + trans * np.sin(theta)

        x_t1 = np.array([x, y, theta])

        return x_t1
        """


if __name__=="__main__":
    pass
