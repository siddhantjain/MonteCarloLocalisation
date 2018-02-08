import sys
import numpy as np
import math



class MotionModel:

    """
    References: Thrun, Sebastian, Wolfram Burgard, and Dieter Fox. Probabilistic robotics. MIT press, 2005.
    [Chapter 5]
    """

    def __init__(self,alphas):
        self.initAlphas=alphas

    def noiseSample(self,mu,sig):
        return np.random.normal(mu,sig)

    def update(self, u_t0, u_t1, x_t0):
        """
        param[in] u_t0 : particle state odometry reading [x, y, theta] at time (t-1) [odometry_frame]   
        param[in] u_t1 : particle state odometry reading [x, y, theta] at time t [odometry_frame]
        param[in] x_t0 : particle state belief [x, y, theta] at time (t-1) [world_frame]
        param[out] x_t1 : particle state belief [x, y, theta] at time t [world_frame]
        """

        #Implementing the odometry motion model
        deltaRot1=math.atan2(u_t1[1]-u_t0[1],u_t0[1]-u_t0[0])
        deltaTrans=math.sqrt((u_t0[0]-u_t1[0])**2+(u_t0[1]-u_t1[1])**2)
        deltaRot2=u_t1[2]-u_t0[2]-deltaRot1

        #calculating sigma for gaussian noise
        sigRot1=abs(self.initAlphas[0]*deltaRot1**2 + self.initAlphas[1]*deltaTrans**2)
        sigTrans = abs(self.initAlphas[2]*deltaTrans**2+self.initAlphas[3]*deltaRot1**2+self.initAlphas[3]*deltaRot2**2)
        sigRot2 = abs(self.initAlphas[0]*deltaRot2**2+self.initAlphas[1]*deltaTrans**2)


        #calculating the hat deltas
        deltaHatRot1=deltaRot1-self.noiseSample(0,sigRot1)
        deltaHatTrans=deltaTrans-self.noiseSample(0,sigTrans)
        deltaHatRot2=deltaRot2-self.noiseSample(0,sigRot2)

        x_t1=x_t0

        #reinitializing
        x_t1[0]= x_t0[0]+deltaHatRot1*math.cos(deltaRot1-deltaHatRot1)
        x_t1[1] = x_t0[1] + deltaHatTrans * math.cos(deltaTrans - deltaHatTrans)
        x_t1[2] = x_t0[2] + deltaHatRot2 * math.cos(deltaRot2 - deltaHatRot2)

        return x_t1



if __name__=="__main__":
    pass