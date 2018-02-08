import numpy as np
import pdb
import random

class Resampling:

    """
    References: Thrun, Sebastian, Wolfram Burgard, and Dieter Fox. Probabilistic robotics. MIT press, 2005.
    [Chapter 4.3]
    """

    def __init__(self):
        """
        TODO : Initialize resampling process parameters here
        """

    def multinomial_sampler(self, X_bar):

        """
        param[in] X_bar : [num_particles x 4] sized array containing [x, y, theta, wt] values for all particles
        param[out] X_bar_resampled : [num_particles x 4] sized array containing [x, y, theta, wt] values for resampled set of particles
        """

        """
        TODO : Add your code here
        """


        return X_bar_resampled

    def low_variance_sampler(self, X_bar):

        """
        param[in] X_bar : [num_particles x 4] sized array containing [x, y, theta, wt] values for all particles
        param[out] X_bar_resampled : [num_particles x 4] sized array containing [x, y, theta, wt] values for resampled set of particles
        """

        """
        TODO : Add your code here
        """

        w = X_bar[:, 3]
        p = X_bar[:, 0:3]
        p3 = []
        w_max = max(w)
        #n = w.shape[0]
        n = w.shape[0]
        idx = random.randint(0, n-1)
        beta = 0.0
        for i in range(w.shape[0]):
            beta = beta + random.random() * w_max * 2
            while (w[idx] < beta):
                beta = beta - w[idx]
                if (idx + 1 == n):
                    idx = 0
                else:
                    idx = idx + 1
                #idx = (idx + 1)% n
            p3.append(p[idx])
        w_new = np.ones([w.shape[0], 1])
        w_new = w_new/w.shape[0]
        X_bar_resampled = np.hstack((p3, w_new))
        return X_bar_resampled

if __name__ == "__main__":
    pass
