'''
Code is written in Python 3
Install the list of packages in the Pipfile using PyEnv


By:     Mahmoud Pourmehrab
E-mail: mpourmehrab@ufl.edu
Date:        Nov 2017
Last update: Dec/15/2017
'''

import numpy as np


class Trajectory:
    '''
    Abstract class for trajectories
    '''

    LAG = 1  # lag on signalization
    RES = 1  # second (be careful not to exceed max size of trajectory
    EPS = 0.01
    DIG = 4  # number of digits to cut

    def __init__(self, lead_veh, fol_veh, gs, gt):

        self.lead_veh = lead_veh
        self.fol_veh = fol_veh
        gt_lagged = gs + self.LAG
        if gt > gt_lagged:
            self.gs, self.gt = gt_lagged, gt
        else:
            raise Exception('Signal lag exceeds the length of green')
        self.stat = False

    def create_trj_domain(self, ts, te):
        '''

        :param ts: start time
        :param te: end time
        :return: trj domain
        '''
        # todo: (Mahmoud) control the last point of trajectory should be zero
        if te - ts % self.RES > self.EPS:
            indep_var = np.append(np.arange(ts, te, Trajectory.RES, dtype=float), te).round(
                self.DIG)
        else:
            indep_var = np.arange(ts, te, Trajectory.RES, dtype=float).round(self.DIG)

        return indep_var

    def linear_trj(self):
        tend = self.fol_veh.trajectory[0, 1] / self.fol_veh.curr_speed

        t = self.create_trj_domain(self.fol_veh.trajectory[0, 0], self.fol_veh.trajectory[0, 0] + tend)
        s = np.array([self.fol_veh.curr_speed for i in range(len(t))])
        d = np.array([self.fol_veh.trajectory[0, 1] -
                      self.fol_veh.curr_speed *
                      (t[i] - self.fol_veh.trajectory[0, 0]) for i in range(len(t))])

        self.set_follower_trj(t, d, s)

    def set_follower_trj(self, t, d, s):

        self.fol_veh.set_trj(t, d, s)
