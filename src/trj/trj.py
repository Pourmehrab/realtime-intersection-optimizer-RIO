'''
Code is written in Python 3
Install the list of packages in the Pipfile using PyEnv


By:     Mahmoud Pourmehrab
E-mail: mpourmehrab@ufl.edu
Date:        Nov 2017
Last update: Dec/08/2017
'''

import numpy as np

class MahmoudTrj:
    '''
    Abstract class for trajectories
    '''
    LAG = 1  # lag on signalization
    RES = 0.2  # second
    EPS = 0.01
    DIG = 2 # number of digits to cut

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
            indep_var = np.append(np.arange(self.fol_veh.det_time, te, MahmoudTrj.RES, dtype=float), te).round(
                self.DIG)
        else:
            indep_var = np.arange(self.fol_veh.det_time, te, MahmoudTrj.RES, dtype=float).round(self.DIG)

        return indep_var

    def reset(self):
        self.stat = False
