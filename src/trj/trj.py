####################################
# File name: trj.py                #
# Author: Mahmoud Pourmehrab       #
# Email: mpourmehrab@ufl.edu       #
# Last Modified: Feb/16/2018       #
####################################


import numpy as np


class Trajectory:
    '''
    Abstract class for trajectories
    '''

    LAG = 1  # lag on signalization
    RES = 1  # second (be careful not to exceed max size of trajectory
    EPS = 0.01
    DIG = 4  # number of digits to cut
    SAT = 1  # saturation headway

    def __init__(self, lead_veh, fol_veh, gs, gt, vmax, vcont):

        self.lead_veh = lead_veh
        self.fol_veh = fol_veh

        self.vmax = vmax
        self.vcont = vcont

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

        self.set_trj_points(t, d, s)

    def set_trj_points(self, t, d, s):

        self.fol_veh.set_trj(t, d, s)

    def insight(self):
        print(''' Trj Planner has received the following request @ {:04.2f} sec:
                dist: {:04.2f} m             initial speed: {:04.2f} m/s
                deceleration: {:04.2f} m/s2   acceleration: {:04.2f} m/s2
                spd limit: {:04.2f} m/s       spd limit @ control: {:04.2f} m/s
                green interval:            [{:04.2f}, {:04.2f}] sec
                '''.format(self.fol_veh.trajectory[0, 0], self.fol_veh.trajectory[0, 1], self.fol_veh.curr_speed,
                           self.fol_veh.max_decel_rate,
                           self.fol_veh.max_accel_rate, self.vmax, self.vcont, self.gs, self.gt))

        if self.fol_veh.curr_speed > self.vmax:
            t = (self.vmax - self.fol_veh.curr_speed) / self.fol_veh.max_decel_rate
            d = (self.vmax ** 2 - self.fol_veh.curr_speed ** 2) / (2 * self.fol_veh.max_decel_rate)
            print('Vehicle can decelerate to spd limit in {:04.2f} sec, {:04.2f} m'.format(t, d))
        else:
            t = (self.vmax - self.fol_veh.curr_speed) / self.fol_veh.max_accel_rate
            d = (self.vmax ** 2 - self.fol_veh.curr_speed ** 2) / (2 * self.fol_veh.max_accel_rate)
            print('Vehicle can accelerate to spd limit in {:04.2f} sec, {:04.2f} m'.format(t, d))
        if self.vcont < self.vmax:
            t = (self.vcont - self.vmax) / self.fol_veh.max_decel_rate
            d = (self.vcont ** 2 - self.vmax ** 2) / (2 * self.fol_veh.max_decel_rate)
            print('then decelerates to control spd limit in {:04.2f} sec, {:04.2f} m'.format(t, d))
        elif self.vcont > self.vmax:
            t = (self.vcont - self.vmax) / self.fol_veh.max_accel_rate
            d = (self.vcont ** 2 - self.vmax ** 2) / (2 * self.fol_veh.max_accel_rate)
            print('then accelerates to control spd limit in {:04.2f} sec, {:04.2f} m'.format(t, d))
