####################################
# File name: trjest.py             #
# Author: Mahmoud Pourmehrab       #
# Email: mpourmehrab@ufl.edu       #
# Last Modified: Feb/16/2018       #
####################################

import numpy as np

from src.trj.trj import Trajectory


class Conventional(Trajectory):
    '''
    Goal: Estimating movement of a conventional vehicle
    '''

    def __init__(self, lead_veh, fol_veh, gs=0, gt=86400, vmax=15, vcont=10):
        super().__init__(lead_veh, fol_veh, gs, gt, vmax, vcont)

    def solve(self, fdeg):
        '''
        :param fdeg: 0 for lead vehicle, 1 for follower vehicle
        '''
        if fdeg < 1:
            self.linear_trj()
        else:
            # indep_var = self.lead_veh.trj_s[self.indx:]
            t, d, v = self.gipps_cf()

            # This part adds the end part that is out of Gipps CF domain
            t_prime = d[-1] / self.fol_veh.des_speed

            tend = self.create_trj_domain(t[-1], t[-1] + t_prime)
            dend = [d[-1] - (tend[i] - t[-1]) * self.fol_veh.des_speed for i in range(len(tend))]
            vend = [self.fol_veh.des_speed for i in range(len(tend))]

            t = t + tend
            d = d + dend
            v = v + vend

    def gipps_cf(self):
        '''
        Gipps car following model

        Refer to:
            Gipps, Peter G. "A behavioural car-following model for computer simulation."
            Transportation Research Part B: Methodological 15.2 (1981): 105-111.

        :return: speed profile of follower CNV
        '''
        vf = self.fol_veh.curr_speed
        vfd = self.fol_veh.desired_speed
        af = self.fol_veh.max_accel_rate
        bf = self.fol_veh.max_decel_rate

        bl = self.lead_veh.max_decel_rate
        # vl will be looped over in the for loop
        Ll = self.lead_veh.length

        # d = self.fol_veh.trajectory[0,1] - self.lead_veh.trj_d[indx] will be looped over in the for loop

        ilead = self.lead_veh.curr_t_indx  # this goes over laed lists
        tf = self.lead_veh.trj_t[ilead:]
        vf = [0 for i in range(len(tf))]
        df = [0 for i in range(len(tf))]

        vf[0] = self.fol_veh.curr_speed
        df[0] = self.fol_veh.trajectory[0, 1]

        ifol = 0  # this goes over follower lists
        iend = len(tf) - 1
        while ifol < iend:
            vl = self.lead_veh.trj_s[ilead]
            gap = df[ifol] - self.lead_veh.trj_d[ilead]
            dt = tf[ifol + 1] - tf[ifol]

            s1 = 1 / 40 + vf[ifol] / vfd
            s2 = (bf * (bl * (2 * (Ll - gap) + dt * (bf * dt + vf[ifol])) + vl ** 2)) / bl

            v1 = vf[ifol] + (5 / 2) * af * dt * (1 - vf[ifol] / vfd) * np.sqrt(s1) if s1 >= 0 else 1024
            v2 = bf * dt + np.sqrt(s2) if s2 >= 0 else 1024

            vf[ifol + 1] = min(v1, v2)
            a = (vf[ifol + 1] - vf[ifol]) / dt
            df[ifol + 1] = df[ifol] - (
                    a * (tf[ifol + 1] ** 2 - tf[ifol] ** 2) / 2 + (vf[ifol + 1] - a * tf[ifol]) * dt)

            ifol += 1
            ilead += 1
        return tf, df, vf

    def augmet2cross(self):
        pass
