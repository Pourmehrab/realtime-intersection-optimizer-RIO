####################################
# File name: trj.py                #
# Author: Mahmoud Pourmehrab       #
# Email: mpourmehrab@ufl.edu       #
# Last Modified: Mar/29/2018       #
####################################


import numpy as np


# -------------------------------------------------------
# TRAJECTORY SUPER CLASS
# -------------------------------------------------------

class Trajectory:
    '''
    Abstract class for trajectories
    Two subclasses (Connected and Conventional) is inherited from this class
    '''

    LAG = 1  # lag on signalization
    RES = 1  # second (be careful not to exceed max size of trajectory
    EPS = 0.01  # small number that lower than that is approximated by zero
    DIG = 4  # number of digits to cut
    SAT = 1  # saturation headway (also in inter.py)

    def __init__(self, lead_veh, fol_veh, gs, gt, vmax, vcont):
        '''

        :param lead_veh: set None if aimed to optimize for lead vehicle
        :param fol_veh: if lead_veh is set to None, this is the first vehicle in a lane
        :param gs: green starts at this absolute second from when simulation starts
        :param gt:  green ends at this time
        :param vmax: maximum speed
        :param vcont: control speed at the final point of trajectory
        '''
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
        :return: trj linear spaced uniform time points
        '''
        if te - ts % self.RES > self.EPS:
            indep_var = np.append(np.arange(ts, te, Trajectory.RES, dtype=float), te).round(
                self.DIG)
        else:
            indep_var = np.arange(ts, te, Trajectory.RES, dtype=float).round(self.DIG)

        return indep_var

    def constant_speed_trj(self):
        '''
        Constructs the trajectory of the lead conventional vehicle assuming they maintain their speed
        '''
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


# -------------------------------------------------------
# CONVENTIONAL TRAJECTORY ESTIMATOR
# -------------------------------------------------------

class Conventional(Trajectory):
    '''
    Goal: Estimating movement of a conventional vehicle
    '''

    def __init__(self, lead_veh, fol_veh, gs=0, gt=86400, vmax=15, vcont=10):
        super().__init__(lead_veh, fol_veh, gs, gt, vmax, vcont)

    def estimate_earliest_arrival(self):
        '''
        For conventional vehicles the earliest arrival time is estimated by assuming constant speed profile
        '''
        self.constant_speed_trj()  # also sets the trajectory (does not return anything)

    def solve(self):
        '''
        :param fdeg: 0 for lead vehicle, 1 for follower vehicle
        '''
        if self.lead_veh is None:
            self.constant_speed_trj()
        else:
            t, d, v = self.follower_Gipps()

            # This part adds the end part that is out of Gipps CF domain
            t_prime = d[-1] / self.fol_veh.des_speed

            tend = self.create_trj_domain(t[-1], t[-1] + t_prime)
            dend = [d[-1] - (tend[i] - t[-1]) * self.fol_veh.des_speed for i in range(len(tend))]
            vend = [self.fol_veh.des_speed for i in range(len(tend))]

            t = t + tend
            d = d + dend
            v = v + vend

    def follower_Gipps(self):
        '''
        Gipps car following model

        Refer to:
            Gipps, Peter G. "A behavioural car-following model for computer simulation."
            Transportation Research Part B: Methodological 15.2 (1981): 105-111.

        :return: speed profile of follower CNV
        '''
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
        # todo write this
        pass


# -------------------------------------------------------
# CONNECTED AND AUTOMATED TRAJECTORY OPTIMIZER
# -------------------------------------------------------

class Connected(Trajectory):

    def __init__(self, lead_veh, fol_veh, gs=0, gt=86400, vmax=15, vcont=10):
        '''
        :param lead_veh:        lead vehicle
        :param fol_veh:         follower vehicle
        :param gs:              start of green in s (from the reference time)
        :param gt:              end of green in s (from the reference time)
        :param vmax:            speed limit in m/s
        :param vcont:           speed limit at the control point in m/s
        '''

        super().__init__(lead_veh, fol_veh, gs, gt, vmax, vcont)

        # solution is of the form v2, v3, a1, a3
        self.x = np.array([vmax, vcont, fol_veh.max_accel_rate, fol_veh.max_decel_rate], dtype=float)
        # assume maximum acceleration/deceleration is gonna be possible
        self.x[2] = fol_veh.max_accel_rate if vmax > fol_veh.curr_speed else fol_veh.max_decel_rate
        self.x[3] = fol_veh.max_accel_rate if vcont > vmax else fol_veh.max_decel_rate

    def estimate_earliest_arrival(self):

        t, d, s = self.get_earliest_trj()

        if self.lead_veh is not None and t < self.lead_veh.earliest_arrival + self.SAT:
            # vehicle is a follower
            # and is objected to saturation headway constraint

            t = self.lead_veh.earliest_arrival + self.SAT

        t0, d0, s0 = self.fol_veh.trajectory[0, 0], self.fol_veh.trajectory[0, 1], self.fol_veh.trajectory[0, 2]
        self.set_trj_points([t0, t], [d0, d], [s0, s])


