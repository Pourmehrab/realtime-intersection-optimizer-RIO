'''
Goal: To optimizes the trajectory of an Automated Vehicle (AV)

By:     Mahmoud Pourmehrab
E-mail: mpourmehrab@ufl.edu
Date:        Nov 2017
Last update: Dec/14/2017
'''

import numpy as np
from scipy.optimize import minimize

from src.trj.trj import MahmoudTrj


class MahmoudAVO(MahmoudTrj):
    def __init__(self, lead_veh, fol_veh, gs=0, gt=86400, fdeg=0, vmax=15, vcont=10):
        '''

        :param lead_veh:        lead vehicle
        :param fol_veh:         follower vehicle
        :param gs:              start of green in s (from the reference time)
        :param gt:              end of green in s (from the reference time)
        :param fdeg:            0: free, 1: dist-const, 2: time and dist-const (only for the first AV)
        :param vmax:            speed limit in m/s
        :param vcont:           speed limit at the control point in m/s
        '''

        super().__init__(lead_veh, fol_veh, gs, gt, vmax, vcont)

        self.insight()

        self.fdeg = fdeg
        self.x = np.array([vmax, vcont, fol_veh.amax, fol_veh.amin], dtype=float)
        self.t1, self.t2, self.t3, self.tt = 0, 0, 0, gt + 1

        self.bnds = (
            (0, self.vmax), (0, self.vcont), (self.fol_veh.amin, self.fol_veh.amax),
            (self.fol_veh.amin, self.fol_veh.amax))
        self.con1 = ({'type': 'ineq',
                      'fun': lambda x, d0, v0, gs:
                      np.array((x[3] * (v0 - x[0]) ** 2 + x[2] * (
                              2 * x[3] * d0 - (x[0] - x[1]) ** 2))
                               / (2 * x[2] * x[3] * x[0]) - gs),
                      'jac': self.ttime_der,
                      'args': (self.fol_veh.dist, self.fol_veh.speed, self.gs,)})

        self.con2 = ({'type': 'eq',
                      'fun': lambda x, d0, v0, t_opt:
                      np.array((x[3] * (v0 - x[0]) ** 2 + x[2] * (
                              2 * x[3] * d0 - (x[0] - x[1]) ** 2))
                               / (2 * x[2] * x[3] * x[0]) - t_opt),
                      'jac': self.ttime_der,
                      'args': (self.fol_veh.dist, self.fol_veh.speed, self.tt,)})

    def solve(self):

        min_travel_time_sol = minimize(self.traveltime,
                                       np.array([self.vmax, self.vcont, self.fol_veh.amax, self.fol_veh.amax]),
                                       args=(self.fol_veh.dist, self.fol_veh.speed, self.gs,), jac=self.ttime_der,
                                       bounds=self.bnds, constraints=self.con1, method='SLSQP', options={'disp': True})

        if self.weakfeasibility(min_travel_time_sol.x):

            if min_travel_time_sol.x[1] < self.vcont - self.EPS:  # see if we can improve discharge speed
                self.tt = min_travel_time_sol.fun

                depart_spd = lambda x, d0, v0, t_opt: np.array(-1 * x[1])  # since it's minimization
                depart_spd_der = lambda x, d0, v0, t_opt: np.array([0, -1, 0, 0])

                max_speed_sol = minimize(depart_spd,
                                         min_travel_time_sol.x,
                                         args=(self.fol_veh.dist, self.fol_veh.speed, min_travel_time_sol.fun,),
                                         jac=depart_spd_der, bounds=self.bnds, constraints=self.con2, method='SLSQP',
                                         options={'disp': True})
                if self.weakfeasibility(max_speed_sol.x):
                    self.setopt(max_speed_sol.x, max_speed_sol.fun)
                else:
                    self.setopt(min_travel_time_sol.x, min_travel_time_sol.fun)

            else:
                self.setopt(min_travel_time_sol.x, min_travel_time_sol.fun)
        else:
            raise Exception('Review arrival info since trj optimizer is infeasible')

    def setopt(self, x, tt):
        self.stat = True
        self.x[0] = x[0]
        self.x[1] = x[1]
        self.x[2] = x[2]
        self.x[3] = x[3]
        self.t1 = self.calct1(x[0], x[2])
        self.t3 = self.calct3(x[0], x[1], x[3])
        self.t2 = tt - self.t1 - self.t3
        self.tt = tt

    def weakfeasibility(self, x):
        if np.sign(x[0] - self.fol_veh.speed) != np.sign(x[2]) or np.sign(x[1] - x[0]) != np.sign(x[3]):
            # Solution is not feasible
            return False
        else:
            # Solution may be feasible
            return True

    def traveltime(self, x, d0, v0, gs):
        return (x[3] * (v0 - x[0]) ** 2 + x[2] * (2 * x[3] * d0 - (x[0] - x[1]) ** 2)) \
               / (2 * x[2] * x[3] * x[0])

    def ttime_der(self, x, d0, v0, gs):
        return np.array([(x[3] * (-v0 ** 2 + x[0] ** 2) + x[2] * (-2 * x[3] * d0 - x[0] ** 2 + x[1] ** 2)) / (
                2 * x[2] * x[3] * x[0] ** 2)
                            , (x[0] - x[1]) / (x[3] * x[0])
                            , -((v0 - x[0]) ** 2 / (2 * x[2] ** 2 * x[0]))
                            , (x[0] - x[1]) ** 2 / (2 * x[3] ** 2 * x[0])])

    def calct1(self, v2, a1):
        dv1 = v2 - self.fol_veh.speed
        if abs(dv1) > self.EPS and abs(a1) > self.EPS:
            return (v2 - self.fol_veh.speed) / a1

    def calct2(self, v2, v3, a1, a3):
        dv1 = v2 - self.fol_veh.speed
        dv3 = v3 - v2
        if abs(dv1) > self.EPS and abs(a1) > self.EPS and abs(dv3) > self.EPS and abs(a3) > self.EPS:
            return (self.fol_veh.dist - (v2 ** 2 - self.fol_veh.speed ** 2) / (2 * a1) - (v3 ** 2 - v2 ** 2) / (
                    2 * a3)) / v2
        elif abs(dv1) <= self.EPS or abs(a1) <= self.EPS:
            return (self.fol_veh.dist - (v3 ** 2 - v2 ** 2) / (2 * a3)) / v2
        elif abs(dv3) <= self.EPS or abs(a3) <= self.EPS:
            return (self.fol_veh.dist - (v2 ** 2 - self.fol_veh.speed ** 2) / (2 * a1)) / v2
        else:
            return self.fol_veh.dist / v2

    def calct3(self, v2, v3, a3):
        dv3 = v3 - v2
        if abs(dv3) > self.EPS and abs(a3) > self.EPS:
            return (v3 - v2) / a3

    def d1(self, dt, a1):
        # print('{}'.format(self.fol_veh.dist - self.fol_veh.speed * dt - a1 * dt ** 2 / 2))
        return self.fol_veh.dist - self.fol_veh.speed * dt - a1 * dt ** 2 / 2

    def d2(self, dt, t1, v2, a1):
        # print('{}'.format(self.d1(t1, a1) - v2 * dt))
        return self.d1(t1, a1) - v2 * dt

    def d3(self, dt, t1, t2, v2, a1, a3):
        # print('{}'.format(self.d2(t2, t1, v2, a1) - v2 * dt - a3 * dt ** 2 / 2))
        return self.d2(t2, t1, v2, a1) - v2 * dt - a3 * dt ** 2 / 2

    def d(self, t, t1, t2, v2, a1, a3):
        dt = t - self.fol_veh.det_time
        if dt <= t1:
            return self.d1(dt, a1)
        elif dt <= t1 + t2:
            return self.d2(dt - t1, t1, v2, a1)
        else:
            return self.d3(dt - t1 - t2, t1, t2, v2, a1, a3)

    def buildtrj(self, x):
        tt = self.traveltime(np.array([x[0], x[1], x[2], x[3]]), self.fol_veh.dist, self.fol_veh.speed, self.gs)
        t1 = self.calct1(x[0], x[2])
        t2 = self.calct2(x[0], x[1], x[2], x[3])

        tend = self.fol_veh.det_time + tt
        if tend % self.RES > self.EPS:
            self.indepVar = np.append(
                np.arange(self.fol_veh.det_time, self.fol_veh.det_time + tt, MahmoudAVO.RES, dtype=float), tend).round(
                self.DIG)
        else:
            self.indepVar = np.arange(self.fol_veh.det_time, self.fol_veh.det_time + tt, MahmoudAVO.RES,
                                      dtype=float).round(self.DIG)

        vd = np.vectorize(self.d)
        self.depVar = vd(self.indepVar, t1, t2, x[0], x[2], x[3])
        # todo: (Mahmoud) control the last point of trajectory should be zero
