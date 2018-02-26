####################################
# File name: trjopt.py             #
# Author: Mahmoud Pourmehrab       #
# Email: mpourmehrab@ufl.edu       #
# Last Modified: Feb/16/2018       #
####################################

import numpy as np
from scipy.optimize import linprog

from src.trj.trj import Trajectory


class Connected(Trajectory):
    K = 10  # poly degree

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

    def solve_earlst(self, fdeg):

        t, d, s = self.get_earliest_trj()

        if fdeg > 0 and t < self.lead_veh.earlst + self.SAT:  # follower is objected to saturation headway constraint
            t = self.lead_veh.earlst + self.SAT

        t0, d0, s0 = self.fol_veh.trajectory[0, 0], self.fol_veh.trajectory[0, 1], self.fol_veh.trajectory[0, 2]
        self.set_trj_points([t0, t], [d0, d], [s0, s])

    def solve(self, fdeg):
        '''
        :param fdeg:  0: free (when lead_veh is None), 1: dist-const, 2: time and dist-const (only for the first AV)
        '''

        if fdeg < 1:  # Solve as a lead vehicle
            t, d, s = self.get_one_comp_trj(self.fol_veh.trajectory[0, 0], self.fol_veh.trajectory[0, 1],
                                            self.fol_veh.curr_speed, self.gs, 0, self.vcont)
        else:
            pass

        self.set_trj_points(t, d, s)

    def get_earliest_trj(self):
        # compute the travel time for each component of the trivial solution
        self.t1 = self.calct1(self.x[0], self.x[2])
        self.t2 = self.calct2(self.x[0], self.x[1], self.x[2], self.x[3])
        self.t3 = self.calct3(self.x[0], self.x[1], self.x[3])
        # compute the total travel time
        tt = self.t1 + self.t2 + self.t3
        # see if it respects time constraint
        self.tt = tt
        # self.get_three_comp_trj()
        # t, d, s = self.get_three_comp_trj()
        # return t, d, s
        return self.tt, 0, self.vcont

    def weakfeasibility(self, x):
        if np.sign(x[0] - self.fol_veh.curr_speed) != np.sign(x[2]) or np.sign(x[1] - x[0]) != np.sign(x[3]):
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
        dv1 = v2 - self.fol_veh.curr_speed
        if abs(dv1) > self.EPS and abs(a1) > self.EPS:
            return (v2 - self.fol_veh.curr_speed) / a1
        else:
            return 0

    def calct2(self, v2, v3, a1, a3):
        dv1 = v2 - self.fol_veh.curr_speed
        dv3 = v3 - v2
        if abs(dv1) > self.EPS and abs(a1) > self.EPS and abs(dv3) > self.EPS and abs(a3) > self.EPS:
            return (self.fol_veh.trajectory[0, 1] - (v2 ** 2 - self.fol_veh.curr_speed ** 2) / (2 * a1) - (
                    v3 ** 2 - v2 ** 2) / (
                            2 * a3)) / v2
        elif abs(dv1) <= self.EPS or abs(a1) <= self.EPS:
            return (self.fol_veh.trajectory[0, 1] - (v3 ** 2 - v2 ** 2) / (2 * a3)) / v2
        elif abs(dv3) <= self.EPS or abs(a3) <= self.EPS:
            return (self.fol_veh.trajectory[0, 1] - (v2 ** 2 - self.fol_veh.curr_speed ** 2) / (2 * a1)) / v2
        else:
            return self.fol_veh.trajectory[0, 1] / v2

    def calct3(self, v2, v3, a3):
        dv3 = v3 - v2
        if abs(dv3) > self.EPS and abs(a3) > self.EPS:
            return (v3 - v2) / a3

    def dv1(self, dt, a1):
        return self.fol_veh.trajectory[
                   0, 1] - self.fol_veh.curr_speed * dt - a1 * dt ** 2 / 2, a1 * dt + self.fol_veh.curr_speed

    def dv2(self, dt, t1, v2, a1):
        return self.dv1(t1, a1)[0] - v2 * dt, v2

    def dv3(self, dt, t1, t2, v2, a1, a3):
        return self.dv2(t2, t1, v2, a1)[0] - v2 * dt - a3 * dt ** 2 / 2, a3 * dt + v2

    def d_three_comp(self, t, t1, t2, v2, a1, a3):
        dt = t - self.fol_veh.trajectory[0, 0]
        if dt <= t1:
            return self.dv1(dt, a1)
        elif dt <= t1 + t2:
            return self.dv2(dt - t1, t1, v2, a1)[0]
        else:
            return self.dv3(dt - t1 - t2, t1, t2, v2, a1, a3)

    def get_three_comp_trj(self):
        tt = self.traveltime(np.array([self.x[0], self.x[1], self.x[2], self.x[3]]),
                             self.fol_veh.trajectory[0, 1], self.fol_veh.curr_speed, self.gs)
        t1 = self.t1
        t2 = self.t2

        tend = self.fol_veh.trajectory[0, 0] + tt
        t = self.create_trj_domain(self.fol_veh.trajectory[0, 0], tend)

        vd = np.vectorize(self.d_three_comp)  # todo Mahmoud: check this
        d, s = vd(t, t1, t2, self.x[0], self.x[2], self.x[3])

        return t, d, s

    def get_one_comp_trj(self, t1, d1, s1, t2, d2, s2):
        '''
        Solves:
        Minimize:     c^T * x

        Subject to:   A_ub * x <= b_ub
                      A_eq * x == b_eq
        (https://docs.scipy.org/doc/scipy/reference/generated/scipy.optimize.linprog.html)

        :param t1: start time stamp
        :param d1: start distance to stop bar
        :param s1: start speed
        :param t2: end time stamp
        :param d2: end distance to stop bar
        :param s2: end speed
        :param k:  poly degree
        :return:   t,d,s vectors
        '''
        tt = t2 - t1
        t = self.create_trj_domain(0, tt)

        c = [round(tt / (n + 1), self.DIG) for n in range(self.K, -1, -1)]
        a1_coeff_der = round(1 / tt, self.DIG)
        A1 = [
            [0 for n in range(self.K, 0, -1)] + [1],
            [0 for n in range(self.K, 1, -1)] + [a1_coeff_der, 0],
            [1 for n in range(self.K, -1, -1)],
            [round(n / tt, self.DIG) for n in range(self.K, 0, -1)] + [0],
        ]
        b1 = [d1, -s1, d2, -s2, ]

        m = 3 * k  # number ot segments is m + 2
        dt = round(1 / (m + 1), 3)  # time steps

        # first set of consts is for non-negative speed
        # second set is for maximum speed
        A2 = [[round(n * (i * dt) ** (n - 1) / t2, self.DIG) for n in range(self.K, 1, -1)] + [a1_coeff_der, 0] for i in
              range(m, 0, -1)] + [
                 [round(-n * (i * dt) ** (n - 1) / t2, self.DIG) for n in range(self.K, 1, -1)] + [-a1_coeff_der, 0]
                 for i in range(m, 0, -1)]
        b2 = [0 for n in range(m)] + [1.5 * self.vmax for n in range(m)]  # todo: relaxed max speed a little

        bnds = [(None, None) for n in range(self.K, -1, -1)]

        print('\nOBJECTIVE:')
        for i in range(self.K, -1, -1):
            print('+{:2.4f}*a{:d} '.format(round(c[i], 1), self.K - i), end='')

        print('\nEQUALITY CONSTS:')
        for j in range(4):
            for i in range(self.K, -1, -1):
                print('+{:2.4f}*a{:d} '.format(round(A1[j][i], 4), self.K - i), end='')
            print(' = {:2.2f}'.format(round(b1[j], 4)))

        print('\nINEQUALITY CONSTS:')
        for j in range(2 * m):
            for i in range(self.K, -1, -1):
                print('{:2.4f}*a{:d} '.format(round(A2[j][i], 4), self.K - i), end='')
            print(' < {:2.2f}'.format(round(b2[j], 4)))

        opt_coeffs = linprog(c, A_ub=A2, b_ub=b2, A_eq=A1, b_eq=b1, bounds=bnds, options={"disp": True})
        if not opt_coeffs.success:
            print('* trj optimizer did not return value. (Did linear trajectory instead)')
            self.linear_trj()
            return

        # def print_matrix(A):
        #     ''' Simply prints matrix A
        #     '''
        #     print('\n'.join([''.join(['{:4}  '.format(item) for item in row])
        #                      for row in A]))
        #     print('\n')
        #
        # print_matrix(A1)
        # print_matrix(A2)

        def hornereval(x, a, tt, k):
            ''' Uses Horner's method to compute and return the polynomial
            a[0] + a[1] x^1 + a[2] x^2 + ... + a[n-1] x^(n-1)
            evaluated at x.

            :param x: the point we want to evaluate polynomial and its derivative
            :param a: the coefficient vector in reverse [a[n-1], ... , a[0]]
            :param t2: the denominator of term inside parenthesis
            :param k: degree of poly
            :return function and its derivative e:
            '''
            dist, speed = 0, 0
            for i in range(k):
                dist = a[i] + (x * dist)
                speed = -(k - i) * a[i] / tt + (x * speed)
            return a[-1] + (x * dist), speed

        def print_poly(a, tt):
            print('f{:d}[t_]:='.format(k))
            for i in range(len(a)):
                if a[i] > 0:
                    print('+{:2.4f}*(t/{:2.4f})^{:d}'.format(a[i], tt, len(a) - i - 1), end=' ')
                elif a[i] < 0:
                    print('{:2.4f}*(t/{:2.4f})^{:d}'.format(a[i], tt, len(a) - i - 1), end=' ')
            print('\n')

        print_poly(opt_coeffs.x, tt)

        d, s = hornereval(t / tt, opt_coeffs.x, tt, self.K)

        return np.round(t + self.fol_veh.trajectory[0, 0], self.DIG), np.round(d, self.DIG), np.round(s, self.DIG)

# HOW TO SOLVE LTVO VIA SCIPY
# self.bnds = (
#     (0, self.vmax), (0, self.vcont), (self.fol_veh.amin, self.fol_veh.amax),
#     (self.fol_veh.amin, self.fol_veh.amax))
#
# self.con = (  # const 0: tt greater than equal to gs
#     {'type': 'ineq',
#      'fun': lambda x, d0, v0, gs_rel:
#      np.array((x[3] * (v0 - x[0]) ** 2 + x[2] * (
#              2 * x[3] * d0 - (x[0] - x[1]) ** 2))
#               / (2 * x[2] * x[3] * x[0]) - gs_rel),
#      'jac': self.ttime_der,
#      'args': (self.fol_veh.dist, self.fol_veh.speed, self.gs-self.fol_veh.det_time,)},
#     # const 1: tt equal to gs
#     {'type': 'eq',
#      'fun': lambda x, d0, v0, t_opt:
#      np.array((x[3] * (v0 - x[0]) ** 2 + x[2] * (
#              2 * x[3] * d0 - (x[0] - x[1]) ** 2))
#               / (2 * x[2] * x[3] * x[0]) - t_opt),
#      'jac': self.ttime_der,
#      'args': (self.fol_veh.dist, self.fol_veh.speed, self.tt,)},
#     # const 2: t1 >=0
#     {'type': 'ineq',
#      'fun': lambda x, d0, v0, t_opt:
#      np.array((x[0] - v0) / x[2]),
#      'jac': lambda x, d0, v0, t_opt:
#      np.array([1 / x[2], 0, (v0 - x[0]) / x[2] ** 2, 0]),
#      'args': (
#          self.fol_veh.dist, self.fol_veh.speed, self.tt,)},
#     # const 3: t3 >=0
#     {'type': 'ineq',
#      'fun': lambda x, d0, v0, t_opt:
#      np.array((x[0] - v0) / x[2]),
#      'jac': lambda x, d0, v0, t_opt:
#      np.array([-1 / x[3], 1 / x[3], 0, (x[1] - x[0]) / x[3] ** 2]),
#      'args': (
#          self.fol_veh.dist, self.fol_veh.speed, self.tt,)},
#     # const 4: t2 >=0
#     {'type': 'ineq',
#      'fun': lambda x, d0, v0, t_opt:
#      np.array((x[0] ** 2 * (x[2] - x[3]) - x[1] ** 2 * x[2] + (v0 ** 2 + 2 * d0 * x[2]) * x[3]) / (
#              2 * x[0] * x[2] * x[3])),
#      'jac': lambda x, d0, v0, t_opt:
#      np.array([((x[0] ** 2 + x[1] ** 2) * x[2] - (v0 ** 2 + x[0] ** 2 + 2 * d0 * x[2]) * x[3]) / (
#              2 * x[2] * x[3] * x[0] ** 2), -1 * x[1] / (x[0] * x[3]),
#                (x[0] - v0) * (v0 + x[0]) / (2 * x[0] * x[2] ** 2),
#                (x[1] ** 2 - x[0] ** 2) / (2 * x[0] * x[3] ** 2)]),
#      'args': (
#          self.fol_veh.dist, self.fol_veh.speed, self.tt,)},
# )
# def solve(self):
#     '''
#     Solves lead vehicle trajectory optimization problem
#     '''
#     min_travel_time_sol = minimize(self.traveltime,
#                                    np.array([self.vmax, self.vcont, self.fol_veh.amax, self.fol_veh.amax]),
#                                    args=(self.fol_veh.dist, self.fol_veh.speed, self.gs - self.fol_veh.det_time,),
#                                    jac=self.ttime_der, bounds=self.bnds,
#                                    constraints=(self.con[k] for k in (0, 2, 3, 4,)), method='SLSQP',
#                                    options={'disp': True})
#     sol = np.round(np.array([min_travel_time_sol.x[k] for k in (0, 1, 2, 3,)]), 3)
#
#     if self.weakfeasibility(sol):
#         if min_travel_time_sol.x[1] < self.vcont - self.EPS:  # see if we can improve discharge speed
#             self.tt = min_travel_time_sol.fun
#
#             def depart_spd(x, d0, v0, t_opt):
#                 np.array(-1 * x[1])  # since it's minimization
#
#             def depart_spd_der(x, d0, v0, t_opt):
#                 np.array([0, -1, 0, 0])
#
#             max_speed_sol = minimize(depart_spd,
#                                      min_travel_time_sol.x,
#                                      args=(self.fol_veh.dist, self.fol_veh.speed, min_travel_time_sol.fun,),
#                                      jac=depart_spd_der, bounds=self.bnds,
#                                      constraints=(self.con[k] for k in (1, 2, 3, 4,)), method='SLSQP',
#                                      options={'disp': True})
#             if self.weakfeasibility(max_speed_sol.x):
#                 self.setopt(max_speed_sol.x, max_speed_sol.fun)
#             else:
#                 self.setopt(min_travel_time_sol.x, min_travel_time_sol.fun)
#
#         else:
#             self.setopt(min_travel_time_sol.x, min_travel_time_sol.fun)
#     else:
#         raise Exception('Review arrival info since trj optimizer is infeasible')
