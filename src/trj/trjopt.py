'''
Goal: To optimizes the trajectory of an Automated Vehicle (AV)

By:     Mahmoud Pourmehrab
E-mail: mpourmehrab@ufl.edu
Date:        Nov 2017
Last update: Dec/14/2017
'''

import numpy as np
from scipy.optimize import linprog

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

        # self.insight()

        self.fdeg = fdeg
        # solution is of the form v2, v3, a1, a3
        self.x = np.array([vmax, vcont, fol_veh.amax, fol_veh.amin], dtype=float)
        # assume maximum acceleration is gonna be possible
        self.x[2] = fol_veh.amax if vmax > fol_veh.speed else fol_veh.amin
        self.x[3] = fol_veh.amax if vcont > vmax else fol_veh.amin


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

    def solve(self):
        # compute the travel time for each component of the trivial solution
        self.t1 = self.calct1(self.x[0], self.x[2])
        self.t2 = self.calct2(self.x[0], self.x[1], self.x[2], self.x[3])
        self.t3 = self.calct3(self.x[0], self.x[1], self.x[3])
        # compute the total travel time
        tt = self.t1 + self.t2 + self.t3
        # see if it respects time constraint
        if tt >= self.gs - self.fol_veh.det_time:
            # trivial solution is optimal
            self.tt = tt
            self.stat = True
            self.set_three_comp_trj()
            return
        else:  # need to do some more work to find the optimal
            # set second component to zero
            self.t2 = 0
            # compute the acceleration for transition to vcont
            a_trans = (self.vcont ** 2 - self.fol_veh.speed ** 2) / (2 * self.fol_veh.dist)
            # Distance feasible, see if time-feasible
            tt = self.calct1(self.vcont, a_trans)
            if tt >= self.gs - self.fol_veh.det_time:
                # optimal solution is found
                # set second component to zero
                self.t3 = 0
                # set first component
                self.x[3] = a_trans
                self.t1 = tt
                self.tt = tt
                self.stat = True
                self.set_three_comp_trj()
                return
            else:  # was not time feasible. More work is needed
                self.set_one_comp_trj()

        # self.weakfeasibility(self.x)

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
    #     d, t = self.buildtrj(sol)
    #     plot = MahmoudVisTrj(0)  # todo:(Mahmoud) do we need lane information in this file?
    #     plot.plotrj(t, d)
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
        else:
            return 0

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

    def d_three_comp(self, t, t1, t2, v2, a1, a3):
        dt = t - self.fol_veh.det_time
        if dt <= t1:
            return self.d1(dt, a1)
        elif dt <= t1 + t2:
            return self.d2(dt - t1, t1, v2, a1)
        else:
            return self.d3(dt - t1 - t2, t1, t2, v2, a1, a3)

    def create_trj_domain(self, ts, te):
        '''

        :param ts: start time
        :param te: end time
        :return: trj domain
        '''
        # todo: (Mahmoud) control the last point of trajectory should be zero
        if te - ts % self.RES > self.EPS:
            indep_var = np.append(np.arange(self.fol_veh.det_time, te, MahmoudAVO.RES, dtype=float), te).round(
                self.DIG)
        else:
            indep_var = np.arange(self.fol_veh.det_time, te, MahmoudAVO.RES, dtype=float).round(self.DIG)

        return indep_var

    def set_three_comp_trj(self):
        tt = self.traveltime(np.array([self.x[0], self.x[1], self.x[2], self.x[3]]),
                             self.fol_veh.dist, self.fol_veh.speed, self.gs)
        t1 = self.calct1(self.x[0], self.x[2])
        t2 = self.calct2(self.x[0], self.x[1], self.x[2], self.x[3])

        tend = self.fol_veh.det_time + tt
        indep_var = self.create_trj_domain(self.fol_veh.det_time, tend)

        vd = np.vectorize(self.d_three_comp)
        dep_var = vd(indep_var, t1, t2, self.x[0], self.x[2], self.x[3])
        self.fol_veh.set_trj(indep_var, dep_var)

    def set_one_comp_trj(self, k=9):
        tt = self.gs - self.fol_veh.det_time
        #
        # d0, v0, t0, vcont = self.fol_veh.dist, self.fol_veh.speed, self.fol_veh.det_time, self.vcont
        #
        # a = (2 * d0 - tt * (v0 + vcont)) / tt ** 3
        # b = (-3 * d0 * (2 * t0 + tt) + tt * (3 * t0 * (v0 + vcont) + tt * (2 * v0 + vcont))) / tt ** 3
        # c = (6 * d0 * t0 * (t0 + tt) - tt * ((t0 + tt) * (3 * t0 + tt) * v0 + t0 * (3 * t0 + 2 * tt) * vcont)) / tt ** 3
        # d = ((t0 + tt) * ((t0 + tt) * (d0 * (-2 * t0 + tt) + t0 * tt * v0) + t0 ** 2 * tt * vcont)) / tt ** 3
        #
        indep_var = self.create_trj_domain(0, tt)
        #
        # dep_var = a * indep_var ** 3 + b * indep_var ** 2 + c * indep_var + d
        # self.fol_veh.set_trj(indep_var, dep_var)
        t1 = 0
        t2 = tt
        s = 1  # scales the constraints

        c = [t2 / (n + 1) for n in range(k, -1, -1)]
        a1_coeff_der = s / t2
        A1 = [
            [0 for n in range(k, 0, -1)] + [s],
            [0 for n in range(k, 1, -1)] + [a1_coeff_der, 0],
            [s for n in range(k, -1, -1)],
            [s * n / t2 for n in range(k, 0, -1)] + [0],
        ]
        b1 = [s * self.fol_veh.dist, -1 * s * self.fol_veh.speed, 0, -1 * s * self.vcont, ]

        m = 2*k
        dt = 1 / (m + 1)

        # for i in range(m, 0, -1):
        #     print('{:3.2f} '.format(i * dt))

        # first set of consts is for nonnegative speed
        # second set is for maximum speed
        # print('/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\.')
        A2 = [[s * n * (i * dt) ** (n - 1) / t2 for n in range(k, 1, -1)] + [a1_coeff_der, 0] for i in
              range(m, 0, -1)] + [[-s * n * (i * dt) ** (n - 1) / t2 for n in range(k, 1, -1)] + [-1 * a1_coeff_der, 0]
                                  for i in range(m, 0, -1)]
        b2 = [0 for n in range(m)] + [self.vmax for n in range(m)]

        bnds = [(None, None) for n in range(k, -1, -1)]
        # for i in range(k, -1, -1):
        #     print('+{:2.4f}*a{:d} '.format(round(c[i], 1), k - i), end='')
        # print('\n')
        # for i in range(k, -1, -1):
        #     print('+{:2.4f}*a{:d} '.format(round(A1[0][i], 4), k - i), end='')
        # print(' =e= {:2.2f}'.format(round(b1[0], 4)))
        # for i in range(k, -1, -1):
        #     print('+{:2.4f}*a{:d} '.format(round(A1[1][i], 4), k - i), end='')
        # print(' =e= {:2.2f}'.format(round(b1[1], 4)))
        # for i in range(k, -1, -1):
        #     print('+{:2.4f}*a{:d} '.format(round(A1[2][i], 4), k - i), end='')
        # print(' =e= {:2.2f}'.format(round(b1[2], 4)))
        # for i in range(k, -1, -1):
        #     print('+{:2.4f}*a{:d} '.format(round(A1[3][i], 4), k - i), end='')
        # print(' =e= {:2.2f}\n'.format(round(b1[3], 4)))

        opt_coeffs = linprog(c, A_ub=A2, b_ub=b2, A_eq=A1, b_eq=b1, bounds=bnds, options={"disp": False})

        # print(opt_coeffs)

        def print_matrix(A):
            print('\n'.join([''.join(['{:4}  '.format(item) for item in row])
                             for row in A]))
            print('\n')

        # print_matrix(A1)
        # print_matrix(A2)

        def hornereval(x, a):
            '''Use Horner's method to compute and return the polynomial
               a[0] + a[1] x^1 + a[2] x^2 + ... + a[n-1] x^(n-1)
            evaluated at x.'''
            dist, speed = 0, 0
            for i in range(len(a) - 1):
                dist = a[i] + (x * dist)
                speed = i + 1 * a[i + 1] + (x * speed)
            return a[-1] + (x * dist), -1 * speed

        def plot_mathematica(a, t2):
            print('f{:d}[t_]:='.format(k))
            for i in range(len(a)):
                if a[i] > 0:
                    print('+{:2.4f}*(t/{:2.4f})^{:d}'.format(a[i], t2, len(a) - i - 1), end=' ')
                elif a[i] < 0:
                    print('{:2.4f}*(t/{:2.4f})^{:d}'.format(a[i], t2, len(a) - i - 1), end=' ')
            print('\n')

        plot_mathematica(opt_coeffs.x, t2)

        dep_var_dist, dep_var_speed = hornereval(indep_var / t2, opt_coeffs.x)
        # print(indep_var)
        # print(dep_var)
        self.fol_veh.set_trj(indep_var + self.fol_veh.det_time, dep_var_dist)
