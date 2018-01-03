'''
Goal: To optimizes the trajectory of an Automated Vehicle (AV)

By:             Mahmoud Pourmehrab
E-mail:         mpourmehrab@ufl.edu
Date:           Nov 2017
Last update:    Jan/01/2018
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
        :param fdeg:            0: free (when lead_veh is None), 1: dist-const, 2: time and dist-const (only for the first AV)
        :param vmax:            speed limit in m/s
        :param vcont:           speed limit at the control point in m/s
        '''

        super().__init__(lead_veh, fol_veh, gs, gt)

        self.vmax, self.vcont = vmax, vcont

        self.insight()

        self.fdeg = fdeg
        # solution is of the form v2, v3, a1, a3
        self.x = np.array([vmax, vcont, fol_veh.amax, fol_veh.amin], dtype=float)
        # assume maximum acceleration/deceleration is gonna be possible
        self.x[2] = fol_veh.amax if vmax > fol_veh.speed else fol_veh.amin
        self.x[3] = fol_veh.amax if vcont > vmax else fol_veh.amin

        if self.fdeg < 1:  # Solve LTVO
            self.solve()
        else:
            self.ctrl_get_one_comp_trj()

    def insight(self):
        print(''' MahmoudTrj(.) received the following request at {:04.2f} sec:
                dist: {:04.2f} m             initial speed: {:04.2f} m/s
                deceleration: {:04.2f} m/s2   acceleration: {:04.2f} m/s2
                spd limit: {:04.2f} m/s       spd limit @ control: {:04.2f} m/s
                green interval:            [{:04.2f}, {:04.2f}] sec
                '''.format(self.fol_veh.det_time, self.fol_veh.dist, self.fol_veh.speed, self.fol_veh.amin,
                           self.fol_veh.amax, self.vmax, self.vcont, self.gs, self.gt))

        if self.fol_veh.speed > self.vmax:
            t = (self.vmax - self.fol_veh.speed) / self.fol_veh.amin
            d = (self.vmax ** 2 - self.fol_veh.speed ** 2) / (2 * self.fol_veh.amin)
            print('Vehicle can decelerate to spd limit in {:04.2f} sec, {:04.2f} m'.format(t, d))
        else:
            t = (self.vmax - self.fol_veh.speed) / self.fol_veh.amax
            d = (self.vmax ** 2 - self.fol_veh.speed ** 2) / (2 * self.fol_veh.amax)
            print('Vehicle can accelerate to spd limit in {:04.2f} sec, {:04.2f} m'.format(t, d))
        if self.vcont < self.vmax:
            t = (self.vcont - self.vmax) / self.fol_veh.amin
            d = (self.vcont ** 2 - self.vmax ** 2) / (2 * self.fol_veh.amin)
            print('then decelerates to control spd limit in {:04.2f} sec, {:04.2f} m'.format(t, d))
        elif self.vcont > self.vmax:
            t = (self.vcont - self.vmax) / self.fol_veh.amax
            d = (self.vcont ** 2 - self.vmax ** 2) / (2 * self.fol_veh.amax)
            print('then accelerates to control spd limit in {:04.2f} sec, {:04.2f} m'.format(t, d))

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
            self.get_three_comp_trj()
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
                t, d, s = self.get_three_comp_trj()
                return
            else:  # was not time feasible. More work is needed
                t, d, s = self.get_one_comp_trj()

        self.fol_veh.set_trj(t, d, s)
        # self.weakfeasibility(self.x)

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

    def dv1(self, dt, a1):
        return self.fol_veh.dist - self.fol_veh.speed * dt - a1 * dt ** 2 / 2, a1 * dt + self.fol_veh.speed

    def dv2(self, dt, t1, v2, a1):
        return self.dv1(t1, a1)[0] - v2 * dt, v2

    def dv3(self, dt, t1, t2, v2, a1, a3):
        return self.dv2(t2, t1, v2, a1)[0] - v2 * dt - a3 * dt ** 2 / 2, a3 * dt + v2

    def d_three_comp(self, t, t1, t2, v2, a1, a3):
        dt = t - self.fol_veh.det_time
        if dt <= t1:
            return self.dv1(dt, a1)
        elif dt <= t1 + t2:
            return self.dv2(dt - t1, t1, v2, a1)[0]
        else:
            return self.dv3(dt - t1 - t2, t1, t2, v2, a1, a3)

    def get_three_comp_trj(self):
        tt = self.traveltime(np.array([self.x[0], self.x[1], self.x[2], self.x[3]]),
                             self.fol_veh.dist, self.fol_veh.speed, self.gs)
        t1 = self.calct1(self.x[0], self.x[2])
        t2 = self.calct2(self.x[0], self.x[1], self.x[2], self.x[3])

        tend = self.fol_veh.det_time + tt
        indep_var = self.create_trj_domain(self.fol_veh.det_time, tend)

        vd = np.vectorize(self.d_three_comp)
        dep_var, speed = vd(indep_var, t1, t2, self.x[0], self.x[2], self.x[3])

        return indep_var, dep_var, speed

    def ctrl_get_one_comp_trj(self):
        t, d, s = self.get_one_comp_trj(self.fol_veh.det_time, self.fol_veh.dist, self.fol_veh.speed, self.gs, 0,
                                        self.vcont)
        return t, d, s

    def get_one_comp_trj(self, t1, d1, s1, t2, d2, s2, k=3):
        '''
        :param t1: start time stamp
        :param d1: start distance to stopbar
        :param s1: start speed
        :param t2: end time stamp
        :param d2: end distance to stopbar
        :param s2: end speed
        :param k:  poly degree
        :return:   t,d,s vectors
        '''
        tt = t2 - t1
        indep_var = self.create_trj_domain(0, tt)
        s = 1  # scales the constraints

        c = [tt / (n + 1) for n in range(k, -1, -1)]
        a1_coeff_der = s / tt
        A1 = [
            [0 for n in range(k, 0, -1)] + [s],
            [0 for n in range(k, 1, -1)] + [a1_coeff_der, 0],
            [s for n in range(k, -1, -1)],
            [s * n / tt for n in range(k, 0, -1)] + [0],
        ]
        b1 = [s * d1, -1 * s * s1, s * d2, -1 * s * s2, ]

        m = 2 * k  # number ot segments is m+2
        dt = 1 / (m + 1)  # time steps

        # first set of consts is for non-negative speed
        # second set is for maximum speed
        A2 = [[s * n * (i * dt) ** (n - 1) / t2 for n in range(k, 1, -1)] + [a1_coeff_der, 0] for i in
              range(m, 0, -1)] + [[-s * n * (i * dt) ** (n - 1) / t2 for n in range(k, 1, -1)] + [-1 * a1_coeff_der, 0]
                                  for i in range(m, 0, -1)]
        b2 = [0 for n in range(m)] + [self.vmax for n in range(m)]

        bnds = [(None, None) for n in range(k, -1, -1)]
        # to print constraints
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

        def print_matrix(A):
            ''' Simply prints matrix A
            '''
            print('\n'.join([''.join(['{:4}  '.format(item) for item in row])
                             for row in A]))
            print('\n')

        # print_matrix(A1)
        # print_matrix(A2)

        def hornereval(x, a, t2, k):
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
                speed = -(k - i) * a[i] / t2 + (x * speed)
            return a[-1] + (x * dist), speed

        def print_poly(a, t2):
            print('f{:d}[t_]:='.format(k))
            for i in range(len(a)):
                if a[i] > 0:
                    print('+{:2.4f}*(t/{:2.4f})^{:d}'.format(a[i], t2, len(a) - i - 1), end=' ')
                elif a[i] < 0:
                    print('{:2.4f}*(t/{:2.4f})^{:d}'.format(a[i], t2, len(a) - i - 1), end=' ')
            print('\n')

        # print_poly(opt_coeffs.x, t2)

        dep_var_dist, dep_var_speed = hornereval(indep_var / t2, opt_coeffs.x, t2, k)

        return indep_var + self.fol_veh.det_time, dep_var_dist, dep_var_speed
