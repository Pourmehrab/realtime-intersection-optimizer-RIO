'''
The standard format of optimization problem:
minimize f(x)
    subject to:
    g_i(x) >= 0,  i = 1,...,m
    h_j(x)  = 0,  j = 1,...,p


By:     Mahmoud Pourmehrab
E-mail: mpourmehrab@ufl.edu
Date:        Nov 2017
Last update: Nov/25/2017
'''

import numpy as np
from cmp.vis import MahmoudVisual
from src.trj import MahmoudTrj
from scipy.optimize import minimize


class MahmoudLAVO(MahmoudTrj):
    def __init__(self, t0, d0, v0, gs=0, gt=86400, fdeg=0, vmax=15, vcont=10, amin=-4.5, amax=3):
        '''

        :param fdeg:        0: free, 1: dist-const, 2: time and dist-const
        '''
        self.insight()

        super().__init__(t0, d0, v0, gs, gt, vmax, vcont, amin, amax)
        self.fdeg = fdeg
        self.x = np.array([vmax, vcont, amax, amin], dtype=float)
        self.t1, self.t2, self.t3, self.tt = 0, 0, 0, gt + 1

    def solve(self):
        if self.fdeg == 0:
            self.LVTOfree()
        elif self.fdeg == 1:
            self.LVTOsingly()
        else:
            self.LVTOdoublly()

    def LVTOfree(self):
        ''' In this case vehicle speeds up to v3, d0 meters of its detection location
        this case we know what's the limit on v3 after traveling d0
        '''
        # slowify = False
        # v3 = self.vcont
        # for a1 in (self.amin, self.amax):
        #     for a3 in (self.amin, self.amax):
        #         v2 = self.vmax  # four solutions are complete, check the bounds
        #         tt = self.tTime(np.array[v2, v3, a1, a3], self.d0, self.v0, self.gs, self.gt)
        #         if self.partialtest(v2, v3, a1, a3):
        #             print('A free trj is found that requires {:04.2f} sec'.format(tt))
        #             if tt < self.gs:
        #                 # slow it down by arriving at epsilon after green starts
        #                 slowify = True
        #             elif tt <= self.gt:
        #                 # a feasible solution is found, compare it to the best one
        #                 if tt < self.tt:
        #                     self.setopt(v2, v3, a1, a3, tt)
        #
        # if not self.stat and slowify:
        #     self.LVTOdoublly()
        bnds = ((0, self.vmax), (0, self.vcont), (self.amin, self.amax), (self.amin, self.amax))
        cons = ({'type': 'ineq',
                 'fun': lambda x, d0, v0, gs: (
                                                      x[3] * (v0 - x[0]) ** 2 + x[2] * (
                                                      2 * x[3] * d0 - (x[0] - x[1]) ** 2))
                                              / (2 * x[2] * x[3] * x[0]) - gs,
                 'jac': self.tTimeDer,
                 'args': (self.d0, self.v0, self.gs,)})

        sol = minimize(self.tTime, np.array([self.vmax, self.vcont, self.amax, self.amax]),
                       args=(self.d0, self.v0, self.gs,), jac=self.tTimeDer,
                       bounds=bnds, constraints=cons, method='SLSQP', options={'disp': True})

        self.buildtrj(sol.x)

        vis = MahmoudVisual(6)
        vis.plotrj(self.indepVar, self.depVar, 2)
        vis.makeplt()

        if sol.success and self.partialtest(sol.x):
            self.setopt(sol.x, sol.fun)
        print(sol.x)

    def LVTOsingly(self):
        ''' exact distance constraint is given
        (only for queuing) '''
        pass

    def LVTOdoublly(self):
        ''' both min time and exact distance constraints are active
        maximize v3 '''
        bnds = ((0, self.vmax), (0, self.vcont), (self.amin, self.amax), (self.amin, self.amax))
        cons = ({'type': 'ineq',
                 'fun': lambda x, d0, v0, gs: (
                                                      x[3] * (v0 - x[0]) ** 2 + x[2] * (
                                                      2 * x[3] * d0 - (x[0] - x[1]) ** 2))
                                              / (2 * x[2] * x[3] * x[0]) - gs,
                 'jac': self.tTimeDer,
                 'args': (self.d0, self.v0, self.gs,)})

        sol = minimize(self.tTime, np.array([self.vmax, self.vcont, self.amax, self.amax]),
                       args=(self.d0, self.v0, self.gs,), jac=self.tTimeDer,
                       bounds=bnds, constraints=cons, method='SLSQP', options={'disp': True})
        print(sol.x)

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

    def partialtest(self, x):
        if np.sign(x[0] - self.v0) != np.sign(x[2]) or np.sign(x[1] - x[0]) != np.sign(x[3]):
            # Solution is not feasible
            return False
        else:
            # Solution may be feasible
            return True

    def tTime(self, x, d0, v0, gs):
        return (x[3] * (v0 - x[0]) ** 2 + x[2] * (2 * x[3] * d0 - (x[0] - x[1]) ** 2)) \
               / (2 * x[2] * x[3] * x[0])

    def tTimeDer(self, x, d0, v0, gs):
        return np.array([(x[3] * (-v0 ** 2 + x[0] ** 2) + x[2] * (-2 * x[3] * d0 - x[0] ** 2 + x[1] ** 2)) / (
                2 * x[2] * x[3] * x[0] ** 2)
                            , (x[0] - x[1]) / (x[3] * x[0])
                            , -((v0 - x[0]) ** 2 / (2 * x[2] ** 2 * x[0]))
                            , (x[0] - x[1]) ** 2 / (2 * x[3] ** 2 * x[0])])

    def calct1(self, v2, a1):
        dv1 = v2 - self.v0
        if abs(dv1) > self.EPS and abs(a1) > self.EPS:
            return (v2 - self.v0) / a1

    def calct2(self, v2, v3, a1, a3):
        dv1 = v2 - self.v0
        dv3 = v3 - v2
        if abs(dv1) > self.EPS and abs(a1) > self.EPS and abs(dv3) > self.EPS and abs(a3) > self.EPS:
            return (self.d0 - (v2 ** 2 - self.v0 ** 2) / (2 * a1) - (v3 ** 2 - v2 ** 2) / (2 * a3)) / v2
        elif abs(dv1) <= self.EPS or abs(a1) <= self.EPS:
            return (self.d0 - (v3 ** 2 - v2 ** 2) / (2 * a3)) / v2
        elif abs(dv3) <= self.EPS or abs(a3) <= self.EPS:
            return (self.d0 - (v2 ** 2 - self.v0 ** 2) / (2 * a1)) / v2
        else:
            return self.d0 / v2

    def calct3(self, v2, v3, a3):
        dv3 = v3 - v2
        if abs(dv3) > self.EPS and abs(a3) > self.EPS:
            return (v3 - v2) / a3

    def d1(self, dt, a1):
        # print('{}'.format(self.d0 - self.v0 * dt - a1 * dt ** 2 / 2))
        return self.d0 - self.v0 * dt - a1 * dt ** 2 / 2

    def d2(self, dt, t1, v2, a1):
        # print('{}'.format(self.d1(t1, a1) - v2 * dt))
        return self.d1(t1, a1) - v2 * dt

    def d3(self, dt, t1, t2, v2, a1, a3):
        # print('{}'.format(self.d2(t2, t1, v2, a1) - v2 * dt - a3 * dt ** 2 / 2))
        return self.d2(t2, t1, v2, a1) - v2 * dt - a3 * dt ** 2 / 2

    def d(self, t, t1, t2, v2, a1, a3):
        dt = t - self.t0
        if dt <= t1:
            return self.d1(dt, a1)
        elif dt <= t1 + t2:
            return self.d2(dt - t1, t1, v2, a1)
        else:
            return self.d3(dt - t1 - t2, t1, t2, v2, a1, a3)

    def buildtrj(self, x):
        tt = self.tTime(np.array([x[0], x[1], x[2], x[3]]), self.d0, self.v0, self.gs)
        t1 = self.calct1(x[0], x[2])
        t2 = self.calct2(x[0], x[1], x[2], x[3])

        tend = self.t0 + tt
        if tend % self.RES > self.EPS:
            self.indepVar = np.append(np.arange(self.t0, self.t0 + tt, MahmoudLAVO.RES, dtype=float), tend).round(
                self.DIG)
        else:
            self.indepVar = np.arange(self.t0, self.t0 + tt, MahmoudLAVO.RES, dtype=float).round(self.DIG)

        vd = np.vectorize(self.d)
        self.depVar = vd(self.indepVar, t1, t2, x[0], x[2], x[3])
        # control: the last point of trajectory should be zero


if __name__ == "__main__":
    x = MahmoudLAVO(13, 500, 17, 40)
    x.solve()
    # x.buildtrj(13.1345, 4.47734, -1.11671, -1.16845)
    #
    # vis = MahmoudVisual(6)
    # vis.plotrj(x.indepVar, x.depVar, 2)
    # vis.makeplt()

    print()
