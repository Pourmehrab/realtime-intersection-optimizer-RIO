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
from src.vis import MahmoudVisual
from src.trj import MahmoudTrj
from scipy.optimize import minimize


class MahmoudLAVO(MahmoudTrj):
    def __init__(self, t0, d0, v0, gs=0, gt=86400, fdeg=2, vmax=15, vcont=10, amin=-4.5, amax=3):
        '''

        :param d0:          distance to control point
        :param v0:          initial speed in m/s
        :param gs:          start of green in s
        :param gt:          end of green in s
        :param fdeg:        0: free, 1: dist-const, 2: time and dist-const
        :param vmax:        speed limit in m/s
        :param vcont:       speed limit at the control point in m/s
        :param amin:        deceleration rate in m/s2
        :param amax:        acceleration rate in m/s2
        '''
        self.t0, self.d0, self.v0 = t0, d0, v0

        self.fdeg = fdeg
        self.vmax, self.vcont = vmax, vcont
        self.amin, self.amax = amin, amax
        self.gs, self.gt = gs, gt
        self.x = np.array([vmax, vcont, amax, amin], dtype=float)
        self.t1, self.t2, self.t3, self.tt, self.stat = 0, 0, 0, gt + 1, False

        self.insight()

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
        slowify = False
        v3 = self.vcont
        for a1 in (self.amin, self.amax):
            for a3 in (self.amin, self.amax):
                v2 = self.vmax  # four solutions are complete, check the bounds
                tt = self.tTime(v2, v3, a1, a3)
                if self.partialtest(v2, v3, a1, a3):
                    print('A free trj is found that requires {:04.2f} sec'.format(tt))
                    if tt < self.gs:
                        # slow it down by arriving at epsilon after green starts
                        slowify = True
                    elif tt <= self.gt:
                        # a feasible solution is found, compare it to the best one
                        if tt < self.tt:
                            self.setopt(v2, v3, a1, a3, tt)

        if not self.stat and slowify:
            # We have time and distance constraint
            # Maximize v3
            v3 = self.vcont
            while v3 >= 0:
                print()

    def LVTOsingly(self):
        pass

    def LVTOdoublly(self):
        pass

    def setopt(self, v2, v3, a1, a3, tt):
        self.stat = True
        self.x[0] = v2
        self.x[1] = v3
        self.x[2] = a1
        self.x[3] = a3
        self.t1 = self.calct1(v2, a1)
        self.t3 = self.calct3(v2, v3, a3)
        self.t2 = tt - self.t1 - self.t3
        self.tt = tt

    def partialtest(self, v2, v3, a1, a3):
        if np.sign(v2 - self.v0) != np.sign(a1) or np.sign(v3 - v2) != np.sign(a3):
            # Solution is not feasible
            return False
        else:
            # Solution may be feasible
            return True

    def tTime(self, v2, v3, a1, a3):
        return (a3 * (self.v0 - v2) ** 2 + a1 * (2 * a3 * self.d0 - (v2 - v3) ** 2)) \
               / (2 * a1 * a3 * v2)

    def calct1(self, v2, a1):
        dv1 = v2 - self.v0
        if abs(dv1) > self.eps and abs(a1) > self.eps:
            return (v2 - self.v0) / a1

    def calct2(self, v2, v3, a1, a3):
        dv1 = v2 - self.v0
        dv3 = v3 - v2
        if abs(dv1) > self.eps and abs(a1) > self.eps and abs(dv3) > self.eps and abs(a3) > self.eps:
            return (self.d0 - (v2 ** 2 - self.v0 ** 2) / (2 * a1) - (v3 ** 2 - v2 ** 2) / (2 * a3)) / v2
        elif abs(dv1) <= self.eps or abs(a1) <= self.eps:
            return (self.d0 - (v3 ** 2 - v2 ** 2) / (2 * a3)) / v2
        elif abs(dv3) <= self.eps or abs(a3) <= self.eps:
            return (self.d0 - (v2 ** 2 - self.v0 ** 2) / (2 * a1)) / v2
        else:
            return self.d0 / v2

    def calct3(self, v2, v3, a3):
        dv3 = v3 - v2
        if abs(dv3) > self.eps and abs(a3) > self.eps:
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

    def buildtrj(self, v2, v3, a1, a3):
        tt = self.tTime(v2, v3, a1, a3)
        t1 = self.calct1(v2, a1)
        t2 = self.calct2(v2, v3, a1, a3)

        tend = self.t0 + tt
        if tend % self.res > self.eps:
            self.indepVar = np.append(np.arange(self.t0, self.t0 + tt, MahmoudLAVO.res, dtype=float), tend).round(
                self.deci)
        else:
            self.indepVar = np.arange(self.t0, self.t0 + tt, MahmoudLAVO.res, dtype=float).round(self.deci)

        vd = np.vectorize(self.d)
        self.depVar = vd(self.indepVar, t1, t2, v2, a1, a3)
        # control: the last point of trajectory should be zero


if __name__ == "__main__":
    x = MahmoudLAVO(13, 500, 17, 40)
    x.buildtrj(13.1345, 4.47734, -1.11671, -1.16845)

    vis = MahmoudVisual(6)
    vis.plotrj(x.indepVar, x.depVar, 2)
    vis.makeplt()

    print()
