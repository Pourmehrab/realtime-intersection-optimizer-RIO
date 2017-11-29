'''
The standard format of optimization problem:
minimize f(x)
    subject to:
    g_i(x) >= 0,  i = 1,...,m
    h_j(x)  = 0,  j = 1,...,p
'''

import numpy as np
from bokeh.plotting import figure
from bokeh.io import export_svgs, show
from bokeh.models import HoverTool


class MahmoudLAVO:
    '''
    By:     Mahmoud Pourmehrab
    E-mail: mpourmehrab@ufl.edu
    Date:        Nov 2017
    Last update: Nov/25/2017
    '''

    lag = np.array([1])  # lag on signalization
    res = 1  # second
    eps = 0.01
    deci = 2

    def __init__(self, t0, d0, v0, gs=0, gt=86400, fdeg=2, vmax=15, vcont=10, amin=-4.5, amax=3):
        '''

        :param d0:          distance to control point
        :param v0:          initial speed in m/s
        :param gs:          start of green in s
        :param gt:          end of green in s
        :param fdeg:        2: free, 1: dist-const, 0: time and dist-const
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
        if self.fdeg == 2:
            self.LVTOsol4sig()
        else:
            pass

    def insight(self):
        print('''MahmoudTrj(.) received the following request:
                dist: {:04.2f} m             initial speed: {:04.2f} m/s
                deceleration: {:04.2f} m/s2   acceleration: {:04.2f} m/s2
                spd limit: {:04.2f} m/s       spd limit @ control: {:04.2f} m/s
                green interval:            [{:04.2f}, {:04.2f}] sec
                '''.format(self.d0, self.v0, self.amin, self.amax, self.vmax, self.vcont, self.gs, self.gt))

        if self.v0 > self.vmax:
            t = (self.vmax - self.v0) / self.amin
            d = (self.vmax ** 2 - self.v0 ** 2) / (2 * self.amin)
            print('Vehicle can decelerate to spd limit in {:04.2f} sec, {:04.2f} m'.format(t, d))
        else:
            t = (self.vmax - self.v0) / self.amax
            d = (self.vmax ** 2 - self.v0 ** 2) / (2 * self.amax)
            print('Vehicle can accelerate to spd limit in {:04.2f} sec, {:04.2f} m'.format(t, d))
        if self.vcont < self.vmax:
            t = (self.vcont - self.vmax) / self.amin
            d = (self.vcont ** 2 - self.vmax ** 2) / (2 * self.amin)
            print('then decelerates to control spd limit in {:04.2f} sec, {:04.2f} m'.format(t, d))
        elif self.vcont > self.vmax:
            t = (self.vcont - self.vmax) / self.amax
            d = (self.vcont ** 2 - self.vmax ** 2) / (2 * self.amax)
            print('then accelerates to control spd limit in {:04.2f} sec, {:04.2f} m'.format(t, d))

    def reset(self):
        self.stat = False

    def LVTOsol4sig(self):
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

    def d1(self, t, a1):
        return self.d0 - self.v0 * (t - self.t0) - a1 * (t - self.t0) ** 2 / 2

    def d2(self, t, t1, v2, a1):
        return self.d1(t1, a1) - v2 * (t - t1)

    def d3(self, t, t1, t2, v2, a1, a3):
        return self.d2(t2, t1, v2, a1) - v2 * (t - t2) - a3 * (t - t2) ** 2 / 2

    def d(self, t, t1, t2, v2, a1, a3):
        if t <= t1:
            return self.d1(t, a1)
        elif t <= t1 + t2:
            return self.d2(t, t1, v2, a1)
        else:
            return self.d3(t, t1, t2, v2, a1, a3)

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

    def blnkfig(self):
        hover = HoverTool(tooltips=[
            ("index", "$index"),
            ("(time,dist)", "($x sec, $y m)"),
        ])
        fig = figure(width=500, height=500, tools=[hover],
                     title="Mouse over the dots")
        fig.title.text = "Time-Space Diagram"
        fig.title.align = "center"
        fig.output_backend = "svg"
        return fig

    def plotrj(self, fig):
        fig.line(self.indepVar, self.depVar, line_width=3)
        return fig

    def exprtrj(self, fig):
        export_svgs(fig, filename="trj.svg")


if __name__ == "__main__":
    x = MahmoudLAVO(0, 500, 17, 40)
    x.buildtrj(13.1345, 4.47734, -1.11671, -1.16845)
    fig = x.plotrj(x.blnkfig())
    show(fig)
    export_svgs(fig, 'trj.svg')

    print()
