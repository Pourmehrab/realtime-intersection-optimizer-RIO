class MahmoudTrj:
    '''
    Abstract class for trajectories
    '''
    LAG = 1  # lag on signalization
    RES = 1  # second
    EPS = 0.01
    DIG = 2

    def __init__(self, t0, d0, v0, gs, gt, vmax, vcont, amin, amax):
        '''

        :param t0:          current time
        :param d0:          distance to control point
        :param v0:          initial speed in m/s
        :param gs:          start of green in s
        :param gt:          end of green in s
        :param vmax:        speed limit in m/s
        :param vcont:       speed limit at the control point in m/s
        :param amin:        deceleration rate in m/s2
        :param amax:        acceleration rate in m/s2
        '''
        self.t0, self.d0, self.v0 = t0, d0, v0
        self.amin, self.amax = amin, amax
        self.vmax, self.vcont = vmax, vcont
        self.gs, self.gt = gs + self.LAG, gt  # todo make sure lag is less than gt-gs
        self.stat = False

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
