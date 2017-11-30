
class MahmoudTrj:
    '''
    Abstract class
    '''
    lag = 1  # lag on signalization
    res = 1  # second
    eps = 0.01
    deci = 2

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
