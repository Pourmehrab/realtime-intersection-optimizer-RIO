'''
Code is written in Python 3
Install the list of packages in the Pipfile using PyEnv


By:     Mahmoud Pourmehrab
E-mail: mpourmehrab@ufl.edu
Date:        Nov 2017
Last update: Dec/08/2017
'''


class MahmoudTrj:
    '''
    Abstract class for trajectories
    '''
    LAG = 1  # lag on signalization
    RES = 1  # second
    EPS = 0.01
    DIG = 2

    def __init__(self, lead_veh, fol_veh, gs, gt, vmax, vcont):

        self.lead_veh = lead_veh
        self.fol_veh = fol_veh
        self.vmax, self.vcont = vmax, vcont
        if gt > gs + self.LAG:
            self.gs, self.gt = gs + self.LAG, gt
        else:
            raise Exception('Signal lag exceeds the length of green')
        self.stat = False

    def insight(self):
        print(''' MahmoudTrj(.) received the following request:
                dist: {:04.2f} m             initial speed: {:04.2f} m/s
                deceleration: {:04.2f} m/s2   acceleration: {:04.2f} m/s2
                spd limit: {:04.2f} m/s       spd limit @ control: {:04.2f} m/s
                green interval:            [{:04.2f}, {:04.2f}] sec
                '''.format(self.fol_veh.dist, self.fol_veh.speed, self.fol_veh.amin, self.fol_veh.amax, self.vmax, self.vcont, self.gs, self.gt))

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

    def reset(self):
        self.stat = False
