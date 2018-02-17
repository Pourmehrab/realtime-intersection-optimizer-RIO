'''
By:     Mahmoud Pourmehrab
E-mail: mpourmehrab@ufl.edu
Date:        November 2017
Last update: Feb/15/2018
'''


class Simulator:
    STEP = 0.1  # time steps to move the simulation forward

    def __init__(self, t):
        # time : sec
        self.clock = t

    def next_sim_step(self):
        # configure sim resolution by changing the increment unit
        self.clock += self.STEP

    def get_clock(self):
        return self.clock
