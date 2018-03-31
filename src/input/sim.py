####################################
# File name: sim.py                #
# Author: Mahmoud Pourmehrab       #
# Email: mpourmehrab@ufl.edu       #
# Last Modified: Feb/16/2018       #
####################################


class Simulator:
    STEP = 20  # simulation resolution: the time steps to move the simulation forward in seconds

    def __init__(self, t):
        self.clock = t  # simulation starting clock in seconds (it gets reset for every scenario)

    def next_sim_step(self):
        self.clock += self.STEP

    def get_clock(self):
        return self.clock
