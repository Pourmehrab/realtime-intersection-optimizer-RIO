####################################
# File name: sim.py                #
# Author: Mahmoud Pourmehrab       #
# Email: mpourmehrab@ufl.edu       #
# Last Modified: Feb/16/2018       #
####################################


class Simulator:
    STEP = 5  # time steps to move the simulation forward in second

    def __init__(self, t):
        # time : sec
        self.clock = t

    def next_sim_step(self):
        # configure sim resolution by changing the increment unit
        # todo (Patrick) this changes when code becomes real-time
        self.clock += self.STEP

    def get_clock(self):
        return self.clock
