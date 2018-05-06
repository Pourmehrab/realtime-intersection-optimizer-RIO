####################################
# File name: time_keeper.py        #
# Author: Mahmoud Pourmehrab       #
# Email: mpourmehrab@ufl.edu       #
# Last Modified: Apr/22/2018       #
####################################


class TimeKeeper:
    """
    Objectives:
        - Keeps the time
        - Moves the simulation clock forward

    :Author:
        Mahmoud Pourmehrab <pourmehrab@gmail.com>
    :Date:
        April-2018
    """

    def __init__(self, sim_start, resolution=2.0):
        """
        Clock keeps the simulation starting time in seconds.


        :param sim_start: start time of simulation to be initialized
        :param resolution: Simulation resolution: the time steps to move the simulation forward in seconds
        """
        self.clock = sim_start
        self._res = resolution



    def next_sim_step(self):
        """Move simulation clock forward"""
        self.clock += self._res


