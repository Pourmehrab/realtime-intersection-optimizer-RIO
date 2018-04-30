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
        - Move forward the time

    :Author:
        Mahmoud Pourmehrab <pourmehrab@gmail.com>
    :Date:
        April-2018
    """

    def __init__(self, t, resolution=2.0):
        """
        Clock keeps the simulation starting time in seconds.


        :param t: start time of simulation to be initialized
        :param resolution: Simulation resolution: the time steps to move the simulation forward in seconds
        """
        self.clock = t
        self._res = resolution

    def next_sim_step(self):
        """Move simulation clock forward"""
        self.clock += self._res
