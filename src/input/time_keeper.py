####################################
# File name: time_keeper.py        #
# Author: Mahmoud Pourmehrab       #
# Email: mpourmehrab@ufl.edu       #
# Last Modified: Apr/22/2018       #
####################################


class TimeKeeper:
    """
    Goals:
        1) Keeps the time
        2) Move forward the time

    .. note::
        Set the **simulation resolution** in :math:`s` at the default value of ``TimeKeeper.__init__()``
    """

    def __init__(self, t, resolution=20.0):
        """
        Clock keeps the simulation starting time in seconds (it gets reset for every scenario)
        Simulation resolution: the time steps to move the simulation forward in seconds
        """
        self.clock = t
        self._res = resolution

    def next_sim_step(self):
        """Move simulation clock forward"""
        self.clock += self._res
