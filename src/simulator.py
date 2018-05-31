####################################
# File name: simulator.py          #
# Author: Mahmoud Pourmehrab       #
# Email: mpourmehrab@ufl.edu       #
# Last Modified: May/30/2018       #
####################################

import os
import csv


class Simulator:
    """
    Objectives:
        - Keeps the time
        - Moves the simulation clock forward

    For time management we use seconds since the **Epoch** , or::

        >>> import time
        >>> time.time()

    .. figure:: images/time.png
            :align: center
            :width: 10cm
            :height: 8cm
            :alt: map to buried treasure

            Time management in python `source <https://wiki.python.org/moin/WorkingWithTime>`_.


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
        :Author:
            Mahmoud Pourmehrab <pourmehrab@gmail.com>
        :Date:
            April-2018
        """
        self.clock = sim_start
        self._res = resolution

    def next_sim_step(self):
        """Move simulation clock forward
        :Author:
            Mahmoud Pourmehrab <pourmehrab@gmail.com>
        :Date:
            April-2018
        """
        self.clock += self._res

    def get_running_clock(self):
        """Get the current clock
        :Author:
            Mahmoud Pourmehrab <pourmehrab@gmail.com>
        :Date:
            April-2018
        """
        return self.clock

    def record_sim_stats(self, sc, inter_name, start_time_stamp, elapsed_time):
        """
        Sets the elapsed time for one simulation of scenario.

        :param sc: scenario number
        :param inter_name: intersection name
        :param start_time_stamp: local time stamp
        :param elapsed_time: elapsed time in mili-seconds
        :Author:
            Mahmoud Pourmehrab <pourmehrab@gmail.com>
        :Date:
            April-2018
        """
        filepath = os.path.join('log/' + inter_name + '/' + start_time_stamp + '_sim_level.csv')
        if os.path.exists(filepath):
            with open(filepath, 'a', newline='') as csvfile:
                writer = csv.writer(csvfile, delimiter=',')
                writer.writerow([sc, elapsed_time, ])
        else:
            with open(filepath, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile, delimiter=',')
                writer.writerow(['sc', 'elapsed time'])
                writer.writerow([sc, elapsed_time, ])
