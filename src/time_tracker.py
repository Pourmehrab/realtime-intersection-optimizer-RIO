####################################
# File name: time_tracker.py       #
# Author: Ash Omidvar              #
# Email: aschkan@ufl.edu           #
# Last Modified: Oct.-2018 - Ash   #
####################################

import os
import csv
import time


# TODO @Pat: You can use this class to log any specific time-related outputs. As of now, I'm only logging total elapsed time.


class Timer:
    """
    Objectives:
        - Keeps the time
        - Moves the opt_clock forward in offline mode

    For time management we use seconds since the **Epoch**::

        >>> import time
        >>> time.time()

        # TODO @Ash: incorporate this discussion in the link below in documentation
            Time management in python `source <https://wiki.python.org/moin/WorkingWithTime>`_.
    :Author:
        Ash Omidvar <aschkan@ufl.com>
    :Date:
        Oct.-2018
    """

    def __init__(self, rio_start=time.time(), opt_time_step=1.00, traf_time_step=2):
        """
        **opt_clock** keeps track of time stamps where trajectory optimization happens.

        :param rio_start (sec): start time of running RIO
        :param opt_time_step (sec): the time steps to call trajectory and signal optimizers (default:0.5 Hz)
        :param traf_time_step (sec): the time step to update traffic (default: 10 Hz)
        """
        self.opt_clock = rio_start
        self.traf_clock = rio_start
        self.opt_res = opt_time_step
        self.traf_res = traf_time_step

    def next_opt_time_step(self):

        """
        This method updates opt_clock
        """
        self.opt_clock += self.opt_res

    def next_traf_time_step(self):
        """
        This method updates traf_clock
        """
        self.traf_clock += self.traf_res

    def log_time_stats(self, sc, inter_name, start_time_stamp, elapsed_time):
        """
        Records time-related system characteristics in log files.

        :param sc: scenario number
        :param inter_name: intersection name
        :param start_time_stamp: local time stamp
        :param elapsed_time: elapsed time in mili-seconds
        """
        filepath = os.path.join('log/' + inter_name + '/' + start_time_stamp + '_time_level.csv')
        if os.path.exists(filepath):
            with open(filepath, 'a', newline='') as csvfile:
                writer = csv.writer(csvfile, delimiter=',')
                writer.writerow([sc, elapsed_time, ])
        else:
            with open(filepath, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile, delimiter=',')
                writer.writerow(['sc', 'elapsed time (ms)', ])
                writer.writerow([sc, elapsed_time, ])
