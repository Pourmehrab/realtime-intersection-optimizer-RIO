####################################
# File name: lane.py               #
# Author: Mahmoud Pourmehrab       #
# Email: mpourmehrab@ufl.edu       #
# Last Modified: Apr/07/2018       #
####################################

import numpy as np


class Lanes:

    def __init__(self, num_lanes):
        '''
        Data Structure for keeping vehicles in order in the lanes in the form of a dictionary of arrays

        Goals:
            1) Keeps vehicles in order
            2) Keep track of index of last vehicle in each lane (useful for applications in Signal())
            3) Remove served vehicles
            4) Check if all lanes are empty

        :param num_lanes: number of lanes
        '''

        self.vehlist = {l: [] for l in range(num_lanes)}
        self.last_vehicle_indx = np.zeros(num_lanes, dtype=np.int) - 1

    def increment_last_veh_indx(self, lane):
        self.last_vehicle_indx[lane] += 1

    def decrement_last_veh_indx(self, lane, n):
        self.last_vehicle_indx[lane] -= n

    def purge_served_vehs(self, lane, indx):
        """
        Deletes vehicles from 0 to ``indx`` where ``indx`` is the pointer to the last served
        .. note:: deletion also includes vehicle at ``indx``

        :param lane: the lane number
        :type lane: int
        :param indx: from vehicle 0 to ``indx`` are intended to be removed by this method
        """
        del self.vehlist[lane][0:indx + 1]
        self.decrement_last_veh_indx(lane, indx + 1)

    def all_served(self, num_lanes):
        """
        :return: True if all lanes are empty, False otherwise
        """
        indx = 0
        while indx < num_lanes:
            if not self.vehlist[indx]:
                # list is empty
                indx += 1
            else:
                # found a lane that has un-served vehicles in it
                return False
        # all lanes are empty
        return True
