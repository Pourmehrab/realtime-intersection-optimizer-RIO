####################################
# File name: lane.py               #
# Author: Mahmoud Pourmehrab       #
# Email: mpourmehrab@ufl.edu       #
# Last Modified: Apr/07/2018       #
####################################

class Lanes:

    def __init__(self, num_lanes):
        '''
        Data Structure for keeping vehicles in order in the lanes

        :param num_lanes: number of lanes
        '''
        # a dictionary of arrays
        self.vehlist = {l: [] for l in range(num_lanes)}

    def set_all_scheduled_arrival(self, scheduled_arrivals):

        num_lanes = len(self.vehlist)
        for lane in range(num_lanes):
            num_of_vehicles = len(self.vehlist[lane])

            num_vehicles_in_lane = len(self.vehlist[lane])
            for veh_indx in range(num_vehicles_in_lane):
                self.vehlist[lane][veh_indx].set_scheduled_arrival(scheduled_arrivals[lane][veh_indx])

    def purge_served_vehs(self, lane, indx):
        '''
        Deletes vehicles from 0 to indx where indx is the pointer to the last served
        note deletion includes indx as well
        '''

        del self.vehlist[lane][0:indx]  # todo check if removes indx or one before

    def all_served(self, num_lanes):
        '''
        :return: True if all lanes are empty, False otherwise
        '''

        indx = 0
        while indx < num_lanes:
            if not self.vehlist[indx]:
                # list is empty
                indx += 1
            else:
                # found a lane that has un-served vehicles in it
                return False
        # all lanes are clear
        return True
