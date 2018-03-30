####################################
# File name: lane.py               #
# Author: Mahmoud Pourmehrab       #
# Email: mpourmehrab@ufl.edu       #
# Last Modified: Feb/17/2018       #
####################################


class Lanes:
    def __init__(self, num_lanes):
        '''
        Data Structure for keeping vehicles in order in the lanes

        :param num_lanes: number of lanes
        '''
        # todo:(Mahmoud) a lane can have speed limit, opt range, etc
        # a dictionary of arrays
        self.vehlist = {l: [] for l in range(num_lanes)}

    def purge_served_vehs(self, lane, indx):
        '''
        Deletes vehicles from 0 to indx where indx is the pointer to the last served
        '''

        del self.vehlist[lane][0:indx]  # todo check if removes indx or one before

    def all_served(self, num_lanes):
        indx = 0
        while indx < num_lanes:
            if not self.vehlist[indx]:
                # list is empty
                indx += 1
            else:
                return False
        return True
