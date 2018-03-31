####################################
# File name: get_traffic.py        #
# Author: Mahmoud Pourmehrab       #
# Email: mpourmehrab@ufl.edu       #
# Last Modified: Feb/16/2018       #
####################################

import os

import pandas as pd

from src.inter.vehicle import Vehicle
from src.trj.trj import Conventional, Connected


class Traffic:
    ''''
    Goal 1) Input Traffic
        This class reads traffic input from csf file (can be generated by VISSIM or any other)
        the csv file can contain scenarios (refer to the file for headers)
    Goal 2) Append a travel time column and save csv
    Goal 3)
    '''

    def __init__(self, inter_name):
        # get the path to the csv file and load up the traffic
        filepath = os.path.join('data/' + inter_name + '.csv')
        if os.path.exists(filepath):
            self.all_vehicles = pd.read_csv(filepath)
        else:
            raise Exception(filepath + ' was not found.')

        self.all_vehicles = self.all_vehicles.sort_values(by=['sc', 'arrival time'])
        self.all_vehicles = self.all_vehicles.reset_index(drop=True)

        # get the scenario number
        self.active_sc = self.all_vehicles['sc'].iloc[0]

        # curr_indx points to the last vehicle added (-1 if none has been yet)
        # note this is cumulative and won't reset after a scenario is done
        self.curr_indx = -1

        # the column to compute the travel time
        self.all_vehicles['departure time'] = 'NaN'
        # the column to store simulation time per scenario
        self.all_vehicles['elapsed time'] = 'NaN'

    def set_travel_time(self, travel_time, indx):
        self.all_vehicles['departure time'][indx] = travel_time

    def set_elapsed_sim_time(self, t):
        self.all_vehicles['elapsed time'][self.curr_indx] = t

    def save_csv(self, inter_name):
        filepath = os.path.join('log/' + inter_name + '_results.csv')
        self.all_vehicles.to_csv(filepath, index=False)

    def last_veh_in_last_sc_arrived(self):
        if self.curr_indx + 1 >= self.all_vehicles.shape[0]:
            return False
        else:
            return True

    def keep_scenario(self):
        indx = self.curr_indx + 1
        if self.all_vehicles['sc'][indx] == self.active_sc:
            return True
        else:
            return False

    def reset_scenario(self):
        indx = self.curr_indx + 1
        self.active_sc = self.all_vehicles['sc'].iloc[indx]

    def get_first_arrival(self):
        filtered_indx = self.all_vehicles['sc'] == self.active_sc
        return self.all_vehicles[filtered_indx]['arrival time'].iloc[0]

    def update_on_vehicles(self, lanes, t, max_speed):
        '''
        Adds vehicles from the csv file

        :param lanes: vehicles are added to this data structure (dictionary of doubly-linked lists)
        :param t: current simulation clock (sim_ctrl.get_clock() gives access)
        :return:
        '''
        indx = self.curr_indx + 1
        max_indx = self.all_vehicles.shape[0] - 1
        while indx <= max_indx and self.all_vehicles['sc'][indx] == self.active_sc and \
                self.all_vehicles['arrival time'][indx] <= t:

            # read the arrived vehicle's information
            lane = self.all_vehicles['lane'][indx] - 1  # csv file has lanes coded in one-based
            det_id = 'xyz'  # todo this changes in real-time mode
            det_type = self.all_vehicles['type'][indx]  # 0: CNV, 1: CAV
            det_time = self.all_vehicles['arrival time'][indx]
            speed = self.all_vehicles['curSpd'][indx]
            dist = self.all_vehicles['dist'][indx]
            des_speed = self.all_vehicles['desSpd'][indx]
            dest = self.all_vehicles['dest'][indx]
            length = self.all_vehicles['L'][indx]
            amin = self.all_vehicles['maxDec'][indx]  # max deceleration (negative value)
            amax = self.all_vehicles['maxAcc'][indx]  # max acceleration
            print('*** A veh of type {:d} detected @ {:2.2f} sec in lane {:d}'.format(det_type, det_time, lane))

            # create the vehicle and get the earliest departure time
            veh = Vehicle(det_id, det_type, det_time, speed, dist, des_speed, dest, length, amin, amax, indx)
            # add it to its lane
            lanes.vehlist[lane] += [veh]  # recall it is an array

            # compute trajectory to get the earliest departure time
            if det_type == 1:
                if len(lanes.vehlist[lane]) == 1:
                    # vehicles is a lead connected vehicle
                    # happens when a connected vehicle is the first in the lane
                    trj_planner = Connected(None, veh, vmax=max_speed)
                    trj_planner.estimate_earliest_arrival()  # pass 0 for lead vehicle
                else:
                    # vehicles is a follower connected vehicle
                    # happens when a connected vehicle is NOT the first in the lane
                    trj_planner = Connected(lanes.vehlist[lane][-2], veh, vmax=max_speed)
                    trj_planner.estimate_earliest_arrival()  # pass 1 for follower vehicle (when first argument is not None)
            else:
                if len(lanes.vehlist[lane]) == 1:
                    # vehicles is a lead conventional vehicle
                    # happens when a conventional vehicle is the first in the lane
                    trj_planner = Conventional(None, veh)
                    trj_planner.solve()  # pass 0 for lead vehicle
                else:
                    # vehicles is a lead conventional vehicle
                    # happens when a conventional vehicle is NOT the first in the lane
                    trj_planner = Conventional(lanes.vehlist[lane][-2], veh)
                    trj_planner.solve()  # pass 1 for follower vehicle (when first argument is not None)

            # now that we have a trj, we can set the earliest departure time
            veh.set_earliest_arrival()

            indx += 1

        # to keep track of how much of csv is processed
        self.curr_indx = indx - 1

    def update_at_stop_bar(self, lanes, t, num_lanes):

        for lane in range(num_lanes):

            if bool(lanes.vehlist[lane]):  # not an empty lane

                veh_indx, upper_veh_indx = 0, len(lanes.vehlist[lane])
                any_veh_served = False
                while veh_indx < upper_veh_indx:

                    trj_indx = lanes.vehlist[lane][veh_indx].last_trj_point_indx
                    dep_time = lanes.vehlist[lane][veh_indx].trajectory[trj_indx, 0]
                    if dep_time <= t:  # served
                        any_veh_served = True
                        self.set_travel_time(dep_time, lanes.vehlist[lane][veh_indx].csv_indx)
                        veh_indx += 1
                    else:
                        break

                if any_veh_served:  # remove them
                    lanes.purge_served_vehs(lane, veh_indx)
