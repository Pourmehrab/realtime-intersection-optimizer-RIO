####################################
# File name: get_traffic.py        #
# Author: Mahmoud Pourmehrab       #
# Email: mpourmehrab@ufl.edu       #
# Last Modified: Apr/07/2018       #
####################################

import os
import csv

import numpy as np
import pandas as pd
from scipy import stats

from src.inter.vehicle import Vehicle
from src.trj.earliest import earliest_arrival_connected, earliest_arrival_conventional


class Traffic:
    ''''
    Goal 1) Input Traffic
        This class reads traffic input from csf file (can be generated by VISSIM or any other)
        the csv file can contain scenarios (refer to the file for headers)
    Goal 2) Append a travel time column and save csv
    Goal 3)
    '''

    def __init__(self, inter_name, num_lanes):
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

        # initialize volumes vector
        self.volumes = np.zeros(num_lanes, dtype=float)

        # open a file to store trajectories
        filepath_trj = os.path.join('data/' + inter_name + '_trjs.csv')
        self.full_traj_csv_file = open(filepath_trj, 'w', newline='')
        writer = csv.writer(self.full_traj_csv_file, delimiter=',')
        writer.writerow(['sc', 'VehID', 'lane', 'time', 'distance', 'speed'])
        self.full_traj_csv_file.flush()

    def set_travel_time(self, travel_time, indx):
        self.all_vehicles['departure time'][indx] = travel_time

    def set_elapsed_sim_time(self, t):
        self.all_vehicles['elapsed time'][self.curr_indx] = t

    def save_csv(self, inter_name):
        filepath = os.path.join('log/' + inter_name + '_results.csv')
        self.all_vehicles.to_csv(filepath, index=False)

    def close_trj_csv(self):
        self.full_traj_csv_file.close()

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

    def get_first_detection_time(self):
        filtered_indx = self.all_vehicles['sc'] == self.active_sc
        return np.nanmin(self.all_vehicles[filtered_indx]['arrival time'].values)

    def update_on_vehicles(self, lanes, num_lanes, simulation_time, max_speed, min_headway, k):
        '''
        Resets the first trajectory points
        Adds vehicles from the csv file

        :param lanes: vehicles are added to this data structure (dictionary of doubly-linked lists)
        :param simulation_time: current simulation clock (sim_ctrl.get_clock() gives access)
        :return:
        '''
        # RESET EXISTING VEHICLES TRAJECTORY
        for lane in range(num_lanes):
            if bool(lanes.vehlist[lane]):  # not an empty lane
                for veh in lanes.vehlist[lane]:
                    veh.reset_trj_points(self.active_sc, lane, simulation_time, self.full_traj_csv_file)

        # SEE IF ANY NEW VEHICLES HAS ARRIVED
        indx = self.curr_indx + 1
        t_earliest = 0  # keeps the earliest arrival at stop bar
        max_indx = self.all_vehicles.shape[0] - 1
        while indx <= max_indx and self.all_vehicles['sc'][indx] == self.active_sc and \
                self.all_vehicles['arrival time'][indx] <= simulation_time:

            # read the arrived vehicle's information
            lane = self.all_vehicles['lane'][indx] - 1  # csv file has lanes coded in one-based
            det_id = 'XYZ' + str(indx)  # todo this changes in real-time mode
            det_type = self.all_vehicles['type'][indx]  # 0: CNV, 1: CAV
            det_time = float(self.all_vehicles['arrival time'][indx])
            speed = float(self.all_vehicles['curSpd'][indx])
            dist = float(self.all_vehicles['dist'][indx])
            des_speed = float(self.all_vehicles['desSpd'][indx])
            dest = int(self.all_vehicles['dest'][indx])
            length = float(self.all_vehicles['L'][indx])
            amin = float(self.all_vehicles['maxDec'][indx])  # max deceleration (negative value)
            amax = float(self.all_vehicles['maxAcc'][indx])  # max acceleration
            print('*** A veh of type {:d} detected @ {:2.2f} sec in lane {:d}'.format(det_type, det_time, lane))

            # create the vehicle and get the earliest departure time
            veh = Vehicle(det_id, det_type, det_time, speed, dist, des_speed,
                          dest, length, amin, amax, k, indx)
            # add it to its lane
            lanes.vehlist[lane] += [veh]  # recall it is an array

            # compute trajectory to get the earliest departure time
            if det_type == 1:
                # For CAVs, the earliest travel time is computed by invoking the following method
                if len(lanes.vehlist[lane]) == 1:
                    # vehicles is a lead connected vehicle
                    # happens when a connected vehicle is the first in the lane
                    t_earliest = earliest_arrival_connected(det_time, speed, dist,
                                                            amin, amax, max_speed)
                else:
                    # vehicles is a follower connected vehicle
                    # happens when a connected vehicle is NOT the first in the lane
                    t_earliest = earliest_arrival_connected(det_time, speed, dist,
                                                            amin, amax, max_speed,
                                                            min_headway, t_earliest)
            else:
                if len(lanes.vehlist[lane]) == 1:
                    # vehicles is a lead conventional vehicle
                    # happens when a conventional vehicle is the first in the lane
                    t_earliest = earliest_arrival_conventional(det_time, speed, dist)
                else:
                    # vehicles is a lead conventional vehicle
                    # happens when a conventional vehicle is NOT the first in the lane
                    t_earliest = earliest_arrival_conventional(det_time, speed, dist,
                                                               min_headway, t_earliest)

            # now that we have a trj, we can set the earliest departure time
            veh.set_earliest_arrival(t_earliest)

            indx += 1

        # to keep track of how much of csv is processed
        self.curr_indx = indx - 1

    def get_volumes(self, lanes, num_lanes, det_range):
        '''
        unit of volume is vehicles /second
        Volume = Density x Space Mean Speed
        '''
        for lane in range(num_lanes):
            num_of_vehs = len(lanes.vehlist[lane])
            self.volumes[lane] = num_of_vehs / det_range * stats.hmean([lanes.vehlist[lane][veh_indx].curr_speed
                                                                        for veh_indx in
                                                                        range(len(lanes.vehlist[lane]))
                                                                        ]) if num_of_vehs > 0 else 0.0

        return self.volumes

    def update_at_stop_bar(self, lanes, simulation_time, num_lanes):
        '''
        This looks for/removes the served vehicles
        '''
        for lane in range(num_lanes):
            if bool(lanes.vehlist[lane]):  # not an empty lane

                veh_indx, upper_veh_indx = 0, len(lanes.vehlist[lane])
                any_veh_served = False
                for veh_indx, veh in enumerate(lanes.vehlist[lane]):
                    trj_indx = veh.last_trj_point_indx
                    dep_time = veh.trajectory[0, trj_indx]
                    if dep_time <= simulation_time:  # served
                        any_veh_served = True
                        self.set_travel_time(dep_time, veh.csv_indx)
                    else:
                        break

                if any_veh_served:  # removes vehicles 0, 1, ..., veh_indx
                    lanes.purge_served_vehs(lane, veh_indx)
