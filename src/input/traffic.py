####################################
# File name: traffic.py            #
# Author: Mahmoud Pourmehrab       #
# Email: mpourmehrab@ufl.edu       #
# Last Modified: Apr/22/2018       #
####################################

import csv
import os

import numpy as np
import pandas as pd
from scipy import stats

from src.inter.vehicle import Vehicle
from src.trj.earliest import earliest_arrival_connected, earliest_arrival_conventional


class Traffic:
    """
    Goals:
        1) Add new vehicles from the csv file to the lanes.vehlist structure
        2) Append travel time, ID, and elapsed time columns and save csv
        3) Manages scenario indexing, resetting, and more
        4) Compute volumes in lanes
        5) remove/record served vehicles

    .. note::
        - The csv should be located under the ``data/`` directory with the valid name consistent to what inputted as an
            argument and what exists in the data.py file.
        - The scenario number should be appended to the name of intersection followed by an underscore.
    """

    def __init__(self, inter_name, sc, log_at_vehicle_level, log_at_trj_point_level):
        """
        Goals:
            1) Set the logging behaviour for outputting requested CSV files and auxiliary output vectors
            2) Import the CSV file that includes the traffic and sorts it
            3) Initialize the first scenario number to run
        """

        # get the path to the csv file and load up the traffic
        filepath = os.path.join('data/' + inter_name + '_' + str(sc) + '.csv')
        if os.path.exists(filepath):
            self.__all_vehicles = pd.read_csv(filepath)
        else:
            raise Exception(filepath + ' was not found.')

        self.__all_vehicles = self.__all_vehicles.sort_values(by=['arrival time'])
        self.__all_vehicles = self.__all_vehicles.reset_index(drop=True)

        # get the scenario number
        self.scenario_num = self.__all_vehicles['sc'].iloc[0]

        # _current_row_indx points to the row of last vehicle added (-1 if none has been yet)
        # note this is cumulative and won't reset after a scenario is done
        self._current_row_indx = -1

        self._log_at_vehicle_level = log_at_vehicle_level
        self._log_at_trj_point_level = log_at_trj_point_level

        if log_at_vehicle_level:
            df_size = len(self.__all_vehicles)
            self._auxilary_departure_times = np.zeros(df_size, dtype=np.float)
            self._axilary_elapsed_time = np.zeros(df_size, dtype=np.float)
            self._auxilary_ID = ['' for i in range(df_size)]

        if log_at_trj_point_level:
            # open a file to store trajectory points
            filepath_trj = os.path.join('log/' + inter_name + '_' + str(sc) + '_trj_point_level.csv')
            self.full_traj_csv_file = open(filepath_trj, 'w', newline='')
            writer = csv.writer(self.full_traj_csv_file, delimiter=',')
            writer.writerow(['sc', 'VehID', 'type', 'lane', 'time', 'distance', 'speed'])
            self.full_traj_csv_file.flush()
        else:
            self.full_traj_csv_file = None

    def set_travel_time(self, travel_time, indx, id):
        """
        Sets the travel time of an individual vehicle that is just served
        :param travel_time: travel time in seconds (computed using base veh.init_time that it was detected first)
        :param indx: row index in the original CSV file that has list of all vehicles
        :param id: ID of the vehicle being recorded
        """

        self._auxilary_departure_times[indx] = travel_time
        self._auxilary_ID[indx] = id

    def set_elapsed_sim_time(self, t):
        """
        Sets the elapsed time for one simulation of scenario
        :param t: elapsed time in seconds
        """
        self._axilary_elapsed_time[self._current_row_indx] = t

    def save_csv(self, inter_name):
        """
        Set the recorded values and save the  CSV at vehicle level
        """
        self.__all_vehicles['departure time'] = self._auxilary_departure_times
        self.__all_vehicles['ID'] = self._auxilary_ID
        # the column to store simulation time per scenario
        self.__all_vehicles['elapsed time'] = self._axilary_elapsed_time

        filepath = os.path.join('log/' + inter_name + '_' + str(self.scenario_num) + '_vehicle_level.csv')
        self.__all_vehicles.to_csv(filepath, index=False)

    def close_trj_csv(self):
        """Closes trajectory CSV file"""
        self.full_traj_csv_file.close()

    def last_veh_arrived(self):
        """
        :return: True if all vehicles from the input csv have been added at some point, False otherwise.
        """
        if self._current_row_indx + 1 >= self.__all_vehicles.shape[0]:
            return False
        else:
            return True

    def get_first_detection_time(self):
        """
        :return: The time when the first vehicle in current scenario shows up.
        """
        filtered_indx = self.__all_vehicles['sc'] == self.scenario_num
        return np.nanmin(self.__all_vehicles[filtered_indx]['arrival time'].values)

    def update_vehicles_info(self, lanes, simulation_time, max_speed, min_headway, k):
        """
        Goals
            1) Add vehicles from the csv file to lanes.vehlist
            2) Assign their earliest arrival time

        :param lanes: vehicles are added to this data structure
        :type lanes: dictionary of array as of Lanes()
        :param simulation_time: current simulation clock in seconds measured from zero
        :param max_speed: maximum allowable speed at the intersection in m/s
        :param min_headway: min headway in sec/veh
        :param k: one more than the degree of polynomial to compute trajectory of connected vehicles. We need it here
        to preallocate the vector that keeps the polynomial coefficients for connected vehicles.
        """

        # SEE IF ANY NEW VEHICLES HAS ARRIVED
        indx = self._current_row_indx + 1
        max_indx = self.__all_vehicles.shape[0] - 1
        t_earliest = 0.0  # keep this since in the loop it gets updated (necessary here)
        while indx <= max_indx and self.__all_vehicles['sc'][indx] == self.scenario_num and \
                self.__all_vehicles['arrival time'][indx] <= simulation_time:

            # read the arrived vehicle's information
            lane = self.__all_vehicles['lane'][indx] - 1  # csv file has lanes coded in one-based
            det_id = 'xyz' + str(indx).zfill(3)  # pad zeros if necessary
            det_type = self.__all_vehicles['type'][indx]  # 0: CNV, 1: CAV
            det_time = float(self.__all_vehicles['arrival time'][indx])
            speed = float(self.__all_vehicles['curSpd'][indx])
            dist = float(self.__all_vehicles['dist'][indx])
            des_speed = float(self.__all_vehicles['desSpd'][indx])
            dest = int(self.__all_vehicles['dest'][indx])
            length = float(self.__all_vehicles['L'][indx])
            amin = float(self.__all_vehicles['maxDec'][indx])  # max deceleration (negative value)
            amax = float(self.__all_vehicles['maxAcc'][indx])  # max acceleration

            # create the vehicle and get the earliest departure time
            veh = Vehicle(det_id, det_type, det_time, speed, dist, des_speed,
                          dest, length, amin, amax, indx, k)
            # print('*** A veh of type ' + veh.map_veh_type2str(det_type) + ' detected @ {:2.2f} sec in lane {:d}'.format(
            #     det_time, lane + 1))

            # append it to its lane
            lanes.vehlist[lane] += [veh]  # recall it is an array
            lanes.increment_last_veh_indx(lane)

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
            elif det_type == 0:
                if len(lanes.vehlist[lane]) == 1:
                    # vehicles is a lead conventional vehicle
                    # happens when a conventional vehicle is the first in the lane
                    t_earliest = earliest_arrival_conventional(det_time, speed, dist)
                else:
                    # vehicles is a lead conventional vehicle
                    # happens when a conventional vehicle is NOT the first in the lane
                    t_earliest = earliest_arrival_conventional(det_time, speed, dist,
                                                               min_headway, t_earliest)
            else:
                raise Exception("The detected vehicle could not be classified.")

            # now that we have a trj, we can set the earliest departure time
            veh.set_earliest_arrival(t_earliest)

            indx += 1

        # to keep track of how much of csv is processed
        self._current_row_indx = indx - 1

    @staticmethod
    def get_volumes(lanes, num_lanes, det_range):
        """
        Unit of volume in each lane is veh/sec/lane
        Volume = Density x Space Mean Speed

        :param lanes: includes all vehicles
        :param num_lanes: number of lanes
        :param det_range: detection range is needed to compute space-mean-speed
        :return volumes: array of volume level per lanes
        """
        # initialize volumes vector
        volumes = np.zeros(num_lanes, dtype=float)
        for lane in range(num_lanes):
            num_of_vehs = len(lanes.vehlist[lane])
            volumes[lane] = num_of_vehs / det_range * stats.hmean([lanes.vehlist[lane][veh_indx].curr_speed
                                                                   for veh_indx in range(len(lanes.vehlist[lane]))
                                                                   ]) if num_of_vehs > 0 else 0.0
        return volumes

    def serve_update_at_stop_bar(self, lanes, simulation_time, num_lanes):
        """
        This looks for/removes the served vehicles

        :param lanes: includes all vehicles
        :param simulation_time: current simulation clock
        :param num_lanes: number of lanes
        """

        for lane in range(num_lanes):
            if bool(lanes.vehlist[lane]):  # not an empty lane
                veh_indx, any_veh_served = 0, False
                for veh in lanes.vehlist[lane]:
                    # RESET EXISTING VEHICLES TRAJECTORY
                    veh.reset_trj_points(self.scenario_num, lane, simulation_time, self.full_traj_csv_file)

                    trj_indx = veh.last_trj_point_indx
                    dep_time = veh.trajectory[0, trj_indx]
                    if dep_time <= simulation_time:  # served
                        any_veh_served = True
                        veh_indx += 1
                        if self._log_at_vehicle_level:
                            self.set_travel_time(dep_time, veh.csv_indx, veh.ID)
                    # else:
                    #     break

                if any_veh_served:  # removes vehicles 0, 1, ..., veh_indx
                    lanes.purge_served_vehs(lane, veh_indx)
