####################################
# File name: intersection.py              #
# Author: Mahmoud Pourmehrab       #
# Email: mpourmehrab@ufl.edu       #
# Last Modified: Apr/22/2018       #
####################################

'''
    Intersection: gets parameters that are needed to specify the configuration of problem
'''
import csv
import os

import numpy as np
import pandas as pd
from scipy import stats

from data.data import *
from src.trajectory import earliest_arrival_connected, earliest_arrival_conventional


class Intersection:
    """
    Goals:
        1) Keeps intersection parameters

    :Author:
        Mahmoud Pourmehrab <pourmehrab@gmail.com>
    :Date:
        April-2018
    """

    def __init__(self, int_name):
        """
        :param int_name: comes from what user input in the command line as the intersection name
        """
        self.name = int_name
        self._max_speed, self._min_headway, self._det_range, self._K, self._M, self._num_lanes = get_general_params(
            int_name)

    def get_poly_params(self):
        """
        :return: K and M
        """
        return self._K, self._M

    def get_num_lanes(self):
        return self._num_lanes

    def get_max_speed(self):
        return self._max_speed

    def get_min_headway(self):
        return self._min_headway

    def get_det_range(self):
        return self._det_range


class Lanes:
    """
    :Author:
        Mahmoud Pourmehrab <pourmehrab@gmail.com>
    :Date:
        April-2018

    """
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


class Vehicle:
    """
        Goals:
        1) Defines the vehicle object that keeps all necessary information
            1-1) Those which are coming from fusion
            1-2) Those which are defined to be decided in the program:
            `trajectory[time, distance, speed], earliest_arrival, scheduled_arrival, poly_coeffs, _do_trj`
        2) Update/record the trajectory points once they are expired
        3) Keep trajectory indexes updated
        4) Print useful info once a plan is scheduled
        5) Decides if a trajectory re-computation is needed
        6) Quality controls the assigned trajectory


    Note:
        1) Make sure the MAX_NUM_TRAJECTORY_POINTS to preallocate the trajectories is enough for given problem

    :Author:
        Mahmoud Pourmehrab <pourmehrab@gmail.com>
    :Date:
        April-2018
    """
    EPS = 0.01  # small number that lower than that is approximated by zero
    MAX_NUM_TRAJECTORY_POINTS = 300  # check if it's enough to preallocate the trajectory
    MIN_DIST_TO_STOP_BAR = 50  # lower than this (in m) do not update schedule todo where else used?

    def __init__(self, det_id, det_type, det_time, speed, dist, des_speed, dest, length, amin, amax, indx, k):
        """
        Data Structure for an individual vehicle
        .. note::
            - The last trajectory point index less than the first means no trajectory has been computed yet
            - The last trajectory index is set to -1 and the first to 0 for initialization purpose
            - The shape of trajectory matrix is :math:`3 *n` where :math:`n` is the maximum number of trajectory points
                to be held. The first, second, and third rows correspond to time, distance, and speed profile,
                 respectively.

        .. warning::
            - The vehicle detection time shall be recorded in ``init_time``. GA uses this field to compute travel time
                when computing *badness* if an individual.

        :param det_id:          the *id* assigned to this vehicle by radio
        :param det_type:        0: Conventional, 1: Connected and Automated Vehicle
        :param det_time:        detection time in :math:`s` from reference time
        :param speed:           detection speed in :math:`m/s`
        :param dist:            detection distance to stop bar in :math:`m`
        :param des_speed:       desired speed in :math:`m/s`
        :param dest:            destination 0: right turn, 1: through, 2: left
        :param length:          length of vehicle in :math:`m`
        :param amin:            desirable deceleration rate in :math:`m/s^2`
        :param amax:            desired acceleration rate in :math:`m/s^2`
        :param indx:            the original row index in the input csv file
        :param k:               number of coefficients to represent the trajectory if vehicle is connected
        """
        self.ID = det_id
        self.veh_type = det_type
        self.init_time = det_time
        self.curr_speed = speed
        self.distance = dist
        self.length = length
        self.max_decel_rate = amin
        self.max_accel_rate = amax
        self.destination = dest
        self.desired_speed = des_speed
        self.trajectory = np.zeros((3, self.MAX_NUM_TRAJECTORY_POINTS), dtype=np.float)  # the shape is important
        self.first_trj_point_indx = 0
        self.trajectory[:, self.first_trj_point_indx] = [det_time, dist, speed, ]
        self.last_trj_point_indx = -1
        self.csv_indx = indx  # is used to find vehicle in original csv file

        if det_type == 1:  # only CAVs trajectories are in the form of polynomials (the also have trajectory matrix)
            self.poly_coeffs = np.zeros(k)

        self.earliest_arrival, self.scheduled_arrival = 0.0, 0.0  # will be set with their set methods
        self.redo_trj_allowed = True  # default value

    def reset_trj_points(self, sc, lane, time_threshold, file):
        """
        Writes trajectory points in the csv file if their time stamp is before the ``time_threshold`` and then removes
        them by updating the first trajectory point.

        .. warning::
            Before calling this make sure at least the first trajectory point's time stamp is less than provided time
            threshold or such a call would be pointless.

        :param sc: scenario number being simulated
        :param lane: lane number that is zero-based  (it records it one-based)
        :param time_threshold: any trajectory point before this is considered expired (normally its simulation time)
        :param file: initialized in ``Traffic.__init__()`` method, if ``None``, this does not record points in csv.
        """
        trj_indx, max_trj_indx = self.first_trj_point_indx, self.last_trj_point_indx
        time, distance, speed = self.trajectory[:, trj_indx]

        if file is None:  # don't have to write csv
            while time < time_threshold and trj_indx <= max_trj_indx:
                trj_indx += 1
                time = self.trajectory[0, trj_indx]

        else:  # get full info and write trajectory points to the csv file
            writer = csv.writer(file, delimiter=',')
            while time < time_threshold and trj_indx <= max_trj_indx:
                writer.writerows([[sc, self.ID, self.veh_type, lane + 1, time, distance, speed]])
                file.flush()
                trj_indx += 1
                time, distance, speed = self.trajectory[:, trj_indx]

        if trj_indx <= max_trj_indx:
            self.set_first_trj_point_indx(trj_indx)
        else:
            raise Exception("The vehicle should've been removed instead of getting updated for trajectory points.")

    def set_earliest_arrival(self, t_earliest):
        """
        Sets the earliest arrival time at the stop bar
        Called under Traffic.update_vehicles_info() method
        """
        self.earliest_arrival = t_earliest  # this is the absolute earliest time

    def set_scheduled_arrival(self, t_scheduled, d_scheduled, s_scheduled, lane, veh_indx, print):
        """
        .. note::
            - When a new vehicle is scheduled, it has two trajectory points: one for the current state and the other for the final state.
            - If the vehicle is closer than ``MIN_DIST_TO_STOP_BAR``, avoid appending the schedule.

        :param t_scheduled: scheduled departure time (:math:`s`)
        :param d_scheduled: scheduled departure distance (:math:`m`)
        :param s_scheduled: scheduled departure speed (:math:`m/s`)
        :param lane: the lane this vehicle is in (*for printing purpose only*)
        :param veh_indx: The index of this vehicle in ots lane (*for printing purpose only*)
        :param print: ``True`` if we want to print schedule
        """
        first_trj_indx = self.first_trj_point_indx
        last_dist_to_stop_bar = self.trajectory[1, first_trj_indx]
        if last_dist_to_stop_bar > self.MIN_DIST_TO_STOP_BAR:
            self.scheduled_arrival = t_scheduled
            self.last_trj_point_indx = self.first_trj_point_indx + 1
            self.trajectory[:, self.last_trj_point_indx] = [t_scheduled, d_scheduled, s_scheduled]

            if print:
                self.print_trj_points(lane, veh_indx)

    def set_poly_coeffs(self, beta):
        """Sets the coefficients that define the polynomial that defines trajectory of a connected vehicle"""
        self.poly_coeffs = beta

    def set_first_trj_point_indx(self, indx):
        """Sets the fist column index that points to the trajectory start"""
        self.first_trj_point_indx = indx

    def set_last_trj_point_indx(self, indx):
        """Sets the last column index that points to the trajectory start"""
        self.last_trj_point_indx = indx

    @staticmethod
    def map_veh_type2str(code):
        """
        For the purpose of printing, this method translates the vehicle codes. Currently, it supports:
            - 0 : Conventional Vehicle (**CNV**)
            - 1 : Connected and Automated Vehicle (**CAV**)
        :param code: numeric code for the vehicle type
        :type code: int
        """
        if code == 1:
            return 'CAV'
        elif code == 0:
            return 'CNV'
        else:
            raise Exception('The numeric code of vehicle type is not known.')

    def print_trj_points(self, lane, veh_indx):
        """
        Print the first and last trajectory points information.
        This may be used either when a plan is scheduled or a trajectory is computed.

        :param lane: zero-based lane number
        :param veh_indx: index to find the vehicle in its lane array
        :param source: specifies which method has called the print
        """
        veh_type_str = self.map_veh_type2str(self.veh_type)
        first_trj_indx, last_trj_indx = self.first_trj_point_indx, self.last_trj_point_indx
        rank = '1st' if veh_indx == 0 else ('2nd' if veh_indx == 1 else str(veh_indx + 1) + 'th')
        lane_rank = rank + ' in L' + str(lane + 1).zfill(2)
        print(
            veh_type_str + ':' + str(self.ID) + ':' + lane_rank +
            ': ({:>4.1f} s, {:>4.1f} m, {:>4.1f} m/s) -> ({:>4.1f}, {:>4.1f}, {:>4.1f}), {:>3d} points.'.format(
                self.trajectory[0, first_trj_indx], self.trajectory[1, first_trj_indx],
                self.trajectory[2, first_trj_indx],
                self.trajectory[0, last_trj_indx], self.trajectory[1, last_trj_indx], self.trajectory[2, last_trj_indx],
                last_trj_indx - first_trj_indx + 1
            ))

    def test_trj_redo_needed(self, min_dist=50):
        """
        Checks if the trajectory model should be run (returns True) or not (False). Cases:
            1) if last trajectory point is not assigned yet, do the trajectory.
            2) if vehicle is closer than a certain distance, do NOT update the trajectory.

        :param min_dist: for lower than this (in meters), no trajectory optimization or car following will be applied
        :return:
        """

        if self.last_trj_point_indx < 0:
            return True  # dont have to set self._do_trj to True since it's already True

        curr_dist = self.trajectory[1, self.first_trj_point_indx]
        if curr_dist <= min_dist:
            self.redo_trj_allowed = False
        else:
            self.redo_trj_allowed = True

    def test_trj_points(self, simulation_time):
        """
        Verifies the trajectory points for following cases:
            1) Non-negative speed (threshold is set to -3 m/s)
            2) Non-negative distance (threshold is set to -3 m)
            3) Expired trajectory point is not removed

            todo add more tests
        :param simulation_time: the current simulation clock
        """

        trj_point_indx = self.first_trj_point_indx
        last_trj_point_indx = self.last_trj_point_indx
        if last_trj_point_indx - trj_point_indx > 0:  # if there are at least two points, check them
            trajectory = self.trajectory
            while trj_point_indx <= last_trj_point_indx:
                time, dist, speed = trajectory[:, trj_point_indx]
                if speed < -3:  # the polynomial may oscillate near zero so let it
                    raise Exception('Negative speed, veh: ' + str(self.ID))
                elif dist < -3:  # the polynomial may oscillate near zero so let it
                    raise Exception('Traj point after the stop bar, veh: ' + str(self.ID))
                elif time < simulation_time:
                    raise Exception('Expired trajectory point is not purged, veh: ' + str(self.ID))
                trj_point_indx += 1


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

    :Author:
        Mahmoud Pourmehrab <pourmehrab@gmail.com>
    :Date:
        April-2018
    """

    def __init__(self, inter_name, sc, log_at_vehicle_level, log_at_trj_point_level, print_detection):
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
        self._print_detection = print_detection

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

    def set_departure_time_for_csv(self, departure_time, indx, id):
        """
        Sets the departure time of an individual vehicle that is just served

        :param departure_time: departure time in seconds
        :param indx: row index in the sorted CSV file that has list of all vehicles
        :param id: ID of the vehicle being recorded
        """

        self._auxilary_departure_times[indx] = departure_time
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

        .. note::
            The fact that all vehicles are *added* does not equal to all *served*. Thus, we check if any vehicle is in any of the incoming lanes before halting the program.
        """
        if self._current_row_indx + 1 >= self.__all_vehicles.shape[0]:
            return True
        else:
            return False

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

            if self._print_detection:
                print(
                    '>>> ' + veh.map_veh_type2str(det_type) + ':' + det_id + ':' + 'L' + str(lane + 1).zfill(2) + ':' +
                    '({:>4.1f} s, {:>4.1f} m, {:>4.1f} m/s)'.format(det_time, dist, speed))

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

            # now that we have a trajectory, we can set the earliest departure time
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

    def serve_update_at_stop_bar(self, lanes, simulation_time, num_lanes, print_departure):
        """
        This looks for/removes the served vehicles

        :param lanes: includes all vehicles
        :param simulation_time: current simulation clock
        :param num_lanes: number of lanes
        """

        for lane in range(num_lanes):

            if bool(lanes.vehlist[lane]):  # not an empty lane

                last_veh_indx_to_remove = -1
                for veh_indx, veh in enumerate(lanes.vehlist[lane]):

                    det_time = veh.trajectory[0, veh.first_trj_point_indx]
                    dep_time = veh.trajectory[0, veh.last_trj_point_indx]
                    if dep_time < simulation_time:  # served! remove it.

                        last_veh_indx_to_remove += 1
                        if print_departure:
                            print('<<< ' + veh.map_veh_type2str(veh.veh_type) + ':' + veh.ID +
                                  '@({:>4.1f} s)'.format(dep_time))
                        if self._log_at_vehicle_level:
                            self.set_departure_time_for_csv(dep_time, veh.csv_indx, veh.ID)

                    elif det_time < simulation_time:  # RESET EXISTING VEHICLES TRAJECTORY
                        veh.reset_trj_points(self.scenario_num, lane, simulation_time, self.full_traj_csv_file)

                    else:  # det_time of all behind this vehicle is larger, so we can stop.
                        break

                if last_veh_indx_to_remove > -1:  # removes vehicles 0, 1, ..., veh_indx
                    lanes.purge_served_vehs(lane, last_veh_indx_to_remove)