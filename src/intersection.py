####################################
# File name: intersection.py       #
# Author: Mahmoud Pourmehrab       #
# Email: mpourmehrab@ufl.edu       #
# Last Modified: Apr/22/2018       #
####################################

'''
    Intersection: gets parameters that are needed to specify the configuration of problem
'''
import os, csv

import numpy as np
import pandas as pd
from scipy import stats

from data.data import *


class Intersection:
    """
    Objectives:
        - Keeps intersection parameters

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
    Dictionary that the key is lane index and value is an arrays that keeps queue of vehicle in that lane.

    Objectives:
        - Keeps vehicles in order
        - Keep track of index of last vehicle in each lane (useful for applications in ``Signal()``)
        - Remove served vehicles, and update first unserved and last vehicle's indices accordingly
        - Check if all lanes are empty



    :Author:
        Mahmoud Pourmehrab <pourmehrab@gmail.com>
    :Date:
        April-2018

    """

    def __init__(self, num_lanes):
        '''
        Data Structure for keeping vehicles in order in the lanes in the form of a dictionary of arrays

        :param num_lanes: number of lanes
        '''

        self.vehlist = {l: [] for l in range(num_lanes)}

        self.reset_first_unsrvd_indx(num_lanes)
        self.last_vehicle_indx = np.zeros(num_lanes, dtype=np.int) - 1

    def decrement_first_unsrvd_indx(self, lane, num_served):
        """
        When vehicles get served, the first index to the unservd vehicle in a lane should change.

        :param n: number of served vehicle
        :param lane: the lane at which the vehicles are served
        """
        self.first_unsrvd_indx[lane] = max(0, self.first_unsrvd_indx[lane] - num_served)

    def increment_first_unsrvd_indx(self, lane):
        self.first_unsrvd_indx[lane] += 1

    def increment_last_veh_indx(self, lane):
        self.last_vehicle_indx[lane] += 1

    def reset_first_unsrvd_indx(self, num_lanes):
        self.first_unsrvd_indx = np.zeros(num_lanes, dtype=int)  # this is the most important variable in this module

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
        num_served = indx + 1
        self.decrement_first_unsrvd_indx(lane, num_served)
        self.decrement_last_veh_indx(lane, num_served)

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
    Objectives:
        - Defines the vehicle object that keeps all necessary information
            - Those which are coming from fusion
            - Those which are defined to be decided in the program: `trajectory[time, distance, speed], earliest_arrival, scheduled_departure, poly_coeffs, _do_trj`
        - Update/record the trajectory points once they are expired
        - Keep trajectory indexes updated
        - Print useful info once a plan is scheduled
        - Decides if a trajectory re-computation is needed
        - Quality controls the assigned trajectory

    .. note:: Make sure the MAX_NUM_TRAJECTORY_POINTS to preallocate the trajectories is enough for given problem

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
        Initializes the vehicle object.

        .. attention::
            - The last trajectory point index less than the first means no trajectory has been computed yet
            - The last trajectory index is set to -1 and the first to 0 for initialization purpose
            - The shape of trajectory matrix is :math:`3*n` where :math:`n` is the maximum number of trajectory points to be held. The first, second, and third rows correspond to time, distance, and speed profile, respectively.
            - The vehicle detection time shall be recorded in ``init_time``. GA depends on this field to compute travel time when computing *badness* if an individual.

        :param det_id:          the *ID* assigned to this vehicle by radio or a generator
        :type det_id:           str
        :param det_type:        0: CNV, 1: CAV
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
        :param self.trajectory: keeps the trajectory points as columns of a 3 by N array that N is ``MAX_NUM_TRAJECTORY_POINTS``
        :param self.first_trj_point_indx: points to the column of the ``trajectory`` array where the current point is stored. This gets updated as the time goes by.
        :param self.last_trj_point_indx: similarly, points to the column of the ``trajectory`` where last trajectory point is stored.
        :param self.poly_coeffs: only CAVs trajectories are represented in the form of polynomials as well as the trajectory matrix
        :type self.poly_coeffs: array
        :param self.earliest_arrival: the earliest arrival time at the stop bar
        :param self.scheduled_departure: the scheduled arrival time at the stop bar
        :param self.reschedule_departure: True if a vehicle is open to receive a new departure time, False if want to keep previous trajectory
        :type self.reschedule_departure: bool
        :param self.freshly_scheduled: True if a vehicle is just scheduled a **different** departure and ready for being assigned a trajectory
        :type self.freshly_scheduled: bool
        .. note::
            - By definition ``scheduled_departure`` is always greater than or equal to ``earliest_arrival``.
            - Prior to run, make sure teh specified size for trajectory array by ``MAX_NUM_TRAJECTORY_POINTS`` is enough to store all under the worst case.
            - A vehicle may be open to be rescheduled but gets the same departure time and therefore ``freshly_scheduled`` should hold ``False`` under that case.
            -
        """
        self.ID = det_id
        self.veh_type = det_type
        self.init_time = det_time
        # self.curr_speed = speed
        self.distance = dist
        self.length = length
        self.max_decel_rate = amin
        self.max_accel_rate = amax
        self.destination = dest
        self.desired_speed = des_speed
        self.csv_indx = indx  # is used to find vehicle in original csv file

        self.trajectory = np.zeros((3, self.MAX_NUM_TRAJECTORY_POINTS), dtype=np.float)  # the shape is important
        self.first_trj_point_indx = 0
        self.last_trj_point_indx = -1
        self.trajectory[:, self.first_trj_point_indx] = [det_time, dist, speed, ]

        if det_type == 1:
            self.poly_coeffs = np.zeros(k)

        self.earliest_departure, self.scheduled_departure = 0.0, 0.0
        self.reschedule_departure, self.freshly_scheduled = True, False

    def reset_trj_points(self, sc, lane, time_threshold, file):
        """
        Writes the trajectory points in the csv file if their time stamp is before the ``time_threshold`` and then removes them by updating the first trajectory point.

        .. warning::
            Before calling this make sure at least the first trajectory point's time stamp is less than provided time threshold or such a call would be pointless.

        :param sc: scenario number being simulated
        :param lane: lane number that is zero-based  (it records it one-based)
        :param time_threshold: any trajectory point before this is considered expired (normally its simulation time)
        :param file: initialized in :any:`Traffic.__init__()` method, if ``None``, this does not record points in csv.
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

    def set_earliest_departure(self, t_earliest):
        """
        Sets the earliest arrival time at the stop bar. Called under :any:`Traffic.update_vehicles_info()` method
        """
        self.earliest_departure = t_earliest  # this is the absolute earliest time

    def set_scheduled_departure(self, t_scheduled, d_scheduled, s_scheduled, lane, veh_indx, print_signal_detail):
        """
        It only schedules if the new departure time is different and vehicle is far enough for trajectory assignment
        .. note::
            - When a new vehicle is scheduled, it has two trajectory points: one for the current state and the other for the final state.
            - If the vehicle is closer than ``MIN_DIST_TO_STOP_BAR``, avoids appending the schedule.
            - Set the ``freshly_scheduled`` to True only if vehicle is getting a new schedule and trajectory planning might become relevant.

        :param t_scheduled: scheduled departure time (:math:`s`)
        :param d_scheduled: scheduled departure distance (:math:`m`)
        :param s_scheduled: scheduled departure speed (:math:`m/s`)
        :param lane: the lane this vehicle is in (*for printing purpose only*)
        :param veh_indx: The index of this vehicle in ots lane (*for printing purpose only*)
        :param print_signal_detail: ``True`` if we want to print schedule
        """
        first_trj_indx = self.first_trj_point_indx
        current_dist_to_stop_bar = self.trajectory[1, first_trj_indx]
        if current_dist_to_stop_bar >= self.MIN_DIST_TO_STOP_BAR and abs(
                t_scheduled - self.trajectory[0, self.last_trj_point_indx]) > self.EPS:
            self.freshly_scheduled = True

            self.scheduled_departure = t_scheduled
            self.set_last_trj_point_indx(self.first_trj_point_indx + 1)
            self.trajectory[:, self.last_trj_point_indx] = [t_scheduled, d_scheduled, s_scheduled]

            if print_signal_detail:
                self.print_trj_points(lane, veh_indx, '@')

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

    def print_trj_points(self, lane, veh_indx, identifier):
        """
        Print the first and last trajectory points information. This may be used either when a plan is scheduled or a trajectory is computed.

        :param lane: zero-based lane number
        :param veh_indx: index to find the vehicle in its lane array
        :param identifier: use ``*`` for optimized trajectory, and ``@`` for scheduled departure
        """
        veh_type_str = self.map_veh_type2str(self.veh_type)
        first_trj_indx, last_trj_indx = self.first_trj_point_indx, self.last_trj_point_indx
        rank = '1st' if veh_indx == 0 else ('2nd' if veh_indx == 1 else str(veh_indx + 1) + 'th')
        lane_rank = rank + ' in L' + str(lane + 1).zfill(2)
        print(
            '>' + identifier + '> ' + veh_type_str + ':' + str(self.ID) + ':' + lane_rank +
            ': ({:>4.1f} s, {:>4.1f} m, {:>4.1f} m/s) -> ({:>4.1f}, {:>4.1f}, {:>4.1f}), {:>3d} points.'.format(
                self.trajectory[0, first_trj_indx], self.trajectory[1, first_trj_indx],
                self.trajectory[2, first_trj_indx],
                self.trajectory[0, last_trj_indx], self.trajectory[1, last_trj_indx],
                self.trajectory[2, last_trj_indx],
                last_trj_indx - first_trj_indx + 1
            ))

    # def needs_traj(self):
    #     """
    #     Checks if the trajectory model should be run (returns ``True``) or not (``False``). Cases:
    #         - If vehicle is closer than a certain distance specified by ``MIN_DIST_TO_STOP_BAR``, no need to update the trajectory.
    #         -
    #     :return: Whether trajectory should be computed (True), or not (False)
    #     """
    #
    #     curr_distance_to_stop_bar = self.trajectory[1, self.first_trj_point_indx]
    #     if self.freshly_scheduled and curr_distance_to_stop_bar > self.MIN_DIST_TO_STOP_BAR:
    #         return True
    #     else:
    #         return False


class Traffic:
    """
    Objectives:
        - Adds new vehicles from the csv file to the ``lanes.vehlist`` structure
        - Appends travel time, ID, and elapsed time columns and save csv
        - Manages scenario indexing, resetting, and more
        - Computes volumes in lanes
        - removes/records served vehicles

    .. note::
        - The csv should be located under the ``data/`` directory with the valid name consistent to what inputted as an
            argument and what exists in the data.py file.
        - The scenario number should be appended to the name of intersection followed by an underscore.

    :Author:
        Mahmoud Pourmehrab <pourmehrab@gmail.com>
    :Date:
        April-2018
    """

    def __init__(self, inter_name, sc, log_at_vehicle_level, log_at_trj_point_level, print_commandline,
                 start_time_stamp):
        """
        Objectives:
            - Sets the logging behaviour for outputting requested CSV files and auxiliary output vectors
            - Imports the CSV file that includes the traffic and sorts it
            - Initializes the first scenario number to run
        """

        # get the path to the csv file and load up the traffic
        filepath = os.path.join(
            'data/' + inter_name + '/' + inter_name + '_' + str(sc) + '.csv')
        if os.path.exists(filepath):
            self.__all_vehicles = pd.read_csv(filepath)
        else:
            raise Exception(filepath + ' was not found.')

        self.__all_vehicles = self.__all_vehicles.sort_values(by=['arrival time'])
        self.__all_vehicles = self.__all_vehicles.reset_index(drop=True)

        # get the scenario number
        self.scenario_num = sc

        # _current_row_indx points to the row of last vehicle added (-1 if none has been yet)
        # note this is cumulative and won't reset after a scenario is done
        self._current_row_indx = -1

        self._log_at_vehicle_level = log_at_vehicle_level
        self._log_at_trj_point_level = log_at_trj_point_level
        self._print_commandline = print_commandline

        if log_at_vehicle_level:
            df_size = len(self.__all_vehicles)
            self._auxilary_departure_times = np.zeros(df_size, dtype=np.float)
            self._axilary_elapsed_time = np.zeros(df_size, dtype=np.float)
            self._auxilary_ID = ['' for i in range(df_size)]

        if log_at_trj_point_level:
            # open a file to store trajectory points
            filepath_trj = os.path.join('log/' + inter_name + '/' + start_time_stamp + '_' + str(
                self.scenario_num) + '_trj_point_level.csv')
            self.full_traj_csv_file = open(filepath_trj, 'w', newline='')
            writer = csv.writer(self.full_traj_csv_file, delimiter=',')
            writer.writerow(['sc', 'VehID', 'type', 'lane', 'time', 'distance', 'speed'])
            self.full_traj_csv_file.flush()
        else:
            self.full_traj_csv_file = None

    def set_departure_time_for_csv(self, departure_time, indx, id):
        """
        Sets the departure time of an individual vehicle that is just served.

        :param departure_time: departure time in seconds
        :param indx: row index in the sorted CSV file that has list of all vehicles
        :param id: ID of the vehicle being recorded
        """

        self._auxilary_departure_times[indx] = departure_time
        self._auxilary_ID[indx] = id

    def set_elapsed_sim_time(self, elapsed_t):
        """
        Sets the elapsed time for one simulation of scenario.

        :param elapsed_t: elapsed time in seconds
        """
        self._axilary_elapsed_time[self._current_row_indx] = elapsed_t

    def save_veh_level_csv(self, inter_name, start_time_stamp):
        """
        Set the recorded values and save the  CSV at vehicle level.
        """
        self.__all_vehicles['departure time'] = self._auxilary_departure_times
        self.__all_vehicles['ID'] = self._auxilary_ID
        # the column to store simulation time per scenario
        self.__all_vehicles['elapsed time'] = self._axilary_elapsed_time

        filepath = os.path.join(
            'log/' + inter_name + '/' + start_time_stamp + '_' + str(
                self.scenario_num) + '_trj_vehicle_level.csv')
        self.__all_vehicles.to_csv(filepath, index=False)

    def close_trj_csv(self):
        """Closes trajectory CSV file."""
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
        return np.nanmin(self.__all_vehicles['arrival time'].values)

    def update_vehicles_info(self, lanes, simulation_time, max_speed, min_headway, k):
        """
        Objectives
            - Appends arrived vehicles from the csv file to :any:`Lanes`
            - Assigns their earliest arrival time

        :param lanes: vehicles are added to this data structure
        :type lanes: Lanes
        :param simulation_time: current simulation clock in seconds measured from zero
        :param max_speed: maximum allowable speed at the intersection in :math:`m/s`
        :param min_headway: min headway in :math:`sec/veh`
        :param k: one more than the degree of polynomial to compute trajectory of connected vehicles. We need it here to preallocate the vector that keeps the polynomial coefficients for connected vehicles.
        """

        # SEE IF ANY NEW VEHICLES HAS ARRIVED
        indx = self._current_row_indx + 1
        max_indx = self.__all_vehicles.shape[0] - 1
        t_earliest = 0.0  # keep this since in the loop it gets updated (necessary here)
        while indx <= max_indx and self.__all_vehicles['arrival time'][indx] <= simulation_time:

            # read the arrived vehicle's information
            lane = int(self.__all_vehicles['lane'][indx]) - 1  # csv file has lanes coded in one-based
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

            if self._print_commandline:
                print(
                    r'\\\ ' + veh.map_veh_type2str(det_type) + ':' + det_id + ':' + 'L' + str(lane + 1).zfill(
                        2) + ':' +
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
            veh.set_earliest_departure(t_earliest)

            indx += 1

        # to keep track of how much of csv is processed
        self._current_row_indx = indx - 1

    @staticmethod
    def get_volumes(lanes, num_lanes, det_range):
        """
        Unit of volume in each lane is :math:`veh/sec/lane`. Uses the fundamental traffic flow equation :math:`F=D*S`.


        :param lanes: includes all vehicles
        :type lanes: Lanes
        :param num_lanes: number of lanes
        :param det_range: detection range is needed to compute space-mean-speed
        :return volumes: array of volume level per lanes
        """
        # initialize volumes vector
        volumes = np.zeros(num_lanes, dtype=float)
        for lane in range(num_lanes):
            num_of_vehs = len(lanes.vehlist[lane])
            if num_of_vehs > 0:
                curr_speed = np.array([veh.trajectory[2, veh.first_trj_point_indx]
                                       for veh in lanes.vehlist[lane]], dtype=np.float)
                indx = curr_speed > 0
                if any(indx):
                    s = stats.hmean(curr_speed[indx])
                    volumes[lane] = num_of_vehs / det_range * s
                else:
                    volumes[lane] = 0.0
            else:
                volumes[lane] = 0.0

        return volumes

    def serve_update_at_stop_bar(self, lanes, simulation_time, num_lanes, print_commandline):
        """
        This looks for/removes the served vehicles.

        :param lanes: includes all vehicles
        :type lanes: Lanes
        :param simulation_time: current simulation clock
        :param num_lanes: number of lanes
        :param print_commandline:
        """

        for lane in range(num_lanes):
            if bool(lanes.vehlist[lane]):  # not an empty lane

                last_veh_indx_to_remove = -1
                for veh_indx, veh in enumerate(lanes.vehlist[lane]):

                    det_time = veh.trajectory[0, veh.first_trj_point_indx]
                    dep_time = veh.trajectory[0, veh.last_trj_point_indx]
                    if dep_time < simulation_time:  # served! remove it.

                        last_veh_indx_to_remove += 1
                        if print_commandline:
                            print('/// ' + veh.map_veh_type2str(veh.veh_type) + ':' + veh.ID +
                                  '@({:>4.1f} s)'.format(dep_time))
                        if self._log_at_vehicle_level:
                            self.set_departure_time_for_csv(dep_time, veh.csv_indx, veh.ID)

                    elif det_time < simulation_time:  # RESET EXISTING VEHICLES TRAJECTORY
                        veh.reset_trj_points(self.scenario_num, lane, simulation_time, self.full_traj_csv_file)

                    else:  # det_time of all behind this vehicle is larger, so we can stop.
                        break

                if last_veh_indx_to_remove > -1:  # removes vehicles 0, 1, ..., veh_indx
                    lanes.purge_served_vehs(lane, last_veh_indx_to_remove)


def earliest_arrival_connected(det_time, speed, dist, amin, amax, max_speed, min_headway=0, t_earliest=0):
    """
    Uses the maximum of the followings to compute the earliest time vehicle can reach to the stop bar:
        - Accelerate/Decelerate to the maximum allowable speed and maintain the speed till departure
        - Distance is short, it accelerates/decelerated to the best speed and departs
        - Departs at the minimum headway with its lead vehicle (only for followers close enough to their lead)

    :param det_time:
    :param speed:
    :param dist:
    :param amin:
    :param amax:
    :param max_speed:
    :param min_headway:
    :param t_earliest: earliest time of lead vehicle that is only needed if the vehicle is a follower vehicle
    :return:

    :Author:
        Mahmoud Pourmehrab <pourmehrab@gmail.com>
    :Date:
        April-2018

    """
    a = amax if speed <= max_speed else amin
    dist_to_max_speed = (max_speed ** 2 - speed ** 2) / (2 * a)

    if dist_to_max_speed <= dist:
        return max(
            det_time + (max_speed - speed) / a + (dist - dist_to_max_speed) / max_speed  # min time to get to stop bar
            , t_earliest + min_headway)

    else:  # not enough time and distance to accelerate/decelerate to max speed
        v_dest = np.sqrt(speed ** 2 + 2 * a * dist)
        return max(
            det_time + (max_speed - v_dest) / a  # min time to get to stop bar
            , t_earliest + min_headway
        )


def earliest_arrival_conventional(det_time, speed, dist, min_headway=0, t_earliest=0):
    """
    Uses the maximum of the followings to compute the earliest time vehicle can reach to the stop bar:
        - Maintains the detected speed till departure
        - Departs at the minimum headway with the vehicle in front

    :param det_time:
    :param speed:
    :param dist:
    :param min_headway:
    :param t_earliest: earliest time of lead vehicle that is only needed if the vehicle is a follower vehicle
    :return:

    .. note::
        Enter ``min_headway`` and ``t_earliest`` as zeros (default values), if a vehicle is the first in its lane.
    :Author:
        Mahmoud Pourmehrab <pourmehrab@gmail.com>
    :Date:
        April-2018
    """
    return max(
        det_time + dist / speed
        , t_earliest + min_headway
    )
