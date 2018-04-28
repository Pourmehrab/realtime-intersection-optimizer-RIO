####################################
# File name: vehicle.py            #
# Author: Mahmoud Pourmehrab       #
# Email: mpourmehrab@ufl.edu       #
# Last Modified: Apr/15/2018       #
####################################

import csv

import numpy as np


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
    """
    EPS = 0.01  # small number that lower than that is approximated by zero
    MAX_NUM_TRAJECTORY_POINTS = 300  # check if it's enough to preallocate the trajectory

    def __init__(self, det_id, det_type, det_time, speed, dist, des_speed, dest, length, amin, amax, indx, k):
        """
        Data Structure for an individual vehicle
        .. note::
            - The last trajectory point index less than the first means no trajectory has been computed yet
            - The last trajectory index is set to -1 and the first to 0 for initialization purpose
            - The shape of trajectory matrix is :math:`3 *n` where :math:`n` is the maximum number of trajectory points
                to be held. The first, second, and third rows correspond to time, distance, and speed profile,
                 respectively.
            -

        :param det_id:          the id assigned to this vehicle by radio
        :param det_type:        0: Conventional, 1: Connected and Automated Vehicle
        :param det_time:        detection time in seconds from reference time
        :param speed:           detection speed in m/s
        :param dist:            detection distance to stop bar in meter
        :param des_speed:       desired speed in m/s
        :param dest:            destination 0: right turn, 1: through, 2: left
        :param length:          length of vehicle in meter
        :param amin:            desirable deceleration rate in m/s2
        :param amax:            desired acceleration rate in m/s2
        :param indx:            the original row index in the input csv file
        :param k:               number of coefficients to represent the trajectory if vehicle is connected
        """
        self.ID = det_id
        self.veh_type = det_type
        self.init_time = det_time  # needed to compute the travel time
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
        Writes trajectory points in the csv file if their time stamp is before the `time_threshold` and then removes
        them by updating the first trj point.

        :param sc: scenario number being simulated
        :param lane: lane number that is zero-based  (it records it one-based)
        :param time_threshold: any trajectory point before this is considered expired (normally its simulation time)
        :param file: initialized in `Traffic.__init__()` method, if ``None``, this does not record in csv.
        """
        trj_indx, max_trj_indx = self.first_trj_point_indx, self.last_trj_point_indx
        time, distance, speed = self.trajectory[:, trj_indx]

        if time < time_threshold:
            if file is None:  # don't have to write csv
                while time < time_threshold and trj_indx <= max_trj_indx:
                    trj_indx += 1
                    time = self.trajectory[0, trj_indx]

            else:  # get full info and write trj points to the csv file
                writer = csv.writer(file, delimiter=',')
                while time < time_threshold and trj_indx <= max_trj_indx:
                    writer.writerows([[sc, self.ID, self.veh_type, lane + 1, time, distance, speed]])
                    file.flush()
                    trj_indx += 1
                    time, distance, speed = self.trajectory[:, trj_indx]

            if trj_indx <= max_trj_indx:
                self.set_first_trj_point_indx(trj_indx)
            else:
                raise Exception("The vehicle should've been removed instead of getting updated for trj points.")

    def set_earliest_arrival(self, t_earliest):
        """
        Sets the earliest arrival time at the stop bar
        Called under Traffic.update_vehicles_info() method
        """
        self.earliest_arrival = t_earliest  # this is the absolute earliest time

    def set_scheduled_arrival(self, t_scheduled, d_scheduled, s_scheduled):
        """
        Note when a new vehicle is scheduled, it has two trajectory points
            One for the current state
            One for the final state

        :param t_scheduled: scheduled departure time (s)
        :param d_scheduled: scheduled departure distance (m)
        :param s_scheduled: scheduled departure speed (m/s)
        :return:
        """
        self.scheduled_arrival = t_scheduled
        self.last_trj_point_indx = self.first_trj_point_indx + 1
        self.trajectory[:, self.last_trj_point_indx] = [t_scheduled, d_scheduled, s_scheduled]

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
        """
        if code == 1:
            return 'CAV'
        elif code == 0:
            return 'CNV'
        else:
            raise Exception('The numeric code of vehicle type is not known.')

    def print_trj_points(self, lane, veh_indx, source):
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
            ': det@({:>4.1f} s, {:>4.1f} m, {:>4.1f} m/s), sch@({:>4.1f}, {:>4.1f}, {:>4.1f}), {:>3d} points, called by '.format(
                self.trajectory[0, first_trj_indx], self.trajectory[1, first_trj_indx],
                self.trajectory[2, first_trj_indx],
                self.trajectory[0, last_trj_indx], self.trajectory[1, last_trj_indx], self.trajectory[2, last_trj_indx],
                last_trj_indx - first_trj_indx + 1
            ) + source)

    def test_trj_redo_needed(self, min_dist=50):
        """
        Checks if the trajectory model should be run (returns True) or not (False). Cases:
            1) if last trj point is not assigned yet, do the trajectory.
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
