####################################
# File name: vehicle.py            #
# Author: Mahmoud Pourmehrab       #
# Email: mpourmehrab@ufl.edu       #
# Last Modified: Apr/07/2018       #
####################################

import csv

import numpy as np
import pandas as pd


class Vehicle:
    MAX_NUM_TRAJECTORY_POINTS = 300

    def __init__(self, det_id, det_type, det_time, speed, dist, des_speed, dest, length, amin, amax, indx, k):
        '''
        Data Structure for an individual vehicle

        :param det_id:          the id assigned to this vehicle by radio
        :param det_type:        0: Conventional, 1: Connected and Automated Vehicle
        :param det_time:        detection time in seconds from reference time
        :param speed:           detection speed in m/s
        :param dist:            detection distance to stop bar in meter
        :param length:          length of vehicle in meter
        :param amin:            desirable deceleration rate in m/s2
        :param amax:            desired acceleration rate in m/s2
        :param dest:            destination 0: right turn, 1: through, 2: left
        :param des_speed:       desired speed in m/s
        :param trajectory       keeps trajectory of vehicle [time, distance, speed]
        :param last_trj_point_indx       last index points to last row in trajectory
        '''
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
        self.trajectory = np.zeros((3, self.MAX_NUM_TRAJECTORY_POINTS), dtype=float)  # the shape is important
        self.first_trj_point_indx = 0
        self.trajectory[:, 0] = [det_time, dist, speed, ]
        self.last_trj_point_indx = -1  # last index < first index means no traj have computed
        self.csv_indx = indx  # is used to find vehicle in original csv file

        if det_type == 1:  # only CAVs trajectories are in the form of polynomials
            self._poly_coeffs = np.zeros(k)

        self.earliest_arrival, self.scheduled_arrival = 0.0, 0.0  # will be set with their set methods
        self._do_trj = True

    def get_vehicle_type(self):
        return self.veh_type

    def reset_trj_points(self, sc, lane, time_threshold, file):
        '''
        writes all traj points before or equal to the time_threshold to a csv file
        resets traj and its index
        look for the header in get_traffic.py __init__ method
        '''
        trj_indx, max_trj_indx = self.first_trj_point_indx, self.last_trj_point_indx
        time = self.trajectory[0, trj_indx]

        # write trajectory points in the csv file and then remove them and then set the first trj point
        if time < time_threshold:
            writer = csv.writer(file, delimiter=',')
            while time < time_threshold and trj_indx <= max_trj_indx:
                time, distance, speed = self.trajectory[:, trj_indx]
                writer.writerows([[sc, self.ID, self.veh_type, lane, time, distance, speed]])
                file.flush()

                trj_indx += 1

            self.set_first_trj_point_indx(trj_indx - 1)

    def set_earliest_arrival(self, t_earliest):
        '''
        It gets the earliest arrival time at the stop bar for the last vehicle just added to this lane
        '''
        self.earliest_arrival = t_earliest  # this is the absolute earliest time

    def get_earliest_arrival(self):
        return self.earliest_arrival

    def set_scheduled_arrival(self, t_scheduled):
        self.scheduled_arrival = t_scheduled

    def get_scheduled_arrival(self):
        return self.scheduled_arrival

    def set_poly_coeffs(self, beta):
        self._poly_coeffs = beta

    def get_poly_coeffs(self):
        return self._poly_coeffs

    def set_first_trj_point_indx(self, indx):
        self.first_trj_point_indx = indx

    def get_first_trj_point_indx(self, indx):
        self.get_trj_point_indx = indx

    def set_last_trj_point_indx(self, indx):
        self.last_trj_point_indx = indx

    def get_last_trj_point_indx(self):
        return self.last_trj_point_indx

    def map_veh_type2str(self, code):
        if code == 1:
            return 'Automated'
        elif code == 0:
            return 'Conventional'
        else:
            raise Exception('The numeric code of vehicle type is not known.')

    def print_trj_points(self, lane, veh_indx):
        veh_type_str = self.map_veh_type2str(self.veh_type)
        first_trj_indx, last_trj_indx = self.first_trj_point_indx, self.last_trj_point_indx
        rank = '1st' if veh_indx == 0 else ('2nd' if veh_indx == 1 else str(veh_indx + 1) + 'th')
        lane_rank = rank + ' in lane ' + str(lane + 1)
        print(
            '* ' + veh_type_str + ' Vehicle ' + str(
                self.ID) + ' ' + lane_rank + ': DETECETED ({:.2f} sec, {:.2f} m, {:.2f} m/s), SCHEDULED ({:.2f} sec, {:.2f} m, {:.2f} m/s), {:d} points'.format(
                self.trajectory[0, first_trj_indx], self.trajectory[1, first_trj_indx],
                self.trajectory[2, first_trj_indx],
                self.trajectory[0, last_trj_indx], self.trajectory[1, last_trj_indx], self.trajectory[2, last_trj_indx],
                last_trj_indx - first_trj_indx + 1
            ))

    # def save_trj_to_csv(self, inter_name):
    #     t, d, s = self.trajectory[:, 0: self.last_trj_point_indx]
    #     df = pd.DataFrame({'time': t, 'distance': d, 'speed': s})
    #     df.to_excel('log/' + inter_name + '_trajectory_' + str(self.ID) + '.xlsx', index=False)

    def test_trj_redo_needed(self, min_dist=50):
        '''
            checks if the trajectory model should solve be run (returns True) or not (False)
            :min_curr_dist: for lower than this (in meters), no trajectory optimization or car following will be applied
            :return:
            '''

        # last_trj_point_indx = self.last_trj_point_indx
        # if last_trj_point_indx < 0:
        #     return True

        curr_dist = self.trajectory[1, 0]
        if curr_dist <= min_dist:
            self._do_trj = False
        else:
            self._do_trj = True

    def redo_trj(self):
        return self._do_trj

    def set_redo_trj_false(self):
        '''
        to bypass the reoptimization for now
        when fusion is added this will be merged with test_trj_redo_needed()
        :return:
        '''
        self._do_trj = False
