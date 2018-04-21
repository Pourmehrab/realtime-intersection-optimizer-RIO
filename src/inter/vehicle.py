####################################
# File name: vehicle.py            #
# Author: Mahmoud Pourmehrab       #
# Email: mpourmehrab@ufl.edu       #
# Last Modified: Apr/15/2018       #
####################################

import csv

import numpy as np
import pandas as pd


class Vehicle:
    EPS = 0.01  # small number that lower than that is approximated by zero
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
        self.trajectory[:, 0] = [det_time, dist, speed, ]
        self.last_trj_point_indx = -1  # last index < first index means no traj have computed
        self.csv_indx = indx  # is used to find vehicle in original csv file

        if det_type == 1:  # only CAVs trajectories are in the form of polynomials
            self.poly_coeffs = np.zeros(k)

        self.earliest_arrival, self.scheduled_arrival = 0.0, 0.0  # will be set with their set methods
        self._do_trj = True

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
            if file is None:  # don't have to write csv
                while time < time_threshold and trj_indx <= max_trj_indx:
                    time = self.trajectory[0, trj_indx]
                    trj_indx += 1

            else:  # get full info and write trj points in the csv
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

    def set_scheduled_arrival(self, t_scheduled, d_scheduled, s_scheduled):
        self.scheduled_arrival = t_scheduled
        self.last_trj_point_indx = 1
        self.trajectory[:, self.last_trj_point_indx] = [t_scheduled, d_scheduled, s_scheduled]

    def set_poly_coeffs(self, beta):
        self.poly_coeffs = beta

    def set_first_trj_point_indx(self, indx):
        self.first_trj_point_indx = indx

    def set_last_trj_point_indx(self, indx):
        self.last_trj_point_indx = indx

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

    def test_trj_points(self, simulation_time):
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
                trj_point_indx += 1
