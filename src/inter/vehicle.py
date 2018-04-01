####################################
# File name: vehicle.py            #
# Author: Mahmoud Pourmehrab       #
# Email: mpourmehrab@ufl.edu       #
# Last Modified: Mar/29/2018       #
####################################

import numpy as np


class Vehicle:
    def __init__(self, det_id, det_type, det_time, speed, dist, des_speed, dest, length, amin, amax, indx,
                 max_num_trajectory_points=300):
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
        self.trajectory = np.zeros((max_num_trajectory_points, 3), dtype=np.float)
        self.first_trj_point_indx = 0
        self.last_trj_point_indx = 0
        # time_diff = 3600 * (det_time[0] - ref_time[0]) + 60 * (det_time[1] - ref_time[1]) + (
        #         det_time[2] - ref_time[2])
        self.trajectory[0, :] = [det_time, dist, speed, ]
        self.csv_indx = indx  # is used to find vehicle in original csv file
        self.last_trj_point_indx = -1  # changes in set_trj()


    def set_earliest_arrival(self, t_earliest):
        '''
        It gets the earliest arrival time at the stop bar for the last vehicle just added to this lane
        '''
        self.earliest_arrival = t_earliest  # this is the absolute earliest time
