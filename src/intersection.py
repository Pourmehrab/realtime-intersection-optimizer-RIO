####################################
# File name: intersection.py       #
# Author: Mahmoud Pourmehrab       #
# Email: pourmehrab@gmail.com      #
# Last Modified: Jun/06/2018       #
####################################

import csv
import operator
import os

import numpy as np
import pandas as pd
from scipy import stats

from data.data import *
from src.trajectory import LeadConventional, LeadConnected, FollowerConventional, FollowerConnected


class Intersection:
    """
    Keeps intersection/simulation parameters

    :Author:
        Mahmoud Pourmehrab <pourmehrab@gmail.com>
    :Date:
        April-2018
    """

    def __init__(self, inter_name):
        """
        :param inter_name: comes from what user input in the command line as the intersection name
        """
        self._general_params = get_general_params(inter_name)


class Lanes:
    """
    Dictionary in which the key is lane index and value is an arrays that keeps a queue of vehicle in that lane.

    Objectives:
        - Keeps vehicles in order
        - Keeps track of index of last vehicle in each lane (useful for applications in :any:`Signal`)
        - Removes served vehicles, and update first unserved and last vehicle's indices accordingly
        - Checks if all lanes are empty

    :Author:
        Mahmoud Pourmehrab <pourmehrab@gmail.com>
    :Date:
        April-2018
    """

    def __init__(self, intersection):
        """
        A dictionary of lists for keeping vehicles in the physical order they are in the lanes

        .. note::
            - Python list has a method `insert(index)` can be used to add vehicles in the middle of the list
            - Use `del lanes.vehlist[lane][vehicle_index]` to remove a vehicle (look at :any`purge_served_vehs` for example)

        :param intersection: keeps parameters related to the intersection
        :type intersection: Intersection
        """
        num_lanes = intersection._general_params.get('num_lanes')

        self.vehlist = {l: [] for l in range(num_lanes)}
        self.reset_first_unsrvd_indx(num_lanes)
        self.last_vehicle_indx = np.zeros(num_lanes, dtype=np.int) - 1

    @staticmethod
    def refresh_earliest_departure_times(lanes, intersection):
        """"
        Computes the earliest departure time for all vehicles.

        :param lanes: includes all vehicles in all lanes
        :type lanes: Lanes
        :param intersection: All the intersection parameters are kept here
        :type intersection: Intersection

        :Author:
            Mahmoud Pourmehrab <pourmehrab@gmail.com>
        :Date:
            April-2018
        """
        # compute trajectory to get the earliest departure time
        num_lanes, min_headway, max_speed = map(intersection._general_params.get,
                                                ['num_lanes', 'min_headway', 'max_speed'])
        for lane in range(num_lanes):
            if bool(lanes.vehlist[lane]):  # not an empty lane
                for veh_indx, veh in enumerate(lanes.vehlist[lane]):
                    if veh.veh_type == 1:
                        # For CAVs, the earliest travel time is computed by invoking the following method
                        if len(lanes.vehlist.get(lane)) == 1:
                            # vehicles is a lead connected vehicle
                            # happens when a connected vehicle is the first in the lane
                            veh.earliest_arrival_connected(max_speed)
                        else:
                            # vehicles is a follower connected vehicle
                            # happens when a connected vehicle is NOT the first in the lane
                            veh.earliest_arrival_connected(max_speed, min_headway,
                                                           lanes.vehlist.get(lane)[-2].earliest_departure)
                    elif veh.veh_type == 0:
                        if len(lanes.vehlist.get(lane)) == 1:
                            # vehicles is a lead conventional vehicle
                            # happens when a conventional vehicle is the first in the lane
                            veh.earliest_arrival_conventional(max_speed)
                        else:
                            # vehicles is a lead conventional vehicle
                            # happens when a conventional vehicle is NOT the first in the lane
                            veh.earliest_arrival_conventional(max_speed, min_headway,
                                                              lanes.vehlist.get(lane)[-2].earliest_departure)
                    else:
                        raise Exception("The detected vehicle could not be classified.")

    def decrement_first_unsrvd_indx(self, lane, num_served):
        """
        When vehicles get served, the first index to the unserved vehicle in a lane should change.

        :param lane: the lane in which the vehicles are served
        :param num_served: number of served vehicle

        :Author:
            Mahmoud Pourmehrab <pourmehrab@gmail.com>
        :Date:
            April-2018
        """
        self.first_unsrvd_indx[lane] = max(0, self.first_unsrvd_indx[lane] - num_served)

    def increment_first_unsrvd_indx(self, lane):
        """
        Adds the unserved vehicle index keeper by one.

        :param lane: the lane index

        :Author:
            Mahmoud Pourmehrab <pourmehrab@gmail.com>
        :Date:
            April-2018
        """
        self.first_unsrvd_indx[lane] += 1

    def increment_last_veh_indx(self, lane):
        """
        Adds the last vehicle index keeper by one.

        :param lane: the lane index

        :Author:
            Mahmoud Pourmehrab <pourmehrab@gmail.com>
        :Date:
            April-2018
        """
        self.last_vehicle_indx[lane] += 1
        assert self.last_vehicle_indx[lane] + 1 == len(self.vehlist.get(lane)), "check this out"

    def decrement_last_veh_indx(self, lane, n):
        """
        Mostly used when vehicles are served. However, other cases due to fusion inaccuracy are possible.

        :param lane: the lane index
        :param n: the number of vehicles to be subtracted from index of this lane

        :Author:
            Mahmoud Pourmehrab <pourmehrab@gmail.com>
        :Date:
            April-2018
        """
        self.last_vehicle_indx[lane] -= n
        assert self.last_vehicle_indx[lane] + 1 == len(self.vehlist.get(lane)), "check this out"

    def set_temp_as_best_departure(self, start_indx_vec, end_indx_vec):
        """
        Happens when the GA finds a good individual

        :Author:
            Mahmoud Pourmehrab <pourmehrab@gmail.com>
        :Date:
            July-2018
        """
        num_lanes = len(self.vehlist)
        for lane in range(num_lanes):
            start_indx = start_indx_vec[lane]
            end_indx = min(end_indx_vec[lane], self.last_vehicle_indx[lane] + 1)
            for veh_indx, veh in enumerate(self.vehlist.get(lane)[start_indx:end_indx], start_indx):
                veh.set_best_temporary_departure(veh.temporary_departure)

    def reset_first_unsrvd_indx(self, num_lanes):
        """
        This is to reset the most important variable in this module which keeps track of the vehicle in which all ahead of it are served.

        :param num_lanes: the number of lanes

        :Author:
            Mahmoud Pourmehrab <pourmehrab@gmail.com>
        :Date:
            April-2018
        """
        self.first_unsrvd_indx = np.zeros(num_lanes, dtype=int)

    def purge_served_vehs(self, lane, indx):
        """
        Deletes vehicles from 0 to ``indx`` where ``indx`` is the pointer to the last served

        .. note:: deletion also includes vehicle at ``indx``

        :param lane: the lane index
        :type lane: int
        :param indx:  The index in which all vehicles with indices less than or equal to this get removed

        :Author:
            Mahmoud Pourmehrab <pourmehrab@gmail.com>
        :Date:
            April-2018
        """
        del self.vehlist.get(lane)[0:indx + 1]
        num_served = indx + 1
        self.decrement_first_unsrvd_indx(lane, num_served)
        self.decrement_last_veh_indx(lane, num_served)

    def all_served(self, num_lanes):
        """
        Checks if all lanes are empty of vehicles.

        :param num_lanes: number of lanes
        :return: ``True`` if all lanes are empty, ``False`` otherwise

        :Author:
            Mahmoud Pourmehrab <pourmehrab@gmail.com>
        :Date:
            April-2018
        """

        lane = 0
        while lane < num_lanes:
            if not self.vehlist[lane]:
                # lane is not empty
                lane += 1
            else:
                # found a lane that has un-served vehicles in it
                return False
        # all lanes are empty
        return True


class Vehicle:
    """
    Objectives:
        - Defines the vehicle object that keeps all necessary information
        - Updates/records the trajectory points once they are expired
        - Keeps trajectory indexes updated
        - Prints useful info once a plan is scheduled
        - Decides if a trajectory re-computation is needed
        - Quality controls the assigned trajectory

    :Author:
        Mahmoud Pourmehrab <pourmehrab@gmail.com>
    :Date:
        April-2018
    """

    def __init__(self, det_id, det_type, det_time, speed, dist, des_speed, dest, length, amin, amax, indx,
                 intersection):
        """
        Initializes the vehicle object.

        .. attention::
            - If the last trajectory point index is less than the first, no trajectory has been computed yet.
            - The last trajectory index is set to -1 and the first to 0 for initialization purposes.
            - The shape of trajectory matrix is :math:`3 \\times n`, where :math:`n` is the maximum number of trajectory points to be held. The first, second, and third rows correspond to time, distance, and speed profile, respectively.
            - The vehicle detection time shall be recorded in ``init_time``. GA depends on this field to compute travel time when computing :term:`badness` of an alternative.

        :param det_id:          the *ID* assigned to vehicle by radio or a generator
        :type det_id:           str
        :param det_type:        0: :term:`CNV`, 1: :term:`CAV`
        :param det_time:        detection time in :math:`s` from reference time
        :param speed:           detection speed in :math:`m/s`
        :param dist:            detection distance to stop bar in :math:`m`
        :param des_speed:       desired speed in :math:`m/s`
        :param dest:            destination 0: right turn; 1: through; 2: left
        :param length:          length of vehicle in :math:`m`
        :param amin:            desirable deceleration rate in :math:`m/s^2`
        :param amax:            desired acceleration rate in :math:`m/s^2`
        :param indx:            the original row index in the input CSV file
        :param intersection:    contains all the intersection parameters
        :type intersection:     Intersection

        :param self.trajectory: keeps the trajectory points as columns of a :math:`3 \\times n` array that :math:`n` is ``MAX_NUM_TRAJECTORY_POINTS``
        :param self.first_trj_point_indx: points to the column of the ``trajectory`` array where the current point is stored. This gets updated as the time goes by.
        :param self.last_trj_point_indx: similarly, points to the column of the ``trajectory`` where the last trajectory point is stored.
        :param self.poly: keeps the reference time and the coefficients to reproduce trajectory of an AV
        :type self.poly: dict
        :param self.earliest_departure: the earliest arrival time at the stop bar
        :param self.scheduled_departure: the scheduled arrival time at the stop bar
        :param self.reschedule_departure: True if a vehicle is open to receive a new departure time, False if want to keep previous trajectory
        :type self.reschedule_departure: bool
        :param self.freshly_scheduled: True if a vehicle is just scheduled a **different** departure and is ready to be assigned a trajectory
        :type self.freshly_scheduled: bool
        :param self._times_sent_to_traj_planner: number of times this vehicle is sent to trajectory planner

        .. note::
            - By definition ``scheduled_departure`` is always greater than or equal to ``earliest_arrival``.
            - Prior to run, make sure the specified size for trajectory array by ``max_num_traj_points`` is enough to store all the trajectory points under the worst case.
            - A vehicle may be open to being rescheduled but gets the same departure time; in that case, ``freshly_scheduled``  should hold False.

        :Author:
            Mahmoud Pourmehrab <pourmehrab@gmail.com>
        :Date:
            April-2018
        """
        self.ID = det_id
        self.veh_type = det_type
        self.init_time = det_time
        self.length = length
        self.max_decel_rate = amin
        self.max_accel_rate = amax
        self.destination = dest
        self.desired_speed = des_speed
        self.csv_indx = indx  # is used to find the vehicle in the original CSV file

        self.trajectory = np.zeros((3, intersection._general_params.get('max_num_traj_points')),
                                   dtype=np.float)  # the shape is important
        self.first_trj_point_indx, self.last_trj_point_indx = 0, -1
        self.trajectory[:, self.first_trj_point_indx] = [det_time, dist, speed, ]

        if det_type == 1:
            self.poly = {'ref time': 0.0, 'coeffs': np.zeros(intersection._general_params.get('k'))}

        self.earliest_departure = 0.0
        self.temporary_departure = 0.0  # a place holder for the place holder when doing GA
        self.best_temporary_departure = 0.0  # a place holder when doing GA
        self.scheduled_departure = 0.0
        self.reschedule_departure, self.freshly_scheduled = True, False
        self._times_sent_to_traj_planner = 0

    def earliest_arrival_connected(self, max_speed, min_headway=0.0, t_earliest=0.0):
        """
        Uses the latest departure time under the following cases to compute the earliest time the connected vehicle can reach the stop bar:
            - Accelerate/Decelerate to the maximum allowable speed and maintain the speed till departure
            - Distance is short, it accelerates/decelerated to the best speed and departs
            - Departs at the minimum headway with its lead vehicle (only for followers close enough to their lead)

        .. note::
            The main assumption is that the CAV *would* accelerate to the maximum speed and maintain the speed util departure.

        :param max_speed: maximum speed limit
        :param min_headway: minimum (saturation) headway at the stop bar
        :param t_earliest: earliest timemap_veh_type2str of lead vehicle that is only needed if the vehicle is a follower vehicle
        :return: The earliest departure time of the subject connected vehicle in seconds from the reference time

        :Author:
            Mahmoud Pourmehrab <pourmehrab@gmail.com>
        :Date:
            April-2018
        """
        det_time, dist, speed = self.get_arrival_schedule()
        a = self.max_accel_rate if speed <= max_speed else self.max_decel_rate
        dist_to_max_speed = (max_speed ** 2 - speed ** 2) / (2 * a)
        if dist_to_max_speed <= dist:
            t = max(
                det_time + (max_speed - speed) / a + (dist - dist_to_max_speed) / max_speed
                # min time to get to stop bar
                , t_earliest + min_headway)
        else:  # not enough time and distance to accelerate/decelerate to max speed
            v_dest = np.sqrt(speed ** 2 + 2 * a * dist)
            t = max(
                det_time + (max_speed - v_dest) / a  # min time to get to stop bar
                , t_earliest + min_headway)
        assert t > 0 and not np.isinf(t) and not np.isnan(t), "check the earliest departure time computation"
        self.earliest_departure = t

    def earliest_arrival_conventional(self, max_speed, min_headway=0.0, t_earliest=0.0):
        """
        Uses the latest departure time under the following cases to compute the earliest time the conventional vehicle can reach the stop bar:
            - Maintains the *estimated mean speed* till departure
            - Departs at the minimum headway with the vehicle in front

        :param min_headway: when 0, the vehicle is a lead and this constraint relaxes
        :param t_earliest: earliest time of lead vehicle that is only needed if the vehicle is a follower vehicle
        :return: The earliest departure time of the subject conventional vehicle in seconds from the reference time

        .. note::
            - Assumes the conventional vehicle would maintain its arrival speed if not restricted by other vehicles or the signal.
            - Enter ``min_headway`` and ``t_earliest`` as zeros (default values), if a vehicle is the first in its lane.
            - Make sure this is compatible with what implemented under :any:`FollowerConventional`
            - There are consequences if this method underestimates/overestimates the earliest departure time.

        :Author:
            Mahmoud Pourmehrab <pourmehrab@gmail.com>
        :Date:
            April-2018
        """
        det_time, dist, speed = self.get_arrival_schedule()
        mean_speed_est = speed  # min(self.desired_speed, max_speed)
        t = max(det_time + dist / mean_speed_est, t_earliest + min_headway)
        assert t > 0 and not np.isinf(t) and not np.isnan(t), "check the earliest departure time computation"
        self.earliest_departure = t

    def reset_trj_points(self, sc, lane, time_threshold, file):
        """
        Writes the trajectory points in the CSV file if the time stamp is before the ``time_threshold`` and then removes those points by updating the pointer to the first trajectory point.

        .. warning::
            Before calling this make sure at least the first trajectory point's time stamp is less than provided time threshold or such a call would be meaningless.

        :param sc: scenario number being simulated
        :param lane: lane number that is zero-based  (it records it one-based)
        :param time_threshold: any trajectory point before this is considered expired (normally its simulation time)
        :param file: The CSV file to be written. It is initialized in :any:`Traffic.__init__()` method, if ``None``, this does not record points in CSV.

        :Author:
            Mahmoud Pourmehrab <pourmehrab@gmail.com>
        :Date:
            April-2018
        """
        trj_indx, max_trj_indx = self.first_trj_point_indx, self.last_trj_point_indx
        time, distance, speed = self.get_arrival_schedule()

        if file is None:  # don't have to write CSV
            while time < time_threshold and trj_indx <= max_trj_indx:
                trj_indx += 1
                time = self.trajectory[0, trj_indx]

        else:  # get full info and write trajectory points to the CSV file
            writer = csv.writer(file, delimiter=',')
            while time < time_threshold and trj_indx <= max_trj_indx:
                writer.writerows([[sc, self.ID, self.veh_type, lane + 1, time, distance, speed]])
                file.flush()
                trj_indx += 1
                time, distance, speed = self.trajectory[:, trj_indx]

        assert trj_indx <= max_trj_indx, "The vehicle should've been removed instead of getting updated for trajectory points"
        self.set_first_trj_point_indx(trj_indx)

    def set_temporary_departure(self, t):
        """
        Sets the temporary departure time. Mostly used for heuristic algorithms like GA.

        :param t:

        :Author:
            Mahmoud Pourmehrab <pourmehrab@gmail.com>
        :Date:
            July-2018
        """
        self.temporary_departure = t

    def set_best_temporary_departure(self, t):
        """
        Sets the best temporary departure time. Mostly used for heuristic algorithms like GA.

        :param t:

        :Author:
            Mahmoud Pourmehrab <pourmehrab@gmail.com>
        :Date:
            July-2018
        """
        self.best_temporary_departure = t

    def set_scheduled_departure(self, t_scheduled, d_scheduled, s_scheduled, lane, veh_indx, intersection):
        """
        It only schedules if the new departure time is different and vehicle is far enough for trajectory assignment
        
        .. note::
            - When a new vehicle is scheduled, it has two trajectory points: one for the current state and the other for the final state.
            - If the vehicle is closer than ``min_dist_to_stop_bar``, avoids appending the schedule.
            - Set the ``freshly_scheduled`` to ``True`` only if vehicle is getting a new schedule and trajectory planning might become relevant.
            - Moves back the first trajectory point to make best use of limited size to store trajectory points

        :param t_scheduled: scheduled departure time (:math:`s`)
        :param d_scheduled: scheduled departure distance (:math:`m`)
        :param s_scheduled: scheduled departure speed (:math:`m/s`)
        :param lane: the lane this vehicle is in (*for printing purpose only*)
        :param veh_indx: The index of this vehicle in its lane (*for printing purpose only*)
        :param intersection:
        :type intersection: Intersection

        :Author:
            Mahmoud Pourmehrab <pourmehrab@gmail.com>
        :Date:
            April-2018
        """
        assert all(map(operator.not_, np.isinf(
            [t_scheduled, d_scheduled, s_scheduled]))), "infinity found in the schedule"

        min_dist_to_stop_bar = intersection._general_params.get("min_dist_to_stop_bar")
        small_positive_num = intersection._general_params.get("small_positive_num")

        det_time, det_dist, det_speed = self.get_arrival_schedule()
        if det_dist >= min_dist_to_stop_bar and abs(
                t_scheduled - self.trajectory[0, self.last_trj_point_indx]) > small_positive_num:
            self.freshly_scheduled = True

            self.set_first_trj_point_indx(0)
            self.trajectory[:, 0] = [det_time, det_dist, det_speed]
            self.set_last_trj_point_indx(1)
            self.trajectory[:, 1] = [t_scheduled, d_scheduled, s_scheduled]
            self.scheduled_departure = t_scheduled

            intersection._general_params.get("print_commandline") and self.print_trj_points(lane, veh_indx, "@")

    def set_poly(self, beta, t_ref):
        """
        Sets the coefficients that define the polynomial that defines trajectory of a connected vehicle

        :param beta: coefficient vector of the assigned polynomial (after solving the LP with CPLEX)
        :param t_ref: reference time in which the polynomial shall be evaluated from

        :Author:
            Mahmoud Pourmehrab <pourmehrab@gmail.com>
        :Date:
            April-2018
        """
        self.poly["ref time"] = t_ref
        self.poly["coeffs"] = beta

    def set_first_trj_point_indx(self, indx):
        """Sets the fist column index that points to the trajectory start

        :param indx: the index to the first trajectory point

        :Author:
            Mahmoud Pourmehrab <pourmehrab@gmail.com>
        :Date:
            April-2018
        """
        self.first_trj_point_indx = indx

    def set_last_trj_point_indx(self, indx):
        """Sets the last column index that points to the trajectory start

        :param indx: the index to the last trajectory point

        :Author:
            Mahmoud Pourmehrab <pourmehrab@gmail.com>
        :Date:
            April-2018
        """
        self.last_trj_point_indx = indx

    @staticmethod
    def map_veh_type2str(code):
        """
        For the purpose of printing, this method translates the vehicle codes. Currently, it supports:
            - 0 : Conventional Vehicle (:term:`CNV`)
            - 1 : Connected and Automated Vehicle (:term:`CAV`)

        :param code: numeric code for the vehicle type
        :type code: int
        :Author:
            Mahmoud Pourmehrab <pourmehrab@gmail.com>
        :Date:
            April-2018
        """
        if code == 1:
            return "CAV"
        elif code == 0:
            return "CNV"
        else:
            raise Exception('The input numeric code of the vehicle type is not known.')

    def increment_times_sent_to_traj_planner(self):
        """Increments the count on how many times sent to trajectory planner

        :Author:
            Mahmoud Pourmehrab <pourmehrab@gmail.com>
        :Date:
            April-2018
        """
        self._times_sent_to_traj_planner += 1

    def get_arrival_schedule(self):
        """
        :return: The triple :math:`(t,d,s)` corresponding to the arrival of subject vehicle

        :Author:
            Mahmoud Pourmehrab <pourmehrab@gmail.com>
        :Date:
            April-2018
        """
        return self.trajectory[:, self.first_trj_point_indx]

    def get_departure_schedule(self):
        """
        :return: The triple :math:`(t,d,s)` corresponding to the departure of subject vehicle

        :Author:
            Mahmoud Pourmehrab <pourmehrab@gmail.com>
        :Date:
            April-2018
        """
        return self.trajectory[:, self.last_trj_point_indx]

    def scale_traj_points(self, final_indx, final_foll_t, dt_total):
        """
        Scales the trajectory of the vehicle according to scaling the trajectory along the time axes.

        .. note::
            Only should get called if points are delayed compared to the scheduled departure time

        :return: ``self.trajectory`` will be scaled after this.

        :Author:
            Mahmoud Pourmehrab <pourmehrab@gmail.com>
        :Date:
            July-2018
        """
        trj = self.trajectory
        curr_foll_t, curr_foll_d, _ = self.get_arrival_schedule()
        t_f = self.scheduled_departure - dt_total

        scale_factor = (t_f - curr_foll_t) / (final_foll_t - curr_foll_t)
        for i in range(self.first_trj_point_indx, final_indx):
            trj[0, i] = curr_foll_t + scale_factor * (trj[0, i] - curr_foll_t)
            trj[2, i] /= scale_factor

    def print_trj_points(self, lane, veh_indx, identifier):
        """
        Print the first and last trajectory point information. This may be used either when a plan is scheduled or a trajectory is computed.

        :param lane: zero-based lane number
        :param veh_indx: index to find the vehicle in its lane array
        :param identifier: is
                            - ``*`` for optimized trajectory
                            -``@`` for scheduled departure.

        Example output to the commandline::

            >@> CNV:xyz004:2nd in L04: (40.0 s, 499.9 m,  7.8 m/s) -> (157.1,  0.0, 17.9),   2 points,  0 attempts

        which reads a conventional vehicle with ID of ``xyz004`` is the second vehicle in the fourth lane. It was detected at time 40 second, distance to stop bar of 499.9 meters, speed of 7.8 m/s and is scheduled to depart at time 157.1 second, at speed of 17.9 m/s before sent to the trajectory optimizer.

        :Author:
            Mahmoud Pourmehrab <pourmehrab@gmail.com>
        :Date:
            April-2018
        """
        veh_type_str = self.map_veh_type2str(self.veh_type)
        rank = '1st' if veh_indx == 0 else (
            '2nd' if veh_indx == 1 else ('3rd' if veh_indx == 2 else str(veh_indx + 1) + 'th'))
        lane_rank = rank + ' in L' + str(lane + 1).zfill(2)
        det_t, det_d, det_s = self.get_arrival_schedule()
        dep_t, dep_d, dep_s = self.get_departure_schedule()
        first_trj_indx, last_trj_indx = self.first_trj_point_indx, self.last_trj_point_indx
        print(
            '>' + identifier + '> ' + veh_type_str + ':' + str(self.ID) + ':' + lane_rank +
            ': ({:>4.1f} s, {:>4.1f} m, {:>4.1f} m/s) -> ({:>4.1f}, {:>4.1f}, {:>4.1f}), {:>3d} points, {:>2d} attempts'.format(
                det_t, det_d, det_s, dep_t, dep_d, dep_s, last_trj_indx - first_trj_indx + 1,
                self._times_sent_to_traj_planner))


class Traffic:
    """
    Objectives:
        - Adds new vehicles from the CSV file to the ``lanes.vehlist`` structure
        - Appends travel time and ID columns; saves CSV
        - Manages scenario indexing, resetting, and more
        - Computes volumes in lanes
        - Removes/records served vehicles

    .. note::
        - The CSV should be located under the ``/data/`` directory with the valid name consistent to what was inputted
            as an argument and what exists in the ``data.py`` file.
        - The scenario number should be appended to the name of intersection followed by an underscore.

    :Author:
        Mahmoud Pourmehrab <pourmehrab@gmail.com>
    :Date:
        April-2018
    """

    def __init__(self, intersection, sc, start_time_stamp):
        """
        Objectives:
            - Sets the logging behaviour for outputting requested CSV files and auxiliary output vectors
            - Imports the CSV file that includes the traffic and sorts it
            - Initializes the first scenario number to run


        :param intersection: containts intersection parameters
        :type intersection: Intersection
        :param sc: scenario number
        :param start_time_stamp: local time stamp to include in the CSV filename

        :Author:
            Mahmoud Pourmehrab <pourmehrab@gmail.com>
        :Date:
            April-2018

        """
        inter_name = intersection._general_params.get('inter_name')
        # get the path to the CSV file and load up the traffic
        filepath = os.path.join(
            'data/' + inter_name + '/' + inter_name + '_' + str(sc) + '.csv')
        assert os.path.exists(filepath), filepath + ' was not found.'
        self.__all_vehicles = pd.read_csv(filepath)

        self.__all_vehicles = self.__all_vehicles.sort_values(by=['arrival time'])
        self.__all_vehicles = self.__all_vehicles.reset_index(drop=True)

        # get the scenario number
        self.scenario_num = sc

        # _current_row_indx points to the row of last vehicle added (-1 if none has been yet)
        self._current_row_indx = -1

        self._log_csv = intersection._general_params.get('log_csv')
        self._print_commandline = intersection._general_params.get('print_commandline')

        if self._log_csv:
            df_size = len(self.__all_vehicles)
            self._auxilary_departure_times = np.zeros(df_size, dtype=np.float)
            self._auxilary_ID = ['' for i in range(df_size)]
            self._auxilary_num_sent_to_trj_planner = np.zeros(df_size, dtype=np.int8)

            # open a file to store trajectory points
            filepath_trj = os.path.join('log/' + inter_name + '/' + start_time_stamp + '_' + str(
                self.scenario_num) + '_trj_point_level.csv')
            self.full_traj_csv_file = open(filepath_trj, 'w', newline='')
            writer = csv.writer(self.full_traj_csv_file, delimiter=',')
            writer.writerow(['sc', 'VehID', 'type', 'lane', 'time', 'distance', 'speed'])
            self.full_traj_csv_file.flush()
        else:
            self.full_traj_csv_file = None

    def set_row_vehicle_level_csv(self, dep_time, veh):
        """
        Sets the departure time of an individual vehicle that is just served.

        :param dep_time: departure time in seconds
        :param veh: subject vehicle to be recorder
        :type veh: Vehicle

        :Author:
            Mahmoud Pourmehrab <pourmehrab@gmail.com>
        :Date:
            April-2018
        """
        indx = veh.csv_indx
        self._auxilary_departure_times[indx] = dep_time
        self._auxilary_ID[indx] = veh.ID
        self._auxilary_num_sent_to_trj_planner[indx] = veh._times_sent_to_traj_planner

    def save_veh_level_csv(self, inter_name, start_time_stamp):
        """
        Set the recorded values and save the  CSV at vehicle level.

        :param inter_name: intersection name
        :param start_time_stamp: local time stamp to include in the CSV filename

        :Author:
            Mahmoud Pourmehrab <pourmehrab@gmail.com>
        :Date:
            April-2018

        """
        self.__all_vehicles['departure time'] = self._auxilary_departure_times
        self.__all_vehicles['ID'] = self._auxilary_ID
        self.__all_vehicles['times_sent_to_trj_planner'] = self._auxilary_num_sent_to_trj_planner

        filepath = os.path.join(
            'log/' + inter_name + '/' + start_time_stamp + '_' + str(self.scenario_num) + '_trj_veh_level.csv')
        self.__all_vehicles.to_csv(filepath, index=False)

    def close_trj_csv(self):
        """Closes trajectory CSV file."""
        self.full_traj_csv_file.close()

    def last_veh_arrived(self):
        """
        :return: True if all vehicles from the input CSV have been added at some point, False otherwise.

        .. note::
            The fact that all vehicles are *added* does not equal to all *served*. Thus, we check if any vehicle is in
             any of the incoming lanes before halting the program.

        :Author:
            Mahmoud Pourmehrab <pourmehrab@gmail.com>
        :Date:
            April-2018
        """
        if self._current_row_indx + 1 >= self.__all_vehicles.shape[0]:
            return True
        else:
            return False

    def get_first_detection_time(self):
        """
        :return: The time when the first vehicle in current scenario shows up. Assumes the CSV file is not sorted in arrival time.

        :Author:
            Mahmoud Pourmehrab <pourmehrab@gmail.com>
        :Date:
            April-2018
        """
        return np.nanmin(self.__all_vehicles['arrival time'].values)

    def get_traffic_info(self, lanes, simulation_time, intersection):
        """
        Objectives
            - Appends arrived vehicles from the CSV file to :any:`Lanes`
            - Assigns their earliest arrival time

        :param lanes: vehicles are added to this data structure
        :type lanes: Lanes
        :param simulation_time: current simulation clock in seconds measured from zero
        :param intersection: intersection
        :type intersection: Intersection

        :Author:
            Mahmoud Pourmehrab <pourmehrab@gmail.com>
        :Date:
            April-2018
        """

        # SEE IF ANY NEW VEHICLES HAS ARRIVED
        indx = self._current_row_indx + 1
        max_indx = self.__all_vehicles.shape[0] - 1
        while indx <= max_indx and self.__all_vehicles['arrival time'][indx] <= simulation_time:
            # read the arrived vehicle's information
            lane = int(self.__all_vehicles['lane'][indx]) - 1  # CSV file has lanes coded in one-based
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
            veh = Vehicle(det_id, det_type, det_time, speed, dist, des_speed, dest, length, amin, amax, indx,
                          intersection)

            self._print_commandline and print(
                r'\\\ ' + veh.map_veh_type2str(det_type) + ':' + det_id + ':' + 'L' + str(lane ).zfill(
                    2) + ':' + '({:>4.1f} s, {:>4.1f} m, {:>4.1f} m/s)'.format(det_time, dist, speed))

            # append it to its lane
            lanes.vehlist[lane] += [veh]  # recall it is an array
            lanes.increment_last_veh_indx(lane)
            indx += 1

        # to keep track of how much of CSV is processed
        self._current_row_indx = indx - 1

    @staticmethod
    def get_volumes(lanes, intersection):
        """
        Unit of volume in each lane is :math:`veh/sec/lane`. Uses the fundamental traffic flow equation :math:`F=D \\times S`.


        :param lanes: includes all vehicles
        :type lanes: Lanes
        :param intersection:
        :type intersection:
        :return volumes: array of volume level per lanes

        :Author:
            Mahmoud Pourmehrab <pourmehrab@gmail.com>
        :Date:
            April-2018
        """
        # initialize volumes vector
        num_lanes = intersection._general_params.get('num_lanes')
        det_range = intersection._general_params.get('det_range')
        volumes = np.zeros(num_lanes, dtype=float)
        for lane in range(num_lanes):
            num_of_vehs = len(lanes.vehlist.get(lane))
            if num_of_vehs > 0:
                curr_speed = np.array([veh.trajectory[2, veh.first_trj_point_indx]
                                       for veh in lanes.vehlist.get(lane)], dtype=np.float)
                indx = curr_speed > 0
                if any(indx):
                    s = stats.hmean(curr_speed[indx])
                    volumes[lane] = num_of_vehs / det_range * s
                else:
                    volumes[lane] = 0.0
            else:
                volumes[lane] = 0.0

        return volumes

    def serve_update_at_stop_bar(self, lanes, simulation_time, intersection):
        """
        This looks for removing the served vehicles.

        :param lanes: includes all the vehicles in all lanes
        :type lanes: Lanes
        :param simulation_time: current simulation clock
        :param intersection:
        :type intersection: Intersection

        :Author:
            Mahmoud Pourmehrab <pourmehrab@gmail.com>
        :Date:
            April-2018
        """
        num_lanes = intersection._general_params.get('num_lanes')
        for lane in range(num_lanes):
            if bool(lanes.vehlist.get(lane)):  # not an empty lane
                last_veh_indx_to_remove = -1
                for veh_indx, veh in enumerate(lanes.vehlist.get(lane)):
                    det_time, _, _ = veh.get_arrival_schedule()
                    dep_time, _, _ = veh.get_departure_schedule()
                    assert dep_time > 0, "no departure is set"
                    if dep_time < simulation_time:  # record/remove departure
                        last_veh_indx_to_remove += 1
                        intersection._general_params.get('print_commandline') and print(
                            '/// ' + veh.map_veh_type2str(veh.veh_type) + ':' + veh.ID + '@({:>4.1f} s)'.format(
                                dep_time))
                        self._log_csv and self.set_row_vehicle_level_csv(dep_time, veh)
                    elif det_time < simulation_time:  # record/remove expired points
                        veh.reset_trj_points(self.scenario_num, lane, simulation_time, self.full_traj_csv_file)

                    else:  # det_time of all behind this vehicle is larger, so we can stop.
                        break

                last_veh_indx_to_remove > -1 and lanes.purge_served_vehs(lane, last_veh_indx_to_remove)


class TrajectoryPlanner:
    """
    Plans trajectories of all type. This makes calls to **trajectory** classes.

    :Author:
        Mahmoud Pourmehrab <pourmehrab@gmail.com>
    :Date:
        April-2018
    """

    def __init__(self, intersection):
        """Instantiates the **trajectory** classes"""

        self.lead_conventional_trj_estimator = LeadConventional(intersection)
        self.lead_connected_trj_optimizer = LeadConnected(intersection)
        self.follower_conventional_trj_estimator = FollowerConventional(intersection)
        self.follower_connected_trj_optimizer = FollowerConnected(intersection)

        self._max_speed = intersection._general_params.get('max_speed')

        # from optional.vis.vistrj import VisualizeSpaceTime  # todo remove after testing
        # self._visualizer = VisualizeSpaceTime(12)

    def plan_trajectory(self, lanes, veh, lane, veh_indx, intersection, tester, identifier):
        """
        :param lanes:
        :type lanes: Lanes
        :param veh:
        :type veh: Vehicle
        :param lane:
        :param veh_indx:
        :param intersection:
        :param identifier: Shows type of assigned trajectory
        :param tester: the test object
        :type tester: test.unit_tests.SimTest

        :Author:
            Mahmoud Pourmehrab <pourmehrab@gmail.com>
        :Date:
            April-2018
        """
        veh.increment_times_sent_to_traj_planner()
        veh_type, departure_time = veh.veh_type, veh.scheduled_departure
        if veh_indx > 0 and veh_type == 1:  # Follower CAV
            lead_veh = lanes.vehlist.get(lane)[veh_indx - 1]
            model = self.follower_connected_trj_optimizer.set_model(veh, lead_veh)
            self.follower_connected_trj_optimizer.solve(veh, lead_veh, model)
        elif veh_indx > 0 and veh_type == 0:  # Follower Conventional
            lead_veh = lanes.vehlist.get(lane)[veh_indx - 1]
            self.follower_conventional_trj_estimator.solve(veh, lead_veh)
        elif veh_indx == 0 and veh_type == 1:  # Lead CAV
            model = self.lead_connected_trj_optimizer.set_model(veh)
            self.lead_connected_trj_optimizer.solve(veh, None, model)
        elif veh_indx == 0 and veh_type == 0:  # Lead Conventional
            self.lead_conventional_trj_estimator.solve(veh)
        else:
            raise Exception('One of lead/follower conventional/connected should have occurred.')

        # self._visualizer.add_multi_trj_matplotlib(veh, lane)  # todo remove after testing
        # self._visualizer.export_matplot(0, 510, 20, 155)  # todo remove after testing

        if tester is not None:
            assert veh.trajectory[1, veh.last_trj_point_indx] < 1, "vehicle did not get to stop bar"
            tester.test_planned_departure(veh)
            tester.test_trj_points(veh)
            veh_indx > 0 and tester.check_for_collision(veh, lead_veh)

        intersection._general_params.get('print_commandline') and veh.print_trj_points(lane, veh_indx, identifier)
