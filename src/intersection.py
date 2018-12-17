####################################
# File name: intersection.py       #
# Author: Ash Omidvar              #
# Email: aschkan@ufl.edu           #
# Last Modified: Dec - 2018        #
####################################

import csv
import operator
import os

import numpy as np
import pandas as pd
from scipy import stats

from data.config import *
from src.trajectory import LeadConventional, LeadConnected, FollowerConventional, FollowerConnected

class Intersection:
    """
    Keeps intersection/simulation parameters

    :Author:
        Ash Omidvar <aschkan@ufl.edu>
    :Date:
        Sep - 2018
    """

    def __init__(self, inter_name):
        """
        :param inter_name: comes from what user input in the command line as the intersection name
        """
        self._inter_config_params = load_inter_params(inter_name)

class Lanes:
    """
    Create data structure in which the key is lane index and value is an arrays a vehicle objects in corresponding lane.

    Objectives:
        - To keep vehicles in order
        - To keep track of served/un-served vehicles (vehicles which have received optimized trajectory/vehicles which
        are waiting for optimized trajectory
        - Other low level methods: Check if all lanes are empty, maintain indices of vehicles in arrays, etc.

    :Author:
        Ash Omidvar <aschkan@ufl.edu>
    :Date:
        Oct-2018
    .. note:: some methods of this class are adopted from the simulation version of the system, developed
    by Mahmoud Pourmehrab <mpourmehrab@ufl.edu>.
    """

    def __init__(self, intersection):
        """
        A dictionary of arrays to keep the sequence of vehicles in each lane

        .. note::
            - Use `insert(index)` to add vehicles in the middle of the list.
            - Use `del lanes.vehlist[lane][vehicle_index]` to remove a vehicle.

        :param intersection: Object to keep parameters related to an intersection
        :type intersection: Intersection
        """
        num_lanes = intersection._inter_config_params.get('num_lanes')

        self.vehlist = {l: [] for l in range(num_lanes)}
        self.reset_first_unsrv_indx(num_lanes)
        self.last_veh_indx = np.zeros(num_lanes, dtype=np.int) - 1

    @staticmethod
    def reset_earliest_departure_times(lanes, intersection):
        """"
        To compute the earliest departure time for all vehicles.

        :param lanes: includes all vehicles in all lanes
        :type lanes: Lanes
        :param intersection: all the intersection parameters are kept here
        :type intersection: Intersection
        """
        # compute trajectory to get the earliest departure time
        num_lanes, min_CAV_headway, min_CNV_headway, max_speed = map(intersection._inter_config_params.get,
                                                                     ['num_lanes', "min_CAV_headway", "min_CNV_headway",
                                                                      'max_speed'])
        for lane in range(num_lanes):
            if bool(lanes.vehlist[lane]):
                for vehIndx, veh in enumerate(lanes.vehlist[lane]):
                    if veh.veh_type == 1:
                        # For CAVs, the earliest departure time is computed by actuating the following:
                        if len(lanes.vehlist.get(lane)) == 1:
                            # vehicles is a lead connected vehicle
                            veh.earliest_arr_cav(max_speed)
                        else:
                            # vehicles is a follower connected vehicle
                            lead_veh = lanes.vehlist.get(lane)[vehIndx - 1]
                            min_headway = min_CAV_headway if lead_veh.veh_type == 1 else min_CNV_headway
                            veh.earliest_arr_cav(max_speed, min_headway, lead_veh.earliest_departure)
                    elif veh.veh_type == 0:
                        if len(lanes.vehlist.get(lane)) == 1:
                            # vehicles is a lead conventional vehicle
                            veh.earliest_arr_cnv(max_speed)
                        else:
                            # vehicles is a lead conventional vehicle
                            lead_veh = lanes.vehlist.get(lane)[vehIndx - 1]
                            min_headway = min_CAV_headway if lead_veh.veh_type == 1 else min_CNV_headway
                            veh.earliest_arr_cnv(max_speed, min_headway, lead_veh.earliest_departure)
                    else:
                        raise Exception("The detected vehicle could not be classified.")

    def reset_first_unsrv_indx(self, num_lanes):
        """
        :param num_lanes: the number of lanes
        """
        self.first_unsrv_pos = np.zeros(num_lanes, dtype=int)

    def remove_srv_vehs(self, lane, indx):
        """
        Deletes vehicles from 0 to ``indx`` where ``indx`` is the pointer to the position of the last served vehicle

        .. note:: deletion also includes vehicle at ``indx``. ``indx`` simply refers to position of vehicles in lane.

        :param lane: the lane index
        :type lane: int
        :param indx:  The index in which all vehicles with indices less than or equal to this get removed
        :type indx: int
        """
        del self.vehlist.get(lane)[0:indx + 1]
        num_served = indx + 1
        self.dec_first_unsrvd_pos(lane, num_served)
        self.dec_last_veh_pos(lane, num_served)

    def all_served(self, num_lanes):
        """
        To check if all lanes are empty. Used in offline mode to wrap up the computation. Ignore in online mode

        :param num_lanes: number of lanes
        :return: ``True`` if all lanes are empty.
        """
        lane = 0
        while lane < num_lanes:
            if not self.vehlist[lane]:
                # Lane is not empty
                lane += 1
            else:
                # There exists a lane that has at least one un-served vehicle.
                return False
        # All lanes are empty.
        return True

    def dec_first_unsrvd_pos(self, lane, num_served):
        """
        To Set the index of the first un-served vehicle after vehicles get served (receive trajectory).

        :param lane: the lane in which the vehicles are served
        :param num_served: number of served vehicle
        """
        self.first_unsrv_pos[lane] = max(0, self.first_unsrv_pos[lane] - num_served)

    def dec_last_veh_pos(self, lane, n):
        """
        :param lane: the lane index
        :param n: the number of vehicles to be subtracted from index of this lane
        """
        self.last_veh_indx[lane] -= n
        assert self.last_veh_indx[lane] + 1 == len(self.vehlist.get(lane)), "Inspect vehicle positions!"

    def inc_first_unsrv_pos(self, lane):
        """
        :param lane: the lane index
        """
        self.first_unsrv_pos[lane] += 1

    def inc_last_veh_pos(self, lane):
        """
        :param lane: the lane index
        """
        self.last_veh_indx[lane] += 1
        assert self.last_veh_indx[lane] + 1 == len(self.vehlist.get(lane)), "Inspect vehicle positions!"



class Vehicle:
    """
    Objectives:
        - To define the vehicle object.
        - TO update/record the trajectory points and indices
        - To return necessary output info.
        - To decide if feedback-loop needs to be actuated

    :Author:
        Mahmoud Pourmehrab <mpourmehrab@ufl.edu>
        Ash Omidvar <aschkan@ufl.edu>
    :Date:
        April-2018
        Oct-2018
    """

    def __init__(self, det_id, det_type, det_time, speed, dist, des_speed, dest, length, a_min, a_max, indx,
                 intersection):
        """
        To initialize the vehicle object.

        .. attention::
            - The last trajectory index is set to -1 and the first to 0 for initialization purposes. Therefore, if the
            last trajectory point index is less than the first, no trajectory has been computed yet.
            - The shape of trajectory matrix is :math:`3 \\times n`, where :math:`n` is the maximum number of trajectory
             points to be kept. The first to third rows correspond to time, distance, and speed profile, respectively.
            - Record vehicle detection time in ``init_time``.

        :param det_id:          the *ID* assigned to vehicle by radio (in online mode) or a generator (in offline mode)
        :type det_id:           str
        :param det_type:        0: :term:`CNV`, 1: :term:`CAV`
        :param det_time:        detection time in :math:`s` from reference time point
        :param speed:           detection speed in :math:`m/s`
        :param dist:            detection distance to stop bar in :math:`m`
        :param des_speed:       desired speed in :math:`m/s`
        :param dest:            destination 0: right turn; 1: through; 2: left
        :param length:          length of vehicle in :math:`m`
        :param a_min:           desirable deceleration rate in :math:`m/s^2`
        :param a_max:           desired acceleration rate in :math:`m/s^2`
        :param indx:            the original row index in the input CSV file
        :param intersection:    contains all the intersection parameters
        :type intersection:     Intersection

        :param self.trajectory: keeps the trajectory points as columns of a :math:`3 \\times n` array that :math:`n` is
         ``max_num_traj_pts``
        :param self.first_trj_point_indx: points to the column of the ``trajectory`` array where the current point is
        stored. This gets updated as time steps get updated
        :param self.last_trj_point_indx: points to the column of the ``trajectory`` where the last trajectory
         point is stored
        :param self.poly: keeps the polynomial degree, reference time and the coefficients to reproduce trajectory
        :type self.poly: dict
        :param self.earliest_departure: the earliest departure time from the stop bar
        :param self.scheduled_departure: the scheduled departure time from the stop bar
        :param self.reschedule_departure: True if a vehicle is available to receive a new departure time,
        False if want to keep the previous trajectory
        :type self.reschedule_departure: bool
        :param self.got_trajectory: True if a vehicle is just scheduled a **different** departure and is ready to be
        assigned a new trajectory
        :type self.got_trajectory: bool
        :param self._call_reps_traj_planner: number of times a vehicle object trajectory is updated.

        .. note::
            - By definition ``scheduled_departure`` is always greater than or equal to ``earliest_arrival``.
            - It is important that user sets an ideal size of trajectory array by ``max_num_traj_points``.
            - A vehicle may be available to be rescheduled but gets the same departure time;
            in that case, ``got_trajectory``  should hold False.

        """
        self.ID = det_id
        self.veh_type = det_type
        self.init_time = det_time
        self.length = length
        self.max_decel_rate = a_min
        self.max_accel_rate = a_max
        self.destination = dest
        self.desired_speed = des_speed
        self.csv_indx = indx  # is used to find the vehicle in log file (offline mode)

        self.trajectory = np.zeros((3, intersection._inter_config_params.get('max_num_traj_points')),
                                   dtype=np.float)
        self.first_trj_point_indx, self.last_trj_point_indx = 0, -1
        self.trajectory[:, self.first_trj_point_indx] = [det_time, dist, speed, ]

        if det_type == 1:
            self.poly = {'ref. time': 0.0, 'coeffs': np.zeros(intersection._inter_config_params.get('k'))}

        self.earliest_departure = 0.0
        self.scheduled_departure = 0.0
        self.reschedule_departure, self.got_trajectory = True, False
        self._call_reps_traj_planner = 0

    def set_sched_dep(self, t_scheduled, d_scheduled, s_scheduled, lane, veh_indx, intersection):
        """
        It only schedules if the new departure time is different and vehicle is far enough for trajectory assignment

        .. note::
            - When a new vehicle is scheduled, it has two trajectory points: one for the current state and the other for the final state.
            - If the vehicle is closer than ``min_dist_to_stop_bar``, avoids appending the schedule.
            - Set the ``got_trajectory`` to ``True`` only if vehicle is getting a new schedule and trajectory planning might become relevant.
            - Moves back the first trajectory point to make best use of limited size to store trajectory points

        :param t_scheduled: scheduled departure time (:math:`s`)
        :param d_scheduled: scheduled departure distance (:math:`m`)
        :param s_scheduled: scheduled departure speed (:math:`m/s`)
        :param lane: the lane this vehicle is in (*for printing purpose only*)
        :param veh_indx: The index of this vehicle in its lane (*for printing purpose only*)
        :param intersection:
        :type intersection: Intersection
        """
        assert all(map(operator.not_, np.isinf(
            [t_scheduled, d_scheduled, s_scheduled]))), "infinity found in the schedule"

        min_dist_to_stop_bar = intersection._inter_config_params.get("min_dist_to_stop_bar")
        # small_positive_num = intersection._inter_config_params.get("small_positive_num")

        det_time, det_dist, det_speed = self.get_arr_sched()
        if det_dist >= min_dist_to_stop_bar:
            # self.got_trajectory = True

            self.set_first_trj_pt_indx(0)
            self.trajectory[:, 0] = [det_time, det_dist, det_speed]
            self.set_last_trj_pt_indx(1)
            self.trajectory[:, 1] = [t_scheduled, d_scheduled, s_scheduled]
            self.scheduled_departure = t_scheduled

            intersection._inter_config_params.get("print_commandline") and self.print_trj_points(lane, veh_indx, "@")

    def earliest_arr_cnv(self, max_speed, min_headway=0.0, t_earliest=0.0):
        """
        Uses the latest departure time under the following cases to compute the earliest time the conventional
        vehicle can reach the stop bar:
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
        """
        det_time, dist, speed = self.get_arr_sched()
        mean_speed_est = min(self.desired_speed, 0.85 * max_speed)
        t = max(det_time + dist / mean_speed_est, t_earliest + min_headway)
        assert t > 0 and not np.isinf(t) and not np.isnan(t), "check the earliest departure time computation"
        self.earliest_departure = t

    def earliest_arr_cav(self, max_speed, min_headway=0.0, t_earliest=0.0):
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

        """
        det_time, dist, speed = self.get_arr_sched()
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

    def reset_trj_pts(self, sc, lane, time_threshold, file):
        """
        Writes the trajectory points in the loog file if the time stamp is before the ``time_threshold``
        and then removes those points by updating the pointer to the first trajectory point.

        .. warning::
            Before calling this make sure at least the first trajectory point's time stamp is less than provided time
            threshold or such a call would be meaningless.

        :param sc: scenario number being simulated
        :param lane: lane number that is zero-based  (it records it one-based)
        :param time_threshold: any trajectory point before this is considered expired (normally its simulation time)
        :param file: The log file to be written. It is initialized in :any:`OffTraffic.__init__()` method, if ``None``,
        this does not record points in CSV.
        """
        trj_indx, max_trj_indx = self.first_trj_point_indx, self.last_trj_point_indx
        time, distance, speed = self.get_arr_sched()

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

        assert trj_indx <= max_trj_indx, "The vehicle should be removed, not  getting updated for trajectory points!"
        self.set_first_trj_pt_indx(trj_indx)

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

        which reads a conventional vehicle with ID of ``xyz004`` is the second vehicle in the fourth lane. It was
        detected at time 40 second, distance to stop bar of 499.9 meters, speed of 7.8 m/s and is scheduled to depart
        at time 157.1 second, at speed of 17.9 m/s before sent to the trajectory optimizer.
        """
        veh_type_str = self.map_veh_type2str(self.veh_type)
        rank = '1st' if veh_indx == 0 else (
            '2nd' if veh_indx == 1 else ('3rd' if veh_indx == 2 else str(veh_indx + 1) + 'th'))
        lane_rank = rank + ' in L' + str(lane + 1).zfill(2)
        det_t, det_d, det_s = self.get_arr_sched()
        dep_t, dep_d, dep_s = self.get_dep_sched()
        first_trj_indx, last_trj_indx = self.first_trj_point_indx, self.last_trj_point_indx
        print(
            '>' + identifier + '> ' + veh_type_str + ':' + str(self.ID) + ':' + lane_rank +
            ': ({:>4.1f} s, {:>4.1f} m, {:>4.1f} m/s) -> ({:>4.1f}, {:>4.1f}, {:>4.1f}), {:>3d} points, {:>2d} attempts'.format(
                det_t, det_d, det_s, dep_t, dep_d, dep_s, last_trj_indx - first_trj_indx + 1,
                self._call_reps_traj_planner))

    def set_poly(self, beta, t_ref):
        """
        Sets the coefficients that define the polynomial that defines trajectory of a connected vehicle

        :param beta: coefficient vector of the assigned polynomial (after solving the LP with CPLEX)
        :param t_ref: reference time in which the polynomial shall be evaluated from
        """
        self.poly["ref time"] = t_ref
        self.poly["coeffs"] = beta

    def set_first_trj_pt_indx(self, indx):
        """Sets the fist column index that points to the trajectory start
        :param indx: the index to the first trajectory point
        """
        self.first_trj_point_indx = indx

    def set_last_trj_pt_indx(self, indx):
        """Sets the last column index that points to the trajectory start
        :param indx: the index to the last trajectory point
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
        """
        if code == 1:
            return "CAV"
        elif code == 0:
            return "CNV"
        else:
            raise Exception('The input numeric code of the vehicle type is not known.')

    def inc_traj_planner_calls(self):
        """
        Increments the count on how many times sent to trajectory planner
        """
        self._call_reps_traj_planner += 1

    def get_arr_sched(self):
        """
        :return: The triple :math:`(t,d,s)` corresponding to the arrival of subject vehicle
        """
        return self.trajectory[:, self.first_trj_point_indx]

    def get_dep_sched(self):
        """
        :return: The triple :math:`(t,d,s)` corresponding to the departure of subject vehicle
        """
        return self.trajectory[:, self.last_trj_point_indx]

    def scale_traj_pts(self, final_indx, final_foll_t, dt_total):
        """
        Scales the trajectory of the vehicle according to scaling the trajectory along the time axes.

        .. note::
            Only should get called if points are delayed compared to the scheduled departure time

        :return: ``self.trajectory`` will be scaled after this.
        """
        trj = self.trajectory
        curr_foll_t, curr_foll_d, _ = self.get_arr_sched()
        t_f = self.scheduled_departure - dt_total

        scale_factor = (t_f - curr_foll_t) / (final_foll_t - curr_foll_t)
        for i in range(self.first_trj_point_indx, final_indx):
            trj[0, i] = curr_foll_t + scale_factor * (trj[0, i] - curr_foll_t)
            trj[2, i] /= scale_factor

#TODO @Pat: 'ONTraffic' class to load data and send to vehlist from traffic listener.

class OffTraffic:
    """
    Objectives:
        - To adds new vehicles from the log file (offline mode) to ``lanes.vehlist`` structure
        - To appends travel time and ID columns and log
        - To computes volumes in lanes and other traffic performance measure
        - To remove served vehicles

    .. note::
        - The CSV should be located under the ``/data/`` directory with the valid name consistent to what was inputted
            as an argument and what exists in the ``config.py`` file.
        - The scenario number should be appended to the name of intersection followed by an underscore.

    :Author:
        Ash Omidvar <aschkan@ufl.edu>
    :Date:
        Nov-2018
    .. note:: some methods of this class are adopted from the simulation version of the system, developed
    by Mahmoud Pourmehrab <mpourmehrab@ufl.edu>.
    """

    def __init__(self, intersection, sc, start_time_stamp):
        """
        Objectives:
            - To set the logging behaviour for outputting requested log files.
            - To imports the log file that includes the traffic arrivals and sort it
            - To initializes the scenario number to run

        :param intersection: contains intersection parameters
        :type intersection: Intersection
        :param sc: scenario number
        :param start_time_stamp: local time stamp to include in the CSV filename
        """
        inter_name = intersection._inter_config_params.get('inter_name')
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

        self._log_csv = intersection._inter_config_params.get('log_csv')
        self._print_commandline = intersection._inter_config_params.get('print_commandline')

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

    def get_traffic_info(self, lanes, simulation_time, intersection):
        """
        Objectives
            - Appends arrived vehicles from the CSV file to :any:`Lanes`
            - Assigns their earliest arrival time

        :param lanes: vehicles are added to this data structure
        :type lanes: Lanes
        :param simulation_time: current simulation opt_clock in seconds measured from zero
        :param intersection: intersection
        :type intersection: Intersection
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
                r'\\\ ' + veh.map_veh_type2str(det_type) + ':' + det_id + ':' + 'L' + str(lane).zfill(
                    2) + ':' + '({:>4.1f} s, {:>4.1f} m, {:>4.1f} m/s)'.format(det_time, dist, speed))

            # append it to its lane
            lanes.vehlist[lane] += [veh]  # recall it is an array
            lanes.inc_last_veh_pos(lane)
            indx += 1

        # to keep track of how much of CSV is processed
        self._current_row_indx = indx - 1

    def set_row_vehicle_level_csv(self, dep_time, veh):
        """
        Sets the departure time of an individual vehicle that is just served.

        :param dep_time: departure time in seconds
        :param veh: subject vehicle to be recorder
        :type veh: Vehicle
        """
        indx = veh.csv_indx
        self._auxilary_departure_times[indx] = dep_time
        self._auxilary_ID[indx] = veh.ID
        self._auxilary_num_sent_to_trj_planner[indx] = veh._call_reps_traj_planner

    def save_veh_level_csv(self, inter_name, start_time_stamp):
        """
        Set the recorded values and save the  CSV at vehicle level.

        :param inter_name: intersection name
        :param start_time_stamp: local time stamp to include in the CSV filename
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

    def last_veh_arr(self):
        """
        :return: True if all vehicles from the input CSV have been added at some point.

        .. note::
            The fact that all vehicles are *added* does not equal to all *served*. Thus, we check if any vehicle is in
             any of the incoming lanes before halting the program.
        """
        if self._current_row_indx + 1 >= self.__all_vehicles.shape[0]:
            return True
        else:
            return False

    def get_first_det_time(self):
        """
        :return: The time when the first vehicle in current scenario shows up. Assumes the CSV file is not sorted in arrival time.
        """
        return np.nanmin(self.__all_vehicles['arrival time'].values)

    @staticmethod
    def get_volumes(lanes, intersection):
        """
        Unit of volume in each lane is :math:`veh/sec/lane`. Uses the fundamental traffic flow equation :math:`F=D \\times S`.


        :param lanes: includes all vehicles
        :type lanes: Lanes
        :param intersection:
        :type intersection:
        :return volumes: array of volume level per lanes
        """
        # initialize volumes vector
        num_lanes = intersection._inter_config_params.get('num_lanes')
        det_range = intersection._inter_config_params.get('det_range')
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
        To remove the served vehicles and print proper notification.

        :param lanes: includes all the vehicles in all lanes
        :type lanes: Lanes
        :param simulation_time: current simulation opt_clock
        :param intersection:
        :type intersection: Intersection

        """
        num_lanes = intersection._inter_config_params.get('num_lanes')
        for lane in range(num_lanes):
            if bool(lanes.vehlist.get(lane)):  # not an empty lane
                last_veh_indx_to_remove = -1
                for veh_indx, veh in enumerate(lanes.vehlist.get(lane)):
                    det_time, _, _ = veh.get_arr_sched()
                    dep_time, _, _ = veh.get_dep_sched()
                    assert dep_time > 0, "no departure is set"
                    if dep_time < simulation_time:  # record/remove departure
                        last_veh_indx_to_remove += 1
                        intersection._inter_config_params.get('print_commandline') and print(
                            '/// ' + veh.map_veh_type2str(veh.veh_type) + ':' + veh.ID + '@({:>4.1f} s)'.format(
                                dep_time))
                        self._log_csv and self.set_row_vehicle_level_csv(dep_time, veh)
                    elif det_time < simulation_time:  # record/remove expired points
                        veh.reset_trj_pts(self.scenario_num, lane, simulation_time, self.full_traj_csv_file)

                    else:  # det_time of all behind this vehicle is larger, so we can stop.
                        break

                last_veh_indx_to_remove > -1 and lanes.remove_srv_vehs(lane, last_veh_indx_to_remove)


class TrajectoryPlanner:
    """
    Plans trajectories of all type. This makes calls to **trajectory** classes.
    # Todo: @Ash: explain planner and 4 optimizers briefly.
    :Author:
        Mahmoud Pourmehrab <pourmehrab@gmail.com>
        Ash Omidvar <aschkan@ufl.edu>
    :Date:
        April-2018
        Nov-2018
    """
# TODO @Ash: handle tester methods.
    def __init__(self, intersection):
        """Instantiates the **trajectory** classes"""

        self.lead_conventional_trj_estimator = LeadConventional(intersection)
        self.lead_connected_trj_optimizer = LeadConnected(intersection)
        self.follower_conventional_trj_estimator = FollowerConventional(intersection)
        self.follower_connected_trj_optimizer = FollowerConnected(intersection)

        self._max_speed = intersection._inter_config_params.get('max_speed')

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

        """

        veh.inc_traj_planner_calls()
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

        # self._visualizer.add_multi_trj_matplotlib(veh, lane)  # todo @Ash: remove after testing
        # self._visualizer.export_matplot(0, 510, 20, 155)  # todo @Ash: remove after testing

        if tester is not None:
            assert veh.trajectory[1, veh.last_trj_point_indx] < 1, "vehicle did not get to stop bar"
            tester.test_planned_departure(veh)
            tester.test_trj_points(veh)
            # veh_indx > 0 and tester.check_for_collision(veh, lead_veh)
    # ToDo @Ash: distinguish the tracks.
        intersection._inter_config_params.get("print_commandline") and veh.print_trj_points(lane, veh_indx, identifier)
