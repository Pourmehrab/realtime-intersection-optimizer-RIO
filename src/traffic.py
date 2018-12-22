import os
import csv
import pandas as pd
import numpy as np
from scipy import stats

from src.intersection import Vehicle

class RealTimeTraffic:
    """
    TODO
    """

    def __init__(self, vehicle_data_queue, track_split_merge_queue, cav_traj_queue, intersection, sc, args):
        """
        """
        self.intersection = intersection
        inter_name = args.intersection
        # set the scenario number
        self.scenario_num = sc
        self._print_commandline = intersection._inter_config_params.get('print_commandline')
        self._vehicle_data_queue = vehicle_data_queue
        self._track_split_merge_queue = track_split_merge_queue
        self._cav_traj_queue = cav_traj_queue
        if args.do_logging: 
            pass
            # df_size = len(self.__all_vehicles)
            # self._auxilary_departure_times = np.zeros(df_size, dtype=np.float)
            # self._auxilary_ID = ['' for i in range(df_size)]
            # self._auxilary_num_sent_to_trj_planner = np.zeros(df_size, dtype=np.int8)

            # # open a file to store trajectory points
            # filepath_trj = os.path.join('log/' + inter_name + '/' + start_time_stamp + '_' + str(
            #     self.scenario_num) + '_trj_point_level.csv')
            # self.full_traj_csv_file = open(filepath_trj, 'w', newline='')
            # writer = csv.writer(self.full_traj_csv_file, delimiter=',')
            # writer.writerow(['sc', 'VehID', 'type', 'lane', 'time', 'distance', 'speed'])
            # self.full_traj_csv_file.flush()
        else:
            self.full_traj_csv_file = None
    
    def get_traffic_info(self, lanes):
        """
        Objectives
            - Appends arrived vehicles from the CSV file to :any:`Lanes`
            - Assigns their earliest arrival time

        :param lanes: vehicles are added to this data structure
        :type lanes: Lanes
        """
        # Read latest msgs from queues
        if self._track_split_merge_queue.count() != 0:
            self._track_split_merge_queue.pop()
            # Handle track split/merge msgs - TODO
        
        if self._vehicle_data_queue.count() != 0:
            vehicle_data_msgs = self._vehicle_data_queue.pop()
            # Lane detection
            for vm in vehicle_data_msgs:
                lane = self.intersection.detect_lane(vm)
                if lane < 0:
                    continue
                # check if the vehicle is already in this lane
                veh_id = vm.track_id + ":" + vm.dsrc_id
                v = lanes.find_and_return_vehicle_by_id(lane, veh_id)
                if v is None:
                    # in the optimization zone?
                    if self.intersection.in_optimization_zone(vm):

                        # convert vehicle message to Vehicle
                        det_id = vm.track_id + ":" + vm.dsrc_id
                        det_type = vm.veh_type
                        #det_time = # ? 
                        dist = self.intersection.UTM_to_distance_from_stopbar(vm.pos[0], vm.pos[1], lane)
                        speed = np.sqrt(vm.speed[0] ** 2 + vm.speed[1] ** 2)
                        #des_speed = ? intersection speed limit?
                        #dest = "" # ?
                        length = vm.veh_len
                        amin = vm.max_decel
                        amax = vm.max_accel

                        #indx? 
                        veh = Vehicle(det_id, det_type, det_time, speed, dist, des_speed, dest, length, amin, amax, indx,
                                      self.intersection)

                        self._print_commandline and print(
                            r'\\\ ' + veh.map_veh_type2str(det_type) + ':' + det_id + ':' + 'L' + str(lane).zfill(
                                2) + ':' + '({:>4.1f} s, {:>4.1f} m, {:>4.1f} m/s)'.format(det_time, dist, speed))

                        # append it to its lane
                        lanes.vehlist[lane] += [veh]  # recall it is an array
                        lanes.inc_last_veh_pos(lane)
                        indx += 1
                else:
                    # TODO
                    # update v with latest data
                    v.update(vm)

        # N.b. eventually, add a flagging system so only vehicles with changes made to them 
        #   get new trajectories, to save on computation

    # TODO: This is all logging stuff that needs to be re-done for real time
    # Need to store all trajectories when before and after update
    # def set_row_vehicle_level_csv(self, dep_time, veh):
    #     """
    #     Sets the departure time of an individual vehicle that is just served.
    #
    #     :param dep_time: departure time in seconds
    #     :param veh: subject vehicle to be recorder
    #     :type veh: Vehicle
    #     """
    #     indx = veh.csv_indx
    #     self._auxilary_departure_times[indx] = dep_time
    #     self._auxilary_ID[indx] = veh.ID
    #     self._auxilary_num_sent_to_trj_planner[indx] = veh._call_reps_traj_planner

    # def save_veh_level_csv(self, inter_name, start_time_stamp):
    #     """
    #     Set the recorded values and save the  CSV at vehicle level.
    #
    #     :param inter_name: intersection name
    #     :param start_time_stamp: local time stamp to include in the CSV filename
    #     """
    #     self.__all_vehicles['departure time'] = self._auxilary_departure_times
    #     self.__all_vehicles['ID'] = self._auxilary_ID
    #     self.__all_vehicles['times_sent_to_trj_planner'] = self._auxilary_num_sent_to_trj_planner
    #
    #     filepath = os.path.join(
    #         'log/' + inter_name + '/' + start_time_stamp + '_' + str(self.scenario_num) + '_trj_veh_level.csv')
    #     self.__all_vehicles.to_csv(filepath, index=False)

    # def close_trj_csv(self):
    #     """Closes trajectory CSV file."""
    #     self.full_traj_csv_file.close()
    #
    # def last_veh_arr(self):
    #     """
    #     :return: True if all vehicles from the input CSV have been added at some point.
    #
    #     .. note::
    #         The fact that all vehicles are *added* does not equal to all *served*. Thus, we check if any vehicle is in
    #          any of the incoming lanes before halting the program.
    #     """
    #     if self._current_row_indx + 1 >= self.__all_vehicles.shape[0]:
    #         return True
    #     else:
    #         return False
    #
    # def get_first_det_time(self):
    #     """
    #     :return: The time when the first vehicle in current scenario shows up. Assumes the CSV file is not sorted in arrival time.
    #     """
    #     return np.nanmin(self.__all_vehicles['arrival time'].values)

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

    def serve_update_at_stop_bar(self, lanes, elapsed_time, intersection):
        """
        To remove the served vehicles and print proper notification.

        :param lanes: includes all the vehicles in all lanes
        :type lanes: Lanes
        :param elapsed_time: time since start
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
                    if dep_time < elapsed_time:  # record/remove departure
                        last_veh_indx_to_remove += 1
                        intersection._inter_config_params.get('print_commandline') and print(
                            '/// ' + veh.map_veh_type2str(veh.veh_type) + ':' + veh.ID + '@({:>4.1f} s)'.format(
                                dep_time))
                        #self._log_csv and self.set_row_vehicle_level_csv(dep_time, veh)
                    elif det_time < elapsed_time:  # record/remove expired points
                        veh.reset_trj_pts(self.scenario_num, lane, elapsed_time, self.full_traj_csv_file)

                    else:  # det_time of all behind this vehicle is larger, so we can stop.
                        break

                last_veh_indx_to_remove > -1 and lanes.remove_srv_vehs(lane, last_veh_indx_to_remove)

    def publish(self, lanes):
        # Push vehicles with a DSRC id out to the TrafficPublisher
        num_lanes = self.intersection._inter_config_params.get('num_lanes')
        for lane in range(num_lanes):
            if bool(lanes.vehlist.get(lane)):  # not an empty lane
                for veh in lanes.vehlist.get(lane):
                    dsrc_id = veh.det_id.split(":")[1]
                    if dsrc_id != "" and veh.got_trajectory:
                        self._cav_traj_queue.append(veh)
class SimTraffic:
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

    def __init__(self, intersection, sc, start_time_stamp, args):
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
        self._intersection = intersection
        #inter_name = intersection._inter_config_params.get('inter_name')
        inter_name = args.intersection
        # get the path to the CSV file and load up the traffic
        filepath = os.path.join(
            'data', inter_name, inter_name + '_' + str(sc) + '.csv')
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
            filepath_trj = os.path.join(args.log_dir, start_time_stamp + '_' + str(
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
        :param simulation_time: current simulation clock in seconds measured from zero
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
                    volumes[lane] = num_of_vehs / det_range[lane] * s
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
        :param simulation_time: current simulation clock
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
                    if dep_time != 0:
                        # n.b. dep_time == 0 for vehicles that arrive but haven't yet 
                        # been assigned a trajectory (e.g., if trajectories are only assigned
                        #  once every 2 seconds)
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
    
    def publish(self, lanes):
        """TODO: incrementally write to CSV """
        pass