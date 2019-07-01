import os
import csv
import pandas as pd
import numpy as np
from scipy import stats
from src.intersection import Vehicle
import datetime as dt


class RealTimeTraffic:
    """
    This class provides methods for processing the data received from the sensor fusion module 
    in real-time. It extracts vehicle data and updates the necessary data structures.
    """

    def __init__(self, vehicle_data_queue, track_split_merge_queue, cav_traj_queue,
                 init_time_stamp, intersection, args):
        """
        Initializes the RealTimeTraffic object. Stores a float, time_since_last_arrival,
        which can be used to get the elapsed time since the last vehicle message has been received.

        :param vehicle_data_queue: The shared queue that gets populated with new traffic data 
        msgs from the sensor fusion
        :type vehicle_data_queue: deque
        :param track_split_merge_queue: The shared queue that gets populated with 
        track split/merge messages from the sensor fusion
        :type track_split_merge_queue: deque
        :param cav_traj_queue: The shared queue that this class populates with outgoing messages
        to be sent via DSRC
        :type cav_traj_queue: deque
        :param init_time_stamp: Datetime timestamp
        :param intersection: the global intersection object
        :type intersection: Intersection
        :param args: the runtime args
        :type args: Namespace
        """
        self.intersection = intersection
        self.scenario_num = args.sc
        self._print_commandline = intersection._inter_config_params.get('print_commandline')
        self._vehicle_data_queue = vehicle_data_queue
        self._track_split_merge_queue = track_split_merge_queue
        self._cav_traj_queue = cav_traj_queue
        self._total_count = 0
        self.time_since_last_arrival = init_time_stamp
        if args.do_logging:
            # open a file to store trajectory points
            filepath_trj = os.path.join(args.log_dir, 'trajectories.csv')
            self.full_traj_csv_file = open(filepath_trj, 'w', newline='')
            writer = csv.writer(self.full_traj_csv_file, delimiter=',')
            writer.writerow(['sc', 'VehID', 'type', 'lane', 'time', 'distance', 'speed', 'calls to traj planner'])
            self.full_traj_csv_file.flush()

            # open a file to store arrivals and departures
            filepath_arr_csv = os.path.join(args.log_dir, 'arrivals_departures.csv')
            self.arrs_deps_csv = open(filepath_arr_csv, 'w', newline='')
            writer = csv.writer(self.arrs_deps_csv, delimiter=',')
            writer.writerow(['vehicleVIN', 'sc', 'lane', 'type',
                'arrival time', 'arrival speed', 'arrival distance', 'L',
                'maxAcc', 'maxDec', 'dest', 'desSpd', 'count', 'total count','departure time'])
            self.arrs_deps_csv.flush()
        else:
            self.full_traj_csv_file = None
            self.arrs_deps_csv = None

    def get_time_since_last_arrival(self, current_time):
        return (current_time - self.time_since_last_arrival).total_seconds()

    def get_traffic_info(self, lanes, time_tracker):
        """
        Reads any new data from shared queues and processes it
        appropriately.

        :param lanes: vehicles are added to this data structure
        :type lanes: Lanes
        :param time_tracker: Timer object for grabbing elapsed time since start
        """
        # Read latest msg from track split/merge queue
        if len(self._track_split_merge_queue) != 0:
            self._track_split_merge_queue.pop()
            # Handle track split/merge msgs - TODO

        # Read latest msg from vehicle data queue
        if len(self._vehicle_data_queue) != 0:

            vehicle_data_msgs = self._vehicle_data_queue.pop()
            # Lane detection
            for vm in vehicle_data_msgs:
                lane = self.intersection.detect_lane(vm)
                if lane == -1:
                    continue
                print("lane detected: {}".format(lane))
                # check if the vehicle is already in this lane
                veh_id = vm.track_id + ":" + vm.dsrc_id
                det_id = veh_id
                det_type = vm.veh_type
                det_time, _ = time_tracker.get_time(vm.timestamp)
                # Convert vehicle state to lane-centric coordinates; essentially, 
                # a 1D coordinate system where the origin is the stopbar of the detected lane.
                dist = self.intersection.UTM_to_distance_from_stopbar(vm.pos[0], vm.pos[1], lane)
                speed = np.sqrt(vm.vel[0] ** 2 + vm.vel[1] ** 2)
                v = lanes.find_and_return_vehicle_by_id(lane, veh_id)

                if v is None:
                    # in the optimization zone?
                    if self.intersection.in_optimization_zone(vm, lane):
                        print("In Opt zone!")
                        self.time_since_last_arrival = time_tracker.get_time()[1]
                        # convert vehicle message to Vehicle
                        des_speed = self.intersection._inter_config_params["max_speed"]
                        # TODO dest = "" # ? - add lookup by lane number for now
                        dest =  1
                        length = vm.veh_len
                        amin = vm.max_decel
                        amax = vm.max_accel

                        veh = Vehicle(det_id, det_type, det_time, speed, dist, des_speed, dest, length, amin, amax,
                                      None, self.intersection)

                        self._print_commandline and print(
                            r'\\\ ' + veh.map_veh_type2str(det_type) + ':' + det_id + ':' + 'L' + str(lane).zfill(
                                2) + ':' + '({:>4.1f} s, {:>4.1f} m, {:>4.1f} m/s)'.format(det_time, dist, speed))

                        # append it to its lane
                        lanes.vehlist[lane] += [veh]  # recall it is an array
                        lanes.inc_last_veh_pos(lane)
                    else:
                        # TODO: when debugging, print or log here
                        print("Not in opt zone!")
                else:
                    # TODO: vehicle gets purged bc current state is 0 before this ever gts called
                    # update v with latest data
                    print("Updating existing vehicle")
                    v.update(det_type, det_time, dist, speed)
        # N.b. eventually, add a flagging system so only vehicles with changes made to them 
        #   get new trajectories, to save on computation

    def close_trj_csv(self):
        """Closes trajectory CSV file."""
        self.full_traj_csv_file.close()

    def close_arrs_deps_csv(self):
        """Closes arrivals and deps csv file"""
        self.arrs_deps_csv.close()
    
    def log_arrival_departure(self, dep_time, lane, veh, lanes, start_time_stamp):
        """Once a veh enters the intersection, log its arrival/departure info """
        if self.arrs_deps_csv is not None:
            writer = csv.writer(self.arrs_deps_csv, delimiter=',')
            abs_arrival_time = dt.timedelta(seconds=veh.arrival_info[0]) + start_time_stamp
            abs_dep_time = dt.timedelta(seconds=dep_time) + start_time_stamp
            data = ['{},{},{},{},{},{:.3f},{:.3f},{},{},{},{},{},{},{},{}'.format(
                veh.ID, self.scenario_num , lane, veh.veh_type, abs_arrival_time,
                veh.arrival_info[2], veh.arrival_info[1],
                veh.length, veh.max_accel_rate, veh.max_decel_rate, veh.destination,
                veh.desired_speed, lanes.count[lane], self._total_count, abs_dep_time
                )]
            writer.writerow(data)
            self.arrs_deps_csv.flush()

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

    def update_trj_or_serve_at_stop_bar(self, lanes, elapsed_time, intersection,
             start_time_stamp=None):
        """
        To remove the served vehicles and print proper notification.
        In realtime mode, vehicles are removed based on whether
        the most recent position as provided by the sensor fusion
        has them currently crossing the stop bar. If the vehicle hasn't departed,
        updates the optimized trajectory indices to reflect current time.

        :param lanes: includes all the vehicles in all lanes
        :type lanes: Lanes
        :param elapsed_time: time since start
        :param intersection:
        :type intersection: Intersection
        :param start_time_stamp: DateTime timestamp for logging in real-time mode
        """
        num_lanes = intersection._inter_config_params.get('num_lanes')
        for lane in range(num_lanes):  # 0-based indexing for lanes internally
            if bool(lanes.vehlist.get(lane)):  # not an empty lane
                last_veh_indx_to_remove = -1
                for veh_indx, veh in enumerate(lanes.vehlist.get(lane)):
                    det_time, _, _ = veh.get_arr_sched()
                    if veh.current_state[1] <= 0.:  # distance from stop bar (m)
                        dep_time, _, _ = veh.get_dep_sched()
                        last_veh_indx_to_remove += 1
                        intersection._inter_config_params.get('print_commandline') and print(
                            '/// ' + veh.map_veh_type2str(veh.veh_type) + ':' + veh.ID + '@({:>4.1f} s)'.format(
                                dep_time))
                        self._total_count += 1
                        # log stuff
                        self.log_arrival_departure(dep_time, lane, veh, lanes, start_time_stamp)
                    elif det_time < elapsed_time:  # record/remove expired points
                        veh.update_trj_pts(self.scenario_num, lane, elapsed_time,
                             self.full_traj_csv_file, start_time_stamp)
                    else:  # distance from stop bar of all behind this vehicle is larger, so we can stop.
                        break

                last_veh_indx_to_remove > -1 and lanes.remove_srv_vehs(lane, last_veh_indx_to_remove)

    def publish(self, lanes, curr_time):
        """
        Push vehicles with a DSRC id out to the TrafficPublisher,
        which will parse the trajectories into an IAM
        message and relay it via DSRC.

        :param lanes: the Lane data structure
        :curr_time: the absolute time stamp
        """
        num_lanes = self.intersection._inter_config_params.get('num_lanes')
        for lane in range(num_lanes):
            if bool(lanes.vehlist.get(lane)):  # not an empty lane
                for veh in lanes.vehlist.get(lane):
                    dsrc_id = veh.ID.split(":")[1]
                    if dsrc_id != "" and veh.got_trajectory:
                        self._cav_traj_queue.append((veh, lane, curr_time))


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

    def __init__(self, intersection, sc, start_time_stamp, args, min_dist):
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
        # inter_name = intersection._inter_config_params.get('inter_name')
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

        self._log_csv = args.do_logging 
        self._print_commandline = intersection._inter_config_params.get('print_commandline')

        if self._log_csv:
            df_size = len(self.__all_vehicles)
            self._auxilary_departure_times = np.zeros(df_size, dtype=np.float)
            self._auxilary_ID = ['' for i in range(df_size)]
            self._auxilary_num_sent_to_trj_planner = np.zeros(df_size, dtype=np.int8)

            # open a file to store trajectory points
            filepath_trj = os.path.join(args.log_dir, 'trj_point_level_' + str(min_dist) + '.csv')
            self.full_traj_csv_file = open(filepath_trj, 'w', newline='')
            self.veh_level_csv_file = os.path.join(args.log_dir, 'trj_veh_level_' + str(min_dist) + '.csv')
            writer = csv.writer(self.full_traj_csv_file, delimiter=',')
            writer.writerow(['sc', 'VehID', 'type', 'lane', 'time', 'distance', 'speed'])
            self.full_traj_csv_file.flush()

            # open a file to store individual vehicle delay
            filepath_delay = os.path.join(args.log_dir, 'veh_delay_' + str(min_dist) + '.csv')
            self.veh_delay_csv_file = open(filepath_delay, 'w', newline='')
            writer = csv.writer(self.veh_delay_csv_file, delimiter=',')
            writer.writerow(["Vehicle #", 'Vehicle Delay'])

            # open a file to store sorted trajectory points
            filepath_sort = os.path.join(args.log_dir, 'trj_point_sort_' + str(min_dist) + '.csv')
            self.trj_sort_csv_file = open(filepath_sort, 'w', newline='')
        else:
            self.full_traj_csv_file = None
        
        self.time_since_last_arrival = 0
    
    def get_time_since_last_arrival(self, current_time):
        return current_time - self.time_since_last_arrival

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
            lane = int(self.__all_vehicles['lane'][indx])
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
            lanes.vehlist[lane] += [veh]  # recall it is a list
            lanes.inc_last_veh_pos(lane)
            indx += 1
            self.time_since_last_arrival = simulation_time

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

        self.__all_vehicles.to_csv(self.veh_level_csv_file, index=False)

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

    def update_trj_or_serve_at_stop_bar(self, lanes, simulation_time, intersection):
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
                            veh.update_trj_pts(self.scenario_num, lane, simulation_time, self.full_traj_csv_file)

                        else:  # det_time of all behind this vehicle is larger, so we can stop.
                            break

                last_veh_indx_to_remove > -1 and lanes.remove_srv_vehs(lane, last_veh_indx_to_remove)

    def publish(self, lanes, curr_time):
        """TODO: incrementally write to CSV """
        # mport pickle
        # import pdb; pdb.set_trace()
        # print("save here")
        pass
