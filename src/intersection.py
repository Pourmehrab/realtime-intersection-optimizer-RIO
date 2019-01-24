import csv
import operator
import numpy as np
import src.geom as geom
from src.config import *
import src.util as util  # Patrick


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

    def detect_lane(self, vehicle_msg):
        """ 
        Video or GPS (UTM) based lane detection. Video 
        lane estimation not yet supported. GPS based detection
        simply computes the best estimate by the smallest
        Euclidean distance to the query point from any lane.

        :param vehicle_msg: a query vehicle message
        :type VehicleMsg: namedtuple
        
        """
        veh_utm_easting = vehicle_msg.pos[0]
        veh_utm_northing = vehicle_msg.pos[1]

        if self._inter_config_params["lane_estimation"] == "gps":
            lane = -1
            best_dist = self._inter_config_params["large_positive_num"]
            for i in range(self._inter_config_params["num_lanes"]):
                lane_info = self._inter_config_params["lanes"][i+1]
                for j in range(len(lane_info.easting)):
                    e = lane_info.easting[j]
                    n = lane_info.northing[j]
                    d = util.euclidean_dist(veh_utm_easting, veh_utm_northing, e, n)
                    if d < best_dist:
                        best_dist = d
                        lane = i+1
        elif self._inter_config_params["lane_estimation"] == "video":
            raise NotImplementedError
        return lane

    def LLA_to_distance_from_stopbar(self, lat, lon, lane):
        """
        Convert a point in latitude, longitude on the provided lane
        to the distance from that lane's stopbar.

        About +- 0.5 m accuracy.

        :param lat: Latitude
        :type float:
        :param lon: Longitude
        :type float:
        :param lane: Lane number
        :type int:
        """
        easting, northing, _, _ = utm.from_latlon(lat, lon)
        return self.UTM_to_distance_from_stopbar(easting, northing, lane)

    def UTM_to_distance_from_stopbar(self, easting, northing, lane, units="m"):
        """
        Convert a point in easting, northing on the provided lane
        to the distance from that lane's stopbar.

        About +- 0.5 m accuracy.

        :param easting:
        :type float:
        :param northing:
        type float:
        :param lane: Lane number
        :type int:
        :param units: [optional], "m" or "ft"
        :type string:
        """
        # starting from stopbar, find closest point, then compute
        lane_info = self._inter_config_params["lanes"][lane]
        stop_bar_easting = lane_info.easting[0]
        stop_bar_northing = lane_info.northing[0]
        lane_anchor_easting = lane_info.easting[-1]
        lane_anchor_northing = lane_info.northing[-1]
        
        if lane_info.is_straight:
            d = util.euclidean_dist(stop_bar_easting, stop_bar_northing, easting, northing)
            anchor_dist = util.euclidean_dist(lane_anchor_easting, lane_anchor_northing,
                    easting, northing)
            if anchor_dist > lane_info.lane_length:
                d = -d # passed the stop bar
            if units == "ft":
                return util.meters_to_feet(d)
            else:
                return d
        else:
            best = self._inter_config_params["large_positive_num"]
            best_idx = -1
            # N.b. distance should be 0 if vehicle has passed the stop bar since
            # the stop bar will be the closest point.
            # However, this will remove vehicles from the lane slightly *before*
            # they cross the stop bar in realtime mode
            for i in range(len(lane_info.easting)):
                d = util.euclidean_dist(lane_info.easting[i], lane_info.northing[i], easting, northing)
                if d < best:
                    best = d
                    best_idx = i
            if units == "ft":
                return util.meters_to_feet(lane_info.distances[best_idx])
            else:
                return lane_info.distances[best_idx]

    def distance_from_stopbar_to_LLA(self, dist, lane, tol=2, unit="m"):
        """ 
        Convert a distance from the stopbar in lane to lat, lon.

        :param dist: Distance from the stopbar.
        :type float: 
        :param lane: Lane number
        :type int:
        :param tol: Number of meters passed 0 tolerated. Negative distances
        from the stopbar will result in 0. Beyond tol, an assertion is triggered.
        :type int:
        :param unit: meters or feet.
        :type string:
        """
        lane_info = self._inter_config_params["lanes"][lane]
        if unit == "ft" or unit == "feet":
            dist = util.feet_to_meters(dist)
        assert dist > -2
        if dist < 0:
            dist = 0
        idx = np.searchsorted(lane_info.distances, dist)
        dist_gps = lane_info.distances[idx]
        # Adjust dist if greater than small threshold
        if dist_gps > dist:
            diff = dist_gps - dist
            next_idx = idx - 1
        else:
            diff = dist - dist_gps
            next_idx = idx + 1
        if diff > tol:
            y = lane_info.northing[next_idx] - lane_info.northing[idx]
            x = lane_info.easting[next_idx] - lane_info.easting[idx]
            theta = np.arctan2(y, x)
            easting = lane_info.easting[idx] + diff * np.cos(theta)
            northing = lane_info.northing[idx] + diff * np.sin(theta)
        else:
            easting = lane_info.easting[idx]
            northing = lane_info.northing[idx]
        return utm.to_latlon(easting, northing, lane_info.utmzone, lane_info.utmletter)

    def in_optimization_zone(self, vm, lane):
        """
        Checks spatial and orientiation constraints 
        to verify whether a vehicle is within a polygonal
        zone that allows for it to be processed
        by the optimization.

        Uses a +-45 cone about the roading orientation.
        If speed is near 0, returns False.
    
        N.b. in the future, for multi-lane roads,
        change lane to road name 

        :param vm: a parsed vehicle message
        :type VehicleMsg: namedtuple
        :param lane: the detected lane of the vehicle
        :type int:
        """
        cone = 90 # cone angle
        h_cone = cone/2 # half cone angle
        opt_zone_info = self._inter_config_params["opt_zones"][lane]
        query = geom.Point(vm.pos[0], vm.pos[1])
        polygon = [geom.Point(x,y) for x,y in zip(opt_zone_info.easting, opt_zone_info.northing)]
        spatial_constraint = geom.point_in_polygon(query, polygon)
        # check for 0 speed
        spd = np.sqrt((vm.vel[0] ** 2) + (vm.vel[1] ** 2))
        if np.isclose(spd, 0, 0.1):
            return False  # don't optimize when spd is approx 0
        veh_heading = util.heading_from_velocity(vm.vel)
        a = opt_zone_info.orientation - h_cone
        b = opt_zone_info.orientation + h_cone
        if a >= 0:
            orientation_constraint = True if a <= veh_heading <= b else False
        else: # Test these cases
            b2 = b; b1 = 0
            a2 = 360; a1 = a + 360
            if b1 <= veh_heading <= b2 or a1 <= veh_heading <= a2:
                orientation_constraint = True
            else:
                orientation_constraint = False
        return spatial_constraint and orientation_constraint

class Lanes:
    """
    Create data structure in which the key is lane index and value is an arrays a vehicle objects in corresponding lane.

    Objectives:
        - To keep vehicles in order
        - To keep track of served/un-served vehicles (vehicles which have received optimized trajectory/vehicles which
        are waiting for optimized trajectory
        - Other low level methods: Check if all lanes are empty, maintain indices of vehicles in arrays, etc.

    :Author:
        Mahmoud Pourmehrab <pourmehrab@gmail.com>

        Ash Omidvar <aschkan@ufl.edu>
    :Date:
        Oct-2018

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

    def find_and_return_vehicle_by_id(self, lane, veh_id):
        """Finds and returns the vehicle in the lane with
        the query id. If not present, returns None.

        :param lane: the lane number for indexing into the vehlist
        :type int:
        :param veh_id: the detection id associated with the desired vehicle
        :type string:
        """
        for v in self.vehlist[lane]:
            if v.ID == veh_id:
                return v
        return None

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
        :param self.resched_dep: True if a vehicle is available to receive a new departure time,
        False if want to keep the previous trajectory
        :type self.resched_dep: bool
        :param self.got_trajectory: True if a vehicle is just scheduled a **different** departure and is 
        assigned a new trajectory. Set within signal.py's ``solve`` method.
        :type self.got_trajectory: bool
        :param self._call_reps_traj_planner: number of times a vehicle object trajectory is updated.

        .. note::
            - By definition ``scheduled_departure`` is always greater than or equal to ``earliest_arrival``.
            - It is important that user sets an ideal size of trajectory array by ``max_num_traj_points``.
            - A vehicle may be available to be rescheduled but gets the same departure time;
            in that case, ``self.got_trajectory``  should hold False.

        """
        self.ID = det_id
        self.veh_type = det_type
        # TODO: Where is this used?
        self.init_time = det_time
        self.length = length
        self.max_decel_rate = a_min
        self.max_accel_rate = a_max
        self.destination = dest
        self.desired_speed = des_speed
        self.csv_indx = indx  # is used to find the vehicle in log file (offline mode)

        self.trajectory = np.zeros((3, intersection._inter_config_params.get('max_num_traj_points')),
                                   dtype=np.float)
        self.min_dist_to_stop_bar = intersection._inter_config_params.get("min_dist_to_stop_bar")
        self.verbose = intersection._inter_config_params.get("print_commandline")
        
        self.first_trj_point_indx, self.last_trj_point_indx = 0, -1
        # computed trajectory
        self.trajectory[:, self.first_trj_point_indx] = [det_time, dist, speed, ]
        
        # Patrick: Most recent position and speed
        self.current_state = np.zeros(3) # timestamp, distance from stopbar in meters, speed in m/s

        if det_type == 1:
            self.poly = {'ref. time': 0.0, 'coeffs': np.zeros(intersection._inter_config_params.get('k'))}

        self.earliest_departure = 0.0
        self.scheduled_departure = 0.0
        self.resched_dep, self.got_trajectory = True, False
        self._call_reps_traj_planner = 0
        # This is for grouping trajectories together during logging
        self._logging_id = id(self)
            
    def set_sched_dep(self, t_scheduled, d_scheduled, s_scheduled, lane, veh_indx):
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

        det_time, det_dist, det_speed = self.get_arr_sched()
        if det_dist >= self.min_dist_to_stop_bar:

            self.set_first_trj_pt_indx(0)
            self.trajectory[:, 0] = [det_time, det_dist, det_speed]
            self.set_last_trj_pt_indx(1)
            self.trajectory[:, 1] = [t_scheduled, d_scheduled, s_scheduled]
            self.scheduled_departure = t_scheduled

            self.verbose and self.print_trj_points(lane, veh_indx, "@")

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
        Writes the trajectory points in the log file if the time stamp is before the ``time_threshold``
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
                writer.writerows([[sc, self.ID, self.veh_type, lane + 1, time, distance, speed, self._call_reps_traj_planner]])
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

    def update(self, veh_type, det_time, dist_from_stopbar, speed):
        """ 
        Update vehicle type, current state, and trajectory
        with latest vehicle data from real-time sensing
        
        :param veh_type: the level of connectivity (CNV or CAV)
        :type int:
        :param det_time: the relative time in seconds when this message
        was received
        :type float:
        :dist_from_stopbar: distance in meters from the stop bar of the lane
        :type float:
        :param speed: vehicle speed
        :type float:
        """
        self.veh_type = veh_type
        self.current_state = np.array([det_time, dist_from_stopbar, speed])
        # Update the first trj point so a new trajectory is computed based on latest info
        if dist_from_stopbar > self.min_dist_to_stop_bar:
            # TODO: Do i need to call self.reset_trj_points?
            self.trajectory[:, self.first_trj_point_indx] = [det_time, dist_from_stopbar, speed]
