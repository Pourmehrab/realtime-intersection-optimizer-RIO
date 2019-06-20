import os
import glob
import utm
from collections import namedtuple
import src.util as util


# -------------------------------------------------------
# Vehicle and Intersection Configuration Parameters
# -------------------------------------------------------
class Lane:
    def __init__(self):
        self.utmzone = ""
        self.utmletter = ""
        self.easting = []
        self.northing = []
        self.distances = []
        self.lane_length = 0
        self.is_straight = True


class OptZone:
    def __init__(self):
        self.utmzone = ""
        self.utmletter = ""
        self.easting = []
        self.northing = []
        self.orientation = -1


def load_optimization_zone_constraints(inter_name):
    """
    Parse and store optimization zone information as
    a dictionary of OptZone objects, indexed by lane number.
    TODO: in the future, index these by road, not lane,
    since for multi-lane roads, the opt zone is shared across
    lanes for a single road.

    :param inter_name: the name of the intersection, must match the
    folder name containing opt_zones.csv
    :type string:
    """
    zones = {}
    dir_path = os.path.dirname(os.path.realpath(__file__))
    opt_file = os.path.join(dir_path, "..", "data", inter_name, "opt_zones.csv")
    prev_lane_number = None
    o = None
    with open(opt_file, 'r') as f:
        f.readline()  # throw away header
        next_line = f.readline()
        while next_line:
            name, lat, lon, orientation = next_line.strip().split(",")
            lane = name.split(".")[0]
            lane_number = int(lane.split("_")[1])
            if lane_number != prev_lane_number:
                if o:
                    zones[prev_lane_number] = o
                o = OptZone()
                prev_lane_number = lane_number
            e, n, zone_num, zone_letter = utm.from_latlon(float(lat), float(lon))
            o.utmzone = zone_num
            o.utmletter = zone_letter
            o.easting.append(e)
            o.northing.append(n)
            if orientation != "-":
                o.orientation = float(orientation)
            next_line = f.readline()
        zones[prev_lane_number] = o
    return zones


def load_lane_geom(inter_name):
    """
    Parse and load lane geometry information into Lane objects and
    store in a dictionary indexed by lane number.
       TODO:
        - add scipy.linregress to test whether GPS points form straight line, to set is_straight
    
    :param inter_name: the name of the intersection, must match the
    folder name containing opt_zones.csv
    :type string:
    """
    dir_path = os.path.dirname(os.path.realpath(__file__))
    lane_file = os.path.join(dir_path, "..", "data", inter_name, "lanes.csv")
    lanes = {}
    prev_lane_id = None
    l = None
    with open(lane_file, 'r') as f:
        f.readline()  # throw away header
        next_line = f.readline()
        while next_line:
            name, lat, lon = next_line.split(",")
            lane_id = int(name.split("_")[1])
            if lane_id != prev_lane_id:
                if l:
                    total_length = 0.
                    l.distances.append(total_length)
                    for i in range(len(l.easting) - 1):
                        total_length += util.euclidean_dist(l.easting[i], l.northing[i], l.easting[i + 1],
                                                            l.northing[i + 1])
                        l.distances.append(total_length)
                    l.lane_length = total_length
                    l.is_straight = True
                    lanes[prev_lane_id] = l
                l = Lane()
                prev_lane_id = lane_id
            e, n, zone_num, zone_letter = utm.from_latlon(float(lat), float(lon))
            l.utmzone = zone_num
            l.utmletter = zone_letter
            l.easting.append(e)
            l.northing.append(n)
            next_line = f.readline()
        lanes[prev_lane_id] = l
    return lanes


def load_inter_params(inter_name):
    """
    :return:
        - inter_name: intersection name
        - max_speed: maximum speed in :math:`m/s`
        - min_headway: the lowest headway at the stop bar in :math:`s` (corresponds to the highest flow)
        - det_range: detection range in :math:`m`
        - k, m: Lead vehicle dynamics polynomial parameters
        - num_lanes: total number of incoming lanes
        - phase_cover_set: a subset of phases that cover all lanes
        - small_positive_num: small number that lower than that is approximated by zero
        - large_positive_num: large number: is a large number to initialize badness of alternatives in GA. Make sure cannot be beaten by worst alternative.
        - lli: a dictionary of sets. The key is the phase number and is zero-based. The value to a key is a set of lanes included in that phase (lanes are also zero-based)
        - pli: a dictionary of sets where the keys are lane numbers and must be zero-based. The value for each key is a set of lane numbers that are in conflict with the key lane (again, must be zero-based).
        - allowable_phases: subset of all possible phases to be used. These are different than the phase_cover_set
        - yellow
        - allred
        - min_green
        - max_green
        - lag_on_green: The lag time from start of green when a vehicle can depart to allow vehicle cross after green (in seconds).
        - max_num_traj_points: check if it's enough to preallocate the trajectory
        - min_dist_to_stop_bar: lower than this (in m) do not update schedule
        - do_traj_computation:
        - trj_time_resolution: time difference between two consecutive trajectory points in seconds used in :any:`discretize_time_interval()` (be careful not to exceed max size of trajectory)
        - log_csv: if set `True`, makes CSV files of the outputs
        - print_commandline:
        - lane_estimation: the method to use to estimate a vehicle's lane upon receiving a detection. Either "gps" or "video" (video not yet supported).
        - opt_zones: [optional] for running at real intersections. Geometric information about zones within
          which vehicles can receive trajectories.
        - lanes: [optional] for running at real intersections. Geometric information about each lane.

    .. note::
        - The distance to stop bar will be input from either CSV file or fusion. However, the number provided here is used for generic computations.
        - odd degree of polynomial is recommended: k to be even and **at least** 5
        - Make sure the ``max_num_traj_points`` to preallocate the trajectories is enough for a given problem


    An intersection configuration can be specified by its lanes and movements (left, through, right) that are allowed in
    each lane. The lane-lane incidence matrix of an intersection is a squared matrix that holds 1 (shown by solid
    circles in the figures), if two lanes are in conflict. The standard types of conflicts that may want to be avoided
    are cross, merge, and diverge conflicts.

    Depending on the design, the definition of conflicts points can be broader or more limited. For instance, if volume
    of a lane is too low and extensive gaps can be found, some of conflict points can be considered as non-conflicting
    points. In the following figures, only cross and merge conflict
    points are indicated.

    .. figure:: images/TERL.JPG
       :width: 4cm
       :height: 4cm
       :align: center
       :alt: map to buried treasure

       The TERL facility.

    .. figure:: images/reserv.JPG
       :width: 8cm
       :height: 8cm
       :align: center
       :alt: map to buried treasure

       The reservation-based intersection.

    .. figure:: images/13th16th.JPG
       :width: 10cm
       :height: 10cm
       :align: center
       :alt: map to buried treasure

    .. warning:: All the parameters defined here are required for running the program
       unless otherwise indicated.


    :Authors:
        Mahmoud Pourmehrab <pourmehrab@gmail.com>
        Ash Omidvar <aschkan@ufl.edu>
    :Date:
        April-2018
        Dec-2018
    """
    lane_info = load_lane_geom(inter_name)
    opt_zone_info = load_optimization_zone_constraints(inter_name)

    if inter_name == "RTS":
        return {
            "max_speed": 6.7,  # m/s (24 kph)
            "min_CAV_headway": 1.5,
            "min_CNV_headway": 2.0,
            "det_range": (240, 300, 80, 100,),
            "k": int(4),
            "m": int(40),
            "num_lanes": int(4),
            "phase_cover_set": (0, 1,),
            "small_positive_num": 0.01,
            "large_positive_num": 999_999_999,
            "pli": {0: {0, 1, },  # North/South bounds throughs
                    1: {2, 3, },  # East/West bounds throughs
                    },
            "lli": {0: {1, 3, },  # Northeast (ATC: 1) - Lane: 0
                    1: {0, 2, },  # Southwest (ATC: 2) - Lane: 1
                    2: {1, 3, },  # Southeast (ATC: 3) - Lane: 2
                    3: {0, 2, },  # Northwest (ATC: 4) - Lane: 3
                    },
            "allowable_phases": (0, 1,),
            "yellow": 3.0,
            "allred": 1.5,
            "min_green": 5.0,
            "max_green": 20.0,
            "lag_on_green": 1.0,
            "max_num_traj_points": int(1_000),
            "min_dist_to_stop_bar": 20,
            "do_traj_computation": True,
            "trj_time_resolution": 1.0,
            "print_commandline": True,
            "lane_estimation": "gps",
            "opt_zones": opt_zone_info,
            "lanes": lane_info
        }
    elif inter_name == "TERL":
        return {
            "max_speed": 17.8816,  # 40 mph
            "min_CAV_headway": 1.5,
            "min_CNV_headway": 2.0,
            "det_range": tuple([500.0] * 6),
            "k": int(20),
            "m": int(40),
            "num_lanes": int(6),
            "phase_cover_set": (0, 1, 2, 3,),
            "small_positive_num": 0.01,
            "large_positive_num": 999_999_999,
            "lli": {0: {1, 2, 3, 4, 5, },
                    1: {0, 3, 5, },
                    2: {0, 3, 4, },
                    3: {0, 1, 2, 4, 5},
                    4: {0, 2, 3, },
                    5: {0, 1, 3, }, },
            "pli": {0: {0, },  # Southbound (signal controller: phase 2)
                    1: {4, 5, },  # Eastbound (signal controller: phase 3)
                    2: {1, 2, },  # Westbound (signal controller: phase 4)
                    3: {3, },  # Northbound (signal controller: phase 6)
                    4: {1, 4, },  # dual opposite throughs
                    5: {2, 5, },  # dual left turns
                    },
            "allowable_phases": (0, 1, 2, 3,),
            "yellow": 1.5,
            "allred": 1.5,
            "min_green": 4.6,
            "max_green": 25.0,
            "lag_on_green": 1.0,
            "max_num_traj_points": int(1_000),
            "min_dist_to_stop_bar": 50,
            "do_traj_computation": True,
            "trj_time_resolution": 1.0,
            "print_commandline": True,
        }
    elif inter_name == "GaleStadium":
        return {
            "max_speed": 8.94, # 20 mph in m/s,
            "min_CAV_headway": 1.5,
            "min_CNV_headway": 2.0,
            "det_range": (38, 59, 11, 190, 20, 156, 49, 14),
            "k": int(4),
            "m": int(40),
            "num_lanes": int(8),
            "phase_cover_set": (0, 1, 2, 3,),
            "small_positive_num": 0.01,
            "large_positive_num": 999_999_999,
            "pli": {0: {0, 5, }, # North bounds all
                    1: {1, 4, }, # South bounds all
                    2: {2, 7, }, # East bounds all
                    3: {3, 6, }, # West bounds all
                   },
            "lli": None,  # Ash: no need for lli in this phasing config.
            "allowable_phases": (0, 1, 2, 3,),
            "yellow": 3.0,
            "allred": 1.5,
            "min_green": 5.0,
            "max_green": 25.0,
            "lag_on_green": 1.0,
            "max_num_traj_points": int(1_000),
            "min_dist_to_stop_bar": 25,
            "do_traj_computation": True,
            "trj_time_resolution": 1.0,
            "print_commandline": True,
            "lane_estimation": "gps",
            "opt_zones": opt_zone_info,
            "lanes": lane_info
        }
    else:
        raise Exception("Simulation parameters are not known for this intersection.")


# -------------------------------------------------------
# SIGNAL CONTROLLER PARAMETERS
# -------------------------------------------------------
# add intersection-specific inputs above, under `load_inter_params(inter_name)`
def get_sig_ctrl_interface_params(inter_name):
    """
    :return:
        - Proper phases to be called

    .. note::
        - Account for SNMP lag time. Depending on the processor capability: [0.1 s - 0.9 s]

    :Author:
        Ash Omidvar <aschkan@ufl.edu>
    :Date:
        May-2018
    """
    if inter_name == "TERL":
        num_phase = 8  # Total Number of phases at the TERL
        al = range(1, num_phase + 1)
        non = [0]
        nonConflict = [[2], [3, 8], [4, 7], [6]]  # Conflict monitor phases

    elif inter_name == "RTS":
        #
        num_phase = 4  # Total Number of phases at RTS
        al = range(1, num_phase + 1)
        non = [0]
        nonConflict = [[1, 2], [4, 3]]  # Conflict monitor phases
        
    elif inter_name == "GaleStadium":
        #
        num_phase = 4  # Total Number of phases at GaleStadium
        al = range(1, num_phase + 1)
        non = [0]
        nonConflict = [[1], [2], [4], [3]]  # Conflict monitor phases
    else:
        raise Exception("Controller parameters are not known for this intersection.")

    return num_phase, al, non, nonConflict
