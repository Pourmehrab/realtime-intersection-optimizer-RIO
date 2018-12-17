################################################
# File name: config.py
# Authors: Mahmoud Pourmehrab / Ash Omidvar
# Emails: pourmehrab@gmail.com / aschkan@ufl.edu      
# Updated (Pourmehrab): May/30/2018
# Updated (Omidvar): Dec/13/2018
################################################

# -------------------------------------------------------
# Vehicle and Intersection Configuration Parameters
# -------------------------------------------------------
import scipy.io


def load_inter_params(inter_name):
    """
    :return:
        - inter_name: intersection name
        - max_speed: maximum speed in :math:`m/s`
        - min_headway: the lowest headway at the stop bar in :math:`s` (corresponds to the highest flow)
        - det_range: detection range in :math:`m`
        - k, m: refer to :any:`LeadConnected` for the definitions
        - num_lanes: total number of incoming lanes
        - phase_cover_set: a subset of mutually exclusive phases that cover all lanes for use in :any:`_set_non_base_scheduled_departures`
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

    .. warning:: All the parameters defined here are required for running the program.


    :Author:
        Mahmoud Pourmehrab <pourmehrab@gmail.com>
        Ash Omidvar <aschkan@ufl.edu>
    :Date:
        April-2018
        Dec-2018
    """

    if inter_name == "13th16th":
        return {"inter_name": "13th16th",
                "max_speed": 15.0,
                "min_CAV_headway": 2.0,
                "min_CNV_headway": 3.0,
                "det_range": 500.0,
                "k": int(10),
                "m": int(20),
                "num_lanes": int(16),
                "phase_cover_set": (17, 9, 8, 15,),
                "small_positive_num": 0.01,
                "large_positive_num": 999_999_999,
                "lli": {0: {6},
                        1: {6, 7, 11, 12, 13, 14, 15},
                        2: {6, 7, 8, 11, 13, 14, 15},
                        3: {6, 7, 8, 9, 10, 14, 15},
                        4: {6, 7, 8, 9, 10, 13, 14, 15},
                        5: {9},
                        6: {0, 1, 2, 3, 4, 9, 10, 14, 15},
                        7: {1, 2, 3, 4, 9, 10, 11, 14, 15},
                        8: {2, 3, 4, 9, 10, 11, 12, 13},
                        9: {3, 4, 5, 6, 7, 8, 12, 13, 14},
                        10: {3, 4, 6, 7, 8, 12, 13, 14, 15},
                        11: {1, 2, 7, 8, 12, 13, 14, 15},
                        12: {1, 2, 3, 8, 9, 10, 11},
                        13: {1, 2, 3, 4, 8, 9, 10, 11},
                        14: {1, 2, 3, 4, 5, 6, 7, 10, 11},
                        15: {1, 2, 3, 4, 6, 7, 10, 11}, },
                "pli": {
                    # 0: {15, 0, 9},
                    # 1: {11, 5, 6},
                    # 2: {0, 1, 5, 8},
                    # 3: {0, 12, 4, 5},
                    # 4: {7, 0, 5, 8},
                    # 5: {15, 0, 5, 8},
                    # 6: {0, 10, 11, 5},
                    0: {15, 0, 8, 14},
                    1: {0, 9, 10, 11},
                    2: {7, 8, 5, 6},
                    # 3: {0, 1, 2, 5, 10},
                    3: {0, 1, 2, 9, 10},
                    4: {0, 3, 4, 5, 11},
                    # 13: {0, 5, 7, 12, 13},
                    # 14: {0, 5, 12, 13, 15},
                    5: {0, 12, 13, 14, 15},
                    6: {5, 6, 7, 12, 13},
                    7: {0, 1, 2, 3, 4, 5}, },
                "allowable_phases": (0, 4, 5, 7,),
                "yellow": 1.5,
                "allred": 1.0,
                "min_green": 5.0,
                "max_green": 25.0,
                "lag_on_green": 1.0,
                "max_num_traj_points": int(1_000),
                "min_dist_to_stop_bar": 50,
                "do_traj_computation": True,
                "trj_time_resolution": 1.0,
                "log_csv": True,
                "print_commandline": True,
                }
    elif inter_name == "TERL":
        return {"inter_name": "TERL",
                "max_speed": 17.8816,  # 40 mph
                "min_CAV_headway": 1.5,
                "min_CNV_headway": 2.0,
                "det_range": 500.0,
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
                "log_csv": True,
                "print_commandline": True,
                }
    elif inter_name == "reserv":
        return {"inter_name": "reserv",
                "max_speed": 15.0,
                "min_CAV_headway": 2.0,
                "min_CNV_headway": 3.0,
                "det_range": 500.0,
                "k": int(20),
                "m": int(40),
                "num_lanes": int(12),
                "phase_cover_set": (0, 1, 2, 3,),
                "small_positive_num": 0.01,
                "large_positive_num": 999_999_999,
                "lli": {0: {3, 4, 5, 6, 7, 8, 9, 10, 11},
                        1: {3, 4, 5, 6, 7, 8, 9, 10, 11},
                        2: {3, 4, 5, 6, 7, 8, 9, 10, 11},
                        3: {0, 1, 2, 6, 7, 8, 9, 10, 11},
                        4: {0, 1, 2, 6, 7, 8, 9, 10, 11},
                        5: {0, 1, 2, 6, 7, 8, 9, 10, 11},
                        6: {0, 1, 2, 3, 4, 5, 9, 10, 11},
                        7: {0, 1, 2, 3, 4, 5, 9, 10, 11},
                        8: {0, 1, 2, 3, 4, 5, 9, 10, 11},
                        9: {0, 1, 2, 3, 4, 5, 6, 7, 8},
                        10: {0, 1, 2, 3, 4, 5, 6, 7, 8},
                        11: {0, 1, 2, 3, 4, 5, 6, 7, 8}, },
                "pli": {0: {0, 1, 2},
                        1: {3, 4, 5},
                        2: {7, 8, 6},
                        3: {9, 10, 11}, },
                "allowable_phases": (0, 1, 2, 3,),
                "yellow": 3.0,
                "allred": 1.5,
                "min_green": 5.0,
                "max_green": 40.0,
                "lag_on_green": 1.0,
                "max_num_traj_points": int(1_0000),
                "min_dist_to_stop_bar": 50,
                "do_traj_computation": False,
                "trj_time_resolution": 1.0,
                "log_csv": True,
                "print_commandline": True,

                }
    elif inter_name == "Gale&Std":
        return {"inter_name": "Gale&Std",
                "max_speed": 15.0,
                "min_CAV_headway": 1.5,
                "min_CNV_headway": 2.0,
                "det_range": 500.0,
                "k": int(20),
                "m": int(40),
                "num_lanes": int(8),
                "phase_cover_set": (0, 1, 2, 3,),
                "small_positive_num": 0.01,
                "large_positive_num": 999_999_999,
                "lli": None,  # todo add
                "pli": {0: {0, 1},  # South Bound
                        1: {2, 3},  # West Bound
                        2: {4, 5},  # North Bound
                        3: {6, 7},  # East Bound
                        4: {0, 4},  # D th
                        5: {2, 6},  # D th
                        6: {1, 5},  # D l
                        7: {3, 7},  # D l
                        },
                "allowable_phases": (0, 1, 2, 3,),
                "yellow": 3.0,
                "allred": 1.5,
                "min_green": 5.0,
                "max_green": 25.0,
                "lag_on_green": 1.0,
                "max_num_traj_points": int(1_000),
                "min_dist_to_stop_bar": 50,
                "do_traj_computation": True,
                "trj_time_resolution": 1.0,
                "log_csv": True,
                "print_commandline": True,
                }

    elif inter_name == "RTS":
        return {"inter_name": "RTS",
                "max_speed": 6.7,  # 15 mph
                "min_CAV_headway": 1.5,
                "min_CNV_headway": 2.0,
                "det_range": 300.01, # Max. distance
                "k": int(20),   # LeadConnected Trajectory method params
                "m": int(40),
                "num_lanes": int(4),
                "phase_cover_set": (0, 1,),
                "small_positive_num": 0.01, # GA params
                "large_positive_num": 999_999_999,
                "pli": {0: {0, 1, },
                        1: {1, 0, },
                        2: {2, 3, },
                        3: {3, 2, }, },
                "lli": {0: {2, 3, },  # Northeast (ATC: 1) - Lane: 1
                        1: {2, 3, },  # Southwest (ATC: 2) - Lane: 2
                        2: {0, 1, },  # Southeast (ATC: 3) - Lane: 3
                        3: {0, 1, },  # Northwest (ATC: 4) - Lane: 4
                        },
                "allowable_phases": (0, 1, 2, 3,),
                "yellow": 3.0,
                "allred": 1.5,
                "min_green": 5,
                "max_green": 20.0,
                "lag_on_green": 1.0,
                "max_num_traj_points": int(1_000),
                "min_dist_to_stop_bar": 20,
                "do_traj_computation": True,
                "trj_time_resolution": 1.0,
                "log_csv": True,
                "print_commandline": True,
                # "gps_points": scipy.io.loadmat('file.mat') # FIXME @ Pat: Please import GPS points here. In order to avoid mapping and its
                # FIXME: confusing consequences, it would be tight if you could follow the lane numbers in accordance
                # FIXME: with what you see above and the schematic map I sent you. I believe for UTC demo you used the
                # FIXME: lane number as annotated on the map I sent you. Alternatively, let me know and I'll change phasing according to your lane numbers.
                }
    else:
        raise Exception("Simulation parameters are not known for this intersection.")
# -------------------------------------------------------
# PRETIMED CONTROL PARAMETERS
# -------------------------------------------------------
def get_pretimed_parameters(inter_name):
    """
    This returns the parameters needed for pre-timed control.

    .. note::
        - The sequence field includes the phases and is zero-based.
        - You need to compute green splits, yellows, and all-reds based on traffic flow theory.

    .. warning::
            Must choose ``num_cycles`` at least 2.
    """
    if inter_name == "13th16th":
        return None  # todo compute these

    elif inter_name == "TERL":
        return {"green_dur": (12.0, 12.0, 12.0, 12.0), "phase_seq": (0, 1, 2, 3,), "yellow": 1.5, "all-red": 1.5,
                "num_cycles": 5}

    elif inter_name == "reserv":
        return {"green_dur": (25.0, 25.0, 25.0, 25.0), "phase_seq": (0, 1, 2, 3,), "yellow": 3.0, "all-red": 1.5,
                "num_cycles": 5}
    elif inter_name == "RTS":
        return None  # todo compute these

    else:
        raise Exception("Pretimed parameters are not known for this intersection.")


# -------------------------------------------------------
# GA CONTROL PARAMETERS
# -------------------------------------------------------
def get_GA_parameters(inter_name):
    """

    :return:
        - max_phase_length: do not include more than this in a phase sequence (is exclusive of the last: 1,2, ..., ``max_phase_length``-1)
        - population_size: this is the maximum size of individuals per iteration of :term:`GA`
        - max_iteration_per_phase:
        - crossover_size: this specifies how many of the individuals from ``population_size`` to be computed using crossover.
        - lambda: The weight factor to convert average travel time to throughput and give the :term:`badness` of an individual.
        - badness_accuracy: 10 raised to the number of digits we want to keep when hashing the :term:`badness` of an individual
        - allowable_phases: subset of all possible phases to be used. These are different than the phase_cover_set

    """
    if inter_name == "13th16th":
        return None  # todo add these

    elif inter_name == "TERL":
        return {"max_phase_length": 4,
                "population_size": 20,
                "max_iteration_per_phase": 10,
                "crossover_size": 10,
                "lambda": 1 / 500,
                "badness_accuracy": 10 ** 2,
                }
    elif inter_name == "reserv":
        return {"max_phase_length": 4,
                "population_size": 20,
                "max_iteration_per_phase": 10,
                "crossover_size": 10,
                "lambda": 1 / 500,
                "badness_accuracy": 10 ** 2,
                }
    elif inter_name == "RTS":
        return None  # todo add these
    else:
        raise Exception("GA parameters are not known for this intersection.")


# -------------------------------------------------------
# SIGNAL CONTROLLER PARAMETERS
# -------------------------------------------------------
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
        non_conflict = [[2], [3, 8], [4, 7], [6]]  # Conflict monitor phases
    elif inter_name == "RTS":
        num_phase = 4  # Total Number of phases at RTS
        al = range(1, num_phase + 1)
        non = [0]
        non_conflict = [[1, 2], [4, 3]]  # Conflict monitor phases
    else:
        raise Exception("Controller parameters are not known for this intersection.")

    return num_phase, al, non, non_conflict
