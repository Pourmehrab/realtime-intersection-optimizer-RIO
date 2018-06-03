# File name: data.py
# Authors: Mahmoud Pourmehrab / Aschkan Omidvar    
# Emails: pourmehrab@gmail.com / aschkan@ufl.edu      
# Updated (Pourmehrab): May/30/2018
# Updated (Omidvar): May/28/2018     
####################################

# -------------------------------------------------------
# GENERAL PARAMETERS
# -------------------------------------------------------
def get_general_params(inter_name):
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


    .. warning:: All the parameters defined here are required for running the program.


    :Author:
        Mahmoud Pourmehrab <pourmehrab@gmail.com>
    :Date:
        April-2018
    """
    if inter_name == "13th16th":
        return {"inter_name": "13th16th",
                "max_speed": 15.0,
                "min_headway": 2.0,
                "det_range": 500.0,
                "k": int(10),
                "m": int(20),
                "num_lanes": int(16),
                "phase_cover_set": (17, 9, 8, 15,),
                "small_positive_num": 0.01,
                "large_positive_num": 999_999_999,
                "lag_on_green": 1.0,
                "max_num_traj_points": int(300),
                "min_dist_to_stop_bar": 50,
                "do_traj_computation": True,
                "trj_time_resolution": 1.0,
                "log_csv": True,
                "print_commandline": True,
                }
    elif inter_name == "TERL":
        return {"inter_name": "TERL",
                "max_speed": 17.8816,  # 40 mph
                "min_headway": 1.5,
                "det_range": 500.0,
                "k": int(20),
                "m": int(40),
                "num_lanes": int(6),
                "phase_cover_set": (0, 1, 2, 3,),
                "small_positive_num": 0.01,
                "large_positive_num": 999_999_999,
                "lag_on_green": 1.0,
                "max_num_traj_points": int(300),
                "min_dist_to_stop_bar": 50,
                "do_traj_computation": True,
                "trj_time_resolution": 1.0,
                "log_csv": True,
                "print_commandline": True,
                }
    elif inter_name == "reserv":
        return {"inter_name": "reserv",
                "max_speed": 15.0,
                "min_headway": 2.0,
                "det_range": 500.0,
                "k": int(20),
                "m": int(40),
                "num_lanes": int(12),
                "phase_cover_set": (0, 1, 2, 3,),
                "small_positive_num": 0.01,
                "large_positive_num": 999_999_999,
                "lag_on_green": 1.0,
                "max_num_traj_points": int(300),
                "min_dist_to_stop_bar": 50,
                "do_traj_computation": True,
                "trj_time_resolution": 1.0,
                "log_csv": True,
                "print_commandline": True,
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

    :Author:
        Mahmoud Pourmehrab <pourmehrab@gmail.com>
    :Date:
        April-2018
    """

    if inter_name == "13th16th":
        return None  # todo compute these

    elif inter_name == "TERL":
        return {"green_dur": (12.0, 12.0, 12.0, 12.0), "phase_seq": (0, 1, 2, 3,), "yellow": 1.5, "all-red": 1.5,
                "num_cycles": 5}

    elif inter_name == "reserv":
        return {"green_dur": (25.0, 25.0, 25.0, 25.0), "phase_seq": (0, 1, 2, 3,), "yellow": 3.0, "all-red": 1.5,
                "num_cycles": 5}

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

    :Author:
        Mahmoud Pourmehrab <pourmehrab@gmail.com>
    :Date:
        April-2018
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
                "allowable_phases": (0, 1, 2, 3,),
                }
    elif inter_name == "reserv":
        return {"max_phase_length": 4,
                "population_size": 20,
                "max_iteration_per_phase": 10,
                "crossover_size": 10,
                "lambda": 1 / 500,
                "badness_accuracy": 10 ** 2,
                "allowable_phases": (0, 1, 2, 3,),
                }

    else:
        raise Exception("GA parameters are not known for this intersection.")


def get_conflict_dict(inter_name):
    """
    Returns a dictionary of sets where the keys are lane numbers and must be one-based. The value for each key is a set
    of lane numbers that are in conflict with the key lane (again, must be one-based).

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

       The intersection of 13th and 16th, Gainesville, FL.

    :Author:
        Mahmoud Pourmehrab <pourmehrab@gmail.com>
    :Date:
        April-2018
    """

    if inter_name == "13th16th":
        lli = {1: {7},
               2: {7, 8, 12, 16, 15, 14, 13},
               3: {7, 8, 12, 16, 15, 14, 9},
               4: {7, 16, 8, 15, 9, 11, 10},
               5: {16, 7, 15, 8, 11, 9, 10, 14},
               6: {10},
               7: {10, 15, 11, 16, 5, 4, 3, 2, 1},
               8: {10, 11, 15, 5, 4, 16, 3, 2, 12},
               9: {5, 10, 4, 11, 14, 12, 13, 3},
               10: {13, 14, 4, 5, 9, 8, 7, 6, 15},
               11: {13, 14, 9, 4, 5, 8, 7, 15, 16},
               12: {13, 14, 9, 3, 15, 16, 2, 8},
               13: {2, 3, 9, 12, 11, 10, 4},
               14: {2, 3, 12, 9, 11, 4, 10, 5},
               15: {2, 3, 12, 4, 5, 8, 7, 11, 6},
               16: {12, 2, 3, 8, 4, 7, 5, 11}}

    elif inter_name == "TERL":
        lli = {1: {2, 3, 4, 5, 6, },
               2: {1, 4, 6, },
               3: {1, 4, 5, },
               4: {1, 2, 3, 5, 6},
               5: {1, 3, 4, },
               6: {1, 2, 4, }, }

    elif inter_name == "reserv":
        lli = {1: {4, 5, 6, 7, 8, 9, 10, 11, 12},
               2: {4, 5, 6, 7, 8, 9, 10, 11, 12},
               3: {4, 5, 6, 7, 8, 9, 10, 11, 12},
               4: {1, 2, 3, 7, 8, 9, 10, 11, 12},
               5: {1, 2, 3, 7, 8, 9, 10, 11, 12},
               6: {1, 2, 3, 7, 8, 9, 10, 11, 12},
               7: {1, 2, 3, 4, 5, 6, 10, 11, 12},
               8: {1, 2, 3, 4, 5, 6, 10, 11, 12},
               9: {1, 2, 3, 4, 5, 6, 10, 11, 12},
               10: {1, 2, 3, 4, 5, 6, 7, 8, 9},
               11: {1, 2, 3, 4, 5, 6, 7, 8, 9},
               12: {1, 2, 3, 4, 5, 6, 7, 8, 9}}
    else:
        raise Exception("Set of conflicting lanes is not known for this intersection.")

    return lli


def get_phases(inter_name):
    """
    :returns: A dictionary of sets. The key is the phase number and is one-based. The value to a key is a set of lanes
        included in that phase (lanes are also one-based).

    :Author:
        Mahmoud Pourmehrab <pourmehrab@gmail.com>
    :Date:
        April-2018
    """

    if inter_name == "13th16th":
        pli = {1: {1, 10, 16, },
               2: {6, 7, 12, },
               3: {1, 2, 6, 9, },
               4: {1, 5, 6, 13, },
               5: {1, 6, 8, 9, },
               6: {1, 6, 9, 16, },
               7: {1, 6, 11, 12, },
               8: {1, 9, 15, 16, },
               9: {1, 10, 11, 12, },
               10: {6, 7, 8, 9, },
               11: {1, 2, 3, 6, 11, },
               12: {1, 2, 3, 10, 11, },
               13: {1, 4, 5, 6, 12, },
               14: {1, 6, 8, 13, 14, },
               15: {1, 6, 13, 14, 16, },
               16: {1, 13, 14, 15, 16, },
               17: {6, 7, 8, 13, 14, },
               18: {1, 2, 3, 4, 5, 6, }}

    if inter_name == "TERL":
        return {1: {1, },  # Southbound (signal controller: phase 2)
                2: {5, 6, },  # Eastbound (signal controller: phase 3)
                3: {2, 3, },  # Westbound (signal controller: phase 4)
                4: {4, },  # Northbound (signal controller: phase 6)
                5: {2, 5, },  # dual opposite throughs
                6: {3, 6, },  # dual left turns
                }

    elif inter_name == "reserv":
        pli = {1: {1, 2, 3, },
               2: {4, 5, 6, },
               3: {7, 8, 9, },
               4: {10, 11, 12, }}
    else:
        raise Exception("Set of phases is not known for this intersection.")

    return pli


def get_signal_params(inter_name):
    """
    Required for GA signal control. ALL yellow, all-red, min green, and max green times are in seconds.

    :Author:
        Mahmoud Pourmehrab <pourmehrab@gmail.com>
    :Date:
        April-2018
    """

    if inter_name == "13th16th":
        yellow = 1.5
        allred = 1.0
        min_green = 5.0
        max_green = 25.0

    elif inter_name == "TERL":
        yellow = 1.5
        allred = 1.5
        min_green = 4.6
        max_green = 25.0

    elif inter_name == "reserv":
        yellow = 3.0
        allred = 1.5
        min_green = 5.0
        max_green = 40.0

    else:
        raise Exception("Signal parameters are not known for this intersection.")

    return yellow, allred, min_green, max_green


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

    else:
        raise Exception("Controller parameters are not known for this intersection.")

    return num_phase, al, non, non_conflict
