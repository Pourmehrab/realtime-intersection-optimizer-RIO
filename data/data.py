####################################
# File name: data.py               #
# Author: Mahmoud Pourmehrab       #
# Email: mpourmehrab@ufl.edu       #
# Last Modified: Apr/22/2018       #
####################################

# GENERAL PARAMETERS
def get_general_params(inter_name):
    """
    Returns max speed (:math:`m/s`), min_headway (:math:`s`), detection range (:math:`m`), :math:`k, m`, number of lanes. Where:
        - :math:`k` =  # :math:`n` will be in 0, ..., k-1 (odd degree of polynomial is preferred: k to be even and **at least** 5)
        - :math:`m` =  # to discretize the time interval

    .. note::
        - The distance to stop bar will be input from either CSV file or fusion. However, the number provided here is used for generic computations.

    .. warning:: Is required for trajectory optimization

    :Author:
        Mahmoud Pourmehrab <pourmehrab@gmail.com>
    :Date:
        April-2018
    """
    if inter_name == '13th16th':
        max_speed = 15.0
        min_headway = 2.0
        det_range = 500.0
        k, m = int(10), int(20)
        num_lanes = int(16)

    elif inter_name == 'TERL':
        max_speed = 17.8816  # 40 mph
        min_headway = 1.5
        det_range = 500.0  # 1640 ft
        k, m = int(15), int(20)
        num_lanes = int(6)

    elif inter_name == 'reserv':
        max_speed = 15.0
        min_headway = 2.0
        det_range = 500.0
        k, m = int(15), int(10)
        num_lanes = int(12)

    else:
        raise Exception('Simulation parameters are not known for this intersection.')

    return max_speed, min_headway, det_range, k, m, num_lanes


# PRETIMED CONTROL PARAMETERS
def get_pretimed_parameters(inter_name):
    """
    This returns the parameters needed for pre-timed control.

    .. note::
        - The sequence field includes the phases and is zero-based.
        - You need to compute green splits and yellows, all-reds based on traffic flow theory.

    :Author:
        Mahmoud Pourmehrab <pourmehrab@gmail.com>
    :Date:
        April-2018
    """

    if inter_name == '13th16th':
        return None  # todo compute these

    elif inter_name == 'TERL':
        return {'green_dur': (12.0, 12.0, 12.0, 12.0), 'phase_seq': (0, 1, 2, 3,), 'yellow': 1.5, 'all-red': 1.5}

    elif inter_name == 'reserv':
        return {'green_dur': (25.0, 25.0, 25.0, 25.0), 'phase_seq': (0, 1, 2, 3,), 'yellow': 3.0, 'all-red': 1.5}

    else:
        raise Exception('Pretimed parameters is not known for this intersection.')


# GA CONTROL PARAMETERS
def get_conflict_dict(inter_name):
    """
    Returns a **dictionary** of sets where the **keys** are lane numbers and must be coded in one-based and the **value** for each key is a set of lane numbers that are in conflict with the key lane (again must be one based).

    An intersection configuration can be specified by its lanes and movements (left, through, right) that are allowed in each lane. The lane-lane incidence matrix of an intersection is a squared matrix that holds 1 (shown by solid circles in the figures), if two lanes are in conflict. The standard types of conflicts that may wanted to be avoided are cross, merge, and diverge conflicts. Depending on the design, the definition of conflicts points can be broader or more limited. For instance, if volume of a lane is too low and extensive gaps can be found, some of conflict points can be relaxed as non-conflicting points. In the following figures, only cross and merge conflict points are indicated.

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

    if inter_name == '13th16th':
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
        # note (17, 9, 8, 15,) covers all lanes

    elif inter_name == 'TERL':
        lli = {1: {2, 3, 4, 5, 6, },
               2: {1, 4, 6, },
               3: {1, 4, 5, },
               4: {1, 2, 3, 5, 6},
               5: {1, 3, 4, },
               6: {1, 2, 4, }, }
        # note (1, 2, 3, 4, ) covers all lanes

    elif inter_name == 'reserv':
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
        raise Exception('Set of conflicting lanes is not known for this intersection.')

    return lli


def get_phases(inter_name):
    """
    Returns a dictionary of sets
    The key is the phase number is one-based
    The value to a key is set of lanes included in that phase (lanes are one-based too)
    Use the phase enumerator for new intersections of refine manually
    The rule is each set must include non-conflicting lanes
    # todo add the phase enumarator to the project

    :Author:
        Mahmoud Pourmehrab <pourmehrab@gmail.com>
    :Date:
        April-2018
    """

    if inter_name == '13th16th':
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

    if inter_name == 'TERL':
        return {1: {1, },  # Southbound (signal controller: phase 2)
                2: {5, 6, },  # Eastbound (signal controller: phase 3)
                3: {2, 3, },  # Westbound (signal controller: phase 4)
                4: {4, },  # Northbound (signal controller: phase 6)
                5: {2, 5, },  # dual opposite throughs
                6: {3, 6, },  # dual left turns
                }

    elif inter_name == 'reserv':
        pli = {1: {1, 2, 3, },
               2: {4, 5, 6, },
               3: {7, 8, 9, },
               4: {10, 11, 12, }}
    else:
        raise Exception('Set of phases is not known for this intersection.')

    return pli


def get_signal_params(inter_name):
    """
    Required for GA signal control
    ALL yellow, all-red, min green, max green times are in seconds

    :Author:
        Mahmoud Pourmehrab <pourmehrab@gmail.com>
    :Date:
        April-2018
    """

    if inter_name == '13th16th':
        yellow = 1.5
        allred = 1.0
        min_green = 5.0
        max_green = 25.0

    elif inter_name == 'TERL':
        yellow = 1.5
        allred = 1.5
        min_green = 4.6
        max_green = 25.0

    elif inter_name == 'reserv':
        yellow = 3.0
        allred = 1.5
        min_green = 5.0
        max_green = 40.0

    else:
        raise Exception('Signal parameters are not known for this intersection.')

    return yellow, allred, min_green, max_green
